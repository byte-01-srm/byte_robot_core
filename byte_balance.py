#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════════════╗
║          BYTE — Quadruped IMU Balance Controller  v2.3                     ║
║          AK60 V3.0 BLDC  |  MPU6050  |  Raspberry Pi  |  Complementary CF ║
╚══════════════════════════════════════════════════════════════════════════════╝

Robot leg layout                Motor joints per leg
  ─────────────────             ┌──────────────────────────────────────────┐
   [FL]     [FR]                │  Joint 0 – Hip   (motor 1/4/7/10)       │
   [BL]     [BR]                │  Joint 1 – Thigh (motor 2/5/8/11)       │
  ─────────────────             │  Joint 2 – Knee  (motor 3/6/9/12)       │
                                └──────────────────────────────────────────┘

NOTE on gear ratio:
  main.py transform_live_angles() already multiplies every angle by gear_ratio
  before sending to the motor. So this script sends angles in USER space —
  NO gear ratio multiplication needed here. main.py handles it.

Correct run order
  1. python3 main.py           → homing + motor server
  2. python3 main_sender.py    → move to standing pose (60, -140, 55)
  3. python3 byte_balance.py   → this file opens shell
  4. Type R + Enter            → capture flat reference NOW (robot standing,
                                 level, still) — REJECTED if tilt > 3 deg
  5. Type S + Enter            → start balance loop

Quick sign check (do once before first real run)
  Hold nose DOWN      → front knee deltas should be POSITIVE  (else PITCH_SIGN = -1)
  Tilt RIGHT side down → right knee deltas should be POSITIVE (else ROLL_SIGN  = -1)

UI commands
  R → capture flat reference (rejected if robot tilted more than 3 deg)
  S → start balance loop
  Q → pause balance loop
  X → exit
"""

import math
import pickle
import socket
import time
import sys
import select

import smbus2


# ═══════════════════════════════════════════════════════════════
#  Socket  (must match main.py)
# ═══════════════════════════════════════════════════════════════
HOST = "127.0.0.1"
PORT = 50000

# ═══════════════════════════════════════════════════════════════
#  MPU6050 I2C
# ═══════════════════════════════════════════════════════════════
MPU_ADDR      = 0x68      # change to 0x69 if AD0 pin is HIGH
I2C_BUS       = 1
REG_PWR_MGMT  = 0x6B
REG_ACCEL_OUT = 0x3B
ACCEL_SCALE   = 16384.0   # LSB/g   (+-2 g range)
GYRO_SCALE    = 131.0     # LSB/(deg/s) (+-250 deg/s range)
ACCEL_BIAS    = [0.0, 0.0, 0.0]   # zeroed — R reference handles chip offset

# ═══════════════════════════════════════════════════════════════
#  Standing pose  (empirical motor-space angles, degrees)
#  Must match exactly what main_sender.py sends
# ═══════════════════════════════════════════════════════════════
STAND_HIP   =  60.0
STAND_THIGH = -140.0
STAND_KNEE  =  55.0

# ═══════════════════════════════════════════════════════════════
#  PD gains
#  Tuning order:
#    1. Set KD=0, raise KP until corrections feel strong enough
#    2. Raise KD until bounce/oscillation after disturbance stops
# ═══════════════════════════════════════════════════════════════
BALANCE_KP = 0.8
BALANCE_KD = 0.05

# ═══════════════════════════════════════════════════════════════
#  Sign convention
# ═══════════════════════════════════════════════════════════════
PITCH_SIGN = +1   # flip to -1 if pitch correction is backwards
ROLL_SIGN  = +1   # flip to -1 if roll correction is backwards

# ═══════════════════════════════════════════════════════════════
#  Safety & filter
# ═══════════════════════════════════════════════════════════════
MAX_KNEE_DELTA   = 20.0   # max knee deviation from standing (degrees)
DEAD_ZONE_DEG    = 1.5    # tilts smaller than this are ignored
CF_ALPHA         = 0.98   # complementary filter weight (gyro vs accel)

# ═══════════════════════════════════════════════════════════════
#  Reference capture tilt guard
#  R is rejected if robot is tilted more than this during capture
# ═══════════════════════════════════════════════════════════════
REF_MAX_TILT_DEG = 3.0

# ═══════════════════════════════════════════════════════════════
#  Loop rate
# ═══════════════════════════════════════════════════════════════
LOOP_HZ = 25.0
LOOP_DT  = 1.0 / LOOP_HZ

# ═══════════════════════════════════════════════════════════════
#  Calibration
# ═══════════════════════════════════════════════════════════════
CALIB_SAMPLES = 150
REF_SAMPLES   = 600


# ══════════════════════════════════════════════════════════════════════════════
#  Utility
# ══════════════════════════════════════════════════════════════════════════════
def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def dead_zone(v, z):
    return 0.0 if abs(v) < z else v


# ══════════════════════════════════════════════════════════════════════════════
#  MPU6050
# ══════════════════════════════════════════════════════════════════════════════
def init_mpu(bus):
    bus.write_byte_data(MPU_ADDR, REG_PWR_MGMT, 0x00)
    time.sleep(0.1)


def read_raw_imu(bus):
    """Returns (ax_g, ay_g, az_g, gx_dps, gy_dps)."""
    d = bus.read_i2c_block_data(MPU_ADDR, REG_ACCEL_OUT, 14)

    def s16(i):
        v = (d[i] << 8) | d[i+1]
        return v - 65536 if v > 32767 else v

    ax = s16(0)  / ACCEL_SCALE - ACCEL_BIAS[0]
    ay = s16(2)  / ACCEL_SCALE - ACCEL_BIAS[1]
    az = s16(4)  / ACCEL_SCALE - ACCEL_BIAS[2]
    gx = s16(8)  / GYRO_SCALE
    gy = s16(10) / GYRO_SCALE
    return ax, ay, az, gx, gy


def get_roll_pitch(ax, ay, az):
    roll  = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))
    return roll, pitch


def calibrate_gyro(bus):
    print(f"  Collecting {CALIB_SAMPLES} samples — keep robot still...")
    sx = sy = 0.0
    for i in range(CALIB_SAMPLES):
        _, _, _, gx, gy = read_raw_imu(bus)
        sx += gx
        sy += gy
        time.sleep(LOOP_DT)
        if (i+1) % 50 == 0:
            print(f"  {i+1}/{CALIB_SAMPLES}...")
    bx = sx / CALIB_SAMPLES
    by = sy / CALIB_SAMPLES
    print(f"  Gyro bias: gx={bx:+.4f} deg/s   gy={by:+.4f} deg/s")
    return bx, by


def take_reference(bus, n=REF_SAMPLES):
    """
    Capture flat reference only if robot is actually close to level.
    Returns (ref_roll, ref_pitch) on success, (None, None) on rejection.

    Robot must be standing, still, on flat ground, and within
    +-REF_MAX_TILT_DEG of level when you run this.
    """
    print(f"  Collecting {n} samples — keep robot still and level...")
    sr = sp = 0.0
    for _ in range(n):
        ax, ay, az, _, _ = read_raw_imu(bus)
        r, p = get_roll_pitch(ax, ay, az)
        sr += r
        sp += p
        time.sleep(0.005)

    rr = sr / n
    rp = sp / n

    if abs(rr) > REF_MAX_TILT_DEG or abs(rp) > REF_MAX_TILT_DEG:
        print()
        print("  *** REFERENCE REJECTED ***")
        print(f"  Measured roll={rr:+.2f} deg   pitch={rp:+.2f} deg")
        print(f"  Both must be within +-{REF_MAX_TILT_DEG} deg to accept.")
        if abs(rr) > REF_MAX_TILT_DEG:
            print(f"  Roll is {abs(rr) - REF_MAX_TILT_DEG:.2f} deg over limit.")
        if abs(rp) > REF_MAX_TILT_DEG:
            print(f"  Pitch is {abs(rp) - REF_MAX_TILT_DEG:.2f} deg over limit.")
        print("  Level the robot and run R again.\n")
        return None, None

    print(f"  Reference accepted — roll={rr:+.3f} deg   pitch={rp:+.3f} deg\n")
    return rr, rp


# ══════════════════════════════════════════════════════════════════════════════
#  Balance core
# ══════════════════════════════════════════════════════════════════════════════
def compute_knee_corrections(pitch, roll, pitch_rate, roll_rate):
    """
    PD correction matrix — all in user-space degrees.
    main.py applies gear_ratio internally so we never touch it here.

    pitch > 0 = nose down -> front knees extend (+), rear retract (-)
    roll  > 0 = right side down -> right knees extend (+), left retract (-)
    """
    pd_p = PITCH_SIGN * (BALANCE_KP * pitch + BALANCE_KD * pitch_rate)
    pd_r = ROLL_SIGN  * (BALANCE_KP * roll  + BALANCE_KD * roll_rate)
    raw = {
        "fl": +pd_p - pd_r,
        "bl": -pd_p - pd_r,
        "fr": +pd_p + pd_r,
        "br": -pd_p + pd_r,
    }
    return {leg: clamp(d, -MAX_KNEE_DELTA, MAX_KNEE_DELTA) for leg, d in raw.items()}


def build_payload(knee_deltas):
    """
    Hip and Thigh held fixed at standing pose every tick.
    Only knee changes. No gear ratio applied — main.py handles it.
    """
    payload = {}
    for leg, delta in knee_deltas.items():
        knee_cmd = clamp(
            STAND_KNEE + delta,
            STAND_KNEE - MAX_KNEE_DELTA,
            STAND_KNEE + MAX_KNEE_DELTA,
        )
        payload[leg] = [STAND_HIP, STAND_THIGH, knee_cmd]
    return payload


def send_payload(payload):
    data = pickle.dumps(payload, protocol=pickle.HIGHEST_PROTOCOL)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.5)
        s.connect((HOST, PORT))
        s.sendall(data)


# ══════════════════════════════════════════════════════════════════════════════
#  Console status bar (replaces matplotlib on headless Pi)
# ══════════════════════════════════════════════════════════════════════════════
def severity_bar(val, max_val, width=20):
    """Simple ASCII tilt severity bar."""
    if max_val == 0:
        return "[" + "-" * width + "]"
    filled = int(min(abs(val) / max_val, 1.0) * width)
    bar = "#" * filled + "-" * (width - filled)
    return f"[{bar}]"


def print_status(roll, pitch, knee_deltas, tick):
    """Rich console status — one line per second."""
    r_bar = severity_bar(roll,  30)
    p_bar = severity_bar(pitch, 30)
    kd    = knee_deltas
    print(
        f"[{tick:>5}] "
        f"roll={roll:+6.2f} deg {r_bar}  "
        f"pitch={pitch:+6.2f} deg {p_bar}  |  "
        f"knee: fl={kd['fl']:+5.1f}  bl={kd['bl']:+5.1f}  "
        f"fr={kd['fr']:+5.1f}  br={kd['br']:+5.1f} deg"
    )


# ══════════════════════════════════════════════════════════════════════════════
#  Balance loop
# ══════════════════════════════════════════════════════════════════════════════
def balance_loop(bus, gx_bias, gy_bias, ref_roll, ref_pitch):

    # Bootstrap complementary filter from accelerometer
    ax_, ay_, az_, _, _ = read_raw_imu(bus)
    roll  = math.degrees(math.atan2(ay_, az_))
    pitch = math.degrees(math.atan2(-ax_, math.sqrt(ay_*ay_ + az_*az_)))
    prev_roll  = roll  - ref_roll
    prev_pitch = pitch - ref_pitch

    tick = 0
    print(f"  Balance loop active @ {LOOP_HZ} Hz")
    print("  Type Q + Enter to pause\n")

    while True:
        t0 = time.monotonic()

        # ── Read IMU ──────────────────────────────────────────────────────────
        ax_, ay_, az_, gx_, gy_ = read_raw_imu(bus)
        gx_ -= gx_bias
        gy_ -= gy_bias

        accel_roll  = math.degrees(math.atan2(ay_, az_))
        accel_pitch = math.degrees(math.atan2(-ax_, math.sqrt(ay_*ay_ + az_*az_)))

        # Complementary filter
        roll  = CF_ALPHA * (roll  + gx_ * LOOP_DT) + (1 - CF_ALPHA) * accel_roll
        pitch = CF_ALPHA * (pitch + gy_ * LOOP_DT) + (1 - CF_ALPHA) * accel_pitch

        # Subtract flat reference so corrections are relative to level
        roll_c  = roll  - ref_roll
        pitch_c = pitch - ref_pitch

        # Tilt rates for D term
        roll_rate  = (roll_c  - prev_roll)  / LOOP_DT
        pitch_rate = (pitch_c - prev_pitch) / LOOP_DT
        prev_roll  = roll_c
        prev_pitch = pitch_c

        # Dead zone
        eff_roll  = dead_zone(roll_c,  DEAD_ZONE_DEG)
        eff_pitch = dead_zone(pitch_c, DEAD_ZONE_DEG)

        # ── Corrections and send ──────────────────────────────────────────────
        knee_deltas = compute_knee_corrections(eff_pitch, eff_roll,
                                               pitch_rate, roll_rate)
        payload = build_payload(knee_deltas)

        try:
            send_payload(payload)
        except ConnectionRefusedError:
            print("[Socket] main.py not reachable — retrying in 1 s...")
            time.sleep(1.0)
            continue
        except OSError as e:
            print(f"[Socket] {e}")
            time.sleep(0.1)
            continue

        # ── Console status every 1 s ──────────────────────────────────────────
        tick += 1
        if tick % int(LOOP_HZ) == 0:
            print_status(roll_c, pitch_c, knee_deltas, tick)

        # ── Non-blocking Q check ──────────────────────────────────────────────
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            if input().strip().upper() == "Q":
                print("  Balance loop paused.\n")
                break

        # ── Rate limit ────────────────────────────────────────────────────────
        elapsed = time.monotonic() - t0
        sleep_t = LOOP_DT - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)


# ══════════════════════════════════════════════════════════════════════════════
#  Main — command shell
# ══════════════════════════════════════════════════════════════════════════════
def main():
    print("=" * 62)
    print("  BYTE  —  IMU Balance Controller  v2.3  (headless)")
    print(f"  Socket  : {HOST}:{PORT}   Loop: {LOOP_HZ} Hz")
    print(f"  KP={BALANCE_KP}  KD={BALANCE_KD}  dead_zone=+-{DEAD_ZONE_DEG} deg")
    print(f"  Reference guard : +-{REF_MAX_TILT_DEG} deg (R rejected if tilted)")
    print(f"  Gear ratio      : handled by main.py")
    print("=" * 62)
    print()
    print("  Make sure main_sender.py has already been run and")
    print("  the robot is STANDING before you type R.\n")

    bus = smbus2.SMBus(I2C_BUS)
    init_mpu(bus)
    print("MPU6050 initialised.\n")

    print("Calibrating gyro bias — keep robot still...")
    gx_bias, gy_bias = calibrate_gyro(bus)
    print()

    ref_roll = ref_pitch = None

    print("Commands:  R → Reference  |  S → Start  |  Q → Stop  |  X → Exit\n")

    while True:
        cmd = input("BYTE > ").strip().upper()

        if cmd == "R":
            print("Capturing flat reference...")
            print(f"Robot must be standing, still, within +-{REF_MAX_TILT_DEG} deg of level.\n")
            result_roll, result_pitch = take_reference(bus)
            if result_roll is not None:
                ref_roll, ref_pitch = result_roll, result_pitch
            # If None, old reference is preserved unchanged

        elif cmd == "S":
            if ref_roll is None:
                print("  Run R first to capture the flat reference.")
                print("  Robot must be standing and level before R.\n")
                continue
            balance_loop(bus, gx_bias, gy_bias, ref_roll, ref_pitch)

        elif cmd == "X":
            print("  Shutting down. Goodbye.")
            break

        else:
            print("  Unknown command. Use R / S / Q / X\n")


if __name__ == "__main__":
    main()