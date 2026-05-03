#!/usr/bin/env python3
# Tapping gait with IMU balance correction — BYTE-01
# =====================================================
# Standing : all feet at [-9.094, 25.0, 9.0]
# Tap      : lift each foot  y: 25.0 → 20.0  then place back  y: 20.0 → 25.0
# Sequence : fl → br → fr → bl  (diagonal walking pattern: LF → RH → RF → LH)
#
# IMU balance:
#   MPU6050 measures roll and pitch of the body in real-time.
#   A PD controller computes lateral (x) and fore-aft (z) foot corrections
#   to shift the virtual CoM back over the support polygon when the bot tilts.
#   Every packet sent to main_with_ik already has these corrections baked in.
#
# Motor map:
#   fl : motors  1, 2, 3   |   bl : motors  4, 5, 6
#   fr : motors  7, 8, 9   |   br : motors 10, 11, 12
#
# Controls:
#   f  →  queue one full tap round
#   n  →  send standing pose + clear queue
#   b  →  print current IMU angles (debug)
#   q  →  quit
#
# Run AFTER main_with_ik.py shows: 🎯 POST-HOMING LIVE CONTROL READY

import math
import pickle
import queue
import socket
import sys
import termios
import threading
import time
import tty

from imu_reader import ImuReader


# ── Socket ────────────────────────────────────────────────────────────────────
HOST = "127.0.0.1"
PORT = 50000


# ── Foot positions ────────────────────────────────────────────────────────────
X_NOM   = -9.094   # nominal lateral hip offset (cm)
Y_STAND =  25.0    # y when foot is on the ground (cm)
Y_TAP   =  20.0    # y when foot is lifted  (smaller y = foot UP)
Z_NOM   =   9.0    # nominal forward reach (cm)

STAND_POS = [X_NOM, Y_STAND, Z_NOM]


# ── Tap timing ────────────────────────────────────────────────────────────────
LIFT_DT  = 0.01    # seconds to lift foot up
HOLD_DT  = 0.01    # seconds to hold at peak
PLACE_DT = 0.01    # seconds to place foot back down
LOOP_HZ  = 50.0    # interpolation send rate (Hz)


# ── Speed override ────────────────────────────────────────────────────────────
# Sent in every packet so main_with_ik.py uses this instead of its safe default.
GAIT_SPEED_DEG_PER_S = 900.0


# ── IMU hardware config ───────────────────────────────────────────────────────
IMU_I2C_BUS  = 1      # Raspberry Pi default I2C bus
IMU_I2C_ADDR = 0x68   # MPU6050 I2C address (0x68 if AD0=LOW, 0x69 if AD0=HIGH)


# ── Balance PD gains ──────────────────────────────────────────────────────────
#
# How it works:
#   When roll_deg > 0 (right side lower), the foot x positions are nudged
#   by +Kp_roll * roll_deg cm. This shifts the stance laterally so the
#   robot's body CoM is pushed back over centre.
#
#   Similarly, pitch_deg > 0 (front lower) nudges z by +Kp_pitch * pitch_deg.
#
#   The derivative term (Kd) damps oscillation by counteracting the rate of tilt.
#
# Tuning guide:
#   1. Start with all gains at 0, confirm the bot stands stable.
#   2. Increase KP_ROLL slowly (0.02 increments) until you see a correction
#      reaction when you manually tilt the bot — but not oscillation.
#   3. Add KD_ROLL (~half of Kp) to damp the wobble.
#   4. Repeat for pitch.
#   5. Tighten BALANCE_DEADBAND_DEG once gains feel right.
#
KP_ROLL  = 0.04   # cm per degree of roll tilt
KD_ROLL  = 0.02   # cm per degree/s of roll rate
KP_PITCH = 0.04   # cm per degree of pitch tilt
KD_PITCH = 0.02   # cm per degree/s of pitch rate

# Angles below this threshold are treated as zero to avoid jitter on flat ground
BALANCE_DEADBAND_DEG = 1.5

# Hard clamp: maximum correction in any direction (cm)
# Protects IK from being pushed out of valid workspace
BALANCE_MAX_CORRECTION_CM = 3.0


# ── Tap sequence — diagonal walking pattern ───────────────────────────────────
TAP_SEQUENCE = ["fl", "br", "fr", "bl"]

LEG_LABEL = {
    "fl": "LF (motors 1,2,3)",
    "bl": "BL (motors 4,5,6)",
    "fr": "FR (motors 7,8,9)",
    "br": "BR (motors 10,11,12)",
}


# ── Global IMU instance ───────────────────────────────────────────────────────
_imu: ImuReader | None = None


def _start_imu() -> ImuReader:
    global _imu
    print("[Gait] Starting IMU reader...", flush=True)
    imu = ImuReader(i2c_bus=IMU_I2C_BUS, i2c_addr=IMU_I2C_ADDR)
    imu.start()
    _imu = imu
    return imu


def _stop_imu():
    global _imu
    if _imu is not None:
        _imu.stop()
        _imu = None


# ── Balance correction ────────────────────────────────────────────────────────
def _get_balance_correction() -> tuple[float, float]:
    """
    Compute (x_corr_cm, z_corr_cm) from live IMU reading.

    x_corr > 0 → shift all feet in +x direction (bot body shifts left, corrects right lean)
    z_corr > 0 → shift all feet in +z direction (bot body shifts back, corrects forward lean)

    Returns (0, 0) if the IMU is not running (safe fallback).
    """
    if _imu is None:
        return 0.0, 0.0

    roll_deg, pitch_deg = _imu.get_angles()
    roll_rate, pitch_rate = _imu.get_rates()

    # Apply deadband — ignore tiny tilts that are just sensor noise
    if abs(roll_deg)  < BALANCE_DEADBAND_DEG:
        roll_deg  = 0.0
        roll_rate = 0.0
    if abs(pitch_deg) < BALANCE_DEADBAND_DEG:
        pitch_deg  = 0.0
        pitch_rate = 0.0

    # PD control output in cm
    x_corr = KP_ROLL  * roll_deg  + KD_ROLL  * roll_rate
    z_corr = KP_PITCH * pitch_deg + KD_PITCH * pitch_rate

    # Safety clamp
    x_corr = max(-BALANCE_MAX_CORRECTION_CM, min(BALANCE_MAX_CORRECTION_CM, x_corr))
    z_corr = max(-BALANCE_MAX_CORRECTION_CM, min(BALANCE_MAX_CORRECTION_CM, z_corr))

    return x_corr, z_corr


# ── Smooth S-curve interpolation ──────────────────────────────────────────────
def _smooth(t: float) -> float:
    """Cycloidal S-curve: 0→1 with zero velocity at both ends."""
    return t - math.sin(2 * math.pi * t) / (2 * math.pi)


def _interp_y(y_start: float, y_end: float, duration: float) -> list[float]:
    """Return list of y values smoothly interpolated over duration seconds."""
    n = max(2, int(duration * LOOP_HZ))
    return [
        y_start + (y_end - y_start) * _smooth(i / n)
        for i in range(n + 1)
    ]


# ── Socket sender ─────────────────────────────────────────────────────────────
def _send(payload: dict):
    """
    Send a leg coordinate payload to main_with_ik.py.

    "speed" is injected so the main loop uses GAIT_SPEED_DEG_PER_S.
    The caller has already applied balance corrections to the coordinates.
    """
    payload["speed"] = GAIT_SPEED_DEG_PER_S
    data = pickle.dumps(payload)
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(0.5)
            s.connect((HOST, PORT))
            s.sendall(data)
    except Exception as e:
        print(f"\r[Socket Error] {e}", flush=True)


def _send_stand():
    """Send all legs to the standing position with live balance correction."""
    x_corr, z_corr = _get_balance_correction()
    pos = {
        leg: [X_NOM + x_corr, Y_STAND, Z_NOM + z_corr]
        for leg in ["fl", "bl", "fr", "br"]
    }
    _send(pos)


# ── Single leg tap ────────────────────────────────────────────────────────────
def tap_leg(leg: str):
    """
    Smoothly lift `leg` from Y_STAND → Y_TAP, hold, then place back.

    All other 3 legs stay at STAND_POS throughout, with balance correction
    applied fresh on every packet — so the 3 grounded feet continuously
    adjust while the 4th is in the air.
    """
    dt = 1.0 / LOOP_HZ

    # LIFT — y decreases = foot goes up
    for y in _interp_y(Y_STAND, Y_TAP, LIFT_DT):
        x_corr, z_corr = _get_balance_correction()
        # Grounded feet carry the balance offset; lifted foot only moves vertically
        base_x = X_NOM + x_corr
        base_z = Z_NOM + z_corr
        pos = {k: [base_x, Y_STAND, base_z] for k in ["fl", "bl", "fr", "br"]}
        pos[leg] = [base_x, y, base_z]   # tapping leg: same x/z, animated y
        _send(pos)
        time.sleep(dt)

    # HOLD at peak — keep correcting during the dwell
    t_hold_end = time.monotonic() + HOLD_DT
    while time.monotonic() < t_hold_end:
        x_corr, z_corr = _get_balance_correction()
        base_x = X_NOM + x_corr
        base_z = Z_NOM + z_corr
        pos = {k: [base_x, Y_STAND, base_z] for k in ["fl", "bl", "fr", "br"]}
        pos[leg] = [base_x, Y_TAP, base_z]
        _send(pos)
        time.sleep(dt)

    # PLACE — y increases = foot returns to ground
    for y in _interp_y(Y_TAP, Y_STAND, PLACE_DT):
        x_corr, z_corr = _get_balance_correction()
        base_x = X_NOM + x_corr
        base_z = Z_NOM + z_corr
        pos = {k: [base_x, Y_STAND, base_z] for k in ["fl", "bl", "fr", "br"]}
        pos[leg] = [base_x, y, base_z]
        _send(pos)
        time.sleep(dt)

    # Snap all legs to exact standing (with current balance correction)
    _send_stand()
    time.sleep(0.02)


# ── Full tap round ────────────────────────────────────────────────────────────
def run_tap_round(round_num: int):
    """LF → BR → FR → BL, one tap per leg."""
    print(
        f"  [Round {round_num}]  "
        f"Sequence: {' → '.join(LEG_LABEL[l].split(' ')[0] for l in TAP_SEQUENCE)}",
        flush=True,
    )
    for leg in TAP_SEQUENCE:
        print(f"    ↑  Tapping {LEG_LABEL[leg]}...", end="  ", flush=True)
        tap_leg(leg)
        if _imu:
            roll, pitch = _imu.get_angles()
            print(f"✓  [roll={roll:+.1f}°  pitch={pitch:+.1f}°]", flush=True)
        else:
            print("✓", flush=True)
    print(f"  [Round {round_num}] ✓ Complete\n", flush=True)


# ── Keyboard ──────────────────────────────────────────────────────────────────
def _getch() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    per_leg   = LIFT_DT + HOLD_DT + PLACE_DT
    per_round = per_leg * len(TAP_SEQUENCE)

    print("=" * 66)
    print("  Tapping Gait + IMU Balance — BYTE-01  (Real Bot)")
    print("=" * 66)
    print(f"  Standing   : x={X_NOM}  y={Y_STAND}  z={Z_NOM} cm")
    print(f"  Lifted     : x={X_NOM}  y={Y_TAP}    z={Z_NOM} cm")
    print(f"  Lift Δy    : {Y_STAND - Y_TAP:.1f} cm  (y↓ = foot UP)")
    print(f"  Sequence   : LF → BR → FR → BL")
    print(f"  Per leg    : lift={LIFT_DT}s  hold={HOLD_DT}s  place={PLACE_DT}s  = {per_leg:.2f}s")
    print(f"  Per round  : {per_round:.2f}s  (~{60/per_round:.1f} rounds/min)")
    print(f"  Speed limit: {GAIT_SPEED_DEG_PER_S:.0f} deg/s")
    print(f"  Socket     : {HOST}:{PORT}")
    print(f"  IMU        : bus={IMU_I2C_BUS}  addr=0x{IMU_I2C_ADDR:02X}")
    print(f"  Balance    : Kp_roll={KP_ROLL}  Kd_roll={KD_ROLL}")
    print(f"               Kp_pitch={KP_PITCH}  Kd_pitch={KD_PITCH}")
    print(f"               deadband=±{BALANCE_DEADBAND_DEG}°  max_correction=±{BALANCE_MAX_CORRECTION_CM} cm")
    print("  ──────────────────────────────────────────────────────────")
    print("  f  →  queue one tap round")
    print("  n  →  standing pose + clear queue")
    print("  b  →  print current IMU angles")
    print("  q  →  quit")
    print("=" * 66)

    # Start IMU reader before moving any legs
    try:
        _start_imu()
    except Exception as exc:
        print(f"\n⚠️  IMU failed to start: {exc}")
        print("   Continuing WITHOUT balance correction.", flush=True)

    print("\nMoving all legs to standing position...", flush=True)
    _send_stand()
    time.sleep(0.5)
    print("Ready.\n", flush=True)

    # ── Queue-based execution ─────────────────────────────────────────────────
    cmd_queue   = queue.Queue()
    stop_event  = threading.Event()
    round_count = 0

    def worker():
        nonlocal round_count
        while not stop_event.is_set():
            try:
                cmd = cmd_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if cmd == "tap":
                round_count += 1
                queued = cmd_queue.qsize()
                print(
                    f"\n▶  Tap round {round_count} "
                    f"({'%d more queued' % queued if queued else 'last in queue'})...",
                    flush=True,
                )
                run_tap_round(round_count)
                print(
                    f"Queue: {cmd_queue.qsize()} remaining.  "
                    f"(f=next  n=stand  b=imu  q=quit)",
                    flush=True,
                )
            cmd_queue.task_done()

    worker_thread = threading.Thread(target=worker, daemon=True)
    worker_thread.start()

    # ── Keyboard loop ─────────────────────────────────────────────────────────
    try:
        while True:
            key = _getch()

            if key in ("q", "\x03"):
                print("\nQuitting...", flush=True)
                break

            elif key == "f":
                cmd_queue.put("tap")
                print(
                    f"  ↳ Queued tap round  (queue depth: {cmd_queue.qsize()})",
                    flush=True,
                )

            elif key == "n":
                with cmd_queue.mutex:
                    cmd_queue.queue.clear()
                _send_stand()
                print("  Standing pose sent. Queue cleared.", flush=True)

            elif key == "b":
                if _imu:
                    roll, pitch = _imu.get_angles()
                    rr, pr = _imu.get_rates()
                    x_corr, z_corr = _get_balance_correction()
                    print(
                        f"  [IMU] roll={roll:+.2f}°  pitch={pitch:+.2f}°  "
                        f"roll_rate={rr:+.1f}°/s  pitch_rate={pr:+.1f}°/s"
                        f"  → x_corr={x_corr:+.3f} cm  z_corr={z_corr:+.3f} cm",
                        flush=True,
                    )
                else:
                    print("  [IMU] Not running.", flush=True)

            else:
                print(f"  Unknown key '{key}'", flush=True)

    finally:
        stop_event.set()
        worker_thread.join(timeout=3.0)
        print("\nReturning all legs to standing...", flush=True)
        _send_stand()
        _stop_imu()
        print("Done.")


if __name__ == "__main__":
    main()