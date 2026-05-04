#!/usr/bin/env python3
"""
mpu_leveling.py
════════════════════════════════════════════════════════════════════════════════
BYTE-01  MPU-6050  One-Shot Chassis Leveling
────────────────────────────────────────────────────────────────────────────────

PURPOSE
-------
After every homing + stand transition the chassis can tilt because the homing
offset shifts slightly each run.  This module reads the MPU-6050 once (averaged)
after the bot stands up, computes how much each leg Y must change to bring the
chassis back to horizontal, and fires a single corrected XYZ payload through
the existing socket to main_with_ik.py — exactly like the gait files do.

FLOW (call from your gait file)
---------------------------------
    leveler = MPULeveler()

    # ── Startup ──────────────────────────────────────────────────────
    send_all_to(SITTING_XYZ)
    leveler.capture_reference()          # <─ call while bot is sitting flat
    time.sleep(1.5)

    smooth_transition(STANDING_XYZ)      # existing stand transition
    leveler.level_after_stand()          # <─ one-shot correction after standing

    # ── Optional: re-level after every explicit 'stand' command ──────
    # Same two lines work inside the 'stand' command branch too.

ROBOT COORDINATE FRAME
-----------------------
    X  = lateral shift        (right = positive)
    Y  = vertical, DOWN +ve   (larger Y = leg more extended downward)
    Z  = forward

    Sitting  nominal:  (-9.094,  11.0, 3.0)
    Standing nominal:  (-9.094,  25.0, 3.0)

MPU-6050 MOUNTING ASSUMPTION
------------------------------
    Board is flat on top of chassis, MPU chip facing up.
    By default this code assumes:
        FORWARD_AXIS = 0   (MPU ax  → robot forward / Z direction)
        LATERAL_AXIS = 1   (MPU ay  → robot lateral / X direction, right +ve)

    If pitch or roll corrections are backwards after a physical tilt test,
    flip the corresponding sign constant:
        FORWARD_SIGN = -1   (reverses pitch correction direction)
        LATERAL_SIGN = -1   (reverses roll  correction direction)

    If the board is rotated 90° on the chassis swap the axis indices:
        FORWARD_AXIS = 1
        LATERAL_AXIS = 0

SIGN CONVENTION FOR CORRECTIONS
---------------------------------
    delta_pitch > 0  →  front of chassis tilted DOWN
                        → front legs get MORE Y (extend to push front UP)
                        → rear  legs get LESS Y

    delta_roll  > 0  →  right side tilted DOWN
                        → right legs get MORE Y
                        → left  legs get LESS Y

    Per-leg Y formula:
        FL :  STANDING_Y + dy_pitch − dy_roll
        FR :  STANDING_Y + dy_pitch + dy_roll
        BL :  STANDING_Y − dy_pitch − dy_roll
        BR :  STANDING_Y − dy_pitch + dy_roll

ROBOT GEOMETRY
--------------
    fore_aft distance (front hip to rear hip)  = 42 cm  →  half = 21 cm
    lateral  distance (left hip to right hip)  = 28 cm  →  half = 14 cm

SOCKET
------
    Sends to HOST:PORT = 127.0.0.1:50000 (same as gait files).
    Payload format  : pickle'd dict  {"fl":[x,y,z], "fr":..., "bl":..., "br":..., "speed": float}
    Speed used      : TRANSITION_SPEED_DEG_PER_S = 120 deg/s  (slow, safe correction)
"""

import math
import pickle
import socket
import time
import smbus2


# ─── Socket (must match main_with_ik.py) ────────────────────────────────────
from config.robot_config import SOCKET_HOST, SOCKET_PORT, LEG_ORDER

HOST = SOCKET_HOST
PORT = SOCKET_PORT

# ─── Speed for the leveling correction send ──────────────────────────────────
LEVEL_SPEED_DEG_PER_S = 120.0     # slow and safe — same as transition speed

# ─── MPU-6050 I²C ────────────────────────────────────────────────────────────
MPU_I2C_BUS  = 1       # Raspberry Pi default I²C bus
MPU_ADDR     = 0x68    # AD0 pulled LOW (default)
PWR_MGMT_1   = 0x6B
ACCEL_XOUT_H = 0x3B    # first of 6 bytes: AX_H AX_L AY_H AY_L AZ_H AZ_L

# ─── Axis mapping ─────────────────────────────────────────────────────────────
# Raw accel tuple index:  0 = ax,  1 = ay,  2 = az
# Default: board X points toward robot front (robot Z), board Y points robot right (X)
FORWARD_AXIS = 0    # which raw accel index carries the robot-forward tilt signal
LATERAL_AXIS = 1    # which raw accel index carries the robot-lateral tilt signal
FORWARD_SIGN = +1   # flip to -1 if pitch correction acts backwards on real hardware
LATERAL_SIGN = +1   # flip to -1 if roll  correction acts backwards on real hardware

# ─── Robot geometry ──────────────────────────────────────────────────────────
FORE_AFT_HALF = 42.0 / 2.0    # cm — distance from chassis centre to front/rear hips
LATERAL_HALF  = 28.0 / 2.0    # cm — distance from chassis centre to left/right hips

# ─── Nominal foot targets ─────────────────────────────────────────────────────
SITTING_XYZ  = [-9.094, 11.0, 3.0]
STANDING_XYZ = [-9.094, 25.0, 3.0]

STANDING_X = STANDING_XYZ[0]
STANDING_Y = STANDING_XYZ[1]
STANDING_Z = STANDING_XYZ[2]

# ─── Y safety clamp ──────────────────────────────────────────────────────────
# Correction should never push a leg beyond these Y bounds.
Y_MIN = SITTING_XYZ[1]          # 11.0 cm — can't retract past sitting height
Y_MAX = STANDING_Y + 6.0        # 31.0 cm — max safe extension beyond nominal stand

# ─── Averaging samples ────────────────────────────────────────────────────────
N_SAMPLES_REF   = 50    # samples for reference capture (once, while sitting)
N_SAMPLES_LEVEL = 30    # samples for post-stand leveling read


# ─────────────────────────────────────────────────────────────────────────────
class MPULeveler:
    """
    Reads the MPU-6050 accelerometer to compute chassis tilt and send a
    one-shot corrected foot-position payload via socket.

    Usage
    -----
        leveler = MPULeveler()

        # while sitting:
        leveler.capture_reference()

        # after standing:
        leveler.level_after_stand()
    """

    def __init__(self):
        self._bus = smbus2.SMBus(MPU_I2C_BUS)
        self._wake_mpu()
        self._ref_pitch    = 0.0
        self._ref_roll     = 0.0
        self._ref_captured = False
        print("[MPU] Initialized — I²C bus 1, address 0x68")

    # ── Hardware ──────────────────────────────────────────────────────────────

    def _wake_mpu(self):
        """Take MPU out of sleep mode and allow 150 ms to stabilise."""
        self._bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x00)
        time.sleep(0.15)

    def _read_raw_accel(self) -> tuple:
        """
        Read one raw accelerometer sample.
        Returns (ax, ay, az) as signed 16-bit integers (±32767).
        Default ±2 g full-scale → 1 g ≈ 16384 LSB.
        """
        data = self._bus.read_i2c_block_data(MPU_ADDR, ACCEL_XOUT_H, 6)

        def to_signed(hi_byte, lo_byte):
            v = (hi_byte << 8) | lo_byte
            return v - 65536 if v > 32767 else v

        ax = to_signed(data[0], data[1])
        ay = to_signed(data[2], data[3])
        az = to_signed(data[4], data[5])
        return ax, ay, az

    def _read_averaged(self, n: int) -> tuple:
        """
        Average n accelerometer readings with 5 ms spacing between each.
        Returns (ax_avg, ay_avg, az_avg) as floats.
        """
        total = [0.0, 0.0, 0.0]
        for _ in range(n):
            raw = self._read_raw_accel()
            for i in range(3):
                total[i] += raw[i]
            time.sleep(0.005)
        return tuple(t / n for t in total)

    # ── Tilt computation ──────────────────────────────────────────────────────

    def _accel_to_tilt(self, ax: float, ay: float, az: float) -> tuple:
        """
        Convert raw accelerometer values to chassis tilt angles in degrees.

        pitch > 0  →  front of chassis tilted DOWN
        roll  > 0  →  right side of chassis tilted DOWN

        The formula uses the full gravity magnitude for the "vertical"
        component so that reading stays stable even with combined pitch+roll.
        """
        accel = [ax, ay, az]

        # Apply axis mapping and sign
        a_forward = FORWARD_SIGN * accel[FORWARD_AXIS]   # gravity component along robot forward
        a_lateral = LATERAL_SIGN * accel[LATERAL_AXIS]   # gravity component along robot right

        # Vertical component = magnitude of gravity minus the two horizontal projections
        g_sq      = ax**2 + ay**2 + az**2
        a_vert_sq = max(g_sq - a_forward**2 - a_lateral**2, 0.0)
        a_vert    = math.sqrt(a_vert_sq)

        pitch = math.degrees(math.atan2(a_forward, a_vert))   # forward tilt
        roll  = math.degrees(math.atan2(a_lateral, a_vert))   # lateral tilt
        return pitch, roll

    # ── Socket send ───────────────────────────────────────────────────────────

    def _send_targets(self, targets: dict):
        """
        Send the corrected XYZ foot targets through the socket.
        Identical pickle format to sidd_gait.py / trot_gait_node_sid.py.
        """
        payload = dict(targets)
        payload["speed"] = LEVEL_SPEED_DEG_PER_S
        data = pickle.dumps(payload)
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(0.5)
                s.connect((HOST, PORT))
                s.sendall(data)
        except Exception as e:
            print(f"[MPU] Socket error: {e}")

    # ── Public API ────────────────────────────────────────────────────────────

    def capture_reference(self):
        """
        Call this ONCE while the bot is in sitting pose resting on its chassis
        (chassis guaranteed horizontal by physical contact with the ground).

        Records the MPU tilt at this known-flat state so that post-stand tilt
        is computed as a delta from here, removing any MPU mounting offset.
        """
        print("[MPU] Capturing reference — keep bot sitting flat...")
        ax, ay, az          = self._read_averaged(N_SAMPLES_REF)
        self._ref_pitch, self._ref_roll = self._accel_to_tilt(ax, ay, az)
        self._ref_captured  = True
        print(f"[MPU] Reference locked — "
              f"pitch_ref={self._ref_pitch:+.3f}°  roll_ref={self._ref_roll:+.3f}°")

    def validate_health(self, n: int = 50) -> bool:
        """
        Perform n consecutive raw reads and return True only if every read
        succeeds with plausible values.

        Used as a startup gate: confirms the MPU is alive on I²C, returning
        sensible (non-zero, in-range) data before the rest of the system
        relies on it for live correction.
        """
        print(f"[MPU] Health check — performing {n} consecutive reads...")
        failures = 0
        for i in range(n):
            try:
                raw = self._read_raw_accel()
                if raw == (0, 0, 0):
                    failures += 1
                    print(f"[MPU] Read {i+1}: all-zero readout (sensor unresponsive)")
                elif any(abs(v) > 32767 for v in raw):
                    failures += 1
                    print(f"[MPU] Read {i+1}: out-of-range {raw}")
            except Exception as e:
                failures += 1
                print(f"[MPU] Read {i+1} FAILED: {e}")
            time.sleep(0.005)

        if failures == 0:
            print(f"[MPU] Health check passed — {n}/{n} reads OK.")
            return True
        print(f"[MPU] Health check FAILED — {failures}/{n} reads errored.")
        return False

    def compute_level_targets(self, n_samples: int = N_SAMPLES_LEVEL,
                              verbose: bool = True) -> dict:
        """
        Read current chassis tilt, compute per-leg Y corrections and return
        the four corrected foot targets.

        Args
        ----
        n_samples : how many MPU samples to average for this read. Lower values
                    speed up the call for use in a live correction loop.
        verbose   : suppress per-call printout when running from a tight loop.

        Returns
        -------
        dict  {"fl": [x, y, z],  "fr": [x, y, z],
               "bl": [x, y, z],  "br": [x, y, z]}

        Raises
        ------
        RuntimeError  if capture_reference() has not been called first.
        """
        if not self._ref_captured:
            raise RuntimeError(
                "[MPU] capture_reference() must be called before compute_level_targets()."
            )

        # ── Read current tilt ─────────────────────────────────────────────────
        ax, ay, az    = self._read_averaged(n_samples)
        pitch, roll   = self._accel_to_tilt(ax, ay, az)

        delta_pitch   = pitch - self._ref_pitch   # + → front tilted down
        delta_roll    = roll  - self._ref_roll    # + → right side tilted down

        # ── Convert angle to cm height difference at each hip ─────────────────
        # tan(angle) × half-distance = how much higher/lower that side needs to move
        dy_pitch = math.tan(math.radians(delta_pitch)) * FORE_AFT_HALF
        dy_roll  = math.tan(math.radians(delta_roll))  * LATERAL_HALF

        # ── Build per-leg Y targets ────────────────────────────────────────────
        #   Front legs: +dy_pitch  (more extension pushes front chassis UP)
        #   Rear  legs: -dy_pitch
        #   Right legs: +dy_roll   (more extension pushes right chassis UP)
        #   Left  legs: -dy_roll
        raw = {
            "fl": STANDING_Y + dy_pitch - dy_roll,
            "fr": STANDING_Y + dy_pitch + dy_roll,
            "bl": STANDING_Y - dy_pitch - dy_roll,
            "br": STANDING_Y - dy_pitch + dy_roll,
        }

        # ── Safety clamp ──────────────────────────────────────────────────────
        clamped = {leg: max(Y_MIN, min(Y_MAX, y)) for leg, y in raw.items()}

        targets = {
            leg: [STANDING_X, clamped[leg], STANDING_Z]
            for leg in ("fl", "fr", "bl", "br")
        }

        # ── Diagnostics ───────────────────────────────────────────────────────
        if verbose:
            print(f"\n[MPU] Tilt — "
                  f"pitch: {delta_pitch:+.3f}°   roll: {delta_roll:+.3f}°")
            print(f"[MPU] Height deltas — "
                  f"dy_pitch: {dy_pitch:+.4f} cm   dy_roll: {dy_roll:+.4f} cm")
            print(f"  {'Leg':<4}  {'Y target':>10}  {'Δ from nominal':>16}  {'Clamped?':>8}")
            print("  " + "─" * 44)
            for leg in ("fl", "fr", "bl", "br"):
                y_target = clamped[leg]
                y_raw    = raw[leg]
                delta    = y_target - STANDING_Y
                clamp_note = " ← CLAMPED" if abs(y_target - y_raw) > 1e-6 else ""
                print(f"  {leg.upper():<4}  {y_target:>10.4f}  {delta:>+16.4f}{clamp_note}")
            print()

        return targets

    def level_after_stand(self):
        """
        One-shot leveling call — run this immediately after every stand transition.

        Internally:
          1. Reads MPU (averaged over N_SAMPLES_LEVEL readings)
          2. Computes delta pitch and roll vs the sitting reference
          3. Derives per-leg Y correction
          4. Sends the corrected payload once via socket to main_with_ik.py
        """
        print("[MPU] Computing post-stand level correction...")
        targets = self.compute_level_targets()
        self._send_targets(targets)
        print("[MPU] Level correction sent — one packet dispatched.")


# ─────────────────────────────────────────────────────────────────────────────
# Integration guide (add these lines to your gait file)
# ─────────────────────────────────────────────────────────────────────────────
#
#   from mpu_leveling import MPULeveler
#
#   leveler = MPULeveler()           # ← once, at module level
#
#   # Inside main(), after send_all_to(SITTING_XYZ):
#   leveler.capture_reference()      # chassis flat on ground → lock reference
#   time.sleep(1.5)
#
#   # After smooth_transition(STANDING_XYZ):
#   leveler.level_after_stand()      # read MPU, correct, send once
#
#   # Inside the 'stand' command branch (after smooth_transition):
#   leveler.level_after_stand()      # same call — works every time
#
# ─────────────────────────────────────────────────────────────────────────────
# Axis troubleshooting
# ─────────────────────────────────────────────────────────────────────────────
# Step 1 — run standalone:  python3 mpu_leveling.py
#           This prints raw tilt values. Physically tilt the bot forward and
#           check that delta_pitch becomes positive. If it goes negative,
#           set FORWARD_SIGN = -1 at the top of this file.
#           Tilt right and check delta_roll is positive. Flip LATERAL_SIGN
#           if not.
#
# Step 2 — if corrections are on the wrong axis entirely (pitch is responding
#           to a lateral tilt), swap FORWARD_AXIS and LATERAL_AXIS.
# ─────────────────────────────────────────────────────────────────────────────


# ─── Standalone test (no socket, no robot needed) ────────────────────────────
if __name__ == "__main__":
    import sys

    print("=" * 60)
    print("  MPU-6050 Leveling — Axis Verification Mode")
    print("  Keep the bot sitting flat, then tilt it manually")
    print("  to verify pitch/roll sign conventions.")
    print("  Ctrl+C to exit.")
    print("=" * 60)

    try:
        leveler = MPULeveler()
    except Exception as e:
        print(f"\n[ERROR] Could not initialise MPU: {e}")
        print("  Check: I²C enabled on Pi?  smbus2 installed?  AD0 wiring?")
        sys.exit(1)

    print("\n[Step 1] Capturing reference (keep chassis flat)...")
    leveler.capture_reference()

    print("\n[Step 2] Live tilt readout — tilt the bot to verify signs:")
    print(f"  {'pitch':>10}  {'roll':>10}  {'dy_pitch':>12}  {'dy_roll':>12}")
    print("  " + "─" * 50)

    try:
        while True:
            ax, ay, az = leveler._read_averaged(10)
            pitch, roll = leveler._accel_to_tilt(ax, ay, az)
            dp = pitch - leveler._ref_pitch
            dr = roll  - leveler._ref_roll
            dy_p = math.tan(math.radians(dp)) * FORE_AFT_HALF
            dy_r = math.tan(math.radians(dr)) * LATERAL_HALF
            print(f"  {dp:>+10.3f}°  {dr:>+10.3f}°  {dy_p:>+12.4f}cm  {dy_r:>+12.4f}cm",
                  end="\r", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\n[Done]")