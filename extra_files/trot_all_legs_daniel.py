#!/usr/bin/env python3
"""
trot_all_legs.py
================
Sends a trot gait to the quadruped over the live socket (same protocol as
main_sender.py).  All motion is expressed as angle offsets on top of the
configurable standing pose, so no IK is involved — the robot moves directly
from its calibrated stand angles.

Trot pattern — diagonal pairs alternate:
  Pair A : fr + bl   (swing during phase [0.0, 0.5))
  Pair B : fl + br   (swing during phase [0.5, 1.0))

Angle array per leg : [motor1°, motor2°, motor3°]
  motor1 = hip pitch   (fore/aft swing, controlled by HIP_SWING)
  motor2 = hip abduct  (kept fixed throughout)
  motor3 = knee        (lift, controlled by TROT_HEIGHT)

Place this file in the same directory as main_sender.py and run:
    python trot_all_legs.py
"""

import time
import math
from main_sender import send_leg_angles   # same directory

# ════════════════════════════════════════════════════════════════════════════
#  ▼▼▼  TUNABLE PARAMETERS — adjust these before running  ▼▼▼
# ════════════════════════════════════════════════════════════════════════════

# ── Standing / resting pose ──────────────────────────────────────────────────
# Motor angles [motor1°, motor2°, motor3°] for each leg at rest.
# The trot starts and ends at these values.
STAND_ANGLES = {
       "fl": [65.0, -135.0, 52.0], # 1, 2, 3 55 (60)  (145 65) 
        "bl": [62.0, -140.0, 55.0], # 4, 5, 6 50
        "fr": [61.0, -140.0, 55.0], # 7, 8, 9 55  (50)
        "br": [57.0, -140.0, 55.0], # 10, 11, 12 50 (45)
}

# ── Trot height ──────────────────────────────────────────────────────────────
# How many degrees motor3 (knee) opens at the peak of the swing arc.
# Larger = higher foot lift.  Typical range: 15 – 40 °.
TROT_HEIGHT: float = 25.0   # degrees

# ── Hip swing ────────────────────────────────────────────────────────────────
# Motor1 (hip pitch) sweeps ± this many degrees fore and aft each step.
# Larger = longer stride.  Typical range: 10 – 25 °.
HIP_SWING: float = 15.0     # degrees

# ── Step time ────────────────────────────────────────────────────────────────
# Duration of ONE half-step (one diagonal pair swings) in seconds.
# One full gait cycle = 2 × STEP_TIME.  Typical range: 0.25 – 0.60 s.
STEP_TIME: float = 0.35     # seconds

# ── Number of complete gait cycles ──────────────────────────────────────────
# One cycle = Pair-A swing + Pair-B swing.
N_CYCLES: int = 4

# ── Hip pitch sign per leg ───────────────────────────────────────────────────
# Depending on how your motors are mounted, left legs may need the opposite
# sign for the hip pitch sweep.  Flip -1 ↔ +1 if a leg swings the wrong way.
HIP_PITCH_SIGN = {
    "fr": +1,
    "br": +1,
    "fl": -1,
    "bl": -1,
}

# ── Knee gear ratio ──────────────────────────────────────────────────────────
# Motor3 (knee) has a 1.5× gear reduction.  All knee angle offsets are
# multiplied by this before being sent so the actual joint moves the intended
# number of degrees.
KNEE_GEAR_RATIO: float = 1.5

# ── Send rate ────────────────────────────────────────────────────────────────
# Packets per second streamed to the socket.  50 – 100 Hz is recommended.
SEND_RATE_HZ: float = 60.0

# ── Socket target ────────────────────────────────────────────────────────────
HOST: str = "127.0.0.1"
PORT: int = 50000

# ════════════════════════════════════════════════════════════════════════════
#  Gait logic — no need to edit below this line
# ════════════════════════════════════════════════════════════════════════════

# Which legs are in each diagonal pair
_PAIR_A = frozenset(("fr", "bl"))   # swings during phase [0.0, 0.5)
_PAIR_B = frozenset(("fl", "br"))   # swings during phase [0.5, 1.0)


def _smooth(t: float) -> float:
    """Raised-cosine smooth-step: t ∈ [0,1] → [0,1], zero velocity at both ends."""
    return 0.5 - 0.5 * math.cos(math.pi * t)


def _swing_offsets(t: float, leg: str) -> list:
    """
    Angle offsets [Δm1, Δm2, Δm3] while the leg is in the air.
    t ∈ [0, 1]: 0 = just left ground (rear), 1 = about to touch down (front).

    Hip pitch: sweeps from −HIP_SWING/2 to +HIP_SWING/2 (forward).
    Knee     : traces a sine arc, peaking at TROT_HEIGHT at mid-swing.
    """
    s = _smooth(t)
    delta_pitch = HIP_PITCH_SIGN[leg] * HIP_SWING * (s - 0.5)
    delta_knee  = TROT_HEIGHT * KNEE_GEAR_RATIO * math.sin(math.pi * t)
    return [delta_pitch, 0.0, delta_knee]


def _stance_offsets(t: float, leg: str) -> list:
    """
    Angle offsets while the leg is on the ground.
    t ∈ [0, 1]: 0 = just planted (front), 1 = about to lift (rear).

    Hip pitch: sweeps from +HIP_SWING/2 to −HIP_SWING/2 (pushes body forward).
    Knee     : stays at ground level (no offset).
    """
    delta_pitch = HIP_PITCH_SIGN[leg] * HIP_SWING * (0.5 - t)
    return [delta_pitch, 0.0, 0.0]


def _build_payload(phase: float) -> dict:
    """
    Compute the full 4-leg angle payload for gait phase ∈ [0, 1).

    Phase convention
    ----------------
    [0.0, 0.5) → Pair A (fr+bl) swinging,  Pair B (fl+br) in stance
    [0.5, 1.0) → Pair B (fl+br) swinging,  Pair A (fr+bl) in stance
    """
    payload = {}

    for leg in ("fl", "bl", "fr", "br"):
        base = STAND_ANGLES[leg]

        # Map the leg's local swing window to [0, 1)
        # Pair A swings first; Pair B is offset by half a cycle.
        if leg in _PAIR_A:
            local = phase                        # swings in [0.0, 0.5)
        else:
            local = (phase + 0.5) % 1.0          # swings in [0.5, 1.0)

        if local < 0.5:                          # swing phase for this leg
            t = local / 0.5
            offsets = _swing_offsets(t, leg)
        else:                                    # stance phase for this leg
            t = (local - 0.5) / 0.5
            offsets = _stance_offsets(t, leg)

        payload[leg] = [b + o for b, o in zip(base, offsets)]

    return payload


def run_trot() -> None:
    dt               = 1.0 / SEND_RATE_HZ
    frames_per_step  = max(2, int(round(STEP_TIME * SEND_RATE_HZ)))
    total_half_steps = N_CYCLES * 2            # each cycle has 2 half-steps

    print("=" * 58)
    print("  🦿  Quadruped Trot Gait Controller")
    print(f"  TROT_HEIGHT  = {TROT_HEIGHT} °")
    print(f"  HIP_SWING    = {HIP_SWING} °")
    print(f"  STEP_TIME    = {STEP_TIME} s  ({frames_per_step} frames / step)")
    print(f"  N_CYCLES     = {N_CYCLES}  →  {total_half_steps} half-steps total")
    print(f"  SEND_RATE    = {SEND_RATE_HZ} Hz")
    print(f"  Target       = {HOST}:{PORT}")
    print("=" * 58)
    print()

    try:
        print("▶  Trotting...\n")

        for half_step in range(total_half_steps):
            cycle_num  = half_step // 2 + 1
            pair_label = "fr+bl swing" if half_step % 2 == 0 else "fl+br swing"
            print(f"  Cycle {cycle_num}/{N_CYCLES}  —  {pair_label}")

            for frame in range(frames_per_step):
                t0 = time.perf_counter()

                # Global phase for this frame
                # Each half-step covers 0.5 of the phase space [0, 1)
                half_offset = (half_step % 2) * 0.5
                phase = (half_offset + (frame / frames_per_step) * 0.5) % 1.0

                payload = _build_payload(phase)
                send_leg_angles(payload, host=HOST, port=PORT)

                # Pace to target send rate
                elapsed = time.perf_counter() - t0
                remaining = dt - elapsed
                if remaining > 0:
                    time.sleep(remaining)

    except KeyboardInterrupt:
        print("\n🛑  Interrupted — returning to stand.")

    finally:
        # Always send the stand pose a couple of times to settle the robot
        print("\n🧍  Returning to stand pose...")
        for _ in range(3):
            send_leg_angles(STAND_ANGLES, host=HOST, port=PORT)
            time.sleep(0.05)
        print("✅  Done.")


if __name__ == "__main__":
    run_trot()