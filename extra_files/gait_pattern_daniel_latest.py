#!/usr/bin/env python3
"""
Configurable quadruped gait generator for the 12-DOF IK pipeline.

This script sends grouped leg coordinate payloads to main_with_ik.py:

    {"fl": [x, y, z], "bl": [x, y, z], "fr": [x, y, z], "br": [x, y, z]}

The gait here is a simple diagonal trot:
    Phase 1: FR + BL swing forward/up, FL + BR stance return/down
    Phase 2: FL + BR swing forward/up, FR + BL stance return/down

Tuning knobs near the top:
    STEP_LENGTH_CM       → forward/back stride length
    WAYPOINTS_PER_PHASE   → number of interpolation samples in each half-step
    STEP_TIME_UP_S        → swing time
    STEP_TIME_DOWN_S      → stance time
    STEP_HEIGHT_CM       → lift/press amount in the Y axis
    STEP_DIRECTION        → +1 or -1 depending on your coordinate convention

Run this after main_with_ik.py is already in post-homing live control mode.
"""

from __future__ import annotations

import math
import pickle
import socket
import sys
import termios
import time
import tty
from typing import Dict, Iterable, List, Tuple

# ── Socket ────────────────────────────────────────────────────────────────────
HOST = "127.0.0.1"
PORT = 50000

# ── Nominal foot position (matches inverse_kinematics.py) ────────────────────
X_NOM = -9.094
Y_NOM = 30.0
Z_NOM = 3.0

# ── Gait parameters you asked to tune ────────────────────────────────────────
STEP_LENGTH_CM = 10.0        # total horizontal stride in cm
WAYPOINTS_PER_PHASE = 30     # number of waypoints in swing / stance
STEP_TIME_UP_S = 0.6       # swing phase duration
STEP_TIME_DOWN_S = 0.6     # stance phase duration
STEP_HEIGHT_CM = 3.0         # vertical lift / press in cm

# Forward/back direction in your z-axis convention.
# Reference gait used -1 for forward.
STEP_DIRECTION = -1

# Optional pause between cycles
CYCLE_PAUSE_S = 0.0

# ── Timing ───────────────────────────────────────────────────────────────────
LOOP_HZ = 50.0
DT = 1.0 / LOOP_HZ

# ── Leg order used by main_with_ik.py ────────────────────────────────────────
LEG_ORDER = ("fl", "bl", "fr", "br")

# ── Starting diagonal pair for trot ──────────────────────────────────────────
PAIR_A = ("fr", "bl")
PAIR_B = ("fl", "br")


def _horiz(t: float) -> float:
    """Smooth 0→1 cycloidal easing."""
    return t - math.sin(2.0 * math.pi * t) / (2.0 * math.pi)


def _vert(t: float) -> float:
    """0 at edges, 1 at the center."""
    return 1.0 - math.cos(2.0 * math.pi * t)


def _make_payload(fl: Tuple[float, float, float],
                  bl: Tuple[float, float, float],
                  fr: Tuple[float, float, float],
                  br: Tuple[float, float, float]) -> Dict[str, List[float]]:
    return {
        "fl": [float(fl[0]), float(fl[1]), float(fl[2])],
        "bl": [float(bl[0]), float(bl[1]), float(bl[2])],
        "fr": [float(fr[0]), float(fr[1]), float(fr[2])],
        "br": [float(br[0]), float(br[1]), float(br[2])],
    }


def _send_payload(payload: Dict[str, List[float]]) -> None:
    data = pickle.dumps(payload)
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(0.5)
            s.connect((HOST, PORT))
            s.sendall(data)
    except Exception as exc:
        print(f"\r[Socket Error] {exc}", flush=True)


def _getch() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _interp_leg(
    start_z: float,
    end_z: float,
    lift_up: bool,
    waypoints: int,
) -> List[Tuple[float, float, float]]:
    """
    Interpolate one leg between two foot z positions.
    X is fixed; Y gets the lift / press arc; Z follows a smooth stride.
    """
    n = max(1, int(waypoints))
    sign = -1.0 if lift_up else 1.0
    pts: List[Tuple[float, float, float]] = []

    for i in range(n + 1):
        t = i / n
        z = start_z + (end_z - start_z) * _horiz(t)
        y = Y_NOM + sign * (STEP_HEIGHT_CM / 2.0) * _vert(t)
        pts.append((X_NOM, y, z))

    return pts


def _cycle_segment(
    moving_legs: Iterable[str],
    support_legs: Iterable[str],
    swing_start_z: float,
    swing_end_z: float,
    phase_duration_s: float,
) -> None:
    """
    Run one half-cycle.

    moving_legs  → swing forward/up
    support_legs → stance return/down
    """
    moving_legs = tuple(moving_legs)
    support_legs = tuple(support_legs)

    moving_pts = _interp_leg(
        start_z=swing_start_z,
        end_z=swing_end_z,
        lift_up=True,
        waypoints=WAYPOINTS_PER_PHASE,
    )

    support_pts = _interp_leg(
        start_z=swing_end_z,
        end_z=swing_start_z,
        lift_up=False,
        waypoints=WAYPOINTS_PER_PHASE,
    )

    # Both trajectories complete in the same phase duration.
    dt = phase_duration_s / max(1, WAYPOINTS_PER_PHASE)
    step_count = min(len(moving_pts), len(support_pts))

    for i in range(step_count):
        swing_pose = moving_pts[i]
        stance_pose = support_pts[i]

        payload = {
            "fl": [X_NOM, Y_NOM, Z_NOM],
            "bl": [X_NOM, Y_NOM, Z_NOM],
            "fr": [X_NOM, Y_NOM, Z_NOM],
            "br": [X_NOM, Y_NOM, Z_NOM],
        }

        for leg in moving_legs:
            payload[leg] = [swing_pose[0], swing_pose[1], swing_pose[2]]
        for leg in support_legs:
            payload[leg] = [stance_pose[0], stance_pose[1], stance_pose[2]]

        _send_payload(payload)
        time.sleep(dt)


def run_gait_cycle(cycle_num: int) -> None:
    """
    Execute one full trot cycle:
        Phase 1: FR + BL swing, FL + BR stance
        Phase 2: FL + BR swing, FR + BL stance
    """
    z_back = Z_NOM
    z_front = Z_NOM + (STEP_LENGTH_CM * STEP_DIRECTION)

    print(
        f"\n[Cycle {cycle_num}] "
        f"step={STEP_LENGTH_CM:.2f} cm, waypoints={WAYPOINTS_PER_PHASE}, "
        f"up={STEP_TIME_UP_S:.2f}s, down={STEP_TIME_DOWN_S:.2f}s, "
        f"dir={STEP_DIRECTION:+d}"
    )

    print(
        f"  Phase 1: FR + BL swing {z_back:.2f} → {z_front:.2f}, "
        f"FL + BR support {z_front:.2f} → {z_back:.2f}"
    )
    _cycle_segment(
        moving_legs=PAIR_A,
        support_legs=PAIR_B,
        swing_start_z=z_back,
        swing_end_z=z_front,
        phase_duration_s=STEP_TIME_UP_S,
    )

    print(
        f"  Phase 2: FL + BR swing {z_back:.2f} → {z_front:.2f}, "
        f"FR + BL support {z_front:.2f} → {z_back:.2f}"
    )
    _cycle_segment(
        moving_legs=PAIR_B,
        support_legs=PAIR_A,
        swing_start_z=z_back,
        swing_end_z=z_front,
        phase_duration_s=STEP_TIME_DOWN_S,
    )

    if CYCLE_PAUSE_S > 0:
        time.sleep(CYCLE_PAUSE_S)

    # Return all legs to nominal at the end of the cycle.
    _send_payload(_make_payload(
        (X_NOM, Y_NOM, Z_NOM),
        (X_NOM, Y_NOM, Z_NOM),
        (X_NOM, Y_NOM, Z_NOM),
        (X_NOM, Y_NOM, Z_NOM),
    ))


def main() -> None:
    print("=" * 72)
    print("  Configurable Diagonal Trot Gait")
    print("=" * 72)
    print(f"  Nominal pose   : x={X_NOM}  y={Y_NOM}  z={Z_NOM} cm")
    print(f"  Step length    : {STEP_LENGTH_CM} cm")
    print(f"  Waypoints      : {WAYPOINTS_PER_PHASE} per phase")
    print(f"  Step up time   : {STEP_TIME_UP_S} s")
    print(f"  Step down time : {STEP_TIME_DOWN_S} s")
    print(f"  Step height    : {STEP_HEIGHT_CM} cm")
    print(f"  Direction      : {STEP_DIRECTION:+d}  (z change = step_length × direction)")
    print("=" * 72)
    print("  f  →  one gait cycle")
    print("  q  →  quit")
    print("=" * 72)

    # Move all legs to nominal first.
    _send_payload(_make_payload(
        (X_NOM, Y_NOM, Z_NOM),
        (X_NOM, Y_NOM, Z_NOM),
        (X_NOM, Y_NOM, Z_NOM),
        (X_NOM, Y_NOM, Z_NOM),
    ))
    time.sleep(0.5)
    print("Ready.\n", flush=True)

    cycle_count = 0

    try:
        while True:
            key = _getch()

            if key in ("q", "\x03"):
                print("\nQuitting...", flush=True)
                break

            if key == "f":
                cycle_count += 1
                print(f"\n▶  Cycle {cycle_count}", flush=True)
                run_gait_cycle(cycle_count)
                print(f"\nReady. Total cycles: {cycle_count}  (f=next  q=quit)\n", flush=True)
            else:
                print(f"\rUnknown key '{key}'", flush=True)

    finally:
        print("\nReturning all legs to nominal...", flush=True)
        _send_payload(_make_payload(
            (X_NOM, Y_NOM, Z_NOM),
            (X_NOM, Y_NOM, Z_NOM),
            (X_NOM, Y_NOM, Z_NOM),
            (X_NOM, Y_NOM, Z_NOM),
        ))
        print("Done.")


if __name__ == "__main__":
    main()
