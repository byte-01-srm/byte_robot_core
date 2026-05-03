#!/usr/bin/env python3
"""
walk_gait_sender.py
====================
Streams the quadruped walk-gait pattern to the real bot by sending
leg-angle payloads over the same TCP socket that main.py listens on.

Flow:
  1. main.py runs → homes all 12 motors → starts socket listener
  2. You run this script → it sends one gait-step payload per cycle tick
  3. main.py receives the payload, applies flip/gear_ratio, and commands
     the AK60-6 V3.0 motors via CAN.

Joint / motor mapping (matches main.py LEG_ORDER = fl, bl, fr, br):
  fl (front-left)  → LF in gait → motors 1(roll), 2(hip), 3(knee)
  bl (back-left)   → LH in gait → motors 4(roll), 5(hip), 6(knee)
  fr (front-right) → RF in gait → motors 7(roll), 8(hip), 9(knee)
  br (back-right)  → RH in gait → motors 10(roll),11(hip),12(knee)

Angle convention:
  The gait node stores positions[] in URDF radians.
  main.py/transform_live_angles expects RAW degrees (pre flip/gear_ratio).
  So we convert: raw_deg = degrees(urdf_rad)
  main.py then does:   if flipped: angle = -angle
                       angle *= gear_ratio
  before sending to the motor.

Usage:
  # Terminal 1 – already running:
  python3 main.py

  # Terminal 2:
  python3 walk_gait_sender.py             # forward walk, loops forever
  python3 walk_gait_sender.py --mode b    # backward
  python3 walk_gait_sender.py --mode r    # clockwise turn
  python3 walk_gait_sender.py --mode l    # anticlockwise turn
  python3 walk_gait_sender.py --cycles 5  # 5 cycles then stop
"""

import argparse
import math
import pickle
import socket
import time
from math import atan2, cos, sin, sqrt

import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
# Socket settings (must match main.py)
# ─────────────────────────────────────────────────────────────────────────────
HOST = "127.0.0.1"
PORT = 50000

# ─────────────────────────────────────────────────────────────────────────────
# Gait configuration  (keep in sync with walk_gait_quad.py)
# ─────────────────────────────────────────────────────────────────────────────
CYCLE_DURATION    = 1.2          # seconds per full gait cycle
NUM_STEPS         = 24           # trajectory samples per cycle
DELAY_PER_STEP    = CYCLE_DURATION / NUM_STEPS

L2 = 22.0                        # thigh length (cm)
L3 = 21.5                        # shank length (cm)

STRIDE_LENGTH     = 16.0         # cm
SWING_HEIGHT      = 6.0          # cm
MU                = 0.05         # dwell fraction at swing endpoints
DELTA             = 0.0          # body-z oscillation (cm)
DUTY_FACTOR       = 0.75         # 75 % stance, 25 % swing
TURN_INNER_STRIDE = 8.0          # cm inside-leg stride for turns

BODY_Z_HEIGHT     = 23.973       # cm
NEUTRAL_FOOT_X    = 0.946        # cm (horizontal offset at neutral)
NEUTRAL_FOOT_Z    = -BODY_Z_HEIGHT

# Phase offsets (walk gait)
PHASE_SHIFTS = {'LH': 0.00, 'LF': 0.25, 'RH': 0.50, 'RF': 0.75}

# URDF joint-axis signs: (hip_pitch_sign, knee_sign)
LEG_JOINT_SIGNS = {
    'LF': (+1, -1),
    'LH': (+1, -1),
    'RF': (-1, +1),
    'RH': (-1, +1),
}

# Neutral-pose offsets (roll, hip_pitch, knee) in radians
LEG_OFFSETS = {
    'LF': (0.0, +2.58, +1.97),
    'LH': (0.0, +2.58, +1.97),
    'RF': (0.0, -2.58, -1.97),
    'RH': (0.0, -2.58, -1.97),
}

# Gait-leg → socket-leg key mapping
GAIT_TO_SOCKET = {'LF': 'fl', 'LH': 'bl', 'RF': 'fr', 'RH': 'br'}

# Physical groupings for turning
PHYS_RIGHT_LEGS = ['LF', 'LH']
PHYS_LEFT_LEGS  = ['RF', 'RH']

MODE_FORWARD       = 'FORWARD'
MODE_BACKWARD      = 'BACKWARD'
MODE_CLOCKWISE     = 'CLOCKWISE'
MODE_ANTICLOCKWISE = 'ANTICLOCKWISE'

# ─────────────────────────────────────────────────────────────────────────────
# IK / Trajectory helpers
# ─────────────────────────────────────────────────────────────────────────────

def calc_2dof_ik(x: float, z: float):
    L = sqrt(x * x + z * z)
    lo, hi = abs(L2 - L3), L2 + L3
    if not (lo <= L <= hi):
        return None
    c3 = (L * L - L2 * L2 - L3 * L3) / (2.0 * L2 * L3)
    c3 = max(-1.0, min(1.0, c3))
    theta3 = math.acos(c3)
    alpha   = atan2(L3 * sin(theta3), L2 + L3 * cos(theta3))
    theta2  = atan2(z, x) - alpha
    return theta2, theta3


def _swing_traj(t_norm: np.ndarray, stride: float, direction: int):
    t   = np.asarray(t_norm, dtype=float)
    T_p = MU
    x   = np.empty_like(t)
    x[t < T_p] = direction * (-stride / 2.0)
    m = (t >= T_p) & (t < 1.0 - T_p)
    tm = (t[m] - T_p) / (1.0 - 2.0 * T_p)
    x[m] = direction * (-stride / 2.0 + stride * (tm - np.sin(2 * np.pi * tm) / (2 * np.pi)))
    x[t >= 1.0 - T_p] = direction * (stride / 2.0)
    z = np.where(
        t < 0.5,
        2.0 * SWING_HEIGHT * (t - np.sin(4 * np.pi * t) / (4 * np.pi)),
        2.0 * SWING_HEIGHT * (1.0 - t + np.sin(4 * np.pi * t) / (4 * np.pi)),
    )
    return x, z


def _stance_traj(t_norm: np.ndarray, stride: float, direction: int):
    t      = np.asarray(t_norm, dtype=float)
    L_span = stride / 2.0
    x = direction * L_span * (1.0 - 2.0 * t)
    z = -DELTA * np.sin(np.pi * t) ** 2
    return x, z


def get_leg_stride(leg_name: str, mode: str):
    if mode == MODE_FORWARD:
        return STRIDE_LENGTH, +1
    if mode == MODE_BACKWARD:
        return STRIDE_LENGTH, -1
    if mode == MODE_CLOCKWISE:
        return (STRIDE_LENGTH, +1) if leg_name in PHYS_RIGHT_LEGS else (TURN_INNER_STRIDE, -1)
    if mode == MODE_ANTICLOCKWISE:
        return (STRIDE_LENGTH, +1) if leg_name in PHYS_LEFT_LEGS else (TURN_INNER_STRIDE, -1)
    return STRIDE_LENGTH, +1


def leg_ik_at_time(t_sec: float, leg_name: str, mode: str):
    """Return (theta2, theta3) in radians for leg at time t_sec."""
    stride, direction = get_leg_stride(leg_name, mode)
    phase_offset = PHASE_SHIFTS[leg_name]
    t_norm    = (t_sec / CYCLE_DURATION) % 1.0
    t_shifted = (t_norm + phase_offset) % 1.0

    if t_shifted < DUTY_FACTOR:
        xi, zi = _stance_traj(np.array([t_shifted / DUTY_FACTOR]), stride, direction)
    else:
        xi, zi = _swing_traj(
            np.array([(t_shifted - DUTY_FACTOR) / (1.0 - DUTY_FACTOR)]),
            stride, direction,
        )

    foot_x = float(xi[0])
    foot_z = float(zi[0]) + NEUTRAL_FOOT_Z

    result = calc_2dof_ik(foot_x, foot_z)
    if result is None:
        result = calc_2dof_ik(NEUTRAL_FOOT_X, NEUTRAL_FOOT_Z)
        if result is None:
            return 0.0, 0.0
    return result  # (theta2, theta3)


def ik_to_urdf_deg(leg_name: str, theta2: float, theta3: float):
    """
    Convert IK angles to URDF position in degrees (pre flip/gear_ratio).
    URDF_angle = offset + sign * IK_angle  (radians) → convert to degrees.
    main.py transform_live_angles will apply flip and gear_ratio on top.
    """
    offsets = LEG_OFFSETS[leg_name]
    signs   = LEG_JOINT_SIGNS[leg_name]

    roll_rad = offsets[0]                          # always 0
    hip_rad  = offsets[1] + signs[0] * theta2
    knee_rad = offsets[2] + signs[1] * theta3

    return (
        math.degrees(roll_rad),
        math.degrees(hip_rad),
        math.degrees(knee_rad),
    )

# ─────────────────────────────────────────────────────────────────────────────
# Socket sender
# ─────────────────────────────────────────────────────────────────────────────

def send_payload(payload: dict, host: str = HOST, port: int = PORT) -> None:
    data = pickle.dumps(payload, protocol=pickle.HIGHEST_PROTOCOL)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.connect((host, port))
        sock.sendall(data)


def build_payload(t_sec: float, mode: str) -> dict:
    """Build the {fl, bl, fr, br} dict for the current time step."""
    payload = {}
    for gait_leg, socket_key in GAIT_TO_SOCKET.items():
        theta2, theta3 = leg_ik_at_time(t_sec, gait_leg, mode)
        roll_deg, hip_deg, knee_deg = ik_to_urdf_deg(gait_leg, theta2, theta3)
        payload[socket_key] = [roll_deg, hip_deg, knee_deg]
    return payload

# ─────────────────────────────────────────────────────────────────────────────
# Transition helper: smoothly move from stand to gait start
# ─────────────────────────────────────────────────────────────────────────────

def transition_to_gait_start(
    start_payload: dict,
    current_payload: dict,
    steps: int = 30,
    step_delay: float = 0.05,
) -> None:
    """
    Linearly interpolate from current_payload to start_payload before
    the gait begins, so the bot eases in without jerking.
    """
    print("  [Transition] Easing into gait start position ...")
    for i in range(1, steps + 1):
        alpha = i / steps
        interp: dict = {}
        for key in ('fl', 'bl', 'fr', 'br'):
            interp[key] = [
                (1.0 - alpha) * c + alpha * s
                for c, s in zip(current_payload[key], start_payload[key])
            ]
        try:
            send_payload(interp)
        except ConnectionRefusedError:
            print("  [Transition] Connection refused — is main.py running?")
            raise
        time.sleep(step_delay)
    print("  [Transition] Done.")

# ─────────────────────────────────────────────────────────────────────────────
# Main gait loop
# ─────────────────────────────────────────────────────────────────────────────

def run_gait(mode: str, num_cycles: int, loop: bool) -> None:
    # Stand position payload (matches main_sender.py homed angles)
    STAND_PAYLOAD = {
        'fl': [60.0, -105.0, 6.0],
        'bl': [60.0, -105.0, 6.0],
        'fr': [60.0, -105.0, 6.0],
        'br': [60.0, -105.0, 6.0],
    }

    t0_payload = build_payload(0.0, mode)

    print(f"\n{'='*60}")
    print(f"  WALK GAIT SENDER — mode={mode}")
    print(f"  Cycles: {'∞' if loop else num_cycles}  |  Steps/cycle: {NUM_STEPS}")
    print(f"  Step delay: {DELAY_PER_STEP*1000:.1f} ms  |  Socket: {HOST}:{PORT}")
    print(f"{'='*60}")

    transition_to_gait_start(t0_payload, STAND_PAYLOAD)

    cycle = 0
    try:
        while loop or cycle < num_cycles:
            cycle_start = time.time()
            print(f"  Cycle {cycle + 1}" + ("" if not loop else " (∞)"))

            for step in range(NUM_STEPS):
                t_sec   = step * DELAY_PER_STEP
                payload = build_payload(t_sec, mode)

                try:
                    send_payload(payload)
                except ConnectionRefusedError:
                    print("\n  [ERROR] main.py socket not available. Is main.py running?")
                    return
                except OSError as exc:
                    print(f"\n  [ERROR] Socket error: {exc}")
                    return

                elapsed    = time.time() - cycle_start - step * DELAY_PER_STEP
                sleep_time = DELAY_PER_STEP - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

            cycle += 1

    except KeyboardInterrupt:
        print("\n  [Interrupted] Returning to stand ...")

    print("  Sending stand payload ...")
    try:
        send_payload(STAND_PAYLOAD)
    except OSError:
        pass
    print("  Done.")


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Stream walk-gait angles to the real quadruped bot via main.py socket."
    )
    parser.add_argument(
        "--mode", "-m",
        choices=["f", "b", "r", "l"],
        default="f",
        help="f=forward  b=backward  r=clockwise  l=anticlockwise  (default: f)",
    )
    parser.add_argument(
        "--cycles", "-c",
        type=int,
        default=0,
        help="Number of gait cycles. 0 = loop forever (default: 0).",
    )
    parser.add_argument(
        "--host",
        default=HOST,
        help=f"Socket host (default: {HOST})",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=PORT,
        help=f"Socket port (default: {PORT})",
    )
    args = parser.parse_args()

    mode_map = {
        "f": MODE_FORWARD,
        "b": MODE_BACKWARD,
        "r": MODE_CLOCKWISE,
        "l": MODE_ANTICLOCKWISE,
    }

    run_gait(
        mode=mode_map[args.mode],
        num_cycles=args.cycles,
        loop=(args.cycles == 0),
    )


if __name__ == "__main__":
    main()
