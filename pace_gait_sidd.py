#!/usr/bin/env python3
"""
pace_gait_node_sid.py
════════════════════════════════════════════════════════════════════════════════
BYTE-01 Pace Gait — Interactive Terminal Controller
(Phase-based cycloidal gait, derived from trot_gait_node_sid.py)
IK is handled externally — this file sends raw XYZ foot targets over socket.

Startup: sends SITTING → smoothly transitions to STANDING

Commands:
  f          → pace forward  1 full cycle
  ff         → pace forward  continuously  (press y to stop)
  b          → pace backward 1 full cycle
  bb         → pace backward continuously  (press y to stop)
  r          → strafe right  1 full cycle
  rr         → strafe right  continuously  (press y to stop)
  l          → strafe left   1 full cycle
  ll         → strafe left   continuously  (press y to stop)
  s          → sit   @ 120 deg/s
  stand      → stand @ 120 deg/s
  q / quit   → exit

Coordinate convention:
  x = lateral shift
  y = height          (up = positive, smaller Y = foot higher on real hardware)
  z = fore-aft reach  (forward = positive)

Foot trajectory equations
──────────────────────────────────────────────────────────────────────────────
Swing phase  (local phase s ∈ [0, 1]):
  z(s) = z0 - S/2 + S * [s - (1/2π)*sin(2πs)]     ← cycloidal (centered)
  y(s) = y0 + H * [1 - cos(2πs)]                   ← lifts to y0+2H at s=0.5

Stance phase (local phase s ∈ [0, 1]):
  z(s) = z0 + S/2 - S*s                            ← linear pushback
  y(s) = y0 - Hg * sin(πs)                         ← small ground push

Pace ipsilateral pairing:
  Pair A  (FL + BL) : phase offset 0.0   ← LEFT  side
  Pair B  (FR + BR) : phase offset 0.5   ← RIGHT side
  phase ∈ [0.0, 0.5) → swing
  phase ∈ [0.5, 1.0) → stance

  Effect: left legs swing while right legs stance, then swap.
  Robot rocks laterally — less stable than trot but valid quadruped gait.
"""

import math
import pickle
import socket
import threading
import time
from mpu_leveling import MPULeveler

HOST = "10.196.200.34"
PORT = 50000

# ─────────────────────────────────────────────────────────────────────────────
# POSES
# ─────────────────────────────────────────────────────────────────────────────
SITTING_XYZ  = [-9.094, 11.0, 10.0]
STANDING_XYZ = [-9.094, 25.0, 10.0]

# ─────────────────────────────────────────────────────────────────────────────
# GAIT PARAMETERS
# ─────────────────────────────────────────────────────────────────────────────
STRIDE_LENGTH  = 3.0     # cm  — S,  total fore-aft foot travel per cycle
STRIDE_X       = 4.0     # cm  — lateral stride for strafe
LIFT_HEIGHT    = 1.0     # cm  — H,  half peak lift (actual peak = y0 + 2H)
GROUND_PUSH    = 0.2     # cm  — Hg, downward stance push
GAIT_FREQUENCY = 2.5      # Hz  — full cycle rate
UPDATE_HZ      = 100.0   # Hz  — send rate

GAIT_SPEED_DEG_PER_S       = 800.0   # deg/s — used during gait frames
TRANSITION_SPEED_DEG_PER_S = 120.0   # deg/s — used during sit / stand

Y_LIFT_SIGN = -1   # -1 because on real hardware smaller Y = foot higher

# ─────────────────────────────────────────────────────────────────────────────
# PHASE OFFSETS  ← THE ONLY CHANGE FROM TROT
#   Pace pairs same-side (ipsilateral) legs:
#     Left  pair: FL + BL → offset 0.0
#     Right pair: FR + BR → offset 0.5
# ─────────────────────────────────────────────────────────────────────────────
PHASE_OFFSET = {
    "fl": 0.0,   # ─┐ Pair A — LEFT side
    "bl": 0.0,   # ─┘
    "fr": 0.5,   # ─┐ Pair B — RIGHT side
    "br": 0.5,   # ─┘
}

ALL_LEGS = ["fl", "fr", "bl", "br"]

# ─────────────────────────────────────────────────────────────────────────────
# RUNTIME STATE
# ─────────────────────────────────────────────────────────────────────────────
current_pos = {leg: list(SITTING_XYZ) for leg in ALL_LEGS}
_stop_flag  = threading.Event()
leveler = MPULeveler()

# ─────────────────────────────────────────────────────────────────────────────
# COMMUNICATION
# ─────────────────────────────────────────────────────────────────────────────
def send_payload(payload: dict, speed: float = GAIT_SPEED_DEG_PER_S):
    payload["speed"] = speed
    data = pickle.dumps(payload)
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(0.5)
            s.connect((HOST, PORT))
            s.sendall(data)
    except Exception as e:
        print(f"\r[Socket Error] {e}", flush=True)


def send_all_to(xyz: list, speed: float = TRANSITION_SPEED_DEG_PER_S):
    payload = {leg: list(xyz) for leg in ALL_LEGS}
    send_payload(payload, speed=speed)
    for leg in ALL_LEGS:
        current_pos[leg] = list(xyz)

# ─────────────────────────────────────────────────────────────────────────────
# SMOOTH TRANSITION  (sit / stand with smoothstep easing)
# ─────────────────────────────────────────────────────────────────────────────
def smooth_transition(target_xyz: list, duration: float = 1.2):
    start    = {leg: list(current_pos[leg]) for leg in ALL_LEGS}
    n_frames = max(1, int(duration * UPDATE_HZ))
    for frame in range(n_frames + 1):
        t    = frame / n_frames
        ease = t * t * (3.0 - 2.0 * t)          # smoothstep
        payload = {
            leg: [
                start[leg][0] + (target_xyz[0] - start[leg][0]) * ease,
                start[leg][1] + (target_xyz[1] - start[leg][1]) * ease,
                start[leg][2] + (target_xyz[2] - start[leg][2]) * ease,
            ]
            for leg in ALL_LEGS
        }
        send_payload(payload, speed=TRANSITION_SPEED_DEG_PER_S)
        time.sleep(1.0 / UPDATE_HZ)
    for leg in ALL_LEGS:
        current_pos[leg] = list(target_xyz)

# ─────────────────────────────────────────────────────────────────────────────
# PHASE-BASED FOOT POSITION  (identical logic to trot — PHASE_OFFSET does the work)
# ─────────────────────────────────────────────────────────────────────────────
def _foot_pos_phase(leg: str, phase: float, axis: str, direction: int) -> list:
    """
    Compute [x, y, z] foot target for a leg at a given global phase.

    axis      : 'z' → forward/backward  |  'x' → left/right strafe
    direction : +1 or -1
    """
    S  = STRIDE_LENGTH if axis == 'z' else STRIDE_X
    S *= direction
    H  = LIFT_HEIGHT
    Hg = GROUND_PUSH
    x0 = STANDING_XYZ[0]
    z0 = STANDING_XYZ[2]
    y0 = STANDING_XYZ[1]

    phi = (phase + PHASE_OFFSET[leg]) % 1.0

    if phi < 0.5:
        # ── Swing ──────────────────────────────────────────────────────────
        s  = phi / 0.5
        dz = -S / 2 + S * (s - math.sin(2 * math.pi * s) / (2 * math.pi))
        dy = H * (1 - math.cos(2 * math.pi * s))
        z  = z0 + dz
        y  = y0 + Y_LIFT_SIGN * dy
    else:
        # ── Stance ─────────────────────────────────────────────────────────
        s  = (phi - 0.5) / 0.5
        dz = S / 2 - S * s
        dy = Hg * math.sin(math.pi * s)
        z  = z0 + dz
        y  = y0 + Y_LIFT_SIGN * dy

    if axis == 'x':
        x_sign = -1 if leg in ("fl", "bl") else 1
        x = x0 + x_sign * (z - z0)
        z = z0
    else:
        x = x0

    return [x, y, z]

# ─────────────────────────────────────────────────────────────────────────────
# CORE GAIT RUNNER
# ─────────────────────────────────────────────────────────────────────────────
def _run_gait_cycle(axis: str, direction: int, cycles: int = 1):
    """
    Execute gait cycles using continuous phase advancement.
    cycles=-1 runs until _stop_flag is set.
    Always completes a full cycle before stopping — all feet land cleanly.
    """
    dt          = 1.0 / UPDATE_HZ
    phase_step  = GAIT_FREQUENCY / UPDATE_HZ
    phase       = 0.0
    cycle_count = 0

    while True:
        while phase < 1.0:
            payload = {}
            for leg in ALL_LEGS:
                x, y, z = _foot_pos_phase(leg, phase, axis, direction)
                payload[leg] = [x, y, z]
            send_payload(payload, speed=GAIT_SPEED_DEG_PER_S)
            time.sleep(dt)
            phase += phase_step

        phase = 0.0
        cycle_count += 1

        for leg in ALL_LEGS:
            current_pos[leg] = list(STANDING_XYZ)

        if cycles != -1 and cycle_count >= cycles:
            break
        if cycles == -1 and _stop_flag.is_set():
            break


def _stop_listener():
    try:
        while True:
            if input().strip().lower() == 'y':
                _stop_flag.set()
                break
    except (EOFError, KeyboardInterrupt):
        _stop_flag.set()

# ─────────────────────────────────────────────────────────────────────────────
# STEP RUNNERS
# ─────────────────────────────────────────────────────────────────────────────
def run_one_cycle(axis: str, direction: int, label: str):
    print(f"● {label} — 1 cycle")
    _run_gait_cycle(axis, direction, cycles=1)
    print("  Done.")


def run_continuous(axis: str, direction: int, label: str):
    _stop_flag.clear()
    print(f"● {label} — continuous  (press  y  to stop)")
    listener = threading.Thread(target=_stop_listener, daemon=True)
    listener.start()
    _run_gait_cycle(axis, direction, cycles=-1)
    print(f"● {label} stopped — all feet on ground.")

# ─────────────────────────────────────────────────────────────────────────────
# MAIN LOOP
# ─────────────────────────────────────────────────────────────────────────────
def main():
    print("╔══════════════════════════════════════════════════╗")
    print("║     BYTE-01 Pace Gait Controller                 ║")
    print("╠══════════════════════════════════════════════════╣")
    print(f"║  STRIDE_LENGTH (S)     = {STRIDE_LENGTH} cm                  ║")
    print(f"║  LIFT_HEIGHT   (H)     = {LIFT_HEIGHT} cm  (peak = {2*LIFT_HEIGHT} cm)  ║")
    print(f"║  GROUND_PUSH   (Hg)    = {GROUND_PUSH} cm                  ║")
    print(f"║  STRIDE_X (strafe)     = {STRIDE_X} cm                  ║")
    print(f"║  GAIT_FREQUENCY        = {GAIT_FREQUENCY} Hz                  ║")
    print(f"║  UPDATE_HZ             = {UPDATE_HZ:.0f} Hz                 ║")
    print(f"║  GAIT_SPEED            = {GAIT_SPEED_DEG_PER_S:.0f} deg/s             ║")
    print(f"║  TRANSITION_SPEED      = {TRANSITION_SPEED_DEG_PER_S:.0f} deg/s             ║")
    print(f"║  Cycle duration        = {1.0/GAIT_FREQUENCY:.2f} s               ║")
    print(f"║  Pairing               = PACE (FL+BL | FR+BR)   ║")
    print("╠══════════════════════════════════════════════════╣")
    print("║  f   → forward  1 cycle     ff  → forward  cont. ║")
    print("║  b   → backward 1 cycle     bb  → backward cont. ║")
    print("║  r   → right    1 cycle     rr  → right    cont. ║")
    print("║  l   → left     1 cycle     ll  → left     cont. ║")
    print("║  y   → stop continuous gait                      ║")
    print("║  s   → sit  @ 120 deg/s                          ║")
    print("║  stand → stand @ 120 deg/s                       ║")
    print("║  q / quit → exit                                 ║")
    print("╚══════════════════════════════════════════════════╝\n")

    print("● Sending SITTING pose...")
    send_all_to(SITTING_XYZ)
    leveler.capture_reference()
    time.sleep(1.5)

    print("● Transitioning to STANDING pose...")
    smooth_transition(STANDING_XYZ, duration=1.5)
    leveler.level_after_stand()
    print("● Standing — ready for commands.\n")

    while True:
        try:
            cmd = input("> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not cmd:
            continue

        if cmd in ("q", "quit"):
            break
        elif cmd == "s":
            print("● Sitting... (120 deg/s)")
            smooth_transition(SITTING_XYZ, duration=1.2)
            print("● Done — sitting pose reached.")
        elif cmd == "stand":
            print("● Standing... (120 deg/s)")
            smooth_transition(STANDING_XYZ, duration=1.2)
            leveler.level_after_stand()
            for leg in ALL_LEGS:
                current_pos[leg] = list(STANDING_XYZ)
            print("● Done — standing pose reached.")
        elif cmd == "f":
            run_one_cycle(axis='z', direction=-1, label="FORWARD")
        elif cmd == "ff":
            run_continuous(axis='z', direction=-1, label="FORWARD")
        elif cmd == "b":
            run_one_cycle(axis='z', direction=+1, label="BACKWARD")
        elif cmd == "bb":
            run_continuous(axis='z', direction=+1, label="BACKWARD")
        elif cmd == "r":
            run_one_cycle(axis='x', direction=+1, label="STRAFE RIGHT")
        elif cmd == "rr":
            run_continuous(axis='x', direction=+1, label="STRAFE RIGHT")
        elif cmd == "l":
            run_one_cycle(axis='x', direction=-1, label="STRAFE LEFT")
        elif cmd == "ll":
            run_continuous(axis='x', direction=-1, label="STRAFE LEFT")
        else:
            print("  Unknown command.")
            print("  Single cycle : f | b | r | l")
            print("  Continuous   : ff | bb | rr | ll  (press y to stop)")
            print("  Pose         : s | stand")
            print("  Exit         : q")

    print("● Returning to sitting pose before exit... (120 deg/s)")
    smooth_transition(SITTING_XYZ, duration=1.2)
    print("● Bye!")


if __name__ == "__main__":
    main()