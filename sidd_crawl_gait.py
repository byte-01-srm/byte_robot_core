#!/usr/bin/env python3
"""
crawl_gait.py
════════════════════════════════════════════════════════════════════════════════
BYTE-01  Crawl (Static Walk) Gait — Interactive Terminal Controller
Converted from trot_gait_node_sid.py — same socket, same IK pipeline.

Key difference from trot
─────────────────────────
  Trot  : 2 legs swing simultaneously (diagonal pairs), swing duty = 50%
           → fast but only 2-point support during swing → unstable on stairs

  Crawl : 1 leg swings at a time, other 3 always in stance, swing duty = 25%
           → slower but always 3-point (triangle) support → safe on stairs

Leg sequence (LF → RH → RF → LH)
───────────────────────────────────
  Cycle [0.00 → 1.00]
    0.00 → 0.25 : FL swings   |  FR, BL, BR in stance
    0.25 → 0.50 : BR swings   |  FL, FR, BL in stance
    0.50 → 0.75 : FR swings   |  FL, BL, BR in stance
    0.75 → 1.00 : BL swings   |  FL, FR, BR in stance

  This order keeps the CoM always inside the support triangle.

Commands
─────────
  f          → crawl forward  1 full cycle
  ff         → crawl forward  continuously  (press y to stop)
  b          → crawl backward 1 full cycle
  bb         → crawl backward continuously
  r          → strafe right   1 full cycle
  rr         → strafe right   continuously
  l          → strafe left    1 full cycle
  ll         → strafe left    continuously
  s          → sit   @ 120 deg/s
  stand      → stand @ 120 deg/s
  q / quit   → exit

Coordinate convention  (unchanged from trot)
──────────────────────────────────────────────
  x = lateral shift
  y = height  (smaller Y = foot higher on real hardware)
  z = fore-aft reach  (forward = positive)

Foot trajectory equations
──────────────────────────────────────────────────────────────────────────────
Swing phase  (local s ∈ [0, 1], runs over SWING_DUTY fraction of cycle):
  z(s) = z0 - S/2 + S * [s - (1/2π)*sin(2πs)]     ← same cycloidal as trot
  y(s) = y0 + Y_LIFT_SIGN * H * [1 - cos(2πs)]    ← peaks at y0 + Y_LIFT_SIGN*2H

Stance phase (local s ∈ [0, 1], runs over (1-SWING_DUTY) fraction of cycle):
  z(s) = z0 + S/2 - S * s                          ← linear pushback
  y(s) = y0 + Y_LIFT_SIGN * Hg * sin(π*s)          ← small ground push

Phase offsets per leg:
  FL = 0.00   BR = 0.25   FR = 0.50   BL = 0.75
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
# POSES  (unchanged from trot)
# ─────────────────────────────────────────────────────────────────────────────
SITTING_XYZ  = [-9.65, 11.0, 10.0]
STANDING_XYZ = [-9.65, 25.0, 10.0]

# ─────────────────────────────────────────────────────────────────────────────
# GAIT PARAMETERS
# ─────────────────────────────────────────────────────────────────────────────
STRIDE_LENGTH   = 5.0    # cm  — S,  total fore-aft foot travel per cycle
STRIDE_X        = 3.0    # cm  — lateral stride for strafe
LIFT_HEIGHT     = 5.0    # cm  — H,  half peak lift  (actual peak = y0 + 2H)
GROUND_PUSH     = 0.2    # cm  — Hg, small downward push during stance
CRAWL_FREQUENCY = 2.0    # Hz  — full cycle rate
#                                 at 0.5 Hz: each leg step takes 0.5 s
#                                 at 0.25 Hz: each step takes 1.0 s (use for stairs)
UPDATE_HZ       = 100.0  # Hz  — send rate

SWING_DUTY = 0.25   # fraction of cycle each leg spends in swing (1 leg at a time)
                    # STANCE_DUTY = 1 - SWING_DUTY = 0.75

GAIT_SPEED_DEG_PER_S       = 400.0  # deg/s — during gait frames
TRANSITION_SPEED_DEG_PER_S = 120.0  # deg/s — sit / stand transitions

Y_LIFT_SIGN = -1   # -1 because smaller Y = foot higher on real hardware

# ─────────────────────────────────────────────────────────────────────────────
# PHASE OFFSETS  (one leg swings at a time — LF → RH → RF → LH)
# ─────────────────────────────────────────────────────────────────────────────
#   FL swings at phase [0.00, 0.25)   → offset = 0.00
#   BR swings at phase [0.25, 0.50)   → offset = 0.25
#   FR swings at phase [0.50, 0.75)   → offset = 0.50
#   BL swings at phase [0.75, 1.00)   → offset = 0.75
#
#   Why this order?
#   When FL swings, the support triangle is FR-BL-BR — CoM stays inside.
#   When BR swings, the support triangle is FL-FR-BL — CoM stays inside.
#   Alternating diagonals (like trot) but one at a time, not both together.
# ─────────────────────────────────────────────────────────────────────────────
PHASE_OFFSET = {
    "fl": 0.00,
    "br": 0.25,
    "bl": 0.50,
    "fr": 0.75,
}

ALL_LEGS = ["fl", "fr", "bl", "br"]

# ─────────────────────────────────────────────────────────────────────────────
# RUNTIME STATE
# ─────────────────────────────────────────────────────────────────────────────
current_pos = {leg: list(SITTING_XYZ) for leg in ALL_LEGS}
_stop_flag  = threading.Event()
leveler = MPULeveler()

# ─────────────────────────────────────────────────────────────────────────────
# COMMUNICATION  (identical to trot file)
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
# SMOOTH TRANSITION  (identical to trot file)
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
# PHASE-BASED FOOT POSITION  (core change from trot)
# ─────────────────────────────────────────────────────────────────────────────
def _foot_pos_phase(leg: str, phase: float, axis: str, direction: int) -> list:
    """
    Compute [x, y, z] foot target for a leg at a given global phase.

    Crawl difference from trot:
      - SWING_DUTY = 0.25  (swing occupies first 25% of local phase)
      - STANCE_DUTY = 0.75 (stance occupies remaining 75%)
      - local s is always normalised [0, 1] within whichever phase the leg is in

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

    if phi < SWING_DUTY:
        # ── Swing phase ────────────────────────────────────────────────────
        # Normalise phi into [0, 1] over the swing window
        s  = phi / SWING_DUTY

        # Cycloidal trajectory — identical formula to trot
        dz = -S / 2 + S * (s - math.sin(2 * math.pi * s) / (2 * math.pi))
        dy = H * (1 - math.cos(2 * math.pi * s))

        z  = z0 + dz
        y  = y0 + Y_LIFT_SIGN * dy

    else:
        # ── Stance phase ───────────────────────────────────────────────────
        # Normalise phi into [0, 1] over the stance window
        s  = (phi - SWING_DUTY) / (1.0 - SWING_DUTY)

        # Linear pushback — same formula as trot, just normalised over 75%
        # The foot travels the full stride S backward during stance regardless
        # of stance duty, so body advance per step is consistent.
        dz = S / 2 - S * s
        dy = Hg * math.sin(math.pi * s)

        z  = z0 + dz
        y  = y0 + Y_LIFT_SIGN * dy

    if axis == 'x':
        # Strafe: transfer the z displacement onto x, zero z
        x_sign = -1 if leg in ("fl", "bl") else 1
        x = x0 + x_sign * (z - z0)
        z = z0
    else:
        x = x0

    return [x, y, z]


# ─────────────────────────────────────────────────────────────────────────────
# CORE GAIT RUNNER  (identical structure to trot)
# ─────────────────────────────────────────────────────────────────────────────
def _run_gait_cycle(axis: str, direction: int, cycles: int = 1):
    """
    Execute crawl cycles using continuous phase advancement.
    cycles=-1 runs until _stop_flag is set.
    Always completes a full cycle before stopping — all feet land cleanly.
    """
    dt          = 1.0 / UPDATE_HZ
    phase_step  = CRAWL_FREQUENCY / UPDATE_HZ
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
# STEP RUNNERS  (identical to trot)
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
    step_time_s = SWING_DUTY / CRAWL_FREQUENCY
    cycle_time_s = 1.0 / CRAWL_FREQUENCY

    print("╔══════════════════════════════════════════════════════╗")
    print("║     BYTE-01 Crawl Gait Controller                    ║")
    print("╠══════════════════════════════════════════════════════╣")
    print(f"║  STRIDE_LENGTH (S)     = {STRIDE_LENGTH} cm                    ║")
    print(f"║  LIFT_HEIGHT   (H)     = {LIFT_HEIGHT} cm  (peak = {2*LIFT_HEIGHT} cm)    ║")
    print(f"║  GROUND_PUSH   (Hg)    = {GROUND_PUSH} cm                    ║")
    print(f"║  STRIDE_X (strafe)     = {STRIDE_X} cm                    ║")
    print(f"║  CRAWL_FREQUENCY       = {CRAWL_FREQUENCY} Hz                    ║")
    print(f"║  UPDATE_HZ             = {UPDATE_HZ:.0f} Hz                   ║")
    print(f"║  SWING_DUTY            = {SWING_DUTY:.0%} per leg               ║")
    print(f"║  Per-leg step time     = {step_time_s:.2f} s                   ║")
    print(f"║  Full cycle time       = {cycle_time_s:.2f} s  (4 steps)         ║")
    print(f"║  Leg sequence          = FL → BR → FR → BL          ║")
    print("╠══════════════════════════════════════════════════════╣")
    print("║  f   → forward  1 cycle     ff  → forward  cont.     ║")
    print("║  b   → backward 1 cycle     bb  → backward cont.     ║")
    print("║  r   → right    1 cycle     rr  → right    cont.     ║")
    print("║  l   → left     1 cycle     ll  → left     cont.     ║")
    print("║  y   → stop continuous gait                          ║")
    print("║  s   → sit  @ 120 deg/s                              ║")
    print("║  stand → stand @ 120 deg/s                           ║")
    print("║  q / quit → exit                                     ║")
    print("╚══════════════════════════════════════════════════════╝\n")

    print("● Sending SITTING pose...")
    send_all_to(SITTING_XYZ)
    leveler.capture_reference() 
    time.sleep(1.5)

    print("● Transitioning to STANDING pose...")
    smooth_transition(STANDING_XYZ, duration=1.5)
    leveler.capture_reference()
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