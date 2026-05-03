#!/usr/bin/env python3
"""
BYTE-01 Trot Gait — Interactive Terminal Controller
====================================================
Startup:  sends SITTING → smoothly transitions to STANDING
Commands:
  f / ff / fff / ffff ...  →  execute N trot steps (each 'f' = 1/2 of a cycle)
  s                        →  sit (smooth transition back to sitting)
  stand                    →  stand (smooth transition to standing, reset gait)
  q / quit                 →  exit (robot sits before closing)

Gait model: trot — 2 diagonal legs swing simultaneously, 2 in stance
  Step 1 : FL + BR swing  |  FR + BL stance
  Step 2 : FR + BL swing  |  FL + BR stance  → full cycle complete
"""

import math
import pickle
import socket
import time

HOST = "127.0.0.1"
PORT = 50000

# ─────────────────────────────────────────────────────────────────────────────
#  POSES
# ─────────────────────────────────────────────────────────────────────────────
SITTING_XYZ  = [-9.094, 11.0, 6.0]
STANDING_XYZ = [-9.094, 25.0, 6.0]

# ─────────────────────────────────────────────────────────────────────────────
#  GAIT PARAMETERS  ← tune these
# ─────────────────────────────────────────────────────────────────────────────
STEP_HEIGHT_Y              = 5.0    # cm  — how high the swing feet lift (Y-axis)
STRIDE_Z                   = 5.0    # cm  — Z sweep per trot step
STEP_TIME                  = 0.5  # 0.25bests   — duration of one trot step
SEND_HZ                    = 40.0   # Hz  — payload send rate

# Walk speed: sent during every gait frame.
GAIT_SPEED_DEG_PER_S       = 500.0  # deg/s

# Transition speed: sent during sit / stand smooth transitions.
TRANSITION_SPEED_DEG_PER_S = 120.0  # deg/s

# Set to -1 if a SMALLER Y value means higher (foot lifts up).
# Set to +1 if a LARGER  Y value means higher.
Y_LIFT_SIGN = -1

# ─────────────────────────────────────────────────────────────────────────────
#  TROT CONSTANTS
# ─────────────────────────────────────────────────────────────────────────────
# Diagonal pairs: (FL+BR) then (FR+BL)
TROT_PAIRS = [
    ("fr", "bl"),   # step 1
    ("fl", "br"),   # step 2  → full cycle
]
NUM_STEPS  = len(TROT_PAIRS)   # = 2 steps per full cycle
ALL_LEGS   = ["fl", "fr", "bl", "br"]
STANDING_Y = STANDING_XYZ[1]

# ─────────────────────────────────────────────────────────────────────────────
#  RUNTIME STATE  (mutable)
# ─────────────────────────────────────────────────────────────────────────────
current_step_index = 0
current_pos = {
    leg: list(SITTING_XYZ) for leg in ALL_LEGS
}

# ─────────────────────────────────────────────────────────────────────────────
#  COMMUNICATION
# ─────────────────────────────────────────────────────────────────────────────
def send_payload(payload: dict, speed: float = GAIT_SPEED_DEG_PER_S):
    """Inject speed into every outgoing packet, then send."""
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
    """Teleport all legs to xyz immediately (no animation)."""
    payload = {leg: list(xyz) for leg in ALL_LEGS}
    send_payload(payload, speed=speed)
    for leg in ALL_LEGS:
        current_pos[leg] = list(xyz)

# ─────────────────────────────────────────────────────────────────────────────
#  SMOOTH TRANSITION  (all legs move together, slow speed)
# ─────────────────────────────────────────────────────────────────────────────
def smooth_transition(target_xyz: list, duration: float = 1.2):
    """
    Smoothly interpolate all legs from their current positions to target_xyz.
    Uses a smoothstep ease-in-out curve.
    Always sends TRANSITION_SPEED_DEG_PER_S (120 deg/s) — safe for sit/stand.
    """
    start    = {leg: list(current_pos[leg]) for leg in ALL_LEGS}
    n_frames = max(1, int(duration * SEND_HZ))
    for frame in range(n_frames + 1):
        t    = frame / n_frames
        ease = t * t * (3.0 - 2.0 * t)       # smoothstep
        payload = {
            leg: [
                start[leg][0] + (target_xyz[0] - start[leg][0]) * ease,
                start[leg][1] + (target_xyz[1] - start[leg][1]) * ease,
                start[leg][2] + (target_xyz[2] - start[leg][2]) * ease,
            ]
            for leg in ALL_LEGS
        }
        send_payload(payload, speed=TRANSITION_SPEED_DEG_PER_S)
        time.sleep(1.0 / SEND_HZ)
    for leg in ALL_LEGS:
        current_pos[leg] = list(target_xyz)

# ─────────────────────────────────────────────────────────────────────────────
#  SINGLE TROT STEP  (the core gait primitive)
# ─────────────────────────────────────────────────────────────────────────────
def animate_leg_step(step_idx: int):
    """
    Execute one trot step:
      • swing_pair  (2 diagonal legs) → sweep -STRIDE_Z forward with Y-lift arc
      • stance_pair (2 diagonal legs) → creep +STRIDE_Z backward

    Z math: swing_dz = -STRIDE_Z, stance_dz = +STRIDE_Z
    After 2 steps (1 full cycle) all feet return exactly to their start Z. ✓
    """
    global current_step_index

    swing_pair  = list(TROT_PAIRS[step_idx % NUM_STEPS])
    stance_pair = [l for l in ALL_LEGS if l not in swing_pair]
    n_frames    = max(1, int(STEP_TIME * SEND_HZ))

    start_z   = {leg: current_pos[leg][2] for leg in ALL_LEGS}
    swing_dz  = -STRIDE_Z   # forward
    stance_dz =  STRIDE_Z   # backward in body frame (= -swing_dz for 2-phase trot)

    for frame in range(n_frames + 1):
        t = frame / n_frames   # 0 → 1

        payload = {}
        for leg in ALL_LEGS:
            x = current_pos[leg][0]           # X never changes

            if leg in swing_pair:
                z = start_z[leg] + swing_dz * t
                # Sine arc for Y lift
                y = STANDING_Y + Y_LIFT_SIGN * STEP_HEIGHT_Y * math.sin(math.pi * t)
            else:
                z = start_z[leg] + stance_dz * t
                y = STANDING_Y

            payload[leg] = [x, y, z]

        send_payload(payload, speed=GAIT_SPEED_DEG_PER_S)
        time.sleep(1.0 / SEND_HZ)

    # Commit final positions
    for leg in swing_pair:
        current_pos[leg][2] = start_z[leg] + swing_dz
        current_pos[leg][1] = STANDING_Y
    for leg in stance_pair:
        current_pos[leg][2] = start_z[leg] + stance_dz
        current_pos[leg][1] = STANDING_Y

    current_step_index += 1

# ─────────────────────────────────────────────────────────────────────────────
#  HELPERS
# ─────────────────────────────────────────────────────────────────────────────
def print_status():
    step_in_cycle  = current_step_index % NUM_STEPS
    next_pair      = TROT_PAIRS[step_in_cycle]
    print(f"  [step {current_step_index} | cycle {current_step_index // NUM_STEPS} "
          f"| next swing: {next_pair[0].upper()} + {next_pair[1].upper()}]")

# ─────────────────────────────────────────────────────────────────────────────
#  MAIN LOOP
# ─────────────────────────────────────────────────────────────────────────────
def main():
    global current_step_index

    print("╔══════════════════════════════════════════════╗")
    print("║       BYTE-01  Trot Gait Controller          ║")
    print("╠══════════════════════════════════════════════╣")
    print(f"║  STEP_HEIGHT_Y              = {STEP_HEIGHT_Y} cm          ║")
    print(f"║  STRIDE_Z                   = {STRIDE_Z} cm          ║")
    print(f"║  STEP_TIME                  = {STEP_TIME} s         ║")
    print(f"║  GAIT_SPEED_DEG_PER_S       = {GAIT_SPEED_DEG_PER_S:.0f} deg/s (trot)║")
    print(f"║  TRANSITION_SPEED_DEG_PER_S = {TRANSITION_SPEED_DEG_PER_S:.0f} deg/s (pose)║")
    print(f"║  Full cycle                 = {STEP_TIME * NUM_STEPS:.2f} s (2 steps) ║")
    print("╠══════════════════════════════════════════════╣")
    print("║  Step 1 (f)  : FL + BR swing                 ║")
    print("║  Step 2 (ff) : FR + BL swing  ← cycle done  ║")
    print("╠══════════════════════════════════════════════╣")
    print("║  f / ff / fff ...  →  N trot steps           ║")
    print("║  s                 →  sit   @ 120 deg/s      ║")
    print("║  stand             →  stand @ 120 deg/s      ║")
    print("║  q / quit          →  exit                   ║")
    print("╚══════════════════════════════════════════════╝\n")

    # ── Startup sequence ──────────────────────────────────────────────────────
    print("● Sending SITTING pose...")
    send_all_to(SITTING_XYZ)
    time.sleep(1.5)

    print("● Transitioning to STANDING pose...")
    smooth_transition(STANDING_XYZ, duration=1.5)
    print("● Standing — ready for gait commands.\n")

    # ── Command loop ──────────────────────────────────────────────────────────
    while True:
        try:
            cmd = input("> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not cmd:
            continue

        # ── Quit ──────────────────────────────────────────────────────────────
        if cmd in ("q", "quit"):
            break

        # ── Sit ───────────────────────────────────────────────────────────────
        elif cmd == "s":
            print("● Sitting...  (120 deg/s)")
            smooth_transition(SITTING_XYZ, duration=1.2)
            current_step_index = 0
            print("● Done — sitting pose reached.")

        # ── Stand ─────────────────────────────────────────────────────────────
        elif cmd == "stand":
            print("● Standing...  (120 deg/s)")
            smooth_transition(STANDING_XYZ, duration=1.2)
            current_step_index = 0
            for leg in ALL_LEGS:
                current_pos[leg] = list(STANDING_XYZ)
            print("● Done — standing pose reached. Gait index reset.")

        # ── Trot steps ────────────────────────────────────────────────────────
        elif all(c == 'f' for c in cmd):
            n = len(cmd)
            print(f"● Executing {n} trot step{'s' if n > 1 else ''}...  ({GAIT_SPEED_DEG_PER_S:.0f} deg/s)")
            for _ in range(n):
                animate_leg_step(current_step_index)
            print_status()

        # ── Unknown ───────────────────────────────────────────────────────────
        else:
            print("  Unknown command. Try: f / ff / fff... | s | stand | q")

    # ── Exit ──────────────────────────────────────────────────────────────────
    print("● Returning to sitting pose before exit...  (120 deg/s)")
    smooth_transition(SITTING_XYZ, duration=1.2)
    print("● Bye!")

if __name__ == "__main__":
    main()