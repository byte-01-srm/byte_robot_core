#!/usr/bin/env python3
"""
BYTE-01 Trot Gait — Interactive Terminal Controller
====================================================
Startup:  sends SITTING → smoothly transitions to STANDING

Commands:
  f / ff / fff ...   →  trot forward  N steps
  b / bb / bbb ...   →  trot backward N steps
  r / rr / rrr ...   →  strafe right  N steps
  l / ll / lll ...   →  strafe left   N steps
  s                  →  sit   @ 120 deg/s
  stand              →  stand @ 120 deg/s
  q / quit           →  exit

Gait model: trot — 2 diagonal legs swing simultaneously, 2 in stance
  Step 1 : FR + BL swing  |  FL + BR stance
  Step 2 : FL + BR swing  |  FR + BL stance  → full cycle complete

Axes:
  Forward / backward  → Z axis  (STRIDE_Z)
  Left    / right     → X axis  (STRIDE_X  — separate, tunable)
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
SITTING_XYZ  = [-9.094, 11.0, 3.0]
STANDING_XYZ = [-9.094, 25.0, 3.0]

# ─────────────────────────────────────────────────────────────────────────────
#  GAIT PARAMETERS  ← tune these
# ─────────────────────────────────────────────────────────────────────────────
STEP_HEIGHT_Y              = 4.0    # cm  — swing foot lift height (Y-axis)
STRIDE_Z                   = 4.0    # cm  — Z sweep per step  (forward / backward)
STRIDE_X                   = 4.0    # cm  — X sweep per step  (left / right strafe)
STEP_TIME                  = 0.25  # s   — duration of one trot step
SEND_HZ                    = 80.0   # Hz  — payload send rate

GAIT_SPEED_DEG_PER_S       = 500.0  # deg/s — used during all motion frames
TRANSITION_SPEED_DEG_PER_S = 120.0  # deg/s — used during sit / stand

# -1 if smaller Y = foot is higher (foot lifts when Y decreases)
# +1 if larger  Y = foot is higher
Y_LIFT_SIGN = -1

# ─────────────────────────────────────────────────────────────────────────────
#  TROT CONSTANTS
# ─────────────────────────────────────────────────────────────────────────────
# Diagonal pairs swung simultaneously
TROT_PAIRS = [
    ("fr", "bl"),   # step 1
    ("fl", "br"),   # step 2  → full cycle
]
NUM_STEPS  = len(TROT_PAIRS)
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
#  SMOOTH TRANSITION  (all legs together, slow speed)
# ─────────────────────────────────────────────────────────────────────────────
def smooth_transition(target_xyz: list, duration: float = 1.2):
    """
    Smoothly interpolate all legs to target_xyz with smoothstep easing.
    Always uses TRANSITION_SPEED_DEG_PER_S (120 deg/s).
    """
    start    = {leg: list(current_pos[leg]) for leg in ALL_LEGS}
    n_frames = max(1, int(duration * SEND_HZ))
    for frame in range(n_frames + 1):
        t    = frame / n_frames
        ease = t * t * (3.0 - 2.0 * t)
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
#  CORE TROT STEP
# ─────────────────────────────────────────────────────────────────────────────
def animate_trot_step(step_idx: int, axis: str, direction: int):
    """
    Execute one trot step in any direction.

    axis      : 'z' → forward/backward motion  (uses STRIDE_Z)
                'x' → left/right strafe motion  (uses STRIDE_X)

    direction : sign that controls which way the swing pair moves
                forward  → axis='z', direction=-1  (swing feet move in -Z)
                backward → axis='z', direction=+1  (swing feet move in +Z)
                right    → axis='x', direction=+1  (swing feet move in +X)
                left     → axis='x', direction=-1  (swing feet move in -X)

    Z math (2-phase trot):
        swing_d  =  stride * direction
        stance_d = -stride * direction
    After 2 steps (1 full cycle) all feet return exactly to start. ✓
    """
    global current_step_index

    swing_pair  = list(TROT_PAIRS[step_idx % NUM_STEPS])
    stance_pair = [l for l in ALL_LEGS if l not in swing_pair]
    n_frames    = max(1, int(STEP_TIME * SEND_HZ))

    stride   = STRIDE_Z if axis == 'z' else STRIDE_X
    swing_d  =  stride * direction   # swing feet go this way
    stance_d = -stride * direction   # stance feet creep the other way

    start_x = {leg: current_pos[leg][0] for leg in ALL_LEGS}
    start_z = {leg: current_pos[leg][2] for leg in ALL_LEGS}

    for frame in range(n_frames + 1):
        t = frame / n_frames   # 0 → 1

        payload = {}
        for leg in ALL_LEGS:
            x = start_x[leg]
            z = start_z[leg]

            if leg in swing_pair:
                if axis == 'z':
                    z = start_z[leg] + swing_d * t
                else:
                    x = start_x[leg] + swing_d * t
                # Y-lift arc for swing legs (same for all directions)
                y = STANDING_Y + Y_LIFT_SIGN * STEP_HEIGHT_Y * math.sin(math.pi * t)
            else:
                if axis == 'z':
                    z = start_z[leg] + stance_d * t
                else:
                    x = start_x[leg] + stance_d * t
                y = STANDING_Y

            payload[leg] = [x, y, z]

        send_payload(payload, speed=GAIT_SPEED_DEG_PER_S)
        time.sleep(1.0 / SEND_HZ)

    # ── Commit final positions ─────────────────────────────────────────────
    for leg in swing_pair:
        if axis == 'z':
            current_pos[leg][2] = start_z[leg] + swing_d
        else:
            current_pos[leg][0] = start_x[leg] + swing_d
        current_pos[leg][1] = STANDING_Y

    for leg in stance_pair:
        if axis == 'z':
            current_pos[leg][2] = start_z[leg] + stance_d
        else:
            current_pos[leg][0] = start_x[leg] + stance_d
        current_pos[leg][1] = STANDING_Y

    current_step_index += 1

# ─────────────────────────────────────────────────────────────────────────────
#  HELPERS
# ─────────────────────────────────────────────────────────────────────────────
def print_status(label: str):
    step_in_cycle = current_step_index % NUM_STEPS
    next_pair     = TROT_PAIRS[step_in_cycle]
    print(f"  [{label} | step {current_step_index} | cycle {current_step_index // NUM_STEPS} "
          f"| next swing: {next_pair[0].upper()} + {next_pair[1].upper()}]")

def run_steps(n: int, axis: str, direction: int, label: str):
    """Execute n trot steps and print status."""
    print(f"● {label} — {n} step{'s' if n > 1 else ''}  ({GAIT_SPEED_DEG_PER_S:.0f} deg/s)")
    for _ in range(n):
        animate_trot_step(current_step_index, axis, direction)
    print_status(label)

# ─────────────────────────────────────────────────────────────────────────────
#  MAIN LOOP
# ─────────────────────────────────────────────────────────────────────────────
def main():
    global current_step_index

    print("╔══════════════════════════════════════════════════╗")
    print("║        BYTE-01  Trot Gait Controller             ║")
    print("╠══════════════════════════════════════════════════╣")
    print(f"║  STEP_HEIGHT_Y              = {STEP_HEIGHT_Y} cm            ║")
    print(f"║  STRIDE_Z  (fwd/back)       = {STRIDE_Z} cm            ║")
    print(f"║  STRIDE_X  (left/right)     = {STRIDE_X} cm            ║")
    print(f"║  STEP_TIME                  = {STEP_TIME} s           ║")
    print(f"║  GAIT_SPEED_DEG_PER_S       = {GAIT_SPEED_DEG_PER_S:.0f} deg/s (gait) ║")
    print(f"║  TRANSITION_SPEED_DEG_PER_S = {TRANSITION_SPEED_DEG_PER_S:.0f} deg/s (pose) ║")
    print(f"║  Full cycle                 = {STEP_TIME * NUM_STEPS:.2f} s  (2 steps)  ║")
    print("╠══════════════════════════════════════════════════╣")
    print("║  f / ff / fff ...  →  forward  N steps           ║")
    print("║  b / bb / bbb ...  →  backward N steps           ║")
    print("║  r / rr / rrr ...  →  strafe right N steps       ║")
    print("║  l / ll / lll ...  →  strafe left  N steps       ║")
    print("║  s                 →  sit   @ 120 deg/s           ║")
    print("║  stand             →  stand @ 120 deg/s           ║")
    print("║  q / quit          →  exit                        ║")
    print("╚══════════════════════════════════════════════════╝\n")

    # ── Startup sequence ──────────────────────────────────────────────────────
    print("● Sending SITTING pose...")
    send_all_to(SITTING_XYZ)
    time.sleep(1.5)

    print("● Transitioning to STANDING pose...")
    smooth_transition(STANDING_XYZ, duration=1.5)
    print("● Standing — ready for commands.\n")

    # ── Command loop ──────────────────────────────────────────────────────────
    while True:
        try:
            cmd = input("> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not cmd:
            continue

        first = cmd[0]
        all_same = all(c == first for c in cmd)
        n = len(cmd)

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

        # ── Forward ───────────────────────────────────────────────────────────
        elif all_same and first == 'f':
            run_steps(n, axis='z', direction=-1, label="FORWARD")

        # ── Backward ──────────────────────────────────────────────────────────
        elif all_same and first == 'b':
            run_steps(n, axis='z', direction=+1, label="BACKWARD")

        # ── Right ─────────────────────────────────────────────────────────────
        elif all_same and first == 'r':
            run_steps(n, axis='x', direction=+1, label="STRAFE RIGHT")

        # ── Left ──────────────────────────────────────────────────────────────
        elif all_same and first == 'l':
            run_steps(n, axis='x', direction=-1, label="STRAFE LEFT")

        # ── Unknown ───────────────────────────────────────────────────────────
        else:
            print("  Unknown command. Try: f/ff... | b/bb... | r/rr... | l/ll... | s | stand | q")

    # ── Exit ──────────────────────────────────────────────────────────────────
    print("● Returning to sitting pose before exit...  (120 deg/s)")
    smooth_transition(SITTING_XYZ, duration=1.2)
    print("● Bye!")

if __name__ == "__main__":
    main()
