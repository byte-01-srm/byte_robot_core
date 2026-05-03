#!/usr/bin/env python3
"""
BYTE-01 Trot Gait — Interactive Terminal Controller
====================================================
Startup: sends SITTING → smoothly transitions to STANDING

Commands:
f          → trot forward  1 full cycle (2 steps)
ff         → trot forward  continuously  (press y to stop)
b          → trot backward 1 full cycle (2 steps)
bb         → trot backward continuously  (press y to stop)
r          → strafe right  1 full cycle (2 steps)
rr         → strafe right  continuously  (press y to stop)
l          → strafe left   1 full cycle (2 steps)
ll         → strafe left   continuously  (press y to stop)
s          → sit   @ 120 deg/s
stand      → stand @ 120 deg/s
q / quit   → exit

Gait model: trot — 2 diagonal legs swing simultaneously, 2 in stance
  Step 1 : FR + BL swing | FL + BR stance
  Step 2 : FL + BR swing | FR + BL stance  → full cycle complete

Axes:
  Forward / backward → Z axis  (STRIDE_Z)
  Left / right       → X axis  (STRIDE_X — separate, tunable)

Foot trajectory (swing phase):
  Cycloidal equations — smooth acceleration/deceleration, no foot slip
    z(t) = S  * { t - [1/(2π)] * sin(2π·t) }   (horizontal sweep)
    y(t) = H  * { 1 - cos(2π·t) }               (vertical lift)

  where  S = stride length  (STRIDE_Z or STRIDE_X)
         H = STEP_HEIGHT_Y  (lift parameter; peak foot height = 2·H at t=0.5)
         t ∈ [0, 1]  over the swing duration

  Stance phase: linear slide-back at constant ground height.
"""

import math
import pickle
import socket
import threading
import time

HOST = "127.0.0.1"
PORT = 50000

# ─────────────────────────────────────────────────────────────────────────────
# POSES
# ─────────────────────────────────────────────────────────────────────────────
SITTING_XYZ  = [-9.094, 11.0,  6.0]
STANDING_XYZ = [-9.094, 25.0,  6.0]

# ─────────────────────────────────────────────────────────────────────────────
# GAIT PARAMETERS  ← tune these
# ─────────────────────────────────────────────────────────────────────────────
STEP_HEIGHT_Y = 3.0    # cm  — cycloidal H parameter; actual peak lift = 2 × H
STRIDE_Z      = 3.0    # cm  — Z sweep per step (forward / backward)
STRIDE_X      = 3.0    # cm  — X sweep per step (left / right strafe)
STEP_TIME     = 0.3  # s   — duration of one trot step
SEND_HZ       = 80.0   # Hz  — payload send rate

GAIT_SPEED_DEG_PER_S       = 400.0   # deg/s — used during all motion frames
TRANSITION_SPEED_DEG_PER_S = 120.0   # deg/s — used during sit / stand

Y_LIFT_SIGN = -1   # -1 if smaller Y = foot higher; +1 if larger Y = foot higher

# ─────────────────────────────────────────────────────────────────────────────
# TROT CONSTANTS
# ─────────────────────────────────────────────────────────────────────────────
TROT_PAIRS = [
    ("fr", "bl"),   # step 1
    ("fl", "br"),   # step 2  → full cycle
]

NUM_STEPS  = len(TROT_PAIRS)
ALL_LEGS   = ["fl", "fr", "bl", "br"]
STANDING_Y = STANDING_XYZ[1]

# ─────────────────────────────────────────────────────────────────────────────
# RUNTIME STATE (mutable)
# ─────────────────────────────────────────────────────────────────────────────
current_step_index = 0
current_pos = {
    leg: list(SITTING_XYZ) for leg in ALL_LEGS
}

# Shared stop flag — set by listener thread when user presses 'y'
_stop_flag = threading.Event()

# ─────────────────────────────────────────────────────────────────────────────
# COMMUNICATION
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
# SMOOTH TRANSITION  (all legs together, slow speed)
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
# CYCLOIDAL TRAJECTORY HELPERS
# ─────────────────────────────────────────────────────────────────────────────
def cycloidal_sweep(stride: float, t: float) -> float:
    """
    Cycloidal horizontal displacement at normalised time t ∈ [0, 1].
      z(t) = S * { t - [1 / (2π)] * sin(2π·t) }
    Zero velocity at lift-off (t=0) and touch-down (t=1).
    """
    return stride * (t - (1.0 / (2.0 * math.pi)) * math.sin(2.0 * math.pi * t))


def cycloidal_lift(H: float, t: float) -> float:
    """
    Cycloidal vertical displacement at normalised time t ∈ [0, 1].
      y(t) = H * { 1 - cos(2π·t) }
    y(0) = y(1) = 0;  peak = 2·H at t = 0.5.
    """
    return H * (1.0 - math.cos(2.0 * math.pi * t))

# ─────────────────────────────────────────────────────────────────────────────
# CORE TROT STEP
# ─────────────────────────────────────────────────────────────────────────────
def animate_trot_step(step_idx: int, axis: str, direction: int):
    """
    Execute one trot step in any direction using cycloidal foot trajectories.

    axis      : 'z' → forward/backward  |  'x' → left/right strafe
    direction : +1 or -1  (controls sweep direction)
    """
    global current_step_index

    swing_pair  = list(TROT_PAIRS[step_idx % NUM_STEPS])
    stance_pair = [l for l in ALL_LEGS if l not in swing_pair]
    n_frames    = max(1, int(STEP_TIME * SEND_HZ))

    stride   = STRIDE_Z if axis == 'z' else STRIDE_X
    swing_d  =  stride * direction
    stance_d = -stride * direction

    start_x = {leg: current_pos[leg][0] for leg in ALL_LEGS}
    start_z = {leg: current_pos[leg][2] for leg in ALL_LEGS}

    for frame in range(n_frames + 1):
        t = frame / n_frames    # 0 → 1

        payload = {}
        for leg in ALL_LEGS:
            x = start_x[leg]
            z = start_z[leg]
            
            x_sign = -1 if (axis == 'x' and leg in ("fl", "bl")) else 1

            if leg in swing_pair:
                sweep = cycloidal_sweep(swing_d, t)
                lift  = cycloidal_lift(STEP_HEIGHT_Y, t)
                if axis == 'z':
                    z = start_z[leg] + sweep
                else:
                    x = start_x[leg] + x_sign * sweep
                y = STANDING_Y + Y_LIFT_SIGN * lift

            else:
                if axis == 'z':
                    z = start_z[leg] + stance_d * t
                else:
                    x = start_x[leg] + x_sign * stance_d * t
                y = STANDING_Y

            payload[leg] = [x, y, z]

        send_payload(payload, speed=GAIT_SPEED_DEG_PER_S)
        time.sleep(1.0 / SEND_HZ)

    # ── Commit final positions ─────────────────────────────────────────────
    for leg in swing_pair:
        if axis == 'z':
            current_pos[leg][2] = start_z[leg] + swing_d
        else:
            x_sign = -1 if leg in ("fl", "bl") else 1
            current_pos[leg][0] = start_x[leg] + x_sign * swing_d
        current_pos[leg][1] = STANDING_Y

    for leg in stance_pair:
        if axis == 'z':
            current_pos[leg][2] = start_z[leg] + stance_d
        else:
            x_sign = -1 if leg in ("fl", "bl") else 1
            current_pos[leg][0] = start_x[leg] + x_sign * stance_d
        current_pos[leg][1] = STANDING_Y

    current_step_index += 1

# ─────────────────────────────────────────────────────────────────────────────
# STEP RUNNERS
# ─────────────────────────────────────────────────────────────────────────────
def print_status(label: str):
    step_in_cycle = current_step_index % NUM_STEPS
    next_pair     = TROT_PAIRS[step_in_cycle]
    print(f"  [{label} | step {current_step_index} | cycle {current_step_index // NUM_STEPS} "
          f"| next swing: {next_pair[0].upper()} + {next_pair[1].upper()}]")


def run_one_cycle(axis: str, direction: int, label: str):
    """Execute exactly one full cycle (2 steps)."""
    print(f"● {label} — 1 cycle ({NUM_STEPS} steps)")
    for _ in range(NUM_STEPS):
        animate_trot_step(current_step_index, axis, direction)
    print_status(label)


def _stop_listener():
    """
    Background thread: blocks on input() waiting for 'y'.
    Sets _stop_flag so the continuous loop exits after the
    current step finishes — all feet land cleanly.
    """
    try:
        while True:
            key = input()
            if key.strip().lower() == 'y':
                _stop_flag.set()
                break
    except (EOFError, KeyboardInterrupt):
        _stop_flag.set()


def run_continuous(axis: str, direction: int, label: str):
    """
    Execute trot steps repeatedly until the user presses 'y'.
    Always finishes the current step before stopping — no mid-air freeze.
    """
    _stop_flag.clear()
    print(f"● {label} — continuous  (press  y  to stop)")

    listener = threading.Thread(target=_stop_listener, daemon=True)
    listener.start()

    cycles = 0
    while not _stop_flag.is_set():
        for _ in range(NUM_STEPS):
            if _stop_flag.is_set():
                break
            animate_trot_step(current_step_index, axis, direction)
        cycles += 1

    print(f"● {label} stopped after {cycles} cycle(s) — all feet on ground.")

# ─────────────────────────────────────────────────────────────────────────────
# MAIN LOOP
# ─────────────────────────────────────────────────────────────────────────────
def main():
    global current_step_index

    print("╔══════════════════════════════════════════════════╗")
    print("║     BYTE-01 Trot Gait Controller                 ║")
    print("╠══════════════════════════════════════════════════╣")
    print(f"║  STEP_HEIGHT_Y (H)     = {STEP_HEIGHT_Y} cm  (peak = {2*STEP_HEIGHT_Y} cm) ║")
    print(f"║  STRIDE_Z  (fwd/back)  = {STRIDE_Z} cm                  ║")
    print(f"║  STRIDE_X  (left/right)= {STRIDE_X} cm                  ║")
    print(f"║  STEP_TIME             = {STEP_TIME} s                 ║")
    print(f"║  GAIT_SPEED            = {GAIT_SPEED_DEG_PER_S:.0f} deg/s             ║")
    print(f"║  TRANSITION_SPEED      = {TRANSITION_SPEED_DEG_PER_S:.0f} deg/s             ║")
    print(f"║  Full cycle            = {STEP_TIME * NUM_STEPS:.2f} s (2 steps)       ║")
    print(f"║  Trajectory            = CYCLOIDAL               ║")
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

        # ── Quit ──────────────────────────────────────────────────────────────
        if cmd in ("q", "quit"):
            break

        # ── Sit ───────────────────────────────────────────────────────────────
        elif cmd == "s":
            print("● Sitting... (120 deg/s)")
            smooth_transition(SITTING_XYZ, duration=1.2)
            current_step_index = 0
            print("● Done — sitting pose reached.")

        # ── Stand ─────────────────────────────────────────────────────────────
        elif cmd == "stand":
            print("● Standing... (120 deg/s)")
            smooth_transition(STANDING_XYZ, duration=1.2)
            current_step_index = 0
            for leg in ALL_LEGS:
                current_pos[leg] = list(STANDING_XYZ)
            print("● Done — standing pose reached. Gait index reset.")

        # ── Forward ───────────────────────────────────────────────────────────
        elif cmd == "f":
            run_one_cycle(axis='z', direction=-1, label="FORWARD")

        elif cmd == "ff":
            run_continuous(axis='z', direction=-1, label="FORWARD")

        # ── Backward ──────────────────────────────────────────────────────────
        elif cmd == "b":
            run_one_cycle(axis='z', direction=+1, label="BACKWARD")

        elif cmd == "bb":
            run_continuous(axis='z', direction=+1, label="BACKWARD")

        # ── Right ─────────────────────────────────────────────────────────────
        elif cmd == "r":
            run_one_cycle(axis='x', direction=+1, label="STRAFE RIGHT")

        elif cmd == "rr":
            run_continuous(axis='x', direction=+1, label="STRAFE RIGHT")

        # ── Left ──────────────────────────────────────────────────────────────
        elif cmd == "l":
            run_one_cycle(axis='x', direction=-1, label="STRAFE LEFT")

        elif cmd == "ll":
            run_continuous(axis='x', direction=-1, label="STRAFE LEFT")

        # ── Unknown ───────────────────────────────────────────────────────────
        else:
            print("  Unknown command.")
            print("  Single cycle : f | b | r | l")
            print("  Continuous   : ff | bb | rr | ll  (press y to stop)")
            print("  Pose         : s | stand")
            print("  Exit         : q")

    # ── Exit ──────────────────────────────────────────────────────────────────
    print("● Returning to sitting pose before exit... (120 deg/s)")
    smooth_transition(SITTING_XYZ, duration=1.2)
    print("● Bye!")


if __name__ == "__main__":
    main()