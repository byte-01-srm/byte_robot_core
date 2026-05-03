#!/usr/bin/env python3
"""
Cycloidal gait for FR leg only — Motor 7 (hip) / 8 (upper) / 9 (knee)
All other motors (1-6, 10-12) hold sitting position.

Press 'f'  →  one step cycle (swing + stance)
Press 'q'  →  quit

Run AFTER main_with_current_temp.py shows: 🎯 POST-HOMING LIVE CONTROL READY
"""

import math
import pickle
import socket
import sys
import termios
import time
import tty


# ── Socket ────────────────────────────────────────────────────────────────────
HOST = "127.0.0.1"
PORT = 50000

# ── Motor map reminder ────────────────────────────────────────────────────────
# Motor 7  → FR hip        can1
# Motor 8  → FR upper leg  can1
# Motor 9  → FR knee       can1

# ── FR leg nominal standing position (x, y, z) in cm ─────────────────────────
X_NOM =  -9.094   # hip abduction constant
Y_NOM =   25.0    # ground contact y
Z_NOM =    6.0    # ground contact z (stride axis)

# ── All other legs hold sitting position (matches homing nudge angles) ────────
# fl → motors 1,2,3   bl → motors 4,5,6   br → motors 10,11,12
SITTING = [-9.094, 25.0, 9.0]

# ── Gait parameters ───────────────────────────────────────────────────────────
STRIDE    = 5.0   # cm — total horizontal stride per step
H_SWING   = 5.0   # cm — actual peak lift during swing  (foot goes UP)
H_STANCE  = 5.0   # cm — actual peak press during stance (foot goes DOWN)

SWING_DURATION  = 0.2    # seconds
STANCE_DURATION = 0.2    # seconds
LOOP_HZ         = 50.0   # coordinate send rate (Hz)

# -1 → stride in -z direction (forward for this robot)
# +1 → stride in +z direction (backward)
SWING_DIRECTION  = -1   # swing moves forward
STANCE_DIRECTION = +1   # stance returns backward


# ─── Trajectory math ──────────────────────────────────────────────────────────

def _horiz(t: float) -> float:
    """0 at t=0, 1 at t=1 — smooth cycloidal S-curve."""
    return t - math.sin(2 * math.pi * t) / (2 * math.pi)


def _vert(t: float) -> float:
    """0 at edges, peak=2 at t=0.5."""
    return 1.0 - math.cos(2 * math.pi * t)


def _trajectory(z_start: float, z_end: float, height: float,
                duration: float, lift_up: bool = True):
    """
    Returns list of (x, y, z) foot coords for one phase.

    lift_up=True  → foot lifts UP   (y decreases) — used for swing
    lift_up=False → foot presses DOWN (y increases) — used for stance
    """
    n = max(1, int(duration * LOOP_HZ))
    sign = -1 if lift_up else +1
    pts = []
    for i in range(n + 1):
        t = i / n
        z = z_start + (z_end - z_start) * _horiz(t)
        y = Y_NOM + sign * (height / 2.0) * _vert(t)
        pts.append((X_NOM, y, z))
    return pts


# ─── Socket sender ─────────────────────────────────────────────────────────────

def _send_fr(x: float, y: float, z: float):
    """
    Send coords for FR leg (motors 7, 8, 9) only.
    All other legs (motors 1-6, 10-12) hold at sitting position.
    """
    payload = {
        "fl": [float(x), float(y), float(z)] ,                     # motors 1, 2, 3  — hold
        "bl": [float(x), float(y), float(z)],                     # motors 4, 5, 6  — hold
        "fr": [float(x), float(y), float(z)],   # motors 7, 8, 9  ← MOVE
        "br": [float(x), float(y), float(z)],                     # motors 10,11,12 — hold
    }
    data = pickle.dumps(payload)
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(0.5)
            s.connect((HOST, PORT))
            s.sendall(data)
    except Exception as e:
        print(f"\r[Socket Error] {e}", flush=True)


# ─── Step cycle ────────────────────────────────────────────────────────────────

def run_step_cycle(cycle_num: int):
    """Execute one complete step: swing forward (up) + stance return (down)."""
    dt = 1.0 / LOOP_HZ

    z_back  = Z_NOM
    z_front = Z_NOM + STRIDE * SWING_DIRECTION

    # ── Swing: foot lifts UP and moves forward ────────────────────────────────
    swing_pts = _trajectory(z_back, z_front, H_SWING, SWING_DURATION, lift_up=True)
    print(
        f"\r  [Cycle {cycle_num}] SWING   "
        f"M7/8/9: z {z_back:.1f} → {z_front:.1f} cm  "
        f"lift ↑ {H_SWING} cm  ({SWING_DURATION}s)",
        flush=True,
    )
    for pt in swing_pts:
        _send_fr(*pt)
        time.sleep(dt)

    # ── Stance: foot presses DOWN and returns backward ────────────────────────
    stance_pts = _trajectory(z_front, z_back, H_STANCE, STANCE_DURATION, lift_up=False)
    print(
        f"\r  [Cycle {cycle_num}] STANCE  "
        f"M7/8/9: z {z_front:.1f} → {z_back:.1f} cm  "
        f"press ↓ {H_STANCE} cm  ({STANCE_DURATION}s)",
        flush=True,
    )
    for pt in stance_pts:
        _send_fr(*pt)
        time.sleep(dt)

    # Settle M7/8/9 back to exact nominal
    _send_fr(X_NOM, Y_NOM, Z_NOM)
    print(
        f"\r  [Cycle {cycle_num}] ✓ Done  "
        f"M7/8/9 back at ({X_NOM}, {Y_NOM}, {Z_NOM})",
        flush=True,
    )


# ─── Keyboard ──────────────────────────────────────────────────────────────────

def _getch() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ─── Main ──────────────────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("  Cycloidal Gait — FR Leg (Motor 7 / 8 / 9)")
    print("=" * 60)
    print(f"  Motor 7  : hip      can1")
    print(f"  Motor 8  : upper    can1")
    print(f"  Motor 9  : knee     can1")
    print(f"  Nominal  : x={X_NOM}  y={Y_NOM}  z={Z_NOM} cm")
    print(f"  Stride   : {STRIDE} cm")
    print(f"  Swing    : H={H_SWING} cm  ↑ lifts up    dir={SWING_DIRECTION}  ({SWING_DURATION}s)")
    print(f"  Stance   : H={H_STANCE} cm  ↓ presses down  dir={STANCE_DIRECTION}  ({STANCE_DURATION}s)")
    print(f"  Other motors (1-6, 10-12): holding sitting position")
    print("=" * 60)
    print("  f  →  one step cycle")
    print("  q  →  quit")
    print("=" * 60)

    print(f"\nMoving M7/8/9 to nominal position...", flush=True)
    _send_fr(X_NOM, Y_NOM, Z_NOM)
    time.sleep(0.5)
    print("Ready.\n", flush=True)

    cycle_count = 0

    try:
        while True:
            key = _getch()

            if key in ('q', '\x03'):
                print("\nQuitting...", flush=True)
                break

            elif key == 'f':
                cycle_count += 1
                print(f"\n▶  Cycle {cycle_count} — M7/8/9 moving...", flush=True)
                run_step_cycle(cycle_count)
                print(
                    f"\nReady. Total cycles: {cycle_count}  (f=next  q=quit)\n",
                    flush=True,
                )

            else:
                print(f"\r  Unknown key '{key}'", flush=True)

    finally:
        print(f"\nReturning M7/8/9 to nominal...", flush=True)
        _send_fr(X_NOM, Y_NOM, Z_NOM)
        print("Done.")


if __name__ == "__main__":
    main()