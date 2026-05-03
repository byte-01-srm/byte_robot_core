#!/usr/bin/env python3
"""
Trot gait — diagonal leg pairs move simultaneously.

Pair A : FR (M7,8,9) + BL (M4,5,6)   — swing together
Pair B : FL (M1,2,3) + BR (M10,11,12) — swing together

Each 'f' press = one complete trot cycle:
  Half 1 → Pair A swings forward (air ↑),  Pair B stances backward (ground ↓)
  Half 2 → Pair B swings forward (air ↑),  Pair A stances backward (ground ↓)
  All legs return to nominal at the end.

Press 'f'  →  one trot cycle
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
# FL: motors  1,  2,  3   can0
# BL: motors  4,  5,  6   can0
# FR: motors  7,  8,  9   can1
# BR: motors 10, 11, 12   can1

# ── Trot diagonal pairs ───────────────────────────────────────────────────────
# Pair A : FR + BL  (front-right + back-left)
# Pair B : FL + BR  (front-left  + back-right)

# ── Per-leg nominal standing position (x, y, z) in cm ────────────────────────
# Adjust individually per leg if they differ
LEG_NOM = {
    "fl": (-9.094, 25.0, 6.0),
    "bl": (-9.094, 25.0, 6.0),
    "fr": (-9.094, 25.0, 6.0),
    "br": (-9.094, 25.0, 6.0),
}

# ── Gait parameters ───────────────────────────────────────────────────────────
STRIDE        = 5.0    # cm — stride length per half-cycle
H_SWING       = 5.0    # cm — peak lift during swing (foot goes UP)
H_STANCE      = 5.0    # cm — peak press during stance (foot goes DOWN)

STEP_DURATION = 0.2    # seconds per half-cycle (swing and stance are equal)
LOOP_HZ       = 50.0   # coordinate send rate (Hz)

# -1 → stride in -z (forward for this robot)
# +1 → stride in +z (backward)
SWING_DIRECTION  = -1
STANCE_DIRECTION = +1


# ─── Trajectory math ──────────────────────────────────────────────────────────

def _horiz(t: float) -> float:
    """0 at t=0, 1 at t=1 — smooth cycloidal S-curve."""
    return t - math.sin(2 * math.pi * t) / (2 * math.pi)


def _vert(t: float) -> float:
    """0 at edges, peak=2 at t=0.5."""
    return 1.0 - math.cos(2 * math.pi * t)


def _trajectory(leg: str, z_start: float, z_end: float,
                height: float, duration: float, lift_up: bool = True):
    """
    Generate (x, y, z) trajectory for one leg over one phase.
    lift_up=True  → foot lifts UP   (swing)
    lift_up=False → foot presses DOWN (stance)
    Uses per-leg x_nom and y_nom from LEG_NOM.
    """
    x_nom, y_nom, _ = LEG_NOM[leg]
    n    = max(1, int(duration * LOOP_HZ))
    sign = -1 if lift_up else +1
    pts  = []
    for i in range(n + 1):
        t = i / n
        z = z_start + (z_end - z_start) * _horiz(t)
        y = y_nom + sign * (height / 2.0) * _vert(t)
        pts.append((x_nom, y, z))
    return pts


# ─── Socket sender ────────────────────────────────────────────────────────────

def _send_all(fl, bl, fr, br):
    """Send one payload with all 4 leg positions simultaneously."""
    payload = {
        "fl": [float(v) for v in fl],
        "bl": [float(v) for v in bl],
        "fr": [float(v) for v in fr],
        "br": [float(v) for v in br],
    }
    data = pickle.dumps(payload)
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(0.5)
            s.connect((HOST, PORT))
            s.sendall(data)
    except Exception as e:
        print(f"\r[Socket Error] {e}", flush=True)


def _send_nominal():
    """Move all 4 legs to their nominal standing positions."""
    _send_all(
        LEG_NOM["fl"],
        LEG_NOM["bl"],
        LEG_NOM["fr"],
        LEG_NOM["br"],
    )


# ─── Trot cycle ───────────────────────────────────────────────────────────────

def run_trot_cycle(cycle_num: int):
    """
    One complete trot cycle — 2 halves.

    z positions after each half:
      Half 1 end : Pair A at z_a_front,  Pair B at z_b_back
      Half 2 end : Pair A at z_nom,      Pair B at z_nom  ← clean reset
    """
    dt = 1.0 / LOOP_HZ

    # z endpoints for each pair (computed from each leg's own z_nom)
    z_fr_nom  = LEG_NOM["fr"][2]
    z_bl_nom  = LEG_NOM["bl"][2]
    z_fl_nom  = LEG_NOM["fl"][2]
    z_br_nom  = LEG_NOM["br"][2]

    z_a_front = z_fr_nom + STRIDE * SWING_DIRECTION    # Pair A swings to here
    z_b_back  = z_fl_nom + STRIDE * STANCE_DIRECTION   # Pair B stances to here

    # ── Half 1: Pair A swings forward, Pair B stances backward ───────────────
    # Pair A (FR + BL): swing z_nom → z_a_front, foot lifts UP
    # Pair B (FL + BR): stance z_nom → z_b_back, foot presses DOWN
    fr_sw_1 = _trajectory("fr", z_fr_nom,  z_a_front, H_SWING,  STEP_DURATION, lift_up=True)
    bl_sw_1 = _trajectory("bl", z_bl_nom,  z_a_front, H_SWING,  STEP_DURATION, lift_up=True)
    fl_st_1 = _trajectory("fl", z_fl_nom,  z_b_back,  H_STANCE, STEP_DURATION, lift_up=False)
    br_st_1 = _trajectory("br", z_br_nom,  z_b_back,  H_STANCE, STEP_DURATION, lift_up=False)

    print(
        f"\r  [Cycle {cycle_num}] Half 1 — "
        f"FR+BL swing ↑ {z_fr_nom:.1f}→{z_a_front:.1f}  "
        f"FL+BR stance ↓ {z_fl_nom:.1f}→{z_b_back:.1f}",
        flush=True,
    )
    for i in range(len(fr_sw_1)):
        _send_all(fl_st_1[i], bl_sw_1[i], fr_sw_1[i], br_st_1[i])
        time.sleep(dt)

    # ── Half 2: Pair B swings forward, Pair A stances backward ───────────────
    # Pair B (FL + BR): swing z_b_back → z_b_nom, foot lifts UP
    # Pair A (FR + BL): stance z_a_front → z_a_nom, foot presses DOWN
    fl_sw_2 = _trajectory("fl", z_b_back,  z_fl_nom, H_SWING,  STEP_DURATION, lift_up=True)
    br_sw_2 = _trajectory("br", z_b_back,  z_br_nom, H_SWING,  STEP_DURATION, lift_up=True)
    fr_st_2 = _trajectory("fr", z_a_front, z_fr_nom, H_STANCE, STEP_DURATION, lift_up=False)
    bl_st_2 = _trajectory("bl", z_a_front, z_bl_nom, H_STANCE, STEP_DURATION, lift_up=False)

    print(
        f"\r  [Cycle {cycle_num}] Half 2 — "
        f"FL+BR swing ↑ {z_b_back:.1f}→{z_fl_nom:.1f}  "
        f"FR+BL stance ↓ {z_a_front:.1f}→{z_fr_nom:.1f}",
        flush=True,
    )
    for i in range(len(fl_sw_2)):
        _send_all(fl_sw_2[i], bl_st_2[i], fr_st_2[i], br_sw_2[i])
        time.sleep(dt)

    # All 4 legs back at nominal
    _send_nominal()
    print(f"\r  [Cycle {cycle_num}] ✓ Done — all legs at nominal", flush=True)


# ─── Keyboard ─────────────────────────────────────────────────────────────────

def _getch() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    print("=" * 65)
    print("  Trot Gait — FR+BL / FL+BR Diagonal Pairs")
    print("=" * 65)
    print("  Pair A : FR (M7/8/9)  + BL (M4/5/6)    swing together")
    print("  Pair B : FL (M1/2/3)  + BR (M10/11/12)  swing together")
    print(f"  Stride : {STRIDE} cm   H_swing: {H_SWING} cm   H_stance: {H_STANCE} cm")
    print(f"  Half   : {STEP_DURATION}s  |  Rate: {LOOP_HZ:.0f} Hz")
    print("  Nominal positions:")
    for leg, (x, y, z) in LEG_NOM.items():
        print(f"    {leg.upper():<4}: x={x}  y={y}  z={z}")
    print("=" * 65)
    print("  Half 1 : FR+BL swing ↑ fwd,  FL+BR stance ↓ bwd")
    print("  Half 2 : FL+BR swing ↑ fwd,  FR+BL stance ↓ bwd")
    print("=" * 65)
    print("  f  →  one trot cycle (half 1 + half 2)")
    print("  q  →  quit")
    print("=" * 65)

    print("\nMoving all legs to nominal...", flush=True)
    _send_nominal()
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
                print(f"\n▶  Trot cycle {cycle_count}...", flush=True)
                run_trot_cycle(cycle_count)
                print(
                    f"\nReady. Total cycles: {cycle_count}  (f=next  q=quit)\n",
                    flush=True,
                )

            else:
                print(f"\r  Unknown key '{key}'", flush=True)

    finally:
        print("\nReturning all legs to nominal...", flush=True)
        _send_nominal()
        print("Done.")


if __name__ == "__main__":
    main()