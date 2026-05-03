#!/usr/bin/env python3
"""
leg_commander.py — Interactive per-leg coordinate sender for BYTE-01
=====================================================================
Set (x, y, z) for each leg independently, then send all at once.

Keybinds:
  1  →  Edit FL  (front-left)
  2  →  Edit BL  (back-left)
  3  →  Edit FR  (front-right)
  4  →  Edit BR  (back-right)
  a  →  Edit ALL legs at once (same coords)
  s  →  Send current payload to robot
  v  →  View current payload
  r  →  Reset all legs to default standing
  q  →  Quit (sends neutral pose first)

Flow example:
  Press 1  → enter new x/y/z for FL
  Press 3  → enter new x/y/z for FR
  Press v  → confirm the values look right
  Press s  → send to robot
"""

import pickle
import socket
import sys
import termios
import tty

# ── Connection ──────────────────────────────────────────────────────────────
HOST = "127.0.0.1"
PORT = 50000

# ── Default position ─────────────────────────────────────────────────────────
DEFAULT = {
    "fl": [-9.094, 25.0, 3.0],
    "bl": [-9.094, 20.0, 3.0],
    "fr": [-9.094, 22.0, 3.0],
    "br": [-9.094, 20.0, 3.0],
}

LEG_NAMES = {
    "1": "fl",
    "2": "bl",
    "3": "fr",
    "4": "br",
}

# ── Helpers ──────────────────────────────────────────────────────────────────

def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def send_payload(payload: dict):
    try:
        data = pickle.dumps(payload)
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(2.0)
            s.connect((HOST, PORT))
            s.sendall(data)
        print("  ✓ Sent successfully.")
    except ConnectionRefusedError:
        print("  ✗ Connection refused — is main_with_ik.py running?")
    except Exception as e:
        print(f"  ✗ Send failed: {e}")


def prompt_coords(leg: str, current: list) -> list:
    """Prompt user for new (x, y, z). Press Enter to keep existing value."""
    print(f"\n  Editing {leg.upper()}  (current: x={current[0]}  y={current[1]}  z={current[2]})")
    print("  Press Enter on any field to keep the current value.\n")
    new = list(current)
    labels = ["x", "y", "z"]
    for i, label in enumerate(labels):
        raw = input(f"    {label} [{current[i]}]: ").strip()
        if raw:
            try:
                new[i] = float(raw)
            except ValueError:
                print(f"    Invalid — keeping {label}={current[i]}")
    return new


def print_payload(payload: dict):
    print("\n  ┌─────────────────────────────────────────────────┐")
    print("  │  Leg   │     x      │     y      │     z      │")
    print("  ├─────────────────────────────────────────────────┤")
    for leg in ["fl", "fr", "bl", "br"]:
        x, y, z = payload[leg]
        print(f"  │  {leg.upper()}   │ {x:>10.3f} │ {y:>10.3f} │ {z:>10.3f} │")
    print("  └─────────────────────────────────────────────────┘\n")


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    payload = {k: list(v) for k, v in DEFAULT.items()}

    print("=" * 54)
    print("  BYTE-01 Leg Commander — Individual Coord Sender")
    print("=" * 54)
    print("  1/2/3/4  →  Edit FL / BL / FR / BR")
    print("  a        →  Edit ALL legs (same coords)")
    print("  s        →  Send current payload")
    print("  v        →  View current payload")
    print("  r        →  Reset all to default standing")
    print("  q        →  Quit")
    print("=" * 54)
    print_payload(payload)

    while True:
        print("  Waiting for key (1/2/3/4/a/s/v/r/q)...", end="", flush=True)
        key = get_key()
        print(f" '{key}'")

        if key in LEG_NAMES:
            leg = LEG_NAMES[key]
            payload[leg] = prompt_coords(leg, payload[leg])
            print(f"  {leg.upper()} updated → x={payload[leg][0]}  y={payload[leg][1]}  z={payload[leg][2]}")

        elif key == "a":
            print("\n  Editing ALL legs (same x/y/z will be applied to FL, BL, FR, BR)")
            ref = prompt_coords("ALL", payload["fl"])
            for leg in ["fl", "bl", "fr", "br"]:
                payload[leg] = list(ref)
            print("  All legs updated.")
            print_payload(payload)

        elif key == "s":
            print_payload(payload)
            send_payload(payload)

        elif key == "v":
            print_payload(payload)

        elif key == "r":
            payload = {k: list(v) for k, v in DEFAULT.items()}
            print("  All legs reset to default standing position.")
            print_payload(payload)

        elif key == "q":
            print("\n  Sending neutral pose and quitting...")
            neutral = {k: list(v) for k, v in DEFAULT.items()}
            send_payload(neutral)
            break

        else:
            print("  Unknown key — use 1/2/3/4/a/s/v/r/q")


if __name__ == "__main__":
    main()