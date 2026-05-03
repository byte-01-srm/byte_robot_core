#!/usr/bin/env python3
"""
ik_fl_sender.py — IK sender for FL leg (motors 1, 2, 3)

STARTUP FLOW
────────────
  1. Moves FL to the known standing pose (FL_STAND_ANGLES_DEG)
  2. Runs forward kinematics on those angles → prints the (wx, wy, wz) coordinates
  3. Enters interactive loop: you type new (wx, wy, wz) → IK → send

COORDINATE CONVENTION  (Y-up, same as _3dof_ik_with_shift_circle_method.py)
────────────────────────────────────────────────────────────────────────────
  wx  →  horizontal fore/aft or left/right   (cm)
  wy  →  HEIGHT  (positive = up)             (cm)
  wz  →  the other horizontal axis           (cm)

JOINT → MOTOR MAPPING (FL leg)
───────────────────────────────
  theta1  (hip rotation)   ->  Motor 1   flipped=True  gear=1.0
  theta2  (upper leg)      ->  Motor 2   flipped=True  gear=1.0
  theta3  (knee)           ->  Motor 3   flipped=True  gear=1.5
"""

import math
import pickle
import socket
import sys
from typing import Dict, List, Tuple

# ── Robot geometry ────────────────────────────────────────────────────────────
L1         =  5.995   # cm  vertical base link
LINK_CONST = -9.094   # cm  constant horizontal offset (circle radius)
L2         = 22.0     # cm  upper leg
L3         = 21.5     # cm  lower leg

# ── Known standing pose for FL (raw degrees, before main applies flip/gear) ───
#    maps to motors 1, 2, 3  as  [theta1_raw_deg, theta2_raw_deg, theta3_raw_deg]
FL_STAND_ANGLES_DEG: List[float] = [65.0, -135.0, 52.0]

# ── Per-joint offsets (degrees) added to IK output before sending ─────────────
#    Tune if the leg does not track correctly after homing.
FL_JOINT_OFFSET_DEG: List[float] = [0.0, 0.0, 0.0]

# ── Default standing angles for legs NOT being controlled ─────────────────────
DEFAULT_STANDING: Dict[str, List[float]] = {
    "bl": [62.0, -105.0,  3.0],   # motors 4, 5, 6
    "fr": [61.0, -105.0,  6.0],   # motors 7, 8, 9
    "br": [57.0, -105.0,  6.0],   # motors 10, 11, 12
}

# ── Socket ────────────────────────────────────────────────────────────────────
HOST = "127.0.0.1"
PORT = 50000


# ─────────────────────────────────────────────────────────────────────────────
# INVERSE KINEMATICS
# ─────────────────────────────────────────────────────────────────────────────

def inverse_kinematics(
    wx: float, wy: float, wz: float,
    L1: float, linkConst: float, L2: float, L3: float,
) -> Tuple[float, float, float]:
    """
    Solve IK for a foot target in the Y-up world frame.
    Returns (theta1, theta2, theta3) in radians.
    """
    # Remap Y-up world -> IK internal Z-up frame
    x, y, z = wx, wz, wy

    home_linkConst = (linkConst, 0, L1)

    radicand = x**2 + y**2 - linkConst**2
    if radicand < 0:
        raise ValueError(
            f"Target inside base circle (sqrt of {radicand:.4f}). "
            "Move foot further from hip origin."
        )

    d = math.sqrt(x**2 + y**2)
    a = math.atan2(y, x)
    b = math.acos(linkConst / d)

    T2x = linkConst * math.cos(a - b)
    T2y = linkConst * math.sin(a - b)
    T = (T2x, T2y)

    theta_A = math.atan2(home_linkConst[1], home_linkConst[0])
    # FIX 1: divide by linkConst before atan2 so its sign is accounted for.
    # When linkConst < 0 a plain atan2(T[1], T[0]) is off by π, producing a
    # theta1 that is ~180° wrong.
    theta_B = math.atan2(T[1] / linkConst, T[0] / linkConst)
    theta1  = theta_B - theta_A
    if theta1 < 0:
        theta1 += 2 * math.pi

    X = x - T[0]
    Y = y - T[1]
    Z = z - L1

    L = math.sqrt(X**2 + Y**2 + Z**2)

    if L > L2 + L3 + 1e-9:
        raise ValueError(
            f"Target out of reach: L={L:.2f} > L2+L3={L2+L3:.2f}"
        )

    cos_t3 = (L**2 - L2**2 - L3**2) / (2 * L2 * L3)
    cos_t3 = max(-1.0, min(1.0, cos_t3))
    # FIX 3: positive acos — this robot's knee bends in the +theta3 direction
    # (standing pose uses theta3 = +52°).  The old -acos forced the wrong elbow.
    theta3 = math.acos(cos_t3)

    # FIX 2: use the *signed* horizontal reach in the leg's sagittal plane.
    # The outward tangent direction at T is (sin θ_B, −cos θ_B).  Projecting
    # (X, Y) onto it gives the signed scalar; atan2(Z, signed_h) then gives
    # the correct beta even when the upper leg swings backward (signed_h < 0).
    dir_x    =  math.sin(theta_B)
    dir_y    = -math.cos(theta_B)
    signed_h = X * dir_x + Y * dir_y

    beta   = math.atan2(Z, signed_h)
    alpha  = math.atan2(L3 * math.sin(theta3), L2 + L3 * math.cos(theta3))
    theta2 = beta - alpha

    return theta1, theta2, theta3


# ─────────────────────────────────────────────────────────────────────────────
# FORWARD KINEMATICS
# ─────────────────────────────────────────────────────────────────────────────

def forward_kinematics(
    theta1: float, theta2: float, theta3: float,
    L1: float, linkConst: float, L2: float, L3: float,
) -> Tuple[float, float, float]:
    """
    Compute foot position (wx, wy, wz) in the Y-up world frame given
    joint angles in radians.

    How it works
    ────────────
    theta1 places the tangent point T on the base circle.
    From T, the leg lies in a vertical plane whose horizontal direction is
    perpendicular to OT (i.e. tangent to the base circle at T).
    theta2 / theta3 set the elevation of the two links inside that plane.
    The outward tangent direction is dir = (sin(theta_B), -cos(theta_B)),
    confirmed by back-substitution against known IK sample points.
    """
    theta_A = math.atan2(0.0, linkConst)   # = pi for negative linkConst
    theta_B = theta_A + theta1

    Tx = linkConst * math.cos(theta_B)
    Ty = linkConst * math.sin(theta_B)

    # Horizontal distance from T to foot in the sagittal plane of the leg
    h_dist = L2 * math.cos(theta2) + L3 * math.cos(theta2 + theta3)

    # Height in IK Z-up frame
    ik_z = L1 + L2 * math.sin(theta2) + L3 * math.sin(theta2 + theta3)

    # Outward tangent direction at T (verified against known sample)
    dir_x =  math.sin(theta_B)
    dir_y = -math.cos(theta_B)

    ik_x = Tx + h_dist * dir_x
    ik_y = Ty + h_dist * dir_y

    # Remap IK Z-up -> Y-up world frame
    wx = ik_x
    wy = ik_z   # height
    wz = ik_y

    return wx, wy, wz


# ─────────────────────────────────────────────────────────────────────────────
# CONVERSION & SENDING HELPERS
# ─────────────────────────────────────────────────────────────────────────────

def raw_angles_to_ik_rad(raw_deg: List[float]) -> Tuple[float, float, float]:
    """
    Raw payload degrees (before main applies flip/gear) -> IK joint radians.
    Subtracts FL_JOINT_OFFSET_DEG before converting.
    """
    t1 = math.radians(raw_deg[0] - FL_JOINT_OFFSET_DEG[0])
    t2 = math.radians(raw_deg[1] - FL_JOINT_OFFSET_DEG[1])
    t3 = math.radians(raw_deg[2] - FL_JOINT_OFFSET_DEG[2])
    return t1, t2, t3


def ik_rad_to_raw_angles(t1: float, t2: float, t3: float) -> List[float]:
    """
    IK joint radians -> raw degree list for the FL payload slot.
    main will apply flip + gear_ratio on top of these.
    """
    raw = [
        math.degrees(t1) + FL_JOINT_OFFSET_DEG[0],
        math.degrees(t2) + FL_JOINT_OFFSET_DEG[1],
        math.degrees(t3) + FL_JOINT_OFFSET_DEG[2],
    ]

    # Normalize to -180 to 180 degrees range to prevent out-of-bounds
    for i in range(len(raw)):
        raw[i] = ((raw[i] + 180) % 360) - 180

    return raw


def build_payload(fl_angles: List[float]) -> Dict[str, List[float]]:
    return {"fl": fl_angles, **DEFAULT_STANDING}


def send_payload(payload: Dict[str, List[float]]) -> None:
    data = pickle.dumps(payload, protocol=pickle.HIGHEST_PROTOCOL)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.connect((HOST, PORT))
        sock.sendall(data)


def move_fl_to(wx: float, wy: float, wz: float) -> None:
    """Full pipeline: coordinates -> IK -> raw angles -> socket send."""
    t1, t2, t3 = inverse_kinematics(wx, wy, wz, L1, LINK_CONST, L2, L3)
    fl_raw = ik_rad_to_raw_angles(t1, t2, t3)

    print(f"  IK     theta1={math.degrees(t1):8.2f} deg  "
          f"theta2={math.degrees(t2):8.2f} deg  "
          f"theta3={math.degrees(t3):8.2f} deg")
    print(f"  Raw    [{fl_raw[0]:.2f},  {fl_raw[1]:.2f},  {fl_raw[2]:.2f}] deg"
          "  (before main applies flip + gear_ratio)")

    send_payload(build_payload(fl_raw))
    print("  Sent.")


# ─────────────────────────────────────────────────────────────────────────────
# STARTUP  —  move to standing pose and show its coordinates via FK
# ─────────────────────────────────────────────────────────────────────────────

def startup_standing_pose() -> Tuple[float, float, float]:
    """
    Send FL_STAND_ANGLES_DEG directly to motors, then compute and display
    the foot coordinates via forward kinematics.
    Returns (wx, wy, wz) of the standing pose.
    """
    print("\n── Startup: moving FL to known standing pose ──────────────────────")
    print(f"  Raw angles  [theta1={FL_STAND_ANGLES_DEG[0]} deg,  "
          f"theta2={FL_STAND_ANGLES_DEG[1]} deg,  "
          f"theta3={FL_STAND_ANGLES_DEG[2]} deg]")

    send_payload(build_payload(FL_STAND_ANGLES_DEG))
    print("  Sent standing pose to motors.")

    # Back-calculate foot position with FK so the user knows their start coords
    t1, t2, t3 = raw_angles_to_ik_rad(FL_STAND_ANGLES_DEG)
    wx, wy, wz = forward_kinematics(t1, t2, t3, L1, LINK_CONST, L2, L3)

    print(f"\n  Forward kinematics result — current foot position:")
    print(f"  +-------------------------------------------------+")
    print(f"  |  wx = {wx:8.3f} cm   (fore/aft horizontal)     |")
    print(f"  |  wy = {wy:8.3f} cm   (HEIGHT — positive = up)  |")
    print(f"  |  wz = {wz:8.3f} cm   (lateral horizontal)      |")
    print(f"  +-------------------------------------------------+")

    return wx, wy, wz


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────

def main() -> None:
    print("=" * 62)
    print("  IK FL-Leg Sender  —  motors 1 (hip), 2 (upper), 3 (knee)")
    print(f"  Geometry  L1={L1}  linkConst={LINK_CONST}  L2={L2}  L3={L3}")
    print(f"  Socket    {HOST}:{PORT}")
    print("=" * 62)

    # ── Move to standing pose, display FK coordinates ─────────────────────────
    try:
        wx0, wy0, wz0 = startup_standing_pose()
    except ConnectionRefusedError:
        print(f"\n  ERROR: Cannot connect to {HOST}:{PORT}.")
        print("  Make sure main_with_current_temp.py is running and homing is complete.")
        sys.exit(1)
    except Exception as exc:
        print(f"\n  ERROR during startup: {exc}")
        sys.exit(1)

    # ── Single-shot CLI mode: python ik_fl_sender.py wx wy wz ────────────────
    if len(sys.argv) == 4:
        try:
            wx, wy, wz = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
        except ValueError:
            print("  Usage: python ik_fl_sender.py <wx> <wy> <wz>")
            sys.exit(1)
        print(f"\n  Target  wx={wx:.3f}  wy={wy:.3f}  wz={wz:.3f}")
        try:
            move_fl_to(wx, wy, wz)
        except (ValueError, ConnectionRefusedError) as exc:
            print(f"  ERROR: {exc}")
            sys.exit(1)
        return

    # ── Interactive loop ──────────────────────────────────────────────────────
    print("\n  Enter new foot coordinates in cm.  Commands:")
    print("  wx wy wz       — move foot to these coordinates")
    print("  reset / r      — return to standing pose")
    print("  q / quit       — exit")
    print(f"\n  Starting position:  wx={wx0:.3f}  wy={wy0:.3f}  wz={wz0:.3f}\n")

    while True:
        try:
            raw = input("  wx  wy  wz > ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n  Exiting.")
            break

        if not raw:
            continue

        if raw.lower() in ("q", "quit", "exit"):
            print("  Exiting.")
            break

        if raw.lower() in ("reset", "r", "stand"):
            print("\n  Returning to standing pose...")
            try:
                wx0, wy0, wz0 = startup_standing_pose()
                print(f"\n  Back to:  wx={wx0:.3f}  wy={wy0:.3f}  wz={wz0:.3f}\n")
            except Exception as exc:
                print(f"  ERROR: {exc}")
            continue

        parts = raw.split()
        if len(parts) != 3:
            print("  Enter exactly 3 numbers, e.g.:  15.4  -30.9  3.2")
            print("  Or type 'reset' / 'q'")
            continue

        try:
            wx, wy, wz = float(parts[0]), float(parts[1]), float(parts[2])
        except ValueError:
            print("  Could not parse numbers. Try again.")
            continue

        print(f"\n  Target  wx={wx:.3f}  wy={wy:.3f} (height)  wz={wz:.3f}")
        try:
            move_fl_to(wx, wy, wz)
        except ValueError as exc:
            print(f"  IK error: {exc}")
        except ConnectionRefusedError:
            print(f"  Cannot connect to {HOST}:{PORT}. Is main still running?")
        print()


if __name__ == "__main__":
    main()