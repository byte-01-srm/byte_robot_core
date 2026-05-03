#!/usr/bin/env python3
import math
import numpy as np


# ─── Robot Link Lengths (cm) ─────────────────────────────────────────────────
L1 = 5.995
LINK_CONST = -9.094
L2 = 22.0
L3 = 21.5

# ─── Per-Leg Calibration Data ─────────────────────────────────────────────────
# For each leg, provide:
#   "coords"    : (x, y, z) foot position in cm at the sitting pose
#   "motor_deg" : (theta1, theta2, theta3) actual motor angles at that pose
#
# Flip and gear_ratio are handled downstream in motor_config.json — not here.
# Right legs and left legs can differ due to mechanical assembly variations.


LEG_CALIBRATION = {
    "fr": {
        "coords":    (-9.094, 11.0, 13.0),
        "motor_deg": (89.0, -110.0, 8.0),
    },
    "fl": {
        "coords":    (-9.094, 11.0, 13.0),   # ← fill actual measured coords
        "motor_deg": (90.0, -110.0, 5.0),   # ← fl theta1 is 62, not 60
    },
    "br": {
        "coords":    (-9.094, 11.0, 13.0),   # ← fill actual measured coords
        "motor_deg": (90.0, -98.0, 6.0),   # ← fill actual motor angles
    },
    "bl": {
        "coords":    (-9.094, 11.0, 13.0),   # ← fill actual measured coords
        "motor_deg": (85.0, -98.0, 8.0),   # ← fill actual motor angles
    },
}

LEGS = ("fl", "bl", "fr", "br")


# ─── Core IK ─────────────────────────────────────────────────────────────────
def inverse_kinematics(x, y, z, L1=L1, linkConst=LINK_CONST, L2=L2, L3=L3):
    """
    Compute joint angles (radians) for a foot target (x, y, z) in cm.

    Returns:
        (theta1, theta2, theta3) in radians
        theta1 : hip abduction / shoulder rotation
        theta2 : upper-leg pitch
        theta3 : knee pitch (always <= 0, leg bends backward)

    Raises:
        ValueError if target is geometrically unreachable.
    """
    home_linkConst = (linkConst, 0, L1)

    radicand = x**2 + y**2 - linkConst**2
    if radicand < 0:
        raise ValueError(
            f"Target inside hip circle: sqrt({radicand:.6f}) undefined. "
            "Move foot further from origin."
        )
    lenTangent = math.sqrt(radicand)

    if lenTangent > L2 + L3:
        raise ValueError(
            f"Target out of reach: lenTangent {lenTangent:.4f} > L2+L3 {L2+L3:.4f}"
        )

    d = math.sqrt(x**2 + y**2)
    a = np.arctan2(y, x)
    b = np.arccos(np.clip(linkConst / d, -1.0, 1.0))

    T = (linkConst * np.cos(a - b), linkConst * np.sin(a - b), L1)

    theta_A = np.arctan2(home_linkConst[1], home_linkConst[0])
    theta_B = np.arctan2(T[1], T[0])
    theta1  = theta_B - theta_A
    if theta1 < 0:
        theta1 = (theta1 + np.pi) % (2 * np.pi) - np.pi

    X = x - T[0]
    Y = y - T[1]
    Z = z - T[2]

    L = np.sqrt(X**2 + Y**2 + Z**2)

    if L > L2 + L3 + 1e-9:
        raise ValueError(
            f"Target out of reach for L2/L3: reach {L:.4f} > {L2+L3:.4f}"
        )

    cos_theta3 = np.clip((L**2 - L2**2 - L3**2) / (2 * L2 * L3), -1.0, 1.0)
    theta3 = -np.arccos(cos_theta3)

    beta   = np.arctan2(Z, np.sqrt(X**2 + Y**2))
    alpha  = np.arctan2(L3 * np.sin(theta3), L2 + L3 * np.cos(theta3))
    theta2 = beta - alpha

    return theta1, theta2, theta3


# ─── Per-Leg Calibration ──────────────────────────────────────────────────────
def compute_ik_offsets(calibration: dict = LEG_CALIBRATION) -> dict:
    """
    Compute per-joint offsets for each leg individually.

    Call this ONCE at startup after homing completes.

    Returns:
        {
            "fl": (offset1, offset2, offset3),
            "bl": (offset1, offset2, offset3),
            "fr": (offset1, offset2, offset3),
            "br": (offset1, offset2, offset3),
        }
        Apply as: motor_deg = ik_deg + offset
    """
    offsets = {}

    print("[IK Calibration] Computing per-leg offsets from sitting position:")

    for leg in LEGS:
        data = calibration[leg]
        x, y, z       = data["coords"]
        motor_deg     = data["motor_deg"]

        t1_ik, t2_ik, t3_ik = inverse_kinematics(x, y, z)
        ik_deg = (math.degrees(t1_ik), math.degrees(t2_ik), math.degrees(t3_ik))

        offsets[leg] = tuple(motor_deg[i] - ik_deg[i] for i in range(3))

        print(f"  [{leg.upper()}]  coords={data['coords']}")
        print(f"         IK output  = {[f'{v:.3f}°' for v in ik_deg]}")
        print(f"         motor_deg  = {[f'{v:.3f}°' for v in motor_deg]}")
        print(f"         offsets    = {[f'{v:.3f}°' for v in offsets[leg]]}")

    return offsets


# ─── Motor Command ────────────────────────────────────────────────────────────
def ik_to_motor_deg(x, y, z, offsets: dict, leg: str) -> tuple:
    """
    Convert foot target (x, y, z) to calibrated motor angles in degrees.

    Args:
        x, y, z  : foot target in cm
        offsets  : dict from compute_ik_offsets()
        leg      : "fl" | "bl" | "fr" | "br"

    Returns:
        (motor_deg1, motor_deg2, motor_deg3) in degrees
        Ready to pass into convert_coords_to_motor_targets → flip + gear_ratio → CAN.
    """
    t1, t2, t3 = inverse_kinematics(x, y, z)
    leg_offsets = offsets[leg]
    print("From Raw IK: ", math.degrees(t1), math.degrees(t2), math.degrees(t3))

    return (
        math.degrees(t1) + leg_offsets[0],
        math.degrees(t2) + leg_offsets[1],
        math.degrees(t3) + leg_offsets[2],
    )


# ─── Entry Point (testing only) ──────────────────────────────────────────────
if __name__ == "__main__":
    print("=" * 60)
    print("  IK Per-Leg Calibration + Motor Angle Test")
    print("=" * 60)

    # Step 1: compute per-leg offsets
    offsets = compute_ik_offsets()

    # Step 2: verify each leg — sitting coords should return exact motor angles
    print("\n[Verification] Sitting position round-trip per leg:")
    print(f"  {'Leg':<6} {'θ1':>10} {'θ2':>10} {'θ3':>10}   Expected")
    print("  " + "-" * 65)
    for leg in LEGS:
        x, y, z = LEG_CALIBRATION[leg]["coords"]
        expected = LEG_CALIBRATION[leg]["motor_deg"]
        m1, m2, m3 = ik_to_motor_deg(x, y, z, offsets, leg)
        match = "✓" if all(abs(m - e) < 0.001 for m, e in zip((m1,m2,m3), expected)) else "✗"
        print(f"  {leg.upper():<6} {m1:>10.3f}° {m2:>10.3f}° {m3:>10.3f}°   "
              f"{list(expected)} {match}")

    # Step 3: test a target position for each leg
    print("\n[Test] Target (-9.094, 5.0, -30.0) for each leg:")
    print(f"  {'Leg':<6} {'θ1':>10} {'θ2':>10} {'θ3':>10}")
    print("  " + "-" * 45)
    for leg in LEGS:
        try:
            m1, m2, m3 = ik_to_motor_deg(-9.094, 5.0, -30.0, offsets, leg)
            print(f"  {leg.upper():<6} {m1:>10.3f}° {m2:>10.3f}° {m3:>10.3f}°")
        except ValueError as e:
            print(f"  {leg.upper():<6} ERROR: {e}")

