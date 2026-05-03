#!/usr/bin/env python3
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

L1 = 5.995
LINK_CONST = -9.094
L2 = 22.0
L3 = 21.5

def inverse_kinematics(x, y, z):
    radicand = x**2 + y**2 - LINK_CONST**2
    if radicand < 0:
        raise ValueError("Target inside hip circle.")
    if math.sqrt(radicand) > L2 + L3:
        raise ValueError("Target out of reach.")
    d = math.sqrt(x**2 + y**2)
    a = np.arctan2(y, x)
    b = np.arccos(np.clip(LINK_CONST / d, -1.0, 1.0))
    T = (LINK_CONST * np.cos(a - b), LINK_CONST * np.sin(a - b))
    theta_A = np.arctan2(0, LINK_CONST)
    theta_B = np.arctan2(T[1], T[0])
    theta1 = theta_B - theta_A
    if theta1 < 0:
        theta1 += 2 * np.pi
    X = x - T[0]; Y = y - T[1]; Z = z - L1
    L = np.sqrt(X**2 + Y**2 + Z**2)
    cos_t3 = np.clip((L**2 - L2**2 - L3**2) / (2 * L2 * L3), -1.0, 1.0)
    theta3 = -np.arccos(cos_t3)
    beta  = np.arctan2(Z, np.sqrt(X**2 + Y**2))
    alpha = np.arctan2(L3 * np.sin(theta3), L2 + L3 * np.cos(theta3))
    theta2 = beta - alpha
    return theta1, theta2, theta3, T

def get_joint_positions(x, y, z):
    t1, t2, t3, T = inverse_kinematics(x, y, z)
    O    = np.array([0.0, 0.0, 0.0])
    Hip  = np.array([T[0], T[1], L1])
    horiz = np.sqrt((x - T[0])**2 + (y - T[1])**2)
    ux = (x - T[0]) / horiz if horiz > 1e-9 else 1.0
    uy = (y - T[1]) / horiz if horiz > 1e-9 else 0.0
    Knee = Hip + np.array([ux * L2 * np.cos(t2),
                            uy * L2 * np.cos(t2),
                            L2 * np.sin(t2)])
    Foot = np.array([x, y, z])
    return O, Hip, Knee, Foot, (math.degrees(t1), math.degrees(t2), math.degrees(t3))

# ── CHANGE THESE to test different positions ─────────────────────────────────
TARGETS = {
    "Sitting":      (-9.094, 11.0,   9.0),
    "Standing":     (-9.094,  5.0, -30.0),
    "Step Forward": (-9.094, 11.0, -20.0),
}
# ─────────────────────────────────────────────────────────────────────────────

fig = plt.figure(figsize=(12, 6))
colors = ["blue", "green", "red", "orange"]

for i, (label, (x, y, z)) in enumerate(TARGETS.items()):
    try:
        O, Hip, Knee, Foot, angles = get_joint_positions(x, y, z)
        pts = np.array([O, Hip, Knee, Foot])
        c = colors[i % len(colors)]

        # 3D view
        ax3d = fig.add_subplot(1, 2, 1, projection='3d')
        ax3d.plot(pts[:,0], pts[:,1], pts[:,2], '-o', color=c, label=label, linewidth=2)
        ax3d.scatter(*Foot, color=c, s=80, zorder=5)
        ax3d.set_xlabel("X (cm)"); ax3d.set_ylabel("Y (cm)"); ax3d.set_zlabel("Z (cm)")
        ax3d.set_title("3D View")

        # Side view (Y vs Z)
        ax2d = fig.add_subplot(1, 2, 2)
        ax2d.plot(pts[:,1], pts[:,2], '-o', color=c, label=label, linewidth=2)
        ax2d.set_xlabel("Y (cm)"); ax2d.set_ylabel("Z (cm)")
        ax2d.set_title("Side View (Y-Z)")
        ax2d.grid(True)
        ax2d.axhline(0, color='gray', linewidth=0.5)

        print(f"{label:15s} → θ1={angles[0]:7.2f}°  θ2={angles[1]:7.2f}°  θ3={angles[2]:7.2f}°")

    except ValueError as e:
        print(f"{label} → ERROR: {e}")

ax3d.legend(); ax2d.legend()
plt.tight_layout()
plt.savefig("leg_check.png", dpi=150)
plt.show()
print("\nSaved: leg_check.png")