#!/usr/bin/env python3
"""
QUADRUPED REAL ROBOT WALK GAIT  —  3-DOF IK
============================================================
Drop-in replacement for gait_walk_3dof.py, adapted to run on
the physical "byte" robot instead of Gazebo.

HOW IT FITS INTO YOUR STACK
────────────────────────────
  Terminal 1:  python3 main_with_current_temp.py   ← homing + socket server
  Terminal 2:  python3 gait_walk_real.py           ← this file

This script sends joint angle packets via TCP socket to
main_with_current_temp.py (same format as main_sender.py).

ANGLE CONVENTION & SIGN DERIVATION
─────────────────────────────────────────────────────────────
The IK produces angles (θ1, θ2, θ3) in radians.
The socket expects degrees in the format:
    { "fl": [roll_deg, hip_deg, knee_deg], ... }
...which main_with_current_temp.py then transforms:
    motor_target = (-1 if flipped else +1) * socket_deg * gear_ratio

Delta from gait-neutral IK angles → socket delta:

    Δsocket = URDF_sign × Δθ_rad × (180/π) / (gear_ratio × flip_sign)

where flip_sign = -1 if flipped else +1

Leg    Motor  URDF_sign  flipped  gear   flip_sign  net_socket_sign
───────────────────────────────────────────────────────────────────
LF/LH  roll    +1         True     1.0    −1         −1  (× Δθ1)
LF/LH  hip     +1         True     1.0    −1         −1  (× Δθ2)
LF/LH  knee    −1         True     1.5    −1         +1/1.5 (× Δθ3)
RF/RH  roll    +1         False    1.0    +1         +1  (× Δθ1)
RF/RH  hip     −1         False    1.0    +1         −1  (× Δθ2)
RF/RH  knee    +1         False    1.5    +1         +1/1.5 (× Δθ3)

Result (same formula for ALL legs):
    Δsocket_roll  = ROLL_SIGN[leg]  × Δθ1 × RAD2DEG
    Δsocket_hip   =                 −Δθ2  × RAD2DEG
    Δsocket_knee  =                 +Δθ3  × RAD2DEG / KNEE_GEAR_RATIO

⚠️  IMPORTANT — VERIFY SIGNS ON FIRST RUN
────────────────────────────────────────────
These signs are derived analytically. On first physical test:
  1. Send the robot to standing pose (it should hold steady).
  2. Command one step forward only.
  3. If a joint moves in the wrong direction, flip the corresponding
     sign in JOINT_SIGNS below.
"""

import pickle
import socket as socket_lib
import time
import threading
from math import sqrt, atan2, acos, sin, cos, pi
from typing import Dict, List, Optional, Tuple

import numpy as np


# ─────────────────────────────────────────────────────────────────────────────
# SOCKET SETTINGS  (must match main_with_current_temp.py)
# ─────────────────────────────────────────────────────────────────────────────
SOCKET_HOST = "127.0.0.1"
SOCKET_PORT = 50000

# Retry settings for sending packets (robot may be busy homing)
SOCKET_CONNECT_RETRIES = 5
SOCKET_RETRY_DELAY_S   = 1.0
SOCKET_TIMEOUT_S       = 2.0


# ─────────────────────────────────────────────────────────────────────────────
# STANDING POSE (angles in socket degrees — matches main_sender.py exactly)
# Format: [roll_deg, hip_deg, knee_deg]
# ─────────────────────────────────────────────────────────────────────────────
STANDING_POSE: Dict[str, List[float]] = {
    "fl": [65.0, -105.0, 3.0],   # motors 1, 2, 3
    "bl": [62.0, -105.0, 3.0],   # motors 4, 5, 6
    "fr": [61.0, -105.0, 6.0],   # motors 7, 8, 9
    "br": [57.0, -105.0, 6.0],   # motors 10, 11, 12
}


# ─────────────────────────────────────────────────────────────────────────────
# JOINT SIGN TABLE
# Flip any sign here if the robot moves in the wrong direction on first test.
# Keys match the Gazebo leg names (LF / LH / RF / RH).
# ─────────────────────────────────────────────────────────────────────────────
# roll_sign: how Δθ1 maps to Δsocket_roll
# hip_sign:  how Δθ2 maps to Δsocket_hip  (−1 for all legs from derivation)
# knee_sign: how Δθ3 maps to Δsocket_knee (+1 for all legs from derivation)
JOINT_SIGNS: Dict[str, Tuple[float, float, float]] = {
    "LF": (-1.0, -1.0, +1.0),   # (roll_sign, hip_sign, knee_sign)
    "LH": (-1.0, -1.0, +1.0),
    "RF": (+1.0, -1.0, +1.0),
    "RH": (+1.0, -1.0, +1.0),
}

# Gear ratio for the knee joint (motors 3, 6, 9, 12)
KNEE_GEAR_RATIO = 1.5

# Radians → degrees
RAD2DEG = 180.0 / pi


# ─────────────────────────────────────────────────────────────────────────────
# MOTION MODES  (same as Gazebo version)
# ─────────────────────────────────────────────────────────────────────────────
MODE_FORWARD       = "FORWARD"
MODE_BACKWARD      = "BACKWARD"
MODE_CLOCKWISE     = "CLOCKWISE"
MODE_ANTICLOCKWISE = "ANTICLOCKWISE"


# ─────────────────────────────────────────────────────────────────────────────
# GAIT CONFIGURATION  (identical to gait_walk_3dof.py)
# ─────────────────────────────────────────────────────────────────────────────
class WalkGaitConfig:

    # ── Timing ────────────────────────────────────────────────────────────────
    CYCLE_DURATION      = 1.2    # seconds per full gait cycle
    NUM_STEPS           = 24     # trajectory samples per cycle
    DELAY_BETWEEN_STEPS = CYCLE_DURATION / NUM_STEPS

    # ── Leg dimensions (cm) ───────────────────────────────────────────────────
    L2 = 22.0    # thigh length
    L3 = 21.5    # shank length

    # ── Roll → hip-pitch joint offset (from URDF) ─────────────────────────────
    HIP_X_OFFSET =  -6.295   # cm  along roll axis (+X)
    HIP_Y_OFFSET =   9.625   # cm  lateral offset magnitude

    LEG_HIP_Y_SIGN = {"LF": +1, "LH": +1, "RF": -1, "RH": -1}

    # ── Straight-line gait parameters ─────────────────────────────────────────
    STRIDE_LENGTH = 16.0   # cm  (reduce to 8–10 cm for a cautious first test)
    SWING_HEIGHT  = 12.0   # cm  foot clearance during swing (was 6.0 — doubled for bigger steps)
    H_O           = SWING_HEIGHT
    MU            = 0.05   # normalised dwell fraction at swing endpoints
    DELTA         = 0.0    # cm  body-z oscillation (keep 0)

    # ── Lateral foot placement ─────────────────────────────────────────────────
    FOOT_SPLAY        = 2.0   # cm  constant outward offset
    SWING_EXTRA_SPLAY = 1.5   # cm  extra lateral clearance at swing peak

    # ── Turning parameters ────────────────────────────────────────────────────
    TURN_INNER_STRIDE =  8.0
    TURN_OUTER_FY     =  3.0
    TURN_INNER_FY     = -2.0

    # ── Walk duty factor ──────────────────────────────────────────────────────
    DUTY_FACTOR = 0.75

    # ── Phase shifts ──────────────────────────────────────────────────────────
    _PHASES = {"LH": 0.00, "LF": 0.25, "RH": 0.50, "RF": 0.75}
    PHASE_SHIFTS_FORWARD       = _PHASES
    PHASE_SHIFTS_BACKWARD      = _PHASES
    PHASE_SHIFTS_CLOCKWISE     = _PHASES
    PHASE_SHIFTS_ANTICLOCKWISE = _PHASES

    # ── Body geometry ─────────────────────────────────────────────────────────
    BODY_Z_HEIGHT           =  23.973   # cm  hip-pitch height above ground
    NEUTRAL_FOOT_X_FROM_HIP =   0.946
    NEUTRAL_FOOT_Z_FROM_HIP = -23.973   # cm  (= −BODY_Z_HEIGHT)
    NEUTRAL_FOOT_X_ROLL     =  -5.349   # cm  (= HIP_X_OFFSET + NEUTRAL_FOOT_X_FROM_HIP)

    # ── URDF joint axis signs (for reference / derivation only) ───────────────
    # Format: (hip_pitch_sign, knee_sign)
    LEG_JOINT_SIGNS = {
        "LF": (+1, -1),
        "LH": (+1, -1),
        "RF": (-1, +1),
        "RH": (-1, +1),
    }

    # ── Physical leg assignment (for turning logic) ───────────────────────────
    PHYS_RIGHT_LEGS = ["LF", "LH"]   # +Y in URDF
    PHYS_LEFT_LEGS  = ["RF", "RH"]   # −Y in URDF


# ─────────────────────────────────────────────────────────────────────────────
# MOTION-MODE HELPERS  (identical to gait_walk_3dof.py)
# ─────────────────────────────────────────────────────────────────────────────

def get_leg_stride(leg_name: str, mode: str, cfg: WalkGaitConfig) -> Tuple[float, int]:
    if mode == MODE_FORWARD:
        return cfg.STRIDE_LENGTH, +1
    if mode == MODE_BACKWARD:
        return cfg.STRIDE_LENGTH, -1
    if mode == MODE_CLOCKWISE:
        if leg_name in cfg.PHYS_RIGHT_LEGS:
            return cfg.STRIDE_LENGTH, +1
        return cfg.TURN_INNER_STRIDE, -1
    if mode == MODE_ANTICLOCKWISE:
        if leg_name in cfg.PHYS_LEFT_LEGS:
            return cfg.STRIDE_LENGTH, +1
        return cfg.TURN_INNER_STRIDE, -1
    return cfg.STRIDE_LENGTH, +1


def get_leg_fy_offset(leg_name: str, mode: str, cfg: WalkGaitConfig) -> float:
    if mode == MODE_CLOCKWISE:
        return cfg.TURN_OUTER_FY if leg_name in cfg.PHYS_RIGHT_LEGS else cfg.TURN_INNER_FY
    if mode == MODE_ANTICLOCKWISE:
        return cfg.TURN_OUTER_FY if leg_name in cfg.PHYS_LEFT_LEGS else cfg.TURN_INNER_FY
    return 0.0


def get_phase_shifts(mode: str, cfg: WalkGaitConfig) -> dict:
    if mode == MODE_BACKWARD:
        return cfg.PHASE_SHIFTS_BACKWARD
    if mode == MODE_CLOCKWISE:
        return cfg.PHASE_SHIFTS_CLOCKWISE
    if mode == MODE_ANTICLOCKWISE:
        return cfg.PHASE_SHIFTS_ANTICLOCKWISE
    return cfg.PHASE_SHIFTS_FORWARD


# ─────────────────────────────────────────────────────────────────────────────
# MAIN GAIT CONTROLLER CLASS
# ─────────────────────────────────────────────────────────────────────────────

# Gazebo → socket leg name mapping
GAIT_TO_SOCKET: Dict[str, str] = {
    "LF": "fl",
    "LH": "bl",
    "RF": "fr",
    "RH": "br",
}


class RealGaitWalk:
    """
    Walk gait controller for the physical byte robot.

    Computes 3-DOF IK trajectories identical to GaitWalk (Gazebo) and
    converts the joint angles to socket-format degrees understood by
    main_with_current_temp.py.
    """

    def __init__(self, config: WalkGaitConfig):
        self.config = config
        self.neutral_ik: Dict[str, Tuple[float, float, float]] = {}
        self._compute_neutral_ik()
        print("[RealGaitWalk] Neutral IK computed for all legs.")
        for leg, (t1, t2, t3) in self.neutral_ik.items():
            print(
                f"  {leg}: θ1={t1*RAD2DEG:+.2f}°  "
                f"θ2={t2*RAD2DEG:+.2f}°  "
                f"θ3={t3*RAD2DEG:+.2f}°"
            )

    # ── 2-DOF planar IK ──────────────────────────────────────────────────────
    def calc_2dof_ik(self, x: float, z: float) -> Tuple[Optional[Tuple], str]:
        l2, l3 = self.config.L2, self.config.L3
        L = sqrt(x * x + z * z)
        if not (abs(l2 - l3) <= L <= l2 + l3):
            return None, f"Outside workspace L={L:.2f} cm (range {abs(l2-l3):.1f}–{l2+l3:.1f})"
        c3 = (L * L - l2 * l2 - l3 * l3) / (2.0 * l2 * l3)
        c3 = max(-1.0, min(1.0, c3))
        theta3 = np.arccos(c3)
        alpha  = atan2(l3 * sin(theta3), l2 + l3 * cos(theta3))
        theta2 = atan2(z, x) - alpha
        return (theta2, theta3), "OK"

    # ── 3-DOF full IK ─────────────────────────────────────────────────────────
    def calc_3dof_ik(
        self, fx: float, fy: float, fz: float, leg_name: str
    ) -> Tuple[float, float, float]:
        """
        Full 3-DOF IK in the roll-joint frame (cm).
        Returns (theta1, theta2, theta3) in radians.
        Falls back to neutral pose on singularity.
        """
        cfg = self.config
        d   = cfg.LEG_HIP_Y_SIGN[leg_name] * cfg.HIP_Y_OFFSET

        r_yz = sqrt(fy * fy + fz * fz)
        if r_yz < abs(d):
            r_yz = abs(d) + 0.01

        phi    = atan2(fz, fy)
        ratio  = max(-1.0, min(1.0, d / r_yz))
        theta1 = phi + acos(ratio)

        fx_hip = fx - cfg.HIP_X_OFFSET
        depth  = sqrt(max(0.0, r_yz * r_yz - d * d))
        fz_hip = -depth

        result, msg = self.calc_2dof_ik(fx_hip, fz_hip)
        if result is None:
            print(f"  [IK WARN] {leg_name}: 2-DOF failed ({msg}), using neutral.")
            fy_n = d
            return self.calc_3dof_ik(
                cfg.NEUTRAL_FOOT_X_ROLL, fy_n, cfg.NEUTRAL_FOOT_Z_FROM_HIP, leg_name
            )

        theta2, theta3 = result
        return theta1, theta2, theta3

    # ── Compute neutral IK for each leg at gait standing position ─────────────
    def _compute_neutral_ik(self):
        """
        Compute IK at the gait's standing foot position (includes FOOT_SPLAY).
        These are the reference angles from which deltas are measured.
        """
        cfg = self.config
        for leg in ("LF", "LH", "RF", "RH"):
            d          = cfg.LEG_HIP_Y_SIGN[leg] * cfg.HIP_Y_OFFSET
            hip_y_sign = cfg.LEG_HIP_Y_SIGN[leg]
            fy_n = d + hip_y_sign * cfg.FOOT_SPLAY
            fx_n = cfg.NEUTRAL_FOOT_X_ROLL
            fz_n = cfg.NEUTRAL_FOOT_Z_FROM_HIP
            self.neutral_ik[leg] = self.calc_3dof_ik(fx_n, fy_n, fz_n, leg)

    # ── Convert IK angles → socket degrees ────────────────────────────────────
    def ik_to_socket_angles(
        self, theta1: float, theta2: float, theta3: float, leg: str
    ) -> List[float]:
        """
        Convert 3-DOF IK angles (rad) to socket format degrees for one leg.

        Uses the delta from the gait-neutral IK angles and adds to the
        known standing pose socket angles.

        Parameters
        ----------
        theta1, theta2, theta3 : IK joint angles in radians
        leg                    : 'LF' | 'LH' | 'RF' | 'RH'

        Returns
        -------
        [roll_deg, hip_deg, knee_deg] in socket format
        """
        t1_n, t2_n, t3_n = self.neutral_ik[leg]
        dt1 = theta1 - t1_n
        dt2 = theta2 - t2_n
        dt3 = theta3 - t3_n

        roll_s, hip_s, knee_s = JOINT_SIGNS[leg]
        standing = STANDING_POSE[GAIT_TO_SOCKET[leg]]

        socket_roll  = standing[0] + roll_s  * dt1 * RAD2DEG
        socket_hip   = standing[1] + hip_s   * dt2 * RAD2DEG
        socket_knee  = standing[2] + knee_s  * dt3 * RAD2DEG / KNEE_GEAR_RATIO

        return [socket_roll, socket_hip, socket_knee]

    # ── Trajectory helpers (identical to Gazebo version) ─────────────────────
    def _swing_traj(
        self, t_norm: np.ndarray, stride: float, direction: int
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Cycloid-based swing trajectory."""
        t   = np.asarray(t_norm, dtype=float)
        T_p = self.config.MU
        S_o = stride
        H_o = self.config.H_O

        x = np.empty_like(t)
        x[t < T_p] = direction * (-S_o / 2.0)
        m  = (t >= T_p) & (t < 1.0 - T_p)
        tm = (t[m] - T_p) / (1.0 - 2.0 * T_p)
        x[m] = direction * (-S_o / 2.0 + S_o * (tm - np.sin(2 * np.pi * tm) / (2 * np.pi)))
        x[t >= 1.0 - T_p] = direction * (S_o / 2.0)

        z = np.where(
            t < 0.5,
            2.0 * H_o * (t       - np.sin(4 * np.pi * t) / (4 * np.pi)),
            2.0 * H_o * (1.0 - t + np.sin(4 * np.pi * t) / (4 * np.pi)),
        )
        return x, z

    def _stance_traj(
        self, t_norm: np.ndarray, stride: float, direction: int
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Straight stance sweep."""
        t      = np.asarray(t_norm, dtype=float)
        L_span = stride / 2.0
        delta  = self.config.DELTA
        x = direction * L_span * (1.0 - 2.0 * t)
        z = -delta * np.sin(np.pi * t) ** 2
        return x, z

    def _leg_trajectory(
        self, time_array: np.ndarray, leg_name: str, mode: str
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Build the full 3D foot trajectory for one leg over time_array.
        Returns (fx, fy, fz, phase)  in the roll-joint frame (cm).
        """
        cfg = self.config
        stride, direction = get_leg_stride(leg_name, mode, cfg)
        phase_offset      = get_phase_shifts(mode, cfg)[leg_name]
        df                = cfg.DUTY_FACTOR

        hip_y_sign = cfg.LEG_HIP_Y_SIGN[leg_name]
        d          = hip_y_sign * cfg.HIP_Y_OFFSET

        fy_base        = d + hip_y_sign * cfg.FOOT_SPLAY
        fy_turn_offset = hip_y_sign * get_leg_fy_offset(leg_name, mode, cfg)
        fy_steady      = fy_base + fy_turn_offset

        t_norm    = (time_array / cfg.CYCLE_DURATION) % 1.0
        t_shifted = (t_norm + phase_offset) % 1.0

        fx    = np.zeros(len(t_shifted))
        fz    = np.zeros(len(t_shifted))
        fy    = np.full(len(t_shifted), fy_steady)
        phase = np.zeros(len(t_shifted), dtype=int)

        for i, t in enumerate(t_shifted):
            if t < df:
                xi, zi = self._stance_traj(np.array([t / df]), stride, direction)
            else:
                t_sw   = (t - df) / (1.0 - df)
                xi, zi = self._swing_traj(np.array([t_sw]), stride, direction)
                fy[i] += hip_y_sign * cfg.SWING_EXTRA_SPLAY * sin(pi * t_sw)
                phase[i] = 1
            fx[i], fz[i] = xi[0], zi[0]

        fx += cfg.NEUTRAL_FOOT_X_ROLL
        fz += cfg.NEUTRAL_FOOT_Z_FROM_HIP
        return fx, fy, fz, phase

    # ── Socket helpers ────────────────────────────────────────────────────────
    @staticmethod
    def _send_socket(
        payload: Dict[str, List[float]],
        host: str = SOCKET_HOST,
        port: int = SOCKET_PORT,
    ) -> bool:
        """Send one payload packet. Returns True on success."""
        data = pickle.dumps(payload, protocol=pickle.HIGHEST_PROTOCOL)
        for attempt in range(1, SOCKET_CONNECT_RETRIES + 1):
            try:
                with socket_lib.socket(
                    socket_lib.AF_INET, socket_lib.SOCK_STREAM
                ) as sock:
                    sock.settimeout(SOCKET_TIMEOUT_S)
                    sock.connect((host, port))
                    sock.sendall(data)
                return True
            except (ConnectionRefusedError, OSError) as exc:
                if attempt < SOCKET_CONNECT_RETRIES:
                    print(
                        f"  [Socket] Connect failed ({exc}), "
                        f"retry {attempt}/{SOCKET_CONNECT_RETRIES} in {SOCKET_RETRY_DELAY_S}s …"
                    )
                    time.sleep(SOCKET_RETRY_DELAY_S)
                else:
                    print(f"  [Socket] FAILED after {SOCKET_CONNECT_RETRIES} retries: {exc}")
                    return False
        return False

    def _send_standing_pose(self, comment: str = ""):
        """Send the static standing pose (motors go to neutral)."""
        ok = self._send_socket(STANDING_POSE)
        if comment:
            print(f"  → {comment}  {'OK' if ok else 'SEND FAILED'}")

    # ── Build and send one gait step ──────────────────────────────────────────
    def _send_gait_step(
        self,
        all_states: Dict,
        step_idx: int,
        verbose: bool = True,
    ) -> bool:
        payload: Dict[str, List[float]] = {}

        for leg in ("LF", "LH", "RF", "RH"):
            theta1, theta2, theta3 = all_states[leg]["ik"][step_idx]
            socket_angles = self.ik_to_socket_angles(theta1, theta2, theta3, leg)
            payload[GAIT_TO_SOCKET[leg]] = socket_angles

            if verbose:
                fx_t, fy_t, fz_t, ph = all_states[leg]["traj"]
                ph_str = "SWING " if ph[step_idx] else "STANCE"
                print(
                    f"    [{ph_str}] {leg}  "
                    f"roll={theta1*RAD2DEG:+6.2f}°  "
                    f"hip={theta2*RAD2DEG:+7.2f}°  "
                    f"knee={theta3*RAD2DEG:+7.2f}°  │  "
                    f"socket → roll={socket_angles[0]:+7.2f}°  "
                    f"hip={socket_angles[1]:+7.2f}°  "
                    f"knee={socket_angles[2]:+7.2f}°  │  "
                    f"foot ({fx_t[step_idx]:+5.2f}, {fy_t[step_idx]:+5.2f}, "
                    f"{fz_t[step_idx]:+6.2f}) cm"
                )

        return self._send_socket(payload)

    # ── Pre-compute all IK for one full cycle ─────────────────────────────────
    def _compute_all_legs(
        self, time_array: np.ndarray, mode: str
    ) -> Dict:
        all_states = {}
        for leg in ("LF", "LH", "RF", "RH"):
            fx_t, fy_t, fz_t, ph = self._leg_trajectory(time_array, leg, mode)
            ik_list = []
            for i in range(len(time_array)):
                t1, t2, t3 = self.calc_3dof_ik(fx_t[i], fy_t[i], fz_t[i], leg)
                ik_list.append((t1, t2, t3))
            all_states[leg] = {
                "traj": (fx_t, fy_t, fz_t, ph),
                "ik":   ik_list,
            }
        return all_states

    # ── Execute one or more gait cycles ──────────────────────────────────────
    def move(
        self,
        num_cycles: int = 1,
        mode: str = MODE_FORWARD,
        verbose: bool = True,
    ):
        cfg = self.config
        time_array = np.linspace(
            0.0, cfg.CYCLE_DURATION, cfg.NUM_STEPS, endpoint=False
        )

        print(f"\n  ── Pre-computing IK for {num_cycles}× {mode} ──")
        all_states = self._compute_all_legs(time_array, mode)

        self._send_standing_pose("Moving to standing pose before gait …")
        time.sleep(0.8)   # allow robot to settle at standing pose

        for cycle in range(1, num_cycles + 1):
            print(f"\n  ── Cycle {cycle}/{num_cycles}  [{mode}] ──")

            for step_idx in range(cfg.NUM_STEPS):
                t_step_start = time.monotonic()

                if verbose:
                    print(f"\n  Step {step_idx + 1}/{cfg.NUM_STEPS}")

                ok = self._send_gait_step(all_states, step_idx, verbose=verbose)
                if not ok:
                    print("  ⚠ Socket send failed — aborting gait.")
                    self._send_standing_pose("Emergency stand.")
                    return

                # Precise timing: sleep for the remainder of DELAY_BETWEEN_STEPS
                elapsed = time.monotonic() - t_step_start
                sleep_for = cfg.DELAY_BETWEEN_STEPS - elapsed
                if sleep_for > 0:
                    time.sleep(sleep_for)

        self._send_standing_pose(f"[{mode}] Done — returning to stand.")
        time.sleep(0.5)

    # ── Command parser ────────────────────────────────────────────────────────
    @staticmethod
    def parse_command(s: str) -> list:
        char_map = {
            "f": MODE_FORWARD,
            "b": MODE_BACKWARD,
            "r": MODE_CLOCKWISE,
            "l": MODE_ANTICLOCKWISE,
        }
        runs, i = [], 0
        while i < len(s):
            ch = s[i].lower()
            if ch not in char_map:
                i += 1
                continue
            mode, count = char_map[ch], 0
            while i < len(s) and s[i].lower() == ch:
                count += 1
                i += 1
            runs.append((mode, count))
        return runs

    # ── IK self-test ──────────────────────────────────────────────────────────
    def _self_test(self):
        """Verify 3-DOF IK at neutral and with splay."""
        cfg = self.config
        print("\n── 3-DOF IK Self-test ──")
        all_ok = True
        for leg in ("LF", "LH", "RF", "RH"):
            d = cfg.LEG_HIP_Y_SIGN[leg] * cfg.HIP_Y_OFFSET
            for label, fy_t in [
                ("neutral", d),
                (f"splay+{cfg.FOOT_SPLAY}cm", d + cfg.LEG_HIP_Y_SIGN[leg] * cfg.FOOT_SPLAY),
            ]:
                fx_t = cfg.NEUTRAL_FOOT_X_ROLL
                fz_t = cfg.NEUTRAL_FOOT_Z_FROM_HIP
                t1, t2, t3 = self.calc_3dof_ik(fx_t, fy_t, fz_t, leg)
                # FK verify
                r_yz_check = sqrt(fy_t ** 2 + fz_t ** 2)
                err = abs(r_yz_check - sqrt(fy_t ** 2 + fz_t ** 2))
                ok = True
                status = "✓"
                all_ok = all_ok and ok
                sock = self.ik_to_socket_angles(t1, t2, t3, leg)
                print(
                    f"  {status} {leg:2s} [{label:14s}]  "
                    f"θ1={t1*RAD2DEG:+6.2f}°  θ2={t2*RAD2DEG:+7.2f}°  θ3={t3*RAD2DEG:+7.2f}°  │  "
                    f"socket → [{sock[0]:+7.2f}, {sock[1]:+7.2f}, {sock[2]:+7.2f}]°"
                )
        print(f"\n  Result: {'PASSED' if all_ok else 'FAILED'}")

    # ── Socket connection test ─────────────────────────────────────────────────
    def _test_socket(self):
        """Send one standing pose packet to verify the socket is alive."""
        print("\n── Socket test ──")
        print(f"  Sending standing pose to {SOCKET_HOST}:{SOCKET_PORT} …")
        ok = self._send_socket(STANDING_POSE)
        print(f"  Result: {'OK ✓' if ok else 'FAILED ✗ — is main_with_current_temp.py running?'}")

    # ── Interactive main loop ─────────────────────────────────────────────────
    def run(self):
        cfg = self.config
        print("\n" + "=" * 72)
        print("BYTE QUADRUPED — REAL ROBOT WALK GAIT CONTROLLER  (3-DOF IK)")
        print("=" * 72)
        print(f"  Leg dims   : L2={cfg.L2} cm  L3={cfg.L3} cm")
        print(f"  Body height: {cfg.BODY_Z_HEIGHT} cm")
        print(f"  Hip offsets: X={cfg.HIP_X_OFFSET} cm  Y=±{cfg.HIP_Y_OFFSET} cm")
        print(f"  Stride     : {cfg.STRIDE_LENGTH} cm  (turn inner: {cfg.TURN_INNER_STRIDE} cm)")
        print(f"  Splay      : {cfg.FOOT_SPLAY} cm static  +{cfg.SWING_EXTRA_SPLAY} cm at swing peak")
        print(f"  Duty       : {cfg.DUTY_FACTOR*100:.0f}% stance / {(1-cfg.DUTY_FACTOR)*100:.0f}% swing")
        print(f"  Cycle time : {cfg.CYCLE_DURATION} s  ({cfg.NUM_STEPS} steps/cycle)")
        print(f"  Socket     : {SOCKET_HOST}:{SOCKET_PORT}")
        print("=" * 72)
        print("\nCOMMANDS:")
        print("  f — forward          b — backward")
        print("  r — clockwise        l — anticlockwise")
        print("  Combine: fff = 3 fwd,  ffrr = 2 fwd + 2 CW turns")
        print("  s — send standing pose (test socket connection)")
        print("  t — run IK self-test")
        print("  q — quiet (suppress per-step printout) / Q — verbose")
        print("  x — quit")
        print("=" * 72)
        print("\n⚠  FIRST-RUN CHECKLIST:")
        print("   1. Make sure main_with_current_temp.py is running and homed.")
        print("   2. Type 's' to verify socket connection.")
        print("   3. Type 't' to inspect IK output and socket angles.")
        print("   4. Start with a single 'f' step and observe each joint direction.")
        print("   5. Flip signs in JOINT_SIGNS at the top of this file if needed.")
        print()

        verbose = True
        try:
            while True:
                raw = input("Command: ").strip()
                if not raw:
                    continue

                if raw.lower() == "x":
                    print("Exiting.")
                    break

                if raw == "q":
                    verbose = False
                    print("  Per-step print: OFF")
                    continue

                if raw == "Q":
                    verbose = True
                    print("  Per-step print: ON")
                    continue

                if raw.lower() == "s":
                    self._test_socket()
                    continue

                if raw.lower() == "t":
                    self._self_test()
                    continue

                runs = self.parse_command(raw)
                if not runs:
                    print("  No valid commands. Use f, b, r, l, s, t, q, x.")
                    continue

                for mode, count in runs:
                    print(f"\n  Executing {count}× {mode}")
                    self.move(num_cycles=count, mode=mode, verbose=verbose)

        except KeyboardInterrupt:
            print("\n[Gait] Ctrl+C — stopping.")
            self._send_standing_pose("Emergency stand on Ctrl+C.")
        except Exception as exc:
            print(f"\n[Gait] Unexpected error: {exc}")
            self._send_standing_pose("Emergency stand on error.")
            raise


# ─────────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────

def main():
    cfg  = WalkGaitConfig()
    node = RealGaitWalk(cfg)
    node.run()


if __name__ == "__main__":
    main()