#!/usr/bin/env python3
"""
fr_leg_calibrate.py  —  FR Leg Cycloidal Trajectory Calibration Tool
─────────────────────────────────────────────────────────────────────
Motors: 7 (hip) / 8 (upper) / 9 (knee)  — CAN1

STARTUP ORDER (run in this exact sequence)
──────────────────────────────────────────
  1. main_with_ik.py     — starts socket server + IK pipeline
  2. main_sender.py      — sends initial standing pose to all 4 legs
  3. fr_leg_calibrate.py — this script (sends x,y,z coords to the socket)

HOW TO USE
──────────
  Press  f  -> run one step cycle with current parameters
  Press  h  -> show history table of all cycles so far
  Press  r  -> reset parameters back to defaults
  Press  q  -> quit and save session log

WHAT HAPPENS AFTER EACH CYCLE
───────────────────────────────
  - You see exact numbers: peak lift, peak press, timing, speed
  - Auto-analysis flags any warnings
  - You tell the script what you observed (drag / slip / jerk / ok)
  - You rate smoothness 1-5
  - Script suggests better values for next cycle
  - You accept suggestion, enter custom values, or keep current

HOW THE PIPELINE WORKS
───────────────────────
  This script sends (x, y, z) foot coordinates in cm.
  main_with_ik.py receives them and runs IK internally -> motor angles -> CAN -> motors.
  You never deal with angles here. Only foot positions in cm.

SESSION LOG
───────────
  Saved to: fr_calibration_log.json after every single cycle.
  Open it after a session to review all parameters tried.
"""

import json
import math
import os
import pickle
import socket
import sys
import termios
import time
import tty
from datetime import datetime


# ===================================================================
#  ANSI COLOURS
# ===================================================================

ANSI = {
    "reset":  "\033[0m",
    "bold":   "\033[1m",
    "green":  "\033[92m",
    "yellow": "\033[93m",
    "red":    "\033[91m",
    "cyan":   "\033[96m",
    "white":  "\033[97m",
    "amber":  "\033[38;5;214m",
    "gray":   "\033[90m",
}

def col(name, text):
    return ANSI[name] + str(text) + ANSI["reset"]

def bold(text):
    return ANSI["bold"] + str(text) + ANSI["reset"]


# ===================================================================
#  SOCKET  (connects to main_with_ik.py)
# ===================================================================

HOST = "127.0.0.1"
PORT = 50000


# ===================================================================
#  ROBOT GEOMETRY  (physical constants — do not change)
# ===================================================================

# FR leg nominal standing foot position (x, y, z) in cm
# Source: trot_gait.py LEG_NOM — this is what main_with_ik.py expects
X_NOM =  -9.094   # hip abduction offset — always fixed
Y_NOM =   30.0    # foot height at standing (bigger = lower)
Z_NOM =    9.0    # forward/back offset at rest

L2 = 22.0         # upper leg length cm
L3 = 21.5         # lower leg length cm
MAX_REACH = L2 + L3   # 43.5 cm absolute max foot distance from hip

# FL, BL, BR hold at their standing position while FR moves
# These match main_sender.py -> main_with_ik.py standing pose
# Source: trot_gait.py LEG_NOM
FL_STAND = [-9.094, 30.0, 9.0]
BL_STAND = [-9.094, 30.0, 9.0]
BR_STAND = [-9.094, 30.0, 9.0]


# ===================================================================
#  GAIT PARAMETERS  (what you are calibrating)
# ===================================================================

DEFAULTS = {
    "stride":           5.0,   # cm  — how far forward the foot travels per step
    "h_swing":          5.0,   # cm  — peak lift height during swing phase
    "h_stance":         2.0,   # cm  — how much foot presses into ground on stance
    "swing_duration":   0.25,  # sec — time for swing phase
    "stance_duration":  0.25,  # sec — time for stance phase
    "loop_hz":         50.0,   # Hz  — position update rate (leave this alone)
}

# Safe limits — warnings if you go outside these
BOUNDS = {
    "stride":          (1.0, 12.0),
    "h_swing":         (1.0, 10.0),
    "h_stance":        (0.0,  5.0),
    "swing_duration":  (0.1,  1.0),
    "stance_duration": (0.1,  1.0),
}


# ===================================================================
#  TRAJECTORY MATH
# ===================================================================

def _horiz(t):
    """
    Cycloidal horizontal profile. t goes 0->1, output goes 0->1.
    Makes the foot go slow-fast-slow instead of constant speed.
    This gives smooth acceleration and deceleration at each end of the step.
    """
    return t - math.sin(2 * math.pi * t) / (2 * math.pi)


def _vert(t):
    """
    Cycloidal vertical profile. t goes 0->1, output peaks at 2.0 when t=0.5.
    Both ends are zero — smooth liftoff and landing, no jerking.
    """
    return 1.0 - math.cos(2 * math.pi * t)


def build_trajectory(z_start, z_end, height, duration, loop_hz, lift_up=True):
    """
    Generate a list of (x, y, z) foot positions for one phase.

    lift_up=True  -> swing phase  (foot lifts up,     y gets smaller)
    lift_up=False -> stance phase (foot presses down,  y gets bigger)

    Peak vertical movement = height cm exactly.
    (_vert peaks at 2.0 and we divide by 2, so peak offset = height.)
    """
    n    = max(1, int(duration * loop_hz))
    sign = -1 if lift_up else +1
    pts  = []
    for i in range(n + 1):
        t = i / n
        z = z_start + (z_end - z_start) * _horiz(t)
        y = Y_NOM + sign * (height / 2.0) * _vert(t)
        pts.append((X_NOM, y, z))
    return pts


# ===================================================================
#  SOCKET SENDER
# ===================================================================

def send_fr(x, y, z):
    """
    Send a (x,y,z) foot target for the FR leg only.
    FL, BL, BR hold at their standing positions.
    main_with_ik.py receives this and does IK internally.
    Returns True on success, False on socket error.
    """
    payload = {
        "fl": [float(v) for v in FL_STAND],
        "bl": [float(v) for v in BL_STAND],
        "fr": [float(x), float(y), float(z)],
        "br": [float(v) for v in BR_STAND],
    }
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(0.5)
            s.connect((HOST, PORT))
            s.sendall(pickle.dumps(payload))
        return True
    except Exception as e:
        print(col("red", "\n  [Socket Error] " + str(e)), flush=True)
        return False


def move_to_nominal():
    """Move FR foot back to nominal standing position."""
    return send_fr(X_NOM, Y_NOM, Z_NOM)


# ===================================================================
#  CYCLE EXECUTION
# ===================================================================

def run_cycle(params, cycle_num):
    """
    Execute one full step: swing phase then stance phase.
    Returns a metrics dict of everything that happened.
    """
    stride    = params["stride"]
    h_swing   = params["h_swing"]
    h_stance  = params["h_stance"]
    swing_dur = params["swing_duration"]
    stan_dur  = params["stance_duration"]
    hz        = params["loop_hz"]
    dt        = 1.0 / hz

    # Foot starts at rest (z_back), swings forward to z_front, stances back
    z_back  = Z_NOM
    z_front = Z_NOM - stride     # negative z = forward

    swing_pts  = build_trajectory(z_back,  z_front, h_swing,  swing_dur, hz, lift_up=True)
    stance_pts = build_trajectory(z_front, z_back,  h_stance, stan_dur,  hz, lift_up=False)

    # Measure peak positions from the trajectory itself
    peak_lift_cm  = Y_NOM - min(pt[1] for pt in swing_pts)
    peak_press_cm = max(pt[1] for pt in stance_pts) - Y_NOM

    # ── SWING ────────────────────────────────────────────────────────
    line = (col("cyan", "[Cycle " + str(cycle_num) + "]") + "  " +
            col("green", "SWING ") + "  " +
            "z: " + str(round(z_back, 1)) + " -> " + str(round(z_front, 1)) + " cm  |  " +
            "lift " + str(round(peak_lift_cm, 1)) + " cm up  |  " +
            str(len(swing_pts)) + " pts @ " + str(int(hz)) + " Hz")
    print("\n  " + line, flush=True)

    t0 = time.time()
    for pt in swing_pts:
        send_fr(*pt)
        time.sleep(dt)
    t_swing = time.time() - t0

    # ── STANCE ───────────────────────────────────────────────────────
    line = (col("cyan", "[Cycle " + str(cycle_num) + "]") + "  " +
            col("amber", "STANCE") + "  " +
            "z: " + str(round(z_front, 1)) + " -> " + str(round(z_back, 1)) + " cm  |  " +
            "press " + str(round(peak_press_cm, 1)) + " cm down  |  " +
            str(len(stance_pts)) + " pts @ " + str(int(hz)) + " Hz")
    print("  " + line, flush=True)

    t0 = time.time()
    for pt in stance_pts:
        send_fr(*pt)
        time.sleep(dt)
    t_stance = time.time() - t0

    # Return to exact nominal position
    move_to_nominal()

    speed_cms = stride / t_stance if t_stance > 0 else 0.0

    return {
        "cycle":           cycle_num,
        "timestamp":       datetime.now().isoformat(),
        "params":          dict(params),
        "peak_lift_cm":    round(peak_lift_cm,  2),
        "peak_press_cm":   round(peak_press_cm, 2),
        "swing_pts":       len(swing_pts),
        "stance_pts":      len(stance_pts),
        "actual_swing_s":  round(t_swing,  3),
        "actual_stance_s": round(t_stance, 3),
        "total_time_s":    round(t_swing + t_stance, 3),
        "speed_cms":       round(speed_cms, 2),
        "z_front":         round(z_front, 3),
    }


# ===================================================================
#  CYCLE REPORT
# ===================================================================

def print_cycle_report(metrics):
    p = metrics["params"]
    print()
    print("  " + "-" * 58)
    print("  " + bold("CYCLE REPORT") + "  #" + str(metrics["cycle"]))
    print("  " + "-" * 58)

    print("\n  " + col("gray", "Parameters used:"))
    print("    Stride          : " + col("white", str(p["stride"])          + " cm"))
    print("    Swing height    : " + col("white", str(p["h_swing"])         + " cm"))
    print("    Stance press    : " + col("white", str(p["h_stance"])        + " cm"))
    print("    Swing duration  : " + col("white", str(p["swing_duration"])  + " s"))
    print("    Stance duration : " + col("white", str(p["stance_duration"]) + " s"))

    print("\n  " + col("gray", "Trajectory results:"))
    print("    Peak foot lift      : " + col("green", str(metrics["peak_lift_cm"])  + " cm"))
    print("    Peak foot press     : " + col("amber", str(metrics["peak_press_cm"]) + " cm"))
    print("    Swing pts sent      : " + str(metrics["swing_pts"]) +
          "  " + col("gray", "(" + str(metrics["actual_swing_s"]) + " s actual)"))
    print("    Stance pts sent     : " + str(metrics["stance_pts"]) +
          "  " + col("gray", "(" + str(metrics["actual_stance_s"]) + " s actual)"))
    print("    Total cycle time    : " + col("cyan", str(metrics["total_time_s"]) + " s"))
    print("    Effective speed     : " + col("cyan", str(metrics["speed_cms"]) + " cm/s"))

    # ── Auto-analysis ──
    print("\n  " + col("gray", "Auto-analysis:"))
    warnings = []

    if metrics["peak_lift_cm"] < 2.0:
        warnings.append(
            col("red", "WARNING") + " Very low lift (" + str(metrics["peak_lift_cm"]) +
            " cm) - foot may drag on ground during swing. Increase h_swing.")

    if metrics["peak_lift_cm"] > 8.0:
        warnings.append(
            col("yellow", "WARNING") + " High lift (" + str(metrics["peak_lift_cm"]) +
            " cm) - wastes energy. Check knee joint doesn't hit limit.")

    if p["stride"] > 9.0:
        warnings.append(
            col("yellow", "WARNING") + " Large stride (" + str(p["stride"]) +
            " cm) - verify IK didn't error. Foot target: z=" + str(metrics["z_front"]) + " cm.")

    if p["swing_duration"] < 0.15:
        warnings.append(
            col("yellow", "WARNING") + " Fast swing (" + str(p["swing_duration"]) +
            " s) - motors may not keep up with commanded trajectory.")

    if p["stance_duration"] < 0.15:
        warnings.append(
            col("yellow", "WARNING") + " Fast stance (" + str(p["stance_duration"]) +
            " s) - foot may slip if not enough time to push.")

    swing_drift  = abs(metrics["actual_swing_s"]  - p["swing_duration"])
    stance_drift = abs(metrics["actual_stance_s"] - p["stance_duration"])
    if swing_drift > 0.04 or stance_drift > 0.04:
        warnings.append(
            col("yellow", "NOTE") + " Timing drift: swing +" +
            str(round(swing_drift * 1000)) + " ms, stance +" +
            str(round(stance_drift * 1000)) + " ms. Normal socket overhead.")

    if warnings:
        for w in warnings:
            print("    " + w)
    else:
        print("    " + col("green", "OK") + " All values look reasonable.")

    print("  " + "-" * 58)


# ===================================================================
#  USER OBSERVATIONS
# ===================================================================

OBS_LABELS = {
    0: "looks good",
    1: "foot dragged",
    2: "robot slipped",
    3: "jerky motion",
    4: "foot too high",
    5: "stride too short",
    6: "stride too long",
}

def get_user_observations():
    print("\n  " + bold("What did you observe?"))
    print("  " + col("gray", "Type number(s) separated by space, then Enter:"))
    print("    1  = foot dragged during swing (not clearing ground)")
    print("    2  = robot slipped / foot slid on stance")
    print("    3  = motion was jerky / not smooth")
    print("    4  = foot went too high (looked excessive)")
    print("    5  = stride too short (robot barely moved)")
    print("    6  = stride too long (leg stretched too far)")
    print("    0  = everything looked good")

    raw = input("\n  " + col("cyan", "Observations") + " (e.g. 1 3): ").strip()
    selected = set()
    for token in raw.split():
        try:
            n = int(token)
            if n in OBS_LABELS:
                selected.add(n)
        except ValueError:
            pass
    if not selected:
        selected = {0}

    rating = None
    while rating is None:
        raw_r = input("  " + col("cyan", "Smoothness") + " (1=bad ... 5=great): ").strip()
        try:
            r = int(raw_r)
            if 1 <= r <= 5:
                rating = r
        except ValueError:
            pass

    notes = input("  " + col("cyan", "Notes") + " (optional, Enter to skip): ").strip()

    return {
        "observations": sorted(selected),
        "rating":       rating,
        "notes":        notes,
    }


# ===================================================================
#  PARAMETER SUGGESTIONS
# ===================================================================

def suggest_next_params(params, metrics, obs):
    """
    Suggest improved parameters based on observations and auto-analysis.
    Returns (suggested_dict, list_of_explanation_strings).
    """
    selected = set(obs["observations"])
    s    = dict(params)
    adjs = []

    if 1 in selected:    # dragged -> lift more
        s["h_swing"] = round(min(s["h_swing"] + 1.5, BOUNDS["h_swing"][1]), 1)
        adjs.append("h_swing +1.5 cm  (foot was dragging, needs more clearance)")

    if 4 in selected:    # too high -> lift less
        s["h_swing"] = round(max(s["h_swing"] - 1.5, BOUNDS["h_swing"][0]), 1)
        adjs.append("h_swing -1.5 cm  (was too high, wasting energy)")

    if 2 in selected:    # slip -> more stance time + more ground press
        s["stance_duration"] = round(min(s["stance_duration"] + 0.05, BOUNDS["stance_duration"][1]), 2)
        s["h_stance"]        = round(min(s["h_stance"] + 0.5, BOUNDS["h_stance"][1]), 1)
        adjs.append("stance_duration +0.05 s  (slip: more time on ground)")
        adjs.append("h_stance +0.5 cm  (slip: more downward press force)")

    if 3 in selected:    # jerky -> slow down both phases
        s["swing_duration"]  = round(min(s["swing_duration"]  + 0.05, BOUNDS["swing_duration"][1]),  2)
        s["stance_duration"] = round(min(s["stance_duration"] + 0.05, BOUNDS["stance_duration"][1]), 2)
        adjs.append("swing_duration +0.05 s  (jerky: more time for motors to follow)")
        adjs.append("stance_duration +0.05 s  (jerky: same)")

    if 5 in selected:    # too short -> bigger stride
        s["stride"] = round(min(s["stride"] + 1.5, BOUNDS["stride"][1]), 1)
        adjs.append("stride +1.5 cm  (was too short, robot barely moved)")

    if 6 in selected:    # too long -> smaller stride
        s["stride"] = round(max(s["stride"] - 1.5, BOUNDS["stride"][0]), 1)
        adjs.append("stride -1.5 cm  (was too long, leg over-extended)")

    # All good + high rating -> gently push performance further
    if 0 in selected and obs["rating"] >= 4 and not adjs:
        s["stride"]          = round(min(s["stride"] + 0.5, BOUNDS["stride"][1]), 1)
        s["swing_duration"]  = round(max(s["swing_duration"]  - 0.02, BOUNDS["swing_duration"][0]),  2)
        s["stance_duration"] = round(max(s["stance_duration"] - 0.02, BOUNDS["stance_duration"][0]), 2)
        adjs.append("stride +0.5 cm  (looked great, pushing further)")
        adjs.append("durations -0.02 s  (looked great, slightly faster)")

    # Clamp everything to safe bounds
    for key, (lo, hi) in BOUNDS.items():
        s[key] = round(max(lo, min(hi, s[key])), 3)

    return s, adjs


def prompt_new_params(current, suggested, adjustments):
    print("\n  " + bold("Suggested values for next cycle:"))

    if adjustments:
        for a in adjustments:
            print("    " + col("amber", "->") + "  " + a)
    else:
        print("    " + col("gray", "(no adjustments — parameters unchanged)"))

    keys   = ["stride", "h_swing", "h_stance", "swing_duration", "stance_duration"]
    labels = {
        "stride":           "Stride (cm)       ",
        "h_swing":          "Swing height (cm) ",
        "h_stance":         "Stance press (cm) ",
        "swing_duration":   "Swing time (s)    ",
        "stance_duration":  "Stance time (s)   ",
    }

    print()
    print("  Parameter              Current   Suggested")
    print("  " + "-" * 44)
    for k in keys:
        cur_str = "{:.3f}".format(current[k])
        sug_str = "{:.3f}".format(suggested[k])
        changed = current[k] != suggested[k]
        sug_display = col("amber", sug_str) if changed else col("gray", sug_str)
        print("  " + labels[k] + "  " + cur_str.rjust(7) + "     " + sug_display)

    print()
    print("    " + col("green", "y") + "  ->  use suggested values")
    print("    " + col("white", "n") + "  ->  keep current values")
    print("    " + col("cyan",  "c") + "  ->  enter custom values")

    choice = input("\n  " + col("cyan", "Choice") + " [y/n/c]: ").strip().lower()

    if choice == "y":
        print("  " + col("green", "OK") + " Using suggested values.")
        return dict(suggested)
    elif choice == "c":
        return prompt_custom_params(current)
    else:
        print("  " + col("gray", "->") + " Keeping current values.")
        return dict(current)


def prompt_custom_params(current):
    print("\n  " + bold("Enter custom values") +
          col("gray", "  (press Enter to keep current)"))
    new = dict(current)
    fields = [
        ("stride",          "Stride cm       ", 1.0,  12.0),
        ("h_swing",         "Swing height cm ", 1.0,  10.0),
        ("h_stance",        "Stance press cm ", 0.0,   5.0),
        ("swing_duration",  "Swing time s    ", 0.1,   1.0),
        ("stance_duration", "Stance time s   ", 0.1,   1.0),
    ]
    for key, label, lo, hi in fields:
        cur = current[key]
        raw = input("    " + label +
                    " [" + str(cur) + "]" +
                    col("gray", " (" + str(lo) + "-" + str(hi) + ")") +
                    ": ").strip()
        if not raw:
            continue
        try:
            val = float(raw)
            if lo <= val <= hi:
                new[key] = round(val, 3)
            else:
                print("      " + col("yellow", "Out of range, keeping " + str(cur)))
        except ValueError:
            print("      " + col("yellow", "Not a number, keeping " + str(cur)))
    return new


# ===================================================================
#  HISTORY TABLE
# ===================================================================

def print_history_table(history):
    if not history:
        print("  " + col("gray", "(no cycles yet)"))
        return

    print("\n  " + bold("Session history:"))
    print("  #    Stride  H_Swg  SwgT  StnT    Speed  Rating  Observations")
    print("  " + "-" * 68)

    for h in history:
        p       = h["params"]
        obs     = h.get("obs", {})
        obs_ids = obs.get("observations", [])
        rating  = obs.get("rating", "?")

        obs_str = ", ".join(OBS_LABELS.get(n, str(n)) for n in obs_ids if n != 0)
        if not obs_str:
            obs_str = "ok"

        if isinstance(rating, int) and rating >= 4:
            r_str = col("green",  str(rating))
        elif isinstance(rating, int) and rating == 3:
            r_str = col("yellow", str(rating))
        elif isinstance(rating, int):
            r_str = col("red",    str(rating))
        else:
            r_str = str(rating)

        print(
            "  " + str(h["cycle"]).rjust(3)                        + "  " +
            "{:.1f}".format(p["stride"]).rjust(6)                  + "  " +
            "{:.1f}".format(p["h_swing"]).rjust(5)                 + "  " +
            "{:.2f}".format(p["swing_duration"]).rjust(4)          + "  " +
            "{:.2f}".format(p["stance_duration"]).rjust(4)         + "  " +
            "{:.1f} c/s".format(h["speed_cms"]).rjust(8)           + "  " +
            r_str.rjust(6)                                         + "  " +
            col("gray", obs_str)
        )
    print()


# ===================================================================
#  SESSION LOG
# ===================================================================

LOG_PATH = "fr_calibration_log.json"

def save_log(history, params):
    data = {
        "session_start": history[0]["timestamp"] if history else datetime.now().isoformat(),
        "session_end":   datetime.now().isoformat(),
        "total_cycles":  len(history),
        "final_params":  params,
        "cycles":        history,
    }
    with open(LOG_PATH, "w") as f:
        json.dump(data, f, indent=2)


# ===================================================================
#  KEYBOARD
# ===================================================================

def _getch():
    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ===================================================================
#  HEADER
# ===================================================================

def print_header(params):
    os.system("clear")
    print()
    print("  " + col("amber", bold("=" * 58)))
    print("  " + col("amber", bold("  BYTE  -  FR LEG CYCLOIDAL CALIBRATION")))
    print("  " + col("amber", bold("  Motors 7 (hip) / 8 (upper) / 9 (knee)")))
    print("  " + col("amber", bold("=" * 58)))
    print()
    print("  " + col("gray", "Pipeline: this script -> main_with_ik.py -> IK -> motors"))
    print("  " + col("gray", "Sends (x,y,z) coords in cm. IK is done by main_with_ik.py."))
    print()
    print("  " + col("gray", "FR nominal standing position:"))
    print("    x = " + str(X_NOM) + " cm  (hip offset, fixed)")
    print("    y = " + str(Y_NOM) + " cm  (standing height — bigger = lower)")
    print("    z = " + str(Z_NOM) + " cm  (forward/back at rest)")
    print()
    print("  " + col("gray", "FL / BL / BR hold at:") +
          " (" + str(FL_STAND[0]) + ", " + str(FL_STAND[1]) + ", " + str(FL_STAND[2]) + ")")
    print()
    print("  " + col("gray", "Active parameters:"))
    print("    Stride           : " + col("white", str(params["stride"])           + " cm"))
    print("    Swing height     : " + col("white", str(params["h_swing"])          + " cm"))
    print("    Stance press     : " + col("white", str(params["h_stance"])         + " cm"))
    print("    Swing duration   : " + col("white", str(params["swing_duration"])   + " s"))
    print("    Stance duration  : " + col("white", str(params["stance_duration"])  + " s"))
    print("    Loop rate        : " + col("white", str(int(params["loop_hz"]))     + " Hz"))
    print()
    print("  " + col("amber", bold("Controls:")))
    print("    " + col("green", "f") + "  ->  run one step cycle")
    print("    " + col("white", "h") + "  ->  show session history table")
    print("    " + col("white", "r") + "  ->  reset to default parameters")
    print("    " + col("red",   "q") + "  ->  quit and save log")
    print("  " + col("amber", bold("=" * 58)))
    print()


# ===================================================================
#  MAIN LOOP
# ===================================================================

def main():
    params      = dict(DEFAULTS)
    history     = []
    cycle_count = 0

    print_header(params)

    # Connectivity check
    print("  Checking connection to main_with_ik.py socket...", flush=True)
    ok = move_to_nominal()
    if not ok:
        print()
        print(col("red", "  Cannot connect to socket at " + HOST + ":" + str(PORT)))
        print(col("red", "  Make sure you have run these in order first:"))
        print(col("red", "    1. main_with_ik.py    (must be running and ready)"))
        print(col("red", "    2. main_sender.py     (sets initial standing pose)"))
        print()
        return

    time.sleep(0.5)
    print("  " + col("green", "Connected. FR foot at nominal standing position."))
    print("  " + col("gray",  "FL / BL / BR holding at standing position."))
    print()
    print("  " + col("gray", "Press f to run the first calibration cycle..."))
    print()

    try:
        while True:
            key = _getch()

            # f — run a cycle
            if key == "f":
                cycle_count += 1
                print("\n  " + col("amber", bold("CYCLE " + str(cycle_count) + " — executing...")),
                      flush=True)

                metrics = run_cycle(params, cycle_count)
                print_cycle_report(metrics)

                obs = get_user_observations()
                metrics["obs"] = obs
                history.append(metrics)
                save_log(history, params)   # save after every cycle

                suggested, adjustments = suggest_next_params(params, metrics, obs)
                params = prompt_new_params(params, suggested, adjustments)

                print()
                print_header(params)
                print_history_table(history)
                print("  " + col("gray", "Press f for next cycle, h for history, q to quit..."))
                print()

            # h — history table
            elif key == "h":
                print_history_table(history)

            # r — reset to defaults
            elif key == "r":
                params = dict(DEFAULTS)
                print("\n  " + col("yellow", "Parameters reset to defaults."))
                print_header(params)

            # q — quit
            elif key in ("q", "\x03"):
                print("\n  " + col("amber", "Quitting..."), flush=True)
                break

    finally:
        print("  Returning FR foot to nominal standing position...", flush=True)
        move_to_nominal()

        if history:
            save_log(history, params)
            print("  " + col("green", "Log saved -> " + LOG_PATH))
            print("  " + col("gray",  "Total cycles: " + str(len(history))))

            rated = [h for h in history if isinstance(h.get("obs", {}).get("rating"), int)]
            if rated:
                best = max(rated, key=lambda h: h["obs"]["rating"])
                bp   = best["params"]
                print()
                print("  " + col("amber", bold("Best rated cycle:")) +
                      "  #" + str(best["cycle"]) +
                      "  rating=" + str(best["obs"]["rating"]))
                print("    stride=" + str(bp["stride"]) +
                      "  h_swing=" + str(bp["h_swing"]) +
                      "  swing_dur=" + str(bp["swing_duration"]) +
                      "  stance_dur=" + str(bp["stance_duration"]))

        print()
        print("  " + col("green", "Done. Goodbye."))
        print()


if __name__ == "__main__":
    main()
