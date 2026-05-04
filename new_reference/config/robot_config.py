# robot_config.py


# Default safety speed limit — used when the sender does NOT include a "speed" key.
# tapping_gait.py sends "speed": 900.0 to override this for fast tap movements.
# test_sender.py and any other client that omits "speed" will use this value.
MAX_LIVE_DEG_PER_S = 120

# ─── Network Settings ────────────────────────────────────────────────────────
SOCKET_HOST = "10.176.243.34" #127.0.0.1
SOCKET_PORT = 50000

# ─── Leg Identifiers ─────────────────────────────────────────────────────────
LEG_ORDER = ("fl", "bl", "fr", "br")

# ─── Sitting Cartesian Coordinates (X, Y, Z in cm) ───────────────────────────
# Used by Inverse Kinematics and as the baseline for live socket tracking.
SIT_COORDS = {
    "fl": (-13.60, -9.09, -15.21),
    "bl": (-13.03, -9.09, -14.00),
    "fr": (-12.76, 9.09, -14.05),
    "br": (-14.46, 9.09, -16.71)
}

# ─── Physical Sitting Motor Angles (Degrees) ─────────────────────────────────
# The absolute hardware angles the motors must reach BEFORE zeroing.
# will not be transformed as per motor_config.json

SIT_TARGETS_DEG = {
    1: -85.0,  2: 114.0, 3: -9.0,    # fl - flipped and gear ratio applied
    4: -85.0,  5: 114.0, 6: -3.0,    # bl - flipped and gear ratio applied
    7: 85.0,  8: -114.0, 9: 6.0,    # fr
    10: 85.0, 11: -116.0, 12: 9.0   # br
}