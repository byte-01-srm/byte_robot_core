#!/usr/bin/env python3

import pickle
import socket
import time

# Note: Run this by this command: python3 -m tests.ik_sender
# This ensures the imports work correctly relative to config

from config.robot_config import SOCKET_HOST, SOCKET_PORT, LEG_ORDER

'''
# ── CHANGE THIS to test ──────────────────────────────────────────────────────
TARGET_LEG = "fr"               # which leg to move: fl, bl, fr, br  
# ────────────────────────────────────────────────────────────────────────────
'''

# how much to shift the leg's foot position in cm (relative to current pose coords)
dx = 0.0 # +x forward, -x backward
dy = 0.0 # +y right, -y left
dz = 0.0 # +z up, -z down  # make +14 to stand (from 0 position)


HOLD_SECONDS  = 3.0                         # how long to hold the position
transformed_coords = [dx, dy, dz]           # convert to robot's ik coordinate system (+x forward, +y right, +z up)
# transformed_coords = [dy, -dz, -dx]       # convert to robot's ik coordinate system (+x right, +y down, +z back) (-x left, -y up, -z forward)

# Dynamically create the payload for all legs using the central config
payload = {leg: list(transformed_coords) for leg in LEG_ORDER}

#payload[TARGET_LEG] = list(TARGET_COORDS)  # only move this one leg

# No "speed" key → main_with_ik.py uses its safe default MAX_LIVE_DEG_PER_S

print(f"All other legs hold sitting position.")

data = pickle.dumps(payload)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((SOCKET_HOST, SOCKET_PORT))
    s.sendall(data)

print(f"Sent. Holding for {HOLD_SECONDS}s... watch the leg.")
time.sleep(HOLD_SECONDS)
print("Done.")