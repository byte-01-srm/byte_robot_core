#!/usr/bin/env python3

# Note: Run this by this command: python3 -m tests.joint_sender
# This ensures the imports work correctly relative to config

import pickle
import socket
from config.robot_config import SOCKET_HOST, SOCKET_PORT

print("=== INTERACTIVE JOINT TESTER ===")
print("Type 'q' to quit at any time.\n")

while True:
    try:
        # 1. Get Motor ID
        mid_str = input("Enter Motor ID (1-12): ").strip()
        if mid_str.lower() == 'q': 
            break
        
        motor_id = int(mid_str)
        if not 1 <= motor_id <= 12:
            print("Invalid Motor ID. Must be between 1 and 12.")
            continue

        # 2. Get Target Angle
        angle_str = input(f"Enter target angle for Motor {motor_id} (degrees): ").strip()
        if angle_str.lower() == 'q': 
            break
            
        target_angle = float(angle_str)

        # 3. Create a payload with ONLY the target motor
        # The receiver will hold all other motors at their previous states
        payload = {
            motor_id: target_angle,
            "speed": 30.0  # Safe testing speed
        }

        # 4. Send the command
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((SOCKET_HOST, SOCKET_PORT))
            s.sendall(pickle.dumps(payload))
        
        print(f"✅ Sent: Motor {motor_id} moving to {target_angle}°\n")

    except ValueError:
        print("❌ Invalid input. Please enter numbers only.")
    except ConnectionRefusedError:
        print("❌ Connection refused. Is main_raw_tester.py running?")
    except KeyboardInterrupt:
        break

print("\nExiting sender.")