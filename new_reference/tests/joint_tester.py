#!/usr/bin/env python3

# Note: Run this by this command: python3 -m tests.joint_tester
# This ensures the imports work correctly relative to other folders

import multiprocessing as mp
import pickle
import queue
import socket
import threading
import time
from typing import Any, Dict, Optional, Tuple


from lib.can_runtime import BusRuntime
from lib.homing_controller import ControlTuning
from config.robot_config import SOCKET_HOST, SOCKET_PORT
from main_with_ik import (
    CAN_CONFIG, 
    collect_motor_ids,
    runtime_for_motor_id,
    drain_latest_packet,
    limit_target_step,
)

LIVE_QUEUE_MAXSIZE = 1
MAX_LIVE_DEG_PER_S = 30.0  # Kept slow for safe testing

def validate_joint_payload(payload: Any) -> Tuple[Dict[int, float], Optional[float]]:
    """Validates payload dict of {motor_id: target_deg}"""
    if not isinstance(payload, dict):
        raise ValueError("payload must be a dict")

    commands: Dict[int, float] = {}
    for key, val in payload.items():
        if key == "speed": continue
        try:
            mid = int(key)
            if 1 <= mid <= 12:
                commands[mid] = float(val)
        except ValueError:
            pass

    speed_override = payload.get("speed", None)
    if speed_override is not None:
        speed_override = float(speed_override)

    return commands, speed_override

def socket_listener_process(dest_queue: mp.Queue, stop_event: mp.Event, port: int, host: str):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.settimeout(0.2)
    try:
        server.bind((host, port))
        server.listen(5)
        print(f"[Socket] Listening on {host}:{port} for raw direct angles", flush=True)

        while not stop_event.is_set():
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            with conn:
                conn.settimeout(0.2)
                data = bytearray()
                while not stop_event.is_set():
                    try:
                        chunk = conn.recv(4096)
                        if not chunk: break
                        data.extend(chunk)
                    except socket.timeout:
                        continue
            if not data: continue

            try:
                payload = pickle.loads(bytes(data))
                commands, speed = validate_joint_payload(payload)
                
                # Drain queue and put fresh command
                while True:
                    try: dest_queue.get_nowait()
                    except queue.Empty: break
                dest_queue.put_nowait((commands, speed))
            except Exception as exc:
                print(f"[Socket] Drop invalid payload: {exc}", flush=True)
    finally:
        server.close()

def run_raw_joint_control(rt0, rt1, tuning, stop_event, live_queue):
    # --- PHASE 1: BOOT AND ZERO IMMEDIATELY ---
    print("\n[PHASE 1] Waiting for CAN bus feedback to stabilize...")
    time.sleep(2.0)

    print("[PHASE 2] Zeroing all motors at their CURRENT physical positions...")
    live_cmd_deg = {}
    for motor_id in range(1, 13):
        runtime = runtime_for_motor_id(motor_id, rt0, rt1)
        runtime.zero_motor(motor_id, permanent=False)
        live_cmd_deg[motor_id] = 0.0

    time.sleep(1.0)
    print("\n✅ Motors zeroed. Ready for raw testing. Send commands via joint_sender.py")

    raw_targets = {i: 0.0 for i in range(1, 13)}
    current_speed_limit = MAX_LIVE_DEG_PER_S

    # --- PHASE 3: PURE RAW CONTROL LOOP ---
    while not stop_event.is_set():
        # Keep watchdogs happy
        rt0.refresh_watchdogs(tuning.feedback_timeout_s, tuning.bus_silence_timeout_s)
        rt1.refresh_watchdogs(tuning.feedback_timeout_s, tuning.bus_silence_timeout_s)
        if rt0.faulted: raise RuntimeError(rt0.get_fault_summary())
        if rt1.faulted: raise RuntimeError(rt1.get_fault_summary())

        item = drain_latest_packet(live_queue)
        if item is not None:
            commands, speed_override = item
            for mid, ang in commands.items():
                raw_targets[mid] = ang
            current_speed_limit = speed_override if speed_override is not None else MAX_LIVE_DEG_PER_S
            print(f"🎯 New RAW Targets applied: {commands}")

        for motor_id in range(1, 13):
            # NO gear ratio limits, NO flips. Just raw step limiting.
            live_cmd_deg[motor_id] = limit_target_step(
                current_cmd_deg=live_cmd_deg[motor_id],
                requested_deg=raw_targets[motor_id],
                max_deg_per_s=current_speed_limit,
                dt=tuning.loop_dt,
            )

            # Send directly to hardware without using motor_config
            runtime = runtime_for_motor_id(motor_id, rt0, rt1)
            runtime.send_position_deg(
                motor_id,
                live_cmd_deg[motor_id],
                kp=tuning.active_kp,
                kd=tuning.active_kd,
                torque=0.0
            )

        time.sleep(tuning.loop_dt)

def main():
    print("=== RAW BARE METAL JOINT TESTER ===")
    print("⚠️  WARNING: NO HOMING. NO GEAR RATIOS. NO INVERSIONS.")
    print("⚠️  Motors will be zeroed EXACTLY where they are right now.\n")
    
    stop_event = threading.Event()
    tuning = ControlTuning(
        active_kp=114.0, active_kd=1.2, hold_kp=54.0, hold_kd=0.9, homed_hold_kp=84.0, homed_hold_kd=1.2, loop_hz=100.0, min_move_time_s=1.2, seconds_per_deg=0.03, trigger_confirm_s=0.15, trigger_velocity_raw_max=4.0
    )

    rt0 = rt1 = live_socket_process = None
    try:
        rt0 = BusRuntime("can0", collect_motor_ids(CAN_CONFIG["can0"]), bitrate=1_000_000)
        rt1 = BusRuntime("can1", collect_motor_ids(CAN_CONFIG["can1"]), bitrate=1_000_000)

        live_queue = mp.Queue(maxsize=1)
        live_socket_stop = mp.Event()
        live_socket_process = mp.Process(
            target=socket_listener_process,
            args=(live_queue, live_socket_stop, SOCKET_PORT, SOCKET_HOST),
            daemon=True,
        )
        live_socket_process.start()

        run_raw_joint_control(rt0, rt1, tuning, stop_event, live_queue)

    except KeyboardInterrupt:
        stop_event.set()
    finally:
        if live_socket_process: live_socket_process.terminate()
        if rt0: rt0.disable_all(); rt0.stop()
        if rt1: rt1.disable_all(); rt1.stop()
        print("✅ Shutdown complete.")

if __name__ == "__main__":
    main()