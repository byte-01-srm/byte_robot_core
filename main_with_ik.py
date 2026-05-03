#!/usr/bin/env python3
# temperature_C for temperature
import sys
import json
import multiprocessing as mp
import pickle
import queue
import socket
import threading
import time
from multiprocessing.queues import Queue as MpQueue
from multiprocessing.synchronize import Event as MpEvent
from typing import Any, Dict, List, Optional, Tuple

from can_runtime import BusRuntime
from homing_controller import BusHomingController, ControlTuning, HomingMotorConfig
from inverse_kinematics import compute_ik_offsets, ik_to_motor_deg


LIVE_SOCKET_HOST = "10.196.200.34"#127.0.0.1
LIVE_SOCKET_PORT = 50000
LIVE_QUEUE_MAXSIZE = 1
MOTOR_CONFIG_PATH = "motor_config.json"


LEG_ORDER: Tuple[str, str, str, str] = ("fl", "bl", "fr", "br")
LegPayload = Dict[str, List[float]]
MotorCommandConfig = Dict[int, Dict[str, float | bool]]

LEG_TO_MOTOR_IDS: Dict[str, List[int]] = {
    "fl": [1,  2,  3],
    "bl": [4,  5,  6],
    "fr": [7,  8,  9],
    "br": [10, 11, 12],
}

# Default safety speed limit — used when the sender does NOT include a "speed" key.
# tapping_gait.py sends "speed": 900.0 to override this for fast tap movements.
# test_sender.py and any other client that omits "speed" will use this value.
MAX_LIVE_DEG_PER_S = 120


CURRENT_LOG_PATH = "/home/byte/ak60_motor_control/multiprocess_code/all_4_legs/cur_test/motor_currents.csv"
CURRENT_LOG_HZ = 5.0
CURRENT_LOG_DT = 1.0 / CURRENT_LOG_HZ

TEMP_LOG_PATH = "/home/byte/ak60_motor_control/multiprocess_code/all_4_legs/cur_test/motor_temps.csv"
TEMP_LOG_HZ = 5.0
TEMP_LOG_DT = 1.0 / TEMP_LOG_HZ

# ─── CLI ──────────────────────────────────────────────────────────────────────

def parse_calibration_flag() -> bool:
    if len(sys.argv) < 2:
        return False
    arg = sys.argv[1].strip().lower()
    if arg == "y":
        return True
    elif arg == "n":
        return False
    else:
        print(f"Unknown argument '{arg}'. Use 'y' or 'n'. Defaulting to False.")
        return False



CAN_CONFIG: Dict[str, Dict[str, List[HomingMotorConfig]]] = {
    "can0": {
        "phase1": [
            HomingMotorConfig(5, -60.0, -1.0, 4.5, 65.0),
            HomingMotorConfig(6, 10.0, 1.0, 3.5, -38.0),
            HomingMotorConfig(4, 60.0, 1.0, 4.5, -90.0),
        ],
        "phase2": [
            HomingMotorConfig(2, -60.0, -1.0, 4.5, 65.0),
            HomingMotorConfig(3, 10.0, 1.0, 3.5, -38.0),#4th val from 5 changed by dan
            HomingMotorConfig(1, 60.0, 1.0, 4.5, -85.0),#4th val from 4 changed by dan
        ],
    },
    "can1": {
        "phase1": [
            HomingMotorConfig(8, 60.0, 1.0, 4.5, -65.0),
            HomingMotorConfig(9, -10.0, -1.0, 3.5, 38.0),
            HomingMotorConfig(7, -60.0, -1.0, 4.5, 80.0),
        ],
        "phase2": [
            HomingMotorConfig(11, 60.0, 1.0, 4.5, -65.0),
            HomingMotorConfig(12, -10.0, -1.0, 4.0, 38.0),
            HomingMotorConfig(10, -60.0, -1.0, 4.5, 90.0),
        ],
    },
}


def collect_motor_ids(bus_cfg: Dict[str, List[HomingMotorConfig]]) -> List[int]:
    ids = []
    for phase_name in ("phase1", "phase2"):
        ids.extend(cfg.motor_id for cfg in bus_cfg[phase_name])
    return sorted(set(ids))


def hold_until_both_phase1_done(
    controller: BusHomingController,
    my_done_event: threading.Event,
    other_done_event: threading.Event,
    stop_event: threading.Event,
):
    my_done_event.set()

    while not stop_event.is_set():
        if other_done_event.is_set():
            return True
        controller.hold_all_once()
        time.sleep(controller.tuning.loop_dt)

    return False


def hold_until_takeover(
    controller: BusHomingController,
    takeover_event: threading.Event,
    stop_event: threading.Event,
):
    while not stop_event.is_set() and not takeover_event.is_set():
        controller.hold_all_once()
        time.sleep(controller.tuning.loop_dt)


def controller_thread_entry(
    controller: BusHomingController,
    my_phase1_done: threading.Event,
    other_phase1_done: threading.Event,
    my_phase2_done: threading.Event,
    takeover_event: threading.Event,
    stop_event: threading.Event,
    errors: List[str],
):
    channel = controller.runtime.channel
    try:
        if controller.phase1_config:
            controller._run_phase(
                sequence_ids=[cfg.motor_id for cfg in controller.phase1_config],
                hold_at_nudge_ids=set(),
                phase_name="PHASE1",
                stop_event=stop_event,
            )

        if not hold_until_both_phase1_done(
            controller=controller,
            my_done_event=my_phase1_done,
            other_done_event=other_phase1_done,
            stop_event=stop_event,
        ):
            return

        if stop_event.is_set():
            return

        if controller.phase2_config:
            controller._run_phase(
                sequence_ids=[cfg.motor_id for cfg in controller.phase2_config],
                hold_at_nudge_ids=set(),
                phase_name="PHASE2",
                stop_event=stop_event,
            )

        my_phase2_done.set()
        hold_until_takeover(
            controller=controller,
            takeover_event=takeover_event,
            stop_event=stop_event,
        )

    except Exception as exc:
        errors.append(f"{channel}: {exc}")
        stop_event.set()
        my_phase1_done.set()
        other_phase1_done.set()
        my_phase2_done.set()
        takeover_event.set()


def load_motor_command_config(path: str) -> MotorCommandConfig:
    with open(path, "r", encoding="utf-8") as fh:
        raw = json.load(fh)

    motors = raw.get("motors")
    if not isinstance(motors, dict):
        raise ValueError("motor_config.json must contain a top-level 'motors' object")

    config: MotorCommandConfig = {}
    for motor_id in range(1, 13):
        entry = motors.get(str(motor_id))
        if not isinstance(entry, dict):
            raise ValueError(f"motor_config.json missing config for motor {motor_id}")

        config[motor_id] = {
            "kp":         float(entry["kp"]),
            "kd":         float(entry["kd"]),
            "flipped":    bool(entry["flipped"]),
            "gear_ratio": float(entry["gear_ratio"]),
        }

    return config


def validate_leg_payload(payload: Any) -> Tuple[LegPayload, Optional[float]]:
    """
    Validates the incoming socket payload.

    Returns:
        (normalized_leg_payload, speed_override)

    speed_override is None if the sender did not include a "speed" key,
    in which case run_post_homing_live_control falls back to MAX_LIVE_DEG_PER_S.
    tapping_gait.py sends "speed": 900.0 to unlock fast tap movements.
    """
    if not isinstance(payload, dict):
        raise ValueError("payload must be a dict with keys: fl, bl, fr, br")

    normalized: LegPayload = {}
    for leg in LEG_ORDER:
        if leg not in payload:
            raise ValueError(f"missing leg '{leg}' in payload")

        values = payload[leg]
        if not isinstance(values, (list, tuple)):
            raise ValueError(f"payload['{leg}'] must be a list or tuple")

        if len(values) != 3:
            raise ValueError(f"payload['{leg}'] must contain exactly 3 values")

        normalized[leg] = [float(v) for v in values]

    # Optional per-packet speed override — absent means use the global default
    speed_override: Optional[float] = None
    if "speed" in payload:
        speed_override = float(payload["speed"])

    return normalized, speed_override


def flatten_leg_payload(payload: LegPayload) -> List[float]:
    flat: List[float] = []
    for leg in LEG_ORDER:
        flat.extend(payload[leg])
    return flat


def transform_live_angles(
    raw_angles_deg: List[float],
    motor_config: MotorCommandConfig,
) -> Dict[int, float]:
    if len(raw_angles_deg) != 12:
        raise ValueError(f"expected 12 motor angles, received {len(raw_angles_deg)}")

    transformed: Dict[int, float] = {}
    for motor_id, raw_angle_deg in enumerate(raw_angles_deg, start=1):
        cfg = motor_config[motor_id]
        angle_deg = float(raw_angle_deg)
        if bool(cfg["flipped"]):
            angle_deg = -angle_deg
        angle_deg *= float(cfg["gear_ratio"])
        transformed[motor_id] = angle_deg

    return transformed


def convert_coords_to_motor_targets(
    leg_payload: LegPayload,
    ik_offsets: dict,                # ← dict (per-leg), not tuple
    motor_config: MotorCommandConfig,
) -> Dict[int, float]:
    """
    Converts foot coordinates (x, y, z) per leg into final motor angles.
    Pipeline: IK → per-leg calibration offset → flip → gear_ratio
    """
    transformed: Dict[int, float] = {}

    for leg in LEG_ORDER:
        x, y, z = leg_payload[leg]
        m1, m2, m3 = ik_to_motor_deg(x, y, z, ik_offsets, leg)  # ← leg added
        raw_angles = [m1, m2, m3]

        for i, motor_id in enumerate(LEG_TO_MOTOR_IDS[leg]):
            cfg = motor_config[motor_id]
            angle = raw_angles[i]
            if bool(cfg["flipped"]):
                angle = -angle
            angle *= float(cfg["gear_ratio"])
            transformed[motor_id] = angle

    return transformed


def runtime_for_motor_id(
    motor_id: int,
    rt0: BusRuntime,
    rt1: BusRuntime,
) -> BusRuntime:
    if 1 <= motor_id <= 6:
        return rt0
    if 7 <= motor_id <= 12:
        return rt1
    raise ValueError(f"invalid motor_id {motor_id}")


def send_live_targets(
    rt0: BusRuntime,
    rt1: BusRuntime,
    targets_deg: Dict[int, float],
    motor_config: MotorCommandConfig,
):
    for motor_id in range(1, 13):
        cfg = motor_config[motor_id]
        runtime = runtime_for_motor_id(motor_id, rt0, rt1)
        runtime.send_position_deg(
            motor_id,
            targets_deg[motor_id],
            kp=float(cfg["kp"]),
            kd=float(cfg["kd"]),
            torque=0.0,
        )


def drain_latest_packet(
    live_queue: MpQueue,
) -> Optional[Tuple[LegPayload, Optional[float]]]:
    """
    Drains the queue and returns the most recent (leg_payload, speed_override) tuple,
    or None if the queue was empty.
    """
    latest = None
    while True:
        try:
            latest = live_queue.get_nowait()
        except queue.Empty:
            break
    return latest


def limit_target_step(
    current_cmd_deg: float,
    requested_deg: float,
    max_deg_per_s: float,
    dt: float,
) -> float:
    max_step = max_deg_per_s * dt
    delta = requested_deg - current_cmd_deg

    if delta > max_step:
        return current_cmd_deg + max_step
    if delta < -max_step:
        return current_cmd_deg - max_step
    return requested_deg


def initialize_live_command_state(
    rt0: BusRuntime,
    rt1: BusRuntime,
) -> Dict[int, float]:
    live_cmd_deg: Dict[int, float] = {}

    for motor_id in range(1, 13):
        runtime = runtime_for_motor_id(motor_id, rt0, rt1)
        st = runtime.get_state_copy(motor_id)
        live_cmd_deg[motor_id] = st.position_deg

    return live_cmd_deg


def current_logger_thread_entry(
    rt0: BusRuntime,
    rt1: BusRuntime,
    stop_event: threading.Event,
    log_path: str = CURRENT_LOG_PATH,
    log_hz: float = CURRENT_LOG_HZ,
) -> None:
    """
    Reads current (Amps) from all 12 motors at `log_hz` Hz and appends
    every sample as a CSV row to `log_path`.

    CSV columns:
        timestamp_s, m1_a, m2_a, ..., m12_a
    """
    log_dt = 1.0 / log_hz
    motor_ids = list(range(1, 13))

    try:
        with open(log_path, "w", buffering=1, encoding="utf-8") as fh:
            fh.seek(0, 2)
            if fh.tell() == 0:
                header = "timestamp_s," + ",".join(f"m{i}_a" for i in motor_ids)
                fh.write(header + "\n")

            print(
                f"[CurrentLogger] Logging {log_hz:.0f} Hz motor currents -> {log_path}",
                flush=True,
            )

            while not stop_event.is_set():
                t_start = time.monotonic()
                ts = time.time()

                currents: List[float] = []
                for motor_id in motor_ids:
                    runtime = runtime_for_motor_id(motor_id, rt0, rt1)
                    try:
                        state = runtime.get_state_copy(motor_id)
                        currents.append(round(state.current_A, 4))
                    except Exception:
                        currents.append(float("nan"))

                row = f"{ts:.4f}," + ",".join(str(c) for c in currents)
                fh.write(row + "\n")

                elapsed = time.monotonic() - t_start
                sleep_for = log_dt - elapsed
                if sleep_for > 0:
                    time.sleep(sleep_for)

    except Exception as exc:
        print(f"[CurrentLogger] Fatal error: {exc}", flush=True)


def temp_logger_thread_entry(
    rt0: BusRuntime,
    rt1: BusRuntime,
    stop_event: threading.Event,
    log_path: str = TEMP_LOG_PATH,
    log_hz: float = TEMP_LOG_HZ,
) -> None:
    """
    Reads temperature (°C) from all 12 motors at `log_hz` Hz and appends
    every sample as a CSV row to `log_path`.

    CSV columns:
        timestamp_s, m1_c, m2_c, ..., m12_c
    """
    log_dt = 1.0 / log_hz
    motor_ids = list(range(1, 13))

    try:
        with open(log_path, "w", buffering=1, encoding="utf-8") as fh:
            fh.seek(0, 2)
            if fh.tell() == 0:
                header = "timestamp_s," + ",".join(f"m{i}_c" for i in motor_ids)
                fh.write(header + "\n")

            print(
                f"[TempLogger] Logging {log_hz:.0f} Hz motor temperatures -> {log_path}",
                flush=True,
            )

            while not stop_event.is_set():
                t_start = time.monotonic()
                ts = time.time()

                temps: List[float] = []
                for motor_id in motor_ids:
                    runtime = runtime_for_motor_id(motor_id, rt0, rt1)
                    try:
                        state = runtime.get_state_copy(motor_id)
                        temps.append(round(state.temperature_C, 4))
                    except Exception:
                        temps.append(float("nan"))

                row = f"{ts:.4f}," + ",".join(str(t) for t in temps)
                fh.write(row + "\n")

                elapsed = time.monotonic() - t_start
                sleep_for = log_dt - elapsed
                if sleep_for > 0:
                    time.sleep(sleep_for)

    except Exception as exc:
        print(f"[TempLogger] Fatal error: {exc}", flush=True)


def socket_listener_process(
    dest_queue: MpQueue,
    stop_event: MpEvent,
    port: int = LIVE_SOCKET_PORT,
    host: str = LIVE_SOCKET_HOST,
):
    print(f"[Socket Process {mp.current_process().pid}] Starting on {host}:{port}", flush=True)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.settimeout(0.2)

    try:
        server.bind((host, port))
        server.listen(5)
        print(f"[Socket Process {mp.current_process().pid}] Listening", flush=True)

        while not stop_event.is_set():
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            except OSError:
                if stop_event.is_set():
                    break
                raise

            with conn:
                conn.settimeout(0.2)
                data = bytearray()

                while not stop_event.is_set():
                    try:
                        chunk = conn.recv(4096)
                    except socket.timeout:
                        continue

                    if not chunk:
                        break
                    data.extend(chunk)

            if not data:
                continue

            try:
                payload = pickle.loads(bytes(data))
                # validate_leg_payload now returns (leg_payload, speed_override)
                packet, speed_override = validate_leg_payload(payload)
            except Exception as exc:
                print(f"[Socket Process] Dropping invalid packet from {addr}: {exc}", flush=True)
                continue

            # Drain stale items so only the latest command is in the queue
            while True:
                try:
                    dest_queue.get_nowait()
                except queue.Empty:
                    break
                except Exception:
                    break

            try:
                # Queue carries (leg_payload, speed_override) together
                dest_queue.put_nowait((packet, speed_override))
            except Exception:
                pass

    except Exception as exc:
        print(f"[Socket Process] Server error: {exc}", flush=True)
        raise
    finally:
        server.close()
        print(f"[Socket Process {mp.current_process().pid}] Shutdown", flush=True)


def run_post_homing_live_control(
    ctrl0: BusHomingController,
    ctrl1: BusHomingController,
    rt0: BusRuntime,
    rt1: BusRuntime,
    tuning: ControlTuning,
    stop_event: threading.Event,
    live_queue: MpQueue,
    motor_config: MotorCommandConfig,
    ik_offsets: dict,               # ← dict (per-leg), not tuple
):
    first_live_packet_seen = False
    last_live_targets_deg: Optional[Dict[int, float]] = None
    live_cmd_deg: Dict[int, float] = initialize_live_command_state(rt0, rt1)

    # Active speed limit — updated each time a packet with a "speed" key arrives.
    # Falls back to MAX_LIVE_DEG_PER_S when the sender omits "speed" (e.g. test_sender).
    current_speed_limit: float = MAX_LIVE_DEG_PER_S

    print("\n" + "=" * 70)
    print("🎯 POST-HOMING LIVE CONTROL READY")
    print(f"   Socket receiver process expects grouped targets on {LIVE_SOCKET_HOST}:{LIVE_SOCKET_PORT}")
    print("   Format: {'fl':[x,y,z], 'bl':[x,y,z], 'fr':[x,y,z], 'br':[x,y,z]}  ← coords in cm")
    print("   Optional: include 'speed': <deg/s> in payload to override the per-packet speed limit")
    print("   CAN routing stays fixed from existing setup: M1-M6 -> can0, M7-M12 -> can1")
    print("   Until the first packet arrives, motors keep holding their homed nudge positions")
    print("   After the first packet, the limiter smoothly chases the newest target")
    print(f"   Default safety limit: {MAX_LIVE_DEG_PER_S:.1f} deg/s per motor  (tapping_gait uses 900)")
    print("=" * 70)

    while not stop_event.is_set():
        rt0.refresh_watchdogs(
            feedback_timeout_s=tuning.feedback_timeout_s,
            bus_silence_timeout_s=tuning.bus_silence_timeout_s,
            fault_on_missing_feedback=True,
        )
        rt1.refresh_watchdogs(
            feedback_timeout_s=tuning.feedback_timeout_s,
            bus_silence_timeout_s=tuning.bus_silence_timeout_s,
            fault_on_missing_feedback=True,
        )

        if rt0.faulted:
            raise RuntimeError(rt0.get_fault_summary())
        if rt1.faulted:
            raise RuntimeError(rt1.get_fault_summary())

        # drain_latest_packet now returns (leg_payload, speed_override) or None
        item = drain_latest_packet(live_queue)
        if item is not None:
            leg_packet, speed_override = item
            last_live_targets_deg = convert_coords_to_motor_targets(
                leg_packet, ik_offsets, motor_config
            )

            # Use the per-packet speed if provided, otherwise fall back to global default
            current_speed_limit = speed_override if speed_override is not None else MAX_LIVE_DEG_PER_S

            if not first_live_packet_seen:
                print(
                    f"✅ First live packet received. "
                    f"Speed limit: {current_speed_limit:.1f} deg/s. "
                    "Switching from homed nudge hold to live target hold."
                )
                first_live_packet_seen = True

        if not first_live_packet_seen:
            ctrl0.hold_all_once()
            ctrl1.hold_all_once()
        else:
            for motor_id in range(1, 13):
                live_cmd_deg[motor_id] = limit_target_step(
                    current_cmd_deg=live_cmd_deg[motor_id],
                    requested_deg=last_live_targets_deg[motor_id],
                    max_deg_per_s=current_speed_limit * motor_config[motor_id]["gear_ratio"],
                    dt=tuning.loop_dt,
                )

            send_live_targets(rt0, rt1, live_cmd_deg, motor_config)

        time.sleep(tuning.loop_dt)


def main():
    time.sleep(3)
    CALIBRATION_REQUIRED = parse_calibration_flag()
    print("=" * 70)
    print("🤖 AK60 | 4-LEG HOMING | 12 MOTORS | 2 CAN BUSES")
    if CALIBRATION_REQUIRED:
        print("   Phase 1: Front Right (can1: M8→M9→M7)")
        print("            Back Left   (can0: M5→M6→M4)  ← simultaneous")
        print("   Phase 2: Back Right  (can1: M11→M12→M10)")
        print("            Front Left  (can0: M2→M3→M1)  ← simultaneous")
    else:
        print("   ⏩ Homing skipped (CALIBRATION_REQUIRED=False)")
    print("   Final:    Post-homing live control with grouped leg socket receiver process")
    print("=" * 70)

    print_lock = threading.Lock()
    stop_event = threading.Event()
    errors: List[str] = []

    can0_phase1_done = threading.Event()
    can1_phase1_done = threading.Event()
    can0_phase2_done = threading.Event()
    can1_phase2_done = threading.Event()
    final_hold_takeover = threading.Event()

    tuning = ControlTuning(
        active_kp=114.0,
        active_kd=1.2,
        hold_kp=54.0,
        hold_kd=0.9,
        homed_hold_kp=84.0,
        homed_hold_kd=1.2,
        loop_hz=100.0,
        min_move_time_s=1.2,
        seconds_per_deg=0.03,
        trigger_confirm_s=0.15,
        trigger_velocity_raw_max=4.0,
        feedback_timeout_s=0.15,
        bus_silence_timeout_s=0.40,
        startup_timeout_s=5.0,
        startup_poll_s=0.01,
        zero_feedback_timeout_s=1.5,
        zero_position_tolerance_deg=5.0,
        max_search_time_s=10.0,
        max_search_travel_deg=220.0,
        status_period_s=0.5,
        safe_idle_kp=48.0,
        safe_idle_kd=0.8,
        nudge_position_tolerance_deg=2.0,
        nudge_settle_time_s=0.12,
        search_max_vel_deg_s=100.0,
        search_max_acc_deg_s2=140.0,
        continuous_search_vel_deg_s=40.0,
        nudge_max_vel_deg_s=60.0,
        nudge_max_acc_deg_s2=168.0,
        advance_target_tolerance_deg=2.0,
        advance_settle_time_s=0.20,
        stall_progress_epsilon_deg=0.25,
        stall_timeout_s=0.50,
        wait_log_period_s=1.0,
        contact_hold_offset_deg=0.5,
        contact_release_move_deg=3.0,
        contact_stall_vel_max=2.0,
    )

    rt0 = None
    rt1 = None
    live_queue = None
    live_socket_stop = None
    live_socket_process = None

    try:
        can0_ids = collect_motor_ids(CAN_CONFIG["can0"])
        can1_ids = collect_motor_ids(CAN_CONFIG["can1"])

        rt0 = BusRuntime("can0", can0_ids, bitrate=1_000_000)
        rt1 = BusRuntime("can1", can1_ids, bitrate=1_000_000)

        ctrl0 = BusHomingController(
            runtime=rt0,
            phase1_config=CAN_CONFIG["can0"]["phase1"],
            phase2_config=CAN_CONFIG["can0"]["phase2"],
            tuning=tuning,
            print_lock=print_lock,
        )
        ctrl1 = BusHomingController(
            runtime=rt1,
            phase1_config=CAN_CONFIG["can1"]["phase1"],
            phase2_config=CAN_CONFIG["can1"]["phase2"],
            tuning=tuning,
            print_lock=print_lock,
        )

        print("\n📡 Waiting for periodic feedback from all motors...")
        ctrl0.verify_periodic_feedback_and_capture_boot_holds()
        ctrl1.verify_periodic_feedback_and_capture_boot_holds()
        print("✅ All motors are streaming feedback.\n")

        print("🧷 Sending low-gain idle hold at captured boot positions for 1 second...")
        for _ in range(int(tuning.loop_hz)):
            ctrl0.send_idle_hold_once()
            ctrl1.send_idle_hold_once()
            time.sleep(tuning.loop_dt)

        if CALIBRATION_REQUIRED:
            print("🚀 Starting phase 1 on both CAN buses...\n")

            t0 = threading.Thread(
                target=controller_thread_entry,
                args=(
                    ctrl0,
                    can0_phase1_done,
                    can1_phase1_done,
                    can0_phase2_done,
                    final_hold_takeover,
                    stop_event,
                    errors,
                ),
                daemon=True,
            )
            t1 = threading.Thread(
                target=controller_thread_entry,
                args=(
                    ctrl1,
                    can1_phase1_done,
                    can0_phase1_done,
                    can1_phase2_done,
                    final_hold_takeover,
                    stop_event,
                    errors,
                ),
                daemon=True,
            )
            t0.start()
            t1.start()

            while not stop_event.is_set():
                if errors:
                    raise RuntimeError(" | ".join(errors))
                if can0_phase2_done.is_set() and can1_phase2_done.is_set():
                    break
                time.sleep(tuning.loop_dt)

            if errors:
                raise RuntimeError(" | ".join(errors))

            for _ in range(5):
                ctrl0.hold_all_once()
                ctrl1.hold_all_once()
                time.sleep(tuning.loop_dt)

            final_hold_takeover.set()

            t0.join()
            t1.join()

            if errors:
                raise RuntimeError(" | ".join(errors))

        else:
            print("⏩ Skipping homing — CALIBRATION_REQUIRED=False. Proceeding to live control...")
            final_hold_takeover.set()

        print(f"\n📄 Loading motor command config from {MOTOR_CONFIG_PATH}...")
        motor_config = load_motor_command_config(MOTOR_CONFIG_PATH)
        print("✅ Motor command config loaded.")

        print("📐 Computing IK calibration offsets...")
        ik_offsets = compute_ik_offsets()
        print("✅ IK calibration offsets computed.")

        current_log_thread = threading.Thread(
            target=current_logger_thread_entry,
            args=(rt0, rt1, stop_event),
            daemon=True,
            name="CurrentLogger",
        )
        current_log_thread.start()
        print(
            f"📊 Motor current logger started at {CURRENT_LOG_HZ:.0f} Hz -> {CURRENT_LOG_PATH}"
        )

        temp_log_thread = threading.Thread(
            target=temp_logger_thread_entry,
            args=(rt0, rt1, stop_event),
            daemon=True,
            name="TempLogger",
        )
        temp_log_thread.start()
        print(
            f"🌡️  Motor temperature logger started at {TEMP_LOG_HZ:.0f} Hz -> {TEMP_LOG_PATH}"
        )

        live_queue = mp.Queue(maxsize=LIVE_QUEUE_MAXSIZE)
        live_socket_stop = mp.Event()
        live_socket_process = mp.Process(
            target=socket_listener_process,
            args=(live_queue, live_socket_stop, LIVE_SOCKET_PORT, LIVE_SOCKET_HOST),
            daemon=True,
        )
        live_socket_process.start()
        print(f"📡 Socket receiver process started with PID {live_socket_process.pid}.")

        run_post_homing_live_control(
            ctrl0=ctrl0,
            ctrl1=ctrl1,
            rt0=rt0,
            rt1=rt1,
            tuning=tuning,
            stop_event=stop_event,
            live_queue=live_queue,
            motor_config=motor_config,
            ik_offsets=ik_offsets,
        )

    except KeyboardInterrupt:
        print("\n🛑 Ctrl+C — stopping...")
        stop_event.set()
        final_hold_takeover.set()
    except Exception as exc:
        print(f"\n❌ Runtime error: {exc}")
        stop_event.set()
        final_hold_takeover.set()
    finally:
        if live_socket_stop is not None:
            live_socket_stop.set()
        if live_socket_process is not None:
            live_socket_process.join(timeout=1.0)
            if live_socket_process.is_alive():
                live_socket_process.terminate()
                live_socket_process.join(timeout=1.0)

        print("\n🔌 Disabling motors and shutting down...")
        try:
            if rt0 is not None:
                rt0.disable_all()
        except Exception:
            pass
        try:
            if rt1 is not None:
                rt1.disable_all()
        except Exception:
            pass
        try:
            if rt0 is not None:
                rt0.stop()
        except Exception:
            pass
        try:
            if rt1 is not None:
                rt1.stop()
        except Exception:
            pass
        print("✅ Done.")


if __name__ == "__main__":
    mp.freeze_support()
    main()