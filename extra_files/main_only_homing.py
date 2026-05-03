#!/usr/bin/env python3

import threading
import time
from typing import Dict, List

from can_runtime import BusRuntime
from homing_controller import BusHomingController, ControlTuning, HomingMotorConfig


CAN_CONFIG: Dict[str, Dict[str, List[HomingMotorConfig]]] = {
    "can0": {
        "phase1": [
            HomingMotorConfig(5, -60.0, -1.0, 4.0, 65.0),
            HomingMotorConfig(6,  10.0,  1.0, 5.0, -38.0),
            HomingMotorConfig(4,  60.0,  1.0, 4.0, -60.0),
        ],
        "phase2": [
            HomingMotorConfig(2, -60.0, -1.0, 4.0, 65.0),
            HomingMotorConfig(3,  10.0,  1.0, 5.0, -38.0),
            HomingMotorConfig(1,  60.0,  1.0, 4.0, -60.0),
        ],
    },
    "can1": {
        "phase1": [
            HomingMotorConfig(8,  60.0,  1.0, 4.0, -65.0),
            HomingMotorConfig(9, -10.0, -1.0, 5.0, 38.0),
            HomingMotorConfig(7, -60.0, -1.0, 4.0, 60.0),
        ],
        "phase2": [
            HomingMotorConfig(11,  60.0,  1.0, 4.0, -65.0),
            HomingMotorConfig(12, -10.0, -1.0, 5.0, 38.0),
            HomingMotorConfig(10, -60.0, -1.0, 4.0, 60.0),
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


def main():
    time.sleep(5)
    print("=" * 70)
    print("🤖 AK60 | 4-LEG HOMING | 12 MOTORS | 2 CAN BUSES")
    print("   Phase 1: Front Right (can1: M8→M9→M7)")
    print("            Back Left   (can0: M5→M6→M4)  ← simultaneous")
    print("   Phase 2: Back Right  (can1: M11→M12→M10)")
    print("            Front Left  (can0: M2→M3→M1)  ← simultaneous")
    print("   Final:    Single thread holds all 12 at nudge positions")
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

        print("\n" + "=" * 70)
        print("🏁 ALL 12 MOTORS HOMED — holding forever. Ctrl+C to exit.")
        print("=" * 70)

        while not stop_event.is_set():
            ctrl0.hold_all_once()
            ctrl1.hold_all_once()
            time.sleep(tuning.loop_dt)

    except KeyboardInterrupt:
        print("\n🛑 Ctrl+C — stopping...")
        stop_event.set()
        final_hold_takeover.set()

    except Exception as exc:
        print(f"\n❌ Runtime error: {exc}")
        stop_event.set()
        final_hold_takeover.set()

    finally:
        print("\nReleasing motors and shutting down...")

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
    main()
