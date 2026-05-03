#!/usr/bin/env python3

import threading
import time

from can_runtime import BusRuntime
from homing_controller import BusHomingController, ControlTuning, HomingMotorConfig

CHANNEL = "can0"
BITRATE = 1_000_000

PHASE1_CONFIG = [
    HomingMotorConfig(5, -60.0, -1.0, 3.4, 15.0),
    HomingMotorConfig(6,  10.0,  1.0, 2.7, -120.0),
    HomingMotorConfig(4,  60.0,  1.0, 5.0, -15.0),
]

PHASE2_CONFIG = []


def main():
    rt = None
    try:
        motor_ids = sorted({cfg.motor_id for cfg in PHASE1_CONFIG + PHASE2_CONFIG})
        print(f"Starting single-leg phase1 pretest on {CHANNEL} motors {motor_ids}")

        rt = BusRuntime(CHANNEL, motor_ids, bitrate=BITRATE)

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
            max_search_travel_deg=180.0,
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
            contact_stall_vel_max=2.0

        )

        ctrl = BusHomingController(
            runtime=rt,
            phase1_config=PHASE1_CONFIG,
            phase2_config=PHASE2_CONFIG,
            tuning=tuning,
            print_lock=threading.Lock(),
        )

        ctrl.verify_periodic_feedback_and_capture_boot_holds()

        print("\nSending low-gain idle hold for 1 second before homing...")
        for _ in range(100):
            ctrl.send_idle_hold_once()
            time.sleep(tuning.loop_dt)

        print("\nStarting single-leg phase-1 homing...\n")
        stop_event = threading.Event()
        ctrl._run_phase(
            sequence_ids=[cfg.motor_id for cfg in PHASE1_CONFIG],
            hold_at_nudge_ids=set(),
            phase_name="PRETEST_PHASE1",
            stop_event=stop_event,
        )

        print("\nPhase complete. Holding final targets for 5 seconds...")
        t_end = time.time() + 5.0
        while time.time() < t_end:
            ctrl.hold_all_once()
            time.sleep(tuning.loop_dt)

        print("Single-leg phase1 pretest PASSED.")

    except KeyboardInterrupt:
        print("\nStopped by user.")
    except Exception as exc:
        print(f"\nPretest FAILED: {exc}")
    finally:
        if rt is not None:
            try:
                rt.disable_all()
            except Exception:
                pass
            try:
                rt.stop()
            except Exception:
                pass


if __name__ == "__main__":
    main()
