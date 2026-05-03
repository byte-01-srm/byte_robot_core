#!/usr/bin/env python3

import time
from can_runtime import BusRuntime

CHANNEL = "can0"
MOTOR_IDS = [4, 5, 6]
BITRATE = 1_000_000

STARTUP_TIMEOUT_S = 5.0
PRINT_PERIOD_S = 0.5
RUN_TIME_S = 20.0


def main():
    rt = None
    try:
        print(f"Starting feedback-only pretest on {CHANNEL} motors {MOTOR_IDS}")
        rt = BusRuntime(CHANNEL, MOTOR_IDS, bitrate=BITRATE)

        ok = rt.wait_for_periodic_feedback(
            timeout_s=STARTUP_TIMEOUT_S,
            poll_period_s=0.01,
            feedback_timeout_s=0.15,
            bus_silence_timeout_s=0.50,
        )
        if not ok:
            raise RuntimeError(rt.get_fault_summary())

        print("All motors detected. Streaming states:\n")

        t_end = time.time() + RUN_TIME_S
        last_print = 0.0

        while time.time() < t_end:
            rt.refresh_watchdogs(
                feedback_timeout_s=0.15,
                bus_silence_timeout_s=0.50,
                fault_on_missing_feedback=True,
            )
            if rt.faulted:
                raise RuntimeError(rt.get_fault_summary())

            now = time.time()
            if now - last_print >= PRINT_PERIOD_S:
                last_print = now
                states = rt.get_all_state_copies()
                for mid in MOTOR_IDS:
                    st = states[mid]
                    print(
                        f"M{mid}: pos={st.position_deg:+7.1f}°  "
                        f"vel={st.velocity_raw:+7.1f}  "
                        f"cur={st.current_A:+6.2f}A  "
                        f"T={st.temperature_C:2d}C  "
                        f"err={st.error_code}  "
                        f"age={st.feedback_age*1000:6.1f}ms  "
                        f"count={st.feedback_count}"
                    )
                print("-" * 90)

            time.sleep(0.01)

        print("Feedback-only pretest PASSED.")

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        if rt is not None:
            try:
                rt.stop()
            except Exception:
                pass


if __name__ == "__main__":
    main()
