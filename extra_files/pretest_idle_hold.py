#!/usr/bin/env python3

import time
from can_runtime import BusRuntime

CHANNEL = "can0"
MOTOR_IDS = [4, 5, 6]
BITRATE = 1_000_000

KP = 40.0
KD = 1.0
LOOP_HZ = 100.0
DT = 1.0 / LOOP_HZ

STARTUP_TIMEOUT_S = 5.0
RUN_TIME_S = 15.0
PRINT_PERIOD_S = 0.5


def main():
    rt = None
    try:
        print(f"Starting idle-hold pretest on {CHANNEL} motors {MOTOR_IDS}")
        rt = BusRuntime(CHANNEL, MOTOR_IDS, bitrate=BITRATE)

        ok = rt.wait_for_periodic_feedback(
            timeout_s=STARTUP_TIMEOUT_S,
            poll_period_s=0.01,
            feedback_timeout_s=0.15,
            bus_silence_timeout_s=0.50,
        )
        if not ok:
            raise RuntimeError(rt.get_fault_summary())

        states = rt.get_all_state_copies()
        print("Captured boot hold positions:")
        for mid in MOTOR_IDS:
            print(f"  M{mid}: {states[mid].boot_hold_deg:+.1f}°")

        print("\nSending low-gain idle hold. Motor should not jump.")
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

            targets = rt.get_idle_hold_targets()
            rt.command_positions_deg(targets, kp=KP, kd=KD, torque=0.0)

            now = time.time()
            if now - last_print >= PRINT_PERIOD_S:
                last_print = now
                states = rt.get_all_state_copies()
                for mid in MOTOR_IDS:
                    st = states[mid]
                    err = st.position_deg - st.boot_hold_deg
                    print(
                        f"M{mid}: hold={st.boot_hold_deg:+7.1f}°  "
                        f"pos={st.position_deg:+7.1f}°  "
                        f"err={err:+6.2f}°  "
                        f"cur={st.current_A:+6.2f}A  "
                        f"age={st.feedback_age*1000:6.1f}ms"
                    )
                print("-" * 90)

            time.sleep(DT)

        print("Idle-hold pretest PASSED.")

    except KeyboardInterrupt:
        print("\nStopped by user.")
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
