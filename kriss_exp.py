#!/usr/bin/env python3
"""
kriss_exp.py — Robust single-motor test tool for AK60 motor 2 (can0).

Modes:
    --position <deg>    Hold a target position (degrees)
    --velocity <val>    Spin at target velocity (raw units, +/-)
    --none              Zero torque — free spin / backdrive by hand

All modes print live telemetry and record to CSV.

Usage:
    sudo python kriss_exp.py --position 60
    sudo python kriss_exp.py --velocity 10
    sudo python kriss_exp.py --none

Kill:
    sudo kill -9 $(pgrep -f kriss_exp.py)
"""

import argparse
import csv
import math
import os
import signal
import sys
import time

from can_runtime import BusRuntime

# ═══════════════════════════════════════════════════════════════════════════════
#  HARDWARE CONFIG
# ═══════════════════════════════════════════════════════════════════════════════
MOTOR_ID  = 2
CHANNEL   = "can0"
BITRATE   = 1_000_000

# ═══════════════════════════════════════════════════════════════════════════════
#  CONTROL TUNING
# ═══════════════════════════════════════════════════════════════════════════════
POSITION_KP  = 54.0   # proportional gain for --position mode
POSITION_KD  = 0.9    # derivative gain  for --position mode

VELOCITY_KP  = 0.0    # kp=0 for velocity control (MIT mode)
VELOCITY_KD  = 0.5    # damping for --velocity mode

# ═══════════════════════════════════════════════════════════════════════════════
#  LOOP & PRINT TIMING
# ═══════════════════════════════════════════════════════════════════════════════
LOOP_HZ   = 100.0
LOOP_DT   = 1.0 / LOOP_HZ
PRINT_HZ  = 10.0
PRINT_DT  = 1.0 / PRINT_HZ

# ═══════════════════════════════════════════════════════════════════════════════
#  WATCHDOG / STARTUP
# ═══════════════════════════════════════════════════════════════════════════════
STARTUP_TIMEOUT_S      = 5.0
FEEDBACK_TIMEOUT_S     = 0.15
BUS_SILENCE_TIMEOUT_S  = 0.40

# ═══════════════════════════════════════════════════════════════════════════════
#  RECORDING
# ═══════════════════════════════════════════════════════════════════════════════
CSV_PATH = "kriss_exp_recording.csv"


# ═══════════════════════════════════════════════════════════════════════════════
#  SIGNAL HANDLING  (makes Ctrl+C and kill reliable)
# ═══════════════════════════════════════════════════════════════════════════════
_shutdown = False

def _handle_signal(sig, frame):
    global _shutdown
    _shutdown = True

signal.signal(signal.SIGINT,  _handle_signal)
signal.signal(signal.SIGTERM, _handle_signal)


# ═══════════════════════════════════════════════════════════════════════════════
#  ARGS
# ═══════════════════════════════════════════════════════════════════════════════
def parse_args():
    parser = argparse.ArgumentParser(
        description="AK60 motor 2 test tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--position", type=float, metavar="DEG",
                       help="Hold target position in degrees")
    group.add_argument("--velocity", type=float, metavar="VEL",
                       help="Spin at target velocity (raw units, +/-)")
    group.add_argument("--none", action="store_true",
                       help="Zero torque — free spin / backdrive by hand")
    return parser.parse_args()


# ═══════════════════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════════════════
def main():
    global _shutdown
    args = parse_args()

    # Describe the active mode
    if args.position is not None:
        mode_label = f"POSITION HOLD  target={args.position:+.1f}°  kp={POSITION_KP}  kd={POSITION_KD}"
    elif args.velocity is not None:
        mode_label = f"VELOCITY  target={args.velocity:+.1f} (raw)  kp={VELOCITY_KP}  kd={VELOCITY_KD}"
    else:
        mode_label = "FREE SPIN  (zero torque — backdrive by hand)"

    print("=" * 60)
    print(f"  Motor {MOTOR_ID}  |  {CHANNEL}  |  {BITRATE//1000} kbps")
    print(f"  Mode  : {mode_label}")
    print(f"  CSV   : {CSV_PATH}")
    print(f"  Loop  : {LOOP_HZ:.0f} Hz   Print: {PRINT_HZ:.0f} Hz")
    print("=" * 60)
    print("  Kill: sudo kill -9 $(pgrep -f kriss_exp.py)\n")

    rt = BusRuntime(CHANNEL, [MOTOR_ID], bitrate=BITRATE)

    try:
        ok = rt.wait_for_periodic_feedback(
            timeout_s             = STARTUP_TIMEOUT_S,
            poll_period_s         = 0.01,
            feedback_timeout_s    = FEEDBACK_TIMEOUT_S,
            bus_silence_timeout_s = BUS_SILENCE_TIMEOUT_S,
        )
        if not ok:
            print(f"[kriss_exp] ERROR: {rt.get_fault_summary()}")
            return

        # Snapshot boot position for position mode initialisation
        boot = rt.get_state_copy(MOTOR_ID).position_deg
        print(f"[kriss_exp] Motor ready.  Boot position: {boot:+.2f}°\n")

        last_print = 0.0

        with open(CSV_PATH, "w", newline="", buffering=1) as f:
            writer = csv.writer(f)
            writer.writerow([
                "timestamp_s", "mode",
                "position_deg", "velocity_raw",
                "current_A", "temperature_C",
            ])

            while not _shutdown:
                rt.refresh_watchdogs(
                    feedback_timeout_s    = FEEDBACK_TIMEOUT_S,
                    bus_silence_timeout_s = BUS_SILENCE_TIMEOUT_S,
                )
                if rt.faulted:
                    print(f"\n[kriss_exp] FAULT: {rt.get_fault_summary()}")
                    break

                # ── Send command based on mode ────────────────────────────────
                if args.position is not None:
                    rt.send_position_deg(
                        MOTOR_ID,
                        position_deg = args.position,
                        kp           = POSITION_KP,
                        kd           = POSITION_KD,
                        torque       = 0.0,
                    )
                    mode_tag = "position"

                elif args.velocity is not None:
                    rt.motors[MOTOR_ID].send_mit_command(
                        position = math.radians(boot),   # ignored (kp=0)
                        velocity = args.velocity,
                        kp       = VELOCITY_KP,
                        kd       = VELOCITY_KD,
                        torque   = 0.0,
                    )
                    mode_tag = "velocity"

                else:  # --none
                    rt.motors[MOTOR_ID].send_mit_command(
                        position = 0.0,
                        velocity = 0.0,
                        kp       = 0.0,
                        kd       = 0.0,
                        torque   = 0.0,
                    )
                    mode_tag = "none"

                # ── Read state & record ───────────────────────────────────────
                st  = rt.get_state_copy(MOTOR_ID)
                now = time.time()

                writer.writerow([
                    f"{now:.4f}", mode_tag,
                    f"{st.position_deg:.2f}",
                    f"{st.velocity_raw:.1f}",
                    f"{st.current_A:.3f}",
                    f"{st.temperature_C}",
                ])

                # ── Live print ────────────────────────────────────────────────
                if now - last_print >= PRINT_DT:
                    print(
                        f"  [{mode_tag:8s}]  "
                        f"pos={st.position_deg:+8.2f}°  "
                        f"vel={st.velocity_raw:+7.1f}  "
                        f"cur={st.current_A:+6.3f} A  "
                        f"temp={st.temperature_C:3d}°C  "
                        f"age={st.feedback_age * 1000:5.1f} ms",
                        flush=True,
                    )
                    last_print = now

                time.sleep(LOOP_DT)

    finally:
        print("\n[kriss_exp] Shutting down — disabling motor...")
        try:
            rt.disable_motor(MOTOR_ID)
            time.sleep(0.1)
        except Exception:
            pass
        try:
            rt.stop()
        except Exception:
            pass
        print(f"[kriss_exp] Recording saved → {CSV_PATH}")
        print("[kriss_exp] Done.")


if __name__ == "__main__":
    main()
