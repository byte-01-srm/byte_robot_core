#!/usr/bin/env python3
"""
imu_reader.py — MPU6050 reader for BYTE-01 balance correction
==============================================================

Runs a background thread that reads the MPU6050 at ~200 Hz and
maintains a complementary-filtered roll and pitch angle.

Usage:
    from imu_reader import ImuReader

    imu = ImuReader(i2c_bus=1, i2c_addr=0x68)
    imu.start()

    roll, pitch = imu.get_angles()   # degrees, thread-safe

    imu.stop()

Mounting convention
-------------------
The MPU6050 is assumed to be mounted flat on the robot body with the
X-axis pointing forward and Y-axis pointing left (chip silkscreen UP).

If your sensor is rotated or flipped, change the two sign constants
MOUNT_ROLL_SIGN and MOUNT_PITCH_SIGN until:
  - roll  > 0  when the RIGHT side of the bot is lower
  - pitch > 0  when the FRONT of the bot is lower

Install dependency:  pip install smbus2
"""

import math
import threading
import time

try:
    import smbus2
    _SMBUS_AVAILABLE = True
except ImportError:
    _SMBUS_AVAILABLE = False


# ── MPU6050 register map ──────────────────────────────────────────────────────
_REG_PWR_MGMT_1   = 0x6B
_REG_ACCEL_XOUT_H = 0x3B   # 6 bytes: AX_H AX_L AY_H AY_L AZ_H AZ_L
_REG_GYRO_XOUT_H  = 0x43   # 6 bytes: GX_H GX_L GY_H GY_L GZ_H GZ_L
_REG_CONFIG       = 0x1A   # DLPF config
_REG_GYRO_CONFIG  = 0x1B
_REG_ACCEL_CONFIG = 0x1C

# Full-scale ranges used in this driver
_GYRO_FS_DEG_S  = 250.0    # ±250 °/s  → LSB sensitivity = 131.0
_ACCEL_FS_G     = 2.0      # ±2 g      → LSB sensitivity = 16384.0
_GYRO_SENS      = 131.0    # LSB / (°/s)
_ACCEL_SENS     = 16384.0  # LSB / g


# ── Mounting orientation ──────────────────────────────────────────────────────
# Flip either sign to +1 or -1 to match your physical sensor orientation.
# Rule: positive roll = right side down; positive pitch = front side down.
MOUNT_ROLL_SIGN  =  1
MOUNT_PITCH_SIGN =  1


# ── Complementary filter constant ────────────────────────────────────────────
# Higher α = trusts gyro more (less jitter, slower drift correction).
# 0.96–0.98 is a good starting range for a 200 Hz loop.
COMPLEMENTARY_ALPHA = 0.97


class ImuReader:
    """
    Background-thread MPU6050 reader.

    Provides roll_deg and pitch_deg via get_angles(), updated at ~200 Hz.
    All public methods are thread-safe.
    """

    def __init__(
        self,
        i2c_bus: int = 1,
        i2c_addr: int = 0x68,
        loop_hz: float = 200.0,
        alpha: float = COMPLEMENTARY_ALPHA,
    ):
        if not _SMBUS_AVAILABLE:
            raise ImportError(
                "smbus2 is not installed. Run: pip install smbus2"
            )

        self._bus_num  = i2c_bus
        self._addr     = i2c_addr
        self._loop_hz  = loop_hz
        self._loop_dt  = 1.0 / loop_hz
        self._alpha    = alpha

        self._roll_deg  = 0.0
        self._pitch_deg = 0.0
        self._roll_rate_dps  = 0.0   # for PD derivative term in balancer
        self._pitch_rate_dps = 0.0

        self._lock       = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._bus: smbus2.SMBus | None = None

        self._initialized = False

    # ── Public API ─────────────────────────────────────────────────────────────

    def start(self):
        """Open I2C, wake the MPU6050, and start the background reader thread."""
        self._bus = smbus2.SMBus(self._bus_num)
        self._init_mpu6050()

        # Warm up: read for 0.5 s to let the filter settle before the gait starts
        print("[IMU] Warming up complementary filter (0.5 s)...", flush=True)
        self._warm_up(duration_s=0.5)
        print(
            f"[IMU] Ready — roll={self._roll_deg:+.2f}°  pitch={self._pitch_deg:+.2f}°",
            flush=True,
        )

        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="ImuReader"
        )
        self._thread.start()

    def stop(self):
        """Stop the background thread and close the I2C bus."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._bus is not None:
            self._bus.close()
        print("[IMU] Stopped.", flush=True)

    def get_angles(self) -> tuple[float, float]:
        """
        Return (roll_deg, pitch_deg) — thread-safe snapshot.

        roll  > 0 : right side lower
        pitch > 0 : front side lower
        """
        with self._lock:
            return self._roll_deg, self._pitch_deg

    def get_rates(self) -> tuple[float, float]:
        """
        Return (roll_rate_dps, pitch_rate_dps) — thread-safe snapshot.
        Useful for the derivative term in the PD balance controller.
        """
        with self._lock:
            return self._roll_rate_dps, self._pitch_rate_dps

    # ── Internal ──────────────────────────────────────────────────────────────

    def _init_mpu6050(self):
        """Wake MPU6050, configure gyro/accel full-scale, enable DLPF."""
        bus = self._bus

        # Wake up (clear sleep bit)
        bus.write_byte_data(self._addr, _REG_PWR_MGMT_1, 0x00)
        time.sleep(0.1)

        # DLPF bandwidth = 44 Hz accel / 42 Hz gyro (config register value = 3)
        # Reduces noise without too much phase lag at 200 Hz sample rate
        bus.write_byte_data(self._addr, _REG_CONFIG, 0x03)

        # Gyro: ±250 °/s  (bits [4:3] = 00)
        bus.write_byte_data(self._addr, _REG_GYRO_CONFIG, 0x00)

        # Accel: ±2 g  (bits [4:3] = 00)
        bus.write_byte_data(self._addr, _REG_ACCEL_CONFIG, 0x00)

        self._initialized = True
        print(
            f"[IMU] MPU6050 initialised at bus={self._bus_num} addr=0x{self._addr:02X}",
            flush=True,
        )

    def _read_raw(self) -> tuple[float, float, float, float, float, float]:
        """
        Read raw 16-bit signed values for accel (ax, ay, az) and gyro (gx, gy, gz).
        Returns values in physical units: g and °/s.
        """
        bus = self._bus

        # Read 14 bytes starting from ACCEL_XOUT_H
        # Layout: AX_H AX_L AY_H AY_L AZ_H AZ_L TEMP_H TEMP_L GX_H GX_L GY_H GY_L GZ_H GZ_L
        raw = bus.read_i2c_block_data(self._addr, _REG_ACCEL_XOUT_H, 14)

        def to_signed(hi, lo):
            val = (hi << 8) | lo
            return val - 65536 if val >= 32768 else val

        ax_raw = to_signed(raw[0],  raw[1])
        ay_raw = to_signed(raw[2],  raw[3])
        az_raw = to_signed(raw[4],  raw[5])
        gx_raw = to_signed(raw[8],  raw[9])
        gy_raw = to_signed(raw[10], raw[11])
        gz_raw = to_signed(raw[12], raw[13])

        ax = ax_raw / _ACCEL_SENS   # g
        ay = ay_raw / _ACCEL_SENS   # g
        az = az_raw / _ACCEL_SENS   # g
        gx = gx_raw / _GYRO_SENS    # °/s
        gy = gy_raw / _GYRO_SENS    # °/s
        gz = gz_raw / _GYRO_SENS    # °/s  (unused but available)

        return ax, ay, az, gx, gy, gz

    def _accel_angles(self, ax, ay, az) -> tuple[float, float]:
        """
        Compute roll and pitch from accelerometer (absolute but noisy).
        Applies MOUNT_ROLL_SIGN and MOUNT_PITCH_SIGN.
        """
        roll  = MOUNT_ROLL_SIGN  * math.degrees(math.atan2(ay, az))
        pitch = MOUNT_PITCH_SIGN * math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))
        return roll, pitch

    def _warm_up(self, duration_s: float):
        """
        Run the complementary filter for `duration_s` seconds to settle
        the initial angle estimate before returning control.
        """
        ax, ay, az, gx, gy, _ = self._read_raw()
        roll, pitch = self._accel_angles(ax, ay, az)
        self._roll_deg  = roll
        self._pitch_deg = pitch

        t_end = time.monotonic() + duration_s
        t_prev = time.monotonic()

        while time.monotonic() < t_end:
            t_now = time.monotonic()
            dt = t_now - t_prev
            t_prev = t_now

            ax, ay, az, gx, gy, _ = self._read_raw()

            roll_accel, pitch_accel = self._accel_angles(ax, ay, az)

            roll_gyro_rate  = MOUNT_ROLL_SIGN  * gx   # °/s mapped to robot roll axis
            pitch_gyro_rate = MOUNT_PITCH_SIGN * gy   # °/s mapped to robot pitch axis

            self._roll_deg  = self._alpha * (self._roll_deg  + roll_gyro_rate  * dt) \
                            + (1 - self._alpha) * roll_accel
            self._pitch_deg = self._alpha * (self._pitch_deg + pitch_gyro_rate * dt) \
                            + (1 - self._alpha) * pitch_accel

            sleep_for = self._loop_dt - (time.monotonic() - t_now)
            if sleep_for > 0:
                time.sleep(sleep_for)

    def _run(self):
        """Background thread: read sensor → complementary filter → update shared state."""
        t_prev = time.monotonic()

        while not self._stop_event.is_set():
            t_now = time.monotonic()
            dt = t_now - t_prev
            t_prev = t_now

            try:
                ax, ay, az, gx, gy, _ = self._read_raw()
            except Exception as exc:
                print(f"[IMU] Read error: {exc}", flush=True)
                time.sleep(0.05)
                continue

            roll_accel, pitch_accel = self._accel_angles(ax, ay, az)

            roll_gyro_rate  = MOUNT_ROLL_SIGN  * gx
            pitch_gyro_rate = MOUNT_PITCH_SIGN * gy

            new_roll  = self._alpha * (self._roll_deg  + roll_gyro_rate  * dt) \
                      + (1 - self._alpha) * roll_accel
            new_pitch = self._alpha * (self._pitch_deg + pitch_gyro_rate * dt) \
                      + (1 - self._alpha) * pitch_accel

            with self._lock:
                self._roll_deg       = new_roll
                self._pitch_deg      = new_pitch
                self._roll_rate_dps  = roll_gyro_rate
                self._pitch_rate_dps = pitch_gyro_rate

            sleep_for = self._loop_dt - (time.monotonic() - t_now)
            if sleep_for > 0:
                time.sleep(sleep_for)


# ── Standalone test ───────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("MPU6050 live angle test — Ctrl+C to exit\n")
    imu = ImuReader(i2c_bus=1, i2c_addr=0x68)
    imu.start()

    try:
        while True:
            roll, pitch = imu.get_angles()
            rr, pr = imu.get_rates()
            print(
                f"\r  roll={roll:+7.2f}°  pitch={pitch:+7.2f}°"
                f"  roll_rate={rr:+7.1f}°/s  pitch_rate={pr:+7.1f}°/s",
                end="",
                flush=True,
            )
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()
    finally:
        imu.stop()