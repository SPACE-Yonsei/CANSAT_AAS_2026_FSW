"""BNO08x I2C IMU driver for `imuapp` (optional hardware path)."""

from __future__ import annotations

import logging
import math
import os
import sys
import time
from pathlib import Path
from typing import Any, Optional

_REPO = Path(__file__).resolve().parents[1]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

from lib import i2c_bus

logger = logging.getLogger(__name__)

_BNO_FEATURE_NAMES = {
    0x01: "accelerometer",
    0x02: "gyroscope",
    0x03: "magnetometer",
    0x05: "rotation_vector",
    0x08: "game_rotation_vector",
}


def _init_progress(msg: str) -> None:
    if os.environ.get("IMU_INIT_PROGRESS", "0").strip() == "1":
        print(msg, flush=True)


def _enable_feature_retry(bno: Any, feature_id: int, attempts: Optional[int] = None) -> None:
    """BNO08x often needs a short settle + retries right after power-up (Blinka / Pi)."""
    if attempts is None:
        try:
            attempts = int(os.environ.get("IMU_ENABLE_FEATURE_ATTEMPTS", "8"), 0)
        except ValueError:
            attempts = 8
        attempts = max(3, min(attempts, 20))
    name = _BNO_FEATURE_NAMES.get(feature_id, f"id={feature_id:#04x}")
    _init_progress(f"IMU: enable {name} …")
    last: Optional[Exception] = None
    for i in range(attempts):
        try:
            bno.enable_feature(feature_id)
            return
        except Exception as exc:
            last = exc
            time.sleep(0.06 * (i + 1))
    raise RuntimeError(f"BNO08x: enable {name} failed after {attempts} tries: {last}") from last


def _drain_bno_packets(bno: Any, seconds: float) -> None:
    """Optional light drain before ``enable_feature``.

    **Avoid** huge ``max_packets`` loops: if the chip signals data-ready but the bus misbehaves,
    Adafruit's I2C reader can **block indefinitely** inside ``read_header``.
    """
    if seconds <= 0:
        return
    try:
        max_calls = int(os.environ.get("IMU_BOOT_DRAIN_MAX_CALLS", "12"), 0)
    except ValueError:
        max_calls = 12
    max_calls = max(1, min(max_calls, 40))
    per_call_max = int(os.environ.get("IMU_BOOT_DRAIN_MAX_PACKETS", "6"), 0) or 6
    per_call_max = max(1, min(per_call_max, 24))

    deadline = time.monotonic() + max(0.0, seconds)
    for _ in range(max_calls):
        if time.monotonic() >= deadline:
            break
        try:
            with i2c_bus.i2c_lock():
                if hasattr(bno, "_process_available_packets"):
                    bno._process_available_packets(max_packets=per_call_max)  # type: ignore[attr-defined]
        except Exception:
            pass
        time.sleep(0.04)


def _quat_to_euler_deg(qi: float, qj: float, qk: float, qr: float) -> tuple[float, float, float]:
    """Vector part (i,j,k) + real (r) → roll, pitch, yaw in degrees."""
    sinp = 2.0 * (qr * qj - qk * qi)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    roll = math.atan2(2.0 * (qr * qi + qj * qk), 1.0 - 2.0 * (qi * qi + qj * qj))
    yaw = math.atan2(2.0 * (qr * qk + qi * qj), 1.0 - 2.0 * (qj * qj + qk * qk))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def _init_imu_once() -> tuple[Any, Any]:
    from adafruit_bno08x import (  # type: ignore
        BNO_REPORT_ACCELEROMETER,
        BNO_REPORT_GAME_ROTATION_VECTOR,
        BNO_REPORT_GYROSCOPE,
        BNO_REPORT_MAGNETOMETER,
        BNO_REPORT_ROTATION_VECTOR,
    )
    from adafruit_bno08x.i2c import BNO08X_I2C  # type: ignore

    addr = int(os.environ.get("IMU_I2C_ADDR", "0x4A"), 0)
    lib_debug = os.environ.get("BNO08X_DEBUG", "").strip() == "1"
    post_open_delay = float(os.environ.get("IMU_POST_OPEN_DELAY_SEC", "0.35"))
    # Default **off**: long drains repeatedly call ``_process_available_packets`` and can hang in I2C read.
    boot_drain = float(os.environ.get("IMU_BOOT_DRAIN_SEC", "0"))
    try:
        ctor_cycles = int(os.environ.get("IMU_OPEN_DRAIN_CYCLES", "2"), 0)
    except ValueError:
        ctor_cycles = 2
    ctor_cycles = max(0, min(ctor_cycles, 6))
    ctor_max_pkt = int(os.environ.get("IMU_OPEN_DRAIN_MAX_PACKETS", "8"), 0) or 8
    ctor_max_pkt = max(1, min(ctor_max_pkt, 24))

    _init_progress("IMU: open I2C + BNO08x …")
    with i2c_bus.i2c_lock():
        i2c = i2c_bus.get_i2c()
        bno = BNO08X_I2C(i2c, address=addr, debug=lib_debug)
        if not lib_debug:
            try:
                bno._debug = False  # type: ignore[attr-defined]
            except Exception:
                pass
            def _noop_dbg(*_a: Any, **_k: Any) -> None:
                return None

            bno._dbg = _noop_dbg  # type: ignore[method-assign]
        for _ in range(ctor_cycles):
            if hasattr(bno, "_process_available_packets"):
                bno._process_available_packets(max_packets=ctor_max_pkt)  # type: ignore[attr-defined]

    if boot_drain > 0:
        _init_progress(f"IMU: boot drain {boot_drain:.2f}s (IMU_BOOT_DRAIN_SEC) …")
    _drain_bno_packets(bno, boot_drain)
    time.sleep(post_open_delay)

    # Accel → gyro → mag, then fusion (Adafruit / Hillcrest bring-up order; gyro-first hung some Pi+I2C setups).
    # If rotation_vector fails (mag / EMI), fall back to game_rotation_vector.
    bno._fsw_use_game_quat = False  # type: ignore[attr-defined]
    for feat in (
        BNO_REPORT_ACCELEROMETER,
        BNO_REPORT_GYROSCOPE,
        BNO_REPORT_MAGNETOMETER,
    ):
        with i2c_bus.i2c_lock():
            _enable_feature_retry(bno, feat)
    with i2c_bus.i2c_lock():
        try:
            _enable_feature_retry(bno, BNO_REPORT_ROTATION_VECTOR)
        except RuntimeError as exc:
            logger.warning(
                "IMU: rotation_vector not available (%s); enabling game_rotation_vector instead",
                exc,
            )
            _enable_feature_retry(bno, BNO_REPORT_GAME_ROTATION_VECTOR)
            bno._fsw_use_game_quat = True  # type: ignore[attr-defined]

    logger.info("IMU BNO08x OK at 0x%02x", addr)
    return i2c, bno


def init_imu() -> tuple[Any, Any]:
    try:
        rounds = int(os.environ.get("IMU_INIT_ROUNDS", "2"), 0)
    except ValueError:
        rounds = 2
    rounds = max(1, min(rounds, 4))
    last_exc: Optional[Exception] = None
    for attempt in range(rounds):
        try:
            return _init_imu_once()
        except RuntimeError as exc:
            last_exc = exc
            logger.warning("IMU: init round %d/%d failed: %s", attempt + 1, rounds, exc)
            if attempt + 1 < rounds:
                i2c_bus.reset_i2c()
                time.sleep(0.25 * (attempt + 1))
    raise RuntimeError(f"BNO08x: init failed after {rounds} round(s): {last_exc}") from last_exc


def read_sensor_data(bno) -> Any:
    """Return 12-tuple for `imuapp`, or False on soft failure."""
    r2d = 180.0 / math.pi
    for _ in range(4):
        try:
            with i2c_bus.i2c_lock():
                if hasattr(bno, "_process_available_packets"):
                    bno._process_available_packets(max_packets=6)  # type: ignore[attr-defined]
                if getattr(bno, "_fsw_use_game_quat", False):
                    qi, qj, qk, qr = bno.game_quaternion
                else:
                    qi, qj, qk, qr = bno.quaternion
                roll, pitch, yaw = _quat_to_euler_deg(float(qi), float(qj), float(qk), float(qr))
                ax, ay, az = bno.acceleration
                mx, my, mz = bno.magnetic
                gx, gy, gz = bno.gyro
            return (
                roll,
                pitch,
                yaw,
                float(ax),
                float(ay),
                float(az),
                float(mx),
                float(my),
                float(mz),
                float(gx) * r2d,
                float(gy) * r2d,
                float(gz) * r2d,
            )
        except Exception:
            time.sleep(0.002)
    return False


def reinit_imu(_i2c_old: Any, _bno_old: Any) -> tuple[Any, Any]:
    """Drop the cached bus so ``init_imu`` opens a fresh handle (``deinit`` alone left a dead singleton)."""
    i2c_bus.reset_i2c()
    return init_imu()


def imu_terminate(_i2c: Any) -> None:
    i2c_bus.reset_i2c()


if __name__ == "__main__":
    import logging

    from lib.sensor_cli import cli_period_sec

    logging.basicConfig(level=logging.INFO, format="%(message)s")
    os.environ.setdefault("IMU_INIT_PROGRESS", "1")
    print(
        "IMU: initializing BNO08x (15–60 s). If this hangs, set IMU_BOOT_DRAIN_SEC=0 (default), "
        "try FSW_I2C_FLOCK=0 for CLI-only tests, I2C 100 kHz, IMU_INIT_PROGRESS=1 shows each step.",
        flush=True,
    )
    try:
        _i2c, bno = init_imu()
    except Exception as exc:
        print(f"IMU: init failed: {exc}", flush=True)
        raise SystemExit(1) from exc
    print("IMU: OK, streaming...", flush=True)
    period = cli_period_sec()
    _last_fail_log = 0.0
    try:
        while True:
            s = read_sensor_data(bno)
            if s is False:
                now = time.monotonic()
                if now - _last_fail_log >= 2.0:
                    print(
                        "IMU: read failed (move module, check 0x4A/0x4B, BNO08X_DEBUG=1)",
                        flush=True,
                    )
                    _last_fail_log = now
            else:
                r, p, y, ax, ay, az, mx, my, mz, gx, gy, gz = s
                print(
                    f"rpy_deg={r:.2f},{p:.2f},{y:.2f} "
                    f"acc={ax:.3f},{ay:.3f},{az:.3f} "
                    f"mag={mx:.2f},{my:.2f},{mz:.2f} "
                    f"gyr_deg_s={gx:.3f},{gy:.3f},{gz:.3f}",
                    flush=True,
                )
            time.sleep(period)
    except KeyboardInterrupt:
        print("", flush=True)
    finally:
        imu_terminate(_i2c)
