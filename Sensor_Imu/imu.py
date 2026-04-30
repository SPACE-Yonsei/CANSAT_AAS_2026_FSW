"""BNO08x I2C IMU driver for `imuapp` (optional hardware path)."""

from __future__ import annotations

import logging
import math
import os
import time
from typing import Any

logger = logging.getLogger(__name__)


def _quat_to_euler_deg(qi: float, qj: float, qk: float, qr: float) -> tuple[float, float, float]:
    """Vector part (i,j,k) + real (r) → roll, pitch, yaw in degrees."""
    sinp = 2.0 * (qr * qj - qk * qi)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    roll = math.atan2(2.0 * (qr * qi + qj * qk), 1.0 - 2.0 * (qi * qi + qj * qj))
    yaw = math.atan2(2.0 * (qr * qk + qi * qj), 1.0 - 2.0 * (qj * qj + qk * qk))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def init_imu() -> tuple[Any, Any]:
    import board  # type: ignore
    import busio  # type: ignore
    from adafruit_bno08x import (  # type: ignore
        BNO_REPORT_ACCELEROMETER,
        BNO_REPORT_GYROSCOPE,
        BNO_REPORT_MAGNETOMETER,
        BNO_REPORT_ROTATION_VECTOR,
    )
    from adafruit_bno08x.i2c import BNO08X_I2C  # type: ignore

    addr = int(os.environ.get("IMU_I2C_ADDR", "0x4A"), 0)
    i2c = busio.I2C(board.SCL, board.SDA)
    lib_debug = os.environ.get("BNO08X_DEBUG", "").strip() == "1"
    bno = BNO08X_I2C(i2c, address=addr, debug=lib_debug)
    # Adafruit driver prints verbose SHTP "Packet" dumps when debug is on; force off unless BNO08X_DEBUG=1.
    if not lib_debug:
        try:
            bno._debug = False  # type: ignore[attr-defined]
        except Exception:
            pass
        bno._dbg = lambda *_a, **_k: None  # type: ignore[method-assign]
    for feat in (
        BNO_REPORT_ACCELEROMETER,
        BNO_REPORT_GYROSCOPE,
        BNO_REPORT_MAGNETOMETER,
        BNO_REPORT_ROTATION_VECTOR,
    ):
        bno.enable_feature(feat)
    logger.info("IMU BNO08x OK at 0x%02x", addr)
    return i2c, bno


def read_sensor_data(bno) -> Any:
    """Return 12-tuple for `imuapp`, or False on soft failure."""
    r2d = 180.0 / math.pi
    for _ in range(4):
        try:
            if hasattr(bno, "_process_available_packets"):
                bno._process_available_packets(max_packets=12)  # type: ignore[attr-defined]
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


def reinit_imu(i2c_old: Any, _bno_old: Any) -> tuple[Any, Any]:
    try:
        if i2c_old is not None:
            i2c_old.deinit()
    except Exception:
        pass
    return init_imu()


def imu_terminate(i2c: Any) -> None:
    try:
        if i2c is not None:
            i2c.deinit()
    except Exception:
        pass
