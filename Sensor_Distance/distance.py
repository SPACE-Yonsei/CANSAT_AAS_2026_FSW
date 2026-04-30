"""VL53L0X ToF distance (mm) for `distanceapp` (optional hardware path)."""

from __future__ import annotations

import logging
import os
from typing import Any

logger = logging.getLogger(__name__)


def init_vl53() -> Any:
    import board  # type: ignore
    import busio  # type: ignore
    import adafruit_vl53l0x  # type: ignore

    i2c = busio.I2C(board.SCL, board.SDA)
    addr = int(os.environ.get("DISTANCE_I2C_ADDR", "0x29"), 0)
    sensor = adafruit_vl53l0x.VL53L0X(i2c, address=addr)
    logger.info("Distance VL53L0X OK at 0x%02x", addr)
    return {"i2c": i2c, "sensor": sensor}


def read_range_mm(dev: dict) -> int:
    r = int(dev["sensor"].range)
    if r <= 0:
        raise RuntimeError("VL53 invalid range")
    return r


def terminate_vl53(dev: dict) -> None:
    try:
        dev["i2c"].deinit()
    except Exception:
        pass
