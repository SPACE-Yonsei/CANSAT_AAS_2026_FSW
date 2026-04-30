"""BMP3xx I2C barometer driver for `barometerapp` (optional hardware path)."""

from __future__ import annotations

import logging
import os
from typing import Any

logger = logging.getLogger(__name__)


def init_bmp() -> Any:
    import board  # type: ignore
    import busio  # type: ignore
    from adafruit_bmp3xx import BMP3XX_I2C  # type: ignore

    addr = int(os.environ.get("BARO_I2C_ADDR", "0x77"), 0)
    i2c = busio.I2C(board.SCL, board.SDA)
    bmp = BMP3XX_I2C(i2c, address=addr)
    logger.info("Barometer BMP3xx OK at 0x%02x", addr)
    return {"i2c": i2c, "bmp": bmp}


def read_bmp(dev: dict) -> tuple[float, float, float]:
    bmp = dev["bmp"]
    pressure = float(bmp.pressure)
    temperature = float(bmp.temperature)
    altitude = float(bmp.altitude)
    return pressure, temperature, altitude


def terminate_bmp(dev: dict) -> None:
    try:
        dev["i2c"].deinit()
    except Exception:
        pass
