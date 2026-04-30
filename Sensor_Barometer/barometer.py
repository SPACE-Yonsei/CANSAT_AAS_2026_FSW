"""BMP3xx I2C barometer (e.g. BMP390) for `barometerapp`."""

from __future__ import annotations

import logging
import os
import time
from typing import Any

from lib import i2c_bus

logger = logging.getLogger(__name__)


def init_bmp() -> Any:
    """Open BMP3xx at ``BARO_I2C_ADDR`` (default ``0x77``)."""
    from adafruit_bmp3xx import BMP3XX_I2C  # type: ignore

    addr = int(os.environ.get("BARO_I2C_ADDR", "0x77"), 0)
    retries = max(1, int(os.environ.get("BARO_INIT_RETRIES", "8"), 10))
    last_exc: Exception | None = None
    i2c: Any = None
    sensor: Any = None
    for attempt in range(retries):
        try:
            with i2c_bus.i2c_lock():
                i2c = i2c_bus.get_i2c()
                sensor = BMP3XX_I2C(i2c, address=addr)
            last_exc = None
            break
        except Exception as exc:
            last_exc = exc
            time.sleep(0.06 * (attempt + 1))
    if last_exc is not None:
        raise last_exc
    logger.info("Barometer BMP3xx (e.g. BMP390) OK at 0x%02x", addr)
    return {"i2c": i2c, "kind": "bmp3xx", "sensor": sensor}


def read_bmp(dev: dict) -> tuple[float, float, float]:
    sensor = dev["sensor"]
    with i2c_bus.i2c_lock():
        p = float(sensor.pressure)
        t = float(sensor.temperature)
        a = float(sensor.altitude)
    return p, t, a


def terminate_bmp(_dev: dict) -> None:
    """Bus is shared; do not deinit here."""
