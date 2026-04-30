"""I2C barometer drivers for `barometerapp` (BMP3xx, BMP280, or BME280 at 0x77)."""

from __future__ import annotations

import logging
import os
from typing import Any

logger = logging.getLogger(__name__)


def _sea_level_altitude_m(pressure_hpa: float, sea_level_hpa: float = 1013.25) -> float:
    return 44330.0 * (1.0 - (pressure_hpa / sea_level_hpa) ** (1.0 / 5.255))


def init_bmp() -> Any:
    import board  # type: ignore
    import busio  # type: ignore

    addr = int(os.environ.get("BARO_I2C_ADDR", "0x77"), 0)
    chip = os.environ.get("BARO_CHIP", "").strip().lower()
    i2c = busio.I2C(board.SCL, board.SDA)

    def try_bmp3xx() -> dict:
        from adafruit_bmp3xx import BMP3XX_I2C  # type: ignore

        dev = BMP3XX_I2C(i2c, address=addr)
        return {"i2c": i2c, "kind": "bmp3xx", "sensor": dev}

    order: list[tuple[str, Any]] = []
    if chip == "bmp3xx":
        order = [("bmp3xx", try_bmp3xx)]
    else:
        order = [
            ("bmp3xx", try_bmp3xx),
        ]

    errors: list[str] = []
    for name, fn in order:
        try:
            dev = fn()
            logger.info("Barometer OK: %s at 0x%02x", name, addr)
            return dev
        except Exception as exc:
            errors.append(f"{name}: {exc}")

    try:
        i2c.deinit()
    except Exception:
        pass
    raise RuntimeError("no barometer chip responded; tried: " + "; ".join(errors))


def read_bmp(dev: dict) -> tuple[float, float, float]:
    kind = dev["kind"]
    sensor = dev["sensor"]
    if kind == "bmp3xx":
        return float(sensor.pressure), float(sensor.temperature), float(sensor.altitude)

    raise RuntimeError(f"unknown barometer kind {kind!r}")


def terminate_bmp(dev: dict) -> None:
    try:
        dev["i2c"].deinit()
    except Exception:
        pass
