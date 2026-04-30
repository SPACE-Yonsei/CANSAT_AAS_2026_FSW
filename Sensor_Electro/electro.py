"""INA228 / INA219 I2C power monitor for `electroapp`."""

from __future__ import annotations

import logging
import os
from typing import Any

logger = logging.getLogger(__name__)


def init_INA228(address: int | None = None) -> Any:
    import board  # type: ignore
    import busio  # type: ignore

    addr = int(os.environ.get("ELECTRO_I2C_ADDR", "0x40"), 0) if address is None else int(address)
    chip = os.environ.get("ELECTRO_CHIP", "").strip().lower()
    i2c = busio.I2C(board.SCL, board.SDA)
    errors: list[str] = []

    if chip in ("", "auto", "ina228"):
        try:
            from adafruit_ina228 import INA228  # type: ignore

            dev = INA228(i2c, address=addr)
            logger.info("Electro INA228 OK at 0x%02x", addr)
            return {"i2c": i2c, "kind": "ina228", "dev": dev}
        except Exception as exc:
            errors.append(f"ina228:{exc}")
            if chip == "ina228":
                try:
                    i2c.deinit()
                except Exception:
                    pass
                raise


    try:
        i2c.deinit()
    except Exception:
        pass
    raise RuntimeError("no power monitor found; " + "; ".join(errors))


def read_voltage(dev: dict) -> float:
    d = dev["dev"]
    if dev["kind"] == "ina228":
        return float(d.bus_voltage)
    raise RuntimeError(dev["kind"])


def read_current(dev: dict) -> float:
    return float(dev["dev"].current)


def read_power(dev: dict) -> float:
    d = dev["dev"]
    if dev["kind"] == "ina228":
        return float(d.power)
    raise RuntimeError(dev["kind"])


def terminate_INA228(dev: dict | None) -> None:
    if not dev:
        return
    try:
        dev["i2c"].deinit()
    except Exception:
        pass
