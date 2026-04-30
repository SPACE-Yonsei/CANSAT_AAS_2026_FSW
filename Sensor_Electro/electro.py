"""INA228 I2C power monitor for `electroapp`."""

from __future__ import annotations

import logging
import os
import sys
import time
from pathlib import Path
from typing import Any

_REPO = Path(__file__).resolve().parents[1]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

from lib import i2c_bus

logger = logging.getLogger(__name__)


def init_INA228(address: int | None = None) -> Any:
    """Open INA228 at ``ELECTRO_I2C_ADDR`` (default ``0x40``)."""
    from adafruit_ina228 import INA228  # type: ignore

    addr = int(os.environ.get("ELECTRO_I2C_ADDR", "0x40"), 0) if address is None else int(address)
    with i2c_bus.i2c_lock():
        i2c = i2c_bus.get_i2c()
        dev = INA228(i2c, address=addr)
    logger.info("Electro INA228 OK at 0x%02x", addr)
    return {"i2c": i2c, "kind": "ina228", "dev": dev}


def read_voltage_current_power(dev: dict) -> tuple[float, float, float]:
    """One lock, one burst — avoids interleaved reads with other apps on the bus."""
    d = dev["dev"]
    with i2c_bus.i2c_lock():
        v = float(d.bus_voltage)
        c = float(d.current)
        p = float(d.power)
    return v, c, p


def read_voltage(dev: dict) -> float:
    return read_voltage_current_power(dev)[0]


def read_current(dev: dict) -> float:
    return read_voltage_current_power(dev)[1]


def read_power(dev: dict) -> float:
    return read_voltage_current_power(dev)[2]


def terminate_INA228(_dev: dict | None) -> None:
    """Bus is shared; do not deinit here."""


if __name__ == "__main__":
    import logging

    from lib.sensor_cli import cli_period_sec

    logging.basicConfig(level=logging.INFO, format="%(message)s")
    print("Electro: initializing INA228...", flush=True)
    try:
        dev = init_INA228()
    except ImportError as exc:
        if "ina228" in str(exc).lower():
            print("Install: pip install adafruit-circuitpython-ina228", flush=True)
        raise SystemExit(1) from exc
    except Exception as exc:
        print(f"Electro: init failed: {exc}", flush=True)
        raise SystemExit(1) from exc
    print("Electro: OK, streaming...", flush=True)
    period = cli_period_sec()
    try:
        while True:
            v, c, p = read_voltage_current_power(dev)
            print(f"bus_V={v:.4f} A={c:.4f} W={p:.4f}", flush=True)
            time.sleep(period)
    except KeyboardInterrupt:
        print("", flush=True)
