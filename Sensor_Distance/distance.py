"""Benewake TF-Luna I2C ToF (cm) for `distanceapp`.

Default 7-bit address **0x10**. Uses **trigger** mode (reg ``0x23`` = 1) and issues
``0x24`` = 1 before each read, then reads 6 bytes from ``0x00`` (dist, flux, temp)
per Benewake / budryerson TFLuna-I2C reference.

Set ``DISTANCE_TFL_TRIGGER=0`` to use continuous mode (no per-sample trigger).
"""

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

_REG_DATA = 0x00
_REG_MODE = 0x23
_REG_TRIGGER = 0x24
_DIST_MAX_CM = 1200


def _flux_min() -> int:
    """Weak-signal threshold; lower for dark targets / bench (e.g. ``DISTANCE_TFL_FLUX_MIN=50``)."""
    try:
        return max(0, int(os.environ.get("DISTANCE_TFL_FLUX_MIN", "100")))
    except ValueError:
        return 100


def _distance_raw_to_mm(dist_raw: int) -> int:
    """Convert 16-bit register value to mm.

    Default ``cm``: Benewake samples distance in centimeters; FSW uses mm.
    Set ``DISTANCE_TFL_DISTANCE_UNIT=mm`` if your firmware reports mm in the register.
    """
    unit = os.environ.get("DISTANCE_TFL_DISTANCE_UNIT", "cm").strip().lower()
    if unit == "mm":
        return int(dist_raw)
    return int(dist_raw) * 10


def _tfl_use_trigger() -> bool:
    return os.environ.get("DISTANCE_TFL_TRIGGER", "1").strip() != "0"


def init_tfluna() -> Any:
    addr = int(os.environ.get("DISTANCE_I2C_ADDR", "0x10"), 0)
    with i2c_bus.i2c_lock():
        i2c = i2c_bus.get_i2c()
        mode_byte = 0x01 if _tfl_use_trigger() else 0x00
        i2c.writeto(addr, bytes([_REG_MODE, mode_byte]))
        time.sleep(0.02)
    mode = "trigger" if _tfl_use_trigger() else "continuous"
    logger.info("Distance TF-Luna I2C OK at 0x%02x (%s mode)", addr, mode)
    return {"kind": "tfluna", "addr": addr}


def read_range_mm(dev: dict) -> int:
    """Return range in millimeters (register scaling via ``DISTANCE_TFL_DISTANCE_UNIT``)."""
    addr = int(dev["addr"])
    with i2c_bus.i2c_lock():
        i2c = i2c_bus.get_i2c()
        if _tfl_use_trigger():
            i2c.writeto(addr, bytes([_REG_TRIGGER, 0x01]))
            time.sleep(0.002)
        buf = bytearray(6)
        i2c.writeto_then_readfrom(addr, bytes([_REG_DATA]), buf, out_end=1, in_end=6)

    dist_raw = int(buf[0]) | (int(buf[1]) << 8)
    flux = int(buf[2]) | (int(buf[3]) << 8)
    flux_min = _flux_min()

    if dist_raw == 0xFFFF:
        raise RuntimeError(f"TF-Luna invalid distance raw={dist_raw}")
    mm = _distance_raw_to_mm(dist_raw)
    if os.environ.get("DISTANCE_TFL_DISTANCE_UNIT", "cm").strip().lower() != "mm":
        if dist_raw > _DIST_MAX_CM:
            raise RuntimeError(f"TF-Luna invalid distance cm={dist_raw}")
    elif mm > 80000:
        raise RuntimeError(f"TF-Luna invalid distance mm={mm}")

    if flux_min and flux < flux_min:
        raise RuntimeError(f"TF-Luna weak signal flux={flux}")
    if flux > 0x8000 or flux == 0xFFFF:
        raise RuntimeError(f"TF-Luna flux out of range flux={flux}")

    return mm


def terminate_tfluna(_dev: dict) -> None:
    """Bus is shared; do not deinit here."""


if __name__ == "__main__":
    import logging

    from lib.sensor_cli import cli_period_sec

    logging.basicConfig(level=logging.INFO, format="%(message)s")
    print("Distance: initializing TF-Luna I2C...", flush=True)
    try:
        d = init_tfluna()
    except Exception as exc:
        print(f"Distance: init failed: {exc}", flush=True)
        raise SystemExit(1) from exc
    print("Distance: OK, streaming...", flush=True)
    period = cli_period_sec()
    try:
        while True:
            try:
                mm = read_range_mm(d)
                print(f"range_mm={mm}", flush=True)
            except Exception as exc:
                print(f"range error: {exc}", flush=True)
            time.sleep(period)
    except KeyboardInterrupt:
        print("", flush=True)
