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
_FLUX_MIN = 100
_DIST_MAX_CM = 1200


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
    """Return range in millimeters (TF-Luna reports centimeters)."""
    addr = int(dev["addr"])
    with i2c_bus.i2c_lock():
        i2c = i2c_bus.get_i2c()
        if _tfl_use_trigger():
            i2c.writeto(addr, bytes([_REG_TRIGGER, 0x01]))
            time.sleep(0.002)
        buf = bytearray(6)
        i2c.writeto_then_readfrom(addr, bytes([_REG_DATA]), buf, out_end=1, in_end=6)

    dist_cm = int(buf[0]) | (int(buf[1]) << 8)
    flux = int(buf[2]) | (int(buf[3]) << 8)

    if dist_cm == 0xFFFF or dist_cm > _DIST_MAX_CM:
        raise RuntimeError(f"TF-Luna invalid distance cm={dist_cm}")
    if flux < _FLUX_MIN:
        raise RuntimeError(f"TF-Luna weak signal flux={flux}")
    if flux > 0x8000 or flux == 0xFFFF:
        raise RuntimeError(f"TF-Luna flux out of range flux={flux}")

    return int(dist_cm) * 10


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
