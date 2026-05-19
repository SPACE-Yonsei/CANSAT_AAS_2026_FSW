"""One I2C handle per process (Blinka: prefer ``board.I2C()``).

Also provides a cross-process lock so baro / power / IMU / ToF subprocesses
do not interleave SMBus transactions on the same bus (common failure mode
with multiprocessing + Blinka).
"""

from __future__ import annotations

import logging
import os
import sys
import warnings
from contextlib import contextmanager
from typing import Any

# Blinka always warns when CircuitPython code sets I2C frequency; Linux SMBus ignores it.
warnings.filterwarnings(
    "ignore",
    message="I2C frequency is not settable in python, ignoring!",
    category=RuntimeWarning,
)

logger = logging.getLogger(__name__)

_i2c: Any = None
_lock_fp: Any = None


def _flock_enabled() -> bool:
    if os.environ.get("FSW_I2C_FLOCK", "1").strip() == "0":
        return False
    return sys.platform != "win32"


try:
    import fcntl  # type: ignore
except ImportError:
    fcntl = None  # type: ignore


def reset_i2c() -> None:
    """Clear the cached bus after ``deinit`` or before a full reopen (e.g. IMU reinit)."""
    global _i2c
    if _i2c is not None:
        try:
            _i2c.deinit()
        except Exception:
            pass
    _i2c = None


def get_i2c() -> Any:
    """Return a process-wide I2C bus. Prefer ``reset_i2c()`` over raw ``deinit``.

    Set ``FSW_I2C_BUS=1`` (etc.) to force ``/dev/i2c-N`` via ``adafruit_extended_bus``.
    This fixes cases where ``i2cdetect -y 1`` shows chips but ``board.I2C()`` talks to a
    different port (common on SBCs / custom Blinka pin maps).
    """
    global _i2c
    if _i2c is not None:
        return _i2c
    bus_raw = os.environ.get("FSW_I2C_BUS", "1").strip()
    if bus_raw:
        try:
            from adafruit_extended_bus import ExtendedI2C  # type: ignore

            bus_id = int(bus_raw, 0)
            _i2c = ExtendedI2C(bus_id)
            logger.info("I2C: ExtendedI2C(%s) -> /dev/i2c-%s", bus_raw, bus_id)
            return _i2c
        except Exception as exc:
            logger.warning(
                "FSW_I2C_BUS=%s failed (%s). pip install adafruit-extended-bus "
                "or unset FSW_I2C_BUS. Falling back to board.I2C().",
                bus_raw,
                exc,
            )

    import board  # type: ignore

    try:
        _i2c = board.I2C()
    except Exception:
        import busio  # type: ignore

        _i2c = busio.I2C(board.SCL, board.SDA)
    return _i2c


@contextmanager
def i2c_lock():
    """Serialize I2C across FSW processes (Linux ``flock``). No-op on Windows or if disabled."""
    if fcntl is None or not _flock_enabled():
        yield
        return
    global _lock_fp
    path = os.environ.get("FSW_I2C_LOCK_FILE", "/tmp/fsw_i2c.lock")
    locked = False
    try:
        if _lock_fp is None:
            _lock_fp = open(path, "a+", encoding="utf-8")
        fcntl.flock(_lock_fp.fileno(), fcntl.LOCK_EX)
        locked = True
    except OSError as exc:
        logger.warning("I2C flock failed (%s); continuing without cross-process lock", exc)
    try:
        yield
    finally:
        if locked and _lock_fp is not None:
            try:
                fcntl.flock(_lock_fp.fileno(), fcntl.LOCK_UN)
            except OSError:
                pass
