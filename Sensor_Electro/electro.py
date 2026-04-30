"""INA228 access shim.

This module provides a minimal interface expected by `electroapp`.
If hardware libs are unavailable, callers should handle exceptions/fallback.
"""

from __future__ import annotations


def init_INA228(address: int = 0x40):
    try:
        import board  # type: ignore
        import busio  # type: ignore
    except Exception as exc:
        raise RuntimeError("INA228 dependencies unavailable") from exc

    # Placeholder object; real register-level implementation can be added here.
    return {"address": address, "i2c": busio.I2C(board.SCL, board.SDA)}


def read_voltage(_dev) -> float:
    raise RuntimeError("INA228 real read not implemented yet")


def read_current(_dev) -> float:
    raise RuntimeError("INA228 real read not implemented yet")


def read_power(_dev) -> float:
    raise RuntimeError("INA228 real read not implemented yet")


def terminate_INA228(dev) -> None:
    try:
        dev["i2c"].deinit()
    except Exception:
        pass
