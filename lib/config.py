"""Core runtime configuration for FSW."""

from __future__ import annotations

import os


# GPIO map
BURNWIRE_GPIO = 5
EGG_SOLENOID_GPIO = 6
PARAFOIL_RIGHT_GPIO = 12
PARAFOIL_LEFT_GPIO = 13


# Relay active level (keep numeric for platform-agnostic compatibility)
RELAY_ACTIVATE_LEVEL = 1
RELAY_DEACTIVATE_LEVEL = 0


# Sensor/IO rates (Hz)
BAROMETER_RATE_HZ = 10
IMU_RATE_HZ = 10
GPS_RATE_HZ = 10
ELECTRO_RATE_HZ = 1
DISTANCE_RATE_HZ = 10
XBEE_RATE_HZ = 10
CAMERA_FPS = 30


def _read_int_env(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, default))
    except ValueError:
        return default


def _read_float_env(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except ValueError:
        return default


# Runtime overrides
STATE_OVERRIDE = _read_int_env("STATE_OVERRIDE", -1)
if STATE_OVERRIDE < 0:
    STATE_OVERRIDE = None
YAW_OFFSET = _read_float_env("YAW_OFFSET", 0.0)
