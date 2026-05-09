"""Persistent mission state storage for reboot recovery."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Optional


_STATE_FILE = Path(__file__).with_name("prevstate.json")


# Public state variables used by other modules
PREV_STATE: int = 0
PREV_ALT_CAL: float = 0.0
PREV_MAX_ALT: float = 0.0
PREV_TARGET_LAT: float = 0.0
PREV_TARGET_LON: float = 0.0
PREV_PACKET_COUNT: int = 0
PREV_ST_TIMEDELTA: float = 0.0
PREV_YAW_OFFSET: float = 0.0
PREV_MOTOR_ENABLED: int = 1
PREV_SOLENOID_COUNT: int = 0
PREV_SOLENOID_DONE: int = 0
PREV_START_LAT: float = 0.0
PREV_START_LON: float = 0.0
PREV_START_LOCKED: int = 0
STATE_OVERRIDE: Optional[int] = None

# Backward-compatible aliases (prefer PREV_* fields in new code)
Target_lat: float = 0.0
Target_lon: float = 0.0
YAW_OFFSET: float = 0.0


def _sync_legacy_aliases() -> None:
    global Target_lat, Target_lon, YAW_OFFSET
    Target_lat = float(PREV_TARGET_LAT)
    Target_lon = float(PREV_TARGET_LON)
    YAW_OFFSET = float(PREV_YAW_OFFSET)


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


def refresh_runtime_overrides() -> None:
    """Load non-persistent runtime overrides from environment variables."""
    global STATE_OVERRIDE, PREV_YAW_OFFSET
    state_override = _read_int_env("STATE_OVERRIDE", -1)
    STATE_OVERRIDE = state_override if state_override >= 0 else None
    if "YAW_OFFSET" in os.environ:
        PREV_YAW_OFFSET = _read_float_env("YAW_OFFSET", PREV_YAW_OFFSET)
    _sync_legacy_aliases()


refresh_runtime_overrides()


def _serialize() -> dict:
    return {
        "PREV_STATE": PREV_STATE,
        "PREV_ALT_CAL": PREV_ALT_CAL,
        "PREV_MAX_ALT": PREV_MAX_ALT,
        "PREV_TARGET_LAT": PREV_TARGET_LAT,
        "PREV_TARGET_LON": PREV_TARGET_LON,
        "PREV_PACKET_COUNT": PREV_PACKET_COUNT,
        "PREV_ST_TIMEDELTA": PREV_ST_TIMEDELTA,
        "PREV_YAW_OFFSET": PREV_YAW_OFFSET,
        "PREV_MOTOR_ENABLED": PREV_MOTOR_ENABLED,
        "PREV_SOLENOID_COUNT": PREV_SOLENOID_COUNT,
        "PREV_SOLENOID_DONE": PREV_SOLENOID_DONE,
        "PREV_START_LAT": PREV_START_LAT,
        "PREV_START_LON": PREV_START_LON,
        "PREV_START_LOCKED": PREV_START_LOCKED,
    }


def _apply(payload: dict) -> None:
    global PREV_STATE, PREV_ALT_CAL, PREV_MAX_ALT
    global PREV_TARGET_LAT, PREV_TARGET_LON, PREV_PACKET_COUNT, PREV_ST_TIMEDELTA
    global PREV_YAW_OFFSET, PREV_MOTOR_ENABLED, PREV_SOLENOID_COUNT, PREV_SOLENOID_DONE
    global PREV_START_LAT, PREV_START_LON, PREV_START_LOCKED

    PREV_STATE = int(payload.get("PREV_STATE", 0))
    PREV_ALT_CAL = float(payload.get("PREV_ALT_CAL", 0.0))
    PREV_MAX_ALT = float(payload.get("PREV_MAX_ALT", 0.0))
    PREV_TARGET_LAT = float(payload.get("PREV_TARGET_LAT", payload.get("Target_lat", 0.0)))
    PREV_TARGET_LON = float(payload.get("PREV_TARGET_LON", payload.get("Target_lon", 0.0)))
    PREV_PACKET_COUNT = int(payload.get("PREV_PACKET_COUNT", 0))
    PREV_ST_TIMEDELTA = float(payload.get("PREV_ST_TIMEDELTA", 0.0))
    PREV_YAW_OFFSET = float(payload.get("PREV_YAW_OFFSET", payload.get("YAW_OFFSET", 0.0)))
    PREV_MOTOR_ENABLED = 1 if int(payload.get("PREV_MOTOR_ENABLED", 1)) else 0
    PREV_SOLENOID_COUNT = max(0, int(payload.get("PREV_SOLENOID_COUNT", 0)))
    PREV_SOLENOID_DONE = 1 if int(payload.get("PREV_SOLENOID_DONE", 0)) else 0
    PREV_START_LAT = float(payload.get("PREV_START_LAT", 0.0))
    PREV_START_LON = float(payload.get("PREV_START_LON", 0.0))
    PREV_START_LOCKED = 1 if int(payload.get("PREV_START_LOCKED", 0)) else 0
    _sync_legacy_aliases()


def _save() -> None:
    _STATE_FILE.write_text(json.dumps(_serialize(), ensure_ascii=True, indent=2), encoding="utf-8")


def init_prevstate() -> None:
    refresh_runtime_overrides()

    if _STATE_FILE.exists():
        try:
            _apply(json.loads(_STATE_FILE.read_text(encoding="utf-8")))
        except (json.JSONDecodeError, OSError, ValueError):
            reset_prevstate()
            return
    else:
        _save()

    # Environment override must win over persisted value when provided.
    refresh_runtime_overrides()

    # Runtime override for test/safety operation
    if STATE_OVERRIDE is not None:
        update_prevstate(STATE_OVERRIDE)


def update_prevstate(state: int) -> None:
    global PREV_STATE
    PREV_STATE = int(state)
    _save()


def update_altcal(alt: float) -> None:
    global PREV_ALT_CAL
    PREV_ALT_CAL = float(alt)
    _save()


def update_maxalt(alt: float) -> None:
    global PREV_MAX_ALT
    PREV_MAX_ALT = float(alt)
    _save()


def update_target_gps(lat: float, lon: float) -> None:
    global PREV_TARGET_LAT, PREV_TARGET_LON
    PREV_TARGET_LAT = float(lat)
    PREV_TARGET_LON = float(lon)
    _sync_legacy_aliases()
    _save()


def update_yaw_offset(offset_deg: float) -> None:
    global PREV_YAW_OFFSET
    PREV_YAW_OFFSET = float(offset_deg)
    _sync_legacy_aliases()
    _save()


def update_motor_enabled(enabled: bool) -> None:
    global PREV_MOTOR_ENABLED
    PREV_MOTOR_ENABLED = 1 if bool(enabled) else 0
    _save()


def update_solenoid_state(count: int, done: bool) -> None:
    global PREV_SOLENOID_COUNT, PREV_SOLENOID_DONE
    PREV_SOLENOID_COUNT = max(0, int(count))
    PREV_SOLENOID_DONE = 1 if bool(done) else 0
    _save()


def update_start_point(lat: float, lon: float, locked: bool) -> None:
    global PREV_START_LAT, PREV_START_LON, PREV_START_LOCKED
    PREV_START_LAT = float(lat)
    PREV_START_LON = float(lon)
    PREV_START_LOCKED = 1 if bool(locked) else 0
    _save()


def clear_start_point() -> None:
    update_start_point(0.0, 0.0, False)


def get_target_gps() -> tuple[float, float]:
    return PREV_TARGET_LAT, PREV_TARGET_LON


def get_start_point() -> Optional[tuple[float, float]]:
    if PREV_START_LOCKED != 1:
        return None
    return PREV_START_LAT, PREV_START_LON


def is_motor_enabled() -> bool:
    return PREV_MOTOR_ENABLED == 1


def get_solenoid_state() -> tuple[int, bool]:
    return PREV_SOLENOID_COUNT, PREV_SOLENOID_DONE == 1


def get_yaw_offset() -> float:
    return PREV_YAW_OFFSET


def update_packet_count(count: int) -> None:
    global PREV_PACKET_COUNT
    PREV_PACKET_COUNT = int(count)
    _save()


def update_st_timedelta(seconds: float) -> None:
    global PREV_ST_TIMEDELTA
    PREV_ST_TIMEDELTA = float(seconds)
    _save()


def reset_prevstate() -> None:
    global PREV_STATE, PREV_ALT_CAL, PREV_MAX_ALT
    global PREV_TARGET_LAT, PREV_TARGET_LON, PREV_PACKET_COUNT, PREV_ST_TIMEDELTA
    global PREV_YAW_OFFSET, PREV_MOTOR_ENABLED, PREV_SOLENOID_COUNT, PREV_SOLENOID_DONE
    global PREV_START_LAT, PREV_START_LON, PREV_START_LOCKED

    PREV_STATE = 0
    PREV_ALT_CAL = 0.0
    PREV_MAX_ALT = 0.0
    PREV_TARGET_LAT = 0.0
    PREV_TARGET_LON = 0.0
    PREV_PACKET_COUNT = 0
    PREV_ST_TIMEDELTA = 0.0
    PREV_YAW_OFFSET = 0.0
    PREV_MOTOR_ENABLED = 1
    PREV_SOLENOID_COUNT = 0
    PREV_SOLENOID_DONE = 0
    PREV_START_LAT = 0.0
    PREV_START_LON = 0.0
    PREV_START_LOCKED = 0
    _sync_legacy_aliases()
    _save()


def reset_control() -> None:
    """Test utility: reset mission control-related persisted values.

    Keeps telemetry packet/timebase counters intact for communication tests.
    """
    global PREV_STATE, PREV_ALT_CAL, PREV_MAX_ALT, PREV_TARGET_LAT, PREV_TARGET_LON
    global PREV_SOLENOID_COUNT, PREV_SOLENOID_DONE
    global PREV_START_LAT, PREV_START_LON, PREV_START_LOCKED
    PREV_STATE = 0
    PREV_ALT_CAL = 0.0
    PREV_MAX_ALT = 0.0
    PREV_TARGET_LAT = 0.0
    PREV_TARGET_LON = 0.0
    PREV_SOLENOID_COUNT = 0
    PREV_SOLENOID_DONE = 0
    PREV_START_LAT = 0.0
    PREV_START_LON = 0.0
    PREV_START_LOCKED = 0
    _sync_legacy_aliases()
    _save()
