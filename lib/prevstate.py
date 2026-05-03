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
Target_lat: float = 0.0
Target_lon: float = 0.0
PREV_PACKET_COUNT: int = 0
PREV_ST_TIMEDELTA: float = 0.0
STATE_OVERRIDE: Optional[int] = None
YAW_OFFSET: float = 0.0


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
    global STATE_OVERRIDE, YAW_OFFSET
    state_override = _read_int_env("STATE_OVERRIDE", -1)
    STATE_OVERRIDE = state_override if state_override >= 0 else None
    YAW_OFFSET = _read_float_env("YAW_OFFSET", 0.0)


refresh_runtime_overrides()


def _serialize() -> dict:
    return {
        "PREV_STATE": PREV_STATE,
        "PREV_ALT_CAL": PREV_ALT_CAL,
        "PREV_MAX_ALT": PREV_MAX_ALT,
        "Target_lat": Target_lat,
        "Target_lon": Target_lon,
        "PREV_PACKET_COUNT": PREV_PACKET_COUNT,
        "PREV_ST_TIMEDELTA": PREV_ST_TIMEDELTA,
    }


def _apply(payload: dict) -> None:
    global PREV_STATE, PREV_ALT_CAL, PREV_MAX_ALT
    global Target_lat, Target_lon, PREV_PACKET_COUNT, PREV_ST_TIMEDELTA

    PREV_STATE = int(payload.get("PREV_STATE", 0))
    PREV_ALT_CAL = float(payload.get("PREV_ALT_CAL", 0.0))
    PREV_MAX_ALT = float(payload.get("PREV_MAX_ALT", 0.0))
    Target_lat = float(payload.get("Target_lat", 0.0))
    Target_lon = float(payload.get("Target_lon", 0.0))
    PREV_PACKET_COUNT = int(payload.get("PREV_PACKET_COUNT", 0))
    PREV_ST_TIMEDELTA = float(payload.get("PREV_ST_TIMEDELTA", 0.0))


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
    global Target_lat, Target_lon
    Target_lat = float(lat)
    Target_lon = float(lon)
    _save()


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
    global Target_lat, Target_lon, PREV_PACKET_COUNT, PREV_ST_TIMEDELTA

    PREV_STATE = 0
    PREV_ALT_CAL = 0.0
    PREV_MAX_ALT = 0.0
    Target_lat = 0.0
    Target_lon = 0.0
    PREV_PACKET_COUNT = 0
    PREV_ST_TIMEDELTA = 0.0
    _save()
