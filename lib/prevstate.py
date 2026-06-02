"""Persistent mission state storage for reboot recovery.

Designed for concurrent access from multiple FSW subprocesses. Every persisted
update goes through a *read-merge-write* path under a cross-process advisory
file lock so that concurrent writers (CommApp packet count, FlightLogic state,
MotorApp start-point lock, etc.) cannot clobber each other's fields.

Lock primitives:
    POSIX: ``fcntl.flock`` on a sidecar ``<prevstate>.lock`` file.
    Windows: ``msvcrt.locking`` on the same sidecar file.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict, Optional


_STATE_FILE = Path(__file__).with_name("prevstate.json")

FIX_TARGET_GPS: bool = True
DEFAULT_TARGET_LAT: float = 38.376000
DEFAULT_TARGET_LON: float = -79.607872


# Public state variables used by other modules
PREV_STATE: int = 0
PREV_ALT_CAL: float = 0.0
PREV_MAX_ALT: float = 0.0
PREV_TARGET_LAT: float = DEFAULT_TARGET_LAT
PREV_TARGET_LON: float = DEFAULT_TARGET_LON
PREV_PACKET_COUNT: int = 0
PREV_ST_TIMEDELTA: float = 0.0
PREV_YAW_OFFSET: float = 0.0
PREV_MOTOR_ENABLED: int = 1
PREV_SOLENOID_COUNT: int = 0
PREV_SOLENOID_DONE: int = 0
PREV_START_LAT: float = 0.0
PREV_START_LON: float = 0.0
PREV_START_LOCKED: int = 0
PREV_BEARING: float = float("nan")
STATE_OVERRIDE: Optional[int] = None

# Backward-compatible aliases (prefer PREV_* fields in new code)
Target_lat: float = DEFAULT_TARGET_LAT
Target_lon: float = DEFAULT_TARGET_LON
YAW_OFFSET: float = 0.0


def _sync_legacy_aliases() -> None:
    global Target_lat, Target_lon, YAW_OFFSET
    Target_lat = float(PREV_TARGET_LAT)
    Target_lon = float(PREV_TARGET_LON)
    YAW_OFFSET = float(PREV_YAW_OFFSET)


def _target_defaults() -> tuple[float, float]:
    return DEFAULT_TARGET_LAT, DEFAULT_TARGET_LON


def _valid_target(lat: float, lon: float) -> bool:
    return (-90.0 <= lat <= 90.0
            and -180.0 <= lon <= 180.0
            and not (abs(lat) < 1e-9 and abs(lon) < 1e-9))


def _resolve_target(payload: Dict[str, Any]) -> tuple[float, float]:
    if FIX_TARGET_GPS:
        return _target_defaults()
    lat = float(payload.get("PREV_TARGET_LAT", payload.get("Target_lat", DEFAULT_TARGET_LAT)))
    lon = float(payload.get("PREV_TARGET_LON", payload.get("Target_lon", DEFAULT_TARGET_LON)))
    if not _valid_target(lat, lon):
        return _target_defaults()
    return lat, lon


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


# ── Cross-process file lock ───────────────────────────────────────────────────

if os.name == "nt":
    import msvcrt  # type: ignore[import-not-found]

    def _platform_lock(fp) -> None:
        # LK_LOCK blocks (with retry) until the byte range is available.
        msvcrt.locking(fp.fileno(), msvcrt.LK_LOCK, 1)

    def _platform_unlock(fp) -> None:
        try:
            fp.seek(0)
            msvcrt.locking(fp.fileno(), msvcrt.LK_UNLCK, 1)
        except OSError:
            pass
else:
    import fcntl  # type: ignore[import-not-found]

    def _platform_lock(fp) -> None:
        fcntl.flock(fp.fileno(), fcntl.LOCK_EX)

    def _platform_unlock(fp) -> None:
        try:
            fcntl.flock(fp.fileno(), fcntl.LOCK_UN)
        except OSError:
            pass


def _lock_path() -> Path:
    # Tied to current ``_STATE_FILE`` so tests that override the path keep working.
    return _STATE_FILE.with_suffix(_STATE_FILE.suffix + ".lock")


class _FileLock:
    """Best-effort cross-process exclusive lock on a sidecar file."""

    def __enter__(self) -> "_FileLock":
        path = _lock_path()
        path.parent.mkdir(parents=True, exist_ok=True)
        # 'a+' creates the lock file if absent; we never read its contents.
        self._fp = path.open("a+", encoding="utf-8")
        try:
            self._fp.seek(0)
            if os.name == "nt":
                # On Windows the byte at offset 0 must exist for LK_LOCK; ensure
                # the file is at least 1 byte long.
                self._fp.write("\0")
                self._fp.flush()
                self._fp.seek(0)
            _platform_lock(self._fp)
        except Exception:
            try:
                self._fp.close()
            except Exception:
                pass
            self._fp = None  # type: ignore[assignment]
            raise
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if getattr(self, "_fp", None) is None:
            return
        try:
            _platform_unlock(self._fp)
        finally:
            try:
                self._fp.close()
            except Exception:
                pass


# ── Serialization ─────────────────────────────────────────────────────────────


def _serialize() -> Dict[str, Any]:
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
        "PREV_BEARING": PREV_BEARING if PREV_BEARING == PREV_BEARING else None,
    }


def _apply(payload: Dict[str, Any]) -> None:
    global PREV_STATE, PREV_ALT_CAL, PREV_MAX_ALT
    global PREV_TARGET_LAT, PREV_TARGET_LON, PREV_PACKET_COUNT, PREV_ST_TIMEDELTA
    global PREV_YAW_OFFSET, PREV_MOTOR_ENABLED, PREV_SOLENOID_COUNT, PREV_SOLENOID_DONE
    global PREV_START_LAT, PREV_START_LON, PREV_START_LOCKED, PREV_BEARING

    PREV_STATE = int(payload.get("PREV_STATE", 0))
    PREV_ALT_CAL = float(payload.get("PREV_ALT_CAL", 0.0))
    PREV_MAX_ALT = float(payload.get("PREV_MAX_ALT", 0.0))
    PREV_TARGET_LAT, PREV_TARGET_LON = _resolve_target(payload)
    PREV_PACKET_COUNT = int(payload.get("PREV_PACKET_COUNT", 0))
    PREV_ST_TIMEDELTA = float(payload.get("PREV_ST_TIMEDELTA", 0.0))
    PREV_YAW_OFFSET = float(payload.get("PREV_YAW_OFFSET", payload.get("YAW_OFFSET", 0.0)))
    PREV_MOTOR_ENABLED = 1 if int(payload.get("PREV_MOTOR_ENABLED", 1)) else 0
    PREV_SOLENOID_COUNT = max(0, int(payload.get("PREV_SOLENOID_COUNT", 0)))
    PREV_SOLENOID_DONE = 1 if int(payload.get("PREV_SOLENOID_DONE", 0)) else 0
    PREV_START_LAT = float(payload.get("PREV_START_LAT", 0.0))
    PREV_START_LON = float(payload.get("PREV_START_LON", 0.0))
    PREV_START_LOCKED = 1 if int(payload.get("PREV_START_LOCKED", 0)) else 0
    _raw_bearing = payload.get("PREV_BEARING", None)
    PREV_BEARING = float(_raw_bearing) if _raw_bearing is not None else float("nan")
    _sync_legacy_aliases()


def _atomic_write(payload: Dict[str, Any]) -> None:
    tmp = _STATE_FILE.with_suffix(_STATE_FILE.suffix + ".tmp")
    tmp.parent.mkdir(parents=True, exist_ok=True)
    tmp.write_text(json.dumps(payload, ensure_ascii=True, indent=2), encoding="utf-8")
    os.replace(str(tmp), str(_STATE_FILE))


def _read_disk_payload() -> Optional[Dict[str, Any]]:
    if not _STATE_FILE.exists():
        return None
    try:
        return json.loads(_STATE_FILE.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError, ValueError):
        return None


def _save() -> None:
    """Write the *current* in-memory snapshot. Held under file lock by callers."""
    _atomic_write(_serialize())


def _atomic_update(field_updates: Dict[str, Any]) -> None:
    """Read disk, merge ``field_updates``, write back atomically.

    All three steps happen under the cross-process file lock so concurrent
    writers from sibling processes cannot lose updates. Runtime env-only
    overrides (``YAW_OFFSET``/``STATE_OVERRIDE``) are re-applied after the
    in-memory snapshot is refreshed so the disk read does not clobber them.
    """
    with _FileLock():
        current = _read_disk_payload() or _serialize()
        current.update(field_updates)
        _atomic_write(current)
        _apply(current)
    refresh_runtime_overrides()


# ── Public API ────────────────────────────────────────────────────────────────


def init_prevstate() -> None:
    refresh_runtime_overrides()

    with _FileLock():
        payload = _read_disk_payload()
        if payload is None:
            # File missing or corrupt: write a fresh default snapshot.
            _apply({})
            _atomic_write(_serialize())
        else:
            _apply(payload)
            _atomic_write(_serialize())

    # Environment override must win over persisted value when provided.
    refresh_runtime_overrides()

    if STATE_OVERRIDE is not None:
        update_prevstate(STATE_OVERRIDE)


def update_prevstate(state: int) -> None:
    _atomic_update({"PREV_STATE": int(state)})


def update_altcal(alt: float) -> None:
    _atomic_update({"PREV_ALT_CAL": float(alt)})


def update_maxalt(alt: float) -> None:
    _atomic_update({"PREV_MAX_ALT": float(alt)})


def update_target_gps(lat: float, lon: float) -> None:
    if FIX_TARGET_GPS:
        lat, lon = _target_defaults()
    _atomic_update(
        {
            "PREV_TARGET_LAT": float(lat),
            "PREV_TARGET_LON": float(lon),
        }
    )


def update_yaw_offset(offset_deg: float) -> None:
    _atomic_update({"PREV_YAW_OFFSET": float(offset_deg)})


def update_motor_enabled(enabled: bool) -> None:
    _atomic_update({"PREV_MOTOR_ENABLED": 1 if bool(enabled) else 0})


def update_solenoid_state(count: int, done: bool) -> None:
    _atomic_update(
        {
            "PREV_SOLENOID_COUNT": max(0, int(count)),
            "PREV_SOLENOID_DONE": 1 if bool(done) else 0,
        }
    )


def update_start_point(lat: float, lon: float, locked: bool) -> None:
    _atomic_update(
        {
            "PREV_START_LAT": float(lat),
            "PREV_START_LON": float(lon),
            "PREV_START_LOCKED": 1 if bool(locked) else 0,
        }
    )


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


def update_bearing(bearing_deg: float) -> None:
    _atomic_update({"PREV_BEARING": float(bearing_deg) if bearing_deg == bearing_deg else None})


def get_bearing() -> float:
    """Return persisted bearing in degrees, or NaN if not set."""
    return PREV_BEARING


def update_packet_count(count: int) -> None:
    _atomic_update({"PREV_PACKET_COUNT": int(count)})


def update_st_timedelta(seconds: float) -> None:
    _atomic_update({"PREV_ST_TIMEDELTA": float(seconds)})


def reset_prevstate() -> None:
    """Force every persisted field back to its default snapshot."""
    global PREV_STATE, PREV_ALT_CAL, PREV_MAX_ALT
    global PREV_TARGET_LAT, PREV_TARGET_LON, PREV_PACKET_COUNT, PREV_ST_TIMEDELTA
    global PREV_YAW_OFFSET, PREV_MOTOR_ENABLED, PREV_SOLENOID_COUNT, PREV_SOLENOID_DONE
    global PREV_START_LAT, PREV_START_LON, PREV_START_LOCKED, PREV_BEARING

    PREV_STATE = 0
    PREV_ALT_CAL = 0.0
    PREV_MAX_ALT = 0.0
    PREV_TARGET_LAT, PREV_TARGET_LON = _target_defaults()
    PREV_PACKET_COUNT = 0
    PREV_ST_TIMEDELTA = 0.0
    PREV_YAW_OFFSET = 0.0
    PREV_MOTOR_ENABLED = 1
    PREV_SOLENOID_COUNT = 0
    PREV_SOLENOID_DONE = 0
    PREV_START_LAT = 0.0
    PREV_START_LON = 0.0
    PREV_START_LOCKED = 0
    PREV_BEARING = float("nan")
    _sync_legacy_aliases()
    with _FileLock():
        _atomic_write(_serialize())


