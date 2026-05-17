"""Motor app: sensor ingestion, guidance orchestration, and actuator output.

Current boundary:
  sensor apps -> MotorSensorCache -> ProduceL1Input/ProduceL1Output -> controller -> servo

This module keeps runtime data in an explicit sensor cache and passes snapshots
into guidance instead of reading guidance/control globals.
"""

from __future__ import annotations

import csv
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
import logging
import math
from pathlib import Path
import threading
import time
from typing import Optional

from lib import appargs, config, msgstructure, prevstate, timebase

from . import control, mag_guidance

LOGGER = logging.getLogger(__name__)

GPS_HISTORY_SEC = 10.0
IMU_HISTORY_SEC = 2.0
BARO_HISTORY_SEC = 10.0
GPS_REGRESSION_SEC = 3.0
GPS_MOTION_MIN_VALID_MPS = 0.3
IMU_ESTIMATE_SEC = 0.50
BARO_REGRESSION_SEC = 3.0

_CONTROL_LOG_LOCK = threading.Lock()
_CONTROL_LOG_FP = None
_CONTROL_LOG_WRITER = None
_CONTROL_LOG_PATH = None
_CONTROL_LOG_HEADER = [
    "host_time",
    "monotonic_s",
    "state",
    "motor_enabled",
    "diag_state",
    "guidance_reason",
    "fail_reason",
    "confidence_scale",
    "guidance_mode",
    "control_mode",
    "nominal",
    "degraded",
    "valid",
    "gps_lat",
    "gps_lon",
    "gps_course_deg",
    "gps_speed_mps",
    "gps_pos_health",
    "gps_motion_health",
    "gps_pos_age_s",
    "gps_motion_age_s",
    "imu_gyrz_deg_s",
    "imu_health",
    "imu_age_s",
    "baro_alt_m",
    "baro_health",
    "baro_age_s",
    "start_lat",
    "start_lon",
    "target_lat",
    "target_lon",
    "pos_N_m",
    "pos_E_m",
    "target_N_m",
    "target_E_m",
    "carrot_N_m",
    "carrot_E_m",
    "carrot_lat",
    "carrot_lon",
    "crossTrack_m",
    "alongTrack_m",
    "L1_distance_m",
    "nu_deg",
    "nu1_deg",
    "nu2_deg",
    "current_heading_deg",
    "lat_acc_cmd_mps2",
    "angular_velocity_cmd_deg_s",
    "angular_velocity_meas_deg_s",
    "angular_velocity_error_deg_s",
    "delta_ff_deg",
    "delta_pid_deg",
    "delta_arm_deg",
    "left_angle_deg",
    "right_angle_deg",
    "left_pw_us",
    "right_pw_us",
    "saturated",
    "sensor_valid",
    "guidance_command_age_s",
    "fallback_mode",
]


def _history_len(rate_hz: float, seconds: float) -> int:
    return max(1, int(math.ceil(max(0.1, rate_hz) * seconds)))


@dataclass
class _GpsFromApp:
    lat: Optional[float] = None
    lon: Optional[float] = None
    course_rad: Optional[float] = None
    speed_mps: Optional[float] = None
    pos_ts: Optional[float] = None
    motion_ts: Optional[float] = None
    rx_ts: Optional[float] = None
    pos_health: int = 0
    motion_health: int = 0


@dataclass
class _ImuFromApp:
    roll_rad: Optional[float] = None
    pitch_rad: Optional[float] = None
    yaw_rad: Optional[float] = None
    accx_mps2: Optional[float] = None
    accy_mps2: Optional[float] = None
    accz_mps2: Optional[float] = None
    magx_uT: Optional[float] = None
    magy_uT: Optional[float] = None
    magz_uT: Optional[float] = None
    gyrx_rad_s: Optional[float] = None
    gyry_rad_s: Optional[float] = None
    gyrz_rad_s: Optional[float] = None
    ts: Optional[float] = None
    rx_ts: Optional[float] = None
    freefall: int = 0   # 1=자유낙하 중, 0=정상
    tumble:   int = 0   # 1=텀블링 중,  0=안정
    health:   int = 0   # 1=하드웨어 정상


@dataclass
class _BaroFromApp:
    alt_m:     Optional[float] = None
    sink_rate: Optional[float] = None
    ts:        Optional[float] = None
    rx_ts:     Optional[float] = None
    health:    int = 0  # 1=하드웨어 정상




@dataclass
class _Cache:
    latest_gps: _GpsFromApp = field(default_factory=_GpsFromApp)

    latest_imu: _ImuFromApp = field(default_factory=_ImuFromApp)
    imu_history: deque[_ImuFromApp] = field(
        default_factory=lambda: deque(
            maxlen=_history_len(float(config.IMU_RATE_HZ), IMU_HISTORY_SEC)
        )
    )

    latest_baro: _BaroFromApp = field(default_factory=_BaroFromApp)

    target_lat: Optional[float] = None
    target_lon: Optional[float] = None
    start_lat: Optional[float] = None
    start_lon: Optional[float] = None
    target_bearing_rad: float = float("nan")  # start→target magnetic bearing (rad)


MOTORAPP_RUNSTATUS: bool = True
MOTOR_ENABLED: bool = True
RELEASE_ACTION_ENABLED: bool = True
EGG_ACTION_ENABLED: bool = True
MANUAL_STEER_MODE: str = config.MOTOR_MANUAL_NEUTRAL
STATE: int = 0
PI = None

_UPDATE_LOCK = threading.Lock()
_CTRL_LOCK = threading.Lock()
_CACHE = _Cache()
_PREV_STATE = -1
_START_POINT_LOCKED = False

# Align with ground_station map: (0,0) means "no fix", not a real position.
_START_NULL_LAT_TOL = 1.0e-4
_START_NULL_LON_TOL = 1.0e-4
# GPS sanity thresholds. Defaults are configured for the current Korea test
# area and must be updated before operating at a distant site.
_GPS_EXPECTED_LON_CENTER_DEG = float(getattr(config, "GPS_EXPECTED_LON_CENTER_DEG", 126.6))
_GPS_EXPECTED_LON_RADIUS_DEG = float(getattr(config, "GPS_EXPECTED_LON_RADIUS_DEG", 20.0))
_GPS_MAX_VALID_SPEED_MPS = float(getattr(config, "GPS_MAX_VALID_SPEED_MPS", 40.0))
_GPS_POS_MAX_DELTA_DEG = 20.0   # deg, relative to locked start longitude
_MANUAL_STEER_DELTA_DEG = 60.0


def _finite_latlon(lat: Optional[float], lon: Optional[float]) -> bool:
    if lat is None or lon is None:
        return False
    try:
        la = float(lat)
        lo = float(lon)
    except (TypeError, ValueError):
        return False
    return (
        math.isfinite(la)
        and math.isfinite(lo)
        and -90.0 <= la <= 90.0
        and -180.0 <= lo <= 180.0
    )


def _is_placeholder_latlon(lat: float, lon: float) -> bool:
    return abs(lat) <= _START_NULL_LAT_TOL and abs(lon) <= _START_NULL_LON_TOL


def _gps_position_sanity_reason(
    lat: Optional[float],
    lon: Optional[float],
    start_lon: Optional[float] = None,
) -> Optional[str]:
    if not _finite_latlon(lat, lon):
        return "lat/lon not finite or out of range"
    la = float(lat)
    lo = float(lon)
    if _is_placeholder_latlon(la, lo):
        return "lat/lon placeholder"
    if abs(lo - _GPS_EXPECTED_LON_CENTER_DEG) > _GPS_EXPECTED_LON_RADIUS_DEG:
        return (
            f"lon={lo:.4f} outside expected "
            f"{_GPS_EXPECTED_LON_CENTER_DEG:.1f}+/-{_GPS_EXPECTED_LON_RADIUS_DEG:.1f} deg"
        )
    if start_lon is not None and abs(lo - float(start_lon)) > _GPS_POS_MAX_DELTA_DEG:
        return f"|lon-start_lon|={abs(lo - float(start_lon)):.2f} > {_GPS_POS_MAX_DELTA_DEG:.1f} deg"
    return None


def _gps_motion_sane(course_deg: float, ground_speed: float) -> bool:
    return (
        math.isfinite(course_deg)
        and math.isfinite(ground_speed)
        and 0.0 <= course_deg < 360.0
        and GPS_MOTION_MIN_VALID_MPS <= ground_speed <= _GPS_MAX_VALID_SPEED_MPS
    )


def _is_binary_health_field(value: str) -> bool:
    try:
        parsed = float(str(value).strip())
    except (TypeError, ValueError):
        return False
    return parsed in (0.0, 1.0)


def _gps_payload_looks_legacy(fields) -> bool:
    """Detect old lat,lon,course_deg,speed_mps,pos_health,motion_health payloads."""
    if len(fields) != 6:
        return False
    if not (_is_binary_health_field(fields[4]) and _is_binary_health_field(fields[5])):
        return False
    try:
        course_deg = float(fields[2])
        speed_mps = float(fields[3])
    except (TypeError, ValueError):
        return False
    return math.isfinite(course_deg) and math.isfinite(speed_mps) and 0.0 <= course_deg < 360.0


def _gps_payload_looks_legacy_fidelity(fields) -> bool:
    """Detect old lat,lon,speed_mps,course_deg,fix_quality,sats,rmc_status payloads."""
    if len(fields) != 7:
        return False
    try:
        speed_mps = float(fields[2])
        course_deg = float(fields[3])
        fix_quality = int(float(fields[4]))
        sats = int(float(fields[5]))
    except (TypeError, ValueError):
        return False
    rmc_status = str(fields[6]).strip().upper()
    return (
        math.isfinite(speed_mps)
        and math.isfinite(course_deg)
        and 0.0 <= course_deg < 360.0
        and fix_quality >= 0
        and sats >= 0
        and rmc_status in {"A", "V"}
    )


def _copy_deque(samples, sample_type, maxlen: Optional[int] = None):
    return deque((sample_type(**vars(sample)) for sample in samples), maxlen=maxlen)


def _gps_position_valid(sample: _GpsFromApp, now: float, age: float) -> bool:
    return bool(
        sample.pos_health
        and sample.lat is not None
        and sample.lon is not None
        and sample.pos_ts is not None
        and timebase.valid_age(sample.pos_ts, now, age)
    )


def _gps_motion_valid(sample: _GpsFromApp, now: float, age: float) -> bool:
    return bool(
        sample.motion_health
        and sample.course_rad is not None
        and sample.speed_mps is not None
        and sample.motion_ts is not None
        and timebase.valid_age(sample.motion_ts, now, age)
    )


def _gps_fresh_for_history(sample: _GpsFromApp, now: float) -> bool:
    return bool(
        _gps_position_valid(sample, now, guidance.POS_FRESH_AGE)
        or _gps_motion_valid(sample, now, guidance.MOTION_FRESH_AGE)
    )


def _imu_gyrz_valid(sample: _ImuFromApp, now: float, age: float) -> bool:
    return bool(
        sample.health
        and sample.gyrz_rad_s is not None
        and sample.ts is not None
        and timebase.valid_age(sample.ts, now, age)
    )


def _baro_alt_valid(sample: _BaroFromApp, now: float, age: float) -> bool:
    return bool(
        sample.health
        and sample.alt_m is not None
        and sample.ts is not None
        and timebase.valid_age(sample.ts, now, age)
    )


_IMU_GYRZ_HISTORY_AGE = 0.50   # s  (was guidance.GYRZ_FRESH_AGE)


def _push_latest_imu_to_history(cache: _Cache, now: float) -> None:
    if _imu_gyrz_valid(cache.latest_imu, now, _IMU_GYRZ_HISTORY_AGE):
        cache.imu_history.append(_ImuFromApp(**vars(cache.latest_imu)))




_CONTROLLER = None
_L1_STATE = None  # kept for dispatch reset compatibility

def _cache_snapshot() -> _Cache:
    return _Cache(
        latest_gps  = _GpsFromApp(**vars(_CACHE.latest_gps)),
        latest_imu  = _ImuFromApp(**vars(_CACHE.latest_imu)),
        imu_history = _copy_deque(_CACHE.imu_history, _ImuFromApp, _CACHE.imu_history.maxlen),
        latest_baro = _BaroFromApp(**vars(_CACHE.latest_baro)),
        target_lat  = _CACHE.target_lat,
        target_lon  = _CACHE.target_lon,
        start_lat   = _CACHE.start_lat,
        start_lon   = _CACHE.start_lon,
        target_bearing_rad = _CACHE.target_bearing_rad,
    )

#handler
def handle_gps(data: str) -> None:
    """Parse GPS payload and update cache.

    Current gpsapp payload is lat,lon,pos_ts,course_deg,spd_mps,motion_ts.
    Legacy harnesses may still send lat,lon,course_deg,spd_mps,pos_health,motion_health.
    Older sensor logs may send lat,lon,spd_mps,course_deg,fix_quality,sats,rmc_status.
    """
    global _START_POINT_LOCKED
    fields = data.split(",")
    if len(fields) not in (6, 7):
        LOGGER.warning("GNSS parse: expected 6 or 7 fields | raw=%r", data)
        return
    try:
        lat       = float(fields[0])
        lon       = float(fields[1])
        rx_ts = timebase.now()
        if _gps_payload_looks_legacy(fields):
            course_deg = float(fields[2])
            speed_mps = float(fields[3])
            pos_ts = rx_ts
            motion_ts = rx_ts
            payload_pos_health = bool(int(float(fields[4])))
            payload_motion_health = bool(int(float(fields[5])))
        elif _gps_payload_looks_legacy_fidelity(fields):
            speed_mps = float(fields[2])
            course_deg = float(fields[3])
            fix_quality = int(float(fields[4]))
            sats = int(float(fields[5]))
            rmc_status = str(fields[6]).strip().upper()
            pos_ts = rx_ts
            motion_ts = rx_ts
            payload_pos_health = (
                fix_quality >= 1
                and sats >= int(getattr(config, "GPS_MIN_SATS", 4))
            )
            payload_motion_health = rmc_status == "A"
        else:
            pos_ts = float(fields[2])
            course_deg = float(fields[3])   # nan when motion invalid
            speed_mps = float(fields[4])    # nan when motion invalid
            motion_ts = float(fields[5])    # nan when motion invalid
            payload_pos_health = True
            payload_motion_health = True
    except (ValueError, IndexError) as exc:
        LOGGER.warning("GNSS parse error: %s | raw=%r", exc, data)
        return

    with _UPDATE_LOCK:
        start_lon = _CACHE.start_lon
    pos_valid = payload_pos_health and _gps_position_sanity_reason(lat, lon, start_lon) is None
    motion_valid = payload_motion_health and pos_valid and _gps_motion_sane(course_deg, speed_mps)

    course_rad = math.radians(course_deg) if motion_valid else float("nan")
    sample = _GpsFromApp(
        lat=lat if pos_valid else None,
        lon=lon if pos_valid else None,
        course_rad=course_rad if motion_valid else None,
        speed_mps=speed_mps if motion_valid else None,
        pos_ts=pos_ts if pos_valid else None,
        motion_ts=motion_ts if motion_valid else None,
        rx_ts=rx_ts,
        pos_health=int(bool(pos_valid)),
        motion_health=int(bool(motion_valid)),
    )
    with _UPDATE_LOCK:
        _CACHE.latest_gps = sample
        if not _START_POINT_LOCKED and STATE >= 3 and pos_valid:
            _CACHE.start_lat = float(lat)
            _CACHE.start_lon = float(lon)
            _START_POINT_LOCKED = True
            prevstate.update_start_point(float(lat), float(lon), True)

def handle_imu(data: str) -> None:
    """Parse IMU payload and update cache.

    Current payload:
      roll,pitch,yaw,ax,ay,az,magx,magy,magz,gyrx,gyry,gyrz_deg_s,sample_ts,freefall,tumble,health

    Legacy sensor-log payload:
      roll,pitch,yaw,ax,ay,az,magx,magy,magz,gyrx,gyry,gyrz_deg_s,health,sample_ts
    """
    fields = data.split(",")
    try:
        if len(fields) not in (14, 16):
            LOGGER.warning("IMU parse: expected 14 or 16 fields, got %d | raw=%r", len(fields), data)
            return
        roll_deg   = float(fields[0])
        pitch_deg  = float(fields[1])
        yaw_deg    = float(fields[2])
        accx_mps2  = float(fields[3])
        accy_mps2  = float(fields[4])
        accz_mps2  = float(fields[5])
        magx_uT    = float(fields[6])
        magy_uT    = float(fields[7])
        magz_uT    = float(fields[8])
        gyrx_deg_s = float(fields[9])
        gyry_deg_s = float(fields[10])
        gyrz_deg_s = float(fields[11])
        if len(fields) == 16:
            sample_ts  = float(fields[12])
            freefall   = int(float(fields[13]))
            tumble     = int(float(fields[14]))
            health     = int(float(fields[15]))
        else:
            health     = int(float(fields[12]))
            sample_ts  = float(fields[13])
            freefall   = 0
            tumble     = 0
        rx_ts = timebase.now()
    except (ValueError, IndexError) as exc:
        LOGGER.warning("IMU parse error: %s | raw=%r", exc, data)
        return

    imu = _ImuFromApp(
        roll_rad=math.radians(roll_deg),
        pitch_rad=math.radians(pitch_deg),
        yaw_rad=math.radians(yaw_deg),
        accx_mps2=accx_mps2,
        accy_mps2=accy_mps2,
        accz_mps2=accz_mps2,
        magx_uT=magx_uT,
        magy_uT=magy_uT,
        magz_uT=magz_uT,
        gyrx_rad_s=math.radians(gyrx_deg_s),
        gyry_rad_s=math.radians(gyry_deg_s),
        gyrz_rad_s=math.radians(-gyrz_deg_s),  # IMU Z-up: gz+= CCW; negate to match nav convention (gz+ = CW = right turn)
        ts=sample_ts,
        rx_ts=rx_ts,
        freefall=freefall,
        tumble=tumble,
        health=health,
    )
    with _UPDATE_LOCK:
        _push_latest_imu_to_history(_CACHE, rx_ts)
        _CACHE.latest_imu = imu


def handle_barometer(data: str) -> None:
    """Parse barometer payload and update cache.

    Current payload:
      alt_m,sample_ts,sink_rate,health

    Legacy sensor-log payload:
      alt_m,health,sample_ts
    """
    fields = data.split(",")
    try:
        if len(fields) not in (3, 4):
            LOGGER.warning("Baro parse: expected 3 or 4 fields, got %d | raw=%r", len(fields), data)
            return
        alt_m     = float(fields[0].strip())
        if len(fields) == 4:
            sample_ts = float(fields[1])
            sink_s    = fields[2].strip()
            sink_rate = None if sink_s == "nan" else float(sink_s)
            health    = int(float(fields[3]))
        else:
            health    = int(float(fields[1]))
            sample_ts = float(fields[2])
            sink_rate = None
        rx_ts = timebase.now()
    except (ValueError, IndexError) as exc:
        LOGGER.warning("Baro parse error: %s | raw=%r", exc, data)
        return

    baro = _BaroFromApp(
        alt_m=alt_m,
        sink_rate=sink_rate,
        ts=sample_ts,
        rx_ts=rx_ts,
        health=health,
    )
    with _UPDATE_LOCK:
        _CACHE.latest_baro = baro


def _auto_compute_bearing(target_lat: float, target_lon: float) -> None:
    """시작 좌표 → 타겟 좌표 방위각을 계산해 _CACHE에 저장한다."""
    start = prevstate.get_start_point()
    if start is None:
        LOGGER.info("Bearing auto-compute skipped: start point not locked")
        return
    slat, slon = start
    slat_r = math.radians(slat)
    tlat_r = math.radians(target_lat)
    dlon_r = math.radians(target_lon - slon)
    x = math.sin(dlon_r) * math.cos(tlat_r)
    y = math.cos(slat_r) * math.sin(tlat_r) - math.sin(slat_r) * math.cos(tlat_r) * math.cos(dlon_r)
    bearing_deg = math.degrees(math.atan2(x, y)) % 360.0
    with _UPDATE_LOCK:
        _CACHE.target_bearing_rad = math.radians(bearing_deg)
    prevstate.update_bearing(bearing_deg)
    LOGGER.info("Bearing auto-computed: %.2f deg (start=%.6f,%.6f target=%.6f,%.6f)",
                bearing_deg, slat, slon, target_lat, target_lon)


def handle_target_coord(data: str) -> None:
    """lat,lon"""
    fields = data.split(",")
    if len(fields) != 2:
        LOGGER.warning("Target parse: expected 2 fields | raw=%r", data)
        return
    try:
        lat = float(fields[0])
        lon = float(fields[1])
    except (ValueError, IndexError) as exc:
        LOGGER.warning("Target parse error: %s | raw=%r", exc, data)
        return
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        LOGGER.warning("Target coord out of range: %.6f, %.6f", lat, lon)
        return
    with _UPDATE_LOCK:
        _CACHE.target_lat = lat
        _CACHE.target_lon = lon
    LOGGER.info("Target updated: %.6f, %.6f", lat, lon)
    _auto_compute_bearing(lat, lon)


def handle_bearing(data: str) -> None:
    """Receive start→target magnetic bearing (deg, 0=N, CW+) from flight logic.

    Expected data format: single float string, e.g. "127.5"
    """
    try:
        bearing_deg = float(data.strip())
    except (ValueError, TypeError) as exc:
        LOGGER.warning("handle_bearing parse error: %s | raw=%r", exc, data)
        return
    if not math.isfinite(bearing_deg):
        LOGGER.warning("handle_bearing: non-finite value %r", bearing_deg)
        return
    with _UPDATE_LOCK:
        _CACHE.target_bearing_rad = math.radians(bearing_deg)
    prevstate.update_bearing(bearing_deg)
    LOGGER.info("Bearing set: %.2f deg (%.4f rad)", bearing_deg, math.radians(bearing_deg))


def handle_flight_state(data: str) -> None:
    global STATE, _PREV_STATE, _START_POINT_LOCKED, _L1_STATE, _CONTROLLER
    try:
        new_state = int(data.split(",")[0])
    except (ValueError, IndexError) as exc:
        LOGGER.warning("State parse error: %s | raw=%r", exc, data)
        return
    if new_state == STATE:
        return

    LOGGER.info("State %d -> %d", STATE, new_state)
    do_ctrl_reset = False
    with _UPDATE_LOCK:
        _PREV_STATE = STATE
        STATE = new_state
        if new_state < 3:
            _CACHE.start_lat = None
            _CACHE.start_lon = None
            _START_POINT_LOCKED = False
            prevstate.clear_start_point()
            do_ctrl_reset = True
        elif new_state in (3, 4):
            gps = _CACHE.latest_gps
            if (
                not _START_POINT_LOCKED
                and gps.pos_ts is not None
                and gps.lat is not None
                and gps.lon is not None
                and -90.0 <= float(gps.lat) <= 90.0
                and -180.0 <= float(gps.lon) <= 180.0
            ):
                _CACHE.start_lat = float(gps.lat)
                _CACHE.start_lon = float(gps.lon)
                _START_POINT_LOCKED = True
                prevstate.update_start_point(float(gps.lat), float(gps.lon), True)

    if do_ctrl_reset and _CONTROLLER is not None and hasattr(control, "controller_reset"):
        with _CTRL_LOCK:
            control.controller_reset(_CONTROLLER)


def handle_release(data: str = "TRIGGER") -> None:
    if not RELEASE_ACTION_ENABLED:
        LOGGER.warning("Burnwire trigger ignored: RELEASE_ACTION_ENABLED=False")
        return
    try:
        from . import Motor_Release
    except Exception as exc:
        LOGGER.error("Burnwire module unavailable: %s", exc)
        return
    if not hasattr(Motor_Release, "activate_burnwire"):
        LOGGER.error("Burnwire module unavailable")
        return
    reason = data.split(":", 1)[1] if isinstance(data, str) and ":" in data else str(data or "UNKNOWN")
    LOGGER.warning("Burnwire trigger received | reason=%s", reason)
    threading.Thread(target=Motor_Release.activate_burnwire, daemon=True, name="Burnwire").start()


def handle_egg_drop() -> None:
    if not EGG_ACTION_ENABLED:
        LOGGER.warning("Egg trigger ignored: EGG_ACTION_ENABLED=False")
        return
    try:
        from . import Motor_Egg
    except Exception as exc:
        LOGGER.error("Egg module unavailable: %s", exc)
        return
    if not hasattr(Motor_Egg, "activate_solenoid"):
        LOGGER.error("Egg module unavailable")
        return
    threading.Thread(target=Motor_Egg.activate_solenoid, daemon=True, name="Solenoid").start()


def handle_mec(data: str) -> None:
    global MOTOR_ENABLED
    cmd = data.strip().upper()
    if cmd == "ON":
        MOTOR_ENABLED = True
        prevstate.update_motor_enabled(True)
        LOGGER.info("MOTOR_ENABLED = True")
    elif cmd == "OFF":
        MOTOR_ENABLED = False
        prevstate.update_motor_enabled(False)
        with _UPDATE_LOCK:
            if PI is not None:
                control.WriteZero(PI)
        LOGGER.info("MOTOR_ENABLED = False -> zero")
    else:
        LOGGER.warning("Unknown MEC command: %r", data)


def _manual_steer_command(now: float, mode: str) -> control.CtrlOutput:
    cmd = control.WriteNeutral(now, f"MANUAL_{mode}")
    steer = str(mode or "").strip().upper()
    if steer == config.MOTOR_MANUAL_LEFT:
        delta = -_MANUAL_STEER_DELTA_DEG
    elif steer == config.MOTOR_MANUAL_RIGHT:
        delta = _MANUAL_STEER_DELTA_DEG
    else:
        cmd.mode = f"MANUAL_{config.MOTOR_MANUAL_NEUTRAL}"
        cmd.fallback_mode = cmd.mode
        return cmd

    left_pw, right_pw, left_angle, right_angle, delta_arm = control.ConnectRoMo(delta)
    cmd.left_pw = left_pw
    cmd.right_pw = right_pw
    cmd.left_angle_deg = left_angle
    cmd.right_angle_deg = right_angle
    cmd.delta_arm_deg = delta_arm
    cmd.angular_velocity_cmd_deg_s = delta
    cmd.valid = True
    cmd.fallback_mode = cmd.mode
    return cmd


def handle_mtr(data: str) -> None:
    global MANUAL_STEER_MODE
    mode = str(data or "").strip().upper()
    if mode in {
        config.MOTOR_MANUAL_LEFT,
        config.MOTOR_MANUAL_RIGHT,
        config.MOTOR_MANUAL_NEUTRAL,
    }:
        MANUAL_STEER_MODE = mode
        LOGGER.info("MANUAL_STEER_MODE = %s", mode)
    else:
        LOGGER.warning("Unknown MTR command: %r", data)


def handle_fac(data: str) -> None:
    global RELEASE_ACTION_ENABLED, EGG_ACTION_ENABLED
    raw = data.strip().upper().replace(" ", "")
    parts = [p for p in raw.split(",") if p]
    if len(parts) == 1 and parts[0] in {"ON", "OFF"}:
        actor = "ALL"
        state = parts[0]
    elif len(parts) == 2 and parts[0] in {"ALL", "REL", "EGG"} and parts[1] in {"ON", "OFF"}:
        actor = parts[0]
        state = parts[1]
    else:
        LOGGER.warning("Unknown FAC command: %r", data)
        return

    enabled = state == "ON"
    if actor in {"ALL", "REL"}:
        RELEASE_ACTION_ENABLED = enabled
    if actor in {"ALL", "EGG"}:
        EGG_ACTION_ENABLED = enabled
    LOGGER.info(
        "FORCE_ACTION gate updated | actor=%s state=%s | release=%s egg=%s",
        actor,
        state,
        RELEASE_ACTION_ENABLED,
        EGG_ACTION_ENABLED,
    )



def _send_diag(main_queue, cmd, g_out, diag_state: str,
               start_lat=None, start_lon=None) -> None:
    if main_queue is None:
        return

    def _fmt(value, digits: int = 4) -> str:
        try:
            f = float(value)
            return "nan" if not math.isfinite(f) else f"{f:.{digits}f}"
        except (TypeError, ValueError):
            return "nan"

    payload = ",".join(
        [
            str(getattr(cmd, "left_pw", 0)),
            str(getattr(cmd, "right_pw", 0)),
            _fmt(start_lat, 6),
            _fmt(start_lon, 6),
            _fmt(getattr(g_out, "target_lat", _CACHE.target_lat), 6),
            _fmt(getattr(g_out, "target_lon", _CACHE.target_lon), 6),
            _fmt(getattr(g_out, "carrot_lat", float("nan")), 6),
            _fmt(getattr(g_out, "carrot_lon", float("nan")), 6),
            _fmt(math.degrees(float(getattr(g_out, "current_heading_rad", float("nan")))), 2),
            diag_state,
            str(int(bool(MOTOR_ENABLED))),
            str(int(bool(RELEASE_ACTION_ENABLED and EGG_ACTION_ENABLED))),
            str(int(bool(RELEASE_ACTION_ENABLED))),
            str(int(bool(EGG_ACTION_ENABLED))),
            _fmt(getattr(g_out, "crossTrack", float("nan"))),
            _fmt(getattr(g_out, "alongTrack", float("nan"))),
            _fmt(getattr(cmd, "angular_velocity_cmd_deg_s", 0.0)),
            _fmt(getattr(cmd, "angular_velocity_meas_deg_s", float("nan"))),
            _fmt(getattr(cmd, "angular_velocity_error_deg_s", 0.0)),
            _fmt(getattr(cmd, "delta_ff_deg", 0.0)),
            _fmt(getattr(cmd, "delta_pid_deg", 0.0)),
            _fmt(getattr(cmd, "delta_arm_deg", 0.0)),
            _fmt(getattr(cmd, "left_angle_deg", 0.0)),
            _fmt(getattr(cmd, "right_angle_deg", 0.0)),
            str(int(bool(getattr(cmd, "saturated", False)))),
            str(int(bool(getattr(cmd, "sensor_valid", False)))),
            _fmt(getattr(cmd, "guidance_command_age_s", 0.0)),
            str(getattr(cmd, "fallback_mode", "")),
            str(getattr(cmd, "mode", "")),
        ]
    )
    msgstructure.send_msg(
        main_queue,
        appargs.MotorAppArg.AppID,
        appargs.CommAppArg.AppID,
        appargs.MotorAppArg.MID_comm_motor_diag,
        payload,
    )


def _fmt_log(value, digits: int = 6) -> str:
    try:
        f = float(value)
        return "" if not math.isfinite(f) else f"{f:.{digits}f}"
    except (TypeError, ValueError):
        return ""


def _age_s(now: float, timestamp: Optional[float]) -> str:
    if timestamp is None:
        return ""
    try:
        age = timebase.age(now, timestamp)
    except (TypeError, ValueError):
        return ""
    return "" if not math.isfinite(age) else f"{age:.4f}"


def _deg_log(rad_value) -> str:
    try:
        return _fmt_log(math.degrees(float(rad_value)), 4)
    except (TypeError, ValueError):
        return ""


def _control_log_writer():
    global _CONTROL_LOG_FP, _CONTROL_LOG_WRITER, _CONTROL_LOG_PATH
    if _CONTROL_LOG_WRITER is not None:
        return _CONTROL_LOG_WRITER

    log_dir = Path("motorlogs")
    log_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    _CONTROL_LOG_PATH = log_dir / f"motor_control_{ts}.csv"
    _CONTROL_LOG_FP = _CONTROL_LOG_PATH.open("a", encoding="utf-8", newline="")
    _CONTROL_LOG_WRITER = csv.writer(_CONTROL_LOG_FP)
    _CONTROL_LOG_WRITER.writerow(_CONTROL_LOG_HEADER)
    _CONTROL_LOG_FP.flush()
    LOGGER.info("Motor control debug log: %s", _CONTROL_LOG_PATH)
    return _CONTROL_LOG_WRITER


def _write_control_debug_log(
    now: float,
    snap: Optional[_Cache],
    l1_input,
    mode,
    g_out,
    cmd,
    diag_state: str,
) -> None:
    try:
        with _CONTROL_LOG_LOCK:
            writer = _control_log_writer()
            gps = snap.latest_gps if snap is not None else _GpsFromApp()
            imu = snap.latest_imu if snap is not None else _ImuFromApp()
            baro = snap.latest_baro if snap is not None else _BaroFromApp()
            writer.writerow(
                [
                    datetime.now().isoformat(timespec="milliseconds"),
                    _fmt_log(now, 6),
                    str(STATE),
                    str(int(bool(MOTOR_ENABLED))),
                    diag_state,
                    str(getattr(g_out, "reason", "")),
                    str(getattr(g_out, "fail_reason", "")),
                    _fmt_log(getattr(g_out, "confidence_scale", None), 4),
                    str(getattr(mode, "value", mode) if mode is not None else ""),
                    str(getattr(cmd, "mode", "")),
                    str(int(bool(getattr(g_out, "nominal", False)))),
                    str(int(bool(getattr(g_out, "degraded", False)))),
                    str(int(bool(getattr(cmd, "valid", False)))),
                    _fmt_log(gps.lat, 8),
                    _fmt_log(gps.lon, 8),
                    _deg_log(gps.course_rad),
                    _fmt_log(gps.speed_mps, 4),
                    str(int(gps.pos_ts is not None)),
                    str(int(gps.motion_ts is not None)),
                    _age_s(now, gps.pos_ts),
                    _age_s(now, gps.motion_ts),
                    _deg_log(imu.gyrz_rad_s),
                    str(imu.health),
                    _age_s(now, imu.ts),
                    _fmt_log(baro.alt_m, 3),
                    str(baro.health),
                    _age_s(now, baro.ts),
                    _fmt_log(getattr(snap, "start_lat", None), 8),
                    _fmt_log(getattr(snap, "start_lon", None), 8),
                    _fmt_log(getattr(g_out, "target_lat", getattr(snap, "target_lat", None)), 8),
                    _fmt_log(getattr(g_out, "target_lon", getattr(snap, "target_lon", None)), 8),
                    _fmt_log(getattr(g_out, "pos_N", getattr(l1_input, "pos_N", None)), 3),
                    _fmt_log(getattr(g_out, "pos_E", getattr(l1_input, "pos_E", None)), 3),
                    _fmt_log(getattr(g_out, "target_N", None), 3),
                    _fmt_log(getattr(g_out, "target_E", None), 3),
                    _fmt_log(getattr(g_out, "carrot_N", None), 3),
                    _fmt_log(getattr(g_out, "carrot_E", None), 3),
                    _fmt_log(getattr(g_out, "carrot_lat", None), 8),
                    _fmt_log(getattr(g_out, "carrot_lon", None), 8),
                    _fmt_log(getattr(g_out, "crossTrack", None), 3),
                    _fmt_log(getattr(g_out, "alongTrack", None), 3),
                    _fmt_log(getattr(g_out, "L1_distance", None), 3),
                    _deg_log(getattr(g_out, "nu", None)),
                    _deg_log(getattr(g_out, "nu1", None)),
                    _deg_log(getattr(g_out, "nu2", None)),
                    _deg_log(getattr(g_out, "current_heading_rad", None)),
                    _fmt_log(getattr(g_out, "lat_acc_cmd_mps2", None), 4),
                    _fmt_log(getattr(cmd, "angular_velocity_cmd_deg_s", None), 4),
                    _fmt_log(getattr(cmd, "angular_velocity_meas_deg_s", None), 4),
                    _fmt_log(getattr(cmd, "angular_velocity_error_deg_s", None), 4),
                    _fmt_log(getattr(cmd, "delta_ff_deg", None), 4),
                    _fmt_log(getattr(cmd, "delta_pid_deg", None), 4),
                    _fmt_log(getattr(cmd, "delta_arm_deg", None), 4),
                    _fmt_log(getattr(cmd, "left_angle_deg", None), 4),
                    _fmt_log(getattr(cmd, "right_angle_deg", None), 4),
                    str(getattr(cmd, "left_pw", "")),
                    str(getattr(cmd, "right_pw", "")),
                    str(int(bool(getattr(cmd, "saturated", False)))),
                    str(int(bool(getattr(cmd, "sensor_valid", False)))),
                    _fmt_log(getattr(cmd, "guidance_command_age_s", None), 4),
                    str(getattr(cmd, "fallback_mode", "")),
                ]
            )
            if _CONTROL_LOG_FP is not None:
                _CONTROL_LOG_FP.flush()
    except Exception:
        LOGGER.debug("Failed to write motor control debug log", exc_info=True)


def _close_control_debug_log() -> None:
    global _CONTROL_LOG_FP, _CONTROL_LOG_WRITER
    with _CONTROL_LOG_LOCK:
        if _CONTROL_LOG_FP is not None:
            try:
                _CONTROL_LOG_FP.flush()
                _CONTROL_LOG_FP.close()
            except Exception:
                pass
        _CONTROL_LOG_FP = None
        _CONTROL_LOG_WRITER = None


def ctrl_parafoil(main_queue=None) -> None:
    """Parafoil control loop."""
    global _CONTROLLER
    period = 1.0 / max(0.1, float(config.MOTOR_RATE_HZ))
    while MOTORAPP_RUNSTATUS:
        now = timebase.now()
        try:
            if not MOTOR_ENABLED or STATE < 3:
                if PI is not None:
                    control.WriteZero(PI)
                idle_cmd = control.WriteNeutral(now, config.MOTOR_REASON_IDLE)
                idle_out = mag_guidance.MagGuidanceOutput(
                    timestamp=now, mode=config.MOTOR_REASON_IDLE, valid=False
                )
                with _UPDATE_LOCK:
                    idle_snap = _cache_snapshot()
                _write_control_debug_log(now, idle_snap, None, None, idle_out, idle_cmd, config.MOTOR_REASON_IDLE)
                _send_diag(main_queue, idle_cmd, idle_out, config.MOTOR_REASON_IDLE,
                           idle_snap.start_lat, idle_snap.start_lon)
                time.sleep(period)
                continue

            if STATE == 5:
                if PI is not None:
                    control.WriteOff(PI)
                landed_cmd = control.WriteNeutral(now, config.MOTOR_REASON_LANDED)
                landed_out = mag_guidance.MagGuidanceOutput(
                    timestamp=now, mode=config.MOTOR_REASON_LANDED, valid=False
                )
                with _UPDATE_LOCK:
                    landed_snap = _cache_snapshot()
                _write_control_debug_log(now, landed_snap, None, None, landed_out, landed_cmd, config.MOTOR_REASON_LANDED)
                _send_diag(main_queue, landed_cmd, landed_out, config.MOTOR_REASON_LANDED,
                           landed_snap.start_lat, landed_snap.start_lon)
                time.sleep(period)
                continue

            with _UPDATE_LOCK:
                snap = _cache_snapshot()

            # DR을 현재 시각으로 갱신 (lock 밖 - snap 은 이미 복사됨)
            if MANUAL_STEER_MODE != config.MOTOR_MANUAL_NEUTRAL:
                manual_cmd = _manual_steer_command(now, MANUAL_STEER_MODE)
                manual_out = mag_guidance.MagGuidanceOutput(
                    timestamp=now,
                    mode=manual_cmd.mode,
                    valid=bool(manual_cmd.valid),
                    angular_velocity_cmd_deg_s=manual_cmd.angular_velocity_cmd_deg_s,
                )
                if PI is not None:
                    control.ProducePulse(PI, manual_cmd)
                _write_control_debug_log(now, snap, None, None, manual_out, manual_cmd, manual_cmd.mode)
                _send_diag(main_queue, manual_cmd, manual_out, manual_cmd.mode,
                           snap.start_lat, snap.start_lon)
                time.sleep(period)
                continue

            # ── Mag-guidance (GPS-free) ────────────────────────────────────────
            imu  = snap.latest_imu
            baro = snap.latest_baro

            mag_inp = mag_guidance.MagGuidanceInput(
                yaw_rad            = imu.yaw_rad,
                gyrz_rad_s         = imu.gyrz_rad_s if imu.gyrz_rad_s is not None else 0.0,
                alt_m              = baro.alt_m,
                target_bearing_rad = snap.target_bearing_rad,
                timestamp          = now,
                imu_ts             = imu.ts,
                baro_ts            = baro.ts,
                imu_health         = imu.health,
                baro_health        = baro.health,
            )
            g_out = mag_guidance.ProduceMagGuidance(mag_inp)

            if g_out.valid:
                if _CONTROLLER is None:
                    _CONTROLLER = control.MakeCtrler()
                gyrz_meas_deg_s = (
                    math.degrees(imu.gyrz_rad_s)
                    if (imu.gyrz_rad_s is not None and math.isfinite(imu.gyrz_rad_s))
                    else float("nan")
                )
                gcmd = control.CtrlInput(
                    angular_velocity_cmd_deg_s=g_out.angular_velocity_cmd_deg_s,
                    valid=True,
                    timestamp=now,
                )
                with _CTRL_LOCK:
                    cmd = control.ProduceCtrlOutput(_CONTROLLER, gcmd, gyrz_meas_deg_s, now)
            else:
                if _CONTROLLER is not None:
                    with _CTRL_LOCK:
                        control.controller_reset(_CONTROLLER)
                cmd = control.WriteNeutral(now, g_out.mode)

            if PI is not None:
                control.ProducePulse(PI, cmd)
            diag_state = g_out.mode
            _write_control_debug_log(now, snap, None, g_out.mode, g_out, cmd, diag_state)
            _send_diag(main_queue, cmd, g_out, diag_state, snap.start_lat, snap.start_lon)

        except Exception as exc:
            LOGGER.error("ctrl_paragldr exception: %s", exc, exc_info=True)
            if PI is not None:
                control.WriteZero(PI)
        time.sleep(period)


def dispatch(msg: str) -> None:
    global MOTORAPP_RUNSTATUS
    unpacked = msgstructure.unpack_msg(msg)
    if unpacked is False:
        return
    mid = unpacked.msg_id
    if mid == appargs.MainAppArg.MID_TerminateProcess:
        MOTORAPP_RUNSTATUS = False
    elif mid == appargs.GpsAppArg.MID_motor_gps:
        handle_gps(unpacked.data)
    elif mid == appargs.ImuAppArg.MID_motor_imu:
        handle_imu(unpacked.data)
    elif mid == appargs.BarometerAppArg.MID_motor_alt:
        handle_barometer(unpacked.data)
    elif mid == appargs.FlightlogicAppArg.MID_motor_TargetCor:
        handle_target_coord(unpacked.data)
    elif mid == appargs.FlightlogicAppArg.MID_motor_bearing:
        handle_bearing(unpacked.data)
    elif mid == appargs.FlightlogicAppArg.MID_motor_state:
        handle_flight_state(unpacked.data)
    elif mid == appargs.FlightlogicAppArg.MID_motor_burnwire:
        handle_release(unpacked.data)
    elif mid == appargs.FlightlogicAppArg.MID_motor_EggDrop:
        handle_egg_drop()
    elif mid == appargs.CommAppArg.MID_RouteCmd_MEC:
        handle_mec(unpacked.data)
    elif mid == appargs.CommAppArg.MID_RouteCmd_MTR:
        handle_mtr(unpacked.data)
    elif mid == appargs.CommAppArg.MID_RouteCmd_FAC:
        handle_fac(unpacked.data)


def init() -> None:
    global PI, MOTOR_ENABLED, RELEASE_ACTION_ENABLED, EGG_ACTION_ENABLED
    global MANUAL_STEER_MODE, _START_POINT_LOCKED, _CONTROLLER, _L1_STATE
    prevstate.init_prevstate()
    MOTOR_ENABLED = prevstate.is_motor_enabled()
    RELEASE_ACTION_ENABLED = True
    EGG_ACTION_ENABLED = True
    MANUAL_STEER_MODE = config.MOTOR_MANUAL_NEUTRAL

    target_lat, target_lon = prevstate.get_target_gps()
    if (
        -90.0 <= float(target_lat) <= 90.0
        and -180.0 <= float(target_lon) <= 180.0
        and not (target_lat == 0.0 and target_lon == 0.0)
    ):
        _CACHE.target_lat = float(target_lat)
        _CACHE.target_lon = float(target_lon)

    start_point = prevstate.get_start_point()
    if start_point is not None:
        lat, lon = start_point
        if -90.0 <= float(lat) <= 90.0 and -180.0 <= float(lon) <= 180.0:
            _CACHE.start_lat = float(lat)
            _CACHE.start_lon = float(lon)
            _START_POINT_LOCKED = True

    saved_bearing = prevstate.get_bearing()
    if math.isfinite(saved_bearing):
        with _UPDATE_LOCK:
            _CACHE.target_bearing_rad = math.radians(saved_bearing)
        LOGGER.info("Bearing restored from prevstate: %.2f deg", saved_bearing)

    _CONTROLLER = control.MakeCtrler()
    PI = control.init_control()

    try:
        from . import Motor_Release
        if hasattr(Motor_Release, "init_burnwire"):
            Motor_Release.init_burnwire()
    except Exception:
        LOGGER.debug("Failed to init burnwire", exc_info=True)

    try:
        from . import Motor_Egg
        if hasattr(Motor_Egg, "init_solenoid"):
            Motor_Egg.init_solenoid()
    except Exception:
        LOGGER.debug("Failed to init solenoid", exc_info=True)

    LOGGER.info(
        "MotorApp init | pigpio=%s | motor_enabled=%s | start_locked=%s",
        getattr(PI, "connected", "N/A"),
        MOTOR_ENABLED,
        _START_POINT_LOCKED,
    )


def motorapp_main(main_queue, main_pipe=None) -> None:
    global MOTORAPP_RUNSTATUS
    if main_pipe is None:
        main_pipe = main_queue
        main_queue = None
    init()

    ctrl_thread = threading.Thread(
        target=ctrl_parafoil,
        args=(main_queue,),
        daemon=True,
        name="MotorControlLoop",
    )
    ctrl_thread.start()
    LOGGER.info("MotorControlLoop started")

    poll_period = 1.0 / max(0.1, float(config.MOTOR_RATE_HZ))
    try:
        while MOTORAPP_RUNSTATUS:
            try:
                if main_pipe.poll(poll_period):
                    dispatch(main_pipe.recv())
            except (KeyboardInterrupt, EOFError, OSError):
                break
    except KeyboardInterrupt:
        pass

    MOTORAPP_RUNSTATUS = False
    ctrl_thread.join(timeout=1.0)
    _close_control_debug_log()
    LOGGER.info("MotorApp exiting")
