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

from lib import appargs, config, msgstructure, prevstate

from . import control, guidance

LOGGER = logging.getLogger(__name__)

GPS_HISTORY_SEC = 10.0
IMU_HISTORY_SEC = 2.0
BARO_HISTORY_SEC = 10.0
GPS_REGRESSION_SEC = 3.0
IMU_ESTIMATE_SEC = 0.50
BARO_REGRESSION_SEC = 3.0
_EARTH_RADIUS_M = 6_371_000.0

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
    "yaw_rate_cmd_deg_s",
    "yaw_rate_meas_deg_s",
    "yaw_rate_error_deg_s",
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
class _DeadReckoning:
    """GPS stale 구간의 추정 위치 (constant-velocity propagation)."""
    lat:        Optional[float] = None
    lon:        Optional[float] = None
    ts:         Optional[float] = None   # 이 추정값이 계산된 시각 (monotonic)
    anchor_lat: Optional[float] = None   # 직전 GPS fix 위치
    anchor_lon: Optional[float] = None
    anchor_ts:  Optional[float] = None   # 직전 GPS fix 시각
    course_rad: Optional[float] = None   # 직전 GPS/추정 motion
    speed_mps:  Optional[float] = None
    motion_ts:  Optional[float] = None
    valid:      bool = False


@dataclass
class _EstimatedSample:
    """한 제어 주기에서 history 기반으로 산출된 추정 센서값."""
    # GPS position
    lat:          Optional[float] = None
    lon:          Optional[float] = None
    pos_ts:       Optional[float] = None
    pos_valid:    bool = False
    # GPS motion
    course_rad:   Optional[float] = None
    speed_mps:    Optional[float] = None
    motion_ts:    Optional[float] = None
    motion_valid: bool = False
    # IMU
    gyrz_rad_s:   Optional[float] = None
    gyrz_ts:      Optional[float] = None
    gyrz_valid:   bool = False
    # Baro
    alt_m:        Optional[float] = None
    sink_rate:    Optional[float] = None
    alt_ts:       Optional[float] = None
    alt_valid:    bool = False
    # 이 샘플이 산출된 제어 주기 시각
    ts:           Optional[float] = None


@dataclass
class _Cache:
    latest_gps: _GpsFromApp = field(default_factory=_GpsFromApp)
    gps_history: deque[_GpsFromApp] = field(
        default_factory=lambda: deque(
            maxlen=_history_len(float(config.GPS_RATE_HZ), GPS_HISTORY_SEC)
        )
    )

    latest_imu: _ImuFromApp = field(default_factory=_ImuFromApp)
    imu_history: deque[_ImuFromApp] = field(
        default_factory=lambda: deque(
            maxlen=_history_len(float(config.IMU_RATE_HZ), IMU_HISTORY_SEC)
        )
    )

    latest_baro: _BaroFromApp = field(default_factory=_BaroFromApp)
    baro_history: deque[_BaroFromApp] = field(
        default_factory=lambda: deque(
            maxlen=_history_len(float(config.BAROMETER_RATE_HZ), BARO_HISTORY_SEC)
        )
    )

    dr: _DeadReckoning = field(default_factory=_DeadReckoning)

    estimated_history: deque[_EstimatedSample] = field(
        default_factory=lambda: deque(
            maxlen=_history_len(
                float(config.MOTOR_RATE_HZ),
                max(guidance.POS_EST_AGE, guidance.MOTION_EST_AGE, guidance.ALT_EST_AGE),
            )
        )
    )

    target_lat: Optional[float] = None
    target_lon: Optional[float] = None
    start_lat: Optional[float] = None
    start_lon: Optional[float] = None


MOTORAPP_RUNSTATUS: bool = True
MOTOR_ENABLED: bool = True
MANUAL_STEER_MODE: Optional[str] = None
RELEASE_ACTION_ENABLED: bool = True
EGG_ACTION_ENABLED: bool = True
STATE: int = 0
PI = None

_UPDATE_LOCK = threading.Lock()
_CACHE = _Cache()
_PREV_STATE = -1
_START_POINT_LOCKED = False

# Align with ground_station map: (0,0) means “no fix”, not a real position.
_START_NULL_LAT_TOL = 1.0e-4
_START_NULL_LON_TOL = 1.0e-4
_MANUAL_STEER_DELTA_DEG = min(40.0, control.DELTA_ARM_MAX_DEG)
_MANUAL_STEER_MODES = {"LEFT", "NEUTRAL", "RIGHT"}

# GPS sanity thresholds. Defaults are configured for the current Korea test
# area and must be updated before operating at a distant site.
_GPS_EXPECTED_LON_CENTER_DEG = float(getattr(config, "GPS_EXPECTED_LON_CENTER_DEG", 126.6))
_GPS_EXPECTED_LON_RADIUS_DEG = float(getattr(config, "GPS_EXPECTED_LON_RADIUS_DEG", 20.0))
_GPS_MAX_VALID_SPEED_MPS = float(getattr(config, "GPS_MAX_VALID_SPEED_MPS", 40.0))
_GPS_POS_MAX_DELTA_DEG = 20.0   # deg, relative to locked start longitude


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
        and 0.5 <= ground_speed <= _GPS_MAX_VALID_SPEED_MPS
    )


def _copy_deque(samples, sample_type, maxlen: Optional[int] = None):
    return deque((sample_type(**vars(sample)) for sample in samples), maxlen=maxlen)


def _gps_position_fresh(sample: _GpsFromApp, now: float) -> bool:
    return bool(
        sample.pos_health
        and sample.lat is not None
        and sample.lon is not None
        and sample.pos_ts is not None
        and 0.0 <= now - sample.pos_ts <= guidance.POS_FRESH_AGE
    )


def _gps_motion_fresh(sample: _GpsFromApp, now: float) -> bool:
    return bool(
        sample.motion_health
        and sample.course_rad is not None
        and sample.speed_mps is not None
        and sample.motion_ts is not None
        and 0.0 <= now - sample.motion_ts <= guidance.MOTION_FRESH_AGE
    )


def _gps_fresh_for_history(sample: _GpsFromApp, now: float) -> bool:
    return bool(_gps_position_fresh(sample, now) or _gps_motion_fresh(sample, now))


def _imu_fresh_for_history(sample: _ImuFromApp, now: float) -> bool:
    return bool(
        sample.health
        and sample.gyrz_rad_s is not None
        and sample.ts is not None
        and 0.0 <= now - sample.ts <= guidance.GYRZ_FRESH_AGE
    )


def _baro_fresh_for_history(sample: _BaroFromApp, now: float) -> bool:
    return bool(
        sample.health
        and sample.alt_m is not None
        and sample.ts is not None
        and 0.0 <= now - sample.ts <= guidance.ALT_FRESH_AGE
    )


def _push_latest_gps_to_history(cache: _Cache, now: float) -> None:
    if _gps_fresh_for_history(cache.latest_gps, now):
        cache.gps_history.append(_GpsFromApp(**vars(cache.latest_gps)))


def _push_latest_imu_to_history(cache: _Cache, now: float) -> None:
    if _imu_fresh_for_history(cache.latest_imu, now):
        cache.imu_history.append(_ImuFromApp(**vars(cache.latest_imu)))


def _push_latest_baro_to_history(cache: _Cache, now: float) -> None:
    if _baro_fresh_for_history(cache.latest_baro, now):
        cache.baro_history.append(_BaroFromApp(**vars(cache.latest_baro)))


def _project_latlon_to_ne(
    lat: float,
    lon: float,
    origin_lat: float,
    origin_lon: float,
) -> tuple[float, float]:
    d_n = math.radians(float(lat) - float(origin_lat)) * _EARTH_RADIUS_M
    d_e = (
        math.radians(float(lon) - float(origin_lon))
        * _EARTH_RADIUS_M
        * math.cos(math.radians(float(origin_lat)))
    )
    return d_n, d_e


def _ne_to_latlon(
    pos_n: float,
    pos_e: float,
    origin_lat: float,
    origin_lon: float,
) -> tuple[float, float]:
    lat = float(origin_lat) + math.degrees(float(pos_n) / _EARTH_RADIUS_M)
    cos_lat = max(1.0e-6, abs(math.cos(math.radians(float(origin_lat)))))
    lon = float(origin_lon) + math.degrees(float(pos_e) / (_EARTH_RADIUS_M * cos_lat))
    return lat, lon


def _valid_gps_position_samples(history, now: float):
    samples = []
    for sample in history:
        if not (
            sample.pos_health
            and sample.lat is not None
            and sample.lon is not None
            and sample.pos_ts is not None
            and 0.0 <= now - sample.pos_ts <= guidance.POS_HISTORY_AGE
        ):
            continue
        samples.append(sample)
    return samples


def _valid_gps_motion_samples(history, now: float):
    samples = []
    for sample in history:
        if not (
            sample.motion_health
            and sample.course_rad is not None
            and sample.speed_mps is not None
            and sample.motion_ts is not None
            and 0.0 <= now - sample.motion_ts <= guidance.MOTION_HISTORY_AGE
        ):
            continue
        samples.append(sample)
    return samples


def _est_position_regression_velocity(
    pos_samples,
    origin_lat: Optional[float],
    origin_lon: Optional[float],
    now: float,
) -> Optional[tuple[float, float]]:
    if origin_lat is None or origin_lon is None or len(pos_samples) < 2:
        return None
    latest = pos_samples[-1]
    oldest = None
    for sample in reversed(pos_samples[:-1]):
        if latest.pos_ts - sample.pos_ts <= GPS_REGRESSION_SEC:
            oldest = sample
        else:
            break
    if oldest is None:
        oldest = pos_samples[-2]
    dt = latest.pos_ts - oldest.pos_ts
    if dt < 0.20:
        return None
    n0, e0 = _project_latlon_to_ne(oldest.lat, oldest.lon, origin_lat, origin_lon)
    n1, e1 = _project_latlon_to_ne(latest.lat, latest.lon, origin_lat, origin_lon)
    v_n = (n1 - n0) / dt
    v_e = (e1 - e0) / dt
    speed = math.hypot(v_n, v_e)
    if not math.isfinite(speed) or speed < 0.5 or speed > _GPS_MAX_VALID_SPEED_MPS:
        return None
    return v_n, v_e


def _est_course_with_gyro_propagation(
    course_rad: float,
    motion_ts: float,
    now: float,
    gyrz_rad_s: Optional[float],
) -> tuple[float, float]:
    if (
        gyrz_rad_s is None
        or not math.isfinite(float(gyrz_rad_s))
        or motion_ts >= now
        or now - motion_ts > guidance.MOTION_HISTORY_AGE
    ):
        return float(course_rad), float(motion_ts)
    return (float(course_rad) + float(gyrz_rad_s) * (now - motion_ts)) % (2.0 * math.pi), now


def _est_gps_from_history(
    history,
    now: Optional[float] = None,
    origin_lat: Optional[float] = None,
    origin_lon: Optional[float] = None,
    gyrz_rad_s: Optional[float] = None,
) -> _GpsFromApp:
    now = time.monotonic() if now is None else now
    freshed = _GpsFromApp()
    pos_samples = _valid_gps_position_samples(history, now)
    motion_samples = _valid_gps_motion_samples(history, now)
    velocity = _est_position_regression_velocity(pos_samples, origin_lat, origin_lon, now)

    if velocity is not None:
        latest_pos = pos_samples[-1]
        age = max(0.0, now - latest_pos.pos_ts)
        n1, e1 = _project_latlon_to_ne(latest_pos.lat, latest_pos.lon, origin_lat, origin_lon)
        v_n, v_e = velocity
        lat, lon = _ne_to_latlon(n1 + v_n * age, e1 + v_e * age, origin_lat, origin_lon)
        speed = math.hypot(v_n, v_e)
        course = math.atan2(v_e, v_n) % (2.0 * math.pi)
        freshed.lat = lat
        freshed.lon = lon
        freshed.pos_ts = now
        freshed.rx_ts = latest_pos.rx_ts
        freshed.pos_health = 1
        freshed.course_rad = course
        freshed.speed_mps = speed
        freshed.motion_ts = now
        freshed.motion_health = 1

    if not freshed.pos_health and pos_samples and motion_samples and origin_lat is not None and origin_lon is not None:
        latest_pos = pos_samples[-1]
        latest_motion = motion_samples[-1]
        course, motion_ts = _est_course_with_gyro_propagation(
            latest_motion.course_rad,
            latest_motion.motion_ts,
            now,
            gyrz_rad_s,
        )
        speed = float(latest_motion.speed_mps)
        pos_age = max(0.0, now - latest_pos.pos_ts)
        n1, e1 = _project_latlon_to_ne(latest_pos.lat, latest_pos.lon, origin_lat, origin_lon)
        dist_m = speed * pos_age
        lat, lon = _ne_to_latlon(
            n1 + dist_m * math.cos(course),
            e1 + dist_m * math.sin(course),
            origin_lat,
            origin_lon,
        )
        freshed.lat = lat
        freshed.lon = lon
        freshed.pos_ts = now
        freshed.rx_ts = latest_pos.rx_ts
        freshed.pos_health = 1
        freshed.course_rad = course
        freshed.speed_mps = speed
        freshed.motion_ts = motion_ts
        freshed.motion_health = 1

    if (
        freshed.motion_health
        and gyrz_rad_s is not None
        and math.isfinite(float(gyrz_rad_s))
        and freshed.course_rad is not None
        and freshed.motion_ts is not None
        and freshed.motion_ts < now
        and now - freshed.motion_ts <= guidance.MOTION_HISTORY_AGE
    ):
        freshed.course_rad, freshed.motion_ts = _est_course_with_gyro_propagation(
            freshed.course_rad,
            freshed.motion_ts,
            now,
            gyrz_rad_s,
        )
    return freshed


def _est_imu_from_history(history, now: Optional[float] = None) -> _ImuFromApp:
    now = time.monotonic() if now is None else now
    recent = [
        sample
        for sample in history
        if (
            sample.health
            and sample.gyrz_rad_s is not None
            and sample.ts is not None
            and 0.0 <= now - sample.ts <= guidance.GYRZ_HISTORY_AGE
            and now - sample.ts <= IMU_ESTIMATE_SEC
        )
    ]
    if recent:
        weighted_sum = 0.0
        weight_total = 0.0
        for sample in recent:
            age = max(0.0, now - sample.ts)
            weight = max(0.05, 1.0 - age / max(IMU_ESTIMATE_SEC, 1.0e-6))
            weighted_sum += float(sample.gyrz_rad_s) * weight
            weight_total += weight
        latest = recent[-1]
        freshed = _ImuFromApp(**vars(latest))
        freshed.gyrz_rad_s = weighted_sum / weight_total
        freshed.health = 1
        return freshed
    for sample in reversed(history):
        if (
            sample.health
            and sample.gyrz_rad_s is not None
            and sample.ts is not None
            and 0.0 <= now - sample.ts <= guidance.GYRZ_HISTORY_AGE
        ):
            return _ImuFromApp(**vars(sample))
    return _ImuFromApp()


def _est_baro_from_history(history, now: Optional[float] = None) -> _BaroFromApp:
    now = time.monotonic() if now is None else now
    samples = [
        sample
        for sample in history
        if (
            sample.health
            and sample.alt_m is not None
            and sample.ts is not None
            and 0.0 <= now - sample.ts <= guidance.ALT_HISTORY_AGE
        )
    ]
    if not samples:
        return _BaroFromApp()

    latest = samples[-1]
    sink_rate = latest.sink_rate
    oldest = None
    for sample in reversed(samples[:-1]):
        if latest.ts - sample.ts <= BARO_REGRESSION_SEC:
            oldest = sample
        else:
            break
    if oldest is not None:
        dt = latest.ts - oldest.ts
        if dt >= 0.20:
            rate = (float(oldest.alt_m) - float(latest.alt_m)) / dt
            if math.isfinite(rate):
                sink_rate = rate

    if sink_rate is None or not math.isfinite(float(sink_rate)):
        return _BaroFromApp(**vars(latest))

    age = max(0.0, now - latest.ts)
    return _BaroFromApp(
        alt_m=float(latest.alt_m) - float(sink_rate) * age,
        sink_rate=float(sink_rate),
        ts=now,
        rx_ts=latest.rx_ts,
        health=1,
    )


def _est_dead_reckon(
    dr: _DeadReckoning,
    fresh_gps: _GpsFromApp,
    est_gps: _GpsFromApp,
    now: float,
) -> _DeadReckoning:
    """Constant-velocity dead reckoning from last GPS fix.

    anchor 갱신: fresh GPS position만 사용한다.
    motion 갱신: fresh GPS motion을 우선 사용하고, 없으면 estimated GPS motion을 보조로 사용한다.
    전파:        anchor로부터 저장된 course_rad + speed_mps 로 now 시각까지 선형 외삽.
    모션 데이터 없으면 anchor 위치를 그대로 사용 (속도 0으로 간주).
    """
    if (
        _gps_position_fresh(fresh_gps, now)
        and (dr.anchor_ts is None or fresh_gps.pos_ts > dr.anchor_ts)
    ):
        dr.anchor_lat = fresh_gps.lat
        dr.anchor_lon = fresh_gps.lon
        dr.anchor_ts  = fresh_gps.pos_ts
        dr.valid = True

    motion_source = fresh_gps if _gps_motion_fresh(fresh_gps, now) else est_gps
    if (
        motion_source is not None
        and motion_source.motion_ts is not None
        and motion_source.course_rad is not None
        and motion_source.speed_mps is not None
        and math.isfinite(float(motion_source.course_rad))
        and math.isfinite(float(motion_source.speed_mps))
        and 0.0 <= now - motion_source.motion_ts <= guidance.MOTION_DR_AGE
        and (dr.motion_ts is None or motion_source.motion_ts > dr.motion_ts)
    ):
        dr.course_rad = motion_source.course_rad
        dr.speed_mps = motion_source.speed_mps
        dr.motion_ts = motion_source.motion_ts

    if not dr.valid or dr.anchor_ts is None:
        return dr

    if now - dr.anchor_ts > guidance.POS_DR_AGE:
        dr.valid = False
        return dr

    speed  = dr.speed_mps
    course = dr.course_rad
    if speed is None or course is None:
        dr.lat = dr.anchor_lat
        dr.lon = dr.anchor_lon
        dr.ts  = now
        return dr
    if dr.motion_ts is not None and now - dr.motion_ts > guidance.MOTION_DR_AGE:
        dr.lat = dr.anchor_lat
        dr.lon = dr.anchor_lon
        dr.ts  = now
        return dr

    dt     = max(0.0, now - dr.anchor_ts)
    dist_m = float(speed) * dt

    earth_r    = 6_371_000.0
    anchor_lat = float(dr.anchor_lat)
    anchor_lon = float(dr.anchor_lon)
    d_N = dist_m * math.cos(float(course))
    d_E = dist_m * math.sin(float(course))
    dr.lat = anchor_lat + math.degrees(d_N / earth_r)
    dr.lon = anchor_lon + math.degrees(d_E / (earth_r * math.cos(math.radians(anchor_lat))))
    dr.ts  = now
    return dr


def _make_estimated_sample(
    freshed_gps: _GpsFromApp,
    freshed_imu: _ImuFromApp,
    freshed_baro: _BaroFromApp,
    now: float,
) -> _EstimatedSample:
    return _EstimatedSample(
        lat          = freshed_gps.lat,
        lon          = freshed_gps.lon,
        pos_ts       = freshed_gps.pos_ts,
        pos_valid    = bool(freshed_gps.pos_health and freshed_gps.lat is not None),
        course_rad   = freshed_gps.course_rad,
        speed_mps    = freshed_gps.speed_mps,
        motion_ts    = freshed_gps.motion_ts,
        motion_valid = bool(freshed_gps.motion_health and freshed_gps.course_rad is not None),
        gyrz_rad_s   = freshed_imu.gyrz_rad_s,
        gyrz_ts      = freshed_imu.ts,
        gyrz_valid   = bool(freshed_imu.health and freshed_imu.gyrz_rad_s is not None),
        alt_m        = freshed_baro.alt_m,
        sink_rate    = freshed_baro.sink_rate,
        alt_ts       = freshed_baro.ts,
        alt_valid    = bool(freshed_baro.health and freshed_baro.alt_m is not None),
        ts           = now,
    )


_CONTROLLER = None
_L1_STATE = None

def _cache_snapshot() -> _Cache:
    latest_gps  = _GpsFromApp(**vars(_CACHE.latest_gps))
    latest_imu  = _ImuFromApp(**vars(_CACHE.latest_imu))
    latest_baro = _BaroFromApp(**vars(_CACHE.latest_baro))
    dr          = _DeadReckoning(**vars(_CACHE.dr))

    return _Cache(
        latest_gps=latest_gps,
        gps_history=_copy_deque(_CACHE.gps_history, _GpsFromApp, _CACHE.gps_history.maxlen),
        latest_imu=latest_imu,
        imu_history=_copy_deque(_CACHE.imu_history, _ImuFromApp, _CACHE.imu_history.maxlen),
        latest_baro=latest_baro,
        baro_history=_copy_deque(_CACHE.baro_history, _BaroFromApp, _CACHE.baro_history.maxlen),
        dr=dr,
        estimated_history=_copy_deque(
            _CACHE.estimated_history, _EstimatedSample, _CACHE.estimated_history.maxlen
        ),
        target_lat=_CACHE.target_lat,
        target_lon=_CACHE.target_lon,
        start_lat=_CACHE.start_lat,
        start_lon=_CACHE.start_lon,
    )

#handler
def handle_gps(data: str) -> None:
    """lat,lon,pos_ts,course_deg,spd_mps,motion_ts — fidelity already verified by gpsapp."""
    global _START_POINT_LOCKED
    fields = data.split(",")
    if len(fields) != 6:
        LOGGER.warning("GNSS parse: expected 6 fields | raw=%r", data)
        return
    try:
        lat       = float(fields[0])
        lon       = float(fields[1])
        pos_ts    = float(fields[2])
        course_deg = float(fields[3])   # nan when motion invalid
        speed_mps  = float(fields[4])   # nan when motion invalid
        motion_ts  = float(fields[5])   # nan when motion invalid
        rx_ts = time.monotonic()
    except (ValueError, IndexError) as exc:
        LOGGER.warning("GNSS parse error: %s | raw=%r", exc, data)
        return

    with _UPDATE_LOCK:
        start_lon = _CACHE.start_lon
    pos_valid = _gps_position_sanity_reason(lat, lon, start_lon) is None
    motion_valid = pos_valid and _gps_motion_sane(course_deg, speed_mps)

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
        _push_latest_gps_to_history(_CACHE, rx_ts)
        _CACHE.latest_gps = sample
        if not _START_POINT_LOCKED and STATE >= 3 and pos_valid:
            _CACHE.start_lat = float(lat)
            _CACHE.start_lon = float(lon)
            _START_POINT_LOCKED = True
            prevstate.update_start_point(float(lat), float(lon), True)

def handle_imu(data: str) -> None:
    """roll,pitch,yaw,ax,ay,az,magx,magy,magz,gyrx,gyry,gyrz_deg_s,sample_ts,freefall,tumble,health"""
    fields = data.split(",")
    try:
        if len(fields) != 16:
            LOGGER.warning("IMU parse: expected 16 fields, got %d | raw=%r", len(fields), data)
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
        sample_ts  = float(fields[12])
        freefall   = int(float(fields[13]))
        tumble     = int(float(fields[14]))
        health     = int(float(fields[15]))
        rx_ts = time.monotonic()
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
        gyrz_rad_s=math.radians(gyrz_deg_s),
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
    """alt_m,sample_ts,sink_rate,health"""
    fields = data.split(",")
    try:
        if len(fields) != 4:
            LOGGER.warning("Baro parse: expected 4 fields, got %d | raw=%r", len(fields), data)
            return
        alt_m     = float(fields[0].strip())
        sample_ts = float(fields[1])
        sink_s    = fields[2].strip()
        sink_rate = None if sink_s == "nan" else float(sink_s)
        health    = int(float(fields[3]))
        rx_ts = time.monotonic()
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
        _push_latest_baro_to_history(_CACHE, rx_ts)
        _CACHE.latest_baro = baro


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
    with _UPDATE_LOCK:
        _PREV_STATE = STATE
        STATE = new_state
        if new_state < 3:
            _CACHE.start_lat = None
            _CACHE.start_lon = None
            _START_POINT_LOCKED = False
            prevstate.clear_start_point()
            if _L1_STATE is not None and hasattr(guidance, "l1_reset"):
                try:
                    guidance.l1_reset(_L1_STATE)
                except Exception:
                    LOGGER.debug("Failed to reset L1 state", exc_info=True)
            if _CONTROLLER is not None and hasattr(control, "controller_reset"):
                control.controller_reset(_CONTROLLER)
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
                control.SetZero(PI)
        LOGGER.info("MOTOR_ENABLED = False -> zero")
    else:
        LOGGER.warning("Unknown MEC command: %r", data)


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


def _manual_steer_command(now: float, mode: str) -> control.CtrlOutput:
    cmd = control.SetNeutral(now, f"MANUAL_{mode}")
    if mode == "LEFT":
        left_pw, right_pw, left_angle, right_angle, delta_arm, _ = control.ConnectRoMo(
            -_MANUAL_STEER_DELTA_DEG
        )
        cmd.left_pw = left_pw
        cmd.right_pw = right_pw
        cmd.left_angle_deg = left_angle
        cmd.right_angle_deg = right_angle
        cmd.delta_arm_deg = delta_arm
    elif mode == "RIGHT":
        left_pw, right_pw, left_angle, right_angle, delta_arm, _ = control.ConnectRoMo(
            _MANUAL_STEER_DELTA_DEG
        )
        cmd.left_pw = left_pw
        cmd.right_pw = right_pw
        cmd.left_angle_deg = left_angle
        cmd.right_angle_deg = right_angle
        cmd.delta_arm_deg = delta_arm
    cmd.valid = True
    cmd.fallback_mode = f"MANUAL_{mode}"
    return cmd


def handle_mtr(data: str) -> None:
    global MANUAL_STEER_MODE
    cmd = data.strip().upper()
    if cmd not in _MANUAL_STEER_MODES:
        LOGGER.warning("Unknown MTR command: %r", data)
        return
    MANUAL_STEER_MODE = cmd
    LOGGER.info("MANUAL_STEER_MODE = %s", MANUAL_STEER_MODE)


def _send_diag(main_queue, cmd, g_out, diag_state: str) -> None:
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
            _fmt(_CACHE.start_lat, 6),
            _fmt(_CACHE.start_lon, 6),
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
            _fmt(getattr(cmd, "yaw_rate_cmd_deg_s", 0.0)),
            _fmt(getattr(cmd, "yaw_rate_meas_deg_s", float("nan"))),
            _fmt(getattr(cmd, "yaw_rate_error_deg_s", 0.0)),
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
        age = now - float(timestamp)
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
                    _fmt_log(getattr(cmd, "yaw_rate_cmd_deg_s", None), 4),
                    _fmt_log(getattr(cmd, "yaw_rate_meas_deg_s", None), 4),
                    _fmt_log(getattr(cmd, "yaw_rate_error_deg_s", None), 4),
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
        now = time.monotonic()
        try:
            if not MOTOR_ENABLED or STATE < 3:
                if PI is not None:
                    control.SetZero(PI)
                idle_cmd = control.SetNeutral(now, "IDLE")
                idle_out = guidance.L1Output(timestamp=now, nominal=False, degraded=False, reason="IDLE")
                with _UPDATE_LOCK:
                    idle_snap = _cache_snapshot()
                _write_control_debug_log(now, idle_snap, None, None, idle_out, idle_cmd, "IDLE")
                _send_diag(main_queue, idle_cmd, idle_out, "IDLE")
                time.sleep(period)
                continue

            if STATE == 5:
                if PI is not None:
                    control.SetOff(PI)
                landed_cmd = control.SetNeutral(now, "LANDED")
                landed_out = guidance.L1Output(timestamp=now, nominal=False, degraded=False, reason="LANDED")
                with _UPDATE_LOCK:
                    landed_snap = _cache_snapshot()
                _write_control_debug_log(now, landed_snap, None, None, landed_out, landed_cmd, "LANDED")
                _send_diag(main_queue, landed_cmd, landed_out, "LANDED")
                time.sleep(period)
                continue

            if MANUAL_STEER_MODE in _MANUAL_STEER_MODES:
                manual_cmd = _manual_steer_command(now, MANUAL_STEER_MODE)
                manual_out = guidance.L1Output(
                    timestamp=now,
                    nominal=False,
                    degraded=False,
                    reason=f"MANUAL_{MANUAL_STEER_MODE}",
                )
                if PI is not None:
                    control.SetServoPulsewidth(PI, manual_cmd)
                with _UPDATE_LOCK:
                    manual_snap = _cache_snapshot()
                _write_control_debug_log(
                    now, manual_snap, None, None, manual_out, manual_cmd, f"MANUAL_{MANUAL_STEER_MODE}"
                )
                _send_diag(main_queue, manual_cmd, manual_out, f"MANUAL_{MANUAL_STEER_MODE}")
                time.sleep(period)
                continue

            with _UPDATE_LOCK:
                snap = _cache_snapshot()

            # DR을 현재 시각으로 갱신 (lock 밖 — snap 은 이미 복사됨)
            freshed_imu = _est_imu_from_history(snap.imu_history, now)
            freshed_gps = _est_gps_from_history(
                snap.gps_history,
                now,
                snap.start_lat,
                snap.start_lon,
                freshed_imu.gyrz_rad_s,
            )
            freshed_baro = _est_baro_from_history(snap.baro_history, now)
            _est_dead_reckon(snap.dr, snap.latest_gps, freshed_gps, now)

            est = _make_estimated_sample(freshed_gps, freshed_imu, freshed_baro, now)
            with _UPDATE_LOCK:
                _CACHE.dr = _DeadReckoning(**vars(snap.dr))
                _CACHE.estimated_history.append(est)

            l1_input, mode = guidance.ProduceL1Input(
                gps=snap.latest_gps,
                imu=snap.latest_imu,
                baro=snap.latest_baro,
                freshed_gps=freshed_gps,
                freshed_imu=freshed_imu,
                freshed_baro=freshed_baro,
                dr=snap.dr,
                origin_lat=snap.start_lat,
                origin_lon=snap.start_lon,
                target_lat=snap.target_lat,
                target_lon=snap.target_lon,
                now=now,
                l1_state=_L1_STATE,
            )
            g_out = guidance.ProduceL1Output(
                l1_input=l1_input,
                mode=mode,
                origin_lat=snap.start_lat,
                origin_lon=snap.start_lon,
                target_lat=snap.target_lat,
                target_lon=snap.target_lon,
                now=now,
                l1_state=_L1_STATE,
            )

            if bool(getattr(g_out, "nominal", False)):
                if _CONTROLLER is None:
                    _CONTROLLER = control.MakeCtrler()
                yaw_rate_meas_deg_s = float("nan")
                if snap.latest_imu.gyrz_rad_s is not None:
                    yaw_rate_meas_deg_s = math.degrees(float(snap.latest_imu.gyrz_rad_s))
                cmd = control.ProduceCtrlOutput(
                    _CONTROLLER,
                    control.ProduceCtrlInput(g_out, now),
                    yaw_rate_meas_deg_s,
                    now,
                )
            else:
                cmd = control.SetNeutral(now, getattr(g_out, "reason", "GUIDANCE_INACTIVE"))

            if PI is not None:
                control.SetServoPulsewidth(PI, cmd)
            diag_state = (
                "DEGRADED" if bool(getattr(g_out, "nominal", False)) and bool(getattr(g_out, "degraded", False))
                else "ACTIVE" if bool(getattr(g_out, "nominal", False))
                else str(getattr(g_out, "reason", "DISABLED") or "DISABLED")
            )
            _write_control_debug_log(now, snap, l1_input, mode, g_out, cmd, diag_state)
            _send_diag(main_queue, cmd, g_out, diag_state)

        except Exception as exc:
            LOGGER.error("ctrl_paragldr exception: %s", exc, exc_info=True)
            if PI is not None:
                control.SetZero(PI)
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
    elif mid == appargs.FlightlogicAppArg.MID_motor_state:
        handle_flight_state(unpacked.data)
    elif mid == appargs.FlightlogicAppArg.MID_motor_burnwire:
        handle_release(unpacked.data)
    elif mid == appargs.FlightlogicAppArg.MID_motor_EggDrop:
        handle_egg_drop()
    elif mid == appargs.CommAppArg.MID_RouteCmd_MEC:
        handle_mec(unpacked.data)
    elif mid == appargs.CommAppArg.MID_RouteCmd_FAC:
        handle_fac(unpacked.data)
    elif mid == appargs.CommAppArg.MID_RouteCmd_MTR:
        handle_mtr(unpacked.data)


def init() -> None:
    global PI, MOTOR_ENABLED, MANUAL_STEER_MODE, RELEASE_ACTION_ENABLED, EGG_ACTION_ENABLED, _START_POINT_LOCKED, _CONTROLLER, _L1_STATE
    prevstate.init_prevstate()
    MOTOR_ENABLED = prevstate.is_motor_enabled()
    MANUAL_STEER_MODE = "NEUTRAL"
    RELEASE_ACTION_ENABLED = True
    EGG_ACTION_ENABLED = True

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

    if hasattr(guidance, "make_l1_state"):
        try:
            _L1_STATE = guidance.make_l1_state()
        except Exception:
            LOGGER.debug("Failed to create L1 state", exc_info=True)

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
