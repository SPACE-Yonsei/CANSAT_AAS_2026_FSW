"""Guidance module: L1 guidance and motor output computation.

Implements ArduPilot L1 navigation controller logic ported to Python.
Reference: libraries/AP_L1_Control/AP_L1_Control.cpp :: update_waypoint()

Sign convention: commanded_yaw_rate < 0 = LEFT turn.
Units: _deg / _rad / _m / _ms / _mps suffixes throughout.
"""

from __future__ import annotations
from dataclasses import dataclass
import enum
import math
import time
from typing import Optional, Tuple

#Sensor age
POS_FRESH_AGE    = 1.0    # s
POS_STALE_MAX    = 3.0    # s
MOTION_FRESH_AGE = 1.0    # s
MOTION_STALE_MAX = 3.0    # s
GYRZ_FRESH_AGE   = 0.20   # s
GYRZ_STALE_MAX   = 0.50   # s
ALT_FRESH_AGE    = 0.50   # s
ALT_STALE_MAX    = 3.0    # s

# ── L1 parameters ─────────────────────────────────────────────────────────────
L1_DAMPING         = 0.75
L1_PERIOD_S        = 8.0
L1_MIN_M           = 5.0
V_MIN_MPS          = 2.0
LAT_ACC_MAX        = 4.0    # m/s^2
COURSE_RATE_MAX    = 0.6    # rad/s
XTRACK_SOFT_FACTOR = 2.0    # × L1_dist
XTRACK_HARD_FACTOR = 4.0    # × L1_dist
XTRACK_SOFT_MIN_M  = 20.0
XTRACK_HARD_MIN_M  = 50.0
# GPS-derived position sanity: reject positions farther than this from origin.
# Catches cases where GPS lon is near zero while origin is at ~126 °E (≈ 11 000 km error).
_MAX_POS_RANGE_M   = 50_000.0   # 50 km


def _project_position_from_origin(
    lat: float,
    lon: float,
    origin_lat: float,
    origin_lon: float,
) -> Optional[Tuple[float, float]]:
    try:
        la = float(lat)
        lo = float(lon)
        ola = float(origin_lat)
        olo = float(origin_lon)
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(v) for v in (la, lo, ola, olo)):
        return None
    earth_r = 6_371_000.0
    dlat = math.radians(la - ola)
    dlon = math.radians(lo - olo)
    pos_N = dlat * earth_r
    pos_E = dlon * earth_r * math.cos(math.radians(ola))
    if math.hypot(pos_N, pos_E) > _MAX_POS_RANGE_M:
        return None
    return pos_N, pos_E

class SensorQuality(enum.Enum):
    FRESH   = "FRESH"
    OLD     = "OLD"
    STALE   = "STALE"
SENSOR_QUALITY = SensorQuality

class ControlMode(enum.Enum):
    ACTIVE_CLOSED_LOOP   = "ACTIVE_CLOSED_LOOP"
    ACTIVE_FEEDFORWARD   = "ACTIVE_FEEDFORWARD"
    DEGRADED_CLOSED_LOOP = "DEGRADED_CLOSED_LOOP"
    DEGRADED_FEEDFORWARD = "DEGRADED_FEEDFORWARD"
    FAIL                 = "FAIL"
CONTORL_MODE = ControlMode

@dataclass
class L1Input:
    pos_N: Optional[float] = None
    pos_E: Optional[float] = None
    course: Optional[float] = None
    ground_speed_mps: Optional[float] = None
    gyrz: Optional[float] = None
    alt: Optional[float] = None
    pos_quality: SensorQuality = SensorQuality.STALE
    motion_quality: SensorQuality = SensorQuality.STALE
    gyrz_quality: SensorQuality = SensorQuality.STALE
    alt_quality: SensorQuality = SensorQuality.STALE
    origin_lat: Optional[float] = None
    origin_lon: Optional[float] = None
    target_lat: Optional[float] = None
    target_lon: Optional[float] = None
    control_mode: Optional[ControlMode] = None
    sink_rate: Optional[float] = None
    freefall: int = 1
    tumble: int = 1
L1_INPUT = L1Input()

#have to add member
@dataclass
class L1Output:
    timestamp: float = 0.0
    active: bool = False
    degraded: bool = False
    reason: str = "INIT"
    yaw_rate_cmd_rad_s: float = 0.0
    lat_acc_cmd_mps2: float = 0.0
    ground_speed_mps: float = 0.0
    L1_distance: float = 0.0
    nu1: float = 0.0
    nu2: float = 0.0
    nu: float = 0.0
    crossTrack: float = 0.0
    alongTrack: float = 0.0
    pos_N: float = 0.0
    pos_E: float = 0.0
    target_N: float = 0.0
    target_E: float = 0.0
    carrot_N: float = 0.0
    carrot_E: float = 0.0
    target_lat: Optional[float] = None
    target_lon: Optional[float] = None
    carrot_lat: Optional[float] = None
    carrot_lon: Optional[float] = None
    current_heading_rad: float = 0.0


def FillFresh(
    l1_input: L1Input,
    gps,
    imu,
    baro,
    start_lat: float,
    start_lon: float,
    now: float,
) -> L1Input:
    """Fill L1Input with fresh sensor values only."""
    gps_course = getattr(gps, "course", getattr(gps, "course_rad", None)) if gps is not None else None
    gps_speed = getattr(gps, "speed", getattr(gps, "speed_mps", None)) if gps is not None else None
    imu_gyrz = getattr(imu, "gyrz", getattr(imu, "gyrz_rad_s", None)) if imu is not None else None
    baro_alt = getattr(baro, "alt", getattr(baro, "alt_m", None)) if baro is not None else None

    if (
        gps is not None
        and gps.lat is not None
        and gps.lon is not None
        and gps.pos_ts is not None
        and now - gps.pos_ts <= POS_FRESH_AGE
        and start_lat is not None
        and start_lon is not None
    ):
        projected = _project_position_from_origin(gps.lat, gps.lon, start_lat, start_lon)
        if projected is None:
            l1_input.pos_N = None
            l1_input.pos_E = None
            l1_input.pos_quality = SensorQuality.STALE
        else:
            l1_input.pos_N, l1_input.pos_E = projected
            l1_input.pos_quality = SensorQuality.FRESH

    if (
        gps is not None
        and gps_course is not None
        and gps_speed is not None
        and gps.motion_ts is not None
        and now - gps.motion_ts <= MOTION_FRESH_AGE
    ):
        l1_input.course = gps_course
        l1_input.ground_speed_mps = gps_speed
        l1_input.motion_quality = SensorQuality.FRESH

    if (
        imu is not None
        and imu_gyrz is not None
        and imu.ts is not None
        and getattr(imu, "health", 1)
        and now - imu.ts <= GYRZ_FRESH_AGE
    ):
        l1_input.gyrz = imu_gyrz
        l1_input.gyrz_quality = SensorQuality.FRESH
        l1_input.freefall = getattr(imu, "freefall", 0)
        l1_input.tumble   = getattr(imu, "tumble",   0)

    if (
        baro is not None
        and baro_alt is not None
        and baro.ts is not None
        and getattr(baro, "health", 1)
        and now - baro.ts <= ALT_FRESH_AGE
    ):
        l1_input.alt         = baro_alt
        l1_input.alt_quality = SensorQuality.FRESH
        l1_input.sink_rate   = getattr(baro, "sink_rate", None)

    return l1_input

def FillOld(
    l1_input: L1Input,
    old_gps,
    old_imu,
    old_baro,
    now: float,
    dr=None,
) -> L1Input:
    """Fill non-fresh fields with recent last-known-good sensor values."""
    old_gps_course = getattr(old_gps, "course", getattr(old_gps, "course_rad", None)) if old_gps is not None else None
    old_gps_speed = getattr(old_gps, "speed", getattr(old_gps, "speed_mps", None)) if old_gps is not None else None
    old_imu_gyrz = getattr(old_imu, "gyrz", getattr(old_imu, "gyrz_rad_s", None)) if old_imu is not None else None
    old_baro_alt = getattr(old_baro, "alt", getattr(old_baro, "alt_m", None)) if old_baro is not None else None

    if l1_input.pos_quality != SensorQuality.FRESH:
        origin_lat = getattr(l1_input, "origin_lat", None)
        origin_lon = getattr(l1_input, "origin_lon", None)
        pos_filled = False

        # 1차: last_gps 로 시도
        if (
            old_gps is not None
            and old_gps.lat is not None
            and old_gps.lon is not None
            and old_gps.pos_ts is not None
            and 0.0 <= now - old_gps.pos_ts <= POS_STALE_MAX
            and origin_lat is not None
            and origin_lon is not None
        ):
            projected = _project_position_from_origin(
                old_gps.lat, old_gps.lon, origin_lat, origin_lon
            )
            if projected is not None:
                l1_input.pos_N, l1_input.pos_E = projected
                l1_input.pos_quality = SensorQuality.OLD
                pos_filled = True

        # 2차: dead reckoning 으로 fallback
        if not pos_filled and dr is not None and getattr(dr, "valid", False):
            dr_lat = getattr(dr, "lat", None)
            dr_lon = getattr(dr, "lon", None)
            if (
                dr_lat is not None
                and dr_lon is not None
                and origin_lat is not None
                and origin_lon is not None
            ):
                projected = _project_position_from_origin(dr_lat, dr_lon, origin_lat, origin_lon)
                if projected is not None:
                    l1_input.pos_N, l1_input.pos_E = projected
                    l1_input.pos_quality = SensorQuality.OLD
                    pos_filled = True

        if not pos_filled:
            l1_input.pos_quality = SensorQuality.STALE

    if l1_input.motion_quality != SensorQuality.FRESH:
        if (
            old_gps is not None
            and old_gps_course is not None
            and old_gps_speed is not None
            and old_gps.motion_ts is not None
        ):
            age = now - old_gps.motion_ts
            if 0.0 <= age <= MOTION_STALE_MAX:
                l1_input.course = old_gps_course
                l1_input.ground_speed_mps = old_gps_speed
                l1_input.motion_quality = SensorQuality.OLD
            else:
                l1_input.motion_quality = SensorQuality.STALE
        else:
            l1_input.motion_quality = SensorQuality.STALE

    if l1_input.gyrz_quality != SensorQuality.FRESH:
        if (
            old_imu is not None
            and old_imu_gyrz is not None
            and old_imu.ts is not None
            and getattr(old_imu, "health", 1)
        ):
            age = now - old_imu.ts
            if 0.0 <= age <= GYRZ_STALE_MAX:
                l1_input.gyrz         = old_imu_gyrz
                l1_input.gyrz_quality = SensorQuality.OLD
                l1_input.freefall     = getattr(old_imu, "freefall", 0)
                l1_input.tumble       = getattr(old_imu, "tumble",   0)
            else:
                l1_input.gyrz_quality = SensorQuality.STALE
        else:
            l1_input.gyrz_quality = SensorQuality.STALE

    if l1_input.alt_quality != SensorQuality.FRESH:
        if (
            old_baro is not None
            and old_baro_alt is not None
            and old_baro.ts is not None
            and getattr(old_baro, "health", 1)
        ):
            age = now - old_baro.ts
            if 0.0 <= age <= ALT_STALE_MAX:
                l1_input.alt         = old_baro_alt
                l1_input.alt_quality = SensorQuality.OLD
            else:
                l1_input.alt_quality = SensorQuality.STALE
        else:
            l1_input.alt_quality = SensorQuality.STALE

    return l1_input

def DecideControlMode(l1_input: L1Input) -> ControlMode:
    # 1=자유낙하 중, 1=텀블링 중 → FAIL
    if l1_input.freefall:
        return ControlMode.FAIL
    if l1_input.tumble:
        return ControlMode.FAIL

    pos_quality    = l1_input.pos_quality
    motion_quality = l1_input.motion_quality
    gyrz_quality   = l1_input.gyrz_quality

    if pos_quality    == SensorQuality.STALE:
        return ControlMode.FAIL
    if motion_quality == SensorQuality.STALE:
        return ControlMode.FAIL

    closed_loop = (
        l1_input.gyrz is not None
        and gyrz_quality in (SensorQuality.FRESH, SensorQuality.OLD)
    )
    active = pos_quality == SensorQuality.FRESH and motion_quality == SensorQuality.FRESH

    if active and closed_loop:
        return ControlMode.ACTIVE_CLOSED_LOOP
    if active:
        return ControlMode.ACTIVE_FEEDFORWARD
    if closed_loop:
        return ControlMode.DEGRADED_CLOSED_LOOP
    return ControlMode.DEGRADED_FEEDFORWARD

#prepocessing: fill fresh -> fill unfresh -> decide control mode -> produce L1 input
#receives data directly from apps 
def ProduceL1Input(
    gps,
    imu,
    baro,
    old_gps,
    old_imu,
    old_baro,
    origin_lat: float,
    origin_lon: float,
    target_lat: float,
    target_lon: float,
    now: float,
    l1_state=None,
    dr=None,
) -> tuple[L1Input, ControlMode]:
    l1_input = L1Input()
    l1_input.origin_lat = origin_lat
    l1_input.origin_lon = origin_lon
    l1_input.target_lat = target_lat
    l1_input.target_lon = target_lon

    FillFresh(l1_input, gps, imu, baro, origin_lat, origin_lon, now)
    FillOld(l1_input, old_gps, old_imu, old_baro, now, dr=dr)
    control_mode = DecideControlMode(l1_input)
    l1_input.control_mode = control_mode
    return (l1_input, control_mode)

def ProduceL1Output(
    l1_input: L1Input,
    mode: ControlMode,
    origin_lat: float,
    origin_lon: float,
    target_lat: float,
    target_lon: float,
    now: float,
    l1_state=None,
) -> L1Output:
    out = L1Output(timestamp=now)
    out.reason = getattr(mode, "value", str(mode))
    out.target_lat = target_lat
    out.target_lon = target_lon

    if mode == ControlMode.FAIL:
        out.reason = "FAIL"
        return out

    pos_N = getattr(l1_input, "pos_N", None)
    pos_E = getattr(l1_input, "pos_E", None)
    course = getattr(l1_input, "course", None)
    speed = getattr(l1_input, "ground_speed_mps", None)
    if (
        pos_N is None
        or pos_E is None
        or course is None
        or speed is None
        or origin_lat is None
        or origin_lon is None
        or target_lat is None
        or target_lon is None
    ):
        out.reason = "FAIL_L1_INPUT"
        return out

    earth_r = 6_371_000.0
    dlat = math.radians(target_lat - origin_lat)
    dlon = math.radians(target_lon - origin_lon)
    target_N = dlat * earth_r
    target_E = dlon * earth_r * math.cos(math.radians(origin_lat))
    path_len = math.hypot(target_N, target_E)
    if path_len <= 1e-6:
        out.reason = "INVALID_PATH"
        return out

    speed_for_l1 = max(float(speed), V_MIN_MPS)
    L1_distance = max((L1_DAMPING * L1_PERIOD_S / math.pi) * speed_for_l1, L1_MIN_M)
    unit_N = target_N / path_len
    unit_E = target_E / path_len
    along = pos_N * unit_N + pos_E * unit_E
    cross = unit_N * pos_E - unit_E * pos_N
    carrot_along = min(max(along + L1_distance, 0.0), path_len)
    carrot_N = carrot_along * unit_N
    carrot_E = carrot_along * unit_E

    path_heading = math.atan2(unit_E, unit_N)
    # nu1: turn angle caused by position error. It drives cross-track error
    # back toward the path.
    nu1 = math.atan2(-cross, max(L1_distance, 1e-6))
    # nu2: turn angle caused by direction error. It aligns current course to
    # the path heading.
    nu2 = (path_heading - course + math.pi) % (2.0 * math.pi) - math.pi
    nu = (nu1 + nu2 + math.pi) % (2.0 * math.pi) - math.pi
    nu_clamped = max(-math.pi / 2.0, min(math.pi / 2.0, nu))
    K_L1 = 4.0 * L1_DAMPING * L1_DAMPING
    lat_acc = K_L1 * speed_for_l1 * speed_for_l1 / L1_distance * math.sin(nu_clamped)
    lat_acc = max(-LAT_ACC_MAX, min(LAT_ACC_MAX, lat_acc))
    yaw_rate = lat_acc / speed_for_l1
    yaw_rate = max(-COURSE_RATE_MAX, min(COURSE_RATE_MAX, yaw_rate))

    out.active = True
    out.degraded = mode in (ControlMode.DEGRADED_CLOSED_LOOP, ControlMode.DEGRADED_FEEDFORWARD)
    out.yaw_rate_cmd_rad_s = yaw_rate
    out.lat_acc_cmd_mps2 = lat_acc
    out.ground_speed_mps = float(speed)
    out.L1_distance = L1_distance
    out.nu1 = nu1
    out.nu2 = nu2
    out.nu = nu
    out.crossTrack = cross
    out.alongTrack = along
    out.pos_N = pos_N
    out.pos_E = pos_E
    out.target_N = target_N
    out.target_E = target_E
    out.carrot_N = carrot_N
    out.carrot_E = carrot_E
    out.carrot_lat = origin_lat + math.degrees(carrot_N / earth_r)
    out.carrot_lon = origin_lon + math.degrees(carrot_E / (earth_r * math.cos(math.radians(origin_lat))))
    out.current_heading_rad = course
    return out
