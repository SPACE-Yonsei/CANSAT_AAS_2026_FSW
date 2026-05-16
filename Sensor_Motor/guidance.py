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
from typing import Optional

#Sensor age
POS_FRESH_AGE    = 1.0    # s
MOTION_FRESH_AGE = 1.0    # s
GYRZ_FRESH_AGE   = 0.20   # s
ALT_FRESH_AGE    = 0.50   # s

# History sample age limits for estimation inputs.
POS_HISTORY_AGE    = 5.0    # s
MOTION_HISTORY_AGE = 5.0    # s
GYRZ_HISTORY_AGE   = 0.50   # s
ALT_HISTORY_AGE    = 3.0    # s

# Estimated value age limits for guidance inputs.
POS_EST_AGE    = 5.0    # s
MOTION_EST_AGE = 5.0    # s
GYRZ_EST_AGE   = 0.50   # s
ALT_EST_AGE    = 3.0    # s

# Dead-reckoning state retention limits.
POS_DR_AGE       = 5.0    # s
MOTION_DR_AGE    = 5.0    # s
GYRZ_DR_AGE      = 0.50   # s
ALT_DR_AGE       = 3.0    # s

# Backward-compatible names. Prefer explicit *_HISTORY_AGE, *_EST_AGE, or *_DR_AGE.
POS_STALE_MAX    = POS_EST_AGE
MOTION_STALE_MAX = MOTION_EST_AGE
GYRZ_STALE_MAX   = GYRZ_EST_AGE
ALT_STALE_MAX    = ALT_EST_AGE

# ── L1 parameters ─────────────────────────────────────────────────────────────
L1_DAMPING         = 0.75
L1_PERIOD_S        = 8.0
L1_MIN_M           = 5.0
V_MIN_MPS          = 2.0
LAT_ACC_MAX        = 4.0    # m/s^2
COURSE_RATE_MAX    = 0.6    # rad/s
# GPS-derived position sanity: reject positions farther than this from origin.
# Catches cases where GPS lon is near zero while origin is at ~126 °E (≈ 11 000 km error).
_MAX_POS_RANGE_M   = 50_000.0   # 50 km
EARTH_RADIUS_M     = 6_371_000.0


def latlon_to_ne(
    lat: float,
    lon: float,
    origin_lat: float,
    origin_lon: float,
) -> tuple[float, float]:
    d_n = math.radians(float(lat) - float(origin_lat)) * EARTH_RADIUS_M
    d_e = (
        math.radians(float(lon) - float(origin_lon))
        * EARTH_RADIUS_M
        * math.cos(math.radians(float(origin_lat)))
    )
    return d_n, d_e


def ne_to_latlon(
    pos_n: float,
    pos_e: float,
    origin_lat: float,
    origin_lon: float,
) -> tuple[float, float]:
    lat = float(origin_lat) + math.degrees(float(pos_n) / EARTH_RADIUS_M)
    cos_lat = max(1.0e-6, abs(math.cos(math.radians(float(origin_lat)))))
    lon = float(origin_lon) + math.degrees(float(pos_e) / (EARTH_RADIUS_M * cos_lat))
    return lat, lon


def project_from_origin(
    lat: float,
    lon: float,
    origin_lat: float,
    origin_lon: float,
    max_range_m: Optional[float] = None,
) -> Optional[tuple[float, float]]:
    try:
        pos_n, pos_e = latlon_to_ne(lat, lon, origin_lat, origin_lon)
    except (TypeError, ValueError):
        return None
    values = (float(lat), float(lon), float(origin_lat), float(origin_lon), pos_n, pos_e)
    if not all(math.isfinite(v) for v in values):
        return None
    if max_range_m is not None and math.hypot(pos_n, pos_e) > float(max_range_m):
        return None
    return pos_n, pos_e


class SensorQuality(enum.Enum):
    FRESH   = "FRESH"
    FRESHED = "FRESHED"
    STALE   = "STALE"

class ControlMode(enum.Enum):
    NOMINAL_CLOSED_LOOP  = "NOMINAL_CLOSED_LOOP"
    NOMINAL_FEEDFORWARD  = "NOMINAL_FEEDFORWARD"
    DEGRADED_CLOSED_LOOP = "DEGRADED_CLOSED_LOOP"
    DEGRADED_FEEDFORWARD = "DEGRADED_FEEDFORWARD"
    FAIL                 = "FAIL"

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
    freefall: int = 0
    tumble: int = 0


@dataclass
class L1State:
    """Reserved state for future degraded/weak L1 command shaping."""
    last_yaw_rate_cmd_rad_s: float = 0.0
    last_update_ts: Optional[float] = None


def make_l1_state() -> L1State:
    return L1State()


def l1_reset(state: L1State) -> None:
    state.last_yaw_rate_cmd_rad_s = 0.0
    state.last_update_ts = None

#have to add member
@dataclass
class L1Output:
    timestamp: float = 0.0
    nominal: bool = False
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
        and getattr(gps, "pos_health", 1)
        and now - gps.pos_ts <= POS_FRESH_AGE
        and start_lat is not None
        and start_lon is not None
    ):
        projected = project_from_origin(
            gps.lat, gps.lon, start_lat, start_lon, max_range_m=_MAX_POS_RANGE_M
        )
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
        and getattr(gps, "motion_health", 1)
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

def FillFreshed(
    l1_input: L1Input,
    freshed_gps,
    freshed_imu,
    freshed_baro,
    now: float,
    dr=None,
) -> L1Input:
    """Fill non-fresh fields with history-derived freshed estimates."""
    freshed_gps_course = getattr(freshed_gps, "course", getattr(freshed_gps, "course_rad", None)) if freshed_gps is not None else None
    freshed_gps_speed = getattr(freshed_gps, "speed", getattr(freshed_gps, "speed_mps", None)) if freshed_gps is not None else None
    freshed_imu_gyrz = getattr(freshed_imu, "gyrz", getattr(freshed_imu, "gyrz_rad_s", None)) if freshed_imu is not None else None
    freshed_baro_alt = getattr(freshed_baro, "alt", getattr(freshed_baro, "alt_m", None)) if freshed_baro is not None else None

    if l1_input.pos_quality != SensorQuality.FRESH:
        origin_lat = getattr(l1_input, "origin_lat", None)
        origin_lon = getattr(l1_input, "origin_lon", None)
        pos_filled = False

        # 1차: history-derived GPS estimate.
        if (
            freshed_gps is not None
            and freshed_gps.lat is not None
            and freshed_gps.lon is not None
            and freshed_gps.pos_ts is not None
            and getattr(freshed_gps, "pos_health", 1)
            and 0.0 <= now - freshed_gps.pos_ts <= POS_EST_AGE
            and origin_lat is not None
            and origin_lon is not None
        ):
            projected = project_from_origin(
                freshed_gps.lat,
                freshed_gps.lon,
                origin_lat,
                origin_lon,
                max_range_m=_MAX_POS_RANGE_M,
            )
            if projected is not None:
                l1_input.pos_N, l1_input.pos_E = projected
                l1_input.pos_quality = SensorQuality.FRESHED
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
                projected = project_from_origin(
                    dr_lat, dr_lon, origin_lat, origin_lon, max_range_m=_MAX_POS_RANGE_M
                )
                if projected is not None:
                    l1_input.pos_N, l1_input.pos_E = projected
                    l1_input.pos_quality = SensorQuality.FRESHED
                    pos_filled = True

        if not pos_filled:
            l1_input.pos_quality = SensorQuality.STALE

    if l1_input.motion_quality != SensorQuality.FRESH:
        if (
            freshed_gps is not None
            and freshed_gps_course is not None
            and freshed_gps_speed is not None
            and freshed_gps.motion_ts is not None
            and getattr(freshed_gps, "motion_health", 1)
        ):
            age = now - freshed_gps.motion_ts
            if 0.0 <= age <= MOTION_EST_AGE:
                l1_input.course = freshed_gps_course
                l1_input.ground_speed_mps = freshed_gps_speed
                l1_input.motion_quality = SensorQuality.FRESHED
            else:
                l1_input.motion_quality = SensorQuality.STALE
        else:
            l1_input.motion_quality = SensorQuality.STALE

    if l1_input.gyrz_quality != SensorQuality.FRESH:
        if (
            freshed_imu is not None
            and freshed_imu_gyrz is not None
            and freshed_imu.ts is not None
            and getattr(freshed_imu, "health", 1)
        ):
            age = now - freshed_imu.ts
            if 0.0 <= age <= GYRZ_EST_AGE:
                l1_input.gyrz         = freshed_imu_gyrz
                l1_input.gyrz_quality = SensorQuality.FRESHED
                l1_input.freefall     = getattr(freshed_imu, "freefall", 0)
                l1_input.tumble       = getattr(freshed_imu, "tumble",   0)
            else:
                l1_input.gyrz_quality = SensorQuality.STALE
        else:
            l1_input.gyrz_quality = SensorQuality.STALE

    if l1_input.alt_quality != SensorQuality.FRESH:
        if (
            freshed_baro is not None
            and freshed_baro_alt is not None
            and freshed_baro.ts is not None
            and getattr(freshed_baro, "health", 1)
        ):
            age = now - freshed_baro.ts
            if 0.0 <= age <= ALT_EST_AGE:
                l1_input.alt         = freshed_baro_alt
                l1_input.alt_quality = SensorQuality.FRESHED
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
        and gyrz_quality in (SensorQuality.FRESH, SensorQuality.FRESHED)
    )
    nominal = pos_quality == SensorQuality.FRESH and motion_quality == SensorQuality.FRESH

    if nominal and closed_loop:
        return ControlMode.NOMINAL_CLOSED_LOOP
    if nominal:
        return ControlMode.NOMINAL_FEEDFORWARD
    if closed_loop:
        return ControlMode.DEGRADED_CLOSED_LOOP
    return ControlMode.DEGRADED_FEEDFORWARD

#prepocessing: fill fresh -> fill unfresh -> decide control mode -> produce L1 input
#receives data directly from apps 
def ProduceL1Input(
    gps,
    imu,
    baro,
    freshed_gps,
    freshed_imu,
    freshed_baro,
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
    FillFreshed(l1_input, freshed_gps, freshed_imu, freshed_baro, now, dr=dr)
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

    target_N, target_E = latlon_to_ne(target_lat, target_lon, origin_lat, origin_lon)
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

    out.nominal = True
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
    out.carrot_lat, out.carrot_lon = ne_to_latlon(carrot_N, carrot_E, origin_lat, origin_lon)
    out.current_heading_rad = course
    return out
