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

class SensorQuality(enum.Enum):
    FRESH   = "FRESH"
    OLD     = "OLD"
    STALE   = "STALE"
    FAIL    = "FAIL"
SENSOR_QUALITY = SensorQuality

class ControlMode(enum.Enum):
    ACTIVE_CLOSED_LOOP   = "ACTIVE_CLOSED_LOOP"
    ACTIVE_FEEDFORWARD   = "ACTIVE_FEEDFORWARD"
    DEGRADED_CLOSED_LOOP = "DEGRADED_CLOSED_LOOP"
    DEGRADED_FEEDFORWARD = "DEGRADED_FEEDFORWARD"
    SAFE_GLIDE           = "SAFE_GLIDE"
    FAIL                 = "FAIL"
CONTORL_MODE = ControlMode

@dataclass
class GpsSns:
    lat: Optional[float] = None
    lon: Optional[float] = None
    course: Optional[float] = None
    speed: Optional[float] = None
    pos_ts: Optional[float] = None
    motion_ts: Optional[float] = None
    pos_health: bool = False
    motion_health: bool = False
GPS_SNS = GpsSns

@dataclass
class ImuSns:
    gyrz: Optional[float] = None
    ts: Optional[float] = None
    health: bool = False
IMU_SNS = ImuSns

@dataclass
class BaroSns:
    alt: Optional[float] = None
    ts: Optional[float] = None
    health: bool = False
BARO_SNS = BaroSns

@dataclass
class L1Input:
    pos_N: Optional[float] = None
    pos_E: Optional[float] = None
    course: Optional[float] = None
    ground_speed_mps: Optional[float] = None
    gyrz: Optional[float] = None
    alt: Optional[float] = None
    pos_quality: SensorQuality = SensorQuality.FAIL
    motion_quality: SensorQuality = SensorQuality.FAIL
    gyrz_quality: SensorQuality = SensorQuality.FAIL
    alt_quality: SensorQuality = SensorQuality.FAIL
    origin_lat: Optional[float] = None
    origin_lon: Optional[float] = None
    target_lat: Optional[float] = None
    target_lon: Optional[float] = None
    control_mode: Optional[ControlMode] = None
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
    desired_heading_rad: float = 0.0


def FillFresh(
    l1_input: L1Input,
    gps: GpsSns,
    imu: ImuSns,
    baro: BaroSns,
    start_lat: float,
    start_lon: float,
    now: float,
) -> L1Input:
    """Fill L1Input with fresh sensor values only."""
    if (
        gps is not None
        and gps.lat is not None
        and gps.lon is not None
        and gps.pos_ts is not None
        and gps.pos_health
        and now - gps.pos_ts <= POS_FRESH_AGE
        and start_lat is not None
        and start_lon is not None
    ):
        earth_r = 6_371_000.0
        dlat = math.radians(gps.lat - start_lat)
        dlon = math.radians(gps.lon - start_lon)
        l1_input.pos_N = dlat * earth_r
        l1_input.pos_E = dlon * earth_r * math.cos(math.radians(start_lat))
        l1_input.pos_quality = SensorQuality.FRESH

    if (
        gps is not None
        and gps.course is not None
        and gps.speed is not None
        and gps.motion_ts is not None
        and gps.motion_health
        and now - gps.motion_ts <= MOTION_FRESH_AGE
    ):
        l1_input.course = gps.course
        l1_input.ground_speed_mps = gps.speed
        l1_input.motion_quality = SensorQuality.FRESH

    if (
        imu is not None
        and imu.gyrz is not None
        and imu.ts is not None
        and imu.health
        and now - imu.ts <= GYRZ_FRESH_AGE
    ):
        l1_input.gyrz = imu.gyrz
        l1_input.gyrz_quality = SensorQuality.FRESH

    if (
        baro is not None
        and baro.alt is not None
        and baro.ts is not None
        and baro.health
        and now - baro.ts <= ALT_FRESH_AGE
    ):
        l1_input.alt = baro.alt
        l1_input.alt_quality = SensorQuality.FRESH

    return l1_input

def FillOld(
    l1_input: L1Input,
    old_gps: GpsSns,
    old_imu: ImuSns,
    old_baro: BaroSns,
    now: float,
) -> L1Input:
    """Fill non-fresh fields with recent last-known-good sensor values."""
    if l1_input.pos_quality != SensorQuality.FRESH:
        if (
            old_gps is not None
            and old_gps.lat is not None
            and old_gps.lon is not None
            and old_gps.pos_ts is not None
            and old_gps.pos_health
        ):
            age = now - old_gps.pos_ts
            origin_lat = getattr(l1_input, "origin_lat", None)
            origin_lon = getattr(l1_input, "origin_lon", None)
            if 0.0 <= age <= POS_STALE_MAX:
                if origin_lat is not None and origin_lon is not None:
                    earth_r = 6_371_000.0
                    dlat = math.radians(old_gps.lat - origin_lat)
                    dlon = math.radians(old_gps.lon - origin_lon)
                    l1_input.pos_N = dlat * earth_r
                    l1_input.pos_E = dlon * earth_r * math.cos(math.radians(origin_lat))
                    l1_input.pos_quality = SensorQuality.OLD
                else:
                    l1_input.pos_quality = SensorQuality.FAIL
            elif age > POS_STALE_MAX:
                l1_input.pos_quality = SensorQuality.STALE
            else:
                l1_input.pos_quality = SensorQuality.FAIL
        else:
            l1_input.pos_quality = SensorQuality.FAIL

    if l1_input.motion_quality != SensorQuality.FRESH:
        if (
            old_gps is not None
            and old_gps.course is not None
            and old_gps.speed is not None
            and old_gps.motion_ts is not None
            and old_gps.motion_health
        ):
            age = now - old_gps.motion_ts
            if 0.0 <= age <= MOTION_STALE_MAX:
                l1_input.course = old_gps.course
                l1_input.ground_speed_mps = old_gps.speed
                l1_input.motion_quality = SensorQuality.OLD
            elif age > MOTION_STALE_MAX:
                l1_input.motion_quality = SensorQuality.STALE
            else:
                l1_input.motion_quality = SensorQuality.FAIL
        else:
            l1_input.motion_quality = SensorQuality.FAIL

    if l1_input.gyrz_quality != SensorQuality.FRESH:
        if (
            old_imu is not None
            and old_imu.gyrz is not None
            and old_imu.ts is not None
            and old_imu.health
        ):
            age = now - old_imu.ts
            if 0.0 <= age <= GYRZ_STALE_MAX:
                l1_input.gyrz = old_imu.gyrz
                l1_input.gyrz_quality = SensorQuality.OLD
            elif age > GYRZ_STALE_MAX:
                l1_input.gyrz_quality = SensorQuality.STALE
            else:
                l1_input.gyrz_quality = SensorQuality.FAIL
        else:
            l1_input.gyrz_quality = SensorQuality.FAIL

    if l1_input.alt_quality != SensorQuality.FRESH:
        if (
            old_baro is not None
            and old_baro.alt is not None
            and old_baro.ts is not None
            and old_baro.health
        ):
            age = now - old_baro.ts
            if 0.0 <= age <= ALT_STALE_MAX:
                l1_input.alt = old_baro.alt
                l1_input.alt_quality = SensorQuality.OLD
            elif age > ALT_STALE_MAX:
                l1_input.alt_quality = SensorQuality.STALE
            else:
                l1_input.alt_quality = SensorQuality.FAIL
        else:
            l1_input.alt_quality = SensorQuality.FAIL

    return l1_input

def DecideControlMode(l1_input: L1Input) -> ControlMode:
    pos_quality = l1_input.pos_quality
    motion_quality = l1_input.motion_quality
    gyrz_quality = l1_input.gyrz_quality

    if pos_quality in (SensorQuality.STALE, SensorQuality.FAIL):
        return ControlMode.FAIL
    if motion_quality in (SensorQuality.STALE, SensorQuality.FAIL):
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
    gps: GpsSns,
    imu: ImuSns,
    baro: BaroSns,
    old_gps: GpsSns,
    old_imu: ImuSns,
    old_baro: BaroSns,
    origin_lat: float,
    origin_lon: float,
    target_lat: float,
    target_lon: float,
    now: float,
    l1_state=None,
) -> tuple[L1Input, ControlMode]:
    l1_input = L1Input()
    l1_input.origin_lat = origin_lat
    l1_input.origin_lon = origin_lon
    l1_input.target_lat = target_lat
    l1_input.target_lon = target_lon

    FillFresh(l1_input, gps, imu, baro, origin_lat, origin_lon, now)
    FillOld(l1_input, old_gps, old_imu, old_baro, now)
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

    desired_heading = math.atan2(carrot_E - pos_E, carrot_N - pos_N)
    nu = (desired_heading - course + math.pi) % (2.0 * math.pi) - math.pi
    K_L1 = 4.0 * L1_DAMPING * L1_DAMPING
    lat_acc = K_L1 * speed_for_l1 * speed_for_l1 / L1_distance * math.sin(nu)
    lat_acc = max(-LAT_ACC_MAX, min(LAT_ACC_MAX, lat_acc))
    yaw_rate = lat_acc / speed_for_l1
    yaw_rate = max(-COURSE_RATE_MAX, min(COURSE_RATE_MAX, yaw_rate))

    out.active = True
    out.degraded = mode in (ControlMode.DEGRADED_CLOSED_LOOP, ControlMode.DEGRADED_FEEDFORWARD)
    out.yaw_rate_cmd_rad_s = yaw_rate
    out.lat_acc_cmd_mps2 = lat_acc
    out.ground_speed_mps = float(speed)
    out.L1_distance = L1_distance
    out.nu1 = math.atan2(cross, max(L1_distance, 1e-6))
    out.nu2 = nu - out.nu1
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
    out.desired_heading_rad = desired_heading
    return out
