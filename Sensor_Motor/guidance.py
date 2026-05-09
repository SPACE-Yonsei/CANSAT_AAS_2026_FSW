"""Guidance module: L1 guidance and motor output computation.

Implements ArduPilot L1 navigation controller logic ported to Python.
Reference: libraries/AP_L1_Control/AP_L1_Control.cpp :: update_waypoint()

Sign convention: commanded_yaw_rate > 0 = LEFT turn.
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
    MISSING = "MISSING"
SENSOR_QUALITY = SensorQuality

class ControlMode(enum.Enum):
    ACTIVE_CLOSED_LOOP   = "ACTIVE_CLOSED_LOOP"
    ACTIVE_FEEDFORWARD   = "ACTIVE_FEEDFORWARD"
    DEGRADED_CLOSED_LOOP = "DEGRADED_CLOSED_LOOP"
    DEGRADED_FEEDFORWARD = "DEGRADED_FEEDFORWARD"
    SAFE_GLIDE           = "SAFE_GLIDE"
CONTORL_MODE = ControlMode

@dataclass
class GpsSns:
    # 필드 정의 (타입 힌트 포함)
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
    pos: Optional[tuple[float, float]] = None  # (lat, lon)
    course: Optional[float] = None  # course (deg)
    speed: Optional[float] = None  # speed (m/s)
    gyrz: Optional[float] = None  # gyro
L1_INPUT = L1Input()

#have to add member
@dataclass
class L1Output:
    L1_distance
    nu1
    nu2
    nu


#have to add parameters
def FillFresh(GpsSns, GyrZ, Alt) -> L1Input:
    now = time.time() if now is None else now
    lcsg = L1Input() if lcsg is None else lcsg

    if (
        gps.lat is not None
        and gps.lon is not None
        and gps.pos_ts is not None
        and gps.pos_health
        and now - gps.pos_ts <= POS_FRESH_AGE
    ):
        lcsg.l = (gps.lat, gps.lon)
        earth_r = 6_371_000.0
        dlat = math.radians(gps.lat - origin_lat)
        dlon = math.radians(gps.lon - origin_lon)
        lcsg.pos_N = dlat * earth_r
        lcsg.pos_E = dlon * earth_r * math.cos(math.radians(origin_lat))
        lcsg.pos_status = SensorQuality.FRESH
    elif gps.pos_ts is not None:
        lcsg.pos_status = SensorQuality.STALE

    if (
        gps.course is not None
        and gps.speed is not None
        and gps.motion_ts is not None
        and gps.motion_health
        and now - gps.motion_ts <= MOTION_FRESH_AGE
    ):
        lcsg.course = gps.course
        lcsg.ground_speed_mps = gps.speed
    
    if (
        gyrz is not None
        and gyrz.gyrz is not None
        and gyrz.ts is not None
        and gyrz.health
        and now - gyrz.ts <= GYRZ_FRESH_AGE
    ):
        lcsg.g = gyrz
        lcsg.gyrz = gyrz.gyrz
        lcsg.gyrz_health = SensorQuality.FRESH
    elif gyrz is not None and gyrz.ts is not None:
        lcsg.gyrz_health = SensorQuality.STALE

    if (
        alt is not None
        and alt.alt is not None
        and alt.ts is not None
        and alt.health
        and now - alt.ts <= ALT_FRESH_AGE
    ):
        lcsg.altitude = alt.alt
        lcsg.alt_health = SensorQuality.FRESH
    elif alt is not None and alt.ts is not None:
        lcsg.alt_health = SensorQuality.STALE

    return lcsg

def FillOld()->L1Input:
    pass

def DecideControlMode()->ControlMode:
    return ControlMode.SAFE_GLIDE

#prepocessing: fill fresh -> fill unfresh -> decide control mode -> produce L1 input
#receives data directly from apps 
def ProduceL1Input(gps: GpsSns, imu: ImuSns, ) -> (L1Input, ControlMode):
    FillFresh()
    FillOld()
    DecideControlMode()

def ProduceL1Output() -> float:
    pass
