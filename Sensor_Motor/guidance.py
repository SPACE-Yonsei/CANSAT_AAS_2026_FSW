import math
import logging
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional
from collections import deque
from Sensor_Motor.motorapp import _Cache

from lib import config

logger = logging.getLogger(__name__)

EARTH_RADIUS_M = 6_371_000.0

GpsAnchor
ImuAnchor
BaroAnchor

# L1 calculate
@dataclass
class MissionFrame:
    origin_lat: float = math.nan
    origin_lon: float = math.nan
    target_lat: float = math.nan
    target_lon: float = math.nan

def latlon_to_ne(lat: float, lon: float, origin_lat: float, origin_lon: float):
    """Return (N, E) metres relative to origin."""
    dLat = math.radians(lat - origin_lat)
    dLon = math.radians(lon - origin_lon)
    N = dLat * EARTH_RADIUS_M
    E = dLon * EARTH_RADIUS_M * math.cos(math.radians(origin_lat))
    return N, E

def ne_to_latlon(N: float, E: float, origin_lat: float, origin_lon: float):
    """Return (lat, lon) from local NE metres."""
    lat = origin_lat + math.degrees(N / EARTH_RADIUS_M)
    lon = origin_lon + math.degrees(
        E / (EARTH_RADIUS_M * math.cos(math.radians(origin_lat)))
    )
    return lat, lon

class ControlMode(str, Enum):
    GPS_TRACKING_CLOSED = "GPS_TRACKING_CLOSED"
    GPS_TRACKING_OPEN   = "GPS_TRACKING_OPEN"
    DR_TRACKING_CLOSED  = "DR_TRACKING_CLOSED"
    DR_TRACKING_OPEN    = "DR_TRACKING_OPEN"
    DETUMBLING          = "DETUMBLING"
    FAIL                = "FAIL"

class DRMethod(str, Enum):
    NONE                   = "NONE"
    GYRO_INTEGRATION       = "GYRO_INTEGRATION"
    ACC_DOUBLE_INTEGRATION = "ACC_DOUBLE_INTEGRATION"
    GYRO_ACC_BLEND         = "GYRO_ACC_BLEND"

#dead reckoning
def compute_dr_confidence(dr_age: float) -> float:
    a1 = config.DR_CONF_AGE_1_S
    a2 = config.DR_CONF_AGE_2_S
    a3 = config.DR_CONF_AGE_3_S
    if dr_age <= a1:
        return 1.0
    if dr_age <= a2:
        return 1.0 - 0.5 * (dr_age - a1) / max(a2 - a1, 1e-6)
    if dr_age <= a3:
        return 0.5 * (1.0 - (dr_age - a2) / max(a3 - a2, 1e-6))
    return 0.0

@dataclass
class L1Input:
    valid: bool = False
    confidence: float = 0.0
    E: float = float("nan")
    N: float = float("nan")
    V: float = float("nan")
    course: float = float("nan")
    point_age: float = float("inf")
    velocity_age: float = float("inf")
    imu_age: float = float("inf")
    barometer_age: float = float("inf")
    dr_age: float = 0.0