"""Guidance module: L1 guidance and motor output computation.

Implements ArduPilot L1 navigation controller logic ported to Python.
Reference: libraries/AP_L1_Control/AP_L1_Control.cpp :: update_waypoint()

Sign convention: commanded_yaw_rate > 0 = LEFT turn.
Units: _deg / _rad / _m / _ms / _mps suffixes throughout.
"""

from __future__ import annotations
import enum
from types import SimpleNamespace
from typing import Optional

# ── Sensor age constants ──────────────────────────────────────────────────────
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

# ── Earth geometry ────────────────────────────────────────────────────────────
EARTH_R = 6_371_000.0  # m

# ═══════════════════════════════════════════════════════════════════════════════
# Status string constants
# ═══════════════════════════════════════════════════════════════════════════════

class SensorQuality(enum):
    FRESH   = "fresh"
    STALE   = "stale"
    MISSING = "missing"

class ControlMode(enum):
    ACTIVE_CLOSED_LOOP   = "ACTIVE_CLOSED_LOOP"
    ACTIVE_FEEDFORWARD   = "ACTIVE_FEEDFORWARD"
    DEGRADED_CLOSED_LOOP = "DEGRADED_CLOSED_LOOP"
    DEGRADED_FEEDFORWARD = "DEGRADED_FEEDFORWARD"
    SAFE_GLIDE           = "SAFE_GLIDE"

point = SimpleNamespace(x,y)
vector = SimpleNamespace(point, course, speed)

class GpsData(NamedTuple):
    lat: Optional[float] = None
    lon: Optional[float] = None
    course_rad: Optional[float] = None
    speed_mps: Optional[float] = None
    ts: Optional[float] = None
    pos_health: bool = False
    motion_health: bool = False