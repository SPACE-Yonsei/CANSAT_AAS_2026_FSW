"""Guidance logic: L1-style carrot-following + robust yaw-rate PI control.

Unit contract:
  - All angles          : degrees
  - gyrz (measured yr)  : deg/s  (must match IMU driver output; see motorapp.handle_imu)
  - commanded_yaw_rate  : deg/s
  - distance            : metres
  - altitude (baro_m)   : metres AGL
  - speed               : m/s

Phases:
  HOMING   — altitude > 50 m, fly toward target
  PATTERN  — 10 m < alt <= 50 m and within 80 m of target; fly figure-8 / loiter
  LANDING  — alt <= 10 m; reduced yaw-rate limit
"""

from __future__ import annotations

import time
from math import atan2, cos, radians, sqrt, tanh
from types import SimpleNamespace
from typing import NamedTuple


class GpsVector(NamedTuple):
    """Position and velocity from GPS, always passed as this type."""
    lat:    float   # decimal degrees, WGS-84
    lon:    float   # decimal degrees, WGS-84
    speed:  float   # m/s (ground speed)
    course: float   # degrees (track over ground)


class GpsFidelity(NamedTuple):
    """GPS quality flags, always passed as this type."""
    fix_quality: int    # 0=no fix, 1=GPS, 2=DGPS, …
    sats:        int    # number of satellites in use
    rmc_status:  str    # 'A'=active/valid, 'V'=void/invalid
    gps_health:  int    # driver-level health flag (0=unhealthy)

# ---------------------------------------------------------------------------
# Guidance tuning constants
# ---------------------------------------------------------------------------

# Lookahead distances (m) selected by altitude band
L_DISTANCE_HIGH: float = 40.0
L_DISTANCE_MID:  float = 30.0
L_DISTANCE_LOW:  float = 15.0

# Yaw-rate limits (deg/s)
YR_MAX:         float = 45.0
LANDING_YR_MAX: float = 20.0

# Heading error at which the PI integrator is reset (deg)
CAPTURE_THRESHOLD_DEG: float = 45.0

# PI controller gains
KP_YR:        float = 1.2
KI_YR:        float = 0.25
MAX_INTEGRAL: float = 10.0   # deg/s  (anti-windup clamp on integrator)

# Slew-rate limit for output yaw-rate (deg/s per second)
MAX_ACCEL: float = 150.0

# GPS integrity thresholds
GPS_JUMP_MAX_SPEED:          float = 200.0  # m/s — above this we call it a jump
GPS_STABLE_COUNT_REQUIRED:   int   = 2      # consecutive stable readings before trusting GPS

# ---------------------------------------------------------------------------
# Module-level state (reset by init_guidance)
# ---------------------------------------------------------------------------

_last_gps: SimpleNamespace | None = None
_gps_stable_count:  int   = 0
_integral_yr:       float = 0.0
_last_cmd_yr:       float = 0.0
_last_ctrl_ts:      float | None = None
_start_point = SimpleNamespace(lat=0.0, lon=0.0)
_target_point = SimpleNamespace(lat=0.0, lon=0.0)


# ---------------------------------------------------------------------------
# Lifecycle
# ---------------------------------------------------------------------------

def init_guidance(_logger=None) -> None:
    """Reset all guidance state. Call once at app startup and on re-init."""
    global _last_gps, _gps_stable_count, _integral_yr, _last_cmd_yr, _last_ctrl_ts
    _last_gps         = None
    _gps_stable_count = 0
    _integral_yr      = 0.0
    _last_cmd_yr      = 0.0
    _last_ctrl_ts     = None


# ---------------------------------------------------------------------------
# GPS integrity checks
# ---------------------------------------------------------------------------

def is_gps_valid(gps_vector: GpsVector, gps_fidelity: GpsFidelity) -> bool:
    """Return True when GPS data passes basic sanity checks."""
    return (
        -90.0 <= gps_vector.lat <= 90.0
        and -180.0 <= gps_vector.lon <= 180.0
        and gps_fidelity.fix_quality >= 1
        and gps_fidelity.sats >= 4
        and gps_fidelity.rmc_status.upper() == "A"
        and gps_fidelity.gps_health >= 1
    )


def _distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Flat-earth distance in metres between two WGS-84 coordinates."""
    dlat = (lat2 - lat1) * 111_000.0
    dlon = (lon2 - lon1) * 111_000.0 * cos(radians((lat1 + lat2) / 2.0))
    return sqrt(dlat * dlat + dlon * dlon)


def is_gps_jump(lat: float, lon: float) -> bool:
    """Return True when the implied ground speed exceeds GPS_JUMP_MAX_SPEED m/s.

    Returns True (i.e. 'jump detected') for the very first call and until
    GPS_STABLE_COUNT_REQUIRED consecutive non-jump readings accumulate.
    """
    global _last_gps, _gps_stable_count
    now = time.time()
    cur = SimpleNamespace(lat=float(lat), lon=float(lon), ts=now)

    if _last_gps is None:
        _last_gps = cur
        _gps_stable_count = 1
        return True  # first reading: wait for confirmation

    dt   = max(1e-3, cur.ts - _last_gps.ts)
    dist = _distance_m(_last_gps.lat, _last_gps.lon, cur.lat, cur.lon)
    speed_mps = dist / dt
    _last_gps = cur

    if speed_mps > GPS_JUMP_MAX_SPEED:
        _gps_stable_count = 0
        return True

    _gps_stable_count += 1
    return _gps_stable_count < GPS_STABLE_COUNT_REQUIRED


# ---------------------------------------------------------------------------
# Coordinate setters
# ---------------------------------------------------------------------------

def set_start_coordinates(lat: float, lon: float) -> None:
    """Lock the start point (called by motorapp on state-3 entry)."""
    _start_point.lat = float(lat)
    _start_point.lon = float(lon)


def set_target_coord(lat: float, lon: float) -> None:
    """Update the target landing coordinate (called by motorapp on TC message)."""
    _target_point.lat = float(lat)
    _target_point.lon = float(lon)


# ---------------------------------------------------------------------------
# Control sub-functions
# ---------------------------------------------------------------------------

def _angle_wrap_deg(angle: float) -> float:
    """Wrap angle into (-180, +180]."""
    while angle >  180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def _outer_loop(angle_to_turn_deg: float, speed_mps: float, lookahead_m: float) -> float:
    """Nonlinear outer-loop: heading error -> desired yaw-rate (deg/s).

    Uses tanh saturation so the output stays within ±YR_MAX.
    The gain scales with speed/lookahead, giving faster response at low altitude.
    """
    gain   = max(0.5, min(2.0, speed_mps / max(5.0, lookahead_m)))
    scaled = angle_to_turn_deg / 45.0
    return YR_MAX * tanh(gain * scaled)


def _yaw_rate_pi_control(
    desired_yr: float,
    measured_yr: float,   # deg/s (same unit as desired_yr)
    dt: float,
    yr_limit: float,
) -> float:
    """PI controller with conditional-integration anti-windup.

    Both desired_yr and measured_yr must be in deg/s.
    """
    global _integral_yr
    err      = desired_yr - measured_yr
    p        = KP_YR * err
    u_unsat  = p + _integral_yr
    u_sat    = max(-yr_limit, min(yr_limit, u_unsat))

    # Integrate only when not saturated or when error reduces saturation
    if (
        abs(u_unsat) < yr_limit
        or (u_unsat >  yr_limit and err < 0)
        or (u_unsat < -yr_limit and err > 0)
    ):
        _integral_yr += KI_YR * err * dt
        _integral_yr  = max(-MAX_INTEGRAL, min(MAX_INTEGRAL, _integral_yr))
        u_sat = max(-yr_limit, min(yr_limit, p + _integral_yr))

    return u_sat


def _slew_limit(cmd: float, dt: float, accel_limit: float = MAX_ACCEL) -> float:
    """Rate-limit the yaw-rate command to prevent actuator shock."""
    global _last_cmd_yr
    delta_max = accel_limit * dt
    delta     = cmd - _last_cmd_yr
    if   delta >  delta_max:
        cmd = _last_cmd_yr + delta_max
    elif delta < -delta_max:
        cmd = _last_cmd_yr - delta_max
    _last_cmd_yr = cmd
    return cmd


# ---------------------------------------------------------------------------
# Main guidance entry point
# ---------------------------------------------------------------------------

def guidance(imu_data, gps_vector: GpsVector, gps_fidelity: GpsFidelity, target, baro_m: float) -> SimpleNamespace:
    """Compute commanded yaw-rate for the current timestep.

    Args:
        imu_data     : SimpleNamespace with .yaw (deg) and .gyrz (deg/s)
        gps_vector   : SimpleNamespace with .lat, .lon, .speed (m/s), .course (deg)
        gps_fidelity : SimpleNamespace with .fix_quality, .sats, .rmc_status
        target       : SimpleNamespace with .lat, .lon  (must not be None here;
                       motorapp FDIR-7 blocks calls when target is None)
        baro_m       : altitude AGL in metres

    Returns:
        SimpleNamespace with:
          .state             : 'HOMING' | 'PATTERN' | 'LANDING' | 'FDIR'
          .distance          : metres to target (0.0 on FDIR)
          .commanded_yaw_rate: deg/s  (0.0 on FDIR)
    """
    _FDIR = SimpleNamespace(state="FDIR", distance=0.0, commanded_yaw_rate=0.0)

    if not is_gps_valid(gps_vector, gps_fidelity):
        return _FDIR
    if is_gps_jump(gps_vector.lat, gps_vector.lon):
        return _FDIR

    distance      = _distance_m(gps_vector.lat, gps_vector.lon, target.lat, target.lon)
    dlat          = (target.lat - gps_vector.lat) * 111_000.0
    mid_lat       = (target.lat + gps_vector.lat) / 2.0
    dlon          = (target.lon - gps_vector.lon) * 111_000.0 * cos(radians(mid_lat))
    desired_hdg   = atan2(dlon, dlat) * 180.0 / 3.141592653589793
    angle_to_turn = _angle_wrap_deg(desired_hdg - float(imu_data.yaw))

    # Altitude-based lookahead selection
    if   baro_m > 300:
        lookahead = L_DISTANCE_HIGH
    elif baro_m > 150:
        lookahead = L_DISTANCE_MID
    else:
        lookahead = L_DISTANCE_LOW

    # Phase selection
    if   baro_m > 50:
        phase    = "HOMING"
    elif baro_m > 10:
        phase    = "PATTERN" if distance < 80 else "HOMING"
    else:
        phase    = "LANDING"

    # Reset integrator on large heading errors (prevents wind-up during large turns)
    if abs(angle_to_turn) > CAPTURE_THRESHOLD_DEG:
        global _integral_yr
        _integral_yr = 0.0

    speed      = max(3.0, float(gps_vector.speed))
    desired_yr = _outer_loop(angle_to_turn, speed, lookahead)
    yr_limit   = LANDING_YR_MAX if baro_m <= 20 else YR_MAX

    global _last_ctrl_ts
    now = time.time()
    dt  = 0.1 if _last_ctrl_ts is None else max(0.01, min(0.3, now - _last_ctrl_ts))
    _last_ctrl_ts = now

    # measured_yr = imu_data.gyrz in deg/s (same unit as desired_yr)
    cmd_yr = _yaw_rate_pi_control(desired_yr, float(imu_data.gyrz), dt, yr_limit)
    cmd_yr = _slew_limit(cmd_yr, dt)
    cmd_yr = max(-yr_limit, min(yr_limit, cmd_yr))

    return SimpleNamespace(state=phase, distance=distance, commanded_yaw_rate=cmd_yr)
