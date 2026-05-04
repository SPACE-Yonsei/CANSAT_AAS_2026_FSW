"""Parafoil guidance: start-target carrot tracking plus L1-style yaw-rate demand.

Unit contract:
  - yaw, heading, course        : deg
  - gyrz, desired/commanded yr  : deg/s
  - distance, cross-track error : m
  - altitude                    : m AGL

This module intentionally keeps the actuator model out of guidance. It outputs
only commanded_yaw_rate in deg/s. Motor effectiveness is tuned in motor_control.
"""

from __future__ import annotations

import math
import time
from types import SimpleNamespace
from typing import NamedTuple


class GpsVector(NamedTuple):
    """Position and ground track from GPS."""
    lat: float
    lon: float
    speed: float
    course: float


class GpsFidelity(NamedTuple):
    """GPS quality flags from the GPS app."""
    fix_quality: int
    sats: int
    rmc_status: str
    gps_health: int


# ---------------------------------------------------------------------------
# Guidance tuning constants
# ---------------------------------------------------------------------------

# ArduPilot L1-inspired path-following knobs. This is not copied code; these
# are the same control concepts adapted to a low-speed parafoil with yaw-rate
# output instead of lateral acceleration output.
L1_PERIOD_SEC: float = 17.0
L1_DAMPING: float = 0.75
L1_MIN_GROUND_SPEED_MPS: float = 3.0
L1_CAPTURE_ANGLE_DEG: float = 90.0

# Lookahead bounds and altitude floors (m).
L_DISTANCE_HIGH: float = 40.0
L_DISTANCE_MID: float = 30.0
L_DISTANCE_LOW: float = 15.0
LOOKAHEAD_MIN_M: float = 10.0
LOOKAHEAD_MAX_M: float = 45.0

# Phase thresholds.
PATTERN_ALTITUDE_M: float = 50.0
LANDING_ALTITUDE_M: float = 10.0
PATTERN_RADIUS_M: float = 80.0

# Yaw-rate limits (deg/s).
YR_MAX: float = 45.0
PATTERN_YR_MAX: float = 35.0
LANDING_YR_MAX: float = 20.0

# Heading error at which the PI integrator is reset (deg).
CAPTURE_THRESHOLD_DEG: float = 45.0

# Inner yaw-rate PI gains.
KP_YR: float = 1.2
KI_YR: float = 0.25
MAX_INTEGRAL: float = 10.0

# Command slew-rate limit (deg/s per second).
MAX_ACCEL: float = 150.0

# GPS integrity thresholds.
GPS_JUMP_MAX_SPEED: float = 200.0
GPS_STABLE_COUNT_REQUIRED: int = 2

EARTH_M_PER_DEG: float = 111_000.0


# ---------------------------------------------------------------------------
# Module-level state
# ---------------------------------------------------------------------------

_last_gps: SimpleNamespace | None = None
_gps_stable_count: int = 0
_integral_yr: float = 0.0
_last_cmd_yr: float = 0.0
_last_ctrl_ts: float | None = None
_start_point = SimpleNamespace(lat=0.0, lon=0.0)
_target_point = SimpleNamespace(lat=0.0, lon=0.0)


def init_guidance(_logger=None) -> None:
    """Reset all runtime state. Call at app startup and before replay tests."""
    global _last_gps, _gps_stable_count, _integral_yr, _last_cmd_yr, _last_ctrl_ts
    _last_gps = None
    _gps_stable_count = 0
    _integral_yr = 0.0
    _last_cmd_yr = 0.0
    _last_ctrl_ts = None


# ---------------------------------------------------------------------------
# GPS integrity and coordinates
# ---------------------------------------------------------------------------

def is_gps_valid(gps_vector: GpsVector, gps_fidelity: GpsFidelity) -> bool:
    """Return True when GPS data passes basic quality checks."""
    return (
        math.isfinite(float(gps_vector.lat))
        and math.isfinite(float(gps_vector.lon))
        and -90.0 <= float(gps_vector.lat) <= 90.0
        and -180.0 <= float(gps_vector.lon) <= 180.0
        and abs(float(gps_vector.lon)) > 1e-9
        and int(gps_fidelity.fix_quality) >= 1
        and int(gps_fidelity.sats) >= 4
        and str(gps_fidelity.rmc_status).upper() == "A"
        and int(gps_fidelity.gps_health) >= 1
    )


def _gps_to_ne(ref_lat: float, ref_lon: float, lat: float, lon: float) -> SimpleNamespace:
    """Convert WGS-84 lat/lon to local North/East metres around ref."""
    north = (lat - ref_lat) * EARTH_M_PER_DEG
    east = (lon - ref_lon) * EARTH_M_PER_DEG * math.cos(math.radians(ref_lat))
    return SimpleNamespace(n=north, e=east)


def _distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    ne = _gps_to_ne(lat1, lon1, lat2, lon2)
    return math.hypot(ne.n, ne.e)


def is_gps_jump(lat: float, lon: float) -> bool:
    """Reject first/unstable samples and unrealistic implied GPS velocity."""
    global _last_gps, _gps_stable_count
    now = time.time()
    cur = SimpleNamespace(lat=float(lat), lon=float(lon), ts=now)

    if _last_gps is None:
        _last_gps = cur
        _gps_stable_count = 1
        return True

    dt = max(1e-3, cur.ts - _last_gps.ts)
    dist = _distance_m(_last_gps.lat, _last_gps.lon, cur.lat, cur.lon)
    speed_mps = dist / dt
    _last_gps = cur

    if speed_mps > GPS_JUMP_MAX_SPEED:
        _gps_stable_count = 0
        return True

    _gps_stable_count += 1
    return _gps_stable_count < GPS_STABLE_COUNT_REQUIRED


def set_start_coordinates(lat: float, lon: float) -> None:
    _start_point.lat = float(lat)
    _start_point.lon = float(lon)


def set_target_coord(lat: float, lon: float) -> None:
    _target_point.lat = float(lat)
    _target_point.lon = float(lon)


# ---------------------------------------------------------------------------
# Guidance helpers
# ---------------------------------------------------------------------------

def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _angle_wrap_deg(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return angle


def _bearing_deg(from_ne: SimpleNamespace, to_ne: SimpleNamespace) -> float:
    """Return navigation bearing: 0=N, +90=E."""
    return math.degrees(math.atan2(to_ne.e - from_ne.e, to_ne.n - from_ne.n))


def _phase(distance_m: float, alt: float) -> str:
    if alt <= LANDING_ALTITUDE_M:
        return "LANDING"
    if alt <= PATTERN_ALTITUDE_M and distance_m <= PATTERN_RADIUS_M:
        return "PATTERN"
    return "HOMING"


def _phase_yaw_limit(phase: str) -> float:
    if phase == "LANDING":
        return LANDING_YR_MAX
    if phase == "PATTERN":
        return PATTERN_YR_MAX
    return YR_MAX


def schedule_lookahead(alt: float, gps_speed_mps: float) -> float:
    """Schedule L1/carrot lookahead by altitude and ground speed.

    ArduPilot L1 uses an L1 distance proportional to damping * period * speed.
    For this parafoil we bound that distance and keep altitude-dependent floors,
    because low-altitude operation needs less aggressive lateral movement.
    """
    speed = max(L1_MIN_GROUND_SPEED_MPS, float(gps_speed_mps))
    l1_distance = (L1_DAMPING * L1_PERIOD_SEC / math.pi) * speed

    if alt > 300.0:
        altitude_floor = L_DISTANCE_HIGH
    elif alt > 150.0:
        altitude_floor = L_DISTANCE_MID
    else:
        altitude_floor = L_DISTANCE_LOW

    return _clamp(max(l1_distance, altitude_floor), LOOKAHEAD_MIN_M, LOOKAHEAD_MAX_M)


def _l1_desired_yaw_rate(heading_error_deg: float, speed_mps: float, lookahead_m: float, yr_limit: float) -> float:
    """Convert heading error to desired yaw rate using an L1-style demand.

    L1 normally commands lateral acceleration: K_L1 * V^2 / L1_dist * sin(Nu).
    With no airspeed and no full aircraft dynamics model, we approximate the
    yaw-rate demand as lateral_accel / ground_speed.
    """
    speed = max(L1_MIN_GROUND_SPEED_MPS, float(speed_mps))
    nu_deg = _clamp(heading_error_deg, -L1_CAPTURE_ANGLE_DEG, L1_CAPTURE_ANGLE_DEG)
    k_l1 = 4.0 * L1_DAMPING * L1_DAMPING
    lateral_accel = k_l1 * speed * speed / max(lookahead_m, 1.0) * math.sin(math.radians(nu_deg))
    desired_yr = math.degrees(lateral_accel / speed)
    return _clamp(desired_yr, -yr_limit, yr_limit)


def _yaw_rate_pi_control(desired_yr: float, measured_yr: float, dt: float, yr_limit: float) -> float:
    """Cascade inner-loop PI with conditional anti-windup."""
    global _integral_yr
    err = desired_yr - measured_yr
    p_term = KP_YR * err
    u_unsat = p_term + _integral_yr
    u_sat = _clamp(u_unsat, -yr_limit, yr_limit)

    if (
        abs(u_unsat) < yr_limit
        or (u_unsat > yr_limit and err < 0.0)
        or (u_unsat < -yr_limit and err > 0.0)
    ):
        _integral_yr = _clamp(_integral_yr + KI_YR * err * dt, -MAX_INTEGRAL, MAX_INTEGRAL)
        u_sat = _clamp(p_term + _integral_yr, -yr_limit, yr_limit)

    return u_sat


def _slew_limit(cmd: float, dt: float, accel_limit: float = MAX_ACCEL) -> float:
    global _last_cmd_yr
    delta_max = accel_limit * dt
    cmd = _clamp(cmd, _last_cmd_yr - delta_max, _last_cmd_yr + delta_max)
    _last_cmd_yr = cmd
    return cmd


def _fdir_result(reason: str) -> SimpleNamespace:
    return SimpleNamespace(
        state="FDIR",
        phase="FDIR",
        fdir_reason=reason,
        distance=0.0,
        crosstrack_error=math.nan,
        along_track=math.nan,
        track_length=math.nan,
        lookahead=math.nan,
        desired_heading=math.nan,
        heading_error=math.nan,
        desired_yaw_rate=0.0,
        commanded_yaw_rate=0.0,
        measured_yaw_rate=0.0,
        gps_course=math.nan,
        course_error=math.nan,
        carrot_e=math.nan,
        carrot_n=math.nan,
        yr_limit=0.0,
    )


def guidance(imu_data, gps_vector: GpsVector, gps_fidelity: GpsFidelity, target, alt: float) -> SimpleNamespace:
    """Compute commanded yaw-rate for one timestep.

    Returns a SimpleNamespace with the legacy fields (.state, .distance,
    .commanded_yaw_rate) plus replay/debug fields used by MotorApp telemetry.
    """
    if target is None:
        return _fdir_result("target missing")
    if not is_gps_valid(gps_vector, gps_fidelity):
        return _fdir_result("gps invalid")
    if is_gps_jump(gps_vector.lat, gps_vector.lon):
        return _fdir_result("gps jump")

    start = _start_point
    cur_ne = _gps_to_ne(start.lat, start.lon, float(gps_vector.lat), float(gps_vector.lon))
    target_ne = _gps_to_ne(start.lat, start.lon, float(target.lat), float(target.lon))
    start_ne = SimpleNamespace(n=0.0, e=0.0)
    track_len = max(1e-6, math.hypot(target_ne.n, target_ne.e))
    track_n = target_ne.n / track_len
    track_e = target_ne.e / track_len

    along_track = cur_ne.n * track_n + cur_ne.e * track_e
    # Positive means current position is to the left of the start->target line.
    crosstrack_error = track_e * cur_ne.n - track_n * cur_ne.e
    distance = math.hypot(target_ne.n - cur_ne.n, target_ne.e - cur_ne.e)

    phase = _phase(distance, float(alt))
    yr_limit = _phase_yaw_limit(phase)
    lookahead = schedule_lookahead(float(alt), float(gps_vector.speed))

    carrot_along = _clamp(along_track + lookahead, 0.0, track_len)
    carrot_ne = SimpleNamespace(n=carrot_along * track_n, e=carrot_along * track_e)

    desired_heading = _bearing_deg(cur_ne, carrot_ne)
    heading_error = _angle_wrap_deg(desired_heading - float(imu_data.yaw))
    track_heading = _bearing_deg(start_ne, target_ne)
    course_error = _angle_wrap_deg(track_heading - float(gps_vector.course))

    global _integral_yr
    if abs(heading_error) > CAPTURE_THRESHOLD_DEG:
        _integral_yr = 0.0

    speed = max(L1_MIN_GROUND_SPEED_MPS, float(gps_vector.speed))
    desired_yr = _l1_desired_yaw_rate(heading_error, speed, lookahead, yr_limit)

    global _last_ctrl_ts
    now = time.time()
    dt = 0.1 if _last_ctrl_ts is None else _clamp(now - _last_ctrl_ts, 0.01, 0.3)
    _last_ctrl_ts = now

    measured_yr = float(imu_data.gyrz)
    cmd_yr = _yaw_rate_pi_control(desired_yr, measured_yr, dt, yr_limit)
    cmd_yr = _slew_limit(cmd_yr, dt)
    cmd_yr = _clamp(cmd_yr, -yr_limit, yr_limit)

    return SimpleNamespace(
        state=phase,
        phase=phase,
        fdir_reason="",
        distance=distance,
        crosstrack_error=crosstrack_error,
        along_track=along_track,
        track_length=track_len,
        lookahead=lookahead,
        desired_heading=desired_heading,
        heading_error=heading_error,
        desired_yaw_rate=desired_yr,
        commanded_yaw_rate=cmd_yr,
        measured_yaw_rate=measured_yr,
        gps_course=float(gps_vector.course),
        course_error=course_error,
        carrot_e=carrot_ne.e,
        carrot_n=carrot_ne.n,
        yr_limit=yr_limit,
    )
