"""Guidance logic (L1-style + robust yaw-rate control)."""

from __future__ import annotations

import time
from math import atan2, cos, radians, sqrt, tanh
from types import SimpleNamespace


# Guidance parameters
L_DISTANCE_HIGH = 40.0
L_DISTANCE_MID = 30.0
L_DISTANCE_LOW = 15.0

YR_MAX = 45.0
LANDING_YR_MAX = 20.0
CAPTURE_THRESHOLD_DEG = 45.0

# PI control parameters
KP_YR = 1.2
KI_YR = 0.25
MAX_INTEGRAL = 10.0
MAX_ACCEL = 150.0  # deg/s^2

# GPS integrity
GPS_JUMP_MAX_SPEED = 200.0
GPS_STABLE_COUNT_REQUIRED = 2


_last_gps: SimpleNamespace | None = None
_gps_stable_count = 0
_integral_yr = 0.0
_last_cmd_yr = 0.0
_last_ctrl_ts: float | None = None
_start_point = SimpleNamespace(lat=0.0, lon=0.0)
_target_point = SimpleNamespace(lat=0.0, lon=0.0)


def init_guidance(_logger=None) -> None:
    global _last_gps, _gps_stable_count, _integral_yr, _last_cmd_yr, _last_ctrl_ts
    _last_gps = None
    _gps_stable_count = 0
    _integral_yr = 0.0
    _last_cmd_yr = 0.0
    _last_ctrl_ts = None


def is_gps_valid(gps_vector, gps_fidelity) -> bool:
    lat = float(gps_vector.lat)
    lon = float(gps_vector.lon)
    fix = int(gps_fidelity.fix_quality)
    sats = int(gps_fidelity.sats)
    status = str(gps_fidelity.rmc_status).upper()
    return -90 <= lat <= 90 and -180 <= lon <= 180 and fix >= 1 and sats >= 4 and status == "A"


def _distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    dlat = (lat2 - lat1) * 111_000.0
    dlon = (lon2 - lon1) * 111_000.0 * cos(radians((lat1 + lat2) / 2.0))
    return sqrt(dlat * dlat + dlon * dlon)


def is_gps_jump(lat: float, lon: float) -> bool:
    """Return True when position jump implies unrealistic speed."""
    global _last_gps, _gps_stable_count
    now = time.time()
    cur = SimpleNamespace(lat=float(lat), lon=float(lon), ts=now)

    if _last_gps is None:
        _last_gps = cur
        _gps_stable_count = 1
        return True

    dt = max(1e-3, cur.ts - _last_gps.ts)
    dist = _distance_m(_last_gps.lat, _last_gps.lon, cur.lat, cur.lon)
    speed = dist / dt
    _last_gps = cur

    if speed > GPS_JUMP_MAX_SPEED:
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


def _angle_wrap_deg(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def _outer_loop(angle_to_turn_deg: float, speed_mps: float, lookahead_m: float) -> float:
    """Nonlinear outer-loop to desired yaw-rate."""
    gain = max(0.5, min(2.0, speed_mps / max(5.0, lookahead_m)))
    scaled = angle_to_turn_deg / 45.0
    return YR_MAX * tanh(gain * scaled)


def _yaw_rate_pi_control(desired_yr: float, measured_yr: float, dt: float, yr_limit: float) -> float:
    global _integral_yr
    err = desired_yr - measured_yr
    p = KP_YR * err
    u_unsat = p + _integral_yr
    u_sat = max(-yr_limit, min(yr_limit, u_unsat))

    # Conditional integration (anti-windup)
    if abs(u_unsat) < yr_limit or (u_unsat > yr_limit and err < 0) or (u_unsat < -yr_limit and err > 0):
        _integral_yr += KI_YR * err * dt
        _integral_yr = max(-MAX_INTEGRAL, min(MAX_INTEGRAL, _integral_yr))
        u_sat = max(-yr_limit, min(yr_limit, p + _integral_yr))

    return u_sat


def _slew_limit(cmd: float, dt: float, accel_limit: float = MAX_ACCEL) -> float:
    global _last_cmd_yr
    delta_max = accel_limit * dt
    delta = cmd - _last_cmd_yr
    if delta > delta_max:
        cmd = _last_cmd_yr + delta_max
    elif delta < -delta_max:
        cmd = _last_cmd_yr - delta_max
    _last_cmd_yr = cmd
    return cmd


def guidance(imu_data, gps_vector, gps_fidelity, target, baro_m: float):
    if not is_gps_valid(gps_vector, gps_fidelity):
        return SimpleNamespace(state="FDIR", distance=0.0, commanded_yaw_rate=0.0)
    if is_gps_jump(gps_vector.lat, gps_vector.lon):
        return SimpleNamespace(state="FDIR", distance=0.0, commanded_yaw_rate=0.0)

    distance = _distance_m(gps_vector.lat, gps_vector.lon, target.lat, target.lon)
    dlat = (target.lat - gps_vector.lat) * 111_000.0
    dlon = (target.lon - gps_vector.lon) * 111_000.0 * cos(radians((target.lat + gps_vector.lat) / 2.0))
    desired_heading_deg = atan2(dlon, dlat) * 180.0 / 3.141592653589793
    my_heading_deg = float(imu_data.yaw)
    angle_to_turn_deg = _angle_wrap_deg(desired_heading_deg - my_heading_deg)

    if baro_m > 300:
        lookahead = L_DISTANCE_HIGH
    elif baro_m > 150:
        lookahead = L_DISTANCE_MID
    else:
        lookahead = L_DISTANCE_LOW

    if baro_m > 50:
        phase = "HOMING"
    elif baro_m > 10:
        phase = "PATTERN" if distance < 80 else "HOMING"
    else:
        phase = "LANDING"

    if abs(angle_to_turn_deg) > CAPTURE_THRESHOLD_DEG:
        global _integral_yr
        _integral_yr = 0.0

    speed = max(3.0, float(gps_vector.speed))
    desired_yr = _outer_loop(angle_to_turn_deg, speed, lookahead)
    yr_limit = LANDING_YR_MAX if baro_m <= 20 else YR_MAX

    global _last_ctrl_ts
    now = time.time()
    if _last_ctrl_ts is None:
        dt = 0.1
    else:
        dt = max(0.01, min(0.3, now - _last_ctrl_ts))
    _last_ctrl_ts = now

    cmd_yr = _yaw_rate_pi_control(desired_yr, float(imu_data.gyrz), dt, yr_limit)
    cmd_yr = _slew_limit(cmd_yr, dt)
    cmd_yr = max(-yr_limit, min(yr_limit, cmd_yr))

    return SimpleNamespace(state=phase, distance=distance, commanded_yaw_rate=cmd_yr)
