"""Navigation state, control-mode selection, and L1 guidance for parafoil control.

Units:
  distance=m, time=s, angle=rad, speed=m/s.
  Local frame is NE: +N north, +E east. course=0 means north, +pi/2 east.
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from enum import Enum
from math import isfinite, nan, pi

from lib import config
from .sensor_types import BaroAnchor, GpsAnchor, ImuAnchor

logger = logging.getLogger(__name__)

EARTH_RADIUS_M = 6_371_000.0


def _wrap_pi(angle: float) -> float:
    return (angle + pi) % (2.0 * pi) - pi


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _ok(v) -> bool:
    try:
        return isfinite(float(v))
    except (TypeError, ValueError):
        return False


def _circular_mean(a: float, b: float) -> float:
    return math.atan2(math.sin(a) + math.sin(b), math.cos(a) + math.cos(b))


def latlon_to_ne(lat: float, lon: float,
                 origin_lat: float, origin_lon: float) -> tuple[float, float]:
    dLat = math.radians(lat - origin_lat)
    dLon = math.radians(lon - origin_lon)
    N = dLat * EARTH_RADIUS_M
    E = dLon * EARTH_RADIUS_M * math.cos(math.radians(origin_lat))
    return N, E


class ControlMode(str, Enum):
    GPS_TRACKING_CLOSED = "GPS_TRACKING_CLOSED"
    GPS_TRACKING_OPEN = "GPS_TRACKING_OPEN"

    # GPS position fresh, GPS motion stale: estimate M only.
    DR_M_GBA_CLOSED = "DR_M_GBA_CLOSED"
    DR_M_GB_CLOSED = "DR_M_GB_CLOSED"
    DR_M_G_CLOSED = "DR_M_G_CLOSED"
    DR_M_YBA_OPEN = "DR_M_YBA_OPEN"
    DR_M_YB_OPEN = "DR_M_YB_OPEN"
    DR_M_Y_OPEN = "DR_M_Y_OPEN"

    # GPS position stale: estimate P and M from DR current state.
    DR_PM_GBA_CLOSED = "DR_PM_GBA_CLOSED"
    DR_PM_GB_CLOSED = "DR_PM_GB_CLOSED"
    DR_PM_G_CLOSED = "DR_PM_G_CLOSED"
    DR_PM_YBA_OPEN = "DR_PM_YBA_OPEN"
    DR_PM_YB_OPEN = "DR_PM_YB_OPEN"
    DR_PM_Y_OPEN = "DR_PM_Y_OPEN"

    DETUMBLING = "DETUMBLING"
    FAIL = "FAIL"


class DRMethod(str, Enum):
    NONE = "NONE"
    GYRO_INTEGRATION = "GYRO_INTEGRATION"
    YAW_DELTA = "YAW_DELTA"
    GYRO_ACC_BLEND = "GYRO_ACC_BLEND"
    YAW_ACC_BLEND = "YAW_ACC_BLEND"


@dataclass
class MissionFrame:
    origin_lat: float = nan
    origin_lon: float = nan
    origin_ready: bool = False
    target_E: float = nan
    target_N: float = nan
    target_ready: bool = False
    _target_lat: float = nan
    _target_lon: float = nan


@dataclass
class DRState:
    anchor_E: float = nan
    anchor_N: float = nan
    anchor_V: float = nan
    anchor_course: float = nan
    anchor_time: float = nan

    current_E: float = nan
    current_N: float = nan
    current_V: float = nan
    current_course: float = nan
    current_time: float = nan

    yaw_at_anchor: float = nan
    gyro_integral: float = 0.0
    last_step_time: float = nan

    method: DRMethod = DRMethod.NONE
    confidence: float = 0.0


@dataclass
class NavState:
    E: float = nan
    N: float = nan
    course: float = nan
    V: float = nan
    vE: float = nan
    vN: float = nan
    confidence: float = 0.0
    valid: bool = False
    timestamp: float = nan
    control_mode: ControlMode = ControlMode.FAIL


@dataclass
class SensorFreshFlags:
    gps_pos_fresh: bool = False
    gps_motion_fresh: bool = False
    imu_yaw_fresh: bool = False
    imu_gyrz_fresh: bool = False
    baro_sink_fresh: bool = False
    acc_fresh: bool = False

    gps_pos_stale: bool = True
    gps_motion_stale: bool = True
    imu_yaw_stale: bool = True
    imu_gyrz_stale: bool = True
    baro_sink_stale: bool = True
    acc_stale: bool = True

    dr_anchor_valid: bool = False
    dr_current_valid: bool = False
    nav_valid: bool = False


@dataclass
class GuidanceState:
    gps: GpsAnchor = field(default_factory=GpsAnchor)
    imu: ImuAnchor = field(default_factory=ImuAnchor)
    baro: BaroAnchor = field(default_factory=BaroAnchor)
    nav: NavState = field(default_factory=NavState)
    dr: DRState = field(default_factory=DRState)
    flags: SensorFreshFlags = field(default_factory=SensorFreshFlags)


@dataclass
class L1Input:
    valid: bool = False
    reason: str = "INIT"
    control_mode: ControlMode = ControlMode.FAIL
    dr_method: DRMethod = DRMethod.NONE
    confidence: float = 0.0
    E: float = nan
    N: float = nan
    V: float = nan
    course: float = nan
    vE: float = nan
    vN: float = nan
    target_E: float = nan
    target_N: float = nan


@dataclass
class L1Output:
    timestamp: float = 0.0
    control_valid: bool = False
    nominal: bool = False
    reason: str = "INIT"
    control_mode: ControlMode = ControlMode.FAIL
    dr_method: DRMethod = DRMethod.NONE
    confidence: float = 0.0
    yaw_rate_cmd: float = 0.0          # rad/s (control.py converts to deg/s)
    yaw_rate_limit_dps: float = 0.0
    nu: float = nan
    target_bearing: float = nan
    distance_to_target: float = nan
    pos_E: float = nan
    pos_N: float = nan
    target_E: float = nan
    target_N: float = nan
    ground_speed_mps: float = 0.0
    pid_enabled: bool = False

    # Spec aliases / debug mirrors (kept alongside the legacy names above so
    # control.py and sensorlog continue to read the originals unchanged).
    valid: bool = False               # mirror of control_valid
    yaw_rate_limit: float = 0.0       # rad/s (same limit as yaw_rate_limit_dps)
    dist_to_target: float = nan       # mirror of distance_to_target
    nav_E: float = nan                # mirror of pos_E
    nav_N: float = nan                # mirror of pos_N
    V: float = 0.0                    # mirror of ground_speed_mps (L1Input.V)
    course: float = nan               # vehicle course (rad) from L1Input


_MISSION_t = MissionFrame()
_STATE_t = GuidanceState()


def dr_anchor_valid(dr: DRState) -> bool:
    return (
        isfinite(dr.anchor_E) and isfinite(dr.anchor_N)
        and isfinite(dr.anchor_V) and isfinite(dr.anchor_course)
        and isfinite(dr.anchor_time)
    )


def dr_current_valid(dr: DRState) -> bool:
    return (
        isfinite(dr.current_E) and isfinite(dr.current_N)
        and isfinite(dr.current_V) and isfinite(dr.current_course)
        and isfinite(dr.current_time)
    )


def dr_is_valid(dr: DRState) -> bool:
    return dr_anchor_valid(dr)


def last_v_valid(dr: DRState) -> bool:
    return isfinite(dr.current_V) or isfinite(dr.anchor_V)


def _last_v(dr: DRState) -> float:
    if isfinite(dr.current_V):
        return dr.current_V
    if isfinite(dr.anchor_V):
        return dr.anchor_V
    return nan


def dr_lock(dr: DRState, E: float, N: float, V: float,
            course: float, yaw: float, now: float) -> None:
    dr.anchor_E = E
    dr.anchor_N = N
    dr.anchor_V = V
    dr.anchor_course = course
    dr.anchor_time = now
    dr.current_E = E
    dr.current_N = N
    dr.current_V = V
    dr.current_course = course
    dr.current_time = now
    dr.yaw_at_anchor = yaw
    dr.gyro_integral = 0.0
    dr.last_step_time = now
    dr.method = DRMethod.NONE
    dr.confidence = 1.0


def _mode_value(mode) -> str:
    raw = getattr(mode, "value", mode)
    return str(raw)


def _is_gps_tracking_mode(mode) -> bool:
    return mode in (ControlMode.GPS_TRACKING_CLOSED, ControlMode.GPS_TRACKING_OPEN)


def _is_dr_mode(mode) -> bool:
    return _mode_value(mode).startswith("DR_")


def is_guidance_mode(mode) -> bool:
    return _is_gps_tracking_mode(mode) or _is_dr_mode(mode)


def _mode_estimates_P(mode) -> bool:
    return "_PM_" in _mode_value(mode)


def _mode_estimates_M(mode) -> bool:
    value = _mode_value(mode)
    return "_M_" in value or "_PM_" in value


def _dr_source_field(mode) -> str:
    if not _is_dr_mode(mode):
        return ""
    parts = _mode_value(mode).split("_")
    if len(parts) >= 4:
        return parts[2]
    return ""


def _mode_uses_gyro(mode) -> bool:
    return "G" in _dr_source_field(mode)


def _mode_uses_yaw(mode) -> bool:
    return "Y" in _dr_source_field(mode)


def _mode_uses_baro(mode) -> bool:
    return "B" in _dr_source_field(mode)


def _mode_uses_acc(mode) -> bool:
    return "A" in _dr_source_field(mode)


def _mode_uses_gyro_feedback(mode) -> bool:
    value = _mode_value(mode)
    return mode == ControlMode.GPS_TRACKING_CLOSED or (
        value.startswith("DR_") and value.endswith("_CLOSED")
    )


def _mode_estimates_position(mode) -> bool:
    return isinstance(mode, ControlMode) and _mode_value(mode).startswith("DR_PM_")


def _mode_estimates_motion(mode) -> bool:
    value = _mode_value(mode)
    return isinstance(mode, ControlMode) and (
        value.startswith("DR_M_") or value.startswith("DR_PM_")
    )


def _safe_dt(now: float, previous: float, max_dt: float = 0.5) -> float:
    if not isfinite(now) or not isfinite(previous):
        return 0.0
    dt = now - previous
    if dt < 0.0:
        return 0.0
    return min(dt, max_dt)


def dr_estimate_course(
    dr: DRState,
    imu: ImuAnchor,
    use_gyro: bool | None = None,
    use_yaw: bool | None = None,
) -> float:
    base = dr.anchor_course
    if not isfinite(base):
        return nan
    if use_gyro is None:
        use_gyro = bool(imu.gyrz_valid)
    if use_yaw is None:
        use_yaw = bool(imu.yaw_valid)

    course_gyro = _wrap_pi(base + dr.gyro_integral) if use_gyro else None
    course_yaw = (
        _wrap_pi(base + _wrap_pi(imu.yaw - dr.yaw_at_anchor))
        if (use_yaw and isfinite(imu.yaw) and isfinite(dr.yaw_at_anchor)) else None
    )
    if course_gyro is not None and course_yaw is not None:
        limit = math.radians(getattr(config, "YAW_GYRO_BLEND_MAX_DEG", 45.0))
        if abs(_wrap_pi(course_yaw - course_gyro)) < limit:
            return _circular_mean(course_yaw, course_gyro)
        return course_gyro
    if course_gyro is not None:
        return course_gyro
    if course_yaw is not None:
        return course_yaw
    return base


def _compute_dr_confidence(age: float) -> float:
    if not isfinite(age) or age < 0.0:
        return 1.0
    a1 = config.DR_CONF_AGE_1_S
    a2 = config.DR_CONF_AGE_2_S
    a3 = config.DR_CONF_AGE_3_S
    if age <= a1:
        conf = 1.0
    elif age <= a2:
        conf = 1.0 - 0.5 * (age - a1) / max(a2 - a1, 1e-6)
    elif age <= a3:
        conf = 0.5 * (1.0 - (age - a2) / max(a3 - a2, 1e-6))
    else:
        conf = 0.0
    return _clamp(conf, 0.0, 1.0)


def lock_origin(lat: float, lon: float) -> None:
    mi_t = _MISSION_t
    mi_t.origin_lat = float(lat)
    mi_t.origin_lon = float(lon)
    mi_t.origin_ready = True
    _STATE_t.gps.E = 0.0
    _STATE_t.gps.N = 0.0
    logger.info("Origin locked: lat=%.6f lon=%.6f", lat, lon)
    if _ok(mi_t._target_lat) and _ok(mi_t._target_lon):
        tN, tE = latlon_to_ne(mi_t._target_lat, mi_t._target_lon, lat, lon)
        mi_t.target_E = tE
        mi_t.target_N = tN
        mi_t.target_ready = True
        logger.info("Target projected on lock: E=%.1f N=%.1f", tE, tN)


def _is_fresh(valid: bool, ts: float, now: float, max_age: float) -> bool:
    return bool(valid) and isfinite(ts) and 0.0 <= now - ts <= max_age


def UpdateRaw(gps=None, imu=None, baro=None, now: float | None = None) -> None:
    st_t = _STATE_t
    mi_t = _MISSION_t

    if gps is not None:
        pos_health = getattr(gps, "pos_health", 0)
        motion_health = getattr(gps, "motion_health", 0)
        if pos_health:
            lat = getattr(gps, "lat", None)
            lon = getattr(gps, "lon", None)
            pos_ts = getattr(gps, "pos_ts", None)
            if _ok(lat) and _ok(lon) and _ok(pos_ts):
                if mi_t.origin_ready:
                    N, E = latlon_to_ne(float(lat), float(lon),
                                        mi_t.origin_lat, mi_t.origin_lon)
                    st_t.gps.E = E
                    st_t.gps.N = N
                st_t.gps.pos_ts = float(pos_ts)
                st_t.gps.pos_valid = True
        if motion_health:
            course_rad = getattr(gps, "course_rad", None)
            speed_mps = getattr(gps, "speed_mps", None)
            motion_ts = getattr(gps, "motion_ts", None)
            if _ok(course_rad) and _ok(speed_mps) and _ok(motion_ts):
                st_t.gps.V = float(speed_mps)
                st_t.gps.course = float(course_rad)
                st_t.gps.motion_ts = float(motion_ts)
                st_t.gps.motion_valid = True

    if imu is not None:
        imu_health = getattr(imu, "health", 0)
        imu_ts = getattr(imu, "ts", None)
        if imu_health and _ok(imu_ts):
            yaw_r = getattr(imu, "yaw_rad", None)
            gyrz = getattr(imu, "gyrz_rad_s", None)
            lax = getattr(imu, "lin_acc_x", None)
            lay = getattr(imu, "lin_acc_y", None)
            lin_ok = bool(getattr(imu, "lin_acc_valid", False))
            st_t.imu.ts = float(imu_ts)
            if _ok(yaw_r):
                st_t.imu.yaw = float(yaw_r)
                st_t.imu.yaw_valid = True
            if _ok(gyrz):
                st_t.imu.gyr_z = float(gyrz)
                st_t.imu.gyrz_valid = True
            if lin_ok and _ok(lax) and _ok(lay):
                st_t.imu.lin_acc_x = float(lax)
                st_t.imu.lin_acc_y = float(lay)
                st_t.imu.lin_acc_valid = True

    if baro is not None:
        baro_health = getattr(baro, "health", 0)
        baro_ts = getattr(baro, "rx_ts", getattr(baro, "ts", None))
        alt_m = getattr(baro, "alt_m", None)
        if baro_health and _ok(baro_ts):
            st_t.baro.ts = float(baro_ts)
            if _ok(alt_m):
                st_t.baro.alt_m = float(alt_m)
            sink = getattr(baro, "sink_rate", None)
            if _ok(sink):
                st_t.baro.sink_rate = float(sink)
                st_t.baro.valid = True


UpdateRaws = UpdateRaw


def ComputeFreshFlags(now: float) -> SensorFreshFlags:
    st_t = _STATE_t
    flags = SensorFreshFlags()
    flags.gps_pos_fresh = _is_fresh(
        st_t.gps.pos_valid, st_t.gps.pos_ts, now, config.GPS_FRESH_MAX_AGE_S
    )
    flags.gps_motion_fresh = _is_fresh(
        st_t.gps.motion_valid, st_t.gps.motion_ts, now, config.GPS_FRESH_MAX_AGE_S
    )
    imu_base_fresh = _is_fresh(True, st_t.imu.ts, now, config.IMU_FRESH_MAX_AGE_S)
    flags.imu_yaw_fresh = imu_base_fresh and st_t.imu.yaw_valid and isfinite(st_t.imu.yaw)
    flags.imu_gyrz_fresh = imu_base_fresh and st_t.imu.gyrz_valid and isfinite(st_t.imu.gyr_z)
    flags.baro_sink_fresh = _is_fresh(
        st_t.baro.valid and isfinite(st_t.baro.sink_rate),
        st_t.baro.ts,
        now,
        config.BARO_FRESH_MAX_AGE_S,
    )
    flags.acc_fresh = (
        imu_base_fresh
        and st_t.imu.lin_acc_valid
        and isfinite(st_t.imu.lin_acc_x)
        and isfinite(st_t.imu.lin_acc_y)
    )
    flags.gps_pos_stale = not flags.gps_pos_fresh
    flags.gps_motion_stale = not flags.gps_motion_fresh
    flags.imu_yaw_stale = not flags.imu_yaw_fresh
    flags.imu_gyrz_stale = not flags.imu_gyrz_fresh
    flags.baro_sink_stale = not flags.baro_sink_fresh
    flags.acc_stale = not flags.acc_fresh
    flags.dr_anchor_valid = dr_anchor_valid(st_t.dr)
    flags.dr_current_valid = dr_current_valid(st_t.dr)
    flags.nav_valid = (
        isfinite(st_t.nav.E) and isfinite(st_t.nav.N)
        and isfinite(st_t.nav.V) and isfinite(st_t.nav.course)
    )
    st_t.flags = flags
    return flags


def FillNav(flags: SensorFreshFlags, now: float) -> None:
    st_t = _STATE_t
    if flags.gps_pos_fresh and flags.gps_motion_fresh:
        st_t.nav.E = st_t.gps.E
        st_t.nav.N = st_t.gps.N
        st_t.nav.V = st_t.gps.V
        st_t.nav.course = st_t.gps.course
        st_t.nav.vE = st_t.gps.V * math.sin(st_t.gps.course)
        st_t.nav.vN = st_t.gps.V * math.cos(st_t.gps.course)
        st_t.nav.confidence = 1.0
        st_t.nav.valid = True
        st_t.nav.timestamp = now
        flags.nav_valid = True


def _bootstrap_course(flags: SensorFreshFlags) -> float:
    st_t = _STATE_t
    dr = st_t.dr
    if flags.imu_yaw_fresh:
        return st_t.imu.yaw
    if isfinite(dr.current_course):
        return dr.current_course
    if isfinite(dr.anchor_course):
        return dr.anchor_course
    return nan


def _bootstrap_speed(flags: SensorFreshFlags) -> float:
    st_t = _STATE_t
    if flags.baro_sink_fresh:
        gain = getattr(config, "DR_SINK_TO_HSPEED_GAIN", 1.0)
        return _clamp(st_t.baro.sink_rate * gain, config.V_MIN_MPS, config.V_MAX_DR_MPS)
    v = _last_v(st_t.dr)
    if isfinite(v):
        return v
    return config.V_MIN_MPS


def FillDRAnchor(flags: SensorFreshFlags, now: float) -> None:
    st_t = _STATE_t
    if flags.gps_pos_fresh and flags.gps_motion_fresh:
        imu_yaw = st_t.imu.yaw if flags.imu_yaw_fresh else nan
        dr_lock(st_t.dr, st_t.gps.E, st_t.gps.N, st_t.gps.V, st_t.gps.course, imu_yaw, now)
    elif flags.gps_pos_fresh and flags.gps_motion_stale:
        course = _bootstrap_course(flags)
        if isfinite(course):
            speed = _bootstrap_speed(flags)
            if not dr_anchor_valid(st_t.dr):
                yaw = st_t.imu.yaw if flags.imu_yaw_fresh else nan
                dr_lock(st_t.dr, st_t.gps.E, st_t.gps.N, speed, course, yaw, now)
            st_t.dr.current_E = st_t.gps.E
            st_t.dr.current_N = st_t.gps.N
            st_t.dr.current_course = course
            st_t.dr.current_V = speed
            st_t.dr.current_time = now
            st_t.dr.confidence = 1.0 if not isfinite(st_t.dr.anchor_time) else _compute_dr_confidence(now - st_t.dr.anchor_time)
    flags.dr_anchor_valid = dr_anchor_valid(st_t.dr)
    flags.dr_current_valid = dr_current_valid(st_t.dr)
    st_t.flags = flags


def SelectControlMode(flags: SensorFreshFlags, now: float) -> ControlMode:
    mi_t = _MISSION_t
    st_t = _STATE_t
    if not mi_t.origin_ready or not mi_t.target_ready:
        return ControlMode.FAIL

    if flags.gps_pos_fresh and flags.gps_motion_fresh:
        return ControlMode.GPS_TRACKING_CLOSED if flags.imu_gyrz_fresh else ControlMode.GPS_TRACKING_OPEN

    if flags.gps_pos_fresh and flags.gps_motion_stale and flags.dr_current_valid:
        if flags.imu_gyrz_fresh and flags.baro_sink_fresh and flags.acc_fresh:
            return ControlMode.DR_M_GBA_CLOSED
        if flags.imu_gyrz_fresh and flags.baro_sink_fresh:
            return ControlMode.DR_M_GB_CLOSED
        if flags.imu_gyrz_fresh and last_v_valid(st_t.dr):
            return ControlMode.DR_M_G_CLOSED
        if flags.imu_yaw_fresh and flags.baro_sink_fresh and flags.acc_fresh:
            return ControlMode.DR_M_YBA_OPEN
        if flags.imu_yaw_fresh and flags.baro_sink_fresh:
            return ControlMode.DR_M_YB_OPEN
        if flags.imu_yaw_fresh and last_v_valid(st_t.dr):
            return ControlMode.DR_M_Y_OPEN

    if flags.dr_current_valid:
        if flags.imu_gyrz_fresh and flags.baro_sink_fresh and flags.acc_fresh:
            return ControlMode.DR_PM_GBA_CLOSED
        if flags.imu_gyrz_fresh and flags.baro_sink_fresh:
            return ControlMode.DR_PM_GB_CLOSED
        if flags.imu_gyrz_fresh and last_v_valid(st_t.dr):
            return ControlMode.DR_PM_G_CLOSED
        if flags.imu_yaw_fresh and flags.baro_sink_fresh and flags.acc_fresh:
            return ControlMode.DR_PM_YBA_OPEN
        if flags.imu_yaw_fresh and flags.baro_sink_fresh:
            return ControlMode.DR_PM_YB_OPEN
        if flags.imu_yaw_fresh and last_v_valid(st_t.dr):
            return ControlMode.DR_PM_Y_OPEN

    return ControlMode.FAIL


def DecideControlMode(gps=None, imu=None, baro=None, now: float | None = None) -> ControlMode:
    if now is None and isinstance(gps, (int, float)):
        now = float(gps)
        gps = imu = baro = None
    if now is None:
        raise TypeError("DecideControlMode requires now")
    UpdateRaw(gps, imu, baro, now)
    flags = ComputeFreshFlags(now)
    FillNav(flags, now)
    FillDRAnchor(flags, now)
    mode = SelectControlMode(flags, now)
    _STATE_t.nav.control_mode = mode
    return mode


def TryInitStateFromPosOnly(now: float) -> None:
    flags = ComputeFreshFlags(now)
    FillDRAnchor(flags, now)


def _update_state_from_dead_reckoning(now: float) -> None:
    """DEPRECATED: superseded by _fill_nav_for_dr_m_mode / _fill_nav_for_dr_pm_mode.

    No longer called by ProduceL1Input. Kept temporarily for reference; remove
    once downstream tooling/docs no longer reference it.
    """
    st_t = _STATE_t
    dr = st_t.dr
    flags = st_t.flags
    mode = st_t.nav.control_mode

    if isfinite(dr.last_step_time):
        dt = now - dr.last_step_time
    elif isfinite(dr.current_time):
        dt = now - dr.current_time
    elif isfinite(dr.anchor_time):
        dt = now - dr.anchor_time
    else:
        dt = 0.0
    dt = _clamp(dt, 0.0, 0.5)

    if not dr_current_valid(dr) and dr_anchor_valid(dr):
        dr.current_E = dr.anchor_E
        dr.current_N = dr.anchor_N
        dr.current_V = dr.anchor_V
        dr.current_course = dr.anchor_course
        dr.current_time = dr.anchor_time

    if _mode_uses_gyro(mode) and flags.imu_gyrz_fresh:
        dr.gyro_integral += st_t.imu.gyr_z * config.GYRZ_SIGN * dt

    course_est = dr_estimate_course(
        dr,
        st_t.imu,
        use_gyro=_mode_uses_gyro(mode) and flags.imu_gyrz_fresh,
        use_yaw=_mode_uses_yaw(mode) and flags.imu_yaw_fresh,
    )
    if not isfinite(course_est):
        course_est = dr.current_course if isfinite(dr.current_course) else dr.anchor_course

    if _mode_uses_baro(mode) and flags.baro_sink_fresh:
        gain = getattr(config, "DR_SINK_TO_HSPEED_GAIN", 1.0)
        V_dr = _clamp(st_t.baro.sink_rate * gain, config.V_MIN_MPS, config.V_MAX_DR_MPS)
    else:
        V_dr = _last_v(dr)
        if not isfinite(V_dr):
            V_dr = config.V_MIN_MPS

    vE = V_dr * math.sin(course_est)
    vN = V_dr * math.cos(course_est)
    method = DRMethod.GYRO_INTEGRATION if _mode_uses_gyro(mode) else DRMethod.YAW_DELTA

    gyrz_too_fast = (
        flags.imu_gyrz_fresh
        and abs(st_t.imu.gyr_z) > math.radians(config.ACC_GYRZ_REJECT_DPS)
    )
    acc_enabled = getattr(config, "USE_ACC_BLEND_CORRECTION", config.USE_ACC_DOUBLE_INTEGRATION)
    if acc_enabled and _mode_uses_acc(mode) and flags.acc_fresh and not gyrz_too_fast:
        lax = st_t.imu.lin_acc_x * config.ACC_X_SIGN
        lay = st_t.imu.lin_acc_y * config.ACC_Y_SIGN
        if math.hypot(lax, lay) <= config.ACC_LIMIT_MPS2:
            aE = lax * math.sin(course_est) + lay * math.cos(course_est)
            aN = lax * math.cos(course_est) - lay * math.sin(course_est)
            vE += config.ACC_BLEND_WEIGHT * aE * dt
            vN += config.ACC_BLEND_WEIGHT * aN * dt
            method = DRMethod.GYRO_ACC_BLEND if _mode_uses_gyro(mode) else DRMethod.YAW_ACC_BLEND

    if _mode_estimates_P(mode):
        dr.current_E += vE * dt
        dr.current_N += vN * dt
    elif _mode_value(mode).startswith("DR_M_") and flags.gps_pos_fresh:
        dr.current_E = st_t.gps.E
        dr.current_N = st_t.gps.N
    else:
        dr.current_E += vE * dt
        dr.current_N += vN * dt

    dr.current_V = V_dr
    dr.current_course = course_est
    dr.current_time = now
    dr.method = method
    dr.last_step_time = now
    dr.confidence = _compute_dr_confidence(now - dr.anchor_time)

    st_t.nav.E = dr.current_E
    st_t.nav.N = dr.current_N
    st_t.nav.V = dr.current_V
    st_t.nav.course = dr.current_course
    st_t.nav.confidence = dr.confidence
    st_t.flags.dr_current_valid = dr_current_valid(dr)


def _estimate_course_for_mode(mode: ControlMode, now: float, dt: float):
    """Estimate course (rad) for a DR mode from gyro and/or yaw.

    Returns (valid, course_rad, reason). G modes integrate gyro_z onto the
    anchor course and blend yaw when both agree; Y modes propagate yaw delta
    from the anchor. A mode whose declared source is no longer fresh is
    defensively rejected with NO_COURSE_SOURCE.
    """
    st_t = _STATE_t
    dr = st_t.dr
    imu = st_t.imu
    flags = st_t.flags

    if _mode_uses_gyro(mode):
        if not (flags.imu_gyrz_fresh and isfinite(imu.gyr_z)):
            return (False, nan, "NO_COURSE_SOURCE")
        dr.gyro_integral += imu.gyr_z * config.GYRZ_SIGN * dt
        base = dr.anchor_course if isfinite(dr.anchor_course) else dr.current_course
        if not isfinite(base):
            return (False, nan, "NO_COURSE_SOURCE")
        course_gyro = _wrap_pi(base + dr.gyro_integral)
        if (flags.imu_yaw_fresh and isfinite(imu.yaw)
                and isfinite(dr.yaw_at_anchor) and isfinite(dr.anchor_course)):
            course_yaw = _wrap_pi(dr.anchor_course + _wrap_pi(imu.yaw - dr.yaw_at_anchor))
            limit = math.radians(getattr(config, "YAW_GYRO_BLEND_MAX_DEG", 45.0))
            if abs(_wrap_pi(course_yaw - course_gyro)) <= limit:
                return (True, _circular_mean(course_yaw, course_gyro), "COURSE_GYRO_YAW")
        return (True, course_gyro, "COURSE_GYRO")

    if _mode_uses_yaw(mode):
        if not (flags.imu_yaw_fresh and isfinite(imu.yaw)):
            return (False, nan, "NO_COURSE_SOURCE")
        if isfinite(dr.yaw_at_anchor) and isfinite(dr.anchor_course):
            course = _wrap_pi(dr.anchor_course + _wrap_pi(imu.yaw - dr.yaw_at_anchor))
        else:
            course = _wrap_pi(imu.yaw)
        return (True, course, "COURSE_YAW")

    return (False, nan, "NO_COURSE_SOURCE")


def _estimate_speed_for_mode(mode: ControlMode):
    """Estimate ground speed (m/s) for a DR mode.

    Returns (valid, V_mps, reason). B modes derive speed from baro sink rate;
    non-B modes fall back to the last known velocity (current_V else anchor_V).
    """
    st_t = _STATE_t
    dr = st_t.dr
    flags = st_t.flags
    v_min = config.V_MIN_MPS
    v_max = config.V_MAX_DR_MPS

    if _mode_uses_baro(mode):
        if (flags.baro_sink_fresh and isfinite(st_t.baro.sink_rate)
                and st_t.baro.sink_rate > 0.0):
            gain = getattr(config, "DR_SINK_TO_HSPEED_GAIN", 1.0)
            return (True, _clamp(st_t.baro.sink_rate * gain, v_min, v_max), "SPEED_BARO")
        v = _last_v(dr)
        if isfinite(v):
            return (True, _clamp(v, v_min, v_max), "SPEED_LASTV_FALLBACK")
        return (False, nan, "NO_SPEED_SOURCE")

    v = _last_v(dr)
    if not isfinite(v):
        return (False, nan, "NO_SPEED_SOURCE")
    return (True, _clamp(v, v_min, v_max), "SPEED_LASTV")


def _apply_acc_correction_if_needed(mode: ControlMode, course: float, V: float, dt: float):
    """Build the NE velocity vector from (course, V), optionally weak-corrected by lin-acc.

    Returns (vE, vN, reason). Linear acceleration is used only as a weak blend
    (ACC_BLEND_WEIGHT), never double-integrated, and is rejected on fast spin or
    out-of-range magnitude. The resulting magnitude is conservatively capped at
    V_MAX_DR_MPS along its own direction.
    """
    st_t = _STATE_t
    flags = st_t.flags
    imu = st_t.imu
    vE = V * math.sin(course)
    vN = V * math.cos(course)

    if not _mode_uses_acc(mode):
        return (vE, vN, "NO_ACC")

    acc_enabled = getattr(config, "USE_ACC_BLEND_CORRECTION", config.USE_ACC_DOUBLE_INTEGRATION)
    if not (acc_enabled and flags.acc_fresh
            and isfinite(imu.lin_acc_x) and isfinite(imu.lin_acc_y)):
        return (vE, vN, "ACC_NOT_FRESH")

    if flags.imu_gyrz_fresh and abs(imu.gyr_z) > math.radians(config.ACC_GYRZ_REJECT_DPS):
        return (vE, vN, "ACC_GYRZ_REJECT")

    lax = imu.lin_acc_x * config.ACC_X_SIGN
    lay = imu.lin_acc_y * config.ACC_Y_SIGN
    if math.hypot(lax, lay) > config.ACC_LIMIT_MPS2:
        return (vE, vN, "ACC_LIMIT")

    yaw_for_acc = imu.yaw if (flags.imu_yaw_fresh and isfinite(imu.yaw)) else course
    aE = lax * math.sin(yaw_for_acc) + lay * math.cos(yaw_for_acc)
    aN = lax * math.cos(yaw_for_acc) - lay * math.sin(yaw_for_acc)
    vE += config.ACC_BLEND_WEIGHT * aE * dt
    vN += config.ACC_BLEND_WEIGHT * aN * dt

    mag = math.hypot(vE, vN)
    if mag > config.V_MAX_DR_MPS and mag > 1e-9:
        scale = config.V_MAX_DR_MPS / mag
        vE *= scale
        vN *= scale
    return (vE, vN, "ACC_BLEND")


def _dr_method_for(mode: ControlMode, acc_applied: bool) -> DRMethod:
    if _mode_uses_gyro(mode):
        return DRMethod.GYRO_ACC_BLEND if acc_applied else DRMethod.GYRO_INTEGRATION
    if _mode_uses_yaw(mode):
        return DRMethod.YAW_ACC_BLEND if acc_applied else DRMethod.YAW_DELTA
    return DRMethod.NONE


def _fill_nav_for_dr_m_mode(mode: ControlMode, now: float):
    """DR_M_*: GPS position is fresh, only motion is estimated.

    P = GPS position, M = estimated course/speed (+optional acc blend).
    Position is never integrated here. Returns (ok, reason).
    """
    st_t = _STATE_t
    dr = st_t.dr
    flags = st_t.flags

    if not flags.gps_pos_fresh:
        return (False, "GPS_POS_STALE")
    if not (isfinite(st_t.gps.E) and isfinite(st_t.gps.N)):
        return (False, "GPS_POS_NAN")

    previous = (dr.last_step_time if isfinite(dr.last_step_time)
                else dr.current_time if isfinite(dr.current_time) else now)
    dt = _safe_dt(now, previous)

    cok, course, creason = _estimate_course_for_mode(mode, now, dt)
    if not cok:
        return (False, creason)
    sok, V, sreason = _estimate_speed_for_mode(mode)
    if not sok:
        return (False, sreason)
    vE, vN, acc_reason = _apply_acc_correction_if_needed(mode, course, V, dt)

    nav = st_t.nav
    nav.E = st_t.gps.E
    nav.N = st_t.gps.N
    nav.course = course
    nav.V = math.hypot(vE, vN)
    nav.vE = vE
    nav.vN = vN
    nav.valid = True
    nav.timestamp = now

    dr.current_E = nav.E
    dr.current_N = nav.N
    dr.current_V = nav.V
    dr.current_course = nav.course
    dr.current_time = now
    dr.last_step_time = now
    dr.method = _dr_method_for(mode, acc_reason == "ACC_BLEND")
    dr.confidence = (_compute_dr_confidence(now - dr.anchor_time)
                     if isfinite(dr.anchor_time) else 1.0)
    nav.confidence = dr.confidence
    flags.dr_current_valid = dr_current_valid(dr)
    return (True, mode.value)


def _fill_nav_for_dr_pm_mode(mode: ControlMode, now: float):
    """DR_PM_*: GPS position is stale — true dead reckoning.

    P and M are both propagated/estimated from dr.current. The entry check is
    position-focused (E/N/course/time); speed is validated separately so a
    missing speed source surfaces as NO_SPEED_SOURCE. Returns (ok, reason).
    """
    st_t = _STATE_t
    dr = st_t.dr
    nav = st_t.nav
    flags = st_t.flags

    if not (isfinite(dr.current_E) and isfinite(dr.current_N)
            and isfinite(dr.current_course) and isfinite(dr.current_time)):
        return (False, "NO_DR_CURRENT")

    previous = dr.last_step_time if isfinite(dr.last_step_time) else dr.current_time
    dt = _safe_dt(now, previous)

    cok, course, creason = _estimate_course_for_mode(mode, now, dt)
    if not cok:
        return (False, creason)
    sok, V, sreason = _estimate_speed_for_mode(mode)
    if not sok:
        return (False, sreason)
    vE, vN, acc_reason = _apply_acc_correction_if_needed(mode, course, V, dt)

    dr.current_E += vE * dt
    dr.current_N += vN * dt
    dr.current_V = math.hypot(vE, vN)
    dr.current_course = course
    dr.current_time = now
    dr.last_step_time = now
    dr.method = _dr_method_for(mode, acc_reason == "ACC_BLEND")
    dr.confidence = (_compute_dr_confidence(now - dr.anchor_time)
                     if isfinite(dr.anchor_time) else 1.0)

    nav.E = dr.current_E
    nav.N = dr.current_N
    nav.V = dr.current_V
    nav.course = dr.current_course
    nav.vE = vE
    nav.vN = vN
    nav.valid = True
    nav.timestamp = now
    nav.confidence = dr.confidence
    flags.dr_current_valid = dr_current_valid(dr)
    return (True, mode.value)


def _make_l1input_from_nav(mode: ControlMode, reason: str) -> L1Input:
    """Build an L1Input from the filled NavState, validating finiteness/readiness.

    confidence is 1.0 for GPS tracking and dr.confidence for DR modes. Any
    non-finite nav/target field, V below V_MIN_MPS, or missing origin/target
    yields an invalid L1Input (reason NAV_INVALID).
    """
    st_t = _STATE_t
    mi_t = _MISSION_t
    nav = st_t.nav

    if _is_gps_tracking_mode(mode):
        confidence = 1.0
        dr_method = DRMethod.NONE
    else:
        confidence = st_t.dr.confidence if isfinite(st_t.dr.confidence) else 1.0
        dr_method = st_t.dr.method

    vE = nav.vE
    vN = nav.vN
    if not (isfinite(vE) and isfinite(vN)) and isfinite(nav.V) and isfinite(nav.course):
        vE = nav.V * math.sin(nav.course)
        vN = nav.V * math.cos(nav.course)

    valid = (
        isfinite(nav.E) and isfinite(nav.N)
        and isfinite(nav.V) and nav.V >= config.V_MIN_MPS
        and isfinite(nav.course)
        and isfinite(mi_t.target_E) and isfinite(mi_t.target_N)
        and mi_t.target_ready and mi_t.origin_ready
    )
    if not valid:
        return L1Input(valid=False, reason="NAV_INVALID", control_mode=mode,
                       dr_method=dr_method, confidence=confidence)

    return L1Input(
        valid=True, reason=reason, control_mode=mode, dr_method=dr_method,
        confidence=confidence,
        E=nav.E, N=nav.N, V=nav.V, course=nav.course, vE=vE, vN=vN,
        target_E=mi_t.target_E, target_N=mi_t.target_N,
    )


def ProduceL1Input(now: float) -> L1Input:
    st_t = _STATE_t
    mi_t = _MISSION_t
    mode = st_t.nav.control_mode

    if mode == ControlMode.FAIL:
        return L1Input(valid=False, reason="FAIL", control_mode=mode)
    if mode == ControlMode.DETUMBLING:
        return L1Input(valid=False, reason="DETUMBLING", control_mode=mode)

    if not mi_t.origin_ready:
        return L1Input(valid=False, reason="NO_ORIGIN", control_mode=mode)
    if not mi_t.target_ready:
        return L1Input(valid=False, reason="NO_TARGET", control_mode=mode)

    if _is_gps_tracking_mode(mode):
        if not (isfinite(st_t.nav.E) and isfinite(st_t.nav.N)
                and isfinite(st_t.nav.V) and isfinite(st_t.nav.course)):
            return L1Input(valid=False, reason="GPS_NAV_INVALID", control_mode=mode)
        return _make_l1input_from_nav(mode, reason="GPS_NAV")

    if _mode_value(mode).startswith("DR_M_"):
        ok, reason = _fill_nav_for_dr_m_mode(mode, now)
        if not ok:
            return L1Input(valid=False, reason=reason, control_mode=mode)
        return _make_l1input_from_nav(mode, reason=reason)

    if _mode_value(mode).startswith("DR_PM_"):
        ok, reason = _fill_nav_for_dr_pm_mode(mode, now)
        if not ok:
            return L1Input(valid=False, reason=reason, control_mode=mode)
        return _make_l1input_from_nav(mode, reason=reason)

    return L1Input(valid=False, reason="UNKNOWN_MODE", control_mode=mode)


# Per-mode yaw-rate limit (deg/s) keyed by ControlMode.value. Missing modes
# (DETUMBLING/FAIL/unknown) fall through to 0.0.
_YAW_RATE_LIMIT_DPS_BY_MODE = {
    ControlMode.GPS_TRACKING_CLOSED.value: "GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS",
    ControlMode.GPS_TRACKING_OPEN.value:   "GPS_TRACKING_OPEN_YAW_RATE_LIMIT_DPS",
    ControlMode.DR_M_GBA_CLOSED.value:     "DR_M_GBA_YAW_RATE_LIMIT_DPS",
    ControlMode.DR_M_GB_CLOSED.value:      "DR_M_GB_YAW_RATE_LIMIT_DPS",
    ControlMode.DR_M_G_CLOSED.value:       "DR_M_G_YAW_RATE_LIMIT_DPS",
    ControlMode.DR_M_YBA_OPEN.value:       "DR_M_YBA_YAW_RATE_LIMIT_DPS",
    ControlMode.DR_M_YB_OPEN.value:        "DR_M_YB_YAW_RATE_LIMIT_DPS",
    ControlMode.DR_M_Y_OPEN.value:         "DR_M_Y_YAW_RATE_LIMIT_DPS",
    ControlMode.DR_PM_GBA_CLOSED.value:    "DR_PM_GBA_YAW_RATE_LIMIT_DPS",
    ControlMode.DR_PM_GB_CLOSED.value:     "DR_PM_GB_YAW_RATE_LIMIT_DPS",
    ControlMode.DR_PM_G_CLOSED.value:      "DR_PM_G_YAW_RATE_LIMIT_DPS",
    ControlMode.DR_PM_YBA_OPEN.value:      "DR_PM_YBA_YAW_RATE_LIMIT_DPS",
    ControlMode.DR_PM_YB_OPEN.value:       "DR_PM_YB_YAW_RATE_LIMIT_DPS",
    ControlMode.DR_PM_Y_OPEN.value:        "DR_PM_Y_YAW_RATE_LIMIT_DPS",
    ControlMode.DETUMBLING.value:          "DETUMBLING_YAW_RATE_LIMIT_DPS",
    ControlMode.FAIL.value:                "FAIL_YAW_RATE_LIMIT_DPS",
}


def _choose_yaw_rate_limit_rad_s(mode: ControlMode) -> float:
    """Return the per-mode yaw-rate limit in rad/s (0.0 for unknown modes)."""
    attr = _YAW_RATE_LIMIT_DPS_BY_MODE.get(_mode_value(mode))
    if attr is None:
        return 0.0
    return math.radians(getattr(config, attr, 0.0))


# Backward-compatible alias (older call sites expect this name, rad/s).
def _choose_yaw_rate_limit(mode: ControlMode) -> float:
    return _choose_yaw_rate_limit_rad_s(mode)


def _target_reached_radius_m() -> float:
    return getattr(config, "TARGET_RADIUS_M", 0.0)


def ProduceL1Output(l1in: L1Input) -> L1Output:
    output_t = L1Output(
        control_mode=l1in.control_mode,
        dr_method=l1in.dr_method,
        confidence=l1in.confidence,
        pos_E=l1in.E, pos_N=l1in.N,
        nav_E=l1in.E, nav_N=l1in.N,
        target_E=l1in.target_E, target_N=l1in.target_N,
        course=l1in.course,
        ground_speed_mps=l1in.V if _ok(l1in.V) else 0.0,
        V=l1in.V if _ok(l1in.V) else 0.0,
    )

    def _invalid(reason: str) -> L1Output:
        output_t.control_valid = False
        output_t.valid = False
        output_t.nominal = False
        output_t.reason = reason
        output_t.yaw_rate_cmd = 0.0
        return output_t

    if not l1in.valid:
        return _invalid(l1in.reason)

    if l1in.control_mode == ControlMode.FAIL:
        return _invalid("FAIL")

    if l1in.control_mode == ControlMode.DETUMBLING:
        lim = _choose_yaw_rate_limit_rad_s(ControlMode.DETUMBLING)
        output_t.control_valid = True
        output_t.valid = True
        output_t.nominal = False
        output_t.reason = "DETUMBLING"
        output_t.pid_enabled = False
        output_t.yaw_rate_cmd = 0.0
        output_t.yaw_rate_limit = lim
        output_t.yaw_rate_limit_dps = math.degrees(lim)
        return output_t

    for v in (l1in.E, l1in.N, l1in.target_E, l1in.target_N, l1in.course, l1in.V):
        if not _ok(v):
            return _invalid("NAN_NAV_STATE")

    if l1in.V < config.V_MIN_MPS:
        return _invalid("V_TOO_SMALL")

    if _is_dr_mode(l1in.control_mode):
        conf = _clamp(l1in.confidence, 0.0, 1.0)
        if conf < getattr(config, "DR_MIN_CONFIDENCE_FOR_CONTROL", 0.15):
            return _invalid("LOW_CONFIDENCE")
    else:
        conf = 1.0

    dE = l1in.target_E - l1in.E
    dN = l1in.target_N - l1in.N
    dist = math.hypot(dE, dN)
    output_t.distance_to_target = dist
    output_t.dist_to_target = dist

    if dist <= _target_reached_radius_m():
        output_t.target_bearing = _wrap_pi(math.atan2(dE, dN))
        return _invalid("TARGET_REACHED")

    target_bearing = _wrap_pi(math.atan2(dE, dN))
    nu = _wrap_pi(target_bearing - l1in.course)
    nu_clamped = _clamp(nu, -pi / 2.0, pi / 2.0)
    if abs(nu) < math.radians(config.NU_DEADBAND_DEG):
        sin_nu_eff = 0.0
    else:
        sin_nu_eff = math.sin(nu_clamped)

    _v_max = config.V_MAX_DR_MPS if _is_dr_mode(l1in.control_mode) else config.V_MAX_MPS
    V_eff = _clamp(l1in.V, config.V_MIN_MPS, _v_max)
    yaw_rate_cmd = 2.0 * V_eff / config.L_GAIN_M * sin_nu_eff
    yaw_rate_cmd *= conf  # confidence scaling: 1.0 for GPS, dr.confidence for DR
    lim = _choose_yaw_rate_limit_rad_s(l1in.control_mode)
    yaw_rate_cmd = _clamp(yaw_rate_cmd, -lim, lim)

    output_t.target_bearing = target_bearing
    output_t.nu = nu
    output_t.yaw_rate_cmd = yaw_rate_cmd
    output_t.yaw_rate_limit = lim
    output_t.yaw_rate_limit_dps = math.degrees(lim)
    output_t.control_valid = True
    output_t.valid = True
    output_t.nominal = True
    output_t.reason = l1in.reason
    output_t.pid_enabled = _mode_uses_gyro_feedback(l1in.control_mode)
    return output_t


def set_target(lat: float, lon: float) -> None:
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        logger.warning("set_target: invalid coords lat=%.6f lon=%.6f", lat, lon)
        return
    if abs(lat) < 1e-9 and abs(lon) < 1e-9:
        logger.warning("set_target: (0,0) sentinel rejected")
        return
    _MISSION_t._target_lat = lat
    _MISSION_t._target_lon = lon
    if _MISSION_t.origin_ready:
        tN, tE = latlon_to_ne(lat, lon, _MISSION_t.origin_lat, _MISSION_t.origin_lon)
        _MISSION_t.target_E = tE
        _MISSION_t.target_N = tN
        _MISSION_t.target_ready = True
        logger.info("set_target: projected E=%.1f N=%.1f", tE, tN)
    else:
        _MISSION_t.target_ready = False
        logger.info("set_target: saved (lat=%.6f lon=%.6f), waiting for origin", lat, lon)


def reset() -> None:
    global _MISSION_t, _STATE_t
    saved_lat = _MISSION_t._target_lat
    saved_lon = _MISSION_t._target_lon
    _MISSION_t = MissionFrame()
    _MISSION_t._target_lat = saved_lat
    _MISSION_t._target_lon = saved_lon
    _STATE_t = GuidanceState()
    logger.info("guidance.reset(): origin/nav/DR cleared, target lat/lon preserved")
