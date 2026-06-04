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
    target_lat: float = nan
    target_lon: float = nan


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
    fail_reason: str = "FAIL"     # SelectControlMode/guard가 FAIL 사유를 담는다
    speed_clamped: bool = False   # DR speed가 V_MAX_DR_MPS로 clamp되었는지
    baro_sink_spike: bool = False # 직전 사이클 baro sink 스파이크 거부 여부
    baro_sink_filtered: float = nan  # EMA 필터링된 sink (하강=양수), 디버그/로그용
    dr_speed_source: str = ""        # DR speed 출처: SPEED_BARO/SPEED_LASTV 등


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


def _safe_dt(now: float, previous: float, max_dt: float = 0.5) -> float:
    if not isfinite(now) or not isfinite(previous):
        return 0.0
    dt = now - previous
    if dt < 0.0:
        return 0.0
    return min(dt, max_dt)


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


def set_origin_point(lat: float, lon: float) -> None:
    _MISSION_t.origin_lat = float(lat)
    _MISSION_t.origin_lon = float(lon)
    _STATE_t.gps.E = 0.0
    _STATE_t.gps.N = 0.0
    logger.info("Origin set: lat=%.6f lon=%.6f", lat, lon)


def set_target_point(lat: float, lon: float) -> None:
    _MISSION_t.target_lat = float(lat)
    _MISSION_t.target_lon = float(lon)
    logger.info("Target set: lat=%.6f lon=%.6f", lat, lon)


def _get_target_ne() -> tuple[float, float] | None:
    mi_t = _MISSION_t
    if not (_ok(mi_t.origin_lat) and _ok(mi_t.origin_lon)):
        return None
    if not (_ok(mi_t.target_lat) and _ok(mi_t.target_lon)):
        return None
    tN, tE = latlon_to_ne(
        mi_t.target_lat, mi_t.target_lon, mi_t.origin_lat, mi_t.origin_lon
    )
    return tE, tN


def _is_fresh(valid: bool, ts: float, now: float, max_age: float) -> bool:
    return bool(valid) and isfinite(ts) and 0.0 <= now - ts <= max_age


def _descent_positive_sink(raw_sink: float) -> float:
    """raw sink_rate를 '하강=양수' 기준으로 변환.

    config.BARO_SINK_POSITIVE_DOWN:
      True  → baro app이 하강 시 양수를 보냄(현 가정) → 그대로.
      False → 하강 시 음수 → 부호 반전.
    """
    if getattr(config, "BARO_SINK_POSITIVE_DOWN", True):
        return raw_sink
    return -raw_sink


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
                if _ok(mi_t.origin_lat) and _ok(mi_t.origin_lon):
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
            st_t.imu.ts = float(imu_ts)
            # 각 채널은 현재 sample에서 유효한 값만 valid=True로 둔다. 무효하면
            # valid=False + NaN clear → 오래된 acc/yaw/gyrz가 fresh처럼 남지 않게 한다.
            # (이전 sample 값을 재사용하려면 DRState anchor/current를 쓰고, raw imu
            #  field에는 남기지 않는다.)
            yaw_r = getattr(imu, "yaw_rad", None)
            if _ok(yaw_r):
                st_t.imu.yaw = float(yaw_r)
                st_t.imu.yaw_valid = True
            else:
                st_t.imu.yaw = nan
                st_t.imu.yaw_valid = False

            gyrz = getattr(imu, "gyrz_rad_s", None)
            if _ok(gyrz):
                st_t.imu.gyr_z = float(gyrz)
                st_t.imu.gyrz_valid = True
            else:
                st_t.imu.gyr_z = nan
                st_t.imu.gyrz_valid = False

            lax = getattr(imu, "lin_acc_x", None)
            lay = getattr(imu, "lin_acc_y", None)
            lin_ok = bool(getattr(imu, "lin_acc_valid", False))
            if lin_ok and _ok(lax) and _ok(lay):
                st_t.imu.lin_acc_x = float(lax)
                st_t.imu.lin_acc_y = float(lay)
                st_t.imu.lin_acc_valid = True
            else:
                st_t.imu.lin_acc_x = nan
                st_t.imu.lin_acc_y = nan
                st_t.imu.lin_acc_valid = False
        else:
            # health=False(하드웨어 이상) 또는 ts 무효: ts는 가능하면 갱신하되
            # 모든 valid flag를 내리고 값을 NaN으로 clear한다.
            if _ok(imu_ts):
                st_t.imu.ts = float(imu_ts)
            st_t.imu.yaw = nan
            st_t.imu.yaw_valid = False
            st_t.imu.gyr_z = nan
            st_t.imu.gyrz_valid = False
            st_t.imu.lin_acc_x = nan
            st_t.imu.lin_acc_y = nan
            st_t.imu.lin_acc_valid = False

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
                # 부호 규약 적용 → 하강=양수. spike 거부 후 EMA 필터링.
                sink_dp = _descent_positive_sink(float(sink))
                sink_max = getattr(config, "DR_BARO_SINK_MAX_MPS", 6.0)
                if isfinite(sink_dp) and abs(sink_dp) <= sink_max:
                    tau = getattr(config, "DR_SINK_EMA_TAU_S", 0.7)
                    prev_f = st_t.baro.filtered_sink_rate
                    prev_ts = st_t.baro.filtered_sink_ts
                    if not (isfinite(prev_f) and isfinite(prev_ts)):
                        filtered = sink_dp                       # 초기화: raw로 시작
                    else:
                        dt = st_t.baro.ts - prev_ts
                        if not isfinite(dt) or dt <= 0.0:
                            filtered = prev_f
                        else:
                            alpha = dt / (tau + dt)
                            filtered = prev_f + alpha * (sink_dp - prev_f)
                    st_t.baro.filtered_sink_rate = filtered
                    st_t.baro.filtered_sink_ts = st_t.baro.ts
                # spike이면 filtered 미갱신(이전 값 유지). baro_sink_fresh 강등은
                # ComputeFreshFlags의 spike 가드가 처리한다.
            else:
                st_t.baro.valid = False


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
    # DR safety guard #4: baro sink 스파이크 거부. |sink| > 임계값이면 이 사이클
    # baro_sink_fresh=False로 강등하고 nav에 플래그를 남긴다 (reason BARO_SINK_SPIKE).
    # config 누락 시 fallback을 inf가 아니라 추천값(6.0)으로 둬 guard가 꺼지지 않게 한다.
    sink_max = getattr(config, "DR_BARO_SINK_MAX_MPS", 6.0)
    sink_spike = (
        flags.baro_sink_fresh and isfinite(st_t.baro.sink_rate)
        and abs(st_t.baro.sink_rate) > sink_max
    )
    if sink_spike:
        flags.baro_sink_fresh = False
    st_t.nav.baro_sink_spike = bool(sink_spike)
    st_t.nav.baro_sink_filtered = st_t.baro.filtered_sink_rate  # 디버그/로그용
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
        # raw가 아니라 spike-reject + EMA 필터링된 sink를 쓴다.
        sink_used = st_t.baro.filtered_sink_rate
        if not isfinite(sink_used):
            sink_used = _descent_positive_sink(st_t.baro.sink_rate)
        return _clamp(sink_used * gain, config.V_MIN_MPS, config.V_MAX_DR_MPS)
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


def _fail(reason: str) -> ControlMode:
    _STATE_t.nav.fail_reason = reason
    return ControlMode.FAIL


def SelectControlMode(flags: SensorFreshFlags, now: float) -> ControlMode:
    mi_t = _MISSION_t
    st_t = _STATE_t
    st_t.nav.fail_reason = "FAIL"
    origin_ready = _ok(mi_t.origin_lat) and _ok(mi_t.origin_lon)
    target_ready = _ok(mi_t.target_lat) and _ok(mi_t.target_lon)
    if not origin_ready or not target_ready:
        return _fail("NO_ORIGIN" if not origin_ready else "NO_TARGET")

    # DR safety guard #5: gyrz가 과도하면 정상 guidance에 G(gyro)를 쓰지 않는다.
    # gyro_ok=False sends GPS tracking through the OPEN path.
    # config 누락 시 fallback을 inf가 아니라 추천값으로 둬 guard가 꺼지지 않게 한다.
    gyrz_max = getattr(config, "DR_MAX_YAW_RATE_DPS_FOR_CONTROL", 120.0)
    gyro_ok = flags.imu_gyrz_fresh and (
        not isfinite(st_t.imu.gyr_z)
        or abs(math.degrees(st_t.imu.gyr_z)) <= gyrz_max
    )

    if flags.gps_pos_fresh and flags.gps_motion_fresh:
        return ControlMode.GPS_TRACKING_CLOSED if gyro_ok else ControlMode.GPS_TRACKING_OPEN

    # DR safety guard #1: anchor가 너무 오래되면 DR을 신뢰하지 않는다.
    dr_age = now - st_t.dr.anchor_time if isfinite(st_t.dr.anchor_time) else float("inf")
    dr_too_old = dr_age > getattr(config, "DR_MAX_AGE_S", 45.0)

    if flags.gps_pos_fresh and flags.gps_motion_stale and flags.dr_current_valid:
        if dr_too_old:
            return _fail("DR_TIMEOUT")
        if gyro_ok and flags.baro_sink_fresh and flags.acc_fresh:
            return ControlMode.DR_M_GBA_CLOSED
        if gyro_ok and flags.baro_sink_fresh:
            return ControlMode.DR_M_GB_CLOSED
        if gyro_ok and last_v_valid(st_t.dr):
            return ControlMode.DR_M_G_CLOSED
        if flags.imu_yaw_fresh and flags.baro_sink_fresh and flags.acc_fresh:
            return ControlMode.DR_M_YBA_OPEN
        if flags.imu_yaw_fresh and flags.baro_sink_fresh:
            return ControlMode.DR_M_YB_OPEN
        if flags.imu_yaw_fresh and last_v_valid(st_t.dr):
            return ControlMode.DR_M_Y_OPEN

    if flags.dr_current_valid:
        if dr_too_old:
            return _fail("DR_TIMEOUT")
        if gyro_ok and flags.baro_sink_fresh and flags.acc_fresh:
            return ControlMode.DR_PM_GBA_CLOSED
        if gyro_ok and flags.baro_sink_fresh:
            return ControlMode.DR_PM_GB_CLOSED
        if gyro_ok and last_v_valid(st_t.dr):
            return ControlMode.DR_PM_G_CLOSED
        if flags.imu_yaw_fresh and flags.baro_sink_fresh and flags.acc_fresh:
            return ControlMode.DR_PM_YBA_OPEN
        if flags.imu_yaw_fresh and flags.baro_sink_fresh:
            return ControlMode.DR_PM_YB_OPEN
        if flags.imu_yaw_fresh and last_v_valid(st_t.dr):
            return ControlMode.DR_PM_Y_OPEN

    return _fail("NO_GUIDANCE_SOURCE")


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

    def _clamped(v_raw, reason):
        # DR safety guard #3: V를 V_MAX_DR_MPS로 clamp하고 nav.speed_clamped 기록.
        v_out = _clamp(v_raw, v_min, v_max)
        st_t.nav.speed_clamped = bool(v_raw > v_max or v_raw < v_min)
        st_t.nav.dr_speed_source = reason
        return (True, v_out, reason)

    if _mode_uses_baro(mode):
        # raw가 아니라 spike-reject + EMA 필터링된 sink(하강=양수)를 쓴다.
        sink_used = st_t.baro.filtered_sink_rate
        if flags.baro_sink_fresh and isfinite(sink_used) and sink_used > 0.0:
            gain = getattr(config, "DR_SINK_TO_HSPEED_GAIN", 1.0)
            return _clamped(sink_used * gain, "SPEED_BARO")
        v = _last_v(dr)
        if isfinite(v):
            return _clamped(v, "SPEED_LASTV_FALLBACK")
        return (False, nan, "NO_SPEED_SOURCE")

    v = _last_v(dr)
    if not isfinite(v):
        return (False, nan, "NO_SPEED_SOURCE")
    return _clamped(v, "SPEED_LASTV")


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

    acc_enabled = config.USE_ACC_BLEND_CORRECTION
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

    # DR safety guard #6: 한 사이클 위치 전파가 DR_MAX_POSITION_JUMP_M보다 크면
    # 비정상으로 보고 update를 reject한다 (current 미갱신, reason DR_POSITION_JUMP).
    step_E = vE * dt
    step_N = vN * dt
    jump_max = getattr(config, "DR_MAX_POSITION_JUMP_M", 3.0)
    if math.hypot(step_E, step_N) > jump_max:
        return (False, "DR_POSITION_JUMP")

    dr.current_E += step_E
    dr.current_N += step_N
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

    target_ne = _get_target_ne()
    valid = (
        isfinite(nav.E) and isfinite(nav.N)
        and isfinite(nav.V) and nav.V >= config.V_MIN_MPS
        and isfinite(nav.course)
        and target_ne is not None
    )
    if not valid:
        return L1Input(valid=False, reason="NAV_INVALID", control_mode=mode,
                       dr_method=dr_method, confidence=confidence)
    target_E, target_N = target_ne

    return L1Input(
        valid=True, reason=reason, control_mode=mode, dr_method=dr_method,
        confidence=confidence,
        E=nav.E, N=nav.N, V=nav.V, course=nav.course, vE=vE, vN=vN,
        target_E=target_E, target_N=target_N,
    )


def ProduceL1Input(now: float) -> L1Input:
    st_t = _STATE_t
    mi_t = _MISSION_t
    mode = st_t.nav.control_mode

    if mode == ControlMode.FAIL:
        return L1Input(valid=False, reason=st_t.nav.fail_reason or "FAIL",
                       control_mode=mode)
    if not (_ok(mi_t.origin_lat) and _ok(mi_t.origin_lon)):
        return L1Input(valid=False, reason="NO_ORIGIN", control_mode=mode)
    if not (_ok(mi_t.target_lat) and _ok(mi_t.target_lon)):
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
# (FAIL/unknown) fall through to 0.0.
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
    ControlMode.FAIL.value:                "FAIL_YAW_RATE_LIMIT_DPS",
}


def _choose_yaw_rate_limit_rad_s(mode: ControlMode) -> float:
    """Return the per-mode yaw-rate limit in rad/s (0.0 for unknown modes)."""
    attr = _YAW_RATE_LIMIT_DPS_BY_MODE.get(_mode_value(mode))
    if attr is None:
        return 0.0
    return math.radians(getattr(config, attr, 0.0))


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

    # mode별 yaw-rate limit을 즉시 채운다. invalid 경로(아래 _invalid)에서도
    # control.py가 항상 finite한 limit을 읽도록 보장한다(yaw_rate_limit_dps 항상 finite).
    lim = _choose_yaw_rate_limit_rad_s(l1in.control_mode)
    output_t.yaw_rate_limit = lim
    output_t.yaw_rate_limit_dps = math.degrees(lim)

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

    for v in (l1in.E, l1in.N, l1in.target_E, l1in.target_N, l1in.course, l1in.V):
        if not _ok(v):
            return _invalid("NAN_NAV_STATE")

    if l1in.V < config.V_MIN_MPS:
        return _invalid("V_TOO_SMALL")

    if _is_dr_mode(l1in.control_mode):
        conf = _clamp(l1in.confidence, 0.0, 1.0)
        # config 누락 시 fallback을 inf/0이 아니라 추천값(0.20)으로 둬 guard가 꺼지지 않게 한다.
        if conf < getattr(config, "DR_MIN_CONFIDENCE_FOR_CONTROL", 0.20):
            return _invalid("LOW_DR_CONFIDENCE")
    else:
        conf = 1.0

    dE = l1in.target_E - l1in.E
    dN = l1in.target_N - l1in.N
    dist = math.hypot(dE, dN)
    output_t.distance_to_target = dist
    output_t.dist_to_target = dist

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
    # lim은 함수 진입부에서 mode 기준으로 이미 채워졌다(output_t.yaw_rate_limit).
    yaw_rate_cmd = _clamp(yaw_rate_cmd, -lim, lim)

    output_t.target_bearing = target_bearing
    output_t.nu = nu
    output_t.yaw_rate_cmd = yaw_rate_cmd
    output_t.control_valid = True
    output_t.valid = True
    output_t.nominal = True
    output_t.reason = l1in.reason
    output_t.pid_enabled = _mode_uses_gyro_feedback(l1in.control_mode)
    return output_t


def reset() -> None:
    """origin/nav/DR을 초기화하고 target lat/lon만 보존한다.

    lock 여부는 motorapp이 관리한다. guidance는 frame 값만 비우고 채운다.
    """
    global _MISSION_t, _STATE_t
    prev = _MISSION_t
    saved_lat = prev.target_lat
    saved_lon = prev.target_lon
    _MISSION_t = MissionFrame()
    _MISSION_t.target_lat = saved_lat
    _MISSION_t.target_lon = saved_lon
    _STATE_t = GuidanceState()
    logger.info("guidance.reset(): origin/nav/DR cleared; target preserved")
