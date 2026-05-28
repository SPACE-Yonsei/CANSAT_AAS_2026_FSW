"""Sensor_Motor/guidance.py — Navigation + L1 Guidance

Pipeline (매 사이클):
  UpdateAnchors(gps, imu, baro, now)
    → DecideControlMode(now)
    → ProduceL1Input(now)
    → ProduceL1Output(l1input)

내부 단위: 거리=m, 시간=s, 각도=rad, 속도=m/s
motorapp에서 전달받는 raw 센서 객체는 duck-typing으로 접근 (임포트 없음).
Anchor 타입은 sensor_types.py에서 임포트.
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from enum import Enum
from math import isfinite, nan, pi
from typing import Optional

from lib import config
from .sensor_types import GpsAnchor, ImuAnchor, BaroAnchor

logger = logging.getLogger(__name__)

EARTH_RADIUS_M = 6_371_000.0


# ── 유틸 ──────────────────────────────────────────────────────────────────────

def _wrap_pi(angle: float) -> float:
    """각도를 [-π, +π] 범위로 정규화."""
    return (angle + pi) % (2.0 * pi) - pi


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _ok(v) -> bool:
    """finite한 float이면 True."""
    try:
        return isfinite(float(v))
    except (TypeError, ValueError):
        return False


def _circular_mean(a: float, b: float) -> float:
    return math.atan2(math.sin(a) + math.sin(b), math.cos(a) + math.cos(b))


# ── 좌표 변환 ─────────────────────────────────────────────────────────────────

def latlon_to_ne(lat: float, lon: float,
                 origin_lat: float, origin_lon: float) -> tuple:
    """(lat, lon) → (N, E) 미터, origin 기준."""
    dLat = math.radians(lat - origin_lat)
    dLon = math.radians(lon - origin_lon)
    N = dLat * EARTH_RADIUS_M
    E = dLon * EARTH_RADIUS_M * math.cos(math.radians(origin_lat))
    return N, E


# ── Enum ──────────────────────────────────────────────────────────────────────

class ControlMode(str, Enum):
    GPS_TRACKING_CLOSED = "GPS_TRACKING_CLOSED"   # GPS pos+vel + IMU gyrz
    GPS_TRACKING_OPEN   = "GPS_TRACKING_OPEN"     # GPS pos+vel, IMU gyrz 없음
    DR_TRACKING_CLOSED  = "DR_TRACKING_CLOSED"    # DR + IMU gyrz
    DR_TRACKING_OPEN    = "DR_TRACKING_OPEN"      # DR + IMU yaw (gyrz 없음)
    DETUMBLING          = "DETUMBLING"
    FAIL                = "FAIL"


class DRMethod(str, Enum):
    NONE             = "NONE"
    GYRO_INTEGRATION = "GYRO_INTEGRATION"
    GYRO_ACC_BLEND   = "GYRO_ACC_BLEND"


# ── MissionFrame (비행 중 1회 설정) ──────────────────────────────────────────

@dataclass
class MissionFrame:
    """비행 중 1회 설정되는 임무 상수. reset()으로 초기화."""
    origin_lat:   float = nan
    origin_lon:   float = nan
    origin_ready: bool  = False
    target_E:     float = nan   # origin 기준 투영 완료 좌표
    target_N:     float = nan
    target_ready: bool  = False
    # 임시 보관용 (투영 전 raw 값, 투영 후에도 삭제하지 않음)
    _target_lat:  float = nan
    _target_lon:  float = nan
    _raw_lat:     float = nan   # 마지막으로 수신한 GPS lat (origin 획득용)
    _raw_lon:     float = nan


# ── DRState (GPS dropout 이후 적분 상태) ─────────────────────────────────────

@dataclass
class DRState:
    """GPS 앵커 + IMU 적분 상태. 메서드 없는 순수 데이터."""

    # ── 앵커 (GPS_TRACKING 사이클마다 덮어씀) ──────────────────────────────
    anchor_E:      float = nan
    anchor_N:      float = nan
    anchor_V:      float = nan    # 속도 (m/s)
    anchor_course: float = nan    # 진행방향 (rad)
    anchor_time:   float = nan    # monotonic 타임스탬프

    # ── IMU 적분 (앵커 잠금 이후 누적, 앵커와 수명 동일) ─────────────────
    yaw_at_anchor:  float = nan   # 앵커 잠금 시점 IMU yaw (delta 계산용)
    gyro_integral:  float = 0.0   # 앵커 이후 누적 yaw 변화 (rad)
    last_step_time: float = nan   # 직전 DR 스텝 시각 (dt 계산용)

    # ── DR 결과 메타 (매 사이클 갱신) ────────────────────────────────────
    method:     DRMethod = DRMethod.NONE
    confidence: float    = 0.0


# ── GuidanceState (매 사이클 갱신) ───────────────────────────────────────────

@dataclass
class NavState:
    """Current navigation estimate."""

    E:            float       = nan
    N:            float       = nan
    course:       float       = nan
    V:            float       = nan
    confidence:   float       = 0.0
    control_mode: ControlMode = ControlMode.FAIL


@dataclass
class GuidanceState:
    """항법 추정값 + 센서 앵커 + DR 서브-상태."""

    # ── 센서 앵커 (마지막 신선값 1개) ─────────────────────────────────────
    gps:  GpsAnchor  = field(default_factory=GpsAnchor)
    imu:  ImuAnchor  = field(default_factory=ImuAnchor)
    baro: BaroAnchor = field(default_factory=BaroAnchor)

    # ── 현재 nav 추정값 (GPS 또는 DR 결과) ───────────────────────────────
    nav: NavState = field(default_factory=NavState)

    # ── DR 서브-상태 ──────────────────────────────────────────────────────
    dr: DRState = field(default_factory=DRState)

    # ── Detumbling 히스테리시스 타이머 ────────────────────────────────────
    detumble_exit_start: float = nan


# ── L1Input / L1Output ────────────────────────────────────────────────────────

@dataclass
class L1Input:
    valid:        bool        = False
    reason:       str         = "INIT"
    control_mode: ControlMode = ControlMode.FAIL
    dr_method:    DRMethod    = DRMethod.NONE
    confidence:   float       = 0.0
    E:            float       = nan   # 현재 위치 (local NE, m)
    N:            float       = nan
    V:            float       = nan   # 현재 속도 (m/s)
    course:       float       = nan   # 현재 진행방향 (rad)
    target_E:     float       = nan
    target_N:     float       = nan


@dataclass
class L1Output:
    timestamp:                  float       = 0.0   # monotonic, _ctrl_cycle에서 스탬프
    control_valid:              bool        = False
    nominal:                    bool        = False
    reason:                     str         = "INIT"
    control_mode:               ControlMode = ControlMode.FAIL
    dr_method:                  DRMethod    = DRMethod.NONE
    confidence:                 float       = 0.0
    yaw_rate_cmd:               float       = 0.0   # rad/s
    yaw_rate_limit_dps:         float       = 0.0
    nu:                         float       = nan   # cross-track angle (rad)
    target_bearing:             float       = nan
    distance_to_target:         float       = nan
    pos_E:                      float       = nan
    pos_N:                      float       = nan
    target_E:                   float       = nan
    target_N:                   float       = nan
    ground_speed_mps:           float       = 0.0
    pid_enabled:                bool        = False


# ── 모듈 전역 ─────────────────────────────────────────────────────────────────
_MISSION_t = MissionFrame()   # 비행 1회 설정
_STATE_t   = GuidanceState()  # 매 사이클 갱신


# ── DR 독립 함수 ──────────────────────────────────────────────────────────────

def dr_is_valid(dr: DRState) -> bool:
    """앵커가 완전히 잠겨있으면 True."""
    return (
        isfinite(dr.anchor_E) and isfinite(dr.anchor_N)
        and isfinite(dr.anchor_V) and isfinite(dr.anchor_course)
        and isfinite(dr.anchor_time)
    )


def dr_lock(dr: DRState, E: float, N: float, V: float,
            course: float, yaw: float, now: float) -> None:
    """GPS 신선 사이클마다 호출. DR 앵커 잠금 + 적분 리셋.

    Args:
        yaw: IMU yaw (rad). yaw_valid=False이면 nan 전달.
    """
    dr.anchor_E       = E
    dr.anchor_N       = N
    dr.anchor_V       = V
    dr.anchor_course  = course
    dr.anchor_time    = now
    dr.yaw_at_anchor  = yaw    # nan이어도 저장 (yaw_valid=False 케이스)
    dr.gyro_integral  = 0.0    # ← 반드시 리셋
    dr.last_step_time = nan
    dr.method         = DRMethod.NONE
    dr.confidence     = 1.0


def dr_reset(dr: DRState) -> None:
    """비행 리셋 시 전체 초기화."""
    dr.anchor_E       = nan
    dr.anchor_N       = nan
    dr.anchor_V       = nan
    dr.anchor_course  = nan
    dr.anchor_time    = nan
    dr.yaw_at_anchor  = nan
    dr.gyro_integral  = 0.0
    dr.last_step_time = nan
    dr.method         = DRMethod.NONE
    dr.confidence     = 0.0


def dr_estimate_course(dr: DRState, imu: ImuAnchor) -> float:
    """현재 heading 추정 (gyro 적분 + yaw delta 블렌드).

    IMU 소스가 모두 없으면 dr.anchor_course 반환 (마지막 알던 방향 유지).
    """
    base = dr.anchor_course
    if not isfinite(base):
        return 0.0

    course_gyro = (
        _wrap_pi(base + dr.gyro_integral)
        if imu.gyrz_valid else None
    )
    course_yaw = (
        _wrap_pi(base + _wrap_pi(imu.yaw - dr.yaw_at_anchor))
        if (imu.yaw_valid and isfinite(dr.yaw_at_anchor)) else None
    )

    if course_gyro is not None and course_yaw is not None:
        if abs(_wrap_pi(course_yaw - course_gyro)) < pi / 4:
            return _circular_mean(course_yaw, course_gyro)
        return course_gyro   # 차이 크면 gyro 우선
    if course_gyro is not None:
        return course_gyro
    if course_yaw is not None:
        return course_yaw
    return base  # IMU 완전 stale → 마지막 알던 방향 유지


def _compute_dr_confidence(age: float) -> float:
    """Return DR confidence in [0, 1]; confidence scales guidance, not mode validity."""
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


# ── UpdateAnchors (매 사이클, 반환값 없음) ────────────────────────────────────

def UpdateAnchors(gps, imu, baro, now: float) -> None:
    """raw 센서 객체(duck-typed)에서 앵커를 갱신한다.

    Args:
        gps  : _GpsFromApp 또는 동일 attribute를 가진 객체
        imu  : _ImuFromApp 또는 동일 attribute를 가진 객체
        baro : _BaroFromApp 또는 동일 attribute를 가진 객체
    """
    st_t = _STATE_t
    mi_t = _MISSION_t

    # ── GPS ──────────────────────────────────────────────────────────────────
    pos_health    = getattr(gps, "pos_health",    0)
    motion_health = getattr(gps, "motion_health", 0)

    if pos_health:
        lat    = getattr(gps, "lat",    None)
        lon    = getattr(gps, "lon",    None)
        pos_ts = getattr(gps, "pos_ts", None)
        if lat is not None and lon is not None and _ok(lat) and _ok(lon) and _ok(pos_ts):
            # 항상 raw lat/lon 저장 (origin 획득용)
            mi_t._raw_lat = float(lat)
            mi_t._raw_lon = float(lon)

            if mi_t.origin_ready:
                # origin 확보된 경우만 E/N 투영
                N, E = latlon_to_ne(float(lat), float(lon),
                                    mi_t.origin_lat, mi_t.origin_lon)
                st_t.gps.E = E
                st_t.gps.N = N

            st_t.gps.pos_ts    = float(pos_ts)
            st_t.gps.pos_valid = True

    if motion_health:
        course_rad = getattr(gps, "course_rad", None)
        speed_mps  = getattr(gps, "speed_mps",  None)
        motion_ts  = getattr(gps, "motion_ts",  None)
        if _ok(course_rad) and _ok(speed_mps) and _ok(motion_ts):
            st_t.gps.V            = float(speed_mps)
            st_t.gps.course       = float(course_rad)
            st_t.gps.motion_ts    = float(motion_ts)
            st_t.gps.motion_valid = True

    # ── IMU ──────────────────────────────────────────────────────────────────
    imu_health = getattr(imu, "health", 0)
    imu_ts     = getattr(imu, "ts",     None)
    if imu_health and _ok(imu_ts):
        yaw_r  = getattr(imu, "yaw_rad",    None)
        gyrz   = getattr(imu, "gyrz_rad_s", None)
        lax    = getattr(imu, "lin_acc_x",  None)
        lay    = getattr(imu, "lin_acc_y",  None)
        lin_ok = bool(getattr(imu, "lin_acc_valid", False))

        st_t.imu.ts = float(imu_ts)
        if _ok(yaw_r):
            st_t.imu.yaw       = float(yaw_r)
            st_t.imu.yaw_valid = True
        if _ok(gyrz):
            st_t.imu.gyr_z      = float(gyrz)
            st_t.imu.gyrz_valid = True
        if lin_ok and _ok(lax) and _ok(lay):
            st_t.imu.lin_acc_x     = float(lax)
            st_t.imu.lin_acc_y     = float(lay)
            st_t.imu.lin_acc_valid = True

    # ── Barometer ────────────────────────────────────────────────────────────
    baro_health = getattr(baro, "health", 0)
    baro_ts     = getattr(baro, "rx_ts",  None)
    alt_m       = getattr(baro, "alt_m",  None)
    if baro_health and _ok(baro_ts) and _ok(alt_m):
        st_t.baro.alt_m = float(alt_m)
        st_t.baro.ts    = float(baro_ts)
        st_t.baro.valid = True
        sink = getattr(baro, "sink_rate", None)
        if _ok(sink):
            st_t.baro.sink_rate = float(sink)


# ── DecideControlMode (매 사이클, freshness inline 계산) ─────────────────────

def DecideControlMode(now: float) -> ControlMode:
    """_STATE_t 기반으로 ControlMode를 결정하고 st_t.nav.control_mode에 저장."""
    st_t = _STATE_t

    pos_fresh  = (st_t.gps.pos_valid
                  and isfinite(st_t.gps.pos_ts)
                  and (now - st_t.gps.pos_ts) <= config.GPS_FRESH_MAX_AGE_S)
    vel_fresh  = (st_t.gps.motion_valid
                  and isfinite(st_t.gps.motion_ts)
                  and (now - st_t.gps.motion_ts) <= config.GPS_FRESH_MAX_AGE_S)
    imu_fresh  = (isfinite(st_t.imu.ts)
                  and (now - st_t.imu.ts) <= config.IMU_FRESH_MAX_AGE_S)
    gyrz_fresh = imu_fresh and st_t.imu.gyrz_valid
    yaw_fresh  = imu_fresh and st_t.imu.yaw_valid

    # 1. DETUMBLING (최우선)
    if config.DETUMBLE_ENABLE and gyrz_fresh:
        gyrz_dps  = abs(math.degrees(st_t.imu.gyr_z))
        currently = (st_t.nav.control_mode == ControlMode.DETUMBLING)
        if currently:
            if gyrz_dps <= config.DETUMBLE_EXIT_THRESHOLD_DPS:
                if not isfinite(st_t.detumble_exit_start):
                    st_t.detumble_exit_start = now
                elif now - st_t.detumble_exit_start >= config.DETUMBLE_EXIT_HOLD_S:
                    # 탈출 조건 충족 → 히스테리시스 타이머 초기화 후 아래 진행
                    st_t.detumble_exit_start = nan
                    # fallthrough to GPS/DR checks below
                else:
                    st_t.nav.control_mode = ControlMode.DETUMBLING
                    return ControlMode.DETUMBLING
            else:
                st_t.detumble_exit_start = nan
                st_t.nav.control_mode = ControlMode.DETUMBLING
                return ControlMode.DETUMBLING
        else:
            if gyrz_dps >= config.DETUMBLE_GYRZ_THRESHOLD_DPS:
                st_t.detumble_exit_start = nan
                st_t.nav.control_mode = ControlMode.DETUMBLING
                return ControlMode.DETUMBLING

    # 2. GPS_TRACKING_CLOSED
    if pos_fresh and vel_fresh and gyrz_fresh:
        st_t.nav.control_mode = ControlMode.GPS_TRACKING_CLOSED
        return ControlMode.GPS_TRACKING_CLOSED

    # 3. GPS_TRACKING_OPEN
    if pos_fresh and vel_fresh:
        st_t.nav.control_mode = ControlMode.GPS_TRACKING_OPEN
        return ControlMode.GPS_TRACKING_OPEN

    # 4. DR_TRACKING_CLOSED
    if dr_is_valid(st_t.dr) and gyrz_fresh:
        st_t.nav.control_mode = ControlMode.DR_TRACKING_CLOSED
        return ControlMode.DR_TRACKING_CLOSED

    # 5. DR_TRACKING_OPEN
    if dr_is_valid(st_t.dr) and yaw_fresh:
        st_t.nav.control_mode = ControlMode.DR_TRACKING_OPEN
        return ControlMode.DR_TRACKING_OPEN

    # 6. FAIL
    st_t.nav.control_mode = ControlMode.FAIL
    return ControlMode.FAIL


# ── DR 위치 추정 (내부) ───────────────────────────────────────────────────────

def _update_state_from_dead_reckoning(now: float) -> None:
    """_STATE_t.dr 앵커를 기반으로 DR 적분을 수행하고 nav 상태를 갱신."""
    st_t = _STATE_t
    dr = st_t.dr

    # dt 계산
    if isfinite(dr.last_step_time):
        dt = now - dr.last_step_time
    elif isfinite(dr.anchor_time):
        dt = now - dr.anchor_time   # 첫 DR 스텝
    else:
        dt = 0.0
    dt = _clamp(dt, 0.0, 0.5)

    # nav 위치 초기화 (첫 DR 스텝 또는 이전에 nan이면 앵커 위치로 초기화)
    if not isfinite(st_t.nav.E) or not isfinite(st_t.nav.N):
        st_t.nav.E = dr.anchor_E
        st_t.nav.N = dr.anchor_N

    imu_fresh = (isfinite(st_t.imu.ts)
                 and (now - st_t.imu.ts) <= config.IMU_FRESH_MAX_AGE_S)

    # ① gyro 적분
    if imu_fresh and st_t.imu.gyrz_valid:
        dr.gyro_integral += st_t.imu.gyr_z * config.GYRZ_SIGN * dt

    # ② heading 추정 (IMU stale 시 anchor_course 유지)
    course_est = dr_estimate_course(dr, st_t.imu)

    # ③ 속도 유지
    V_dr = dr.anchor_V
    V_dr = max(0.0, V_dr)

    # ④ EN 속도
    vE = V_dr * math.sin(course_est)
    vN = V_dr * math.cos(course_est)

    # ⑤ 가속도계 보정 (선택적)
    method = DRMethod.GYRO_INTEGRATION
    if (config.USE_ACC_DOUBLE_INTEGRATION
            and imu_fresh and st_t.imu.lin_acc_valid):
        lax = st_t.imu.lin_acc_x * config.ACC_X_SIGN
        lay = st_t.imu.lin_acc_y * config.ACC_Y_SIGN
        if math.hypot(lax, lay) <= config.ACC_LIMIT_MPS2:
            aE = lax * math.sin(course_est) + lay * math.cos(course_est)
            aN = lax * math.cos(course_est) - lay * math.sin(course_est)
            vE += config.ACC_BLEND_WEIGHT * aE * dt
            vN += config.ACC_BLEND_WEIGHT * aN * dt
            method = DRMethod.GYRO_ACC_BLEND

    # ⑥ 위치 갱신
    st_t.nav.E      += vE * dt
    st_t.nav.N      += vN * dt
    st_t.nav.V       = V_dr
    st_t.nav.course  = course_est
    dr.method      = method
    dr.last_step_time = now

    # ⑦ 신뢰도: DR은 유지하고 L1 yaw-rate 명령만 시간에 따라 약화한다.
    confidence = _compute_dr_confidence(now - dr.anchor_time)
    dr.confidence = confidence
    st_t.nav.confidence = confidence


# ── ProduceL1Input ────────────────────────────────────────────────────────────

def ProduceL1Input(now: float) -> L1Input:
    """_STATE_t + _MISSION으로 L1Input을 생성.

    이 함수는 내부 nav 상태를 갱신하는 부수 효과가 있다.
    """
    st_t = _STATE_t
    mi_t = _MISSION_t

    # ── origin 획득 ──────────────────────────────────────────────────────────
    if not mi_t.origin_ready:
        pos_fresh = (st_t.gps.pos_valid
                     and isfinite(st_t.gps.pos_ts)
                     and (now - st_t.gps.pos_ts) <= config.GPS_FRESH_MAX_AGE_S)
        if pos_fresh and _ok(mi_t._raw_lat) and _ok(mi_t._raw_lon):
            mi_t.origin_lat   = mi_t._raw_lat
            mi_t.origin_lon   = mi_t._raw_lon
            mi_t.origin_ready = True
            # origin 자체 위치는 E=0, N=0
            st_t.gps.E = 0.0
            st_t.gps.N = 0.0
            logger.info("Origin set: lat=%.6f lon=%.6f", mi_t.origin_lat, mi_t.origin_lon)
            # target 재투영
            if _ok(mi_t._target_lat) and _ok(mi_t._target_lon):
                tN, tE = latlon_to_ne(mi_t._target_lat, mi_t._target_lon,
                                      mi_t.origin_lat, mi_t.origin_lon)
                mi_t.target_E = tE
                mi_t.target_N = tN
                mi_t.target_ready = True
                logger.info("Target projected: E=%.1f N=%.1f", tE, tN)

    if not mi_t.origin_ready:
        return L1Input(valid=False, reason="NO_ORIGIN")
    if not mi_t.target_ready:
        return L1Input(valid=False, reason="NO_TARGET")

    mode = st_t.nav.control_mode

    # ── DETUMBLING ────────────────────────────────────────────────────────────
    if mode == ControlMode.DETUMBLING:
        return L1Input(
            valid=True, reason="DETUMBLING",
            control_mode=ControlMode.DETUMBLING, dr_method=DRMethod.NONE,
            confidence=1.0,
            E=st_t.nav.E, N=st_t.nav.N, V=st_t.nav.V, course=st_t.nav.course,
            target_E=mi_t.target_E, target_N=mi_t.target_N,
        )

    # ── GPS_TRACKING (pos+vel 신선) ───────────────────────────────────────────
    if mode in (ControlMode.GPS_TRACKING_CLOSED, ControlMode.GPS_TRACKING_OPEN):
        if not (isfinite(st_t.gps.E) and isfinite(st_t.gps.N)
                and isfinite(st_t.gps.V) and isfinite(st_t.gps.course)):
            return L1Input(valid=False, reason="GPS_NAN")

        st_t.nav.E          = st_t.gps.E
        st_t.nav.N          = st_t.gps.N
        st_t.nav.course     = st_t.gps.course
        st_t.nav.V          = st_t.gps.V
        st_t.nav.confidence = 1.0

        # DR 앵커 갱신
        imu_yaw = st_t.imu.yaw if st_t.imu.yaw_valid else nan
        dr_lock(st_t.dr, st_t.gps.E, st_t.gps.N, st_t.gps.V, st_t.gps.course, imu_yaw, now)

        return L1Input(
            valid=True, reason="GPS_TRACKING",
            control_mode=mode, dr_method=DRMethod.NONE, confidence=1.0,
            E=st_t.nav.E, N=st_t.nav.N, V=st_t.nav.V, course=st_t.nav.course,
            target_E=mi_t.target_E, target_N=mi_t.target_N,
        )

    # ── GPS POS-ONLY → DR 앵커 초기화 시도 (vel stale, 앵커 없을 때만) ───────
    pos_fresh = (st_t.gps.pos_valid
                 and isfinite(st_t.gps.pos_ts)
                 and (now - st_t.gps.pos_ts) <= config.GPS_FRESH_MAX_AGE_S)
    imu_fresh = (isfinite(st_t.imu.ts)
                 and (now - st_t.imu.ts) <= config.IMU_FRESH_MAX_AGE_S)

    if pos_fresh and not dr_is_valid(st_t.dr):
        if imu_fresh and (st_t.imu.yaw_valid or st_t.imu.gyrz_valid):
            course0 = st_t.imu.yaw if st_t.imu.yaw_valid else st_t.dr.anchor_course
            imu_yaw = st_t.imu.yaw if st_t.imu.yaw_valid else nan
            if isfinite(st_t.gps.E) and isfinite(st_t.gps.N) and isfinite(course0):
                dr_lock(st_t.dr, st_t.gps.E, st_t.gps.N, 0.0, course0, imu_yaw, now)
                logger.info(
                    "DR anchor init (pos-only): E=%.1f N=%.1f course=%.1f°",
                    st_t.gps.E, st_t.gps.N, math.degrees(course0),
                )
        else:
            # IMU도 stale → heading 추정 불가
            return L1Input(valid=False, reason="NO_HEADING_SOURCE")

    # ── DR_TRACKING ───────────────────────────────────────────────────────────
    if mode in (ControlMode.DR_TRACKING_CLOSED, ControlMode.DR_TRACKING_OPEN):
        if not dr_is_valid(st_t.dr):
            return L1Input(valid=False, reason="NO_DR_ANCHOR")

        _update_state_from_dead_reckoning(now)

        return L1Input(
            valid=True, reason="DR_TRACKING",
            control_mode=mode, dr_method=st_t.dr.method,
            confidence=st_t.dr.confidence,
            E=st_t.nav.E, N=st_t.nav.N, V=st_t.nav.V, course=st_t.nav.course,
            target_E=mi_t.target_E, target_N=mi_t.target_N,
        )

    # ── FAIL ─────────────────────────────────────────────────────────────────
    return L1Input(valid=False, reason="FAIL")


# ── ProduceL1Output (pure 계산) ───────────────────────────────────────────────

def _choose_yaw_rate_limit(mode: ControlMode) -> float:
    """ControlMode에 따른 yaw rate 한계 반환 (rad/s)."""
    if mode == ControlMode.GPS_TRACKING_CLOSED:
        return math.radians(config.GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS)
    if mode == ControlMode.GPS_TRACKING_OPEN:
        return math.radians(config.GPS_TRACKING_OPEN_YAW_RATE_LIMIT_DPS)
    if mode == ControlMode.DR_TRACKING_CLOSED:
        return math.radians(config.DR_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS)
    if mode == ControlMode.DR_TRACKING_OPEN:
        return math.radians(config.DR_TRACKING_OPEN_YAW_RATE_LIMIT_DPS)
    if mode == ControlMode.DETUMBLING:
        return math.radians(config.DETUMBLING_YAW_RATE_LIMIT_DPS)
    return math.radians(config.FAIL_YAW_RATE_LIMIT_DPS)


def ProduceL1Output(l1in: L1Input) -> L1Output:
    """L1Input으로 yaw_rate_cmd를 계산한다. 상태 접촉 없음 (pure)."""
    output_t = L1Output(
        control_mode=l1in.control_mode,
        dr_method=l1in.dr_method,
        confidence=l1in.confidence,
        pos_E=l1in.E, pos_N=l1in.N,
        target_E=l1in.target_E, target_N=l1in.target_N,
        ground_speed_mps=l1in.V if _ok(l1in.V) else 0.0,
    )

    # ── Invalid / FAIL ────────────────────────────────────────────────────────
    if not l1in.valid:
        output_t.control_valid = False
        output_t.nominal       = False
        output_t.reason        = l1in.reason
        return output_t

    # ── DETUMBLING ────────────────────────────────────────────────────────────
    if l1in.control_mode == ControlMode.DETUMBLING:
        lim = _choose_yaw_rate_limit(ControlMode.DETUMBLING)
        output_t.control_valid      = True
        output_t.nominal            = False
        output_t.reason             = "DETUMBLING"
        output_t.pid_enabled        = False
        output_t.yaw_rate_limit_dps = math.degrees(lim)
        return output_t

    # ── NaN 검사 ─────────────────────────────────────────────────────────────
    for v in (l1in.E, l1in.N, l1in.target_E, l1in.target_N, l1in.course, l1in.V):
        if not _ok(v):
            output_t.control_valid = False
            output_t.reason        = "NAN_NAV_STATE"
            return output_t

    dE   = l1in.target_E - l1in.E
    dN   = l1in.target_N - l1in.N
    dist = math.hypot(dE, dN)
    output_t.distance_to_target = dist

    #need correction
    # ── 목표 도달 ─────────────────────────────────────────────────────────────
    if dist <= config.TARGET_RADIUS_M:
        output_t.control_valid  = True
        output_t.nominal        = True
        output_t.reason         = "TARGET_REACHED"
        output_t.target_bearing = _wrap_pi(math.atan2(dE, dN))
        output_t.nu             = 0.0
        return output_t

    # ── L1 계산 ───────────────────────────────────────────────────────────────
    target_bearing = _wrap_pi(math.atan2(dE, dN))   # North 기준
    nu             = _wrap_pi(target_bearing - l1in.course)

    # nu 데드밴드 (잔진동 방지)
    if abs(nu) < math.radians(config.NU_DEADBAND_DEG):
        sin_nu_eff = 0.0
    else:
        sin_nu_eff = math.sin(_clamp(nu, -pi / 2.0, pi / 2.0))

    # DR 모드일 때 속도 상한 완화 (포화 nu 억제)
    _v_max = (config.V_MAX_DR_MPS
              if l1in.control_mode in (ControlMode.DR_TRACKING_CLOSED,
                                       ControlMode.DR_TRACKING_OPEN)
              else config.V_MAX_MPS)
    V_eff = _clamp(l1in.V, config.V_MIN_MPS, _v_max)

    yaw_rate_cmd = 2.0 * V_eff / config.L_GAIN_M * sin_nu_eff
    yaw_rate_cmd *= l1in.confidence

    lim          = _choose_yaw_rate_limit(l1in.control_mode)
    yaw_rate_cmd = _clamp(yaw_rate_cmd, -lim, lim)

    output_t.target_bearing             = target_bearing
    output_t.nu                         = nu
    output_t.yaw_rate_cmd               = yaw_rate_cmd
    output_t.yaw_rate_limit_dps         = math.degrees(lim)
    output_t.control_valid              = True
    output_t.nominal                    = True
    output_t.reason                     = l1in.reason
    output_t.pid_enabled                = True

    logger.debug(
        "L1 mode=%s dist=%.1fm bear=%.1f° nu=%.1f° cmd=%.2f°/s conf=%.2f",
        l1in.control_mode.value, dist,
        math.degrees(target_bearing), math.degrees(nu),
        math.degrees(yaw_rate_cmd), l1in.confidence,
    )
    return output_t


# ── 공개 API ─────────────────────────────────────────────────────────────────

def set_target(lat: float, lon: float) -> None:
    """타겟 좌표 설정. origin이 이미 있으면 즉시 투영."""
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
    """비행 리셋: origin/nav/DR 전체 초기화. target lat/lon은 보존."""
    global _MISSION_t, _STATE_t
    saved_lat = _MISSION_t._target_lat
    saved_lon = _MISSION_t._target_lon
    _MISSION_t = MissionFrame()
    _MISSION_t._target_lat = saved_lat
    _MISSION_t._target_lon = saved_lon
    _STATE_t = GuidanceState()
    logger.info("guidance.reset(): origin/nav/DR cleared, target lat/lon preserved")
