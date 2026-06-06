"""Sensor_Motor/control.py — yaw-rate 제어 + 서보 믹서

Pipeline:
  ProduceCtrlInput(l1_output, now)
    → step(ctrl_input, gyrz_meas_deg_s, now)
    → MoveServo(pi, ctrl_output)

Arm 각도 규약:
  0 deg   = arm up (완전히 올림)
  80 deg  = neutral (패러포일 정상 비행)
  160 deg = full brake/down
  positive delta_arm_deg = 오른쪽 회전 명령
    (right arm angle ↑, left arm angle ↓)
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Optional

from lib import config
from .guidance import ControlMode

logger = logging.getLogger(__name__)


# ── GPIO + 서보 캘리브레이션 (config에서 읽음) ────────────────────────────────
PARAFOIL_LEFT_MOTOR_PIN  = config.PARAFOIL_LEFT_GPIO
PARAFOIL_RIGHT_MOTOR_PIN = config.PARAFOIL_RIGHT_GPIO

ARM_MIN_DEG       = config.ARM_MIN_DEG
ARM_MAX_DEG       = config.ARM_MAX_DEG
NEUTRAL_ARM_DEG   = config.NEUTRAL_ARM_DEG
DELTA_ARM_MAX_DEG = 2.0 * min(
    NEUTRAL_ARM_DEG - ARM_MIN_DEG,
    ARM_MAX_DEG - NEUTRAL_ARM_DEG,
)

LEFT_ZERO     = config.LEFT_SERVO_ZERO_US
RIGHT_ZERO    = config.RIGHT_SERVO_ZERO_US
PULSE_PER_DEG = config.SERVO_PULSE_PER_DEG

LEFT_NEUTRAL     = int(LEFT_ZERO  - NEUTRAL_ARM_DEG * PULSE_PER_DEG)
RIGHT_NEUTRAL    = int(RIGHT_ZERO + NEUTRAL_ARM_DEG * PULSE_PER_DEG)
LEFT_ZERO_PULSE  = int(LEFT_ZERO)
RIGHT_ZERO_PULSE = int(RIGHT_ZERO)
LEFT_MIN_PULSE   = int(LEFT_ZERO  - ARM_MAX_DEG * PULSE_PER_DEG)
LEFT_MAX_PULSE   = int(LEFT_ZERO  - ARM_MIN_DEG * PULSE_PER_DEG)
RIGHT_MIN_PULSE  = int(RIGHT_ZERO + ARM_MIN_DEG * PULSE_PER_DEG)
RIGHT_MAX_PULSE  = int(RIGHT_ZERO + ARM_MAX_DEG * PULSE_PER_DEG)

GYRO_SPIKE_LIMIT_DEG_S = config.GYRO_SPIKE_LIMIT_DEG_S
INTEGRAL_DECAY_RATE    = config.INTEGRAL_DECAY_RATE
MAX_ARM_RATE_DEG_S     = 60.0


# ── 내부 유틸 ─────────────────────────────────────────────────────────────────

def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _clamp_dt(now: float, previous: float, default: float,
              lo: float, hi: float) -> float:
    """이산 제어 업데이트용 유효 dt 반환."""
    try:
        prev = float(previous)
    except (TypeError, ValueError):
        return default
    if prev <= 0.0 or not math.isfinite(prev):
        return default
    dt = max(0.0, now - prev)
    return max(lo, min(hi, dt)) if math.isfinite(dt) else default


def _as_control_mode(value) -> ControlMode:
    if isinstance(value, ControlMode):
        return value
    raw = getattr(value, "value", value)
    try:
        return ControlMode(str(raw))
    except (TypeError, ValueError):
        return ControlMode.FAIL


# ── ControlMode 문자열 호환 헬퍼 ──────────────────────────────────────────────
# Enum / 문자열 어느 쪽이 와도 안전하게 분류한다.

def _mode_str(mode) -> str:
    if hasattr(mode, "value"):
        return mode.value
    return str(mode)


def _is_fail_mode(mode) -> bool:
    return _mode_str(mode) == "FAIL"


def _is_gps_closed(mode) -> bool:
    return _mode_str(mode) == "GPS_TRACKING_CLOSED"


def _is_gps_open(mode) -> bool:
    return _mode_str(mode) == "GPS_TRACKING_OPEN"


def _is_dr_m(mode) -> bool:
    return _mode_str(mode).startswith("DR_M_")


def _is_dr_pm(mode) -> bool:
    return _mode_str(mode).startswith("DR_PM_")


def _is_dr(mode) -> bool:
    return _is_dr_m(mode) or _is_dr_pm(mode)


def _select_pid_gains(mode, kp_override):
    """모드별 (kp, ki, kd) 반환. kp_override가 있으면 kp만 대체.

    GPS_CLOSED → KP_GPS_CLOSED, DR_M_*_CLOSED → KP_DR_M_CLOSED,
    DR_PM_*_CLOSED → KP_DR_PM_CLOSED. PID는 *_CLOSED 모드에서만 활성이므로
    open 모드에서 호출돼도 결과는 사용되지 않는다.
    """
    if _is_dr_pm(mode):
        kp = config.KP_DR_PM_CLOSED
        ki = getattr(config, "KI_DR_PM_CLOSED", 0.0)
        kd = getattr(config, "KD_DR_PM_CLOSED", 0.0)
    elif _is_dr_m(mode):
        kp = config.KP_DR_M_CLOSED
        ki = getattr(config, "KI_DR_M_CLOSED", 0.0)
        kd = getattr(config, "KD_DR_M_CLOSED", 0.0)
    else:  # GPS_CLOSED 및 fallback
        kp = config.KP_GPS_CLOSED
        ki = getattr(config, "KI_GPS_CLOSED", config.CTRL_K_I)
        kd = getattr(config, "KD_GPS_CLOSED", config.KD_YAW_RATE)
    if kp_override is not None:
        kp = float(kp_override)
    return kp, ki, kd


def _ff_scale_for_mode(mode) -> float:
    """피드포워드 권한 스케일. GPS=1.0, DR_M=0.8, DR_PM=0.6, 그 외=0.0."""
    if _is_gps_closed(mode) or _is_gps_open(mode):
        return 1.0
    if _is_dr_pm(mode):
        return getattr(config, "DR_PM_FF_SCALE", 0.6)
    if _is_dr_m(mode):
        return getattr(config, "DR_M_FF_SCALE", 0.8)
    return 0.0


# ── Dataclass 정의 ────────────────────────────────────────────────────────────

@dataclass
class CtrlInput:
    angular_velocity_cmd_deg_s: float = 0.0
    ground_speed_mps:           float = 0.0
    valid:                      bool  = False
    timestamp:                  float = 0.0
    pid_enabled:                bool  = True
    control_mode:  ControlMode     = ControlMode.FAIL
    dr_method:     Optional[str]   = None
    kp_override:   Optional[float] = None
    # guidance가 mode별로 고른 yaw-rate limit(deg/s). FF 정규화·명령 클램핑의 기준.
    # 기본값은 GPS_CLOSED limit(안전 fallback).
    yaw_rate_limit_dps: float = config.GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS


@dataclass
class CtrlOutput:
    timestamp:    float = 0.0
    left_pw:      int   = LEFT_NEUTRAL
    right_pw:     int   = RIGHT_NEUTRAL
    left_angle_deg:  float = NEUTRAL_ARM_DEG
    right_angle_deg: float = NEUTRAL_ARM_DEG
    delta_arm_deg:   float = 0.0
    delta_ff_deg:    float = 0.0
    delta_pid_deg:   float = 0.0
    delta_total_deg: float = 0.0          # d_ff + d_pid, 합산 클램핑 전 (디버그)
    angular_velocity_cmd_deg_s:   float = 0.0
    angular_velocity_meas_deg_s:  float = float("nan")
    angular_velocity_error_deg_s: float = 0.0
    motor_cmd:     float = 0.0
    saturated:     bool  = False
    sensor_valid:  bool  = False
    gyro_rejected: bool  = False
    valid:         bool  = False
    control_mode:  ControlMode = ControlMode.FAIL
    # 진단/단위 추적용 (모두 deg 또는 deg/s 단위; 무차원 스케일 제외)
    kp_used:       float = 0.0
    ff_scale:      float = 1.0
    # 이 사이클에 실제로 적용된 mode별 yaw-rate limit(deg/s). 디버그/로그용.
    yaw_rate_limit_dps: float = config.GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS
    # FF 데드밴드/정규화 기준(이번 사이클 적용값) + DR PID 활성 여부 + cap 전 FF. 로그용.
    ff_deadband_dps: float = config.CTRL_ANGULAR_VELOCITY_DEADBAND_DEG_S
    ff_ref_dps:      float = config.GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS
    dr_pid_active:   bool  = False
    delta_ff_pre_cap_deg: float = 0.0
    reason:        str   = "INIT"


# ── 모듈 레벨 제어 상태 ───────────────────────────────────────────────────────

_integral_deg:         float = 0.0
_prev_error_deg:       float = 0.0
_prev_time:            float = 0.0
_prev_left_angle_deg:  float = NEUTRAL_ARM_DEG
_prev_right_angle_deg: float = NEUTRAL_ARM_DEG


def reset() -> None:
    global _integral_deg, _prev_error_deg, _prev_time
    global _prev_left_angle_deg, _prev_right_angle_deg
    _integral_deg         = 0.0
    _prev_error_deg       = 0.0
    _prev_time            = 0.0
    _prev_left_angle_deg  = NEUTRAL_ARM_DEG
    _prev_right_angle_deg = NEUTRAL_ARM_DEG


def WriteNeutral(now: float, control_mode: ControlMode = ControlMode.FAIL) -> CtrlOutput:
    """중립 PWM CtrlOutput 생성. 하드웨어 접촉 없음."""
    return CtrlOutput(timestamp=now, control_mode=_as_control_mode(control_mode))


# ── 입력 변환 (guidance → control 단위 변환) ─────────────────────────────────

def _resolve_yaw_rate_limit_dps(limit_dps) -> float:
    """유효한 yaw-rate limit(deg/s)을 반환한다.

    값이 없거나(None) NaN/0 이하이면 GPS_CLOSED limit을 안전 기본값으로 쓴다.
    이로써 mode별 limit이 비정상이어도 FF 정규화·클램핑이 항상 유효한 분모를 갖는다.
    """
    try:
        v = float(limit_dps)
    except (TypeError, ValueError):
        return config.GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS
    if not math.isfinite(v) or v <= 0.0:
        return config.GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS
    return v


def ProduceCtrlInput(l1_output, now: float) -> CtrlInput:
    """L1Output(rad/s) → CtrlInput(deg/s) 변환."""
    is_valid = bool(
        getattr(l1_output, "control_valid", False)
        or getattr(l1_output, "nominal", False)
    )
    rad = getattr(l1_output, "yaw_rate_cmd", 0.0)
    rad = float(rad or 0.0)

    # guidance가 고른 mode별 limit을 그대로 전달한다(없거나 NaN이면 GPS_CLOSED fallback).
    limit_dps = _resolve_yaw_rate_limit_dps(getattr(l1_output, "yaw_rate_limit_dps", None))

    return CtrlInput(
        angular_velocity_cmd_deg_s=math.degrees(rad),
        ground_speed_mps=float(getattr(l1_output, "ground_speed_mps", 0.0) or 0.0),
        valid=is_valid,
        timestamp=float(getattr(l1_output, "timestamp", now) or now),
        pid_enabled=bool(getattr(l1_output, "pid_enabled", True)),
        control_mode=_as_control_mode(getattr(l1_output, "control_mode", ControlMode.FAIL)),
        dr_method=getattr(l1_output, "dr_method", None),
        kp_override=getattr(l1_output, "kp_override", None),
        yaw_rate_limit_dps=limit_dps,
    )


# ── 피드포워드 형상 + 믹서 ────────────────────────────────────────────────────

def angular_velocity_to_delta_ff(
    cmd_dps: float,
    limit_dps: Optional[float] = None,
    deadband_dps: Optional[float] = None,
    ref_dps: Optional[float] = None,
) -> float:
    """yaw-rate 명령(deg/s) → 차동 arm 각도(deg).

    Expo 곡선 + 데드밴드 + 부호 보존 (양수=오른쪽 회전).

    limit_dps: 명령 클램핑 한계(deg/s). None/NaN/0 이하이면 GPS_CLOSED limit fallback.
    deadband_dps: FF 데드밴드(deg/s). None이면 config.CTRL_ANGULAR_VELOCITY_DEADBAND_DEG_S.
        DR은 더 작은 데드밴드를 받아 작은 yaw_rate_cmd도 FF로 살린다.
    ref_dps: expo 곡선 정규화 분모(deg/s). None이면 limit_dps(기존 동작).
        clamp limit과 분리한 "감도" 노브 — DR은 limit(50 등)보다 작은 ref로 정규화해
        작은 명령에도 nu 비례 응답이 살아나게 한다. x는 [0,1]로 cap한다.
    """
    lim = _resolve_yaw_rate_limit_dps(limit_dps)
    deadband = (config.CTRL_ANGULAR_VELOCITY_DEADBAND_DEG_S
                if deadband_dps is None else float(deadband_dps))
    ref = lim if ref_dps is None else _resolve_yaw_rate_limit_dps(ref_dps)
    clamped = _clamp(cmd_dps, -lim, lim)
    if abs(clamped) < deadband:
        return 0.0
    x = min(abs(clamped) / ref, 1.0)
    delta = (config.CTRL_DELTA_MIN_EFFECTIVE_DEG
             + (DELTA_ARM_MAX_DEG - config.CTRL_DELTA_MIN_EFFECTIVE_DEG) * (x ** config.CTRL_EXPO))
    return math.copysign(delta, clamped)


def ConnectRoMo(delta_arm_deg: float):
    """차동 편향각 → arm 각도 + PWM 펄스.

    양수 delta → 오른쪽 회전 (right arm ↑, left arm ↓).
    Returns: (left_pw, right_pw, left_angle_deg, right_angle_deg, delta_arm_deg)
    """
    delta       = _clamp(delta_arm_deg, -DELTA_ARM_MAX_DEG, DELTA_ARM_MAX_DEG)
    left_angle  = _clamp(NEUTRAL_ARM_DEG - delta / 2.0, ARM_MIN_DEG, ARM_MAX_DEG)
    right_angle = _clamp(NEUTRAL_ARM_DEG + delta / 2.0, ARM_MIN_DEG, ARM_MAX_DEG)
    left_pw  = int(_clamp(LEFT_ZERO  - left_angle  * PULSE_PER_DEG,
                          LEFT_MIN_PULSE,  LEFT_MAX_PULSE))
    right_pw = int(_clamp(RIGHT_ZERO + right_angle * PULSE_PER_DEG,
                          RIGHT_MIN_PULSE, RIGHT_MAX_PULSE))
    return left_pw, right_pw, left_angle, right_angle, delta


# ── 메인 컨트롤러 스텝 ────────────────────────────────────────────────────────

def ProduceCtrlOutput(
    cmd: CtrlInput,
    angular_velocity_meas_deg_s: float,
    now: float,
) -> CtrlOutput:
    """CtrlInput + gyrz 측정값으로 서보 명령을 계산.

    Pipeline:
      1. 유효성 확인.
      2. yaw-rate 명령 클램핑.
      3. Expo 형상 피드포워드 (FF) — DELTA_ARM_MAX_DEG 이내.
      4. PID 폐루프 보정 (gyrz 유효 + pid_enabled일 때) — DELTA_ARM_MAX_DEG 이내.
      5. FF + PID 합산, DELTA_ARM_MAX_DEG로 클램핑.
      6. Arm 각도 slew-rate 제한 후 PWM 출력.
    """
    global _integral_deg, _prev_error_deg, _prev_time
    global _prev_left_angle_deg, _prev_right_angle_deg

    control_mode = _as_control_mode(cmd.control_mode)
    out_t = CtrlOutput(timestamp=now, control_mode=control_mode)
    is_dr = _is_dr(control_mode)

    # ── FAIL 모드 → 중립 ─────────────────────────────────────────────────────
    if _is_fail_mode(control_mode):
        out_t.reason = "FAIL"
        return out_t

    # ── 유효하지 않은 명령 → 중립 ────────────────────────────────────────────
    if not cmd.valid:
        out_t.reason = "INVALID_CMD"
        return out_t

    # ── NaN 명령 → 중립 ──────────────────────────────────────────────────────
    angular_velocity_cmd_deg_s = float(cmd.angular_velocity_cmd_deg_s)
    if not math.isfinite(angular_velocity_cmd_deg_s):
        out_t.reason = "NAN_CMD"
        return out_t
    raw_cmd = angular_velocity_cmd_deg_s

    # ── yaw-rate 명령 클램핑 (mode별 limit 사용) ─────────────────────────────
    # guidance가 mode별로 고른 limit을 그대로 쓴다. GPS_CLOSED limit으로 덮어쓰지 않는다.
    yaw_rate_limit_dps = _resolve_yaw_rate_limit_dps(cmd.yaw_rate_limit_dps)
    out_t.yaw_rate_limit_dps = yaw_rate_limit_dps
    angular_velocity_cmd_deg_s = _clamp(
        angular_velocity_cmd_deg_s,
        -yaw_rate_limit_dps,
         yaw_rate_limit_dps,
    )
    command_clamped = not math.isclose(
        angular_velocity_cmd_deg_s, raw_cmd, rel_tol=0.0, abs_tol=1.0e-9
    )
    out_t.angular_velocity_cmd_deg_s = angular_velocity_cmd_deg_s

    # PID + slew 공유 dt
    dt = _clamp_dt(now, _prev_time, 0.1, 0.01, 0.2)

    # ── 피드포워드 (모드별 권한 스케일 적용) ─────────────────────────────────
    # limit과 ff_scale은 서로 다른 역할이므로 중복 약화가 아니다:
    #   - yaw_rate_limit_dps: 명령(yaw-rate)의 최대 크기. FF curve 정규화의 분모.
    #     mode 신뢰도가 낮을수록 작은 limit → 같은 입력에서 더 완만한 곡선.
    #   - ff_scale: 산출된 arm authority 자체를 DR 신뢰도에 따라 줄이는 출력 스케일.
    # 즉 limit은 "얼마나 빠르게 돌라고 명령하나", ff_scale은 "그 명령에 팔을 얼마나
    # 깊게 쓰나"를 정한다. limit이 이미 명령을 줄이지만, 추가로 ff_scale을 곱하는 것은
    # DR mode에서 보수적 권한을 한 단계 더 두기 위한 의도된 설계다.
    ff_scale = _ff_scale_for_mode(control_mode)
    # DR은 GPS와 분리된 데드밴드/정규화 기준을 쓴다(작은 명령도 nu 비례로 살림).
    if is_dr:
        ff_deadband = getattr(config, "DR_CTRL_ANGULAR_VELOCITY_DEADBAND_DEG_S",
                              config.CTRL_ANGULAR_VELOCITY_DEADBAND_DEG_S)
        ff_ref = getattr(config, "DR_FF_REF_DPS", None)
    else:
        ff_deadband = config.CTRL_ANGULAR_VELOCITY_DEADBAND_DEG_S
        ff_ref = None
    delta_ff = angular_velocity_to_delta_ff(
        angular_velocity_cmd_deg_s, yaw_rate_limit_dps,
        deadband_dps=ff_deadband, ref_dps=ff_ref) * ff_scale
    delta_ff_pre_cap = delta_ff
    # DR FF 출력 cap: 감도 상향이 full hard-over/나선으로 가지 않도록 별도 제한.
    if is_dr:
        dr_ff_cap = getattr(config, "DR_FF_DELTA_LIMIT_DEG", DELTA_ARM_MAX_DEG)
        delta_ff = _clamp(delta_ff, -dr_ff_cap, dr_ff_cap)
    out_t.ff_scale = ff_scale
    out_t.ff_deadband_dps = ff_deadband
    out_t.ff_ref_dps = (yaw_rate_limit_dps if ff_ref is None else ff_ref)
    out_t.delta_ff_pre_cap_deg = delta_ff_pre_cap

    # ── Gyro 스파이크 거부 ────────────────────────────────────────────────────
    gyro_finite  = math.isfinite(angular_velocity_meas_deg_s)
    gyro_spike   = gyro_finite and abs(angular_velocity_meas_deg_s) > GYRO_SPIKE_LIMIT_DEG_S
    sensor_valid = gyro_finite and not gyro_spike
    out_t.sensor_valid  = sensor_valid
    out_t.gyro_rejected = gyro_spike

    # ── PID 폐루프 ────────────────────────────────────────────────────────────
    integral  = _integral_deg
    delta_pid = 0.0
    error     = 0.0
    delta_sum = delta_ff

    pid_active = bool(
        cmd.pid_enabled
        and DELTA_ARM_MAX_DEG > 0.0
        and sensor_valid
    )
    # DR_PID_ENABLED=False면 guidance가 DR에서 pid_enabled=False로 내려 pid_active=False다.
    # 이 플래그로 "DR에서 PID가 실제로 돌았는지"를 로그로 검증한다(정상 시 항상 0).
    out_t.dr_pid_active = bool(is_dr and pid_active)

    if pid_active:
        out_t.angular_velocity_meas_deg_s = angular_velocity_meas_deg_s
        error = angular_velocity_cmd_deg_s - angular_velocity_meas_deg_s
        if abs(error) < config.CTRL_ERROR_DEADBAND_DEG_S:
            error = 0.0
        derivative         = (error - _prev_error_deg) / dt
        integral_candidate = _clamp(
            _integral_deg + error * dt,
            -config.CTRL_I_LIMIT_DEG, config.CTRL_I_LIMIT_DEG,
        )
        kp, ki, kd = _select_pid_gains(control_mode, cmd.kp_override)
        delta_pid = _clamp(
            kp * error + ki * integral_candidate + kd * derivative,
            -DELTA_ARM_MAX_DEG, DELTA_ARM_MAX_DEG,
        )
        integral = integral_candidate
        out_t.kp_used = kp
        out_t.angular_velocity_error_deg_s = error
        delta_sum = delta_ff + delta_pid

    else:
        # gyro 없음: FF만 사용. 누적 windup 감쇄.
        integral  = _integral_deg * INTEGRAL_DECAY_RATE
        delta_sum = delta_ff

    # ── 합산 + 총 클램핑 ──────────────────────────────────────────────────────
    # DR은 DR_FF_DELTA_LIMIT_DEG, 그 외는 일반 천장(±DELTA_ARM_MAX_DEG)으로 클램핑.
    delta_limit = (getattr(config, "DR_FF_DELTA_LIMIT_DEG", DELTA_ARM_MAX_DEG)
                   if is_dr else DELTA_ARM_MAX_DEG)
    authority_saturated = abs(delta_sum) > delta_limit
    saturated = command_clamped or authority_saturated
    out_t.saturated = saturated
    delta_total = _clamp(delta_sum, -delta_limit, delta_limit)

    # ── delta_total → arm 각도 ────────────────────────────────────────────────
    _, _, left_des, right_des, delta_arm = ConnectRoMo(delta_total)
    # ── Slew-rate 제한 ────────────────────────────────────────────────────────
    max_step    = MAX_ARM_RATE_DEG_S * dt
    left_angle  = _clamp(left_des,
                         _prev_left_angle_deg - max_step,
                         _prev_left_angle_deg + max_step)
    right_angle = _clamp(right_des,
                         _prev_right_angle_deg - max_step,
                         _prev_right_angle_deg + max_step)

    left_pw  = int(_clamp(LEFT_ZERO  - left_angle  * PULSE_PER_DEG,
                          LEFT_MIN_PULSE,  LEFT_MAX_PULSE))
    right_pw = int(_clamp(RIGHT_ZERO + right_angle * PULSE_PER_DEG,
                          RIGHT_MIN_PULSE, RIGHT_MAX_PULSE))

    out_t.delta_ff_deg    = delta_ff
    out_t.delta_pid_deg   = delta_pid
    out_t.delta_total_deg = delta_sum
    out_t.delta_arm_deg   = delta_arm
    out_t.motor_cmd       = delta_arm
    out_t.left_angle_deg  = left_angle
    out_t.right_angle_deg = right_angle
    out_t.left_pw         = left_pw
    out_t.right_pw        = right_pw
    out_t.valid           = True
    out_t.reason          = "SATURATED" if saturated else "OK"

    # ── PID 상태 업데이트 ─────────────────────────────────────────────────────
    if pid_active:
        _prev_error_deg = error
        # 조건부 anti-windup: 오차가 포화 방향과 같을 때만 적분 차단
        error_aggravates = saturated and (
            error != 0.0
            and math.copysign(1.0, delta_sum) == math.copysign(1.0, error)
        )
        if not error_aggravates:
            _integral_deg = integral
    else:
        _integral_deg = integral

    _prev_time            = now
    _prev_left_angle_deg  = left_angle
    _prev_right_angle_deg = right_angle
    return out_t


# ── pigpio 바인딩 ──────────────────────────────────────────────────────────────

def init_control():
    """pigpio 초기화 + 서보 영점 설정. pi handle 또는 None 반환."""
    try:
        import pigpio
        pi = pigpio.pi()
        if pi.connected:
            pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN,  LEFT_ZERO_PULSE)
            pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, RIGHT_ZERO_PULSE)
            logger.info("pigpio connected; servos zeroed")
            return pi
        logger.warning("pigpio.pi() not connected; no servo output")
    except Exception as exc:
        logger.warning("pigpio init failed (%s); no servo output", exc)
    return None


def WriteZero(pi) -> None:
    """양쪽 arm을 0 deg (arm up)로 설정."""
    if pi is None:
        return
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN,  LEFT_ZERO_PULSE)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, RIGHT_ZERO_PULSE)


def WriteOff(pi) -> None:
    """서보 PWM 차단 (pulse width = 0)."""
    if pi is None:
        return
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN,  0)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)


def MoveServo(pi, cmd: CtrlOutput) -> None:
    """CtrlOutput을 서보 PWM 채널에 기록."""
    if pi is None:
        return
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN,  cmd.left_pw)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, cmd.right_pw)
