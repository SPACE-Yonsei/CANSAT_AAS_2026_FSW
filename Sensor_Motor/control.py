"""Sensor_Motor/control.py — yaw-rate 제어 + 서보 믹서

Pipeline:
  ProduceCtrlInput(l1_output, now)
    → ProduceCtrlOutput(ctrler, ctrl_input, gyrz_meas_deg_s, now)
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
from dataclasses import dataclass, field
from typing import Optional

from lib import config

logger = logging.getLogger(__name__)


# ── 모드 / 폴백 문자열 상수 ───────────────────────────────────────────────────
CTRL_MODE_NEUTRAL          = "NEUTRAL"
CTRL_MODE_CLOSED_LOOP      = "CLOSED_LOOP"
CTRL_MODE_FEEDFORWARD_ONLY = "FEEDFORWARD_ONLY"
CTRL_MODE_GUIDANCE_TIMEOUT = "GUIDANCE_TIMEOUT"

CTRL_FALLBACK_NONE                = "NONE"
CTRL_FALLBACK_GUIDANCE_TIMEOUT    = "GUIDANCE_TIMEOUT"
CTRL_FALLBACK_GUIDANCE_ATTENUATED = "GUIDANCE_ATTENUATED"
CTRL_FALLBACK_GYRO_SPIKE          = "GYRO_SPIKE"


# ── GPIO + 서보 캘리브레이션 (config에서 읽음) ────────────────────────────────
PARAFOIL_LEFT_MOTOR_PIN  = config.PARAFOIL_LEFT_GPIO
PARAFOIL_RIGHT_MOTOR_PIN = config.PARAFOIL_RIGHT_GPIO

ARM_MIN_DEG      = config.ARM_MIN_DEG
ARM_MAX_DEG      = config.ARM_MAX_DEG
NEUTRAL_ARM_DEG  = config.NEUTRAL_ARM_DEG
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

# 타임아웃 / 스파이크 / 적분 감쇄 (config에서)
GUIDANCE_TIMEOUT_ATTENUATE_S = config.GUIDANCE_TIMEOUT_ATTENUATE_S
GUIDANCE_TIMEOUT_FAIL_S      = config.GUIDANCE_TIMEOUT_FAIL_S
GYRO_SPIKE_LIMIT_DEG_S       = config.GYRO_SPIKE_LIMIT_DEG_S
INTEGRAL_DECAY_RATE          = config.INTEGRAL_DECAY_RATE


# ── 내부 유틸 ─────────────────────────────────────────────────────────────────

def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _cmd_age(now: float, timestamp: float) -> float:
    """now - timestamp. 어느 쪽이든 non-finite이면 math.inf."""
    try:
        age = float(now) - float(timestamp)
    except (TypeError, ValueError):
        return math.inf
    return age if math.isfinite(age) else math.inf


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


def _is_detumbling_mode(control_mode) -> bool:
    return (
        control_mode == config.CONTROL_MODE_DETUMBLING
        or getattr(control_mode, "value", None) == config.CONTROL_MODE_DETUMBLING
    )


# ── Dataclass 정의 ────────────────────────────────────────────────────────────

@dataclass
class ControlConfig:
    # 피드포워드 형상
    ANGULAR_VELOCITY_CMD_MAX_DEG_S:  float = config.GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS
    ANGULAR_VELOCITY_DEADBAND_DEG_S: float = config.CTRL_ANGULAR_VELOCITY_DEADBAND_DEG_S
    DELTA_FF_MAX_DEG:                float = 160.0
    DELTA_MIN_EFFECTIVE_DEG:         float = config.CTRL_DELTA_MIN_EFFECTIVE_DEG
    EXPO:                            float = config.CTRL_EXPO

    # PID
    ERROR_DEADBAND_DEG_S: float = config.CTRL_ERROR_DEADBAND_DEG_S
    K_P:                  float = config.KP_GPS_CLOSED
    K_I:                  float = config.CTRL_K_I
    K_D:                  float = config.KD_YAW_RATE
    I_LIMIT_DEG:          float = config.CTRL_I_LIMIT_DEG
    DELTA_PID_MAX_DEG:    float = 160.0

    # 총 권한 + slew
    DELTA_TOTAL_MAX_DEG: float = 160.0
    MAX_ARM_RATE_DEG_S:  float = 200.0


@dataclass
class _PIDState:
    integral_deg:   float = 0.0
    prev_error_deg: float = 0.0
    prev_time:      float = 0.0


@dataclass
class Ctrler:
    config: ControlConfig = field(default_factory=ControlConfig)
    pid:    _PIDState     = field(default_factory=_PIDState)
    prev_left_angle_deg:  float = NEUTRAL_ARM_DEG
    prev_right_angle_deg: float = NEUTRAL_ARM_DEG


@dataclass
class CtrlInput:
    angular_velocity_cmd_deg_s: float = 0.0
    ground_speed_mps:           float = 0.0
    valid:                      bool  = False
    timestamp:                  float = 0.0
    pid_enabled:                bool  = True
    control_mode:  Optional[str]   = None
    dr_method:     Optional[str]   = None
    kp_override:   Optional[float] = None


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
    angular_velocity_cmd_deg_s:   float = 0.0
    angular_velocity_meas_deg_s:  float = float("nan")
    angular_velocity_error_deg_s: float = 0.0
    motor_cmd:     float = 0.0
    saturated:     bool  = False
    sensor_valid:  bool  = False
    valid:         bool  = False
    mode:          str   = CTRL_MODE_NEUTRAL
    fallback_mode: str   = CTRL_FALLBACK_NONE
    guidance_command_age_s: float = 0.0


# ── 팩토리 / 리셋 / 중립 ──────────────────────────────────────────────────────

def MakeCtrler(cfg: Optional[ControlConfig] = None) -> Ctrler:
    """Ctrler 인스턴스 생성. cfg=None이면 기본값 사용."""
    return Ctrler(config=cfg or ControlConfig())


def controller_reset(ctl: Ctrler) -> None:
    """PID 상태 + 이전 arm 각도 초기화."""
    ctl.pid = _PIDState()
    ctl.prev_left_angle_deg  = NEUTRAL_ARM_DEG
    ctl.prev_right_angle_deg = NEUTRAL_ARM_DEG


def WriteNeutral(now: float, mode: str = CTRL_MODE_NEUTRAL) -> CtrlOutput:
    """중립 PWM CtrlOutput 생성. 하드웨어 접촉 없음."""
    cmd_t = CtrlOutput(timestamp=now)
    cmd_t.mode = mode
    cmd_t.fallback_mode = mode
    return cmd_t


# ── 입력 변환 (guidance → control 단위 변환) ─────────────────────────────────

def ProduceCtrlInput(l1_output, now: float) -> CtrlInput:
    """L1Output(rad/s) → CtrlInput(deg/s) 변환."""
    is_valid = bool(
        getattr(l1_output, "control_valid", False)
        or getattr(l1_output, "nominal", False)
    )
    rad = getattr(l1_output, "yaw_rate_cmd", 0.0)
    rad = float(rad or 0.0)

    return CtrlInput(
        angular_velocity_cmd_deg_s=math.degrees(rad),
        ground_speed_mps=float(getattr(l1_output, "ground_speed_mps", 0.0) or 0.0),
        valid=is_valid,
        timestamp=float(getattr(l1_output, "timestamp", now) or now),
        pid_enabled=bool(getattr(l1_output, "pid_enabled", True)),
        control_mode=getattr(l1_output, "control_mode", None),
        dr_method=getattr(l1_output, "dr_method", None),
        kp_override=getattr(l1_output, "kp_override", None),
    )


# ── 피드포워드 형상 + 믹서 ────────────────────────────────────────────────────

def angular_velocity_to_delta_ff(cmd_dps: float, cfg: ControlConfig) -> float:
    """yaw-rate 명령(deg/s) → 차동 arm 각도(deg).

    Expo 곡선 + 데드밴드 + 부호 보존 (양수=오른쪽 회전).
    """
    clamped = _clamp(cmd_dps,
                     -cfg.ANGULAR_VELOCITY_CMD_MAX_DEG_S,
                      cfg.ANGULAR_VELOCITY_CMD_MAX_DEG_S)
    if abs(clamped) < cfg.ANGULAR_VELOCITY_DEADBAND_DEG_S:
        return 0.0
    x = abs(clamped) / cfg.ANGULAR_VELOCITY_CMD_MAX_DEG_S
    delta = (cfg.DELTA_MIN_EFFECTIVE_DEG
             + (cfg.DELTA_FF_MAX_DEG - cfg.DELTA_MIN_EFFECTIVE_DEG) * (x ** cfg.EXPO))
    return math.copysign(delta, clamped)


def ConnectRoMo(delta_arm_deg: float):
    """차동 편향각 → arm 각도 + PWM 펄스.

    양수 delta → 오른쪽 회전 (right arm ↑, left arm ↓).
    Returns: (left_pw, right_pw, left_angle_deg, right_angle_deg, delta_arm_deg)
    """
    delta      = _clamp(delta_arm_deg, -DELTA_ARM_MAX_DEG, DELTA_ARM_MAX_DEG)
    left_angle = _clamp(NEUTRAL_ARM_DEG - delta / 2.0, ARM_MIN_DEG, ARM_MAX_DEG)
    right_angle = _clamp(NEUTRAL_ARM_DEG + delta / 2.0, ARM_MIN_DEG, ARM_MAX_DEG)
    left_pw  = int(_clamp(LEFT_ZERO  - left_angle  * PULSE_PER_DEG,
                          LEFT_MIN_PULSE,  LEFT_MAX_PULSE))
    right_pw = int(_clamp(RIGHT_ZERO + right_angle * PULSE_PER_DEG,
                          RIGHT_MIN_PULSE, RIGHT_MAX_PULSE))
    return left_pw, right_pw, left_angle, right_angle, delta


# ── 메인 컨트롤러 스텝 ────────────────────────────────────────────────────────

def ProduceCtrlOutput(
    ctl: Ctrler,
    cmd: CtrlInput,
    angular_velocity_meas_deg_s: float,
    now: float,
) -> CtrlOutput:
    """CtrlInput + gyrz 측정값으로 서보 명령을 계산.

    Pipeline:
      1. 유효성 확인; guidance 명령 age 체크 (FAIL / 감쇄 구간).
      2. yaw-rate 명령 클램핑.
      3. Expo 형상 피드포워드 (FF) — DELTA_FF_MAX_DEG 이내.
      4. PID 폐루프 보정 (gyrz 유효 + pid_enabled일 때) — DELTA_PID_MAX_DEG 이내.
      5. FF + PID 합산, DELTA_TOTAL_MAX_DEG로 클램핑.
      6. Arm 각도 slew-rate 제한 후 PWM 출력.
    """
    out_t = CtrlOutput(timestamp=now)
    cfg_t = ctl.config

    # ── 유효하지 않은 명령 → 중립 ────────────────────────────────────────────
    if not cmd.valid:
        out_t.mode = CTRL_MODE_NEUTRAL
        out_t.fallback_mode = config.MOTOR_REASON_GUIDANCE_INACTIVE
        return out_t

    # ── NaN 명령 → 중립 ──────────────────────────────────────────────────────
    raw_cmd = float(cmd.angular_velocity_cmd_deg_s)
    if not math.isfinite(raw_cmd):
        out_t.mode = CTRL_MODE_NEUTRAL
        out_t.fallback_mode = CTRL_FALLBACK_NONE
        return out_t

    angular_velocity_cmd_deg_s = raw_cmd
    age = _cmd_age(now, cmd.timestamp)
    out_t.guidance_command_age_s = age

    # ── 2단계 guidance 타임아웃 ───────────────────────────────────────────────
    if age > GUIDANCE_TIMEOUT_FAIL_S:
        out_t.mode          = CTRL_MODE_GUIDANCE_TIMEOUT
        out_t.fallback_mode = CTRL_FALLBACK_GUIDANCE_TIMEOUT
        out_t.valid = False
        out_t.left_angle_deg  = NEUTRAL_ARM_DEG
        out_t.right_angle_deg = NEUTRAL_ARM_DEG
        out_t.left_pw         = LEFT_NEUTRAL
        out_t.right_pw        = RIGHT_NEUTRAL
        return out_t

    if age > GUIDANCE_TIMEOUT_ATTENUATE_S:
        angular_velocity_cmd_deg_s *= 0.5
        out_t.fallback_mode = CTRL_FALLBACK_GUIDANCE_ATTENUATED
    else:
        out_t.fallback_mode = CTRL_FALLBACK_NONE

    # ── yaw-rate 명령 클램핑 ──────────────────────────────────────────────────
    angular_velocity_cmd_deg_s = _clamp(
        angular_velocity_cmd_deg_s,
        -cfg_t.ANGULAR_VELOCITY_CMD_MAX_DEG_S,
         cfg_t.ANGULAR_VELOCITY_CMD_MAX_DEG_S,
    )
    command_clamped = not math.isclose(
        angular_velocity_cmd_deg_s, raw_cmd, rel_tol=0.0, abs_tol=1.0e-9
    )
    out_t.angular_velocity_cmd_deg_s = angular_velocity_cmd_deg_s

    # PID + slew 공유 dt
    dt = _clamp_dt(now, ctl.pid.prev_time, 0.1, 0.01, 0.2)

    # ── 피드포워드 ────────────────────────────────────────────────────────────
    delta_ff = angular_velocity_to_delta_ff(angular_velocity_cmd_deg_s, cfg_t)

    # ── Gyro 스파이크 거부 ────────────────────────────────────────────────────
    gyro_finite  = math.isfinite(angular_velocity_meas_deg_s)
    gyro_spike   = gyro_finite and abs(angular_velocity_meas_deg_s) > GYRO_SPIKE_LIMIT_DEG_S
    sensor_valid = gyro_finite and not gyro_spike
    out_t.sensor_valid = sensor_valid
    if gyro_spike:
        out_t.fallback_mode = CTRL_FALLBACK_GYRO_SPIKE

    detumbling_active = _is_detumbling_mode(cmd.control_mode)

    # ── PID 폐루프 / 텀블링 제동 ──────────────────────────────────────────────
    integral  = ctl.pid.integral_deg
    delta_pid = 0.0
    error     = 0.0
    delta_sum = delta_ff

    pid_active = bool(
        (not detumbling_active)
        and cmd.pid_enabled
        and cfg_t.DELTA_PID_MAX_DEG > 0.0
        and sensor_valid
    )

    if detumbling_active:
        out_t.mode  = config.CONTROL_MODE_DETUMBLING
        delta_ff  = 0.0
        out_t.angular_velocity_meas_deg_s = angular_velocity_meas_deg_s
        if sensor_valid:
            error = -angular_velocity_meas_deg_s
            if abs(angular_velocity_meas_deg_s) > cfg_t.ERROR_DEADBAND_DEG_S:
                delta_sum = -math.copysign(
                    config.DETUMBLE_BRAKE_DELTA_DEG,
                    angular_velocity_meas_deg_s,
                )
            else:
                delta_sum = 0.0
            out_t.angular_velocity_error_deg_s = error
        else:
            delta_sum = 0.0

    elif pid_active:
        out_t.angular_velocity_meas_deg_s = angular_velocity_meas_deg_s
        error = angular_velocity_cmd_deg_s - angular_velocity_meas_deg_s
        if abs(error) < cfg_t.ERROR_DEADBAND_DEG_S:
            error = 0.0
        derivative         = (error - ctl.pid.prev_error_deg) / dt
        integral_candidate = _clamp(
            ctl.pid.integral_deg + error * dt,
            -cfg_t.I_LIMIT_DEG, cfg_t.I_LIMIT_DEG,
        )
        kp      = float(cmd.kp_override) if cmd.kp_override is not None else cfg_t.K_P
        delta_pid = _clamp(
            kp * error + cfg_t.K_I * integral_candidate + cfg_t.K_D * derivative,
            -cfg_t.DELTA_PID_MAX_DEG, cfg_t.DELTA_PID_MAX_DEG,
        )
        integral = integral_candidate
        out_t.angular_velocity_error_deg_s = error
        out_t.mode  = CTRL_MODE_CLOSED_LOOP
        delta_sum = delta_ff + delta_pid

    else:
        # gyro 없음: FF만 사용. 누적 windup 감쇄.
        integral = ctl.pid.integral_deg * INTEGRAL_DECAY_RATE
        out_t.mode  = CTRL_MODE_FEEDFORWARD_ONLY
        delta_sum = delta_ff

    # ── 합산 + 총 클램핑 ──────────────────────────────────────────────────────
    authority_saturated = abs(delta_sum) > cfg_t.DELTA_TOTAL_MAX_DEG
    saturated = command_clamped or authority_saturated
    out_t.saturated = saturated
    delta_total   = _clamp(delta_sum, -cfg_t.DELTA_TOTAL_MAX_DEG, cfg_t.DELTA_TOTAL_MAX_DEG)

    # ── delta_total → arm 각도 ────────────────────────────────────────────────
    _, _, left_des, right_des, delta_arm = ConnectRoMo(delta_total)

    # ── Slew-rate 제한 ────────────────────────────────────────────────────────
    max_step    = cfg_t.MAX_ARM_RATE_DEG_S * dt
    left_angle  = _clamp(left_des,
                         ctl.prev_left_angle_deg - max_step,
                         ctl.prev_left_angle_deg + max_step)
    right_angle = _clamp(right_des,
                         ctl.prev_right_angle_deg - max_step,
                         ctl.prev_right_angle_deg + max_step)

    left_pw  = int(_clamp(LEFT_ZERO  - left_angle  * PULSE_PER_DEG,
                          LEFT_MIN_PULSE,  LEFT_MAX_PULSE))
    right_pw = int(_clamp(RIGHT_ZERO + right_angle * PULSE_PER_DEG,
                          RIGHT_MIN_PULSE, RIGHT_MAX_PULSE))

    out_t.delta_ff_deg    = delta_ff
    out_t.delta_pid_deg   = delta_pid
    out_t.delta_arm_deg   = delta_arm
    out_t.motor_cmd       = delta_arm
    out_t.left_angle_deg  = left_angle
    out_t.right_angle_deg = right_angle
    out_t.left_pw         = left_pw
    out_t.right_pw        = right_pw
    out_t.valid           = True

    # ── PID 상태 업데이트 ─────────────────────────────────────────────────────
    if pid_active:
        ctl.pid.prev_error_deg = error
        # 조건부 anti-windup: 오차가 포화 방향과 같을 때만 적분 차단
        error_aggravates = saturated and (
            error != 0.0
            and math.copysign(1.0, delta_sum) == math.copysign(1.0, error)
        )
        if not error_aggravates:
            ctl.pid.integral_deg = integral
    elif not detumbling_active:
        ctl.pid.integral_deg = integral

    ctl.pid.prev_time           = now
    ctl.prev_left_angle_deg     = left_angle
    ctl.prev_right_angle_deg    = right_angle
    return out_t


# ── Detumbling 전용 출력 ──────────────────────────────────────────────────────

def ProduceDetumbleOutput(now: float) -> CtrlOutput:
    """DETUMBLING 모드: yaw_rate_cmd=0 중립 유지.

    ProduceCtrlOutput에 mode=DETUMBLING으로 전달하지 않고,
    단순히 중립 서보 명령을 반환한다.
    실제 텀블링 제동(gyrz 기반 brake)은 ProduceCtrlOutput 내 detumbling_active
    경로가 담당한다.
    """
    return WriteNeutral(now, mode=config.CONTROL_MODE_DETUMBLING)


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
    """CtrlOutput을 서보 PWM 채널에 기록.

    stale_control.py의 ProducePulse를 MoveServo로 이름 통일.
    """
    if pi is None:
        return
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN,  cmd.left_pw)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, cmd.right_pw)
