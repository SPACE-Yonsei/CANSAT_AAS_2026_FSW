"""Brake/servo control module for CanSat parafoil.

Sign convention: positive delta_arm_deg = right turn (right arm down, left arm up).
Angles: 0 deg = arm up, 160 deg = arm down, neutral = 80 deg.

Authority budget: FF claims up to DELTA_FF_MAX_DEG; PID trims within DELTA_PID_MAX_DEG.
Both sum to at most DELTA_TOTAL_MAX_DEG. Arm angles are slew-rate limited before pulse output.
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field, replace
from typing import Optional

from lib import config, timebase

logger = logging.getLogger(__name__)


PARAFOIL_LEFT_MOTOR_PIN  = config.PARAFOIL_LEFT_GPIO
PARAFOIL_RIGHT_MOTOR_PIN = config.PARAFOIL_RIGHT_GPIO

# 서보 암 기하학 — config.py 기준값 사용
ARM_MIN_DEG     = config.ARM_MIN_DEG
ARM_MAX_DEG     = config.ARM_MAX_DEG
ZERO_ARM_DEG    = config.ARM_MIN_DEG        # 0도 = 암 위쪽 (up-zero 프레임)
NEUTRAL_ARM_DEG = config.NEUTRAL_ARM_DEG
DELTA_ARM_MAX_DEG = 2.0 * min(
    NEUTRAL_ARM_DEG - ARM_MIN_DEG,
    ARM_MAX_DEG - NEUTRAL_ARM_DEG,
)

# PWM 매핑 — config.py 캘리브레이션 값 사용
LEFT_ZERO     = config.LEFT_SERVO_ZERO_US
RIGHT_ZERO    = config.RIGHT_SERVO_ZERO_US
PULSE_PER_DEG = config.SERVO_PULSE_PER_DEG

LEFT_NEUTRAL  = int(LEFT_ZERO - NEUTRAL_ARM_DEG * PULSE_PER_DEG)
RIGHT_NEUTRAL = int(RIGHT_ZERO + NEUTRAL_ARM_DEG * PULSE_PER_DEG)

LEFT_ZERO_PULSE  = int(LEFT_ZERO)
RIGHT_ZERO_PULSE = int(RIGHT_ZERO)

LEFT_MIN_PULSE  = int(LEFT_ZERO - ARM_MAX_DEG * PULSE_PER_DEG)
LEFT_MAX_PULSE  = int(LEFT_ZERO - ARM_MIN_DEG * PULSE_PER_DEG)
RIGHT_MIN_PULSE = int(RIGHT_ZERO + ARM_MIN_DEG * PULSE_PER_DEG)
RIGHT_MAX_PULSE = int(RIGHT_ZERO + ARM_MAX_DEG * PULSE_PER_DEG)

# 가이던스 타임아웃 / 자이로 스파이크 / 적분 감쇠 — config.py
GUIDANCE_TIMEOUT_ATTENUATE_S = config.GUIDANCE_TIMEOUT_ATTENUATE_S
GUIDANCE_TIMEOUT_FAIL_S      = config.GUIDANCE_TIMEOUT_FAIL_S
GYRO_SPIKE_LIMIT_DEG_S       = config.GYRO_SPIKE_LIMIT_DEG_S
INTEGRAL_DECAY_RATE          = config.INTEGRAL_DECAY_RATE


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


@dataclass
class ControlConfig:
    # FF 형상
    ANGULAR_VELOCITY_CMD_MAX_DEG_S:  float = config.MOTOR_NOMINAL_CLOSED_LOOP_ANGULAR_VELOCITY_CMD_MAX_DEG_S
    ANGULAR_VELOCITY_DEADBAND_DEG_S: float = config.CTRL_ANGULAR_VELOCITY_DEADBAND_DEG_S
    DELTA_FF_MAX_DEG:                float = config.MOTOR_NOMINAL_CLOSED_LOOP_DELTA_FF_MAX_DEG
    DELTA_MIN_EFFECTIVE_DEG:         float = config.CTRL_DELTA_MIN_EFFECTIVE_DEG
    EXPO:                            float = config.CTRL_EXPO

    # PID
    ERROR_DEADBAND_DEG_S: float = config.CTRL_ERROR_DEADBAND_DEG_S
    K_P:                  float = config.KP_GPS_CLOSED
    K_I:                  float = config.CTRL_K_I
    K_D:                  float = config.KD_YAW_RATE
    I_LIMIT_DEG:          float = config.CTRL_I_LIMIT_DEG
    DELTA_PID_MAX_DEG:    float = config.MOTOR_NOMINAL_CLOSED_LOOP_DELTA_PID_MAX_DEG

    # 권한 / 슬루
    DELTA_TOTAL_MAX_DEG: float = config.MOTOR_NOMINAL_CLOSED_LOOP_DELTA_TOTAL_MAX_DEG
    MAX_ARM_RATE_DEG_S:  float = config.MOTOR_NOMINAL_CLOSED_LOOP_MAX_ARM_RATE_DEG_S


@dataclass
class _PIDState:
    integral_deg: float = 0.0
    prev_error_deg: float = 0.0
    prev_time: float = 0.0


@dataclass
class Ctrler:
    config: ControlConfig = field(default_factory=ControlConfig)
    pid: _PIDState = field(default_factory=_PIDState)
    prev_left_angle_deg: float = NEUTRAL_ARM_DEG
    prev_right_angle_deg: float = NEUTRAL_ARM_DEG


@dataclass
class CtrlInput:
    angular_velocity_cmd_deg_s: float = 0.0
    lat_acc_cmd_mps2: float = 0.0
    ground_speed_mps: float = 0.0
    valid: bool = False
    timestamp: float = 0.0
    pid_enabled: bool = True
    angular_velocity_cmd_max_deg_s: Optional[float] = None
    delta_ff_max_deg: Optional[float] = None
    delta_pid_max_deg: Optional[float] = None
    delta_total_max_deg: Optional[float] = None
    max_arm_rate_deg_s: Optional[float] = None


@dataclass
class CtrlOutput:
    timestamp: float
    left_pw: int = LEFT_NEUTRAL
    right_pw: int = RIGHT_NEUTRAL
    left_angle_deg: float = NEUTRAL_ARM_DEG
    right_angle_deg: float = NEUTRAL_ARM_DEG
    delta_arm_deg: float = 0.0
    delta_ff_deg: float = 0.0
    delta_pid_deg: float = 0.0
    angular_velocity_cmd_deg_s: float = 0.0
    angular_velocity_meas_deg_s: float = float("nan")
    angular_velocity_error_deg_s: float = 0.0
    saturated: bool = False
    sensor_valid: bool = False
    valid: bool = False
    mode: str = config.CTRL_MODE_NEUTRAL
    fallback_mode: str = config.CTRL_MODE_NEUTRAL
    guidance_command_age_s: float = 0.0


def WriteNeutral(now: float, mode: str = config.CTRL_MODE_NEUTRAL) -> CtrlOutput:
    """Produce a neutral PWM command object without touching hardware."""
    cmd = CtrlOutput(timestamp=now)
    cmd.mode = mode
    cmd.fallback_mode = mode
    return cmd


def ProduceCtrlInput(g_out, now: float) -> CtrlInput:
    # Accept nominal AND degraded guidance modes — both produce usable commands.
    # FAIL mode sets nominal=False, degraded=False; only that path yields valid=False.
    is_valid = bool(
        getattr(g_out, "control_valid", False)
        or getattr(g_out, "nominal", False)
        or getattr(g_out, "degraded", False)
    )
    return CtrlInput(
        angular_velocity_cmd_deg_s=math.degrees(float(getattr(g_out, "angular_velocity_cmd_rad_s", 0.0) or 0.0)),
        lat_acc_cmd_mps2=float(getattr(g_out, "lat_acc_cmd_mps2", 0.0) or 0.0),
        ground_speed_mps=float(getattr(g_out, "ground_speed_mps", 0.0) or 0.0),
        valid=is_valid,
        timestamp=float(getattr(g_out, "timestamp", now) or now),
        pid_enabled=bool(getattr(g_out, "pid_enabled", True)),
        angular_velocity_cmd_max_deg_s=getattr(g_out, "angular_velocity_cmd_max_deg_s", None),
        delta_ff_max_deg=getattr(g_out, "delta_ff_max_deg", None),
        delta_pid_max_deg=getattr(g_out, "delta_pid_max_deg", None),
        delta_total_max_deg=getattr(g_out, "delta_total_max_deg", None),
        max_arm_rate_deg_s=getattr(g_out, "max_arm_rate_deg_s", None),
    )


def angular_velocity_to_delta_ff(angular_velocity_cmd_deg_s: float, cfg: ControlConfig) -> float:
    """Map angular velocity command to feedforward differential arm deflection.

    Expo curve gives proportional authority at small inputs and approaches
    DELTA_FF_MAX_DEG at full command, preserving DELTA_PID_MAX_DEG headroom for trim.
    Below DEADBAND returns zero so tiny commands don't cause dithering.
    Sign: positive = right turn (right arm down, left arm up).
    """
    clamped = _clamp(angular_velocity_cmd_deg_s, -cfg.ANGULAR_VELOCITY_CMD_MAX_DEG_S, cfg.ANGULAR_VELOCITY_CMD_MAX_DEG_S)
    if abs(clamped) < cfg.ANGULAR_VELOCITY_DEADBAND_DEG_S:
        return 0.0
    x = abs(clamped) / cfg.ANGULAR_VELOCITY_CMD_MAX_DEG_S
    delta = cfg.DELTA_MIN_EFFECTIVE_DEG + (cfg.DELTA_FF_MAX_DEG - cfg.DELTA_MIN_EFFECTIVE_DEG) * (x ** cfg.EXPO)
    return math.copysign(delta, clamped)


def ConnectRoMo(delta_arm_deg: float):
    """Map differential arm deflection to arm angles and PWM pulses.

    Positive delta_arm_deg = right turn: right arm angle increases (more brake),
    left arm angle decreases (less brake).
    Returns (left_pw, right_pw, left_angle_deg, right_angle_deg, delta_arm_deg).
    """
    delta = _clamp(delta_arm_deg, -DELTA_ARM_MAX_DEG, DELTA_ARM_MAX_DEG)
    left_angle = _clamp(NEUTRAL_ARM_DEG - delta / 2.0, ARM_MIN_DEG, ARM_MAX_DEG)
    right_angle = _clamp(NEUTRAL_ARM_DEG + delta / 2.0, ARM_MIN_DEG, ARM_MAX_DEG)

    left_pw = int(_clamp(LEFT_ZERO - left_angle * PULSE_PER_DEG, LEFT_MIN_PULSE, LEFT_MAX_PULSE))
    right_pw = int(_clamp(RIGHT_ZERO + right_angle * PULSE_PER_DEG, RIGHT_MIN_PULSE, RIGHT_MAX_PULSE))

    return left_pw, right_pw, left_angle, right_angle, delta


def MakeCtrler(cfg: Optional[ControlConfig] = None) -> Ctrler:
    return Ctrler(config=cfg or ControlConfig())


def controller_reset(ctl: Ctrler) -> None:
    ctl.pid = _PIDState()
    ctl.prev_left_angle_deg = NEUTRAL_ARM_DEG
    ctl.prev_right_angle_deg = NEUTRAL_ARM_DEG


def ProduceCtrlOutput(
    ctl: Ctrler,
    cmd: CtrlInput,
    angular_velocity_meas_deg_s: float,
    now: float,
) -> CtrlOutput:
    """Compute brake servo command from GuidanceCommand + optional gyro feedback.

    Pipeline:
      1. Expo-shaped FF sets the bulk of the command within DELTA_FF_MAX_DEG.
      2. PID trims within DELTA_PID_MAX_DEG using yaw-rate error.
      3. Sum is clamped to DELTA_TOTAL_MAX_DEG; saturation blocks integral growth.
      4. Desired arm angles from ConnectRoMo are slew-rate limited before pulse output.
    """
    out = CtrlOutput(timestamp=now)
    cfg_base = ctl.config
    cfg = replace(
        cfg_base,
        ANGULAR_VELOCITY_CMD_MAX_DEG_S=float(cmd.angular_velocity_cmd_max_deg_s)
        if cmd.angular_velocity_cmd_max_deg_s is not None else cfg_base.ANGULAR_VELOCITY_CMD_MAX_DEG_S,
        DELTA_FF_MAX_DEG=float(cmd.delta_ff_max_deg)
        if cmd.delta_ff_max_deg is not None else cfg_base.DELTA_FF_MAX_DEG,
        DELTA_PID_MAX_DEG=float(cmd.delta_pid_max_deg)
        if cmd.delta_pid_max_deg is not None else cfg_base.DELTA_PID_MAX_DEG,
        DELTA_TOTAL_MAX_DEG=float(cmd.delta_total_max_deg)
        if cmd.delta_total_max_deg is not None else cfg_base.DELTA_TOTAL_MAX_DEG,
        MAX_ARM_RATE_DEG_S=float(cmd.max_arm_rate_deg_s)
        if cmd.max_arm_rate_deg_s is not None else cfg_base.MAX_ARM_RATE_DEG_S,
    )

    if not cmd.valid:
        out.mode = config.CTRL_MODE_NEUTRAL
        out.fallback_mode = config.MOTOR_REASON_GUIDANCE_INACTIVE
        return out

    angular_velocity_cmd_raw_deg_s = float(cmd.angular_velocity_cmd_deg_s)
    angular_velocity_cmd_deg_s = angular_velocity_cmd_raw_deg_s

    age = timebase.age(now, cmd.timestamp)
    out.guidance_command_age_s = age

    # --- Two-stage guidance timeout ---
    if age > GUIDANCE_TIMEOUT_FAIL_S:
        # Command is too stale to trust; revert to neutral.
        out.mode = config.CTRL_MODE_GUIDANCE_TIMEOUT
        out.fallback_mode = config.CTRL_FALLBACK_GUIDANCE_TIMEOUT
        out.valid = False
        out.left_angle_deg = NEUTRAL_ARM_DEG
        out.right_angle_deg = NEUTRAL_ARM_DEG
        out.left_pw = LEFT_NEUTRAL
        out.right_pw = RIGHT_NEUTRAL
        return out

    if age > GUIDANCE_TIMEOUT_ATTENUATE_S:
        # Stale but not dead: attenuate to limit uncommanded drift.
        angular_velocity_cmd_deg_s *= 0.5
        out.fallback_mode = config.CTRL_FALLBACK_GUIDANCE_ATTENUATED
    else:
        out.fallback_mode = config.CTRL_FALLBACK_NONE

    angular_velocity_cmd_deg_s = _clamp(
        angular_velocity_cmd_deg_s,
        -cfg.ANGULAR_VELOCITY_CMD_MAX_DEG_S,
        cfg.ANGULAR_VELOCITY_CMD_MAX_DEG_S,
    )
    command_clamped = not math.isclose(
        angular_velocity_cmd_deg_s,
        angular_velocity_cmd_raw_deg_s,
        rel_tol=0.0,
        abs_tol=1.0e-9,
    )
    out.angular_velocity_cmd_deg_s = angular_velocity_cmd_deg_s

    # dt is shared by PID integration and slew-rate limit; always advances so
    # slew tracking stays accurate even when the gyro is temporarily unavailable.
    dt = timebase.clamp_dt(now, ctl.pid.prev_time, default_s=0.1, min_s=0.01, max_s=0.2)

    # --- Feedforward: expo-shaped, deadbanded ---
    delta_ff = angular_velocity_to_delta_ff(angular_velocity_cmd_deg_s, cfg)

    # --- Gyro spike rejection ---
    # A single IMU glitch can saturate the integral in one cycle; discard the sample instead.
    gyro_spike = math.isfinite(angular_velocity_meas_deg_s) and abs(angular_velocity_meas_deg_s) > GYRO_SPIKE_LIMIT_DEG_S
    sensor_valid = math.isfinite(angular_velocity_meas_deg_s) and not gyro_spike
    out.sensor_valid = sensor_valid
    if gyro_spike:
        out.fallback_mode = config.CTRL_FALLBACK_GYRO_SPIKE

    # --- PID closed-loop trim (only when gyro measurement is valid) ---
    integral = ctl.pid.integral_deg
    delta_pid = 0.0
    error = 0.0

    pid_active = bool(cmd.pid_enabled and cfg.DELTA_PID_MAX_DEG > 0.0 and sensor_valid)

    if pid_active:
        out.angular_velocity_meas_deg_s = angular_velocity_meas_deg_s
        error = angular_velocity_cmd_deg_s - angular_velocity_meas_deg_s
        if abs(error) < cfg.ERROR_DEADBAND_DEG_S:
            error = 0.0
        derivative = (error - ctl.pid.prev_error_deg) / dt
        integral_candidate = _clamp(ctl.pid.integral_deg + error * dt, -cfg.I_LIMIT_DEG, cfg.I_LIMIT_DEG)
        delta_pid = _clamp(
            cfg.K_P * error + cfg.K_I * integral_candidate + cfg.K_D * derivative,
            -cfg.DELTA_PID_MAX_DEG,
            cfg.DELTA_PID_MAX_DEG,
        )
        integral = integral_candidate
        out.angular_velocity_error_deg_s = error
        out.mode = config.CTRL_MODE_CLOSED_LOOP
    else:
        # No gyro: FF only. Decay the integral so stale windup does not accumulate.
        integral = ctl.pid.integral_deg * INTEGRAL_DECAY_RATE
        out.mode = config.CTRL_MODE_FEEDFORWARD_ONLY

    # --- Combine FF + PID and clamp total authority ---
    delta_sum = delta_ff + delta_pid
    authority_saturated = abs(delta_sum) > cfg.DELTA_TOTAL_MAX_DEG
    saturated = command_clamped or authority_saturated
    out.saturated = saturated
    delta_total = _clamp(delta_sum, -cfg.DELTA_TOTAL_MAX_DEG, cfg.DELTA_TOTAL_MAX_DEG)

    # --- Desired arm angles from total deflection command ---
    _, _, left_angle_des, right_angle_des, delta_arm = ConnectRoMo(delta_total)

    # --- Slew-rate limit: prevent sudden large arm movements ---
    max_step = cfg.MAX_ARM_RATE_DEG_S * dt
    left_angle = _clamp(left_angle_des,
                        ctl.prev_left_angle_deg - max_step,
                        ctl.prev_left_angle_deg + max_step)
    right_angle = _clamp(right_angle_des,
                         ctl.prev_right_angle_deg - max_step,
                         ctl.prev_right_angle_deg + max_step)

    # Pulses computed from slew-limited angles, not raw desired angles
    left_pw = int(_clamp(LEFT_ZERO - left_angle * PULSE_PER_DEG, LEFT_MIN_PULSE, LEFT_MAX_PULSE))
    right_pw = int(_clamp(RIGHT_ZERO + right_angle * PULSE_PER_DEG, RIGHT_MIN_PULSE, RIGHT_MAX_PULSE))

    out.delta_ff_deg = delta_ff
    out.delta_pid_deg = delta_pid
    out.delta_arm_deg = delta_arm
    out.left_angle_deg = left_angle
    out.right_angle_deg = right_angle
    out.left_pw = left_pw
    out.right_pw = right_pw
    out.valid = True

    # --- State update ---
    if pid_active:
        ctl.pid.prev_error_deg = error
        # Conditional anti-windup: only block integral growth when the error is
        # pushing the output deeper into saturation (same sign as delta_sum).
        # If error is opposite to saturation (recovery phase), allow integration.
        error_aggravates = saturated and (
            error != 0.0 and math.copysign(1.0, delta_sum) == math.copysign(1.0, error)
        )
        if not error_aggravates:
            ctl.pid.integral_deg = integral
        # else: hold integral — neither grow nor reset
    else:
        # Sensor absent: commit the decayed integral so windup slowly drains.
        ctl.pid.integral_deg = integral

    # Always advance time so dt is correct on next cycle regardless of sensor state.
    ctl.pid.prev_time = now
    ctl.prev_left_angle_deg = left_angle
    ctl.prev_right_angle_deg = right_angle

    return out


def init_control():
    """Initialize pigpio and set servos to zero. Returns pi handle or None."""
    try:
        import pigpio

        pi = pigpio.pi()
        if pi.connected:
            pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, LEFT_ZERO_PULSE)
            pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, RIGHT_ZERO_PULSE)
            logger.info("pigpio connected; servos initialized to zero")
            return pi
        logger.warning("pigpio.pi() not connected; running without servo output")
    except Exception as exc:
        logger.warning("pigpio init failed (%s); running without servo output", exc)
    return None


def WriteZero(pi) -> None:
    """Set both arms to 0 deg."""
    if pi is None:
        return
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, LEFT_ZERO_PULSE)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, RIGHT_ZERO_PULSE)


def WriteOff(pi) -> None:
    if pi is None:
        return
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, 0)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)


def Set180(pi) -> None:
    """Set both arms to 180 deg, clamped by configured servo limits."""
    if pi is None:
        return
    left_pw = int(_clamp(LEFT_ZERO - 180.0 * PULSE_PER_DEG, LEFT_MIN_PULSE, LEFT_MAX_PULSE))
    right_pw = int(_clamp(RIGHT_ZERO + 180.0 * PULSE_PER_DEG, RIGHT_MIN_PULSE, RIGHT_MAX_PULSE))
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_pw)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_pw)


def ProducePulse(pi, cmd: CtrlOutput) -> None:
    if pi is None:
        return
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, cmd.left_pw)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, cmd.right_pw)
