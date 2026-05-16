"""Brake/servo control module for CanSat parafoil.

Sign convention: negative yaw_rate_cmd turns left by lowering the left arm.
Angles use the motor-arm frame requested for this project:
0 deg = arm pointing up, and angle increases as the arm moves down.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

from lib import config


PARAFOIL_LEFT_MOTOR_PIN = config.PARAFOIL_LEFT_GPIO
PARAFOIL_RIGHT_MOTOR_PIN = config.PARAFOIL_RIGHT_GPIO

# Arm geometry in the new up-zero frame.
ARM_MIN_DEG = 0.0
ARM_MAX_DEG = 160.0
ZERO_ARM_DEG = 0.0
NEUTRAL_ARM_DEG = 80.0
DELTA_ARM_MAX_DEG = 2.0 * min(
    NEUTRAL_ARM_DEG - ARM_MIN_DEG,
    ARM_MAX_DEG - NEUTRAL_ARM_DEG,
)
MAX_ARM_RATE_DEG_S = 60.0

# PWM mapping. Zero means the arm points up.
LEFT_ZERO = 2480
RIGHT_ZERO = 636
PULSE_PER_DEG = 2000.0 / 180.0

LEFT_NEUTRAL = int(LEFT_ZERO - NEUTRAL_ARM_DEG * PULSE_PER_DEG)
RIGHT_NEUTRAL = int(RIGHT_ZERO + NEUTRAL_ARM_DEG * PULSE_PER_DEG)

LEFT_ZERO_PULSE = int(LEFT_ZERO)
RIGHT_ZERO_PULSE = int(RIGHT_ZERO)

LEFT_MIN_PULSE = int(LEFT_ZERO - ARM_MAX_DEG * PULSE_PER_DEG)
LEFT_MAX_PULSE = int(LEFT_ZERO - ARM_MIN_DEG * PULSE_PER_DEG)
RIGHT_MIN_PULSE = int(RIGHT_ZERO + ARM_MIN_DEG * PULSE_PER_DEG)
RIGHT_MAX_PULSE = int(RIGHT_ZERO + ARM_MAX_DEG * PULSE_PER_DEG)

GUIDANCE_TIMEOUT_S = 0.5
V_MIN_MPS = 2.0


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


@dataclass
class ControlConfig:
    K_FF: float = 1.0
    K_P: float = 0.0
    K_I: float = 0.0
    K_D: float = 0.0
    MAX_ARM_RATE_DEG_S: float = MAX_ARM_RATE_DEG_S


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
    yaw_rate_cmd_deg_s: float = 0.0
    lat_acc_cmd_mps2: float = 0.0
    ground_speed_mps: float = 0.0
    valid: bool = False
    timestamp: float = 0.0


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
    yaw_rate_cmd_deg_s: float = 0.0
    yaw_rate_meas_deg_s: float = float("nan")
    yaw_rate_error_deg_s: float = 0.0
    saturated: bool = False
    sensor_valid: bool = False
    valid: bool = False
    mode: str = "NEUTRAL"
    fallback_mode: str = "NEUTRAL"
    guidance_command_age_s: float = 0.0


def WriteNeutral(now: float, mode: str = "NEUTRAL") -> CtrlOutput:
    """Produce a neutral PWM command object without touching hardware."""
    cmd = CtrlOutput(timestamp=now)
    cmd.mode = mode
    cmd.fallback_mode = mode
    return cmd


def ProduceCtrlInput(g_out, now: float) -> CtrlInput:
    return CtrlInput(
        yaw_rate_cmd_deg_s=math.degrees(float(getattr(g_out, "yaw_rate_cmd_rad_s", 0.0) or 0.0)),
        lat_acc_cmd_mps2=float(getattr(g_out, "lat_acc_cmd_mps2", 0.0) or 0.0),
        ground_speed_mps=float(getattr(g_out, "ground_speed_mps", 0.0) or 0.0),
        valid=bool(getattr(g_out, "nominal", False)),
        timestamp=float(getattr(g_out, "timestamp", now) or now),
    )


def ConnectRoMo(yaw_rate_cmd_deg_s: float):
    """Map yaw-rate command to right/left motor pulses.

    The current control assumption is 1 deg/s yaw-rate command maps to
    1 deg differential arm command. Negative yaw turns left, so the left arm
    moves down and the right arm moves up.
    """
    delta_arm_deg = _clamp(yaw_rate_cmd_deg_s, -DELTA_ARM_MAX_DEG, DELTA_ARM_MAX_DEG)
    left_angle = _clamp(NEUTRAL_ARM_DEG - delta_arm_deg / 2.0, ARM_MIN_DEG, ARM_MAX_DEG)
    right_angle = _clamp(NEUTRAL_ARM_DEG + delta_arm_deg / 2.0, ARM_MIN_DEG, ARM_MAX_DEG)

    left_pw = int(LEFT_ZERO - left_angle * PULSE_PER_DEG)
    right_pw = int(RIGHT_ZERO + right_angle * PULSE_PER_DEG)
    left_pw = int(_clamp(left_pw, LEFT_MIN_PULSE, LEFT_MAX_PULSE))
    right_pw = int(_clamp(right_pw, RIGHT_MIN_PULSE, RIGHT_MAX_PULSE))

    return left_pw, right_pw, left_angle, right_angle, delta_arm_deg


def MakeCtrler(cfg: Optional[ControlConfig] = None) -> Ctrler:
    return Ctrler(config=cfg or ControlConfig())


def controller_reset(ctl: Ctrler) -> None:
    ctl.pid = _PIDState()
    ctl.prev_left_angle_deg = NEUTRAL_ARM_DEG
    ctl.prev_right_angle_deg = NEUTRAL_ARM_DEG


def ProduceCtrlOutput(
    ctl: Ctrler,
    cmd: CtrlInput,
    yaw_rate_meas_deg_s: float,
    now: float,
) -> CtrlOutput:
    """Compute brake servo command from GuidanceCommand + optional gyro feedback."""
    out = CtrlOutput(timestamp=now)

    yaw_rate_cmd = cmd.yaw_rate_cmd_deg_s
    if yaw_rate_cmd == 0.0 and cmd.lat_acc_cmd_mps2 != 0.0 and cmd.ground_speed_mps > 0.0:
        yaw_rate_cmd = math.degrees(
            cmd.lat_acc_cmd_mps2 / max(cmd.ground_speed_mps, V_MIN_MPS)
        )

    out.yaw_rate_cmd_deg_s = yaw_rate_cmd
    out.guidance_command_age_s = now - cmd.timestamp

    if out.guidance_command_age_s > GUIDANCE_TIMEOUT_S:
        out.mode = "GUIDANCE_TIMEOUT"
        out.fallback_mode = "GUIDANCE_TIMEOUT"
        out.valid = False
        out.left_angle_deg = NEUTRAL_ARM_DEG
        out.right_angle_deg = NEUTRAL_ARM_DEG
        out.left_pw = LEFT_NEUTRAL
        out.right_pw = RIGHT_NEUTRAL
        return out

    delta_ff = ctl.config.K_FF * yaw_rate_cmd

    sensor_valid = math.isfinite(yaw_rate_meas_deg_s)
    out.sensor_valid = sensor_valid

    integral = ctl.pid.integral_deg
    delta_pid = 0.0
    error = 0.0

    if sensor_valid:
        out.yaw_rate_meas_deg_s = yaw_rate_meas_deg_s
        error = yaw_rate_cmd - yaw_rate_meas_deg_s
        dt = (now - ctl.pid.prev_time) if ctl.pid.prev_time > 0 else 0.1
        dt = max(dt, 1e-4)
        integral = ctl.pid.integral_deg + error * dt
        derivative = (error - ctl.pid.prev_error_deg) / dt
        delta_pid = (
            ctl.config.K_P * error
            + ctl.config.K_I * integral
            + ctl.config.K_D * derivative
        )
        out.yaw_rate_error_deg_s = error
        out.mode = "CLOSED_LOOP"
    else:
        out.mode = "FEEDFORWARD_ONLY"

    delta_arm = delta_ff + delta_pid
    saturated = abs(delta_arm) > DELTA_ARM_MAX_DEG
    out.saturated = saturated

    if saturated:
        delta_arm = _clamp(delta_arm, -DELTA_ARM_MAX_DEG, DELTA_ARM_MAX_DEG)
        if sensor_valid:
            integral = 0.0

    left_pw, right_pw, left_angle, right_angle, delta_arm = ConnectRoMo(delta_arm)

    out.delta_ff_deg = delta_ff
    out.delta_pid_deg = delta_pid
    out.delta_arm_deg = delta_arm
    out.left_angle_deg = left_angle
    out.right_angle_deg = right_angle
    out.left_pw = left_pw
    out.right_pw = right_pw
    out.valid = True
    out.fallback_mode = "NONE"

    if sensor_valid and not saturated:
        ctl.pid.integral_deg = integral
        ctl.pid.prev_error_deg = error
        ctl.pid.prev_time = now
    elif sensor_valid and saturated:
        ctl.pid.integral_deg = 0.0
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
            return pi
    except Exception:
        pass
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
