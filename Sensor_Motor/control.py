"""Brake/servo control module for CanSat parafoil.

Sign convention: positive delta_arm_deg = right turn (right arm down, left arm up).
Angles: 0 deg = arm up, 160 deg = arm down, neutral = 80 deg.

Authority budget: FF claims up to DELTA_FF_MAX_DEG; PID trims within DELTA_PID_MAX_DEG.
Both sum to at most DELTA_TOTAL_MAX_DEG. Arm angles are slew-rate limited before pulse output.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

from lib import config, timebase


PARAFOIL_LEFT_MOTOR_PIN = config.PARAFOIL_LEFT_GPIO
PARAFOIL_RIGHT_MOTOR_PIN = config.PARAFOIL_RIGHT_GPIO

# Arm geometry in the up-zero frame.
ARM_MIN_DEG = 0.0
ARM_MAX_DEG = 160.0
ZERO_ARM_DEG = 0.0
NEUTRAL_ARM_DEG = 80.0
DELTA_ARM_MAX_DEG = 2.0 * min(
    NEUTRAL_ARM_DEG - ARM_MIN_DEG,
    ARM_MAX_DEG - NEUTRAL_ARM_DEG,
)

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

# Two-stage guidance timeout thresholds.
# 0 … ATTENUATE: normal. ATTENUATE … FAIL: 50 % command. > FAIL: neutral.
GUIDANCE_TIMEOUT_ATTENUATE_S = 0.5
GUIDANCE_TIMEOUT_FAIL_S = 1.5

# Gyro spike rejection threshold: beyond this the sample is discarded.
GYRO_SPIKE_LIMIT_DEG_S = 250.0

# Per-cycle integral decay factor when the gyro is unavailable.
INTEGRAL_DECAY_RATE = 0.95

V_MIN_MPS = 2.0


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


@dataclass
class ControlConfig:
    # Feedforward shaping
    YAW_RATE_CMD_MAX_DEG_S: float = 40.0       # command saturation ceiling
    YAW_RATE_DEADBAND_DEG_S: float = 3.0       # below this, FF output is zero to avoid dithering
    DELTA_FF_MAX_DEG: float = 115.0            # FF authority budget (leaves room for PID trim)
    DELTA_MIN_EFFECTIVE_DEG: float = 8.0       # minimum FF deflection above deadband
    EXPO: float = 0.8                          # <1 gives finer control near center

    # PID
    ERROR_DEADBAND_DEG_S: float = 2.0          # suppress trim chatter for small errors
    K_P: float = 0.35
    K_I: float = 0.02
    K_D: float = 0.0
    I_LIMIT_DEG: float = 25.0                  # anti-windup clamp on accumulated integral
    DELTA_PID_MAX_DEG: float = 45.0            # PID authority budget

    # Authority and slew
    DELTA_TOTAL_MAX_DEG: float = 160.0         # hard limit on FF + PID sum
    MAX_ARM_RATE_DEG_S: float = 100.0          # per-arm slew-rate limit


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
    # Accept nominal AND degraded guidance modes — both produce usable commands.
    # FAIL mode sets nominal=False, degraded=False; only that path yields valid=False.
    is_valid = bool(getattr(g_out, "nominal", False) or getattr(g_out, "degraded", False))
    return CtrlInput(
        yaw_rate_cmd_deg_s=math.degrees(float(getattr(g_out, "yaw_rate_cmd_rad_s", 0.0) or 0.0)),
        lat_acc_cmd_mps2=float(getattr(g_out, "lat_acc_cmd_mps2", 0.0) or 0.0),
        ground_speed_mps=float(getattr(g_out, "ground_speed_mps", 0.0) or 0.0),
        valid=is_valid,
        timestamp=float(getattr(g_out, "timestamp", now) or now),
    )


def yaw_rate_to_delta_ff(yaw_rate_cmd_deg_s: float, cfg: ControlConfig) -> float:
    """Map yaw-rate command to feedforward differential arm deflection.

    Expo curve gives proportional authority at small inputs and approaches
    DELTA_FF_MAX_DEG at full command, preserving DELTA_PID_MAX_DEG headroom for trim.
    Below DEADBAND returns zero so tiny commands don't cause dithering.
    Sign: positive = right turn (right arm down, left arm up).
    """
    clamped = _clamp(yaw_rate_cmd_deg_s, -cfg.YAW_RATE_CMD_MAX_DEG_S, cfg.YAW_RATE_CMD_MAX_DEG_S)
    if abs(clamped) < cfg.YAW_RATE_DEADBAND_DEG_S:
        return 0.0
    x = abs(clamped) / cfg.YAW_RATE_CMD_MAX_DEG_S
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
    yaw_rate_meas_deg_s: float,
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
    cfg = ctl.config

    yaw_rate_cmd = cmd.yaw_rate_cmd_deg_s
    if yaw_rate_cmd == 0.0 and cmd.lat_acc_cmd_mps2 != 0.0 and cmd.ground_speed_mps > 0.0:
        yaw_rate_cmd = math.degrees(cmd.lat_acc_cmd_mps2 / max(cmd.ground_speed_mps, V_MIN_MPS))

    age = timebase.age(now, cmd.timestamp)
    out.guidance_command_age_s = age

    # --- Two-stage guidance timeout ---
    if age > GUIDANCE_TIMEOUT_FAIL_S:
        # Command is too stale to trust; revert to neutral.
        out.mode = "GUIDANCE_TIMEOUT"
        out.fallback_mode = "GUIDANCE_TIMEOUT"
        out.valid = False
        out.left_angle_deg = NEUTRAL_ARM_DEG
        out.right_angle_deg = NEUTRAL_ARM_DEG
        out.left_pw = LEFT_NEUTRAL
        out.right_pw = RIGHT_NEUTRAL
        return out

    if age > GUIDANCE_TIMEOUT_ATTENUATE_S:
        # Stale but not dead: attenuate to limit uncommanded drift.
        yaw_rate_cmd *= 0.5
        out.fallback_mode = "GUIDANCE_ATTENUATED"
    else:
        out.fallback_mode = "NONE"

    out.yaw_rate_cmd_deg_s = yaw_rate_cmd

    # dt is shared by PID integration and slew-rate limit; always advances so
    # slew tracking stays accurate even when the gyro is temporarily unavailable.
    dt = timebase.clamp_dt(now, ctl.pid.prev_time, default_s=0.1, min_s=0.01, max_s=0.2)

    # --- Feedforward: expo-shaped, deadbanded ---
    delta_ff = yaw_rate_to_delta_ff(yaw_rate_cmd, cfg)

    # --- Gyro spike rejection ---
    # A single IMU glitch can saturate the integral in one cycle; discard the sample instead.
    gyro_spike = math.isfinite(yaw_rate_meas_deg_s) and abs(yaw_rate_meas_deg_s) > GYRO_SPIKE_LIMIT_DEG_S
    sensor_valid = math.isfinite(yaw_rate_meas_deg_s) and not gyro_spike
    out.sensor_valid = sensor_valid
    if gyro_spike:
        out.fallback_mode = "GYRO_SPIKE"

    # --- PID closed-loop trim (only when gyro measurement is valid) ---
    integral = ctl.pid.integral_deg
    delta_pid = 0.0
    error = 0.0

    if sensor_valid:
        out.yaw_rate_meas_deg_s = yaw_rate_meas_deg_s
        error = yaw_rate_cmd - yaw_rate_meas_deg_s
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
        out.yaw_rate_error_deg_s = error
        out.mode = "CLOSED_LOOP"
    else:
        # No gyro: FF only. Decay the integral so stale windup does not accumulate.
        integral = ctl.pid.integral_deg * INTEGRAL_DECAY_RATE
        out.mode = "FEEDFORWARD_ONLY"

    # --- Combine FF + PID and clamp total authority ---
    delta_sum = delta_ff + delta_pid
    saturated = abs(delta_sum) > cfg.DELTA_TOTAL_MAX_DEG
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
    if sensor_valid:
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
