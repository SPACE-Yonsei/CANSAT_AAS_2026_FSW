"""Brake/servo control module for CanSat parafoil.

Implements actuator mixer, PID brake controller, and pigpio servo interface.

Sign convention: negative yaw_rate_cmd turns left by lowering the left arm.
Units: _deg, _rad, _mps, _pwm suffixes.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Optional

from lib import config

# ── GPIO pins ─────────────────────────────────────────────────────────────────
PARAFOIL_LEFT_MOTOR_PIN  = config.PARAFOIL_LEFT_GPIO
PARAFOIL_RIGHT_MOTOR_PIN = config.PARAFOIL_RIGHT_GPIO

# ── Arm geometry (degrees) ────────────────────────────────────────────────────
# 0 deg = arm pointing down (earth), 180 deg = arm pointing up.
# STATE < 3 parks both arms at 180 deg. Active glide is centered at 100 deg
# and each arm is limited to +/-80 deg from neutral: 20..180 deg.
ARM_MIN_DEG        = 20.0
ARM_MAX_DEG        = 180.0
PARKED_ARM_DEG     = 180.0
NEUTRAL_ARM_DEG    = 100.0
DELTA_ARM_MAX_DEG  = 2.0 * (PARKED_ARM_DEG - NEUTRAL_ARM_DEG)
MAX_ARM_RATE_DEG_S = 60.0

# ── PWM mapping ───────────────────────────────────────────────────────────────
# LEFT  arm: 0 deg → LEFT_ZERO pulse,  180 deg → 2400 µs  (calibrated)
# RIGHT arm: 0 deg → RIGHT_ZERO pulse, 180 deg →  600 µs  (calibrated, mirrored)
# Derived: LEFT_ZERO = 2400 - 2000 = 400
#          RIGHT_ZERO = 600 + 2000 = 2600
LEFT_ZERO     = 400
RIGHT_ZERO    = 2600
PULSE_PER_DEG = 2000.0 / 180.0

# Operational neutral (STATE 3-4, guidance delta=0)  @ 100 deg
LEFT_NEUTRAL  = int(LEFT_ZERO  + NEUTRAL_ARM_DEG * PULSE_PER_DEG)   # ~1511
RIGHT_NEUTRAL = int(RIGHT_ZERO - NEUTRAL_ARM_DEG * PULSE_PER_DEG)   # ~1489

# Parked position (STATE < 3, set_neutral)  @ 180 deg
LEFT_PARKED   = int(LEFT_ZERO  + PARKED_ARM_DEG  * PULSE_PER_DEG)   # 2400
RIGHT_PARKED  = int(RIGHT_ZERO - PARKED_ARM_DEG  * PULSE_PER_DEG)   # 600

LEFT_MIN_PULSE  = int(LEFT_ZERO  + ARM_MIN_DEG * PULSE_PER_DEG)
LEFT_MAX_PULSE  = int(LEFT_ZERO  + ARM_MAX_DEG * PULSE_PER_DEG)
RIGHT_MIN_PULSE = int(RIGHT_ZERO - ARM_MAX_DEG * PULSE_PER_DEG)
RIGHT_MAX_PULSE = int(RIGHT_ZERO - ARM_MIN_DEG * PULSE_PER_DEG)

# ── Controller constants ──────────────────────────────────────────────────────
GUIDANCE_TIMEOUT_S = 0.5   # s — stale GuidanceCommand → neutral
V_MIN_MPS          = 2.0   # m/s — minimum speed for lat_acc→yaw_rate conversion


# ═══════════════════════════════════════════════════════════════════════════════
# Helper
# ═══════════════════════════════════════════════════════════════════════════════

def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


# ═══════════════════════════════════════════════════════════════════════════════
# Dataclasses
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class ControlConfig:
    K_FF: float            = 1.0
    K_P: float             = 0.0
    K_I: float             = 0.0
    K_D: float             = 0.0
    MAX_ARM_RATE_DEG_S: float = MAX_ARM_RATE_DEG_S


@dataclass
class _PIDState:
    integral_deg: float   = 0.0
    prev_error_deg: float = 0.0
    prev_time: float      = 0.0


@dataclass
class BrakeControllerState:
    config: ControlConfig           = field(default_factory=ControlConfig)
    pid: _PIDState                  = field(default_factory=_PIDState)
    prev_left_angle_deg: float      = NEUTRAL_ARM_DEG
    prev_right_angle_deg: float     = NEUTRAL_ARM_DEG


@dataclass
class GuidanceCommand:
    yaw_rate_cmd_deg_s: float  = 0.0
    lat_acc_cmd_mps2: float    = 0.0
    ground_speed_mps: float    = 0.0
    valid: bool                = False
    timestamp: float           = 0.0


@dataclass
class BrakeCommand:
    timestamp: float
    left_pw: int                    = LEFT_NEUTRAL
    right_pw: int                   = RIGHT_NEUTRAL
    left_angle_deg: float           = NEUTRAL_ARM_DEG
    right_angle_deg: float          = NEUTRAL_ARM_DEG
    delta_arm_deg: float            = 0.0
    delta_ff_deg: float             = 0.0
    delta_pid_deg: float            = 0.0
    yaw_rate_cmd_deg_s: float       = 0.0
    yaw_rate_meas_deg_s: float      = float("nan")
    yaw_rate_error_deg_s: float     = 0.0
    saturated: bool                 = False
    sensor_valid: bool              = False
    valid: bool                     = False
    mode: str                       = "NEUTRAL"
    fallback_mode: str              = "NEUTRAL"
    guidance_command_age_s: float   = 0.0


# ═══════════════════════════════════════════════════════════════════════════════
# Actuator mixer (feedforward-only, used by legacy control())
# ═══════════════════════════════════════════════════════════════════════════════

def actuator_mixer(yaw_rate_cmd_deg_s: float):
    """Map yaw rate command to PWM pulses.

    Returns (left_pw, right_pw, left_angle_deg, right_angle_deg, delta_arm_deg, offset).
    Negative yaw_rate turns left: left arm goes below neutral, right arm goes above neutral.
    """
    delta_arm_deg = _clamp(yaw_rate_cmd_deg_s, -DELTA_ARM_MAX_DEG, DELTA_ARM_MAX_DEG)
    left_angle    = _clamp(NEUTRAL_ARM_DEG + delta_arm_deg / 2, ARM_MIN_DEG, ARM_MAX_DEG)
    right_angle   = _clamp(NEUTRAL_ARM_DEG - delta_arm_deg / 2, ARM_MIN_DEG, ARM_MAX_DEG)
    left_pw  = int(LEFT_ZERO  + left_angle  * PULSE_PER_DEG)
    right_pw = int(RIGHT_ZERO - right_angle * PULSE_PER_DEG)
    left_pw  = int(_clamp(left_pw,  LEFT_MIN_PULSE, LEFT_MAX_PULSE))
    right_pw = int(_clamp(right_pw, RIGHT_MIN_PULSE, RIGHT_MAX_PULSE))
    offset   = 0.0
    return left_pw, right_pw, left_angle, right_angle, delta_arm_deg, offset


# ═══════════════════════════════════════════════════════════════════════════════
# Controller factory and functions
# ═══════════════════════════════════════════════════════════════════════════════

def make_controller_state(cfg: Optional[ControlConfig] = None) -> BrakeControllerState:
    return BrakeControllerState(config=cfg or ControlConfig())


def controller_reset(ctl: BrakeControllerState) -> None:
    ctl.pid                  = _PIDState()
    ctl.prev_left_angle_deg  = NEUTRAL_ARM_DEG
    ctl.prev_right_angle_deg = NEUTRAL_ARM_DEG


def controller_update(
    ctl: BrakeControllerState,
    cmd: GuidanceCommand,
    yaw_rate_meas_deg_s: float,
    now: float,
) -> BrakeCommand:
    """Compute brake servo command from GuidanceCommand + optional gyro feedback.

    yaw_rate_cmd_deg_s is mapped to arm difference in degrees. lat_acc_cmd_mps2
    is only used as a fallback when yaw_rate_cmd_deg_s is zero.
    Implements feedforward with optional PID terms. Default PID gains are zero,
    so the command maps directly to arm difference.
    """
    out = BrakeCommand(timestamp=now)

    # Use yaw-rate command directly. lat_acc is only a fallback when guidance did
    # not provide a yaw-rate command.
    yaw_rate_cmd = cmd.yaw_rate_cmd_deg_s
    if yaw_rate_cmd == 0.0 and cmd.lat_acc_cmd_mps2 != 0.0 and cmd.ground_speed_mps > 0.0:
        yaw_rate_cmd = math.degrees(
            cmd.lat_acc_cmd_mps2 / max(cmd.ground_speed_mps, V_MIN_MPS)
        )

    out.yaw_rate_cmd_deg_s    = yaw_rate_cmd
    out.guidance_command_age_s = now - cmd.timestamp

    # Guidance timeout — neutralize
    if out.guidance_command_age_s > GUIDANCE_TIMEOUT_S:
        out.mode         = "GUIDANCE_TIMEOUT"
        out.fallback_mode = "GUIDANCE_TIMEOUT"
        out.valid        = False
        out.left_angle_deg  = NEUTRAL_ARM_DEG
        out.right_angle_deg = NEUTRAL_ARM_DEG
        out.left_pw  = LEFT_NEUTRAL
        out.right_pw = RIGHT_NEUTRAL
        return out

    # Feedforward term
    delta_ff = ctl.config.K_FF * yaw_rate_cmd

    # PID (closed-loop)
    sensor_valid = math.isfinite(yaw_rate_meas_deg_s)
    out.sensor_valid = sensor_valid

    integral  = ctl.pid.integral_deg
    delta_pid = 0.0
    error     = 0.0

    if sensor_valid:
        out.yaw_rate_meas_deg_s = yaw_rate_meas_deg_s
        error = yaw_rate_cmd - yaw_rate_meas_deg_s
        dt    = (now - ctl.pid.prev_time) if ctl.pid.prev_time > 0 else 0.1
        dt    = max(dt, 1e-4)   # guard against zero dt
        integral   = ctl.pid.integral_deg + error * dt
        derivative = (error - ctl.pid.prev_error_deg) / dt
        delta_pid  = (
            ctl.config.K_P * error
            + ctl.config.K_I * integral
            + ctl.config.K_D * derivative
        )
        out.yaw_rate_error_deg_s = error
        out.mode = "CLOSED_LOOP"
    else:
        out.mode = "FEEDFORWARD_ONLY"

    delta_arm = delta_ff + delta_pid

    # Saturation check
    saturated = abs(delta_arm) > DELTA_ARM_MAX_DEG
    out.saturated = saturated

    if saturated:
        delta_arm = _clamp(delta_arm, -DELTA_ARM_MAX_DEG, DELTA_ARM_MAX_DEG)
        # Anti-windup: reset integrator on saturation
        if sensor_valid:
            integral = 0.0

    left_pw, right_pw, left_angle, right_angle, delta_arm, _ = actuator_mixer(delta_arm)

    out.delta_ff_deg    = delta_ff
    out.delta_pid_deg   = delta_pid
    out.delta_arm_deg   = delta_arm
    out.left_angle_deg  = left_angle
    out.right_angle_deg = right_angle
    out.left_pw         = left_pw
    out.right_pw        = right_pw
    out.valid           = True
    out.fallback_mode   = "NONE"

    # State update
    if sensor_valid and not saturated:
        ctl.pid.integral_deg   = integral
        ctl.pid.prev_error_deg = error
        ctl.pid.prev_time      = now
    elif sensor_valid and saturated:
        ctl.pid.integral_deg = 0.0   # anti-windup: reset on saturation
        ctl.pid.prev_time    = now

    ctl.prev_left_angle_deg  = left_angle
    ctl.prev_right_angle_deg = right_angle

    return out


# ═══════════════════════════════════════════════════════════════════════════════
# pigpio interface
# ═══════════════════════════════════════════════════════════════════════════════

def init_control():
    """Initialize pigpio and park servos (arms fully up). Returns pi handle or None."""
    try:
        import pigpio
        pi = pigpio.pi()
        if pi.connected:
            pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN,  LEFT_PARKED)
            pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, RIGHT_PARKED)
            return pi
    except Exception:
        pass
    return None


def set_neutral(pi) -> None:
    """Park arms at 180 deg (STATE < 3 or MOTOR_ENABLED=False)."""
    if pi is None:
        return
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN,  LEFT_PARKED)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, RIGHT_PARKED)


def set_motors_off(pi) -> None:
    if pi is None:
        return
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN,  0)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)


def set_brake_command(pi, cmd: BrakeCommand) -> None:
    if pi is None:
        return
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN,  cmd.left_pw)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, cmd.right_pw)


def terminate_control(pi) -> None:
    if pi is None:
        return
    try:
        pi.stop()
    except Exception:
        pass


def control(pi, yaw_rate_cmd_deg_s: float):
    """Legacy single-call feedforward control function.

    Returns SimpleNamespace(left_pulse, right_pulse, expected_yaw_rate).
    """
    left_pw, right_pw, left_angle, right_angle, delta, _ = actuator_mixer(yaw_rate_cmd_deg_s)
    if pi is not None:
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN,  left_pw)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_pw)
    return SimpleNamespace(
        left_pulse=left_pw,
        right_pulse=right_pw,
        expected_yaw_rate=yaw_rate_cmd_deg_s,
    )
