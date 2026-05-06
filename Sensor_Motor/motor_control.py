#!/usr/bin/env python3
"""Yaw-rate control, parafoil arm mixing, and servo PWM output.

Control chain:
  L1 guidance command -> yaw-rate FF/PID -> arm differential mixer -> PWM

Sign convention:
  yaw_rate_cmd_deg_s > 0 = right turn
  delta_arm_deg > 0      = right turn
  right turn             = left arm up, right arm down

Arm model:
  left_angle_deg  = NEUTRAL_ARM_DEG + delta_arm_deg / 2
  right_angle_deg = NEUTRAL_ARM_DEG - delta_arm_deg / 2
"""
from __future__ import annotations

import logging
import math
import sys
import time
from dataclasses import dataclass
from types import SimpleNamespace
from typing import Optional

try:
    import pigpio as _pigpio_module
except ImportError:
    _pigpio_module = None

logger = logging.getLogger(__name__)
_WARNED_FALLBACKS: set[str] = set()

# Hardware pins
PARAFOIL_LEFT_MOTOR_PIN: int = 13   # GPIO BCM pin
PARAFOIL_RIGHT_MOTOR_PIN: int = 12  # GPIO BCM pin
LEFT_GPIO = PARAFOIL_LEFT_MOTOR_PIN
RIGHT_GPIO = PARAFOIL_RIGHT_MOTOR_PIN

# Servo calibration: 0 deg is arm down, 180 deg is arm up.
PULSE_PER_DEG: float = 2000.0 / 180.0  # us/deg, fixed servo calibration
LEFT_ZERO: int = 600                   # us, servo 0 deg pulse width
RIGHT_ZERO: int = 2500                 # us, servo 0 deg pulse width

ARM_MIN_DEG = 0.0
ARM_MAX_DEG = 180.0
NEUTRAL_ARM_DEG = 60.0
DELTA_ARM_MAX_DEG = 60.0
MAX_ARM_RATE_DEG_S = 60.0
MIN_GROUND_SPEED_MPS = 1.0

LEFT_MIN = int(LEFT_ZERO + ARM_MIN_DEG * PULSE_PER_DEG)
LEFT_MAX = int(LEFT_ZERO + ARM_MAX_DEG * PULSE_PER_DEG)
RIGHT_MIN = int(RIGHT_ZERO - ARM_MAX_DEG * PULSE_PER_DEG)
RIGHT_MAX = int(RIGHT_ZERO - ARM_MIN_DEG * PULSE_PER_DEG)
LEFT_NEUTRAL = int(LEFT_ZERO + NEUTRAL_ARM_DEG * PULSE_PER_DEG)
RIGHT_NEUTRAL = int(RIGHT_ZERO - NEUTRAL_ARM_DEG * PULSE_PER_DEG)

# Legacy names used by older tests/replay tools.
PULSE_MIN = min(LEFT_MIN, RIGHT_MIN)
PULSE_MAX = max(LEFT_MAX, RIGHT_MAX)
LEFT_MAX_PULSE = LEFT_MAX
RIGHT_MIN_PULSE = RIGHT_MIN


@dataclass
class ControlConfig:
    """Conservative yaw-rate controller parameters."""
    K_FF: float = 1.0       # deg arm difference / (deg/s yaw-rate command)
    K_P: float = 0.10       # deg arm difference / (deg/s yaw-rate error)
    K_I: float = 0.0
    K_D: float = 0.0
    I_MAX_DEG: float = 10.0
    DELTA_ARM_MAX_DEG: float = DELTA_ARM_MAX_DEG
    ARM_MIN_DEG: float = ARM_MIN_DEG
    ARM_MAX_DEG: float = ARM_MAX_DEG
    NEUTRAL_ARM_DEG: float = NEUTRAL_ARM_DEG
    MAX_ARM_RATE_DEG_S: float = MAX_ARM_RATE_DEG_S
    MIN_GROUND_SPEED_MPS: float = MIN_GROUND_SPEED_MPS
    COMMAND_TIMEOUT_S: float = 0.5
    DT_MIN_S: float = 0.01
    DT_MAX_S: float = 0.5
    FEEDFORWARD_ONLY_ON_INVALID_IMU: bool = True


@dataclass
class GuidanceCommand:
    yaw_rate_cmd_deg_s: Optional[float] = None
    lat_acc_cmd_mps2: Optional[float] = None
    ground_speed_mps: Optional[float] = None
    target_bearing_deg: Optional[float] = None
    valid: bool = False
    timestamp: float = 0.0


@dataclass
class BrakeCommand:
    timestamp: float
    mode: str = "NEUTRAL"
    valid: bool = False
    left_angle_deg: float = NEUTRAL_ARM_DEG
    right_angle_deg: float = NEUTRAL_ARM_DEG
    delta_arm_deg: float = 0.0
    yaw_rate_cmd_deg_s: float = 0.0
    yaw_rate_meas_deg_s: float = 0.0
    yaw_rate_error_deg_s: float = 0.0
    delta_ff_deg: float = 0.0
    delta_pid_deg: float = 0.0
    pid_p: float = 0.0
    pid_i: float = 0.0
    pid_d: float = 0.0
    saturated: bool = False
    sensor_valid: bool = False
    guidance_command_age_s: float = math.inf
    fallback_mode: str = ""
    left_pw: int = LEFT_NEUTRAL
    right_pw: int = RIGHT_NEUTRAL

    @property
    def left_pulse(self) -> int:
        return self.left_pw

    @property
    def right_pulse(self) -> int:
        return self.right_pw

    @property
    def diffBrake(self) -> float:
        return _clamp(self.delta_arm_deg / DELTA_ARM_MAX_DEG, -1.0, 1.0)

    @property
    def baseBrake(self) -> float:
        return 0.0


class YawRatePID:
    def __init__(self, config: ControlConfig) -> None:
        self.cfg = config
        self.integral_deg = 0.0
        self.prev_error: Optional[float] = None

    def reset(self) -> None:
        self.integral_deg = 0.0
        self.prev_error = None

    def update(self, error_deg_s: float, dt: float, allow_integrator: bool) -> tuple[float, float, float, float]:
        p = self.cfg.K_P * error_deg_s

        if allow_integrator and self.cfg.K_I != 0.0:
            self.integral_deg = _clamp(
                self.integral_deg + error_deg_s * dt * self.cfg.K_I,
                -self.cfg.I_MAX_DEG,
                self.cfg.I_MAX_DEG,
            )
        i = self.integral_deg

        if self.prev_error is None or self.cfg.K_D == 0.0:
            d = 0.0
        else:
            d = self.cfg.K_D * (error_deg_s - self.prev_error) / dt
        self.prev_error = error_deg_s

        return p + i + d, p, i, d


class ParafoilBrakeController:
    """Converts L1 yaw-rate/lateral-accel commands into servo arm targets."""

    def __init__(self, config: Optional[ControlConfig] = None) -> None:
        self.cfg = config or ControlConfig()
        self.pid = YawRatePID(self.cfg)
        self._last_ts: Optional[float] = None
        self._left_angle = self.cfg.NEUTRAL_ARM_DEG
        self._right_angle = self.cfg.NEUTRAL_ARM_DEG

    def reset(self) -> None:
        self.pid.reset()
        self._last_ts = None
        self._left_angle = self.cfg.NEUTRAL_ARM_DEG
        self._right_angle = self.cfg.NEUTRAL_ARM_DEG

    def update(
        self,
        guidance_cmd: GuidanceCommand | float,
        yaw_rate_meas_deg_s: Optional[float],
        now: float,
    ) -> BrakeCommand:
        dt, dt_valid = self._tick(now)
        cmd = self._coerce_guidance_cmd(guidance_cmd, now)
        out = BrakeCommand(timestamp=now)
        out.guidance_command_age_s = now - cmd.timestamp if cmd.timestamp else math.inf

        if not cmd.valid:
            return self._fallback(out, "GUIDANCE_INVALID", "guidance invalid")
        if out.guidance_command_age_s > self.cfg.COMMAND_TIMEOUT_S:
            return self._fallback(out, "GUIDANCE_TIMEOUT", "guidance command timeout")

        yaw_cmd = self._resolve_yaw_rate_cmd(cmd)
        if yaw_cmd is None:
            return self._fallback(out, "COMMAND_INVALID", "cannot resolve yaw-rate command")

        meas_valid = _is_finite(yaw_rate_meas_deg_s)
        out.sensor_valid = meas_valid
        if not meas_valid and not self.cfg.FEEDFORWARD_ONLY_ON_INVALID_IMU:
            return self._fallback(out, "IMU_INVALID", "yaw-rate measurement invalid")

        yaw_meas = float(yaw_rate_meas_deg_s) if meas_valid else 0.0
        error = yaw_cmd - yaw_meas
        delta_ff = self.cfg.K_FF * yaw_cmd

        if meas_valid and dt_valid:
            # First calculate without adding I. If actuator is saturated and error
            # drives further into saturation, hold the integrator.
            preview_pid, _, _, _ = self.pid.update(error, dt, allow_integrator=False)
            preview = delta_ff + preview_pid
            preview_sat = abs(preview) >= self.cfg.DELTA_ARM_MAX_DEG
            allow_i = not preview_sat or (preview * error < 0.0)
            delta_pid, pid_p, pid_i, pid_d = self.pid.update(error, dt, allow_integrator=allow_i)
            mode = "CLOSED_LOOP"
            fallback = ""
        else:
            self.pid.prev_error = None
            delta_pid = pid_p = pid_i = pid_d = 0.0
            mode = "FEEDFORWARD_ONLY"
            fallback = "imu invalid" if not meas_valid else "dt invalid"
            if fallback:
                _log_fallback_once(fallback)

        raw_delta = delta_ff + delta_pid
        delta_arm = _clamp(raw_delta, -self.cfg.DELTA_ARM_MAX_DEG, self.cfg.DELTA_ARM_MAX_DEG)
        saturated = abs(delta_arm - raw_delta) > 1e-9

        left_target = _clamp(
            self.cfg.NEUTRAL_ARM_DEG + delta_arm / 2.0,
            self.cfg.ARM_MIN_DEG,
            self.cfg.ARM_MAX_DEG,
        )
        right_target = _clamp(
            self.cfg.NEUTRAL_ARM_DEG - delta_arm / 2.0,
            self.cfg.ARM_MIN_DEG,
            self.cfg.ARM_MAX_DEG,
        )

        left_angle = self._slew(self._left_angle, left_target, dt)
        right_angle = self._slew(self._right_angle, right_target, dt)
        self._left_angle = left_angle
        self._right_angle = right_angle

        out.mode = mode
        out.valid = True
        out.left_angle_deg = left_angle
        out.right_angle_deg = right_angle
        out.delta_arm_deg = left_angle - right_angle
        out.yaw_rate_cmd_deg_s = yaw_cmd
        out.yaw_rate_meas_deg_s = yaw_meas
        out.yaw_rate_error_deg_s = error
        out.delta_ff_deg = delta_ff
        out.delta_pid_deg = delta_pid
        out.pid_p = pid_p
        out.pid_i = pid_i
        out.pid_d = pid_d
        out.saturated = saturated
        out.fallback_mode = fallback
        out.left_pw, out.right_pw = angles_to_pwm(left_angle, right_angle)
        _log_command(out)
        return out

    def _coerce_guidance_cmd(self, guidance_cmd: GuidanceCommand | float, now: float) -> GuidanceCommand:
        if isinstance(guidance_cmd, GuidanceCommand):
            return guidance_cmd
        # Legacy API: numeric command is rad/s course-rate command.
        if _is_finite(guidance_cmd):
            return GuidanceCommand(
                yaw_rate_cmd_deg_s=math.degrees(float(guidance_cmd)),
                valid=True,
                timestamp=now,
            )
        return GuidanceCommand(valid=False, timestamp=now)

    def _resolve_yaw_rate_cmd(self, cmd: GuidanceCommand) -> Optional[float]:
        if _is_finite(cmd.yaw_rate_cmd_deg_s):
            return float(cmd.yaw_rate_cmd_deg_s)
        if not _is_finite(cmd.lat_acc_cmd_mps2):
            return None
        if not _is_finite(cmd.ground_speed_mps):
            _log_fallback_once("invalid ground speed for lat-acc command")
            return None
        gs = max(float(cmd.ground_speed_mps), self.cfg.MIN_GROUND_SPEED_MPS)
        return math.degrees(float(cmd.lat_acc_cmd_mps2) / gs)

    def _fallback(self, out: BrakeCommand, mode: str, reason: str) -> BrakeCommand:
        _log_fallback_once(reason)
        self.pid.reset()
        self._left_angle = self.cfg.NEUTRAL_ARM_DEG
        self._right_angle = self.cfg.NEUTRAL_ARM_DEG
        out.mode = mode
        out.valid = False
        out.fallback_mode = reason
        out.left_angle_deg = self.cfg.NEUTRAL_ARM_DEG
        out.right_angle_deg = self.cfg.NEUTRAL_ARM_DEG
        out.left_pw, out.right_pw = angles_to_pwm(out.left_angle_deg, out.right_angle_deg)
        _log_command(out)
        return out

    def _tick(self, now: float) -> tuple[float, bool]:
        if self._last_ts is None:
            self._last_ts = now
            return 0.1, True
        dt = now - self._last_ts
        self._last_ts = now
        if dt < self.cfg.DT_MIN_S or dt > self.cfg.DT_MAX_S:
            return _clamp(dt, self.cfg.DT_MIN_S, self.cfg.DT_MAX_S), False
        return dt, True

    def _slew(self, previous: float, target: float, dt: float) -> float:
        max_step = self.cfg.MAX_ARM_RATE_DEG_S * max(dt, self.cfg.DT_MIN_S)
        return previous + _clamp(target - previous, -max_step, max_step)


def angles_to_pwm(left_angle_deg: float, right_angle_deg: float) -> tuple[int, int]:
    left = _clamp(left_angle_deg, ARM_MIN_DEG, ARM_MAX_DEG)
    right = _clamp(right_angle_deg, ARM_MIN_DEG, ARM_MAX_DEG)
    left_pw = _clamp_pw(LEFT_ZERO + left * PULSE_PER_DEG, LEFT_MIN, LEFT_MAX)
    right_pw = _clamp_pw(RIGHT_ZERO - right * PULSE_PER_DEG, RIGHT_MIN, RIGHT_MAX)
    return left_pw, right_pw


def actuator_mixer(yaw_rate_cmd_deg_s: float) -> tuple:
    delta = _clamp(ControlConfig().K_FF * yaw_rate_cmd_deg_s, -DELTA_ARM_MAX_DEG, DELTA_ARM_MAX_DEG)
    left_angle = _clamp(NEUTRAL_ARM_DEG + delta / 2.0, ARM_MIN_DEG, ARM_MAX_DEG)
    right_angle = _clamp(NEUTRAL_ARM_DEG - delta / 2.0, ARM_MIN_DEG, ARM_MAX_DEG)
    left_pw, right_pw = angles_to_pwm(left_angle, right_angle)
    return left_pw, right_pw, left_angle, right_angle, delta, delta


_legacy_controller = ParafoilBrakeController(ControlConfig())


def control(pi, courseRateCmd: float, yawRateMeas: Optional[float] = None):
    """Legacy compatibility API.

    `courseRateCmd` and `yawRateMeas` are rad/s for existing simulation callers.
    New runtime code should use ParafoilBrakeController.update(GuidanceCommand).
    """
    yaw_meas_deg = math.degrees(yawRateMeas) if _is_finite(yawRateMeas) else None
    cmd = _legacy_controller.update(courseRateCmd, yaw_meas_deg, time.time())
    set_brake_command(pi, cmd)
    return SimpleNamespace(
        left_pulse=cmd.left_pw,
        right_pulse=cmd.right_pw,
        left_cmd_deg=cmd.left_angle_deg,
        right_cmd_deg=cmd.right_angle_deg,
        actual_delta_deg=-cmd.delta_arm_deg,
        expected_yaw_rate=courseRateCmd,
        diffBrake=cmd.diffBrake,
        baseBrake=cmd.baseBrake,
    )


def init_control():
    pigpio_mod = sys.modules.get("pigpio", _pigpio_module)
    if pigpio_mod is None:
        return None
    pi = pigpio_mod.pi()
    if not pi.connected:
        return None
    set_neutral(pi)
    return pi


def set_neutral(pi) -> None:
    if pi is None:
        return
    pi.set_servo_pulsewidth(LEFT_GPIO, LEFT_NEUTRAL)
    pi.set_servo_pulsewidth(RIGHT_GPIO, RIGHT_NEUTRAL)


def set_motors_off(pi) -> None:
    if pi is None:
        return
    pi.set_servo_pulsewidth(LEFT_GPIO, 0)
    pi.set_servo_pulsewidth(RIGHT_GPIO, 0)


def set_brake_command(pi, cmd: BrakeCommand) -> None:
    if pi is None:
        return
    pi.set_servo_pulsewidth(LEFT_GPIO, cmd.left_pw)
    pi.set_servo_pulsewidth(RIGHT_GPIO, cmd.right_pw)


def terminate_control(pi) -> None:
    if pi is None:
        return
    set_motors_off(pi)
    pi.stop()


def _is_finite(value: object) -> bool:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return False
    return math.isfinite(v)


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else (hi if x > hi else x)


def _clamp_pw(pw: float, lo: int, hi: int) -> int:
    return max(lo, min(hi, int(pw)))


def _log_command(cmd: BrakeCommand) -> None:
    logger.info(
        "parafoil_control mode=%s valid=%s yaw_cmd=%.3f yaw_meas=%.3f "
        "yaw_err=%.3f delta_ff=%.3f delta_pid=%.3f delta=%.3f "
        "left=%.2f right=%.2f sat=%s fallback=%s sensor_valid=%s age=%.3f",
        cmd.mode,
        cmd.valid,
        cmd.yaw_rate_cmd_deg_s,
        cmd.yaw_rate_meas_deg_s,
        cmd.yaw_rate_error_deg_s,
        cmd.delta_ff_deg,
        cmd.delta_pid_deg,
        cmd.delta_arm_deg,
        cmd.left_angle_deg,
        cmd.right_angle_deg,
        cmd.saturated,
        cmd.fallback_mode,
        cmd.sensor_valid,
        cmd.guidance_command_age_s,
    )


def _log_fallback_once(reason: str) -> None:
    if reason in _WARNED_FALLBACKS:
        return
    _WARNED_FALLBACKS.add(reason)
    logger.warning("Parafoil control fallback: %s", reason)
