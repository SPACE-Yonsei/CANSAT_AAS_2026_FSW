#!/usr/bin/env python3
"""
ParafoilBrakeController and servo PWM mapping.

Controller: PI course-rate feedback → differential brake [-1, 1].
Servo:      differential brake + base brake → left/right PWM.

Sign convention (consistent with L1Guidance):
  courseRateCmd > 0  →  right turn  →  diffBrake > 0  →  right brake pulled
  courseRateCmd < 0  →  left turn   →  diffBrake < 0  →  left brake pulled
"""
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

# ── Servo hardware config ──────────────────────────────────────────────────────
LEFT_GPIO  = 13
RIGHT_GPIO = 12

LEFT_NEUTRAL  = 1720   # µs
RIGHT_NEUTRAL = 1780   # µs

LEFT_MIN  = 600
LEFT_MAX  = 2120
RIGHT_MIN = 880
RIGHT_MAX = 2500

# Symmetric max deflection (conservative: smaller of the two sides)
_LEFT_MAX_DEFLECT  = min(LEFT_NEUTRAL  - LEFT_MIN,  LEFT_MAX  - LEFT_NEUTRAL)   # 400 µs
_RIGHT_MAX_DEFLECT = min(RIGHT_NEUTRAL - RIGHT_MIN, RIGHT_MAX - RIGHT_NEUTRAL)  # 720 µs
_DEFLECT_PW        = min(_LEFT_MAX_DEFLECT, _RIGHT_MAX_DEFLECT)                 # 400 µs

# Backward-compatible names used by tests and replay tools.
PARAFOIL_LEFT_MOTOR_PIN = LEFT_GPIO
PARAFOIL_RIGHT_MOTOR_PIN = RIGHT_GPIO
PULSE_MIN = LEFT_MIN
PULSE_MAX = RIGHT_MAX
LEFT_MAX_PULSE = LEFT_MAX
RIGHT_MIN_PULSE = RIGHT_MIN


# ── Data classes ───────────────────────────────────────────────────────────────
@dataclass
class ControlConfig:
    """Tunable PI controller parameters."""
    Kp:           float = 0.8    # (rad/s error) → diffBrake
    Ki:           float = 0.15   # 1/s
    MAX_INTEGRAL: float = 0.5    # anti-windup clamp [diffBrake·s]
    MAX_ACCEL:    float = 4.0    # slew rate [diffBrake/s]
    BASE_BRAKE:   float = 0.0    # symmetric brake offset [0, 1]


@dataclass
class BrakeCommand:
    """Complete actuator command produced by ParafoilBrakeController."""
    timestamp:  float
    diffBrake:  float = 0.0          # [-1, 1], + = right turn
    baseBrake:  float = 0.0          # [0, 1], symmetric
    left_pw:    int   = LEFT_NEUTRAL  # µs
    right_pw:   int   = RIGHT_NEUTRAL # µs


# ── ParafoilBrakeController ────────────────────────────────────────────────────
class ParafoilBrakeController:
    """
    PI controller: courseRateCmd [rad/s] → diffBrake [-1, 1].

    yawRateMeas = gz converted to rad/s (done upstream in NavigationStateEstimator).
    When yawRateMeas is None, falls back to open-loop proportional.
    """

    def __init__(self, config: Optional[ControlConfig] = None) -> None:
        self.cfg              = config or ControlConfig()
        self._integral:  float           = 0.0
        self._last_cmd:  float           = 0.0
        self._last_ts:   Optional[float] = None

    def update(
        self,
        courseRateCmd: float,
        yawRateMeas:   Optional[float],
        now: float,
    ) -> BrakeCommand:
        cmd = BrakeCommand(timestamp=now)
        dt  = self._tick(now)

        if yawRateMeas is None:
            raw = self.cfg.Kp * courseRateCmd
        else:
            error = courseRateCmd - yawRateMeas
            u     = self.cfg.Kp * error + self.cfg.Ki * self._integral

            # Anti-windup: accumulate only when unsaturated or error opposes saturation
            if abs(u) < 1.0 or error * u < 0.0:
                self._integral = _clamp(
                    self._integral + error * dt,
                    -self.cfg.MAX_INTEGRAL, self.cfg.MAX_INTEGRAL,
                )
                u = self.cfg.Kp * error + self.cfg.Ki * self._integral

            raw = u

        diffBrake = self._slew(_clamp(raw, -1.0, 1.0), dt)

        cmd.diffBrake = diffBrake
        cmd.baseBrake = self.cfg.BASE_BRAKE
        cmd.left_pw, cmd.right_pw = _brake_to_pw(diffBrake, self.cfg.BASE_BRAKE)
        return cmd

    def reset(self) -> None:
        self._integral = 0.0
        self._last_cmd = 0.0
        self._last_ts  = None

    def _tick(self, now: float) -> float:
        if self._last_ts is None:
            self._last_ts = now
            return 0.1
        dt = _clamp(now - self._last_ts, 0.02, 0.5)
        self._last_ts = now
        return dt

    def _slew(self, target: float, dt: float) -> float:
        delta     = target - self._last_cmd
        max_delta = self.cfg.MAX_ACCEL * dt
        result    = self._last_cmd + _clamp(delta, -max_delta, max_delta)
        self._last_cmd = result
        return result


# ── Brake → PWM mapping ────────────────────────────────────────────────────────
def _brake_to_pw(diffBrake: float, baseBrake: float) -> tuple:
    """
    diffBrake > 0  →  right turn  →  right brake pulled, left released.
    baseBrake      →  symmetric pull on both brakes (speed / flare control).
    """
    base_off  = int(baseBrake  * _DEFLECT_PW)
    right_off = int( diffBrake * _DEFLECT_PW) + base_off
    left_off  = int(-diffBrake * _DEFLECT_PW) + base_off

    left_pw  = _clamp_pw(LEFT_NEUTRAL  + left_off,  LEFT_MIN,  LEFT_MAX)
    right_pw = _clamp_pw(RIGHT_NEUTRAL + right_off, RIGHT_MIN, RIGHT_MAX)
    return left_pw, right_pw


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else (hi if x > hi else x)


def _clamp_pw(pw: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, int(pw)))


_legacy_controller = ParafoilBrakeController(ControlConfig())


def actuator_mixer(courseRateCmd: float) -> tuple:
    """Stateless compatibility mixer."""
    diffBrake = _clamp(ControlConfig().Kp * courseRateCmd, -1.0, 1.0)
    left_pw, right_pw = _brake_to_pw(diffBrake, 0.0)
    pulse_offset = diffBrake * _DEFLECT_PW
    return left_pw, right_pw, 0.0, 0.0, diffBrake, pulse_offset


def control(pi, courseRateCmd: float, yawRateMeas: Optional[float] = None):
    cmd = _legacy_controller.update(courseRateCmd, yawRateMeas, time.time())
    set_brake_command(pi, cmd)
    return SimpleNamespace(
        left_pulse=cmd.left_pw,
        right_pulse=cmd.right_pw,
        left_cmd_deg=(cmd.left_pw - LEFT_NEUTRAL) / _DEFLECT_PW * 60.0,
        right_cmd_deg=(cmd.right_pw - RIGHT_NEUTRAL) / _DEFLECT_PW * 60.0,
        actual_delta_deg=-cmd.diffBrake * 120.0,
        expected_yaw_rate=courseRateCmd,
        diffBrake=cmd.diffBrake,
        baseBrake=cmd.baseBrake,
    )


# ── pigpio lifecycle ───────────────────────────────────────────────────────────
def init_control():
    pigpio_mod = sys.modules.get("pigpio", _pigpio_module)
    if pigpio_mod is None:
        return None
    pi = pigpio_mod.pi()
    if not pi.connected:
        return None
    pi.set_servo_pulsewidth(LEFT_GPIO,  LEFT_NEUTRAL)
    pi.set_servo_pulsewidth(RIGHT_GPIO, RIGHT_NEUTRAL)
    return pi


def set_neutral(pi) -> None:
    if pi is None:
        return
    pi.set_servo_pulsewidth(LEFT_GPIO,  LEFT_NEUTRAL)
    pi.set_servo_pulsewidth(RIGHT_GPIO, RIGHT_NEUTRAL)


def set_motors_off(pi) -> None:
    if pi is None:
        return
    pi.set_servo_pulsewidth(LEFT_GPIO,  0)
    pi.set_servo_pulsewidth(RIGHT_GPIO, 0)


def set_brake_command(pi, cmd: BrakeCommand) -> None:
    if pi is None:
        return
    pi.set_servo_pulsewidth(LEFT_GPIO,  cmd.left_pw)
    pi.set_servo_pulsewidth(RIGHT_GPIO, cmd.right_pw)


def terminate_control(pi) -> None:
    if pi is None:
        return
    set_motors_off(pi)
    pi.stop()
