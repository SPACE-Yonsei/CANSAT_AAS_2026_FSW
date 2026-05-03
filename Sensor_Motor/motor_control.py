"""Parafoil servo mixer and PWM output (pigpio backend with dummy fallback).

Physical wiring (verify against actual airframe before flight):
  LEFT_GPIO  = PARAFOIL_LEFT_GPIO  = 13   (left brake line servo)
  RIGHT_GPIO = PARAFOIL_RIGHT_GPIO = 12   (right brake line servo)

Mixer convention:
  commanded_yaw_rate > 0  =>  turn LEFT
  commanded_yaw_rate < 0  =>  turn RIGHT

  LEFT_SIGN  = +1.0 : positive yaw-rate pulls LEFT brake  (increases left pulse)
  RIGHT_SIGN = -1.0 : positive yaw-rate releases RIGHT brake (decreases right pulse)

  If the cansat flies in the opposite direction, swap LEFT_SIGN / RIGHT_SIGN here.
  Do NOT change guidance logic or sensor sign conventions.

Hardware fallback:
  If pigpio is unavailable (PC / CI), _DummyPi records pulses in a dict.
  All public functions work identically in both modes.
"""

from __future__ import annotations

from dataclasses import dataclass
from types import SimpleNamespace

from lib import config

# ---------------------------------------------------------------------------
# GPIO pin assignment (must match config.py and physical wiring)
# ---------------------------------------------------------------------------
LEFT_GPIO:  int = config.PARAFOIL_LEFT_GPIO    # 13
RIGHT_GPIO: int = config.PARAFOIL_RIGHT_GPIO   # 12

# ---------------------------------------------------------------------------
# Servo pulse calibration (microseconds)
# ---------------------------------------------------------------------------
LEFT_NEUTRAL:  int = 1500
RIGHT_NEUTRAL: int = 1500

LEFT_MIN:  int = 600
LEFT_MAX:  int = 2500
RIGHT_MIN: int = 600
RIGHT_MAX: int = 2500

# ---------------------------------------------------------------------------
# Mixer gain: yaw-rate [deg/s] -> pulse offset [us]
# Tune this to match actual brake-line throw.
# ---------------------------------------------------------------------------
K_PULSE_PER_DPS: float = 8.0

# ---------------------------------------------------------------------------
# Servo direction signs (physical correction only — do not touch guidance)
# +1.0 : increasing pulse PULLS brake  (normal)
# -1.0 : increasing pulse RELEASES brake (reverse — flip if cansat yaws backwards)
# ---------------------------------------------------------------------------
LEFT_SIGN:  float = +1.0
RIGHT_SIGN: float = -1.0


# ---------------------------------------------------------------------------
# Dummy backend (used when pigpio is not available)
# ---------------------------------------------------------------------------

class _DummyPi:
    """Records servo pulse commands without any hardware access."""

    def __init__(self) -> None:
        self.pulses: dict[int, int] = {}

    def set_servo_pulsewidth(self, pin: int, pulse: int) -> None:
        self.pulses[pin] = pulse

    def stop(self) -> None:
        return


# ---------------------------------------------------------------------------
# Handle
# ---------------------------------------------------------------------------

@dataclass
class ControlHandle:
    pi: object
    connected: bool   # True when a real pigpio daemon is reachable


# ---------------------------------------------------------------------------
# Internal helper
# ---------------------------------------------------------------------------

def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def init_control(_logger=None) -> ControlHandle:
    """Initialise PWM backend and output neutral pulse on both servos.

    Returns a ControlHandle. Always succeeds — falls back to _DummyPi on error.
    """
    try:
        import pigpio  # type: ignore
        pi = pigpio.pi()
        connected = bool(getattr(pi, "connected", False))
        if not connected:
            pi = _DummyPi()
    except Exception:
        pi = _DummyPi()
        connected = False

    handle = ControlHandle(pi=pi, connected=connected)
    set_neutral(handle)
    return handle


def actuator_mixer(commanded_yaw_rate: float):
    """Map commanded yaw-rate (deg/s) -> (left_pulse, right_pulse, pulse_offset).

    Differential brake mixer:
      left  servo: neutral + LEFT_SIGN  * offset
      right servo: neutral + RIGHT_SIGN * offset

    Both pulses are clamped to [MIN, MAX].
    """
    offset       = commanded_yaw_rate * K_PULSE_PER_DPS
    left_pulse   = int(round(_clamp(LEFT_NEUTRAL  + LEFT_SIGN  * offset, LEFT_MIN,  LEFT_MAX)))
    right_pulse  = int(round(_clamp(RIGHT_NEUTRAL + RIGHT_SIGN * offset, RIGHT_MIN, RIGHT_MAX)))
    return left_pulse, right_pulse, offset


def control(pi_handle, commanded_yaw_rate: float) -> SimpleNamespace:
    """Output servo pulses for the given yaw-rate command.

    Args:
        pi_handle          : ControlHandle (or raw _DummyPi for tests)
        commanded_yaw_rate : deg/s

    Returns SimpleNamespace with left_pulse, right_pulse, expected_yaw_rate.
    """
    left_pulse, right_pulse, offset = actuator_mixer(commanded_yaw_rate)
    backend = pi_handle.pi if isinstance(pi_handle, ControlHandle) else pi_handle
    backend.set_servo_pulsewidth(LEFT_GPIO,  left_pulse)
    backend.set_servo_pulsewidth(RIGHT_GPIO, right_pulse)
    return SimpleNamespace(
        expected_yaw_rate = commanded_yaw_rate,
        left_pulse        = left_pulse,
        right_pulse       = right_pulse,
        left_cmd_deg      = (LEFT_SIGN  * offset) / K_PULSE_PER_DPS,
        right_cmd_deg     = (RIGHT_SIGN * offset) / K_PULSE_PER_DPS,
        actual_delta_deg  = float(offset),
    )


def set_neutral(pi_handle) -> None:
    """Output neutral pulse on both servos (brakes released — safe idle)."""
    backend = pi_handle.pi if isinstance(pi_handle, ControlHandle) else pi_handle
    backend.set_servo_pulsewidth(LEFT_GPIO,  LEFT_NEUTRAL)
    backend.set_servo_pulsewidth(RIGHT_GPIO, RIGHT_NEUTRAL)


def set_motors_off(pi_handle) -> None:
    """Set pulse to 0 on both servos (pigpio disables PWM — use at LANDED)."""
    backend = pi_handle.pi if isinstance(pi_handle, ControlHandle) else pi_handle
    backend.set_servo_pulsewidth(LEFT_GPIO,  0)
    backend.set_servo_pulsewidth(RIGHT_GPIO, 0)
