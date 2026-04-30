"""Parafoil servo mixer and PWM output control.

This module is hardware-friendly (pigpio) but also test-friendly:
- If pigpio is unavailable, it falls back to a dummy backend.
- All command outputs are clamped to safe pulse ranges.
"""

from __future__ import annotations

from dataclasses import dataclass
from types import SimpleNamespace


LEFT_GPIO = 13
RIGHT_GPIO = 12

LEFT_NEUTRAL = 1500
RIGHT_NEUTRAL = 1500

LEFT_MIN = 600
LEFT_MAX = 2500
RIGHT_MIN = 600
RIGHT_MAX = 2500

# Yaw-rate [deg/s] to pulse [us] gain
K_PULSE_PER_DPS = 8.0


class _DummyPi:
    def __init__(self) -> None:
        self.pulses: dict[int, int] = {}

    def set_servo_pulsewidth(self, pin: int, pulse: int) -> None:
        self.pulses[pin] = pulse

    def stop(self) -> None:
        return


@dataclass
class ControlHandle:
    pi: object
    connected: bool


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def init_control(_logger=None):
    """Initialize PWM backend and force neutral actuator state."""
    try:
        import pigpio  # type: ignore

        pi = pigpio.pi()
        connected = bool(getattr(pi, "connected", 0))
        if not connected:
            pi = _DummyPi()
    except Exception:
        pi = _DummyPi()
        connected = False

    handle = ControlHandle(pi=pi, connected=connected)
    set_neutral(handle)
    return handle


def actuator_mixer(commanded_yaw_rate: float):
    """Map commanded yaw rate to left/right servo pulse widths.

    Current strategy mirrors the existing baseline behavior:
    both servos move in the same direction from neutral.
    """
    pulse_offset = commanded_yaw_rate * K_PULSE_PER_DPS
    left_pulse = int(round(_clamp(LEFT_NEUTRAL + pulse_offset, LEFT_MIN, LEFT_MAX)))
    right_pulse = int(round(_clamp(RIGHT_NEUTRAL + pulse_offset, RIGHT_MIN, RIGHT_MAX)))
    return left_pulse, right_pulse, pulse_offset


def control(pi, commanded_yaw_rate: float):
    left_pulse, right_pulse, pulse_offset = actuator_mixer(commanded_yaw_rate)
    pi_handle = pi if isinstance(pi, ControlHandle) else ControlHandle(pi=pi, connected=False)
    backend = pi_handle.pi
    backend.set_servo_pulsewidth(LEFT_GPIO, left_pulse)
    backend.set_servo_pulsewidth(RIGHT_GPIO, right_pulse)

    return SimpleNamespace(
        expected_yaw_rate=commanded_yaw_rate,
        left_pulse=left_pulse,
        right_pulse=right_pulse,
        left_cmd_deg=pulse_offset / K_PULSE_PER_DPS,
        right_cmd_deg=pulse_offset / K_PULSE_PER_DPS,
        actual_delta_deg=float(pulse_offset),
    )


def set_neutral(pi) -> None:
    pi_handle = pi if isinstance(pi, ControlHandle) else ControlHandle(pi=pi, connected=False)
    backend = pi_handle.pi
    backend.set_servo_pulsewidth(LEFT_GPIO, LEFT_NEUTRAL)
    backend.set_servo_pulsewidth(RIGHT_GPIO, RIGHT_NEUTRAL)


def set_motors_off(pi) -> None:
    pi_handle = pi if isinstance(pi, ControlHandle) else ControlHandle(pi=pi, connected=False)
    backend = pi_handle.pi
    backend.set_servo_pulsewidth(LEFT_GPIO, 0)
    backend.set_servo_pulsewidth(RIGHT_GPIO, 0)
