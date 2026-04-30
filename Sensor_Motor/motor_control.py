"""Parafoil actuator control abstraction."""

from __future__ import annotations

from types import SimpleNamespace


def init_control(_logger=None):
    # Real implementation should initialize pigpio and set neutral.
    return {"connected": True}


def control(pi, commanded_yaw_rate: float):
    # In this baseline, we expose the command as feedback.
    return SimpleNamespace(
        expected_yaw_rate=commanded_yaw_rate,
        left_pulse=1500.0 + commanded_yaw_rate,
        right_pulse=1500.0 + commanded_yaw_rate,
        left_cmd_deg=commanded_yaw_rate,
        right_cmd_deg=commanded_yaw_rate,
        actual_delta_deg=0.0,
    )


def set_neutral(_pi) -> None:
    return


def set_motors_off(_pi) -> None:
    return
