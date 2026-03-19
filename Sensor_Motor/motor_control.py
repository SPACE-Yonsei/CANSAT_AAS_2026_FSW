#!/usr/bin/env python3
import math
import time
import types


PARAFOIL_LEFT_MOTOR_PIN  = 12
PARAFOIL_RIGHT_MOTOR_PIN = 13

PULSE_PER_DEG = 2000.0 / 180.0

LEFT_ZERO  = 600
RIGHT_ZERO = 2500

MAX_ANGLE_SCOPE = 120

NEUTRAL_DEG = 60.0
LEFT_NEUTRAL  = int(LEFT_ZERO  + NEUTRAL_DEG * PULSE_PER_DEG)
RIGHT_NEUTRAL = int(RIGHT_ZERO - NEUTRAL_DEG * PULSE_PER_DEG)

PULSE_MIN = 500
PULSE_MAX = 2500

K_delta = 1.0


def init_control():
    import pigpio
    pi = pigpio.pi()
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, LEFT_NEUTRAL)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, RIGHT_NEUTRAL)
    return pi


def terminate_parafoil_motor(pi):
    if pi is not None:
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, LEFT_NEUTRAL)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, RIGHT_NEUTRAL)
        time.sleep(0.1)
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, 0)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)
        pi.stop()


def actuator_mixer(commanded_yaw_rate: float) -> tuple:
    desired_delta_deg = commanded_yaw_rate / K_delta

    left_raw  = NEUTRAL_DEG + desired_delta_deg / 2.0
    right_raw = NEUTRAL_DEG - desired_delta_deg / 2.0

    left_cmd_deg  = max(0.0, min(float(MAX_ANGLE_SCOPE), left_raw))
    right_cmd_deg = max(0.0, min(float(MAX_ANGLE_SCOPE), right_raw))

    actual_delta_deg = left_cmd_deg - right_cmd_deg
    expected_yaw_rate = K_delta * actual_delta_deg

    return left_cmd_deg, right_cmd_deg, actual_delta_deg, expected_yaw_rate


def _servo_pulse_left(angle_deg: float) -> int:
    pulse = int(LEFT_ZERO + angle_deg * PULSE_PER_DEG)
    return max(PULSE_MIN, min(PULSE_MAX, pulse))


def _servo_pulse_right(angle_deg: float) -> int:
    pulse = int(RIGHT_ZERO - angle_deg * PULSE_PER_DEG)
    return max(PULSE_MIN, min(PULSE_MAX, pulse))


def control(pi, commanded_yaw_rate: float) -> types.SimpleNamespace:
    left_cmd_deg, right_cmd_deg, actual_delta_deg, expected_yaw_rate = \
        actuator_mixer(commanded_yaw_rate)

    left_pulse  = _servo_pulse_left(left_cmd_deg)
    right_pulse = _servo_pulse_right(right_cmd_deg)

    if pi is not None:
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_pulse)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_pulse)

    return types.SimpleNamespace(
        left_cmd_deg=left_cmd_deg,
        right_cmd_deg=right_cmd_deg,
        actual_delta_deg=actual_delta_deg,
        expected_yaw_rate=expected_yaw_rate,
        left_pulse=left_pulse,
        right_pulse=right_pulse
    )


def set_neutral(pi):
    if pi is not None:
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, LEFT_NEUTRAL)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, RIGHT_NEUTRAL)
