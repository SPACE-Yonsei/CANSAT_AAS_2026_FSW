#!/usr/bin/env python3
import math
import time
import types
from datetime import datetime

_sim_log = open("0320_sim.txt", "a", encoding="utf-8")
DEBUG_CONTROL = True  # 제어 출력 디버그 프린트 on/off

def _dbg(line: str):
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    full = f"[{ts}] {line}"
    print(full)
    _sim_log.write(full + "\n")
    _sim_log.flush()


PARAFOIL_LEFT_MOTOR_PIN: int  = 13   # GPIO BCM pin
PARAFOIL_RIGHT_MOTOR_PIN: int = 12   # GPIO BCM pin

PULSE_PER_DEG: float = 2000.0 / 180.0  # μs/deg

LEFT_ZERO: int  = 600   # μs, 서보 0° 펄스폭
RIGHT_ZERO: int = 2500  # μs, 서보 0° 펄스폭

MAX_ANGLE_SCOPE: int = 120  # deg, 서보 최대 각도

NEUTRAL_DEG: float = 60.0  # deg, 서보 중립 각도
LEFT_NEUTRAL: int  = int(LEFT_ZERO  + NEUTRAL_DEG * PULSE_PER_DEG)  # μs
RIGHT_NEUTRAL: int = int(RIGHT_ZERO - NEUTRAL_DEG * PULSE_PER_DEG)  # μs

PULSE_MIN: int = 500   # μs
PULSE_MAX: int = 2500  # μs

K_delta: float = 1.0   # (°/s)/deg, yaw rate ↔ 서보 각도 변환 계수


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

    # commanded_yaw_rate > 0 이면
    # right_cmd_deg > NEUTRAL_DEG 가 되도록 부호 반전
    left_raw  = NEUTRAL_DEG - desired_delta_deg / 2.0
    right_raw = NEUTRAL_DEG + desired_delta_deg / 2.0

    left_cmd_deg  = max(0.0, min(float(MAX_ANGLE_SCOPE), left_raw))
    right_cmd_deg = max(0.0, min(float(MAX_ANGLE_SCOPE), right_raw))

    if DEBUG_CONTROL and (left_raw != left_cmd_deg or right_raw != right_cmd_deg):
        _dbg(f"[ACTUATOR] CLAMP — L_raw={left_raw:.1f}→{left_cmd_deg:.1f}° "
             f"R_raw={right_raw:.1f}→{right_cmd_deg:.1f}°")

    # 부호 일관성도 함께 수정
    actual_delta_deg = right_cmd_deg - left_cmd_deg
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
    if DEBUG_CONTROL:
        _dbg(f"[ACTUATOR] SET_NEUTRAL — L_pw={LEFT_NEUTRAL} R_pw={RIGHT_NEUTRAL}")
    if pi is not None:
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, LEFT_NEUTRAL)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, RIGHT_NEUTRAL)


def set_motors_off(pi):
    """서보 신호 완전 차단 (LANDED state용)"""
    if DEBUG_CONTROL:
        _dbg("[ACTUATOR] MOTORS_OFF — pw=0 (signal cut)")
    if pi is not None:
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, 0)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)
