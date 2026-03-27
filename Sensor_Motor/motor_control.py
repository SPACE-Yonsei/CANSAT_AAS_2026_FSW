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

PULSE_PER_DEG: float = 2000.0 / 180.0  # μs/deg, 서보 물리 보정값 (고정)

LEFT_ZERO: int  = 600   # μs, 서보 0° 펄스폭
RIGHT_ZERO: int = 2500  # μs, 서보 0° 펄스폭

MAX_ANGLE_SCOPE: int = 120  # deg, 서보 기계적 최대 각도

NEUTRAL_DEG: float = 60.0  # deg, 서보 중립 각도
LEFT_NEUTRAL: int  = int(LEFT_ZERO  + NEUTRAL_DEG * PULSE_PER_DEG)  # μs
RIGHT_NEUTRAL: int = int(RIGHT_ZERO - NEUTRAL_DEG * PULSE_PER_DEG)  # μs

LEFT_MAX_PULSE:  int = int(LEFT_ZERO  + MAX_ANGLE_SCOPE * PULSE_PER_DEG)  # μs, LEFT 120°
RIGHT_MIN_PULSE: int = int(RIGHT_ZERO - MAX_ANGLE_SCOPE * PULSE_PER_DEG)  # μs, RIGHT 120°

PULSE_MIN: int = 500   # μs
PULSE_MAX: int = 2500  # μs

K_pulse: float = PULSE_PER_DEG * 2  # μs/(°/s), yaw rate → 서보 펄스 오프셋 변환 계수 (×2 튜닝)


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
    # cmd_yr > 0 (오른쪽 회전): 두 펄스 모두 중립에서 증가
    #   LEFT:  팔 위로 → 왼쪽 당김 해제
    #   RIGHT: 팔 아래로(2500 방향) → 오른쪽 당김
    pulse_offset = commanded_yaw_rate / 2.0 * K_pulse

    left_raw_pw  = LEFT_NEUTRAL  + pulse_offset
    right_raw_pw = RIGHT_NEUTRAL + pulse_offset

    left_pulse  = max(PULSE_MIN,       min(LEFT_MAX_PULSE,  int(left_raw_pw)))
    right_pulse = max(RIGHT_MIN_PULSE, min(PULSE_MAX,       int(right_raw_pw)))

    if DEBUG_CONTROL and (int(left_raw_pw) != left_pulse or int(right_raw_pw) != right_pulse):
        l_deg = (left_pulse  - LEFT_ZERO)  / PULSE_PER_DEG
        r_deg = (RIGHT_ZERO  - right_pulse) / PULSE_PER_DEG
        _dbg(f"[ACTUATOR] CLAMP — L={left_pulse}μs({l_deg:.1f}°) R={right_pulse}μs({r_deg:.1f}°)")

    left_cmd_deg  = (left_pulse  - LEFT_ZERO)  / PULSE_PER_DEG
    right_cmd_deg = (RIGHT_ZERO  - right_pulse) / PULSE_PER_DEG
    actual_delta_deg  = right_cmd_deg - left_cmd_deg
    actual_offset     = ((left_pulse - LEFT_NEUTRAL) + (right_pulse - RIGHT_NEUTRAL)) / 2.0
    expected_yaw_rate = actual_offset / K_pulse * 2.0

    return left_pulse, right_pulse, left_cmd_deg, right_cmd_deg, actual_delta_deg, expected_yaw_rate


def control(pi, commanded_yaw_rate: float) -> types.SimpleNamespace:
    left_pulse, right_pulse, left_cmd_deg, right_cmd_deg, actual_delta_deg, expected_yaw_rate = \
        actuator_mixer(commanded_yaw_rate)

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
