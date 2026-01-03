#!/usr/bin/env python3
"""
파라포일 모터 제어

왼쪽 모터 (GPIO 12): 500 = 올림(당김), 1500 = 내림(풀림)
오른쪽 모터 (GPIO 13): 500 = 올림(당김), 1500 = 내림(풀림)

왼쪽으로 선회: 왼쪽 내림(1500) + 오른쪽 올림(500)
오른쪽으로 선회: 왼쪽 올림(500) + 오른쪽 내림(1500)
직진: 양쪽 내림(1500, 1500)
"""

import time

PARAFOIL_LEFT_MOTOR_PIN = 12   # GPIO 12, physical pin 32
PARAFOIL_RIGHT_MOTOR_PIN = 13  # GPIO 13, physical pin 33

# 모터 펄스 범위
MOTOR_UP = 500     # 줄 당김 (올림)
MOTOR_DOWN = 1500  # 줄 풀림 (내림)

# Hardware-safe pulse boundaries
PULSE_MIN = 500
PULSE_MAX = 1500

def init_parafoil_motor():
    import pigpio
    pi = pigpio.pi()
    # 초기화: 양쪽 모두 내림 (직진 상태)
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, MOTOR_DOWN)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, MOTOR_DOWN)
    return pi

def terminate_parafoil_motor(pi):
    if pi is not None:
        # 종료 시 양쪽 모두 내림
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, MOTOR_DOWN)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, MOTOR_DOWN)
        time.sleep(0.1)
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, 0)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)

def _clamp_pulse(pulse: int) -> int:
    """Clamp servo pulsewidth to the safe range."""
    return max(PULSE_MIN, min(PULSE_MAX, pulse))

def rotate_parafoil_motor(pi, turn: float):
    """
    파라포일 모터 제어 (비례 제어)
    
    Args:
        pi: pigpio instance
        turn: 각도 차이 (-180 ~ +180)
              - 음수: 왼쪽 선회 (왼쪽 내림 + 오른쪽 올림)
              - 양수: 오른쪽 선회 (왼쪽 올림 + 오른쪽 내림)
    """
    TURN_THRESHOLD = 15  # 데드존 (±15도)
    MAX_TURN_ANGLE = 90  # 최대 속도를 위한 각도
    
    # 데드존 내: 직진 (양쪽 모두 내림)
    if abs(turn) <= TURN_THRESHOLD:
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, MOTOR_DOWN)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, MOTOR_DOWN)
        return
    
    # 회전 크기 계산
    turn_magnitude = abs(turn)
    effective_turn = min(turn_magnitude - TURN_THRESHOLD, MAX_TURN_ANGLE - TURN_THRESHOLD)
    
    # 속도 비율 (0.0 ~ 1.0)
    speed_ratio = effective_turn / (MAX_TURN_ANGLE - TURN_THRESHOLD)
    
    # 펄스 범위: 1500(내림) ~ 500(올림)
    pulse_range = MOTOR_DOWN - MOTOR_UP  # 1000
    
    if turn < 0:  # 왼쪽 선회: 왼쪽 내림, 오른쪽 올림
        left_pulse = MOTOR_DOWN  # 왼쪽 내림 (1500)
        right_pulse = int(MOTOR_DOWN - (speed_ratio * pulse_range))  # 오른쪽 올림 (1500 → 500)
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, _clamp_pulse(left_pulse))
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, _clamp_pulse(right_pulse))
    else:  # 오른쪽 선회: 왼쪽 올림, 오른쪽 내림
        left_pulse = int(MOTOR_DOWN - (speed_ratio * pulse_range))  # 왼쪽 올림 (1500 → 500)
        right_pulse = MOTOR_DOWN  # 오른쪽 내림 (1500)
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, _clamp_pulse(left_pulse))
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, _clamp_pulse(right_pulse))
    
    return
