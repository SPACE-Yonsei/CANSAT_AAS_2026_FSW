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

# Hardware-safe pulse boundaries (절대 1500 초과 금지!)
PULSE_MIN = 500
PULSE_MAX = 1500  # 12, 13번 모터 모두 1500 초과 펄스 금지

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
    """Clamp servo pulsewidth to safe range (500-1500). 절대 1500 초과 금지!"""
    if pulse > 1500:
        pulse = 1500
    if pulse < 500:
        pulse = 500
    return pulse

def rotate_parafoil_motor(pi, turn: float):
    """
    파라포일 모터 제어 (ON/OFF 제어)
    
    Args:
        pi: pigpio instance
        turn: 각도 차이 (-180 ~ +180)
              - 음수: 왼쪽 선회 (왼쪽 내림 + 오른쪽 올림)
              - 양수: 오른쪽 선회 (왼쪽 올림 + 오른쪽 내림)
    """
    TURN_THRESHOLD = 15  # 데드존 (±15도)
    
    # 데드존 내: 직진 (양쪽 모두 내림)
    if abs(turn) <= TURN_THRESHOLD:
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, MOTOR_DOWN)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, MOTOR_DOWN)
        return
    
    if turn < 0:  # 왼쪽 선회: 왼쪽 내림(1500), 오른쪽 올림(500)
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, MOTOR_DOWN)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, MOTOR_UP)
    else:  # 오른쪽 선회: 왼쪽 올림(500), 오른쪽 내림(1500)
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, MOTOR_UP)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, MOTOR_DOWN)
    
    return
