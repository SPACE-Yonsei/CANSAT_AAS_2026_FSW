#!/usr/bin/env python3
"""
Parafoil motor control

Left motor (GPIO 12): 500 = pull up, 1500 = release down
Right motor (GPIO 13): 500 = pull up, 1500 = release down

Turn left: left release down (1500) + right pull up (500)
Turn right: left pull up (500) + right release down (1500)
Straight: both release down (1500, 1500)
"""

import time
import pigpio

# 핀 설정
PARAFOIL_LEFT_MOTOR_PIN = 12 
PARAFOIL_RIGHT_MOTOR_PIN = 13  

# 펄스 범위
PARAFOIL_MOTOR_MIN_PULSE = 530 
PARAFOIL_MOTOR_MAX_PULSE = 2470
def init_parafoil_motor():
    """Initialize parafoil motor (PWM disabled initially)."""
    pi = pigpio.pi()
    
    if not pi.connected:
        raise RuntimeError(
            "pigpio 데몬 연결 실패. 'sudo pigpiod' 실행 확인 필요"
        )
    
    # PWM 비활성화 (초기 상태)
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, 0)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)
    
    return pi

def terminate_parafoil_motor(pi):
    if pi is not None and pi.connected:
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, 0)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)
        pi.stop()

def rotate_parafoil_motor(pi, error: float):

    left_neutral = 1500
    right_neutral = 1500
    left_pulse = left_neutral
    right_pulse = right_neutral        
    print(f"초기 펄스 -> 좌: {left_pulse}, 우: {right_pulse}")
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, left_pulse)
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, right_pulse)
    
    if angle <-90:
        angle=-90
    elif angle >90:
        angle=90
    error_purse = abs(angle * 10.7778)
    if angle > 5:
        # 우회전 -> 오른쪽 당김 (1500에서 뺌)
        left_pulse = left_neutral
        right_pulse = int(right_neutral - error_purse)

        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_pulse)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_pulse)

    elif angle < -5:
        # 좌회전 -> 왼쪽 당김 (1500에서 더함)
        left_pulse = int(left_neutral + error_purse)
        right_pulse = right_neutral

        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_pulse)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_pulse)
        
    else:
        # 직진
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_pulse)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_pulse)
    
    return
