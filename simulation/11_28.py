#!/usr/bin/env python3

import time
import math
import pigpio

# 핀 설정
PARAFOIL_LEFT_MOTOR_PIN = 12 
PARAFOIL_RIGHT_MOTOR_PIN = 13  

# 펄스 범위 설정
PARAFOIL_MOTOR_MIN_PULSE = 530 
PARAFOIL_MOTOR_MAX_PULSE = 2470

def init_parafoil_motor():
    pi = pigpio.pi()
    # 시작 시 안전하게 0(정지) 혹은 중립 위치로 설정
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, 0)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)
    return pi

def terminate_parafoil_motor(pi):
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, 0)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)
    pi.stop()
    # [수정 1] 여기에 있던 잘못된 'return angle' 삭제함

if __name__ == "__main__":
    pi = init_parafoil_motor()
    
    # 기준 중립값 (왼쪽은 2500이 풀린 상태, 오른쪽은 500이 풀린 상태라고 가정)
    left_neutral = 2500
    right_neutral = 500
    
    try:
        print("서보 모터 제어 시작 (Ctrl+C로 종료)")
        
        while True:
            target_pos = [(1
