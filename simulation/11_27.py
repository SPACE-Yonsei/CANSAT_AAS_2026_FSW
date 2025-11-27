#!/usr/bin/env python3

import time
import math
# Target Degree based on IMU
TARGET_DEGREE = 0
PARAFOIL_LEFT_MOTOR_PIN = 12 # gpio 12, physical pin 32
PARAFOIL_RIGHT_MOTOR_PIN = 13  # gpio 13, physical pin 33

# Calibrate the pulse range, us unit
#PARAFOIL_MOTOR_MIN_PULSE = 530 # 0.5ms
#PARAFOIL_MOTOR_MAX_PULSE = 2470 # 2.5ms
PARAFOIL_MOTOR_STOP_PULSE = 1  # 1us

def init_parafoil_motor():
    import pigpio
    pi = pigpio.pi()
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, 0)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)
    return pi

def terminate_parafoil_motor(pi):
    pi.stop()

def angle_to_pulse(angle) -> int:
    if angle < 0:
        angle = 0
    elif angle > 180:
        angle = 180
    
    return int(PARAFOIL_MOTOR_MIN_PULSE + ((angle/180)*(PARAFOIL_MOTOR_MAX_PULSE - PARAFOIL_MOTOR_MIN_PULSE)))

if __name__ == "__main__":
    pi = init_parafoil_motor()

    try:
        print("서보 모터를 지정된 펄스 값으로 자동 순환합니다.")
        print("프로그램을 종료하려면 Ctrl+C를 누르세요.")
        
        while True:
            # 정의된 위치들을 하나씩 순F회
            for error in range(90):
                # 왼쪽 모터는 530에서 2470으로, 오른쪽 모터는 2470에서 530으로 움직이도록 계산
                left_pulse = int(PARAFOIL_MOTOR_MIN_PULSE + ((PARAFOIL_MOTOR_MAX_PULSE - PARAFOIL_MOTOR_MIN_PULSE) / 89) * error)
                right_pulse = int(PARAFOIL_MOTOR_MAX_PULSE - ((PARAFOIL_MOTOR_MAX_PULSE - PARAFOIL_MOTOR_MIN_PULSE) / 89) * error)
                
                print(f"왼쪽 모터 펄스: {left_pulse}")                
                pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_pulse)
                time.sleep(0.5)
                print(f"오른쪽 모터 펄스: {right_pulse}")
                pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_pulse)
                time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n프로그램을 종료합니다.")

    finally:
        # 프로그램 종료 시 모터를 안전하게 정지
        terminate_parafoil_motor(pi)
