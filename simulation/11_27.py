#!/usr/bin/env python3

import time

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
                # *** 현재 적용되는 펄스 값을 print하는 기능 ***
                print(f"현재 각도: {error}µs")
                
                pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, int(500+10/9*error))
                time.sleep(0.5)
                pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, int(2500-10/9*error))
                time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n프로그램을 종료합니다.")

    finally:
        # 프로그램 종료 시 모터를 안전하게 정지
        terminate_parafoil_motor(pi)
