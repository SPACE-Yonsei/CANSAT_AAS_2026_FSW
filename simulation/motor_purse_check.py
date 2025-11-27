#!/usr/bin/env python3

import time

# Target Degree based on IMU
TARGET_DEGREE = 0
PARAFOIL_LEFT_MOTOR_PIN = 12 # gpio 12, physical pin 32
PARAFOIL_RIGHT_MOTOR_PIN = 13  # gpio 13, physical pin 33

# Calibrate the pulse range, us unit
PARAFOIL_MOTOR_MIN_PULSE = 530 # 0.5ms
PARAFOIL_MOTOR_MAX_PULSE = 2470 # 2.5ms
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



def rotate_parafoil_motor(pi, turn:float):

    TURN_THRESHOLD = 15
    willing_to_turn = turn # 모터가 돌아야 하는 각도 - 180~ 180도 사이로 입력

    if willing_to_turn < -TURN_THRESHOLD:
        # Turn Left: Pull the left motor line
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, PARAFOIL_MOTOR_MAX_PULSE)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)
    elif willing_to_turn > TURN_THRESHOLD:
        # Turn Right: Pull the right motor line
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, 0)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, PARAFOIL_MOTOR_MAX_PULSE)
    else:
        # Go Straight: Keep motors idle
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, 0)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)

    return

#############################################


if __name__ == "__main__":
    pi = init_parafoil_motor()

    # 모터를 움직일 펄스 값들을 리스트로 정의
    pulse_positions = [
        PARAFOIL_MOTOR_MIN_PULSE,  # 최소 위치 (약 0도)
        1500,                     # 중간 위치 (약 90도)
        PARAFOIL_MOTOR_MAX_PULSE,  # 최대 위치 (약 180도)
        1500                      # 다시 중간 위치로
    ]

    try:
        print("서보 모터를 지정된 펄스 값으로 자동 순환합니다.")
        print("프로그램을 종료하려면 Ctrl+C를 누르세요.")
        
        while True:
            # 정의된 위치들을 하나씩 순F회
            for pulse in pulse_positions:
                # *** 현재 적용되는 펄스 값을 print하는 기능 ***
                print(f"현재 적용된 펄스 값: {pulse}µs")
                
                pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, pulse)
                time.sleep(1)
                pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, pulse)
                time.sleep(1)

    except KeyboardInterrupt:
        print("\n프로그램을 종료합니다.")
        pass

    finally:
        # 프로그램 종료 시 모터를 안전하게 정지
        terminate_parafoil_motor(pi)
