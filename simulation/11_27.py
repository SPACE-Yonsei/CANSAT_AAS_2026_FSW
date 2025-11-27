#!/usr/bin/env python3

import time

try:
    import pigpio
except ImportError:
    print("pigpio 라이브러리를 찾을 수 없습니다. 'pip install pigpio'로 설치해주세요.")
    print("가상 객체(mock)를 사용하여 시뮬레이션합니다.")
    from unittest.mock import Mock
    pigpio = Mock()

# Target Degree based on IMU
TARGET_DEGREE = 0
PARAFOIL_LEFT_MOTOR_PIN = 12 # gpio 12, physical pin 32
PARAFOIL_RIGHT_MOTOR_PIN = 13  # gpio 13, physical pin 33

# Calibrate the pulse range, us unit
#PARAFOIL_MOTOR_MIN_PULSE = 530 # 0.5ms
#PARAFOIL_MOTOR_MAX_PULSE = 2470 # 2.5ms
PARAFOIL_MOTOR_STOP_PULSE = 1  # 1us

def init_parafoil_motor():
    try:
        pi = pigpio.pi()
        if not pi.connected:
            raise RuntimeError("pigpiod 데몬에 연결할 수 없습니다. 터미널에 'sudo pigpiod'를 실행했는지 확인하세요.")
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, PARAFOIL_MOTOR_STOP_PULSE)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, PARAFOIL_MOTOR_STOP_PULSE)
        print("pigpio에 연결되었고 모터가 초기화되었습니다.")
        return pi
    except Exception as e:
        print(f"모터 초기화 실패: {e}")
        return pigpio # Mock 객체 반환

def terminate_parafoil_motor(pi):
    if pi and hasattr(pi, 'connected') and pi.connected:
        print("\n모터 작동을 중지하고 연결을 해제합니다.")
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, PARAFOIL_MOTOR_STOP_PULSE)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, PARAFOIL_MOTOR_STOP_PULSE)
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
        
        for error in list(range(10, 100, 10)):
            # 왼쪽 모터는 530에서 2470으로, 오른쪽 모터는 2470에서 530으로 움직이도록 계산
            left_pulse = int(2500 - (10/9) * error)
            right_pulse = int(500 + (10/9) * error)
            
            print(f"Error: {error:2d} -> Left Pulse: {left_pulse}, Right Pulse: {right_pulse}")
            pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_pulse)
            pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_pulse)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n프로그램을 종료합니다.")

    finally:
        # 프로그램 종료 시 모터를 안전하게 정지
        terminate_parafoil_motor(pi)
