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

    try:
        print("서보 모터 상호 역방향 제어를 시작합니다.")
        print("Left: 2470 -> 1500 | Right: 1500 -> 2470")
        print("Ctrl+C를 눌러 종료하세요.")
        
        while True:
            # 1. 0도 -> 90도 (정방향 진행)
            # range(91)은 0부터 90까지
            for angle in range(91):
                # 변화량 계산 (90도 동안 약 970펄스 변화 -> 1도당 약 10.77)
                # angle이 커질수록 change값도 커짐
                change = angle * 10.7778
                
                # 왼쪽: 2470에서 시작해서 점점 줄어듦 (2470 -> 1500)
                left_pulse = int(2500 - change)
                
                # 오른쪽: 1500에서 시작해서 점점 늘어남 (1500 -> 2470)
                right_pulse = int(500 + change)
                
                # 출력 (\r로 같은 줄에 갱신)
                print(f"[전진] 각도: {angle:2d} | 좌: {left_pulse} | 우: {right_pulse}", end='\r')
                
                pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_pulse)
                pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_pulse)
                
                # 모터 반응 속도 고려 (너무 빠르면 모터가 못 따라감)
                time.sleep(0.02)

            print() # 줄바꿈

            # 2. 90도 -> 0도 (역방향 복귀 - 모터 보호용)
            # range(90, -1, -1)은 90부터 0까지 1씩 감소
            for angle in range(90, -1, -1):
                change = angle * 10.7778
                
                left_pulse = int(1500 - change)
                right_pulse = int(1500 + change)
                
                print(f"[복귀] 각도: {angle:2d} | 좌: {left_pulse} | 우: {right_pulse}", end='\r')
                
                pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_pulse)
                pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_pulse)
                
                time.sleep(0.02)
            
            print() # 줄바꿈
            print("--- 1회 왕복 완료, 1초 대기 ---")
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n프로그램을 종료합니다.")
        
    finally:
        # 프로그램 종료 시 안전하게 정지
        terminate_parafoil_motor(pi)
