#!/usr/bin/env python3

import time
import math
import pigpio

# 핀 설정
PARAFOIL_LEFT_MOTOR_PIN = 12 
PARAFOIL_RIGHT_MOTOR_PIN = 13  

# 펄스 범위
PARAFOIL_MOTOR_MIN_PULSE = 530 
PARAFOIL_MOTOR_MAX_PULSE = 2470

def init_parafoil_motor():
    pi = pigpio.pi()
    if not pi.connected:
        print("[에러] pigpio 데몬 연결 실패. 'sudo pigpiod' 실행 확인 필요")
        exit()
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, 0)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)
    return pi

def terminate_parafoil_motor(pi):
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, 0)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)
    pi.stop()

if __name__ == "__main__":
    pi = init_parafoil_motor()
    
    left_neutral = 2500
    right_neutral = 500
    
    try:
        print("서보 모터 제어 시작 (Ctrl+C로 종료)")
        
        while True:
            target_pos = [(1,1), (-1,1), (-1,-1), (1,-1)]
            
            for pos in target_pos:
                # [수정 1] 매번 중립 값으로 초기화 (안전장치)
                left_pulse = left_neutral
                right_pulse = right_neutral
                
                dx, dy = pos
                angle = math.degrees(math.atan2(dx, dy))
                
                # 변화량 계산 (음수 각도 고려하여 절대값 사용 추천)
                change = abs(angle * 10.7778)
                
                # [수정 2] 로직 명확화
                if angle > 5:
                    # 우회전 -> 왼쪽 당김 (2500에서 뺌)
                    left_pulse = int(left_neutral - change)
                elif angle < -5:
                    # 좌회전 -> 오른쪽 당김 (500에서 더함)
                    right_pulse = int(right_neutral + change)
                
                # 범위 제한 (하드웨어 보호)
                left_pulse = max(PARAFOIL_MOTOR_MIN_PULSE, min(left_pulse, PARAFOIL_MOTOR_MAX_PULSE))
                right_pulse = max(PARAFOIL_MOTOR_MIN_PULSE, min(right_pulse, PARAFOIL_MOTOR_MAX_PULSE))

                # [수정 3] 모터 구동 명령 (반드시 둘 다 보내야 함)
                pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_pulse)
                pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_pulse)
                
                print(f"목표:{pos} | 각도:{angle:5.1f} | 좌:{left_pulse} | 우:{right_pulse}")
                
                # [수정 4] ★★★ 여기가 없어서 안 움직였던 것입니다! ★★★
                # 모터가 움직일 물리적 시간을 줍니다.
                time.sleep(1.0) 

            print("--- 1회 순회 완료 ---")
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n종료합니다.")
    finally:
        terminate_parafoil_motor(pi)
