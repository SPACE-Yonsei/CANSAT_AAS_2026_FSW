#!/usr/bin/env python3

import time
import math
import pigpio

# 핀 설정
PARAFOIL_LEFT_MOTOR_PIN = 12 
PARAFOIL_RIGHT_MOTOR_PIN = 13  

# 펄스 범위 제한 (하드웨어 보호)
PARAFOIL_MOTOR_MIN_PULSE = 530 
PARAFOIL_MOTOR_MAX_PULSE = 2470

def init_parafoil_motor():
    pi = pigpio.pi()
    if not pi.connected:
        print("[Error] pigpio 데몬에 연결 실패. 'sudo pigpiod' 실행 확인 필요.")
        exit()
    # 초기화 시 둘 다 0(Stop)
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, 0)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)
    return pi

def terminate_parafoil_motor(pi):
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, 0)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)
    pi.stop()

if __name__ == "__main__":
    pi = init_parafoil_motor()
    
    # [설정] 중립(Neutral) 위치 정의
    # 왼쪽: 2500이 풀린 상태(Neutral), 줄어들면 당겨짐
    # 오른쪽: 500이 풀린 상태(Neutral), 늘어나면 당겨짐
    left_neutral = 2500
    right_neutral = 500
    
    try:
        print("서보 모터 제어 시작 (Ctrl+C로 종료)")
        
        while True:
            # 테스트를 위한 4방위 좌표
            target_pos = [(1,1), (-1,1), (-1,-1), (1,-1)]
            
            for pos in target_pos:
                # [핵심 수정 1] 매 루프 시작 시 일단 '중립'으로 초기화
                # 이렇게 해야 턴이 끝났을 때 모터가 원위치로 돌아옵니다.
                left_pulse = left_neutral
                right_pulse = right_neutral
                
                dx, dy = pos
                angle = math.degrees(math.atan2(dx, dy))
                
                # 변화량 계산 (1도당 펄스 변화량)
                change = abs(angle * 10.7778)
                
                # [핵심 수정 2] 로직 분기
                # 한쪽이 당겨질 때, 다른 쪽은 위에서 초기화한 'neutral' 값을 유지합니다.
                if angle > 5:
                    # 우측 턴 -> 왼쪽 모터를 당김 (2500 -> 작아짐)
                    left_pulse = int(left_neutral - change)
                    
                elif angle < -5: 
                    # 좌측 턴 -> 오른쪽 모터를 당김 (500 -> 커짐)
                    # 주의: 오른쪽은 500이 최소이므로 당기려면 더해야(+) 합니다.
                    right_pulse = int(right_neutral + change) 

                # [안전 장치] 펄스 범위 제한 (Clamp)
                left_pulse = max(PARAFOIL_MOTOR_MIN_PULSE, min(left_pulse, PARAFOIL_MOTOR_MAX_PULSE))
                right_pulse = max(PARAFOIL_MOTOR_MIN_PULSE, min(right_pulse, PARAFOIL_MOTOR_MAX_PULSE))

                # [실행] 계산된 값을 모터에 입력 (If문 밖에서 둘 다 실행해야 동기화됨)
                pi.set_servo_pulsewidth(PARAFOIL_LEFT_
