#!/usr/bin/env python3
"""
Parafoil Motors Test Script
두 모터(GPIO 12, 13)에 1250~1750 펄스를 차례로 계속 주는 테스트 코드

사용법:
    python3 Sensor_Motor/test_parafoil_motors.py
"""

import time
import pigpio

PARAFOIL_LEFT_MOTOR_PIN = 12  # GPIO 12, physical pin 32
PARAFOIL_RIGHT_MOTOR_PIN = 13  # GPIO 13, physical pin 33

# 펄스 범위
PULSE_MIN = 1250  # 최소 펄스 (µs)
PULSE_MAX = 1750  # 최대 펄스 (µs)
PULSE_STEP = 50   # 펄스 증가 단계 (µs)
DELAY = 0.5       # 각 펄스 간 지연 시간 (초)

def test_parafoil_motors():
    """
    두 모터에 1250~1750 펄스를 차례로 계속 주는 테스트
    """
    print("="*70)
    print("파라포일 모터 테스트 시작")
    print("="*70)
    print(f"\n설정:")
    print(f"  왼쪽 모터: GPIO {PARAFOIL_LEFT_MOTOR_PIN} (물리 핀 32)")
    print(f"  오른쪽 모터: GPIO {PARAFOIL_RIGHT_MOTOR_PIN} (물리 핀 33)")
    print(f"  펄스 범위: {PULSE_MIN} ~ {PULSE_MAX} µs")
    print(f"  펄스 단계: {PULSE_STEP} µs")
    print(f"  지연 시간: {DELAY} 초")
    print("\n⚠️  모터를 테스트합니다. 종료하려면 Ctrl+C를 누르세요.\n")
    
    # pigpio 초기화
    try:
        pi = pigpio.pi()
        if not pi.connected:
            print("❌ pigpio 데몬에 연결할 수 없습니다.")
            print("   sudo pigpiod 명령으로 pigpio 데몬을 시작해주세요.")
            return
        
        print("✅ pigpio 연결 성공\n")
        
        # 초기화: 모든 모터 정지
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, 0)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)
        time.sleep(0.5)
        
        cycle = 0
        try:
            while True:
                cycle += 1
                print(f"\n{'='*70}")
                print(f"사이클 {cycle}")
                print(f"{'='*70}")
                
                # 왼쪽 모터 테스트: 1250 ~ 1750
                print(f"\n왼쪽 모터 (GPIO {PARAFOIL_LEFT_MOTOR_PIN}) 테스트:")
                for pulse in range(PULSE_MIN, PULSE_MAX + 1, PULSE_STEP):
                    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, pulse)
                    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)
                    print(f"  펄스: {pulse} µs")
                    time.sleep(DELAY)
                
                # 왼쪽 모터 정지
                pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, 0)
                time.sleep(0.5)
                
                # 오른쪽 모터 테스트: 1250 ~ 1750
                print(f"\n오른쪽 모터 (GPIO {PARAFOIL_RIGHT_MOTOR_PIN}) 테스트:")
                for pulse in range(PULSE_MIN, PULSE_MAX + 1, PULSE_STEP):
                    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, 0)
                    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, pulse)
                    print(f"  펄스: {pulse} µs")
                    time.sleep(DELAY)
                
                # 오른쪽 모터 정지
                pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)
                time.sleep(0.5)
                
        except KeyboardInterrupt:
            print("\n\n⚠️  사용자가 프로그램을 중단했습니다.")
        
        finally:
            # 모든 모터 정지
            print("\n모터 정지 중...")
            pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, 0)
            pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)
            time.sleep(0.5)
            
            # pigpio 종료
            pi.stop()
            print("✅ pigpio 연결 종료")
            print("✅ 테스트 완료")
    
    except Exception as e:
        print(f"\n❌ 오류 발생: {e}")
        return

if __name__ == "__main__":
    test_parafoil_motors()

