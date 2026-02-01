#!/usr/bin/env python3
"""
MG92B 서보 모터 실제 테스트 (실제 하드웨어)
GPIO 핀 12(왼쪽), 13(오른쪽)을 사용하여 실제 모터를 제어합니다.

주의: 이 스크립트는 실제 모터를 움직입니다!
"""

import sys
import os
import time
import pigpio

# 상위 디렉토리를 path에 추가
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from Sensor_Motor import Motor_Parafoil as mp

def pulse_to_angle(pulse):
    """펄스를 MG92B 각도로 변환 (500us=0°, 2500us=180°)"""
    if pulse == 0:
        return None
    return (pulse - 500) * 180 / 2000

def print_motor_info(pi, label=""):
    """현재 모터 상태 출력"""
    if label:
        print(f"\n{label}")

    # 현재 저장된 펄스값 읽기
    left_pulse = mp.current_left_pulse
    right_pulse = mp.current_right_pulse

    if left_pulse > 0:
        left_angle = pulse_to_angle(left_pulse)
        print(f"  왼쪽 모터 (GPIO 12): {left_pulse:4d} us → {left_angle:6.2f}°")

    if right_pulse > 0:
        right_angle = pulse_to_angle(right_pulse)
        print(f"  오른쪽 모터 (GPIO 13): {right_pulse:4d} us → {right_angle:6.2f}°")

def main():
    print("=" * 70)
    print("MG92B 서보 모터 실제 테스트")
    print("=" * 70)
    print("⚠️  주의: 이 스크립트는 실제 모터를 움직입니다!")
    print()
    print(f"GPIO 핀:")
    print(f"  - 왼쪽 모터: GPIO {mp.PARAFOIL_LEFT_MOTOR_PIN}")
    print(f"  - 오른쪽 모터: GPIO {mp.PARAFOIL_RIGHT_MOTOR_PIN}")
    print()
    print(f"제어 범위:")
    print(f"  - Dead zone: ±{mp.THRESHOLD}°")
    print(f"  - 최대 범위: ±{mp.max_angle_scope + mp.THRESHOLD}°")
    print()

    response = input("계속하시겠습니까? (y/n): ")
    if response.lower() != 'y':
        print("테스트 취소")
        return

    print()
    print("=" * 70)

    # 초기화
    try:
        print("\n초기화 중...")
        pi = mp.init_parafoil_motor()
        print("✓ 초기화 완료 (중립 위치)")
        print_motor_info(pi, "초기 상태:")
        time.sleep(2)

        # 테스트 케이스
        test_cases = [
            (0, "Dead zone - 직진"),
            (20, "오른쪽 회전 (작음)"),
            (0, "중립 복귀"),
            (-20, "왼쪽 회전 (작음)"),
            (0, "중립 복귀"),
            (60, "오른쪽 회전 (중간)"),
            (0, "중립 복귀"),
            (-60, "왼쪽 회전 (중간)"),
            (0, "중립 복귀"),
            (100, "오른쪽 회전 (큼)"),
            (0, "중립 복귀"),
            (-100, "왼쪽 회전 (큼)"),
            (0, "중립 복귀"),
            (134, "범위 경계 (움직임)"),
            (135, "범위 초과 (정지)"),
            (100, "범위 복귀 (움직임 재개)"),
            (0, "최종 중립 복귀"),
        ]

        print("\n테스트 시작...")
        print("-" * 70)

        for i, (error, description) in enumerate(test_cases, 1):
            print(f"\n[{i:2d}/{len(test_cases)}] error = {error:4d}° → {description}")

            # 모터 제어
            mp.rotate_parafoil_motor(pi, error)
            print_motor_info(pi)

            # 상태 저장 확인
            print(f"  저장된 상태: L={mp.current_left_pulse}us, R={mp.current_right_pulse}us")

            # 다음 테스트까지 대기
            time.sleep(1.5)
            print("-" * 70)

        print()
        print("=" * 70)
        print("✓ 모든 테스트 완료")
        print("=" * 70)

    except KeyboardInterrupt:
        print("\n\n⚠️  테스트 중단 (Ctrl+C)")

    except Exception as e:
        print(f"\n❌ 오류 발생: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # 안전하게 종료
        print("\n종료 중...")
        if 'pi' in locals():
            mp.terminate_parafoil_motor(pi)
            print("✓ 모터 안전하게 종료됨 (중립 위치)")
        else:
            print("⚠️  pi 객체를 찾을 수 없습니다")

if __name__ == "__main__":
    main()
