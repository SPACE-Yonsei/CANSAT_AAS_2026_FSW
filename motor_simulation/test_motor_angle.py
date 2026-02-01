#!/usr/bin/env python3
"""
MG92B 서보 모터 각도 테스트 파일
Motor_Parafoil.py의 rotate_parafoil_motor 함수를 테스트합니다.

MG92B 서보 모터 스펙:
- 펄스 범위: 500us ~ 2500us
- 각도 범위: 0° ~ 180°
- 중립(90°): 1500us
"""

import sys
import os

# 상위 디렉토리를 path에 추가
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# Mock pigpio module
class MockPi:
    def __init__(self):
        self.left_pulse = 0
        self.right_pulse = 0

    def pulse_to_angle(self, pulse):
        """펄스를 각도로 변환 (500us=0°, 2500us=180°)"""
        if pulse == 0:
            return None
        angle = (pulse - 500) * 180 / 2000
        return angle

    def set_servo_pulsewidth(self, pin, pulse):
        if pin == 12:  # Left motor
            self.left_pulse = pulse
            angle = self.pulse_to_angle(pulse)
            if angle is not None:
                print(f"  왼쪽 모터: {pulse:4d} us → {angle:6.2f}°")
            else:
                print(f"  왼쪽 모터: OFF")
        elif pin == 13:  # Right motor
            self.right_pulse = pulse
            angle = self.pulse_to_angle(pulse)
            if angle is not None:
                print(f"  오른쪽 모터: {pulse:4d} us → {angle:6.2f}°")
            else:
                print(f"  오른쪽 모터: OFF")

# Import Motor_Parafoil
from Sensor_Motor import Motor_Parafoil

# Mock pigpio in Motor_Parafoil
import Sensor_Motor.Motor_Parafoil as mp
original_pigpio = None

def setup_mock():
    """pigpio를 mock으로 교체"""
    global original_pigpio
    import sys

    # Create mock pigpio module
    class MockPigpioModule:
        @staticmethod
        def pi():
            return MockPi()

    sys.modules['pigpio'] = MockPigpioModule()

def print_motor_state(error, pi):
    """모터 상태 출력 (MG92B 기준)"""
    # MG92B: 500us=0°, 2500us=180°, 1500us=90°
    if pi.left_pulse > 0:
        left_mg92b_angle = (pi.left_pulse - 500) * 180 / 2000
        left_deviation = mp.max_angle_scope - left_mg92b_angle  # 중립(120°)에서 벗어난 각도
        print(f"    → MG92B 실제 각도: {left_mg92b_angle:.1f}° (중립 120°에서 {left_deviation:+.1f}°)")

    if pi.right_pulse > 0:
        right_mg92b_angle = (pi.right_pulse - 500) * 180 / 2000
        right_deviation = mp.max_angle_scope - right_mg92b_angle
        print(f"    → MG92B 실제 각도: {right_mg92b_angle:.1f}° (중립 120°에서 {right_deviation:+.1f}°)")

    print()

def main():
    setup_mock()

    print("=" * 70)
    print("MG92B 서보 모터 각도 테스트")
    print("=" * 70)
    print("MG92B 스펙: 500us(0°) ~ 1500us(90°) ~ 2500us(180°)")
    print()
    print(f"왼쪽 모터 설정:")
    print(f"  - Zero(0°): {mp.left_zero} us")
    print(f"  - Neutral(120°): {mp.left_neutral} us → MG92B: {(mp.left_neutral-500)*180/2000:.1f}°")
    print(f"오른쪽 모터 설정:")
    print(f"  - Zero(0°): {mp.right_zero} us")
    print(f"  - Neutral(120°): {mp.right_neutral} us → MG92B: {(mp.right_neutral-500)*180/2000:.1f}°")
    print()
    print(f"제어 범위:")
    print(f"  - Dead zone: ±{mp.THRESHOLD}°")
    print(f"  - 최대 범위: ±{mp.max_angle_scope + mp.THRESHOLD}° (±135°)")
    print("=" * 70)
    print()

    # Initialize motor
    pi = mp.init_parafoil_motor()
    print("✓ 초기화 완료")
    print(f"  저장된 상태: 왼쪽={mp.current_left_pulse} us, 오른쪽={mp.current_right_pulse} us")
    print()

    # Test cases
    test_cases = [
        (0, "Dead zone (직진)"),
        (10, "Dead zone 내부 (직진)"),
        (-10, "Dead zone 내부 (직진)"),
        (20, "오른쪽 회전 (작음)"),
        (-20, "왼쪽 회전 (작음)"),
        (60, "오른쪽 회전 (중간)"),
        (-60, "왼쪽 회전 (중간)"),
        (120, "오른쪽 회전 (최대)"),
        (-120, "왼쪽 회전 (최대)"),
        (134, "범위 경계 (움직임)"),
        (-134, "범위 경계 (움직임)"),
        (135, "범위 초과 (정지)"),
        (-135, "범위 초과 (정지)"),
        (150, "범위 초과 (정지)"),
        (-150, "범위 초과 (정지)"),
        (100, "범위 복귀 (움직임 재개)"),
    ]

    for i, (error, description) in enumerate(test_cases, 1):
        print(f"[{i:2d}] error = {error:4d}° → {description}")
        mp.rotate_parafoil_motor(pi, error)
        print_motor_state(error, pi)
        print(f"  저장된 상태: L={mp.current_left_pulse}us, R={mp.current_right_pulse}us")
        print("-" * 70)

    print()
    print("=" * 70)
    print("✓ 테스트 완료")
    print("=" * 70)

if __name__ == "__main__":
    main()
