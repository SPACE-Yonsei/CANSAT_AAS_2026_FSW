#!/usr/bin/env python3
import pigpio
import time

# GPIO 핀
LEFT_PIN = 12
RIGHT_PIN = 13

# 각도를 펄스로 변환 (MG92B: 0°=500us, 180°=2500us)
def angle_to_pulse(angle):
    return int(500 + (angle * 2000 / 180))

# 테스트
pi = pigpio.pi()
test_angles = [0, 135, 180]

print("=" * 60)
print("MG92B 서보 모터 테스트 (GPIO 12: 왼쪽, GPIO 13: 오른쪽)")
print("=" * 60)

for angle in test_angles:
    pulse = angle_to_pulse(angle)

    print(f"\n각도: {angle}° → 펄스: {pulse} us")

    # 왼쪽 모터
    pi.set_servo_pulsewidth(LEFT_PIN, pulse)
    print(f"  왼쪽 모터 (GPIO {LEFT_PIN}): {pulse} us")

    # 오른쪽 모터
    pi.set_servo_pulsewidth(RIGHT_PIN, pulse)
    print(f"  오른쪽 모터 (GPIO {RIGHT_PIN}): {pulse} us")

    time.sleep(2)

# 종료
print("\n중립 위치(90°)로 복귀")
neutral_pulse = angle_to_pulse(90)
pi.set_servo_pulsewidth(LEFT_PIN, neutral_pulse)
pi.set_servo_pulsewidth(RIGHT_PIN, neutral_pulse)
time.sleep(1)

pi.set_servo_pulsewidth(LEFT_PIN, 0)
pi.set_servo_pulsewidth(RIGHT_PIN, 0)
pi.stop()
print("✓ 완료")
