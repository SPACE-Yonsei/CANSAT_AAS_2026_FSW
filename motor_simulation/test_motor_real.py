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

pi.set_servo_pulsewidth(LEFT_PIN, 1170)
time.sleep(1)
pi.set_servo_pulsewidth(LEFT_PIN, 2500)
time.sleep(1)
pi.set_servo_pulsewidth(RIGHT_PIN, 1170)
time.sleep(1)
pi.set_servo_pulsewidth(RIGHT_PIN, 2500)

pi.set_servo_pulsewidth(LEFT_PIN, 0)
pi.set_servo_pulsewidth(RIGHT_PIN, 0)
pi.stop()
print("✓ 완료")
