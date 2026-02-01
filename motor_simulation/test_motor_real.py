#!/usr/bin/env python3
import pigpio
import time

# GPIO 핀
LEFT_PIN = 12
RIGHT_PIN = 13

pulse_per_degree = 2000/180

left_neutral = int(left_zero - 30 * pulse_per_degree)
right_neutral = int(right_zero + 30* pulse_per_degree)

# 테스트
pi = pigpio.pi()

pi.set_servo_pulsewidth(LEFT_PIN, left_neutral)
time.sleep(1)

pi.set_servo_pulsewidth(RIGHT_PIN, right_neutral)
time.sleep(1)
pi.stop()
print("✓ 완료")
