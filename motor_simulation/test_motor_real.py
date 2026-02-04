#!/usr/bin/env python3
import pigpio
import time

LEFT_PIN = 13
RIGHT_PIN = 12

pulse_per_degree = 2000/180

left_zero = 2500
right_zero = 600
left_neutral = int(left_zero - 120 * pulse_per_degree)
right_neutral = int(right_zero + 120 * pulse_per_degree)

print(f"left_neutral: {left_neutral}μs")
print(f"right_neutral: {right_neutral}μs")

pi = pigpio.pi()
if not pi.connected:
    print("❌ pigpio 연결 실패")
    exit()

print("LEFT 모터 0(2500) 이동..")
pi.set_servo_pulsewidth(LEFT_PIN, 2500)
time.sleep(10)

# print("LEFT 모터 120 이동...")
# pi.set_servo_pulsewidth(LEFT_PIN, left_neutral)
# time.sleep(10)

print("LEFT 모터 0(600) 이동..")
pi.set_servo_pulsewidth(LEFT_PIN, 600)
time.sleep(10)

print("RIGHT 모터 (600) 이동...")
pi.set_servo_pulsewidth(RIGHT_PIN, 600) 
time.sleep(10)

# print("RIGHT 모터 120 이동...")
# pi.set_servo_pulsewidth(RIGHT_PIN, right_neutral)
# time.sleep(10)

print("RIGHT 모터 (2500) 이동...")
pi.set_servo_pulsewidth(RIGHT_PIN, 2500) 
time.sleep(10)

# print("right 1500")
# pi.set_servo_pulsewidth(RIGHT_PIN, 1500)
# time.sleep(5)

# print("right 2500")
# pi.set_servo_pulsewidth(RIGHT_PIN, 2500)
# time.sleep(5)


# PWM 신호만 끄고 (모터 토크 해제)
pi.set_servo_pulsewidth(LEFT_PIN, 0)
pi.set_servo_pulsewidth(RIGHT_PIN, 0)

pi.stop()
print("✓ 완료")