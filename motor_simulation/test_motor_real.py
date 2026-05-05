#!/usr/bin/env python3
import pigpio
import time
import signal
import sys


RIGHT_PIN = 12
LEFT_PIN = 13

pulse_per_degree = 2000 / 180

left_zero = 600
right_zero = 2500
left_neutral = int(left_zero + 120 * pulse_per_degree)
right_neutral = int(right_zero - 120 * pulse_per_degree)

print(f"left_neutral:  {left_neutral}us  ({120:.1f} deg)")
print(f"right_neutral: {right_neutral}us  ({120:.1f} deg)")

pi = pigpio.pi()
if not pi.connected:
    print("ERROR: pigpio connection failed")
    sys.exit(1)


def shutdown(*_):
    print("\nCtrl+C detected — stopping motors and exiting")
    pi.set_servo_pulsewidth(LEFT_PIN,  0)
    pi.set_servo_pulsewidth(RIGHT_PIN, 0)
    pi.stop()
    sys.exit(0)


signal.signal(signal.SIGINT,  shutdown)
signal.signal(signal.SIGTERM, shutdown)

cycle = 0
while True:
    cycle += 1
    print(f"\n--- Cycle {cycle} ---")

    print(f"LEFT  motor -> 0 deg  ({left_zero}us)")
    pi.set_servo_pulsewidth(LEFT_PIN, left_zero)
    time.sleep(1)

    print(f"LEFT  motor -> 120 deg  ({left_neutral}us)")
    pi.set_servo_pulsewidth(LEFT_PIN, left_neutral)
    time.sleep(1)

    print(f"RIGHT motor -> 0 deg  ({right_zero}us)")
    pi.set_servo_pulsewidth(RIGHT_PIN, right_zero)
    time.sleep(1)

    print(f"RIGHT motor -> 120 deg  ({right_neutral}us)")
    pi.set_servo_pulsewidth(RIGHT_PIN, right_neutral)
    time.sleep(1)
