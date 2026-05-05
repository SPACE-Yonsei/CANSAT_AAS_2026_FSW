#!/usr/bin/env python3
import pigpio
import time
import signal
import sys


RIGHT_PIN = 12
LEFT_PIN  = 13

pulse_per_degree = 2000 / 180

left_zero    = 600
right_zero   = 2500
left_neutral = int(left_zero  + 120 * pulse_per_degree)
right_neutral = int(right_zero - 120 * pulse_per_degree)

# 60-deg intermediate positions
left_mid  = int(left_zero  +  60 * pulse_per_degree)
right_mid = int(right_zero -  60 * pulse_per_degree)

print(f"left_neutral:  {left_neutral}us  (120.0 deg)")
print(f"right_neutral: {right_neutral}us  (120.0 deg)")
print(f"left_mid:      {left_mid}us  ( 60.0 deg)")
print(f"right_mid:     {right_mid}us  ( 60.0 deg)")

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


def smooth_move(pin, start_us, end_us, steps=30, step_delay=0.02):
    """Interpolate from start_us to end_us in `steps` increments."""
    for i in range(1, steps + 1):
        pw = int(start_us + (end_us - start_us) * i / steps)
        pi.set_servo_pulsewidth(pin, pw)
        time.sleep(step_delay)


def move_left(from_us, to_us, label):
    deg = (to_us - left_zero) / pulse_per_degree
    print(f"LEFT  motor -> {deg:.0f} deg  ({to_us}us)")
    smooth_move(LEFT_PIN, from_us, to_us)
    time.sleep(1)
    return to_us


def move_right(from_us, to_us, label):
    deg = (right_zero - to_us) / pulse_per_degree
    print(f"RIGHT motor -> {deg:.0f} deg  ({to_us}us)")
    smooth_move(RIGHT_PIN, from_us, to_us)
    time.sleep(1)
    return to_us


cycle = 0
l_cur = left_zero
r_cur = right_zero

while True:
    cycle += 1
    print(f"\n--- Cycle {cycle} ---")

    # LEFT: 0 -> 60 -> 120
    l_cur = move_left(l_cur, left_zero, "0 deg")
    l_cur = move_left(l_cur, left_mid,  "60 deg")
    l_cur = move_left(l_cur, left_neutral, "120 deg")

    # RIGHT: 0 -> 60 -> 120
    r_cur = move_right(r_cur, right_zero, "0 deg")
    r_cur = move_right(r_cur, right_mid,  "60 deg")
    r_cur = move_right(r_cur, right_neutral, "120 deg")
