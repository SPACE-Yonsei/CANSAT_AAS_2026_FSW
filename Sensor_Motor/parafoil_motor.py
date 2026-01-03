#!/usr/bin/env python3

#from gpiozero import AngularServo
#import math, random, time

import time

PARAFOIL_LEFT_MOTOR_PIN = 12 # gpio 12, physical pin 32
PARAFOIL_RIGHT_MOTOR_PIN = 13  # gpio 13, physical pin 33

# Calibrate the pulse range
PARAFOIL_LEFT_MOTOR_MIN_PULSE = 1500   # Neutral/stop position
PARAFOIL_LEFT_MOTOR_MAX_PULSE = 2470   # Maximum speed (reverse)
PARAFOIL_RIGHT_MOTOR_MIN_PULSE = 530   # Maximum speed (forward)
PARAFOIL_RIGHT_MOTOR_MAX_PULSE = 1500  # Neutral/stop position

# Hardware-safe pulse boundaries
PULSE_MIN = 1400
PULSE_MAX = 2400

def init_parafoil_motor():
    import pigpio
    pi = pigpio.pi()
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, 0)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)
    return pi

def terminate_parafoil_motor(pi):
    if pi is not None:
        pi.stop()

def _clamp_pulse(pulse: int) -> int:
    """Clamp servo pulsewidth to the safe 1400-2400 µs range."""
    return max(PULSE_MIN, min(PULSE_MAX, pulse))

def rotate_parafoil_motor(pi, turn: float):
    """
    Control parafoil motors with proportional speed control.
    
    Args:
        pi: pigpio instance
        turn: Angle difference in degrees (-180 to +180)
              - Negative: turn left (use left motor)
              - Positive: turn right (use right motor)
    """
    TURN_THRESHOLD = 15  # Dead zone in degrees (±15 degrees)
    MAX_TURN_ANGLE = 90  # Maximum turn angle for full speed
    
    # If within dead zone, stop motors (set to neutral position)
    if abs(turn) <= TURN_THRESHOLD:
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, PARAFOIL_LEFT_MOTOR_MIN_PULSE)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, PARAFOIL_RIGHT_MOTOR_MAX_PULSE)
        return
    
    # Calculate turn magnitude (absolute value)
    turn_magnitude = abs(turn)
    
    # Effective turn after subtracting dead zone
    # Range: 0 to (MAX_TURN_ANGLE - TURN_THRESHOLD)
    effective_turn = min(turn_magnitude - TURN_THRESHOLD, MAX_TURN_ANGLE - TURN_THRESHOLD)
    
    # Calculate speed ratio (0.0 to 1.0)
    speed_ratio = effective_turn / (MAX_TURN_ANGLE - TURN_THRESHOLD)
    
    if turn < 0:  # Turn Left - use left motor
        # Left motor: 1500 (neutral) to 2470 (max speed reverse)
        # speed_ratio 0.0 -> 1500, speed_ratio 1.0 -> 2470
        left_pulse_range = PARAFOIL_LEFT_MOTOR_MAX_PULSE - PARAFOIL_LEFT_MOTOR_MIN_PULSE
        left_motor_pulse = int(PARAFOIL_LEFT_MOTOR_MIN_PULSE + (speed_ratio * left_pulse_range))
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, _clamp_pulse(left_motor_pulse))
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, _clamp_pulse(PARAFOIL_RIGHT_MOTOR_MAX_PULSE))  # Right motor neutral
    else:  # Turn Right - use right motor
        # Right motor: 1500 (neutral) to 530 (max speed forward)
        # speed_ratio 0.0 -> 1500, speed_ratio 1.0 -> 530
        right_pulse_range = PARAFOIL_RIGHT_MOTOR_MAX_PULSE - PARAFOIL_RIGHT_MOTOR_MIN_PULSE
        right_motor_pulse = int(PARAFOIL_RIGHT_MOTOR_MAX_PULSE - (speed_ratio * right_pulse_range))
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, _clamp_pulse(PARAFOIL_LEFT_MOTOR_MIN_PULSE))  # Left motor neutral
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, _clamp_pulse(right_motor_pulse))
    
    return
