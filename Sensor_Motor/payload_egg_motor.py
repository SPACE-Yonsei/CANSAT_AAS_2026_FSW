#!/usr/bin/env python3
# Payload-Egg ejection motor control (MG92B, angle control)
# GPIO 6: Payload-Egg ejection motor

import time

PAYLOAD_EGG_MOTOR_PIN = 5
PAYLOAD_EGG_INITIAL_DEGREE = 0
PAYLOAD_EGG_RELEASE_DEGREE = 90
PAYLOAD_EGG_MOTOR_MIN_PULSE = 500
PAYLOAD_EGG_MOTOR_MAX_PULSE = 2500

def angle_to_pulse(angle) -> int:
    if angle < 0:
        angle = 0
    elif angle > 180:
        angle = 180
    
    return int(PAYLOAD_EGG_MOTOR_MIN_PULSE + ((angle/180)*(PAYLOAD_EGG_MOTOR_MAX_PULSE - PAYLOAD_EGG_MOTOR_MIN_PULSE)))

def init_MG92B():
    import pigpio
    pi = pigpio.pi()
    if pi is None:
        raise RuntimeError("Failed to initialize pigpio")
    pi.set_servo_pulsewidth(PAYLOAD_EGG_MOTOR_PIN, angle_to_pulse(PAYLOAD_EGG_INITIAL_DEGREE))
    return pi

def egg_motor_initial(pi):
    """Set egg motor to initial position (holding egg)."""
    if pi is None:
        return
    pi.set_servo_pulsewidth(PAYLOAD_EGG_MOTOR_PIN, angle_to_pulse(PAYLOAD_EGG_INITIAL_DEGREE))
    return

def egg_motor_release(pi):
    """Set egg motor to release position (dropping egg)."""
    if pi is None:
        return
    pi.set_servo_pulsewidth(PAYLOAD_EGG_MOTOR_PIN, angle_to_pulse(PAYLOAD_EGG_RELEASE_DEGREE))
    return

def terminate_MG92B(pi):
    """Terminate egg motor (stop PWM)."""
    if pi is None:
        return
    pi.set_servo_pulsewidth(PAYLOAD_EGG_MOTOR_PIN, 0)
    # Note: pi.stop() should be called by the caller to clean up all motors
    return

