#!/usr/bin/env python3
"""
Parafoil motor control

Left motor (GPIO 12): 500 = pull up, 1500 = release down
Right motor (GPIO 13): 500 = pull up, 1500 = release down

Turn left: left release down (1500) + right pull up (500)
Turn right: left pull up (500) + right release down (1500)
Straight: both release down (1500, 1500)
"""

import time

PARAFOIL_LEFT_MOTOR_PIN = 12   # GPIO 12, physical pin 32
PARAFOIL_RIGHT_MOTOR_PIN = 13  # GPIO 13, physical pin 33

# Motor pulse range
MOTOR_UP = 500     # Pull up (line pull)
MOTOR_DOWN = 1500  # Release down (line release)

# Hardware-safe pulse boundaries (절대 1500 초과 금지!)
PULSE_MIN = 500
PULSE_MAX = 1500  # 12, 13번 모터 모두 1500 초과 펄스 금지

def init_parafoil_motor():
    """Initialize parafoil motor (both motors set to release down - straight position)."""
    import pigpio
    pi = pigpio.pi()
    # Initialize: both motors release down (straight position)
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, MOTOR_DOWN)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, MOTOR_DOWN)
    return pi

def terminate_parafoil_motor(pi):
    """Terminate parafoil motor (both motors set to release down, then stop PWM)."""
    if pi is not None:
        # On termination: both motors release down
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, MOTOR_DOWN)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, MOTOR_DOWN)
        time.sleep(0.1)
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, 0)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)

def _clamp_pulse(pulse: int) -> int:
    """Clamp servo pulsewidth to safe range (500-1500). Never exceed 1500!"""
    if pulse > 1500:
        pulse = 1500
    if pulse < 500:
        pulse = 500
    return pulse

def rotate_parafoil_motor(pi, turn: float):
    """
    Control parafoil motor (ON/OFF control).
    
    Args:
        pi: pigpio instance
        turn: Angle difference (-180 ~ +180 degrees)
              - Negative: turn left (left release down + right pull up)
              - Positive: turn right (left pull up + right release down)
    """
    TURN_THRESHOLD = 15  # Dead zone (±15 degrees)
    
    # Within dead zone: straight (both motors release down)
    if abs(turn) <= TURN_THRESHOLD:
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, MOTOR_DOWN)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, MOTOR_DOWN)
        return
    
    if turn < 0:  # Turn left: left release down (1500), right pull up (500)
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, MOTOR_DOWN)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, MOTOR_UP)
    else:  # Turn right: left pull up (500), right release down (1500)
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, MOTOR_UP)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, MOTOR_DOWN)
    
    return
