#!/usr/bin/env python3
import time
import pigpio

PARAFOIL_LEFT_MOTOR_PIN = 12   # GPIO 12, physical pin 32
PARAFOIL_RIGHT_MOTOR_PIN = 13  # GPIO 13, physical pin 33

# 2500 when left angle is 0 angle
# 500 when right angle is 0 angle

purse_per_degree = 2000/180
max_angle_scope = 120 # degrees
max_purse_scope = max_angle_scope * purse_per_degree

left_zero= 2500
right_zero = 500

left_neutral = left_zero - max_purse_scope
right_neutral = right_zero + max_purse_scope

THRESHOLD = 15  # degrees

def init_parafoil_motor():
    """Initialize parafoil motor (both motors set to release down - straight position)."""
    import pigpio
    pi = pigpio.pi()
    # Initialize: both motors release down (straight position)
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_neutral)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_neutral)
    return pi

def terminate_parafoil_motor(pi):
    """Terminate parafoil motor (both motors set to release down, then stop PWM)."""
    if pi is not None:
        # On termination: both motors release down
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_neutral)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_neutral)
        time.sleep(0.1)
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, 0)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)

def rotate_parafoil_motor(pi, error: float):
    """
    Control parafoil motor with smooth transition.

    Neutral (straight): Both motors at 120° (1167, 1833)
    Left turn: Left fixed, Right lowers (1833 → 500)
    Right turn: Right fixed, Left lowers (1167 → 2500)
    Range: ±135° input (±15° dead zone + ±120° control)
    """
    
    real_max_angle_scope = max_angle_scope + THRESHOLD
    if error > real_max_angle_scope:
        error = real_max_angle_scope
    elif error < -real_max_angle_scope:
        error = -real_max_angle_scope

    if abs(error) < THRESHOLD:
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_neutral)   # 1167
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_neutral) # 1833
        return

    if error < 0:
        effective_error = error + THRESHOLD
        purse_to_rotate_motor = int(effective_error * purse_per_degree)

        left_pulse = left_neutral + purse_to_rotate_motor
        right_pulse = right_neutral
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_pulse)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_pulse)

    else:
        
        effective_error = error - THRESHOLD
        purse_to_rotate_motor = int(effective_error * purse_per_degree)

        left_pulse = left_neutral
        right_pulse = right_neutral + purse_to_rotate_motor

        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_pulse)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_pulse)
