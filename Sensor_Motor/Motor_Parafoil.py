#!/usr/bin/env python3
import time

PARAFOIL_LEFT_MOTOR_PIN = 12   # GPIO 12, physical pin 32
PARAFOIL_RIGHT_MOTOR_PIN = 13  # GPIO 13, physical pin 33

# Servo pulse width constants
MIN_PULSE_WIDTH = 500   # Minimum valid servo pulse width (µs)
MAX_PULSE_WIDTH = 2500  # Maximum valid servo pulse width (µs)

# Calibration values
# 2500 when left angle is 0 angle
# 500 when right angle is 0 angle
pulse_per_degree = 2000 / 180
max_angle_scope = 120  # degrees
max_pulse_scope = max_angle_scope * pulse_per_degree

left_zero = 2500
right_zero = 500
left_neutral = left_zero - max_pulse_scope   # 1167
right_neutral = right_zero + max_pulse_scope  # 1833

THRESHOLD = 15  # degrees (dead zone)


right_pulse = right_neutral
left_pulse = left_neutral

def init_parafoil_motor():
    """Initialize parafoil motor (both motors set to neutral - straight position)."""
    pi = pigpio.pi()
    
    # Initialize: both motors at neutral position (straight)
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_neutral)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_neutral)
    
    return pi


def terminate_parafoil_motor(pi):
    """Terminate parafoil motor (both motors set to neutral, then stop PWM)."""
    if pi is not None:
        # On termination: both motors to neutral
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_neutral)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_neutral)
        time.sleep(0.1)
        
        # Stop PWM signals
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, 0)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)
        
        pi.stop()


def rotate_parafoil_motor(pi, error: float):
    """
<<<<<<< HEAD
    Control parafoil motor with smooth transition.ㅇ

    Neutral (straight): Both motors at 120° (1167, 1833)
    Left turn: Left fixed, Right lowers (1833 → 500)
    Right turn: Right fixed, Left lowers (1167 → 2500)
    Range: ±135° input (±15° dead zone + ±120° control)
    """
    global left_pulse, right_pulse
=======
    Control parafoil motor with smooth transition.
    
    Neutral (straight): Both motors at 120° (1167, 1833)
    Left turn (error < 0): Right motor fixed at 1833, Left lowers (1167 → 500)
    Right turn (error > 0): Left motor fixed at 1167, Right raises (1833 → 2500)
    
    Args:
        pi: pigpio.pi() instance
        error: Heading error in degrees
               Range: ±135° input (±15° dead zone + ±120° control)
    """
    # Clamp error to maximum range
>>>>>>> d1d33e28f86c9afda4a9ee397bdcb4739bae05a1
    real_max_angle_scope = max_angle_scope + THRESHOLD
    
    if error > real_max_angle_scope:
        error = real_max_angle_scope
    elif error < -real_max_angle_scope:
        error = -real_max_angle_scope
    
    # Dead zone: if error is small, go straight
    if abs(error) < THRESHOLD:
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, int(left_neutral))   # 1167
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, int(right_neutral))  # 1833
        return
<<<<<<< HEAD

    if error < 0:
        effective_error = error + THRESHOLD
        purse_to_rotate_motor = int(effective_error * purse_per_degree)

        #left_pulse = left_neutral
        right_pulse = right_neutral + purse_to_rotate_motor
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_pulse)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_pulse)

    else:
        
        effective_error = error - THRESHOLD
        purse_to_rotate_motor = int(effective_error * purse_per_degree)

        left_pulse = left_neutral + purse_to_rotate_motor
        #right_pulse = right_neutral

        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_pulse)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_pulse)
=======
    
    # Calculate effective error (subtract dead zone)
    if error < 0:  # Left turn
        effective_error = error + THRESHOLD  # -135 → -120
        pulse_to_rotate_motor = int(effective_error * pulse_per_degree)
        
        # Left motor moves down, Right motor fixed
        left_pulse = left_neutral + pulse_to_rotate_motor  # 1167 + (-1333) would be negative
        right_pulse = right_neutral
        
    else:  # Right turn
        effective_error = error - THRESHOLD  # +135 → +120
        pulse_to_rotate_motor = int(effective_error * pulse_per_degree)
        
        # Right motor moves up, Left motor fixed
        left_pulse = left_neutral
        right_pulse = right_neutral + pulse_to_rotate_motor  # 1833 + 1333 = 3166 would exceed max
    
    # Safety clamp: ensure pulse widths are within valid servo range
    left_pulse = max(MIN_PULSE_WIDTH, min(MAX_PULSE_WIDTH, int(left_pulse)))
    right_pulse = max(MIN_PULSE_WIDTH, min(MAX_PULSE_WIDTH, int(right_pulse)))
    
    # Apply pulse widths
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_pulse)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_pulse)


# Example usage
if __name__ == "__main__":
    pi = init_parafoil_motor()
    
    try:
        # Test: straight
        print("Straight")
        rotate_parafoil_motor(pi, 0)
        time.sleep(2)
        
        # Test: left turn
        print("Left turn -60°")
        rotate_parafoil_motor(pi, -60)
        time.sleep(2)
        
        # Test: right turn
        print("Right turn +60°")
        rotate_parafoil_motor(pi, 60)
        time.sleep(2)
        
        # Test: max left
        print("Max left -135°")
        rotate_parafoil_motor(pi, -135)
        time.sleep(2)
        
        # Test: max right
        print("Max right +135°")
        rotate_parafoil_motor(pi, 135)
        time.sleep(2)
        
    finally:
        # Return to neutral and terminate
        print("Terminate")
        terminate_parafoil_motor(pi)
>>>>>>> d1d33e28f86c9afda4a9ee397bdcb4739bae05a1
