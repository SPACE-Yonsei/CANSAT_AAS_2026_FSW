#!/usr/bin/env python3
import time
import pigpio

PARAFOIL_LEFT_MOTOR_PIN = 12
PARAFOIL_RIGHT_MOTOR_PIN = 13

MIN_PULSE_WIDTH = 500
MAX_PULSE_WIDTH = 2500

pulse_per_degree = 2000 / 180
max_angle_scope = 120
max_pulse_scope = max_angle_scope * pulse_per_degree

left_zero = 2500
right_zero = 500

left_neutral = int(left_zero - max_pulse_scope)   # 1167
right_neutral = int(right_zero + max_pulse_scope)  # 1833

THRESHOLD = 15
SMOOTH_STEP = 50  # 한 번에 이동할 최대 펄스 폭 (부드러운 정도 조절)

# Global variables - 현재 모터 위치 기억
current_left_pulse = left_neutral
current_right_pulse = right_neutral


def init_parafoil_motor():
    """Initialize parafoil motor."""
    global current_left_pulse, current_right_pulse
    import pigpio
    pi = pigpio.pi()
    
    current_left_pulse = left_neutral
    current_right_pulse = right_neutral
    
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, current_left_pulse)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, current_right_pulse)
    return pi


def terminate_parafoil_motor(pi):
    """Terminate parafoil motor."""
    global current_left_pulse, current_right_pulse
    if pi is not None:
        # Smoothly return to neutral
        while abs(current_left_pulse - left_neutral) > 10 or abs(current_right_pulse - right_neutral) > 10:
            if current_left_pulse < left_neutral:
                current_left_pulse = min(current_left_pulse + SMOOTH_STEP, left_neutral)
            elif current_left_pulse > left_neutral:
                current_left_pulse = max(current_left_pulse - SMOOTH_STEP, left_neutral)
                
            if current_right_pulse < right_neutral:
                current_right_pulse = min(current_right_pulse + SMOOTH_STEP, right_neutral)
            elif current_right_pulse > right_neutral:
                current_right_pulse = max(current_right_pulse - SMOOTH_STEP, right_neutral)
            
            pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, current_left_pulse)
            pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, current_right_pulse)
            time.sleep(0.05)
        
        current_left_pulse = left_neutral
        current_right_pulse = right_neutral
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, current_left_pulse)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, current_right_pulse)
        time.sleep(0.1)
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, 0)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)
        pi.stop()


def rotate_parafoil_motor(pi, error: float):
    """Control parafoil motor with smooth transition."""
    global current_left_pulse, current_right_pulse

    # Clamp error
    real_max_angle_scope = max_angle_scope + THRESHOLD
    error = max(-real_max_angle_scope, min(real_max_angle_scope, error))

    # Calculate target pulses
    if abs(error) < THRESHOLD:
        target_left_pulse = left_neutral
        target_right_pulse = right_neutral
    elif error < 0:  # Left turn
        effective_error = error + THRESHOLD
        pulse_to_rotate = int(effective_error * pulse_per_degree)

        target_left_pulse = left_neutral
        target_right_pulse = right_neutral + pulse_to_rotate
    else:  # Right turn
        effective_error = error - THRESHOLD
        pulse_to_rotate = int(effective_error * pulse_per_degree)

        target_left_pulse = left_neutral + pulse_to_rotate
        target_right_pulse = right_neutral

    # Smooth transition to target pulse
    if current_left_pulse < target_left_pulse:
        current_left_pulse = min(current_left_pulse + SMOOTH_STEP, target_left_pulse)
    elif current_left_pulse > target_left_pulse:
        current_left_pulse = max(current_left_pulse - SMOOTH_STEP, target_left_pulse)

    if current_right_pulse < target_right_pulse:
        current_right_pulse = min(current_right_pulse + SMOOTH_STEP, target_right_pulse)
    elif current_right_pulse > target_right_pulse:
        current_right_pulse = max(current_right_pulse - SMOOTH_STEP, target_right_pulse)

    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, current_left_pulse)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, current_right_pulse)