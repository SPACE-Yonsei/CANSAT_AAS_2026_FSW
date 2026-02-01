#!/usr/bin/env python3
import time
import pigpio

PARAFOIL_LEFT_MOTOR_PIN = 12   # GPIO 12, physical pin 32
PARAFOIL_RIGHT_MOTOR_PIN = 13  # GPIO 13, physical pin 33

# 2500 when left angle is 0 angle
# 500 when right angle is 0 angle

pulse_per_degree = 2000/180
max_angle_scope = 120 # degrees
max_pulse_scope = max_angle_scope * pulse_per_degree

left_zero = 2500
right_zero = 500

left_neutral = int(left_zero - max_pulse_scope)
right_neutral = int(right_zero + max_pulse_scope)

THRESHOLD = 15  # degrees

# 현재 모터 위치 추적 (error가 범위를 넘으면 현재 위치 유지용)
current_left_pulse = left_neutral
current_right_pulse = right_neutral

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
    Control parafoil motor based on error (direction).

    Neutral (straight): Both motors at 120° (1167, 1833)
    Left turn (error < 0): Left fixed at neutral, Right lowers (1833 → 500)
    Right turn (error > 0): Right fixed at neutral, Left raises (1167 → 2500)
    Range: ±135° input (±15° dead zone + ±120° control)

    방향에 따라서만 모터를 움직임 (원상복귀 없음)
    """

    # error 값 제한 (±135도)
    real_max_angle_scope = max_angle_scope + THRESHOLD
    if error > real_max_angle_scope:
        error = real_max_angle_scope
    elif error < -real_max_angle_scope:
        error = -real_max_angle_scope

    if abs(error) < THRESHOLD:
        # Dead zone: 직진
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_neutral)   # 1167
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_neutral) # 1833
        return

    if error < 0:
        # 왼쪽 회전: 왼쪽 모터 neutral, 오른쪽 모터를 error에 비례해서 작동
        effective_error = error + THRESHOLD
        pulse_to_rotate_motor = int(effective_error * pulse_per_degree)

        left_pulse = left_neutral
        right_pulse = right_neutral + pulse_to_rotate_motor
    else:
        # 오른쪽 회전: 오른쪽 모터 neutral, 왼쪽 모터를 error에 비례해서 작동
        effective_error = error - THRESHOLD
        pulse_to_rotate_motor = int(effective_error * pulse_per_degree)

        left_pulse = left_neutral + pulse_to_rotate_motor
        right_pulse = right_neutral

    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_pulse)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_pulse)
