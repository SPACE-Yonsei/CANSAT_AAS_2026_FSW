#!/usr/bin/env python3
import time


PARAFOIL_RIGHT_MOTOR_PIN = 12  # GPIO 12, physical pin 32
PARAFOIL_LEFT_MOTOR_PIN = 13  # GPIO 13, physical pin 33

# 2500 when left angle is 0 angle
# 500 when right angle is 0 angle

pulse_per_degree = 2000/180
#max_angle_scope = 120 # degrees
#max_pulse_scope = max_angle_scope * pulse_per_degree

left_zero = 2500
right_zero = 600
MAX_ANGLE_SCOPE = 120  # degrees

right_neutral = int(right_zero + MAX_ANGLE_SCOPE * pulse_per_degree)
left_neutral = int(left_zero - MAX_ANGLE_SCOPE * pulse_per_degree)

THRESHOLD = 15  # degrees

# 현재 모터 위치 추적 (error가 범위를 넘으면 현재 위치 유지용)
current_left_pulse = left_neutral
current_right_pulse = right_neutral

def init_parafoil_motor():
    """Initialize parafoil motor (both motors set to release down - straight position)."""
    global current_left_pulse, current_right_pulse
    import pigpio
    pi = pigpio.pi()

    # 현재 위치를 neutral로 초기화
    current_left_pulse = left_neutral
    current_right_pulse = right_neutral

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
    
    print(f"operate before error={error:.1f}")
    if error>=135:
        error=135
    elif error<=-135:
        error=-135
    print(f"operate after error={error:.1f}")

    global current_left_pulse, current_right_pulse
        #go straight
    if abs(error) < THRESHOLD:
        left_pulse = left_neutral
        right_pulse = right_neutral
    elif error < 0:
        #turn left
        effective_error = error + THRESHOLD
        pulse_to_rotate_motor = int(effective_error * pulse_per_degree)

        left_pulse = left_neutral + pulse_to_rotate_motor
        right_pulse = right_neutral
    else:
        #turn right
        effective_error = error - THRESHOLD
        pulse_to_rotate_motor = int(effective_error * pulse_per_degree)

        left_pulse = left_neutral
        right_pulse = right_neutral + pulse_to_rotate_motor

    # 현재 상태 저장
    current_left_pulse = left_pulse
    current_right_pulse = right_pulse
    print(error)
    # 모터에 적용
    #print(f"Parafoil Motor Control - Left Pulse: {left_pulse}μs, Right Pulse: {right_pulse}μs")
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_pulse)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_pulse)