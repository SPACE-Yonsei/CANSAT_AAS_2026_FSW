#!/usr/bin/env python3
import time

PARAFOIL_LEFT_MOTOR_PIN = 12 # GPIO 12, physical pin 32
PARAFOIL_RIGHT_MOTOR_PIN = 13 # GPIO 13, physical pin 33

# 2500 when left angle is 0 angle
# 500 when right angle is 0 angle

pulse_per_degree = 2000/180
#max_angle_scope = 120 # degrees
#max_pulse_scope = max_angle_scope * pulse_per_degree

right_zero = 2500
left_zero = 600
MAX_ANGLE_SCOPE = 120  # degrees

left_neutral = int(left_zero + 60 * pulse_per_degree)
right_neutral = int(right_zero - 60 * pulse_per_degree)

THRESHOLD = 8  # degrees

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

def rotate_parafoil_motor(pi,yaw, error: float):
    global current_left_pulse, current_right_pulse

    if error>=MAX_ANGLE_SCOPE+THRESHOLD:
        error=MAX_ANGLE_SCOPE+THRESHOLD #128
    elif error<=-(MAX_ANGLE_SCOPE+THRESHOLD):
        error=-(MAX_ANGLE_SCOPE+THRESHOLD) #-128

    abs_error_for_turn = abs(error) - THRESHOLD
    pulse_to_rotate_motor = 2*abs(int(abs_error_for_turn/2 * pulse_per_degree))

    if error > THRESHOLD:
        
        left_pulse = left_neutral + pulse_to_rotate_motor
        right_pulse = right_neutral + pulse_to_rotate_motor

        left_pulse=min(2500,left_pulse)
        right_pulse=min(2500,right_pulse)

        print(f"right moved => error: {error}, left_pulse: {left_pulse}, right_pulse: {right_pulse}, effective_error: {abs_error_for_turn}")

    elif error < -THRESHOLD:
        
        left_pulse = left_neutral - pulse_to_rotate_motor
        right_pulse = right_neutral - pulse_to_rotate_motor
        
        # if left_pulse < 600:
        #     left_pulse = 600
        # if right_pulse < 600:
        #     right_pulse = 600
        left_pulse=max(600,left_pulse)
        right_pulse=max(600,right_pulse)

        print(f"left moved => error: {error}, left_pulse: {left_pulse}, right_pulse: {right_pulse}, effective_error: {abs_error_for_turn}")
    else: #go straight
        left_pulse = left_neutral
        right_pulse = right_neutral
        print(f"both neutral => error: {error}, left_pulse: {left_pulse}, right_pulse: {right_pulse}")
    
    # 현재 상태 저장
    current_left_pulse = left_pulse
    current_right_pulse = right_pulse

    # 모터에 적용
    #print(f"Parafoil Motor Control - Left Pulse: {left_pulse}μs, Right Pulse: {right_pulse}μs")
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_pulse)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_pulse)