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


# PID 상태 변수
pid_last_time = time.time()
pid_integral = 0.0
pid_prev_error = 0.0
pid_prev_yaw = None

# PID 게인 (튜닝 필요)
Kp = 1.0      # 비례 게인
Ki = 0.05    # 적분 게인 (느린 정상상태 오차 제거)
Kd = 0.01    # 미분 게인 (오버슈트 억제)
INTEGRAL_MAX = 50.0  # Anti-windup 한계

def reset_pid():
    global pid_last_time, pid_integral, pid_prev_error, pid_prev_yaw
    pid_last_time = time.time()
    pid_integral = 0.0
    pid_prev_error = 0.0
    pid_prev_yaw = None


def compute_pid(error: float, yaw: float = None) -> float:
    """
    PID 제어 출력 계산
    Returns: p + i + d (도 단위, 양수=오른쪽, 음수=왼쪽)
    """
    global pid_last_time, pid_integral, pid_prev_error, pid_prev_yaw

    current_time = time.time()
    dt = current_time - pid_last_time
    if dt <= 0 or dt > 0.5:
        dt = 0.1
    pid_last_time = current_time

    # 데드밴드 내: 적분 리셋, 출력 0
    if abs(error) <= THRESHOLD:
        pid_integral = 0.0
        pid_prev_error = 0.0
        return 0.0

    # error 범위 제한
    if error>=MAX_ANGLE_SCOPE+THRESHOLD:
        error=MAX_ANGLE_SCOPE+THRESHOLD #128
    elif error<=-(MAX_ANGLE_SCOPE+THRESHOLD):
        error=-(MAX_ANGLE_SCOPE+THRESHOLD) #-128
        
    # 데드밴드 제외한 effective error
    if error > 0:
        effective_error = error - THRESHOLD
    else:
        effective_error = error + THRESHOLD

    # --- P항 ---
    p_term = Kp * effective_error

    # --- I항 (Anti-windup) ---
    pid_integral += effective_error * dt *100
    # pid_integral = max(-INTEGRAL_MAX, min(INTEGRAL_MAX, pid_integral))
    if pid_integral > INTEGRAL_MAX:
        pid_integral = INTEGRAL_MAX
    elif pid_integral < -INTEGRAL_MAX:
        pid_integral = -INTEGRAL_MAX
    i_term = Ki * pid_integral

    # --- D항 (Derivative on Measurement) ---
    if yaw is not None and pid_prev_yaw is not None:
        delta_yaw = yaw - pid_prev_yaw
        if delta_yaw > 180:
            delta_yaw -= 360
        elif delta_yaw < -180:
            delta_yaw += 360
        d_term = -Kd * (delta_yaw / dt)
    else:
        d_term = Kd * (effective_error - pid_prev_error) / dt

    # 상태 저장
    pid_prev_error = effective_error
    if yaw is not None:
        pid_prev_yaw = yaw

    output = p_term + i_term + d_term
    print(f"\nPID Compute => error: {error:.1f}, P: {p_term:.2f}, I: {i_term:.2f}, D: {d_term:.2f}, Output: {output:.2f}")
    output = max(-MAX_ANGLE_SCOPE, min(MAX_ANGLE_SCOPE, output))

    return output

def rotate_parafoil_motor(pi, yaw: float, error: float):
    """
    PID 기반 모터 제어
    호출: rotate_parafoil_motor(pi, yaw, error)
    """
    global current_left_pulse, current_right_pulse

    # PID 출력 계산
    pid_output = compute_pid(error, yaw)

    if pid_output == 0.0:
        left_pulse = left_neutral
        right_pulse = right_neutral
        print(f"neutral => error: {error:.1f}, pid: {pid_output:.2f}, "
              f"L: {left_pulse}, R: {right_pulse}\n")
    else:
        abs_output = abs(pid_output)
        e = int(abs(abs_output / 2 * pulse_per_degree))

        if pid_output > 0:
            left_pulse = left_neutral + e
            right_pulse = right_neutral + e
            left_pulse = min(2500, left_pulse)
            right_pulse = min(2500, right_pulse)
            print(f"RIGHT => error: {error:.1f}, pid: {pid_output:.2f}, "
                  f"L: {left_pulse}, R: {right_pulse}\n")
        else:
            left_pulse = left_neutral - e
            right_pulse = right_neutral - e
            left_pulse = max(600, left_pulse)
            right_pulse = max(600, right_pulse)
            print(f"LEFT => error: {error:.1f}, pid: {pid_output:.2f}, "
                  f"L: {left_pulse}, R: {right_pulse}\n")

    current_left_pulse = left_pulse
    current_right_pulse = right_pulse

    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_pulse)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_pulse)

# def rotate_parafoil_motor(pi, yaw, error: float):
#     global current_left_pulse, current_right_pulse

#     if error>=MAX_ANGLE_SCOPE+THRESHOLD:
#         error=MAX_ANGLE_SCOPE+THRESHOLD #128
#     elif error<=-(MAX_ANGLE_SCOPE+THRESHOLD):
#         error=-(MAX_ANGLE_SCOPE+THRESHOLD) #-128

#     abs_error_for_turn = abs(error) - THRESHOLD
#     e = abs(int(abs_error_for_turn/2 * pulse_per_degree))
    
#     if error > THRESHOLD:
        
#         left_pulse = left_neutral + e
#         right_pulse = right_neutral + e

#         left_pulse=min(2500,left_pulse)
#         right_pulse=min(2500,right_pulse)

#         print(f"right moved => error: {error}, left_pulse: {left_pulse}, right_pulse: {right_pulse}, effective_error: {abs_error_for_turn}")

#     elif error < -THRESHOLD:
        
#         left_pulse = left_neutral - e
#         right_pulse = right_neutral - e
        
#         # if left_pulse < 600:
#         #     left_pulse = 600
#         # if right_pulse < 600:
#         #     right_pulse = 600
#         left_pulse=max(600,left_pulse)
#         right_pulse=max(600,right_pulse)

#         print(f"left moved => error: {error}, left_pulse: {left_pulse}, right_pulse: {right_pulse}, effective_error: {abs_error_for_turn}")
#     else: #go straight
#         left_pulse = left_neutral
#         right_pulse = right_neutral
#         print(f"both neutral => error: {error}, left_pulse: {left_pulse}, right_pulse: {right_pulse}")
    
#     # 현재 상태 저장
#     current_left_pulse = left_pulse
#     current_right_pulse = right_pulse

#     # 모터에 적용
#     #print(f"Parafoil Motor Control - Left Pulse: {left_pulse}μs, Right Pulse: {right_pulse}μs")
#     pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_pulse)
#     pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_pulse)
