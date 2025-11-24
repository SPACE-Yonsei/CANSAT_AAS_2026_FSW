import time

TARGET_DEGREE = 0
PARAFOIL_LEFT_MOTOR_PIN = 12
PARAFOIL_RIGHT_MOTOR_PIN = 13

# Calibrate the pulse range, us unit
PARAFOIL_MOTOR_MIN_PULSE = 530
PARAFOIL_MOTOR_MAX_PULSE = 2470
PARAFOIL_MOTOR_MID_PULSE = 1500
PARAFOIL_MOTOR_STOP_PULSE = 0  # 0us, Stop generating pulses

# def angle_to_pulse(angle) -> int:
#     if angle < 0:
#         angle = 0
#     elif angle > 180:
#         angle = 180
    
#     return int(PARAFOIL_MOTOR_MIN_PULSE + ((angle/180)*(PARAFOIL_MOTOR_MAX_PULSE - PARAFOIL_MOTOR_MIN_PULSE)))



# def rotate_parafoil_motor(pi, turn:float):

#     TURN_THRESHOLD = 15
#     willing_to_turn = turn # 모터가 돌아야 하는 각도 - 180~ 180도 사이로 입력

#     if willing_to_turn < -TURN_THRESHOLD:
#         # Turn Left: Pull the left motor line
#         pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, PARAFOIL_MOTOR_MAX_PULSE)
#         pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, PARAFOIL_MOTOR_STOP_PULSE)
#     elif willing_to_turn > TURN_THRESHOLD:
#         # Turn Right: Pull the right motor line
#         pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, PARAFOIL_MOTOR_STOP_PULSE)
#         pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, PARAFOIL_MOTOR_MAX_PULSE)
#     else:
#         # Go Straight: Keep motors idle
#         pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, PARAFOIL_MOTOR_STOP_PULSE)
#         pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, PARAFOIL_MOTOR_STOP_PULSE)

#     return



def init_parafoil_motor():
    import pigpio
    pi = pigpio.pi()
    if not pi.connected:
        # pigpiod 데몬에 연결 실패 시 예외 발생
        raise RuntimeError("Could not connect to pigpiod. Is it running?")
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, 0)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)
    return pi

def terminate_parafoil_motor(pi):
    pi.stop()

def angle_to_purse(angle):
    return int(PARAFOIL_MOTOR_MIN_PULSE + ((angle/45)*(PARAFOIL_MOTOR_MAX_PULSE - PARAFOIL_MOTOR_MIN_PULSE)))
#############################################

# dead_zone_deg = 10.0
#         if abs(error) < dead_zone_deg:
#             error = 0.0
        
#         elif error > 90:
#             error = 90
#         elif error < -90:
#             error = -90



if __name__ == "__main__":
    pi = init_parafoil_motor()

    try:
        print("모터를 0도 -> 80도 -> 0도로 왕복 운동합니다.")
        print("프로그램을 종료하려면 Ctrl+C를 누르세요.")

        while True:
            print("\n--- 0도에서 80도로 이동 ---")
            # 0부터 90 미만까지 10씩 증가 (0, 10, 20, ..., 80)
            for angle in range(0, 90, 10):
                pulse = angle_to_purse(angle)
                print(f"Angle: {angle}° -> Pulse: {pulse}µs")
                pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, pulse)
                pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, pulse)
                time.sleep(0.5)

            time.sleep(1) # 최대 각도에서 잠시 대기

            print("\n--- 80도에서 0도로 이동 ---")
            # 80부터 0까지 10씩 감소 (80, 70, ..., 0)
            for angle in range(80, -1, -10):
                pulse = angle_to_purse(angle)
                print(f"Angle: {angle}° -> Pulse: {pulse}µs")
                pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, pulse)
                pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, pulse)
                time.sleep(0.5)
            
            time.sleep(2) # 한 사이클 후 잠시 대기

    except KeyboardInterrupt:
        print("\n프로그램을 종료합니다.")

    finally:
        print("모터를 정지합니다.")
        terminate_parafoil_motor(pi)
