import time

TARGET_DEGREE = 0
PARAFOIL_LEFT_MOTOR_PIN = 12 # gpio 12, physical pin 32
PARAFOIL_RIGHT_MOTOR_PIN = 13  # gpio 13, physical pin 33

# Calibrate the pulse range, us unit
PARAFOIL_MOTOR_MIN_PULSE = 1250
PARAFOIL_MOTOR_MAX_PULSE = 1750
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

#############################################


if __name__ == "__main__":
    pi = init_parafoil_motor()

    # 모터를 움직일 펄스 값들을 리스트로 정의
    pulse_positions = [
        PARAFOIL_MOTOR_MIN_PULSE,  # 최소 위치 (약 0도)
        #PARAFOIL_MOTOR_MID_PULSE,  # 중간 위치 (약 90도)
        PARAFOIL_MOTOR_MAX_PULSE,  # 최대 위치 (약 180도)
        #PARAFOIL_MOTOR_MID_PULSE   # 다시 중간 위치로
    ]

    try:
        print("서보 모터를 지정된 펄스 값으로 자동 순환합니다.")
        print("프로그램을 종료하려면 Ctrl+C를 누르세요.")

        while True:
            # 1. 왼쪽 모터 테스트 (MIN -> MAX -> STOP)
            print("--- 왼쪽 모터 테스트 ---")
            for pulse in pulse_positions:
                print(f"  - 왼쪽 모터 {pulse}µs 위치로 이동")
                pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, pulse)
                time.sleep(3)
            pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, PARAFOIL_MOTOR_STOP_PULSE) # 왼쪽 모터 정지
            time.sleep(1)

            # 2. 오른쪽 모터 테스트 (MIN -> MAX -> STOP)
            print("--- 오른쪽 모터 테스트 ---")
            for pulse in pulse_positions:
                print(f"  - 오른쪽 모터 {pulse}µs 위치로 이동")
                pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, pulse)
                time.sleep(3)
            pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, PARAFOIL_MOTOR_STOP_PULSE) # 오른쪽 모터 정지
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n프로그램을 종료합니다.")

    finally:
        terminate_parafoil_motor(pi)
