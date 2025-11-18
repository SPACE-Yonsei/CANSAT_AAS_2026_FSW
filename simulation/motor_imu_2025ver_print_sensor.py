#### sudo apt install -y python3-gpiozero
#### #!/usr/bin/env python3 지우면 안됩니다

#!/usr/bin/env python3

#from gpiozero import AngularServo
#import math, random, time

import time

# Target Degree based on IMU
TARGET_DEGREE = 0

PAYLOAD_MOTOR_PIN = 12

# Calibrate the pulse range
PAYLOAD_MOTOR_MIN_PULSE = 500
PAYLOAD_MOTOR_MAX_PULSE = 2500

def init_MG92B():
    import pigpio
    pi = pigpio.pi()
    pi.set_servo_pulsewidth(PAYLOAD_MOTOR_PIN, 0)

    return pi

def terminate_MG92B(pi):
    pi.stop()

def angle_to_pulse(angle) -> int:
    if angle < 0:
        angle = 0
    elif angle > 180:
        angle = 180
    
    return int(PAYLOAD_MOTOR_MIN_PULSE + ((angle/180)*(PAYLOAD_MOTOR_MAX_PULSE - PAYLOAD_MOTOR_MIN_PULSE)))

prev_yaw = 0

# Minimum degree motor activation when degree change
ROTATION_THRESHOLD_DEG = 3

def rotate_MG92B_ByYaw(pi, yaw:float):
    global prev_yaw

    yaw = (yaw+TARGET_DEGREE) % 360
    
    if abs(prev_yaw - yaw) > ROTATION_THRESHOLD_DEG:
        if 90 > yaw >= 0:   #1사분면    0~90 
            pi.set_servo_pulsewidth(PAYLOAD_MOTOR_PIN, angle_to_pulse(90-yaw))
        elif 180 > yaw >= 90:  #2사분면  
            pi.set_servo_pulsewidth(PAYLOAD_MOTOR_PIN, angle_to_pulse(3))
        elif 270 > yaw >= 180:  #3사분면  
            pi.set_servo_pulsewidth(PAYLOAD_MOTOR_PIN, angle_to_pulse(177))
        elif 360 >= yaw >= 270:   #4사분면
#            pi.set_servo_pulsewidth(PAYLOAD_MOTOR_PIN, angle_to_pulse(yaw - 270)) 
            pi.set_servo_pulsewidth(PAYLOAD_MOTOR_PIN, angle_to_pulse(450 - yaw))
        else:
            pass
    # 1. 목표 방향(TARGET_DEGREE)을 기준으로 현재 yaw 값을 보정합니다.
    #    예: TARGET_DEGREE가 0이면, yaw 값은 변하지 않습니다.
    corrected_yaw = (yaw + TARGET_DEGREE) % 360

    # 2. yaw 값의 변화가 최소 임계값(ROTATION_THRESHOLD_DEG)보다 클 때만 모터를 움직입니다.
    #    이렇게 하면 센서의 미세한 떨림에 모터가 반응하지 않습니다.
    #    360도와 0도 사이의 경계 문제를 해결하기 위해 각도 차이를 올바르게 계산합니다.
    diff = abs(corrected_yaw - prev_yaw)
    if min(diff, 360 - diff) > ROTATION_THRESHOLD_DEG:
        # 3. 서보모터의 목표 각도를 계산합니다.
        #    서보의 중앙(90도)을 기준으로, yaw가 증가하면(오른쪽 회전) 서보 각도는 감소(왼쪽 회전)시켜 보상합니다.
        #    -90 ~ +90 범위의 보상 각도를 계산하고, 서보의 중앙(90도)에 더해줍니다.
        #    (corrected_yaw + 180) % 360 - 180: yaw를 -180 ~ 180 범위로 변환합니다.
        compensation_angle = -((corrected_yaw + 180) % 360 - 180)
        servo_angle = 90 + compensation_angle

        # 4. 계산된 각도를 펄스 값으로 변환하여 모터를 제어합니다.
        pi.set_servo_pulsewidth(PAYLOAD_MOTOR_PIN, angle_to_pulse(servo_angle))

        # 5. 현재 yaw 값을 prev_yaw로 업데이트하여 다음 비교에 사용합니다.
        prev_yaw = yaw
    return

if __name__ == "__main__":
    from adafruit_bno055 import BNO055_I2C
    import board, busio

    i2c    = busio.I2C(board.SCL, board.SDA)
    sensor = BNO055_I2C(i2c)

    pi = init_MG92B()

    try:
        print("BNO055 + MG92B 보정 시작 (Ctrl+C 종료)")
        # 센서 워밍업
        time.sleep(0.5)
        while True:
            # --- 값 확인을 위한 print문 추가 ---
            print("="*20)
            print("sensor 객체 자체 출력:", sensor)
            print(f"온도: {sensor.temperature} C")
            print(f"자이로스코프 (x, y, z): {sensor.gyro}")
            print(f"가속도 (x, y, z): {sensor.acceleration}")
            print(f"중력 (x, y, z): {sensor.gravity}")
            print(f"오일러 각 (heading, roll, pitch): {sensor.euler}")
            # ------------------------------------

            heading = sensor.euler[0]  # (heading, roll, pitch)
            if heading is None:
                # 아직 센서가 준비 안 됐으면 재시도
                time.sleep(0.1)
                print("sensor not ready")
                continue

            rotate_MG92B_ByYaw(pi, heading)
            time.sleep(1) # 값을 천천히 보기 위해 1초로 변경

    except KeyboardInterrupt:
        pass

    finally:
        terminate_MG92B(pi)
