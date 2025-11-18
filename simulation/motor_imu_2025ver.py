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
            heading = sensor.euler[0]  # (heading, roll, pitch)
            if heading is None:
                # 아직 센서가 준비 안 됐으면 재시도
                time.sleep(0.1)
                print("sensor not ready")
                continue

            rotate_MG92B_ByYaw(pi, heading)
            time.sleep(0.1)

    except KeyboardInterrupt:
        pass

    finally:
        terminate_MG92B(pi)
