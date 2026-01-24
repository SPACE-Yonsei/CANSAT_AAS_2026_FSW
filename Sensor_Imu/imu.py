import time
import math
import os
import sys
from datetime import datetime
from contextlib import redirect_stdout, redirect_stderr
from io import StringIO

# 이동평균 필터 윈도우
angle_window = [[], [], []]  # (YAW, ROLL, PITCH)
WINDOW_SIZE = 5

# BNO085 장착 방향 보정
IMU_MOUNTED_ON_BOTTOM = True  # Z축이 아래로 향함
IMU_FORWARD_AXIS = 'Y'        # 캔위성 앞쪽 방향

# 로그 설정
log_dir = './sensorlogs'
if not os.path.exists(log_dir):
    os.makedirs(log_dir)

imulogfile = open(os.path.join(log_dir, 'imu.txt'), 'a')

def log_imu(text):
    t = datetime.now().isoformat(sep=' ', timespec='milliseconds')
    imulogfile.write(f'{t},{text}\n')
    imulogfile.flush()


def init_imu():
    import board
    import adafruit_bno08x
    from adafruit_bno08x.i2c import BNO08X_I2C
    
    i2c = board.I2C()
    
    # 디버그 출력 억제
    with redirect_stdout(StringIO()), redirect_stderr(StringIO()):
        sensor = BNO08X_I2C(i2c, debug=False)
    
    # 디버그 속성 비활성화
    if hasattr(sensor, '_debug'):
        sensor._debug = False
    
    # 필수 기능 활성화 (디버그 출력 억제)
    with redirect_stdout(StringIO()), redirect_stderr(StringIO()):
        sensor.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
        sensor.enable_feature(adafruit_bno08x.BNO_REPORT_ACCELEROMETER)
        sensor.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
        sensor.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)
        #sensor.enable_feature(adafruit_bno08x.BNO_REPORT_LINEAR_ACCELERATION)
        #sensor.enable_feature(adafruit_bno08x.BNO_REPORT_GRAVITY)
    
    time.sleep(0.5)
    print("BNO08x initialized")
    return i2c, sensor


def read_sensor_data(sensor):
    global angle_window
    
    # 쿼터니언 읽기 (디버그 출력 억제)
    try:
        with redirect_stdout(StringIO()), redirect_stderr(StringIO()):
            quat = sensor.quaternion
        if quat is None:
            return False
    except Exception:
        return False
    
    x, y, z, w = quat
    
    # 쿼터니언 → 오일러각 변환
    yaw = math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2)))
    roll = math.degrees(math.atan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2)))
    
    pitch_val = max(-1, min(1, 2*(w*y - z*x)))
    pitch = math.degrees(math.asin(pitch_val))
    
    # 장착 방향 보정
    if IMU_MOUNTED_ON_BOTTOM:
        yaw = -yaw
    if IMU_FORWARD_AXIS == 'Y':
        yaw += 90
    
    # YAW_OFFSET 적용
    try:
        from lib import config
        yaw += config.YAW_OFFSET
    except:
        pass
    
    # 0~360도 범위로 정규화
    yaw = yaw % 360
    roll = roll % 360
    
    # 이동평균 필터
    angle_window[0].append(yaw)
    angle_window[1].append(roll)
    angle_window[2].append(pitch)
    
    for i in range(3):
        if len(angle_window[i]) > WINDOW_SIZE:
            angle_window[i].pop(0)
    
    avg_yaw = round(sum(angle_window[0]) / len(angle_window[0]), 4)
    avg_roll = round(sum(angle_window[1]) / len(angle_window[1]), 4)
    avg_pitch = round(sum(angle_window[2]) / len(angle_window[2]), 4)
    
    # 가속도, 자이로, 자기장 읽기 (디버그 출력 억제)
    try:
        with redirect_stdout(StringIO()), redirect_stderr(StringIO()):
            accX, accY, accZ = sensor.linear_acceleration
        accX, accY, accZ = round(accX, 4), round(accY, 4), round(accZ, 4)
    except:
        accX = accY = accZ = 0
    
    try:
        with redirect_stdout(StringIO()), redirect_stderr(StringIO()):
            magX, magY, magZ = sensor.magnetic
        magX, magY, magZ = round(magX, 4), round(magY, 4), round(magZ, 4)
    except:
        magX = magY = magZ = 0
    
    try:
        with redirect_stdout(StringIO()), redirect_stderr(StringIO()):
            gyrX, gyrY, gyrZ = sensor.gyro
        gyrX, gyrY, gyrZ = round(gyrX, 4), round(gyrY, 4), round(gyrZ, 4)
    except:
        gyrX = gyrY = gyrZ = 0
    
    # 중력 벡터 → 기울기 계산 (디버그 출력 억제)
    try:
        with redirect_stdout(StringIO()), redirect_stderr(StringIO()):
            graX, graY, graZ = sensor.gravity
        graX, graY, graZ = round(graX, 4), round(graY, 4), round(graZ, 4)
        tilt_angle = round(math.degrees(math.atan2(math.sqrt(graX**2 + graY**2), abs(graZ))), 4)
        tilt_direction = round(math.degrees(math.atan2(graY, graX)) % 360, 4)
    except:
        graX = graY = graZ = 0
        tilt_angle = tilt_direction = 0
    
    log_imu(f"{avg_roll:.4f},{avg_pitch:.4f},{avg_yaw:.4f},{accX},{accY},{accZ},{magX},{magY},{magZ},{gyrX},{gyrY},{gyrZ},{tilt_angle},{tilt_direction},{graX},{graY},{graZ}")
    
    return (avg_roll, avg_pitch, avg_yaw, accX, accY, accZ, magX, magY, magZ, gyrX, gyrY, gyrZ, tilt_angle, tilt_direction, graX, graY, graZ)


def imu_terminate(i2c):
    if i2c is not None:
        i2c.deinit()


def reset_angle_window():
    global angle_window
    angle_window = [[], [], []]


def reinit_imu(i2c, sensor):
    """에러 발생 시 IMU 재초기화"""
    # I2C 버스 완전히 해제
    try:
        if i2c is not None:
            i2c.deinit()
    except:
        pass
    
    # 센서 객체 정리
    try:
        if sensor is not None:
            del sensor
    except:
        pass
    
    # 충분한 대기 시간 (I2C 버스 안정화)
    time.sleep(2)
    
    # 윈도우 리셋
    reset_angle_window()
    
    # 재시도 로직 (최대 3회)
    max_retries = 3
    for attempt in range(max_retries):
        try:
            return init_imu()
        except Exception as e:
            if attempt < max_retries - 1:
                print(f"Reinit attempt {attempt + 1}/{max_retries} failed: {e}")
                time.sleep(1)
                continue
            else:
                raise RuntimeError(f"Failed to reinitialize IMU after {max_retries} attempts: {e}")


if __name__ == "__main__":
    i2c, sensor = init_imu()
    
    try:
        while True:
            data = read_sensor_data(sensor)
            if data == False:
                # 에러 발생 시 재초기화
                #print("Read error - Reinitializing...")
                try:
                    i2c, sensor = reinit_imu(i2c, sensor)
                    #print("Reinitialization successful")
                    time.sleep(0.5)  # 재초기화 후 안정화 대기
                except Exception as e:
                    #print(f"Reinitialization failed: {e}")
                    #print("Retrying in 2 seconds...")
                    time.sleep(2)
                    try:
                        i2c, sensor = reinit_imu(i2c, sensor)
                        #print("Reinitialization successful after retry")
                    except Exception as e2:
                        #print(f"Reinitialization failed again: {e2}")
                        break  # 재초기화 실패 시 루프 종료
                continue
            
            error_count = 0
            print(data)
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        imu_terminate(i2c)
