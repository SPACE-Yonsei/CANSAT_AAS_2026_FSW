import time
import math
from datetime import datetime
import os
import sys
from contextlib import redirect_stdout, redirect_stderr
from io import StringIO

# Variables for moving window filter
angle_window = [[],[],[]] # (ROLL, PITCH, YAW)
window_size = 5

# BNO085 장착 방향 보정
# BNO085 Y축 = 캔위성 앞쪽, X축 = 캔위성 왼쪽, 바닥에 장착 (Z축 아래)
# 표준 yaw는 X축 기준이므로 Y축 기준으로 변환 필요 (+90°)
# Z축이 뒤집혀서 yaw 회전 방향 반전 필요
IMU_MOUNTED_ON_BOTTOM = True  # True = Z축이 아래로 향함
IMU_FORWARD_AXIS = 'Y'        # 'X' 또는 'Y' (캔위성 앞쪽 방향)

log_dir = './sensorlogs'
if not os.path.exists(log_dir): 
    os.makedirs(log_dir)

## Create sensor log file
imulogfile = open(os.path.join(log_dir, 'imu.txt'), 'a')

try:
    offsetfile = open(os.path.join('./imu/offset.txt'),mode='r')
    magneto_offset = tuple(map(int,offsetfile.readline().strip().split(sep=',')))
    gyro_offset = tuple(map(int,offsetfile.readline().strip().split(sep=',')))
    accel_offset = tuple(map(int,offsetfile.readline().strip().split(sep=',')))
    offsetfile.close()
except: 
    magneto_offset = (0,0,0)
    gyro_offset = (0,0,0)
    accel_offset = (0,0,0)

def log_imu(text):
    t = datetime.now().isoformat(sep=' ', timespec='milliseconds')
    string_to_write = f'{t},{text}\n'
    imulogfile.write(string_to_write)
    imulogfile.flush()

def init_imu():
    import board
    import adafruit_bno08x
    from adafruit_bno08x.i2c import BNO08X_I2C
    import time
    
    MAX_RETRIES = 3
    # BNO08x possible addresses: 0x4A (default), 0x4B (alternate)
    POSSIBLE_ADDRESSES = [0x4a, 0x4b]
    
    for attempt in range(MAX_RETRIES):
        i2c = None
        try:
            # Initialize I2C interface
            i2c = board.I2C()  # board.SCL과 board.SDA 사용
            
            # Try each possible address
            sensor = None
            used_address = None
            for addr in POSSIBLE_ADDRESSES:
                try:
                    # 디버그 출력 억제를 위해 stdout/stderr 리다이렉트
                    with redirect_stdout(StringIO()), redirect_stderr(StringIO()):
                        sensor = BNO08X_I2C(i2c, address=addr, debug=False)
                    # 추가로 _debug 속성도 False로 설정
                    if hasattr(sensor, '_debug'):
                        sensor._debug = False
                    used_address = addr
                    print(f"BNO08x found at address 0x{addr:02x}")
                    break
                except ValueError as e:
                    # "No I2C device at address" error
                    continue
            
            if sensor is None:
                raise RuntimeError(f"No BNO08x found at addresses {[hex(a) for a in POSSIBLE_ADDRESSES]}")
            
            # 센서 소프트 리셋
            sensor.initialize()
            time.sleep(1.5)  # 센서 초기화 대기 (더 길게)
            
            # Feature 활성화 (개별 try-except로 오류 처리)
            # Only enable rotation vector first - most critical
            features = [
                (adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR, "Rotation Vector"),
            ]
            
            # Try to enable additional features if possible
            optional_features = [
                (adafruit_bno08x.BNO_REPORT_ACCELEROMETER, "Accelerometer"),
                (adafruit_bno08x.BNO_REPORT_GYROSCOPE, "Gyroscope"),
                (adafruit_bno08x.BNO_REPORT_MAGNETOMETER, "Magnetometer"),
                (adafruit_bno08x.BNO_REPORT_LINEAR_ACCELERATION, "Linear Acceleration"),
                (adafruit_bno08x.BNO_REPORT_GRAVITY, "Gravity"),
            ]
            
            enabled_count = 0
            # Enable rotation vector first (required)
            for feature, name in features:
                try:
                    sensor.enable_feature(feature)
                    enabled_count += 1
                    time.sleep(0.5)  # 더 긴 대기 시간
                except Exception as e:
                    print(f"Warning: Failed to enable {name}: {e}")
            
            if enabled_count == 0:
                raise RuntimeError("Failed to enable Rotation Vector - cannot proceed")
            
            # Enable optional features
            for feature, name in optional_features:
                try:
                    sensor.enable_feature(feature)
                    enabled_count += 1
                    time.sleep(0.3)
                except Exception as e:
                    print(f"Warning: Failed to enable {name}: {e}")
            
            print(f"BNO08x initialized at 0x{used_address:02x}: {enabled_count}/{len(features) + len(optional_features)} features enabled")
            time.sleep(1)  # 최종 안정화 대기
            
            return i2c, sensor
            
        except Exception as e:
            print(f"IMU init attempt {attempt + 1}/{MAX_RETRIES} failed: {e}")
            if i2c is not None:
                try:
                    i2c.deinit()  # I2C 해제 후 재시도
                except:
                    pass
            if attempt < MAX_RETRIES - 1:
                time.sleep(2)  # 재시도 전 대기
            else:
                raise RuntimeError(f"Failed to initialize BNO08x after {MAX_RETRIES} attempts: {e}")

def read_sensor_data(sensor):
    global angle_window

    # BNO085는 quaternion 직접 접근이 아닌 update를 통해 데이터 읽음
    try:
        # 디버그 출력 억제
        with redirect_stdout(StringIO()), redirect_stderr(StringIO()):
            quat_info = sensor.quaternion
    except KeyError as e:
        # KeyError occurs when BNO08x receives unknown report type (e.g., 0x77 from I2C bus noise)
        # Return False to indicate data read failure, let caller handle retry
        print("IMU KeyError (possible I2C bus noise)")
        return False
    except (OSError, RuntimeError) as e:
        # I2C communication error
        print(f"IMU communication error: {e}")
        return False
    
    if quat_info is None:
        # 쿼터니언 데이터 없음
        if len(angle_window[0]) == 0: 
            angle_window[0].append(0)
        if len(angle_window[1]) == 0:
            angle_window[1].append(0)
        if len(angle_window[2]) == 0:
            angle_window[2].append(0)
    else:
        # BNO085 쿼터니언은 (x, y, z, w) 형식 (BNO055는 (w, x, y, z))
        x, y, z, w = quat_info

        w = round(w, 4)
        x = round(x, 4)
        y = round(y, 4)
        z = round(z, 4)
        
        # 쿼터니언으로부터 yaw (heading) 계산 (라디안 단위)
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
        yaw_deg = math.degrees(yaw)
        
        # BNO085 장착 방향 보정
        if IMU_MOUNTED_ON_BOTTOM:
            # Z축이 아래로 향하면 yaw 회전 방향 반전
            yaw_deg = -yaw_deg
        
        if IMU_FORWARD_AXIS == 'Y':
            # Y축이 앞쪽이면 90° 오프셋 적용 (X축 기준 → Y축 기준)
            yaw_deg = yaw_deg + 90
        
        # config.txt에서 설정한 YAW_OFFSET 적용 (현장에서 0점 조절용)
        try:
            from lib import config
            yaw_deg = yaw_deg + config.YAW_OFFSET
        except (ImportError, ModuleNotFoundError):
            # config를 로드할 수 없으면 YAW_OFFSET 0으로 사용
            pass

        # 쿼터니언으로부터 pitch 계산 (라디안 단위)
        try: # arcsin 함수의 정의역 문제
            pitch_cal = 2*(w*y - z*x)
            if pitch_cal < -1:
                pitch_cal = -1.00
            if pitch_cal > 1:
                pitch_cal = 1.00
            pitch_cal = round(pitch_cal, 4)
            
            pitch = math.asin(pitch_cal)
            # 라디안을 도(degree)로 변환
            pitch_deg = math.degrees(pitch)
            
        except ValueError:
            # 정의역을 넘어버렸을 때 -> 직전 값을 가져와서 대체함.
            pitch_deg = angle_window[2][-1]     
            
        # 쿼터니언으로부터 roll 계산 (라디안 단위)
        roll = math.atan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))
        
        # 라디안을 도(degree)로 변환
        roll_deg = math.degrees(roll)
        
        # 음수 각도를 0~360도로 변환 (YAW_OFFSET 적용 후)
        if yaw_deg < 0:
            yaw_deg += 360
        elif yaw_deg >= 360:
            yaw_deg -= 360
        
        if roll_deg < 0:
            roll_deg += 360
        ''' 
        if pitch_deg < 0:
            pitch_deg += 360
        '''
        angle_window[0].append(yaw_deg)
        angle_window[1].append(roll_deg)
        angle_window[2].append(pitch_deg)

        # YAW 이동평균필터 리스트 최신화
        if len(angle_window[0]) > window_size:
            angle_window[0].pop(0)

        # ROLL 이동평균필터 리스트 최신화
        if len(angle_window[1]) > window_size:
            angle_window[1].pop(0)

        # PITCH 이동평균필터 리스트 최신화
        if len(angle_window[2]) > window_size:
            angle_window[2].pop(0)

    # Accelerometer, Magnetometer, Gyroscope 데이터 읽기 (BNO085)
    avg_yaw = sum(angle_window[0])/len(angle_window[0])
    avg_roll = sum(angle_window[1])/len(angle_window[1])
    avg_pitch = sum(angle_window[2])/len(angle_window[2])
    
    avg_yaw = round(avg_yaw, 4)
    avg_roll = round(avg_roll, 4)
    avg_pitch = round(avg_pitch, 4)

    # BNO085는 linear_acceleration, gravity 등을 직접 제공
    # KeyError can occur on any sensor access due to I2C bus noise
    # 디버그 출력 억제를 위해 모든 센서 읽기를 리다이렉트 안에서 수행
    with redirect_stdout(StringIO()), redirect_stderr(StringIO()):
        try:
            accX, accY, accZ = sensor.linear_acceleration
        except (KeyError, OSError, RuntimeError, TypeError):
            accX = accY = accZ = None
        
        try:
            magX, magY, magZ = sensor.magnetic
        except (KeyError, OSError, RuntimeError, TypeError):
            magX = magY = magZ = None
        
        try:
            gyrX, gyrY, gyrZ = sensor.gyro
        except (KeyError, OSError, RuntimeError, TypeError):
            gyrX = gyrY = gyrZ = None

    # Error Checking, if None is contained, set the value to 0
    if accX is None or accY is None or accZ is None:
        accX = 0
        accY = 0
        accZ = 0
    else:
        accX = round(accX, 4)
        accY = round(accY, 4)
        accZ = round(accZ, 4)

    if magX is None or magY is None or magZ is None:
        magX = 0
        magY = 0
        magZ = 0
    else:
        magX = round(magX, 4)
        magY = round(magY, 4)
        magZ = round(magZ, 4)

    if gyrX is None or gyrY is None or gyrZ is None:
        gyrX = 0
        gyrY = 0
        gyrZ = 0
    else:
        gyrX = round(gyrX, 4)
        gyrY = round(gyrY, 4)
        gyrZ = round(gyrZ, 4)

    # Read Gravity vector
    with redirect_stdout(StringIO()), redirect_stderr(StringIO()):
        try:
            graX, graY, graZ = sensor.gravity
        except (KeyError, OSError, RuntimeError, TypeError):
            graX = graY = graZ = None
    
    # Calculate tilt angle from gravity vector
    tilt_angle = 0.0
    tilt_direction = 0.0
    
    if graX is not None and graY is not None and graZ is not None:
        # 중력 벡터의 크기
        gravity_magnitude = math.sqrt(graX**2 + graY**2 + graZ**2)
        
        if gravity_magnitude > 0:
            tilt_angle = math.degrees(math.atan2(math.sqrt(graX**2 + graY**2), abs(graZ)))
            tilt_direction = math.degrees(math.atan2(graY, graX))
            if tilt_direction < 0:
                tilt_direction += 360
            
            tilt_angle = round(tilt_angle, 4)
            tilt_direction = round(tilt_direction, 4)
        else:
            tilt_angle = 0.0
            tilt_direction = 0.0
    else:
        tilt_angle = 0.0
        tilt_direction = 0.0
        graX = graY = graZ = 0
    
    # Gravity vector 처리
    if graX is None or graY is None or graZ is None:
        graX = 0
        graY = 0
        graZ = 0
    else:
        graX = round(graX, 4)
        graY = round(graY, 4)
        graZ = round(graZ, 4)
    
    log_imu(f"{avg_roll:.4f}, {avg_pitch:.4f}, {avg_yaw:.4f}, {accX:.2f}, {accY:.2f}, {accZ:.2f}, {magX:.2f}, {magY:.2f}, {magZ:.2f}, {gyrX:.2f}, {gyrY:.2f}, {gyrZ:.2f}, {tilt_angle:.4f}, {tilt_direction:.4f}, {graX:.2f}, {graY:.2f}, {graZ:.2f}")
    return (avg_roll, avg_pitch, avg_yaw, accX, accY, accZ, magX, magY, magZ, gyrX, gyrY, gyrZ, tilt_angle, tilt_direction, graX, graY, graZ)

def imu_terminate(i2c):
    i2c.deinit()
    return

if __name__ == "__main__":
    i2c, sensor = init_imu()
    #print(f'Offset : {sensor.offsets_magnetometer}')
    error_count = 0
    MAX_CONSECUTIVE_ERRORS = 10
    
    try:
        while True:
            data = read_sensor_data(sensor)
            if data == False:
                error_count += 1
                print(f"Read error ({error_count}/{MAX_CONSECUTIVE_ERRORS})")
                if error_count >= MAX_CONSECUTIVE_ERRORS:
                    try:
                        imu_terminate(i2c)
                    except:
                        pass
                    time.sleep(2)
                    i2c, sensor = init_imu()
                    error_count = 0
                time.sleep(0.1)
                continue
            
            # Reset error count on successful read
            error_count = 0
            print(data)
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        imu_terminate(i2c)

