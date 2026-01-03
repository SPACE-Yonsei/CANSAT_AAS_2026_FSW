import time
import math
from datetime import datetime
import os

# Variables for moving window filter
angle_window = [[],[],[]] # (ROLL, PITCH, YAW)
window_size = 5

# BNO055 장착 방향 보정
# BNO055 Y축 = 캔위성 앞쪽, X축 = 캔위성 왼쪽, 바닥에 장착 (Z축 아래)
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
    import adafruit_bno055
    # Initializa I2C interface
    i2c = board.I2C()  # board.SCL과 board.SDA 사용
    sensor = adafruit_bno055.BNO055_I2C(i2c)

    # Calibration results go HERE
    
    sensor.offsets_magnetometer = magneto_offset
    sensor.offsets_gyroscope = gyro_offset
    sensor.offsets_accelerometer = accel_offset
    #print(type(accel_offset[0]))
    
    return i2c, sensor

def read_sensor_data(sensor):
    global angle_window

    q = sensor.quaternion

    # Pass this data when none is includued in quaternion
    if None in q:
        # In case that the angle window is empty, put 0
        if len(angle_window[0]) == 0: 
            angle_window[0].append(0)
        if len(angle_window[1]) == 0:
            angle_window[1].append(0)
        if len(angle_window[2]) == 0:
            angle_window[2].append(0)
    else:
        w, x, y, z = q

        w = round(w, 4)
        x = round(x, 4)
        y = round(y, 4)
        z = round(z, 4)
        # 쿼터니언으로부터 yaw (heading) 계산 (라디안 단위)
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
        yaw_deg = math.degrees(yaw)
        
        # BNO055 장착 방향 보정
        if IMU_MOUNTED_ON_BOTTOM:
            # Z축이 아래로 향하면 yaw 회전 방향 반전
            yaw_deg = -yaw_deg
        
        if IMU_FORWARD_AXIS == 'Y':
            # Y축이 앞쪽이면 90° 오프셋 적용 (X축 기준 → Y축 기준)
            yaw_deg = yaw_deg + 90

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
        
        # 음수 각도를 0~360도로 변환
        if yaw_deg < 0:
            yaw_deg += 360
        
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

    # Accelometer and Mag, gyro
    avg_yaw = sum(angle_window[0])/len(angle_window[0])
    avg_roll = sum(angle_window[1])/len(angle_window[1])
    avg_pitch = sum(angle_window[2])/len(angle_window[2])
    
    # Round all data to 2 digits

    avg_yaw = round(avg_yaw, 4)
    avg_roll = round(avg_roll, 4)
    avg_pitch = round(avg_pitch, 4)

    #sen_yaw, sen_roll, sen_pitch = sensor.euler 
    accX, accY, accZ = sensor.acceleration

    # Error Checking, if None is contained, set the value to 0
    if accX == None or accY == None or accZ == None:
        accX = 0
        accY = 0
        accZ = 0
    else:
        accX = round(accX, 4)
        accY = round(accY, 4)
        accZ = round(accZ, 4)

    magX, magY, magZ = sensor.magnetic
    if magX is None or magY is None or magZ is None:
        magX = 0
        magY = 0
        magZ = 0
    else:
        magX = round(magX, 4)
        magY = round(magY, 4)
        magZ = round(magZ, 4)

    gyrX, gyrY, gyrZ = sensor.gyro
    if gyrX is None or gyrY is None or gyrZ is None:
        gyrX = 0
        gyrY = 0
        gyrZ = 0
    else:
        gyrX = round(gyrX, 4)
        gyrY = round(gyrY, 4)
        gyrZ = round(gyrZ, 4)

    # Read Linear Acc and Gravity vector
    lin_accX, lin_accY, lin_accZ = sensor.linear_acceleration
    graX, graY, graZ = sensor.gravity
    
    # Calculate tilt angle from gravity vector
    # 기울기 각도 계산: 중력 벡터로부터 캔위성이 수직선에서 몇 도 기울어졌는지 계산
    tilt_angle = 0.0
    tilt_direction = 0.0  # 기울기 방향 (0-360도)
    
    if graX is not None and graY is not None and graZ is not None:
        # 중력 벡터의 크기
        gravity_magnitude = math.sqrt(graX**2 + graY**2 + graZ**2)
        
        if gravity_magnitude > 0:
            # 기울기 각도: 수직선(중력 방향)과 z축 사이의 각도
            # 0도 = 수직, 90도 = 수평
            # arccos(gz / |g|) 또는 arctan2(sqrt(gx^2 + gy^2), gz)
            tilt_angle = math.degrees(math.atan2(math.sqrt(graX**2 + graY**2), abs(graZ)))
            
            # 기울기 방향: xy 평면에서 중력 벡터의 투영 방향
            tilt_direction = math.degrees(math.atan2(graY, graX))
            if tilt_direction < 0:
                tilt_direction += 360
            
            tilt_angle = round(tilt_angle, 4)
            tilt_direction = round(tilt_direction, 4)
        else:
            tilt_angle = 0.0
            tilt_direction = 0.0
    else:
        # 중력 벡터를 읽을 수 없는 경우
        tilt_angle = 0.0
        tilt_direction = 0.0
    
    # Linear acceleration 처리
    if lin_accX is None or lin_accY is None or lin_accZ is None:
        lin_accX = 0
        lin_accY = 0
        lin_accZ = 0
    else:
        lin_accX = round(lin_accX, 4)
        lin_accY = round(lin_accY, 4)
        lin_accZ = round(lin_accZ, 4)
    
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
    try:
        while True:
            data = read_sensor_data(sensor)
            print(data)
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        imu_terminate(i2c)

