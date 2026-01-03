import time
import board
import busio

# 라이브러리 임포트 (설치 여부 확인)
try:
    from adafruit_vl53l5cx import VL53L5CX
    from adafruit_bno08x.i2c import BNO08X_I2C
    from adafruit_bno08x import BNO_REPORT_ACCELEROMETER
    from adafruit_ina238 import INA238
except ImportError as e:
    print(f"라이브러리 로드 실패: {e}\n'pip install'을 먼저 진행하세요.")

# I2C 버스 설정
i2c = busio.I2C(board.SCL, board.SDA)

def check_sensors():
    devices = {
        'VL53L5CX (Distance)': None,
        'BNO085 (IMU)': None,
        'INA238 (Power)': None
    }

    # 1. VL53L5CX 초기화 (0x29)
    try:
        vl53 = VL53L5CX(i2c)
        vl53.resolution = 16  # 4x4 모드
        vl53.start_ranging()
        devices['VL53L5CX (Distance)'] = vl53
        print("[OK] VL53L5CX 연결 성공")
    except Exception as e:
        print(f"[FAIL] VL53L5CX: {e}")

    # 2. BNO085 초기화 (0x4A)
    try:
        bno = BNO08X_I2C(i2c)
        bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        devices['BNO085 (IMU)'] = bno
        print("[OK] BNO085 연결 성공")
    except Exception as e:
        print(f"[FAIL] BNO085: {e} (PS0/PS1 핀 확인 필요)")

    # 3. INA238 초기화 (0x40)
    try:
        ina = INA238(i2c, address=0x40)
        ina.shunt_resistor = 0.1  # 션트 저항 값 (모듈에 맞춰 수정)
        devices['INA238 (Power)'] = ina
        print("[OK] INA238 연결 성공")
    except Exception as e:
        print(f"[FAIL] INA238: {e} (VCC/VS 핀 확인 필요)")
    
    return devices

sensors = check_sensors()

print("\n--- 데이터 측정 시작 ---")
try:
    while True:
        output = []

        # 거리 데이터 (VL53L5CX)
        vl = sensors['VL53L5CX (Distance)']
        if vl and vl.data_ready:
            output.append(f"Dist: {vl.distance[7]}mm") # 중앙 구역

        # 가속도 데이터 (BNO085)
        bn = sensors['BNO085 (IMU)']
        if bn:
            accel = bn.acceleration
            output.append(f"Acc: {accel[0]:.2f}, {accel[1]:.2f}")

        # 전력 데이터 (INA238)
        in_sen = sensors['INA238 (Power)']
        if in_sen:
            output.append(f"Pwr: {in_sen.bus_voltage:.2f}V, {in_sen.current:.3f}A")

        if output:
            print(" | ".join(output))
        else:
            print("인식된 센서가 없습니다. 배선을 확인하세요.", end="\r")

        time.sleep(0.5)

except KeyboardInterrupt:
    print("\n중단되었습니다.")
