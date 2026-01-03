import time
import board
import busio
import adafruit_vl53l1x

# I2C 설정
i2c = busio.I2C(board.SCL, board.SDA)

# 센서 초기화 (VL53L5CX는 초기화에 시간이 몇 초 걸릴 수 있습니다)
print("센서를 초기화 중입니다...")
vl53 = adafruit_vl53l1x.VL53L1X(i2c)

# 기본 설정 (해상도 4x4, 15Hz 업데이트)
vl53.resolution = 16  # 4x4 영역
vl53.start_ranging()

print("측정을 시작합니다.")

try:
    while True:
        if vl53.data_ready:
            # 거리 데이터 가져오기 (단위: mm)
            distance_data = vl53.distance
            
            # 4x4 영역 중 가장 중앙에 가까운 데이터(예: 7번 인덱스) 출력
            center_distance = distance_data[7]
            
            print(f"중앙 거리: {center_distance} mm")
            
        time.sleep(0.1)

except KeyboardInterrupt:
    vl53.stop_ranging()
    print("측정 종료")
