import time
import board
import busio
import adafruit_vl53l1x

# I2C 통신 초기화
i2c = busio.I2C(board.SCL, board.SDA)

def initialize_sensor():
    try:
        sensor = adafruit_vl53l1x.VL53L1X(i2c)
        sensor.distance_mode = 2  # 장거리 모드
        sensor.timing_budget = 100
        sensor.start_ranging()
        return sensor
    except Exception as e:
        print(f"초기화 실패: {e}")
        return None

vl53 = initialize_sensor()

if vl53:
    try:
        while True:
            try:
                if vl53.data_ready:
                    distance = vl53.distance
                    if distance is not None:
                        print(f"측정 거리: {distance} mm")
                    vl53.clear_interrupt()
                
                # 통신 부하를 줄이기 위해 루프 대기 시간 유지
                time.sleep(0.05)
                
            except OSError:
                print("통신 오류 발생. 잠시 후 재시도한다.")
                time.sleep(1)
                # 통신 오류 발생 시 센서 재연결 시도 로직 등을 추가할 수 있다
                
    except KeyboardInterrupt:
        vl53.stop_ranging()
        print("측정을 종료한다.")
else:
    print("센서를 연결할 수 없어 프로그램을 종료한다.")
