import time
import board
import busio
import adafruit_vl53l1x

# I2C 통신을 초기화한다
i2c = busio.I2C(board.SCL, board.SDA)

# VL53L1X 센서 객체를 생성한다
# 패키지 명칭과 클래스 호출 방식이 변경되었다
vl53 = adafruit_vl53l1x.VL53L1X(i2c)

# 거리 측정 모드를 설정한다
# 1은 단거리 모드(최대 약 1.3m), 2는 장거리 모드(최대 약 4m)이다
vl53.distance_mode = 2
# 측정에 할당할 시간을 설정한다 (단위: ms)
vl53.timing_budget = 100

# 거리 측정을 시작한다
vl53.start_ranging()

print("측정을 시작한다. 종료하려면 Ctrl+C를 누른다.")

try:
    while True:
        # 센서에서 데이터가 준비되었는지 확인한다
        if vl53.data_ready:
            # 단일 거리 값을 mm 단위로 가져온다
            # L5CX와 달리 인덱스 접근이 필요 없다
            distance = vl53.distance
            
            if distance is not None:
                print(f"측정 거리: {distance} mm")
            
            # 다음 측정을 위해 데이터 준비 상태를 초기화한다
            # 이 명령어가 없으면 데이터가 갱신되지 않는다
            vl53.clear_interrupt()
            
        # 루프 사이에 짧은 대기 시간을 둔다
        time.sleep(0.05)

except KeyboardInterrupt:
    # 프로그램 종료 시 센서 작동을 중지한다
    vl53.stop_ranging()
    print(측정을 종료한다.)
