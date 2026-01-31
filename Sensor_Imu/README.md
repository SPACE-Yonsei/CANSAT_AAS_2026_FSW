### Introduction
IMU code for CANSAT AAS 2025

I2C Stretch 필요함

### Library
pip3 install adafruit-circuitpython-bno055
pip install adafruit-circuitpython-bno08x

### 하드웨어 설정 (10초 주기 에러 방지)

**1. I2C 클럭 스트레칭 문제 해결**
라즈베리 파이 제로 2W에서 BNO085 사용 시 I2C 클럭 스트레칭으로 인한 타임아웃 에러가 발생할 수 있습니다.

**2. INT 핀 연결 (선택사항, 권장)**
BNO085의 INT 핀을 GPIO에 연결하면 10초 주기 타임아웃 에러를 더 효과적으로 방지할 수 있습니다.
- **필수는 아니지만 권장됨**: INT 핀 없이도 동작하지만, 연결 시 더 안정적
- INT 핀을 GPIO 핀(예: GPIO 5)에 연결
- `imu.py`에서 `USE_INT_PIN = board.D5` (또는 연결된 GPIO 핀)로 설정
- INT 핀 미사용 시: `USE_INT_PIN = None` (기본값)

### Function
imu.py
- cal_imu : imu를 calibration 함. 
    - 1차 (Mag) : 8자 그리기. (수평이든 수직이든)
    - 2차 (Acc) : roll 방향으로 45도씩 돌리고 기다렸다 반복.
    - 3차 (Gyr) : 그냥 두면 됨.
    
    - 단!!! init_imu가 먼저 시작되어야 함!  
- init_imu : imu를 초기 설정함.
- read_sensor_data : yaw, roll, pitch를 계산하며, 이동평균을 토대로 출력한다. (motor의 급격한 운동 방지)

