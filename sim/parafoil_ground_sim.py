#!/usr/bin/env python3

import math
import sys
import argparse
import time

try:
    import pigpio
except Exception:
    pigpio = None

try:
    import board
    import busio
    from adafruit_bno055 import BNO055_I2C
except Exception:
    BNO055_I2C = None
    busio = None
    board = None

# --- [중요] 사용자 하드웨어에 맞춰진 펄스 설정 ---
PARAFOIL_MIN_PULSE = 1250  # 0.5ms (최소/풀림 위치가 0이 아니라면 이 값 사용)
PARAFOIL_MAX_PULSE = 1750  # 2.5ms (최대/당김 위치)
TURN_THRESHOLD = 15        # 이 각도 이상 차이나야 모터가 움직임

def haversine_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """두 GPS 좌표 사이의 방위각 계산"""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    y = math.sin(dlambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    brng = math.degrees(math.atan2(y, x))
    return (brng + 360.0) % 360.0

def normalize_angle_deg(angle: float) -> float:
    """각도를 -180 ~ 180 범위로 변환"""
    a = (angle + 180.0) % 360.0 - 180.0
    return 180.0 if a == -180.0 else a

def init_bno055() -> object:
    if BNO055_I2C is None or busio is None or board is None:
        return None
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        sensor = BNO055_I2C(i2c)
        return sensor
    except Exception:
        return None

def read_bno055_heading_deg(sensor) -> float | None:
    try:
        e = sensor.euler
        if e is None or e[0] is None:
            return None
        return float(e[0]) % 360.0
    except Exception:
        return None

def main() -> int:
    parser = argparse.ArgumentParser(description="Parafoil Steering with Validated Pulse Range")
    parser.add_argument("--pigpio", action="store_true", default=True, help="Use pigpio (default: True)")
    parser.add_argument("--left-gpio", type=int, default=12, help="GPIO for Left Motor")
    parser.add_argument("--right-gpio", type=int, default=13, help="GPIO for Right Motor")
    parser.add_argument("--bno055", action="store_true", help="Use BNO055 Sensor")
    args = parser.parse_args()

    # --- pigpio 초기화 ---
    if pigpio is None:
        print("Error: pigpio module not found.")
        return 1

    pi = None
    try:
        pi = pigpio.pi()
        if not pi.connected:
            print("Error: Failed to connect to pigpio daemon. (Try 'sudo pigpiod')")
            return 1
        pi.set_mode(args.left_gpio, pigpio.OUTPUT)
        pi.set_mode(args.right_gpio, pigpio.OUTPUT)
        # 초기화 시 모터 0으로 설정 (안전)
        pi.set_servo_pulsewidth(args.left_gpio, 0)
        pi.set_servo_pulsewidth(args.right_gpio, 0)
    except Exception as e:
        print(f"pigpio init error: {e}")
        return 1

    # --- 센서 초기화 ---
    sensor = None
    if args.bno055:
        sensor = init_bno055()
        if sensor is None:
            print("Warning: Failed to init BNO055. Using manual input.")
    
    print("=== Parafoil Auto-Steering Started ===")
    print(f"Settings: Left Pin {args.left_gpio}, Right Pin {args.right_gpio}")
    print(f"Pulse Range: {PARAFOIL_MIN_PULSE} ~ {PARAFOIL_MAX_PULSE}")

    # 테스트용 좌표 (현 위치 -> 목표 위치)
    # 실제 사용 시에는 GPS에서 받아온 값을 넣어야 합니다.
    # 여기서는 테스트를 위해 북쪽(0도)을 바라보고 있다고 가정하고, 타겟을 동쪽(90도)으로 설정해봅니다.
    t_lat = 37.0
    t_lon = 127.1  # 목표지점 (동쪽)
    c_lat = 37.0
    c_lon = 127.0  # 현재지점

    try:
        while True:
            # 1. 현재 헤딩값 읽기
            if sensor is not None:
                imu_heading = read_bno055_heading_deg(sensor)
                if imu_heading is None:
                    time.sleep(0.01)
                    continue
            else:
                # 센서가 없으면 테스트를 위해 사용자 입력 받음 (엔터 치면 갱신)
                try:
                    raw = input("Current Heading (deg) or 'q': ").strip()
                    if raw == 'q': break
                    if raw == '': continue
                    imu_heading = float(raw)
                except:
                    continue

            # 2. 목표 방위각 및 에러 계산
            bearing = haversine_bearing(c_lat, c_lon, t_lat, t_lon)
            err = normalize_angle_deg(bearing - imu_heading)
            
            # 3. 모터 제어 로직 (보내주신 코드의 로직 적용)
            # err가 양수(+)이면 목표가 오른쪽 -> 오른쪽으로 돌아야 함 -> 오른쪽 줄 당김
            # err가 음수(-)이면 목표가 왼쪽 -> 왼쪽으로 돌아야 함 -> 왼쪽 줄 당김
            
            left_pulse = 0
            right_pulse = 0
            action = "Straight"

            if err < -TURN_THRESHOLD: 
                # 왼쪽으로 턴 (왼쪽 줄 당김, 오른쪽 풀기)
                action = "LEFT TURN"
                left_pulse = PARAFOIL_MAX_PULSE
                right_pulse = 0 # 또는 PARAFOIL_MIN_PULSE
                
            elif err > TURN_THRESHOLD:
                # 오른쪽으로 턴 (오른쪽 줄 당김, 왼쪽 풀기)
                action = "RIGHT TURN"
                left_pulse = 0 # 또는 PARAFOIL_MIN_PULSE
                right_pulse = PARAFOIL_MAX_PULSE
                
            else:
                # 직진 (둘 다 풀기)
                action = "Straight"
                left_pulse = 0 # 또는 PARAFOIL_MIN_PULSE
                right_pulse = 0 # 또는 PARAFOIL_MIN_PULSE

            # 4. 실제 모터 출력
            pi.set_servo_pulsewidth(args.left_gpio, left_pulse)
            pi.set_servo_pulsewidth(args.right_gpio, right_pulse)

            # 상태 출력
            print(f"Head: {imu_heading:5.1f}° | Target: {bearing:5.1f}° | Err: {err:5.1f}° | Cmd: {action:10s} | L: {left_pulse} / R: {right_pulse}")

            # 센서 모드일 때는 빠르게 루프 (50ms = 20Hz)
            if sensor is not None:
                time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        if pi:
            pi.set_servo_pulsewidth(args.left_gpio, 0)
            pi.set_servo_pulsewidth(args.right_gpio, 0)
            pi.stop()

    return 0

if __name__ == "__main__":
    sys.exit(main())