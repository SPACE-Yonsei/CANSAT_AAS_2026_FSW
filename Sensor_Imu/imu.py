import time
import math
import os
import sys
from datetime import datetime
from contextlib import redirect_stdout, redirect_stderr
try:
    import fcntl
except Exception:
    fcntl = None
from io import StringIO

# 이동평균 필터 윈도우
angle_window = [[], [], []]  # (YAW, ROLL, PITCH)
WINDOW_SIZE = 5
READ_FAIL_REINIT_THRESHOLD = 5
I2C_FREQUENCY = int(os.getenv("IMU_I2C_FREQUENCY", "400000"))
I2C_LOCK_PATH = os.getenv("I2C_LOCK_PATH", "/tmp/i2c-1.lock")
I2C_LOCK_TIMEOUT_SEC = float(os.getenv("I2C_LOCK_TIMEOUT_SEC", "2.0"))

LAST_VALID_SENSORS = {
    "acc": (0.0, 0.0, 0.0),
    "mag": (0.0, 0.0, 0.0),
    "gyr": (0.0, 0.0, 0.0),
}
MAG_FILTER_ALPHA = float(os.getenv("IMU_MAG_FILTER_ALPHA", "0.2"))
MAG_FIELD_MIN = float(os.getenv("IMU_MAG_FIELD_MIN", "5.0"))
MAG_FIELD_MAX = float(os.getenv("IMU_MAG_FIELD_MAX", "150.0"))
MAG_NORM_SPIKE_RATIO = float(os.getenv("IMU_MAG_NORM_SPIKE_RATIO", "3.0"))
YAW_CORRECTION_GAIN = float(os.getenv("IMU_YAW_CORRECTION_GAIN", "0.02"))
MAG_FILTER_STATE = {"x": 0.0, "y": 0.0, "z": 0.0, "init": False, "norm": None}
REPORT_INTERVAL_US = int(os.getenv("IMU_REPORT_INTERVAL_US", "100000"))

# Hampel filter configuration (spike rejection)
HAMPEL_WINDOW_SIZE = int(os.getenv("IMU_HAMPEL_WINDOW_SIZE", "7"))
HAMPEL_THRESHOLD = float(os.getenv("IMU_HAMPEL_THRESHOLD", "3.0"))
HAMPEL_MIN_MAD = float(os.getenv("IMU_HAMPEL_MIN_MAD", "2.0"))


class I2CLock:
    def __init__(self, path=I2C_LOCK_PATH):
        self.path = path
        self.fd = None

    def __enter__(self):
        if fcntl is None:
            return self
        self.fd = open(self.path, "w")
        start = time.time()
        while True:
            try:
                fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except BlockingIOError:
                if time.time() - start > I2C_LOCK_TIMEOUT_SEC:
                    raise TimeoutError("I2C lock timeout")
                time.sleep(0.01)
        return self

    def __exit__(self, exc_type, exc, tb):
        if fcntl is None:
            return False
        try:
            fcntl.flock(self.fd, fcntl.LOCK_UN)
            self.fd.close()
        except Exception:
            pass
        return False

# BNO055 장착 방향 보정
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

def _wrap_angle_deg(angle):
    return angle % 360

def _angle_diff_deg(target, current):
    diff = (target - current + 180) % 360 - 180
    return diff

def _mag_norm_is_valid(mag_norm):
    if not (MAG_FIELD_MIN <= mag_norm <= MAG_FIELD_MAX):
        return False
    prev_norm = MAG_FILTER_STATE.get("norm")
    if prev_norm is not None and prev_norm > 0:
        if mag_norm > prev_norm * MAG_NORM_SPIKE_RATIO:
            return False
        if mag_norm < prev_norm / MAG_NORM_SPIKE_RATIO:
            return False
    return True

def _hampel_filter_angle(window: list, new_value: float) -> float:
    """
    Hampel filter for angular data (handles 0/360 wraparound)
    Returns: filtered angle value
    """
    # Window 업데이트
    window.append(new_value)
    if len(window) > HAMPEL_WINDOW_SIZE:
        window.pop(0)

    # 최소 3개 샘플 필요
    if len(window) < 3:
        return sum(window) / len(window)

    # Unwrap angles (0/360 경계 처리)
    unwrapped = [window[0]]
    for i in range(1, len(window)):
        diff = _angle_diff_deg(window[i], unwrapped[i-1])
        unwrapped.append(unwrapped[i-1] + diff)

    # Median 및 MAD 계산
    sorted_vals = sorted(unwrapped)
    n = len(sorted_vals)
    median_val = sorted_vals[n // 2] if n % 2 == 1 else (sorted_vals[n//2-1] + sorted_vals[n//2]) / 2

    deviations = [abs(x - median_val) for x in unwrapped]
    sorted_devs = sorted(deviations)
    mad = sorted_devs[n // 2] if n % 2 == 1 else (sorted_devs[n//2-1] + sorted_devs[n//2]) / 2

    # Outlier detection & replacement
    threshold = HAMPEL_THRESHOLD * max(mad, HAMPEL_MIN_MAD)
    filtered = []
    for val in unwrapped:
        if abs(val - median_val) > threshold:
            filtered.append(median_val)  # Replace outlier
        else:
            filtered.append(val)

    # 평균 계산
    avg = sum(filtered) / len(filtered)

    # Wrap back to 0-360
    return _wrap_angle_deg(avg)

def _filter_mag(mx, my, mz):
    if not MAG_FILTER_STATE["init"]:
        MAG_FILTER_STATE["x"] = mx
        MAG_FILTER_STATE["y"] = my
        MAG_FILTER_STATE["z"] = mz
        MAG_FILTER_STATE["init"] = True
        return mx, my, mz
    a = MAG_FILTER_ALPHA
    MAG_FILTER_STATE["x"] = a * mx + (1 - a) * MAG_FILTER_STATE["x"]
    MAG_FILTER_STATE["y"] = a * my + (1 - a) * MAG_FILTER_STATE["y"]
    MAG_FILTER_STATE["z"] = a * mz + (1 - a) * MAG_FILTER_STATE["z"]
    return MAG_FILTER_STATE["x"], MAG_FILTER_STATE["y"], MAG_FILTER_STATE["z"]

def init_imu(i2c=None):
    import board
    import busio
    import adafruit_bno055
    
    max_retries = 5
    last_error = None
    
    if i2c is None:
        i2c = busio.I2C(board.SCL, board.SDA, frequency=I2C_FREQUENCY)

    for attempt in range(max_retries):
        try:
            # 버스 잠금 + 스캔으로 주소 존재 확인
            with I2CLock():
                t0 = time.time()
                while not i2c.try_lock():
                    time.sleep(0.01)
                    if time.time() - t0 > 1.0:
                        raise RuntimeError("I2C lock timeout")

                try:
                    addrs = i2c.scan()
                finally:
                    i2c.unlock()

            # BNO055 기본 주소: 0x28 또는 0x29
            if 0x28 not in addrs and 0x29 not in addrs:
                last_error = RuntimeError(f"BNO055 not found on I2C bus. scan={addrs}")
                time.sleep(0.5)
                continue

            addr = 0x28 if 0x28 in addrs else 0x29

            with I2CLock():
                # BNO055 초기화
                sensor = adafruit_bno055.BNO055_I2C(i2c, address=addr)
                time.sleep(0.2)
                
                # BNO055 모드 설정 (NDOF: 9축 융합 모드)
                sensor.mode = adafruit_bno055.NDOF_MODE
                time.sleep(0.1)

            time.sleep(0.5)
            print(f"BNO055 initialized at address 0x{addr:02X}")
            return i2c, sensor

        except (KeyError, IndexError, OSError, RuntimeError, ValueError) as e:
            last_error = e
            if attempt < max_retries - 1:
                time.sleep(0.5)
                continue
            break

    raise RuntimeError(f"IMU initialization failed after {max_retries} attempts: {last_error}")


def read_sensor_data(sensor):
    global angle_window
    global LAST_VALID_SENSORS
    
    try:
        with I2CLock():
            # Quaternion 읽기 (Euler 대신 사용)
            quat = sensor.quaternion
            acc = sensor.acceleration
            mag = sensor.magnetic
            gyr = sensor.gyro
            
        # Quaternion None 체크
        if quat is None or None in quat:
            return False
            
        # BNO055 quaternion 순서: (w, x, y, z)
        w, x, y, z = quat
        
    except Exception as e:
        print(f"Read error: {e}")
        return False
    
    # Quaternion → Euler 변환 (A 코드와 동일한 방식)
    yaw = math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2)))
    roll = math.degrees(math.atan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2)))
    
    pitch_val = max(-1, min(1, 2*(w*y - z*x)))
    pitch = math.degrees(math.asin(pitch_val))
    
    # 장착 방향 보정 (A 코드와 동일)
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
    yaw = _wrap_angle_deg(yaw)
    roll = _wrap_angle_deg(roll)
    pitch = _wrap_angle_deg(pitch)
    
    # 가속도, 자이로, 자기장 처리
    try:
        if acc and None not in acc:
            accX, accY, accZ = acc
            LAST_VALID_SENSORS["acc"] = (accX, accY, accZ)
        else:
            accX, accY, accZ = LAST_VALID_SENSORS["acc"]
            
        if mag and None not in mag:
            magX, magY, magZ = mag
            mag_norm = math.sqrt(magX * magX + magY * magY + magZ * magZ)
            if _mag_norm_is_valid(mag_norm):
                magX, magY, magZ = _filter_mag(magX, magY, magZ)
                MAG_FILTER_STATE["norm"] = mag_norm
                LAST_VALID_SENSORS["mag"] = (magX, magY, magZ)
            else:
                magX, magY, magZ = LAST_VALID_SENSORS["mag"]
        else:
            magX, magY, magZ = LAST_VALID_SENSORS["mag"]
            
        if gyr and None not in gyr:
            gyrX, gyrY, gyrZ = gyr
            LAST_VALID_SENSORS["gyr"] = (gyrX, gyrY, gyrZ)
        else:
            gyrX, gyrY, gyrZ = LAST_VALID_SENSORS["gyr"]
            
    except Exception as e:
        print(f"Sensor data error: {e}")
        accX, accY, accZ = LAST_VALID_SENSORS["acc"]
        magX, magY, magZ = LAST_VALID_SENSORS["mag"]
        gyrX, gyrY, gyrZ = LAST_VALID_SENSORS["gyr"]

    # Yaw drift 보정 (자기장 기반)
    try:
        if magX != 0.0 or magY != 0.0:
            mag_heading = math.degrees(math.atan2(magY, magX))
            if IMU_MOUNTED_ON_BOTTOM:
                mag_heading = -mag_heading
            if IMU_FORWARD_AXIS == 'Y':
                mag_heading += 90
            try:
                from lib import config
                mag_heading += config.YAW_OFFSET
            except Exception:
                pass
            mag_heading = _wrap_angle_deg(mag_heading)
            yaw = _wrap_angle_deg(yaw + YAW_CORRECTION_GAIN * _angle_diff_deg(mag_heading, yaw))
    except Exception:
        pass
    
    # Hampel 필터 (보정된 yaw 사용, 스파이크 제거)
    avg_yaw = round(_hampel_filter_angle(angle_window[0], yaw), 4)
    avg_roll = round(_hampel_filter_angle(angle_window[1], roll), 4)
    avg_pitch = round(_hampel_filter_angle(angle_window[2], pitch), 4)

    accX, accY, accZ = round(accX, 4), round(accY, 4), round(accZ, 4)
    gyrX, gyrY, gyrZ = round(gyrX, 4), round(gyrY, 4), round(gyrZ, 4)
    magX, magY, magZ = round(magX, 4), round(magY, 4), round(magZ, 4)
    
    log_imu(f"{avg_roll:.4f},{avg_pitch:.4f},{avg_yaw:.4f},{accX},{accY},{accZ},{magX},{magY},{magZ},{gyrX},{gyrY},{gyrZ}")
    
    return (avg_roll, avg_pitch, avg_yaw, accX, accY, accZ, magX, magY, magZ, gyrX, gyrY, gyrZ)


def imu_terminate(i2c):
    if i2c is not None:
        i2c.deinit()


def reset_angle_window():
    global angle_window
    angle_window = [[], [], []]


def reinit_imu(i2c, sensor):
    """에러 발생 시 IMU 재초기화"""
    try:
        if sensor is not None:
            del sensor
    except Exception:
        pass
    
    time.sleep(2)
    
    # 윈도우 리셋
    reset_angle_window()
    
    # 재시도 로직 (최대 3회)
    max_retries = 3
    last_error = None
    for attempt in range(max_retries):
        try:
            i2c, sensor = init_imu(i2c)
            return i2c, sensor
        except Exception as e:
            last_error = e
            if attempt < max_retries - 1:
                time.sleep(1)
                continue
            break
    raise RuntimeError(f"Failed to reinitialize IMU after {max_retries} attempts: {last_error}")


if __name__ == "__main__":
    i2c, sensor = init_imu()
    
    try:
        consecutive_failures = 0
        while True:
            data = read_sensor_data(sensor)
            if data == False:
                consecutive_failures += 1
                if consecutive_failures < READ_FAIL_REINIT_THRESHOLD:
                    time.sleep(0.1)
                    continue
                consecutive_failures = 0
                try:
                    i2c, sensor = reinit_imu(i2c, sensor)
                    time.sleep(0.5)
                except Exception as e:
                    print(f"Reinit error: {e}")
                    time.sleep(0.5)
                    try:
                        i2c, sensor = reinit_imu(i2c, sensor)     
                    except Exception as e2:
                        print(f"Reinit failed: {e2}")
                        break
                continue
            
            consecutive_failures = 0
            print(data)
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\nTerminating IMU...")
        imu_terminate(i2c)
