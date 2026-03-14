import time
import os
try:
    import fcntl
except Exception:
    fcntl = None
from datetime import datetime

I2C_BUS_NUM = 1
GNSS_ADDR   = 0x42
READ_SIZE   = 32

I2C_LOCK_PATH = os.getenv("I2C_LOCK_PATH", "/tmp/i2c-1.lock")
I2C_LOCK_TIMEOUT_SEC = float(os.getenv("I2C_LOCK_TIMEOUT_SEC", "2.0"))


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

############################################################
# log 데이터 수신
############################################################
log_dir = './sensorlogs'
if not os.path.exists(log_dir):
    os.makedirs(log_dir)

gpslogfile = open(os.path.join(log_dir, 'gps.txt'), 'a')

def log_gps(text):
    t = datetime.now().isoformat(sep=' ', timespec='milliseconds')
    string_to_write = f'{t},{text}\n'
    gpslogfile.write(string_to_write)
    gpslogfile.flush()

##############################################################
# GPS 데이터 수신 (GNSS 7 Click I2C)
##############################################################
from smbus2 import SMBus

def init_gps():
    bus = SMBus(I2C_BUS_NUM)
    return bus


def _i2c_read_block(bus):
    try:
        with I2CLock():
            data = bus.read_i2c_block_data(GNSS_ADDR, 0xFF, READ_SIZE)
    except OSError as e:
        print(f"[DEBUG][i2c] OSError with reg 0xFF: {e}, retrying with 0x00")
        with I2CLock():
            data = bus.read_i2c_block_data(GNSS_ADDR, 0x00, READ_SIZE)
    #print(f"[DEBUG][i2c] raw chunk: {bytes(data)}")
    return bytes(data)


_read_buffer = b''


def read_gps(pi, timeout: float = 1.0):
    global _read_buffer
    bus = pi
    NMEA_lines = []
    got_valid_data = False
    start = time.time()

    while time.time() - start < timeout:
        if bus is None:
            break

        try:
            chunk = _i2c_read_block(bus)
        except OSError:
            time.sleep(0.01)
            continue

        if not chunk or not chunk.strip(b"\x00\xff"):
            print(f"[DEBUG][read_gps] empty/no-data chunk (all 0x00 or 0xFF), skipping")
            if got_valid_data:
                break
            time.sleep(0.01)
            continue

        got_valid_data = True
        _read_buffer += chunk

        while b'\n' in _read_buffer:
            line, _read_buffer = _read_buffer.split(b'\n', 1)
            line = line + b'\n'
            if b'$' not in line:
                continue
            line = line[line.find(b'$'):]
            print(f"[DEBUG][read_gps] NMEA line: {line.decode('ascii', errors='ignore').strip()}")
            NMEA_lines.append(line)

    print(f"[DEBUG][read_gps] total NMEA lines collected: {len(NMEA_lines)}")
    return NMEA_lines


def parse_gps_data(NMEA_lines):
    gga_data = None
    rmc_data = None
    gps_data = None

    for line in NMEA_lines:
        try:
            if isinstance(line, bytes):
                decoded_line = line.decode('ascii', errors='ignore').strip()
            else:
                decoded_line = line.strip()
        except UnicodeDecodeError:
            continue

        # GGA
        if decoded_line.startswith(('$GPGGA', '$GNGGA')):
            parts = decoded_line.split(',')
            print(f"[DEBUG][parse] GGA ({len(parts)} fields): {parts}")
            if len(parts) > 7:
                gga_data = parts

        # RMC
        elif decoded_line.startswith(('$GPRMC', '$GNRMC')):
            parts = decoded_line.split(',')
            print(f"[DEBUG][parse] RMC ({len(parts)} fields): {parts}")
            if len(parts) > 10:
                rmc_data = parts
        else:
            continue

    if gga_data and rmc_data:
        gps_data = [gga_data, rmc_data]
    else:
        print(f"[DEBUG][parse] parse result - gga_data: {'OK' if gga_data else 'MISSING'}, rmc_data: {'OK' if rmc_data else 'MISSING'}")

    return gps_data


def terminate_gps(pi):
    try:
        if pi is not None:
            pi.close()
    except Exception:
        pass
    return


##############################################################
# GPS 데이터 가공
##############################################################
def unit_convert_deg(raw_angle):
    deg = int(raw_angle // 100)
    minutes = raw_angle - deg * 100
    decimal_deg = deg + minutes / 60
    return decimal_deg


def gps_readdata(pi):
    # Reduce timeout from 1.0 to 0.2 seconds to avoid I2C bus blocking
    # This allows faster polling when running with other sensors
    NMEA_lines = read_gps(pi, timeout=0.2)
    gps_data = parse_gps_data(NMEA_lines)
    modified_gps_data = []

    if gps_data is not None:
        gga = gps_data[0]
        rmc = gps_data[1] if len(gps_data) > 1 else None

        # 시간
        if gga[1]:
            gps_time_raw = gga[1]
            hour, minute, second = gps_time_raw[0:2], gps_time_raw[2:4], gps_time_raw[4:6]
            gps_time = f"{hour}:{minute}:{second}"
        else:
            gps_time = "00:00:00"

        # 고도
        try:
            alt = round(float(gga[9]), 2) if gga[9] else 0
        except (ValueError, IndexError):
            alt = 0

        # 위도
        try:
            lat = unit_convert_deg(float(gga[2])) if gga[2] else 0
            # gga[3]가 'S'(남위)이면 음수, 'N'(북위)이면 양수
            if len(gga) > 3 and gga[3] == 'S':
                lat = lat * -1
        except (ValueError, IndexError):
            lat = 0

        # 경도
        try:
            lon = unit_convert_deg(float(gga[4])) if gga[4] else 0
            # gga[5]가 'W'(서경)이면 음수, 'E'(동경)이면 양수
            if len(gga) > 5 and gga[5] == 'W':
                lon = lon * -1
        except (ValueError, IndexError):
            lon = 0

        # 사용 위성 수
        try:
            fixed_sat = int(gga[7]) if gga[7] else 0
        except (ValueError, IndexError):
            fixed_sat = 0
        
        # Fix quality 확인 (디버깅용)
        try:
            fix_quality = int(gga[6]) if len(gga) > 6 and gga[6] else 0
        except (ValueError, IndexError):
            fix_quality = 0

        print(f"----------GGA[1] time: {gps_time}, GGA[2] lat: {lat}, GGA[4] lon: {lon}, GGA[6] fix_quality: {fix_quality}, GGA[7] fixed_sat: {fixed_sat}")

        # RMC 메시지에서 상태, 지상 속도, 방향 추출
        rmc_status = "V"  # V=void, A=active
        ground_speed_knots = 0.0
        ground_speed_ms = 0.0  # m/s 단위로 변환한 속도
        course_over_ground = 0.0  # 방향 (도, 0-360)

        if rmc is not None and len(rmc) > 8:
            try:
                # 상태 추출 (RMC[2] = Status, A=active, V=void)
                if rmc[2]:
                    rmc_status = rmc[2]

                # 속도 추출 (RMC[7] = Speed over ground in knots)
                if rmc[7]:
                    ground_speed_knots = float(rmc[7])
                    # 노트를 m/s로 변환: 1 knot = 0.514444 m/s
                    ground_speed_ms = ground_speed_knots * 0.514444

                # 방향 추출 (RMC[8] = Course over ground in degrees, 0-360)
                if rmc[8]:
                    course_over_ground = float(rmc[8])
                    # 0-360 범위로 정규화
                    while course_over_ground < 0:
                        course_over_ground += 360
                    while course_over_ground >= 360:
                        course_over_ground -= 360
                print(f"----------RMC[2] Status: {rmc_status}, RMC[7] Ground Speed (m/s): {ground_speed_ms}, RMC[8] Course: {course_over_ground}")
            except (ValueError, IndexError, TypeError):
                # 파싱 오류 시 기본값 유지
                rmc_status = "V"
                ground_speed_knots = 0.0
                ground_speed_ms = 0.0
                course_over_ground = 0.0

        modified_gps_data = [gps_time, alt, lat, lon, fixed_sat, fix_quality, rmc_status, ground_speed_ms, course_over_ground]
        print(f"[DEBUG][gps_readdata] output: time={gps_time}, alt={alt}, lat={lat}, lon={lon}, sats={fixed_sat}, fix={fix_quality}, status={rmc_status}, spd={ground_speed_ms:.3f}m/s, cog={course_over_ground}")
        # Fix quality가 0이면 fix가 없는 상태이므로 로그에 기록
        if fix_quality == 0:
            log_gps(f"{gps_time},{alt},{lat},{lon},{fixed_sat},fix_quality={fix_quality}")
        else:
            log_gps(f"{gps_time},{alt},{lat},{lon},{fixed_sat}")
        return modified_gps_data

    # 데이터 없을 때 None 반환 (gpsapp이 이전 값을 유지하도록)
    return None


if __name__ == "__main__":
    pi = init_gps()
    try:
        while True:
            gps_data = gps_readdata(pi)
            print(gps_data)
            if gps_data is None:
                print("No GPS data")
                time.sleep(0.05)
    except KeyboardInterrupt:
        print("Stop")
    finally:
        terminate_gps(pi)