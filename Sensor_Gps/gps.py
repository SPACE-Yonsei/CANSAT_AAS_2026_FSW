import time
import os
try:
    import fcntl
except Exception:
    fcntl = None
from datetime import datetime

def _env_int(name: str, default: int) -> int:
    try:
        return int(os.getenv(name, str(default)))
    except (TypeError, ValueError):
        return default


I2C_BUS_NUM = _env_int("GPS_I2C_BUS", _env_int("FSW_I2C_BUS", 1))
GNSS_ADDR   = 0x42
READ_SIZE   = 32

I2C_LOCK_PATH = os.getenv("FSW_I2C_LOCK_FILE", "/tmp/fsw_i2c.lock")
I2C_LOCK_TIMEOUT_SEC = float(os.getenv("I2C_LOCK_TIMEOUT_SEC", "2.0"))
NMEA_CACHE_MAX_AGE_SEC = float(os.getenv("GPS_NMEA_CACHE_MAX_AGE_SEC", "2.0"))

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
    global _read_buffer, _last_gga_data, _last_rmc_data, _last_gga_ts, _last_rmc_ts
    _read_buffer = b''
    _last_gga_data = None
    _last_rmc_data = None
    _last_gga_ts = 0.0
    _last_rmc_ts = 0.0
    bus = SMBus(I2C_BUS_NUM)
    return bus


def _i2c_read_block(bus):
    try:
        with I2CLock():
            data = bus.read_i2c_block_data(GNSS_ADDR, 0xFF, READ_SIZE)
    except OSError as e:
        #print(f"[DEBUG][i2c] OSError with reg 0xFF: {e}, retrying with 0x00")
        with I2CLock():
            data = bus.read_i2c_block_data(GNSS_ADDR, 0x00, READ_SIZE)
    #print(f"[DEBUG][i2c] raw chunk: {bytes(data)}")
    return bytes(data)


_read_buffer = b''
_last_gga_data = None
_last_rmc_data = None
_last_gga_ts = 0.0
_last_rmc_ts = 0.0


def nmea_checksum_ok(sentence: str) -> bool:
    sentence = sentence.strip()
    if not sentence.startswith("$") or "*" not in sentence:
        return False

    body, checksum_text = sentence[1:].split("*", 1)

    try:
        expected = int(checksum_text[:2], 16)
    except ValueError:
        return False

    actual = 0
    for ch in body:
        actual ^= ord(ch)

    return actual == expected


def _gps_time_seconds(parts):
    try:
        raw = parts[1]
        if len(raw) < 6:
            return None
        return int(raw[0:2]) * 3600 + int(raw[2:4]) * 60 + int(raw[4:6])
    except (TypeError, ValueError, IndexError):
        return None


def _nmea_times_match(gga, rmc) -> bool:
    gga_s = _gps_time_seconds(gga)
    rmc_s = _gps_time_seconds(rmc)
    if gga_s is None or rmc_s is None:
        return False
    diff = abs(gga_s - rmc_s)
    diff = min(diff, 86400 - diff)
    return diff <= 1


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
            #print(f"[DEBUG][read_gps] empty/no-data chunk (all 0x00 or 0xFF), skipping")
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
            #print(f"[DEBUG][read_gps] NMEA line: {line!r}")
            NMEA_lines.append(line)

    #print(f"[DEBUG][read_gps] total NMEA lines collected: {len(NMEA_lines)}")
    return NMEA_lines


def parse_gps_data(NMEA_lines):
    global _last_gga_data, _last_rmc_data, _last_gga_ts, _last_rmc_ts
    gga_data = None
    rmc_data = None
    gps_data = None
    now = time.monotonic()

    for line in NMEA_lines:
        try:
            if isinstance(line, bytes):
                decoded_line = line.decode('ascii').strip()
            else:
                decoded_line = line.strip()
        except UnicodeDecodeError:
            continue

        if not nmea_checksum_ok(decoded_line):
            continue

        # GGA
        if decoded_line.startswith(('$GPGGA', '$GNGGA')):
            parts = decoded_line.split(',')
            #print(f"[DEBUG][parse] GGA ({len(parts)} fields): {parts}")
            if len(parts) > 7:
                gga_data = parts
                _last_gga_data = parts
                _last_gga_ts = now

        # RMC
        elif decoded_line.startswith(('$GPRMC', '$GNRMC')):
            parts = decoded_line.split(',')
            #print(f"[DEBUG][parse] RMC ({len(parts)} fields): {parts}")
            if len(parts) > 10:
                rmc_data = parts
                _last_rmc_data = parts
                _last_rmc_ts = now
        else:
            continue

    gga_fresh = _last_gga_data is not None and (now - _last_gga_ts) <= NMEA_CACHE_MAX_AGE_SEC
    rmc_fresh = _last_rmc_data is not None and (now - _last_rmc_ts) <= NMEA_CACHE_MAX_AGE_SEC

    # Position/altitude are GGA-based; stale GGA invalidates the row.
    # RMC may be absent, stale, or from a different GPS time.
    if gga_fresh:
        rmc = _last_rmc_data if rmc_fresh and _nmea_times_match(_last_gga_data, _last_rmc_data) else None
        rmc_ts = _last_rmc_ts if rmc is not None else 0.0
        gps_data = [_last_gga_data, rmc, _last_gga_ts, rmc_ts]

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


def _parse_nmea_coord(raw_value, hemisphere, is_lat: bool):
    try:
        if not raw_value or hemisphere not in ("N", "S", "E", "W"):
            return None
        value = unit_convert_deg(float(raw_value))
        limit = 90.0 if is_lat else 180.0
        if not (0.0 <= value <= limit):
            return None
        if hemisphere in ("S", "W"):
            value *= -1
        return value
    except (TypeError, ValueError):
        return None


def gps_readdata(pi):
    NMEA_lines = read_gps(pi, timeout=0.08)
    gps_data = parse_gps_data(NMEA_lines)
    if gps_data is None:
        return None

    gga = gps_data[0]
    rmc = gps_data[1] if len(gps_data) > 1 else None
    gga_sample_ts = float(gps_data[2]) if len(gps_data) > 2 else 0.0
    rmc_sample_ts = float(gps_data[3]) if len(gps_data) > 3 else 0.0

    if len(gga) > 1 and gga[1]:
        gps_time_raw = gga[1]
        hour, minute, second = gps_time_raw[0:2], gps_time_raw[2:4], gps_time_raw[4:6]
        gps_time = f"{hour}:{minute}:{second}"
    else:
        gps_time = "00:00:00"

    try:
        alt = round(float(gga[9]), 2) if len(gga) > 9 and gga[9] else None
    except (ValueError, IndexError):
        alt = None

    try:
        fixed_sat = int(gga[7]) if len(gga) > 7 and gga[7] else 0
    except (ValueError, IndexError):
        fixed_sat = 0

    try:
        fix_quality = int(gga[6]) if len(gga) > 6 and gga[6] else 0
    except (ValueError, IndexError):
        fix_quality = 0

    lat = None
    lon = None
    if fix_quality >= 1:
        lat = _parse_nmea_coord(gga[2] if len(gga) > 2 else "", gga[3] if len(gga) > 3 else "", True)
        lon = _parse_nmea_coord(gga[4] if len(gga) > 4 else "", gga[5] if len(gga) > 5 else "", False)

    try:
        hdop = float(gga[8]) if len(gga) > 8 and gga[8] else float('inf')
    except (ValueError, IndexError):
        hdop = float('inf')

    rmc_status = "V"
    ground_speed_ms = None
    course_over_ground = None

    if rmc is not None and len(rmc) > 8:
        try:
            rmc_status = rmc[2].strip().upper() if rmc[2] else "V"
            if rmc_status == "A":
                if rmc[7]:
                    ground_speed_ms = float(rmc[7]) * 0.514444
                if rmc[8]:
                    course_over_ground = float(rmc[8]) % 360.0
        except (ValueError, IndexError, TypeError):
            rmc_status = "V"
            ground_speed_ms = None
            course_over_ground = None

    modified_gps_data = [
        gps_time,             # [0]
        alt,                  # [1]
        lat,                  # [2]
        lon,                  # [3]
        fixed_sat,            # [4]
        fix_quality,          # [5]
        rmc_status,           # [6]
        ground_speed_ms,      # [7]
        course_over_ground,   # [8]
        gga_sample_ts,        # [9] monotonic GGA sample timestamp
        hdop,                 # [10]
        rmc_sample_ts,        # [11] monotonic RMC sample timestamp, or 0.0
    ]

    if fix_quality == 0:
        log_gps(f"{gps_time},{alt},{lat},{lon},{fixed_sat},fix_quality={fix_quality}")
    else:
        log_gps(f"{gps_time},{alt},{lat},{lon},{fixed_sat}")
    return modified_gps_data


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
