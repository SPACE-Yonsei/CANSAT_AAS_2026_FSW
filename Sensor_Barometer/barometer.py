# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import os
import math
try:
    import fcntl
except Exception:
    fcntl = None
from collections import deque
from datetime import datetime

log_dir = './sensorlogs'
if not os.path.exists(log_dir): 
    os.makedirs(log_dir)

## Create sensor log file
barometerlogfile = open(os.path.join(log_dir, 'barometer.txt'), 'a')

SEA_LEVEL_PRESSURE_HPA = 1013.25
try:
    SEA_LEVEL_PRESSURE_HPA = float(os.getenv("BMP3XX_SEA_LEVEL_PRESSURE", "1013.25"))
except ValueError:
    SEA_LEVEL_PRESSURE_HPA = 1013.25

I2C_LOCK_PATH = os.getenv("I2C_LOCK_PATH", "/tmp/i2c-1.lock")


class I2CLock:
    def __init__(self, path=I2C_LOCK_PATH):
        self.path = path
        self.fd = None

    def __enter__(self):
        if fcntl is None:
            return self
        self.fd = open(self.path, "w")
        fcntl.flock(self.fd, fcntl.LOCK_EX)
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

PRESSURE_WINDOW = deque(maxlen=5)
TEMPERATURE_WINDOW = deque(maxlen=5)
ALTITUDE_WINDOW = deque(maxlen=5)

def _sanitize(value, min_val, max_val):
    try:
        if isinstance(value, (int, float)) and math.isfinite(value) and min_val <= value <= max_val:
            return float(value)
    except Exception:
        pass
    return None

def _median(values):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[len(ordered) // 2]

def log_barometer(text):
    t = datetime.now().isoformat(sep=' ', timespec='milliseconds')
    string_to_write = f'{t},{text}\n'
    barometerlogfile.write(string_to_write)
    barometerlogfile.flush()
    
def init_barometer():
    import adafruit_bmp3xx
    import board

    # I2C setup
    i2c = board.I2C()
    with I2CLock():
        bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)
    bmp.pressure_oversampling = 8
    bmp.temperature_oversampling = 2
    bmp.sea_level_pressure = SEA_LEVEL_PRESSURE_HPA

    return i2c, bmp

# Read Barometer data and returns tuple (pressure, temperature, altitude)
def read_barometer(bmp, offset:float):
    global altitude_altZero   # if altitude_altZero != 0 이라면, 57번 줄 offset을 altitude_altZero로 사용하도록 변경해야함.
    with I2CLock():
        pressure = _sanitize(bmp.pressure, 300.0, 1100.0)
        temperature = _sanitize(bmp.temperature, -40.0, 85.0)
        altitude = _sanitize(bmp.altitude, -500.0, 10000.0)
    offset = _sanitize(offset, -10000.0, 10000.0) or 0.0

    if pressure is not None:
        PRESSURE_WINDOW.append(pressure)
    if temperature is not None:
        TEMPERATURE_WINDOW.append(temperature)
    if altitude is not None:
        ALTITUDE_WINDOW.append(altitude)

    pressure = _median(PRESSURE_WINDOW)
    temperature = _median(TEMPERATURE_WINDOW)
    altitude = _median(ALTITUDE_WINDOW)

    if pressure is None:
        pressure = 0.0
    if temperature is None:
        temperature = 0.0
    if altitude is None:
        altitude = 0.0

    pressure = round(pressure, 2)
    temperature = round(temperature, 2)
    altitude = round(altitude, 2)

    # Apply offset
    altitude = round(altitude - offset, 2)
    log_barometer(f"{pressure:.2f}, {temperature:.2f}, {altitude:.2f}")
    
    return ( pressure, temperature, altitude )

def terminate_barometer(i2c):
    if i2c is not None:
        i2c.deinit()
    return

if __name__ == "__main__":
    i2c, bmp = init_barometer()
    try:
        while True:
            data = read_barometer(bmp, 0)
            print(data)
            time.sleep(1)
    except KeyboardInterrupt:
        terminate_barometer(i2c)

