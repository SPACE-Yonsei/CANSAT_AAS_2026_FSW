# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import os
import math
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

def _sanitize(value, min_val, max_val, default=0.0):
    try:
        if isinstance(value, (int, float)) and math.isfinite(value) and min_val <= value <= max_val:
            return float(value)
    except Exception:
        pass
    return default

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
    bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)
    bmp.pressure_oversampling = 8
    bmp.temperature_oversampling = 2
    bmp.sea_level_pressure = SEA_LEVEL_PRESSURE_HPA

    return i2c, bmp

# Read Barometer data and returns tuple (pressure, temperature, altitude)
def read_barometer(bmp, offset:float):
    global altitude_altZero   # if altitude_altZero != 0 이라면, 57번 줄 offset을 altitude_altZero로 사용하도록 변경해야함.
    pressure = _sanitize(bmp.pressure, 300.0, 1100.0)
    temperature = _sanitize(bmp.temperature, -40.0, 85.0)
    altitude = _sanitize(bmp.altitude, -500.0, 10000.0)
    offset = _sanitize(offset, -10000.0, 10000.0)

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

