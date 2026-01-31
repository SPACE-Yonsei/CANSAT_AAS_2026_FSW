#!/usr/bin/env python3
"""
TF-Luna I2C Distance Sensor Driver
TF-Luna - Time-of-Flight LiDAR sensor (0.2m ~ 8m)

I2C Address: 0x10 (default)
Range: 0.2m ~ 8m
Resolution: 1cm
"""

import time
import os
try:
    import fcntl
except Exception:
    fcntl = None
from lib import events, appargs

# I2C Address
TFLUNA_I2C_ADDR = 0x10

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

# Register addresses (from TF-Luna datasheet)
REG_DISTANCE_LOW = 0x00   # Distance low byte
REG_DISTANCE_HIGH = 0x01  # Distance high byte
REG_FLUX_LOW = 0x02       # Signal strength low byte
REG_FLUX_HIGH = 0x03      # Signal strength high byte
REG_TEMP_LOW = 0x04       # Temperature low byte
REG_TEMP_HIGH = 0x05      # Temperature high byte

# Global sensor instance holder
_sensor = None


def init_TFLuna(address=TFLUNA_I2C_ADDR):
    global _sensor
    
    try:
        import board
        import busio
        
        i2c = busio.I2C(board.SCL, board.SDA)
        
        class TFLuna:
            def __init__(self, i2c, address):
                self.i2c = i2c
                self.address = address
            
            def _read_register(self, register, length=1):
                """Read register from TF-Luna"""
                with I2CLock():
                    while not self.i2c.try_lock():
                        pass
                    try:
                        result = bytearray(length)
                        self.i2c.writeto_then_readfrom(
                            self.address, bytes([register]), result
                        )
                        return result
                    finally:
                        self.i2c.unlock()
            
            def read_distance_cm(self):
                """Read distance in cm"""
                # Read distance register (0x00 = low, 0x01 = high)
                buf = self._read_register(REG_DISTANCE_LOW, 2)
                distance_cm = buf[0] | (buf[1] << 8)
                return distance_cm
            
            def read_distance_mm(self):
                """Read distance in mm"""
                distance_cm = self.read_distance_cm()
                return distance_cm * 10
            
            def read_flux(self):
                """Read signal strength"""
                buf = self._read_register(REG_FLUX_LOW, 2)
                flux = buf[0] | (buf[1] << 8)
                return flux
            
            def read_temperature(self):
                """Read temperature in degrees Celsius"""
                buf = self._read_register(REG_TEMP_LOW, 2)
                temp_raw = buf[0] | (buf[1] << 8)
                # Temperature is in 0.01°C units
                return temp_raw / 100.0
            
            def unlock(self):
                """Compatibility (no persistent lock held)."""
                return
        
        _sensor = TFLuna(i2c, address)
        
        # Wait a bit for sensor to stabilize
        time.sleep(0.1)
        
        return _sensor
        
    except Exception as e:
        # print(f"TF-Luna init error: {e}")
        events.LogEvent(appargs.DistanceAppArg.AppName, events.EventType.warning, f"TF-Luna init error: {e}")
        return None


def read_distance(sensor=None) -> int:
    global _sensor
    
    if sensor is None:
        sensor = _sensor
    
    if sensor is None:
        return 0
    
    try:
        distance_mm = sensor.read_distance_mm()
        
        # TF-Luna range: 0.2m (200mm) to 8m (8000mm)
        if distance_mm < 200 or distance_mm > 8000:
            return 0
        
        return int(distance_mm)
        
    except Exception as e:
        #print(f"TF-Luna read error: {e}")
        return 0    


def read_distance_data(sensor=None) -> int:
    return read_distance(sensor)


def terminate_TFLuna(sensor=None):
    global _sensor
    
    if sensor is None:
        sensor = _sensor
    
    if sensor is not None:
        try:
            sensor.unlock()
        except:
            pass
    
    _sensor = None


def init_distance_sensor(address=TFLUNA_I2C_ADDR):
    """
    Initialize TF-Luna I2C sensor.
    Alias for init_TFLuna() for backward compatibility.
    """
    return init_TFLuna(address)


def terminate_distance_sensor(sensor=None):
    """
    Alias for terminate_TFLuna() for backward compatibility.
    """
    terminate_TFLuna(sensor)


# Test function
if __name__ == "__main__":
    print("TF-Luna I2C Distance Sensor Test")
    print("=" * 30)
    
    sensor = init_TFLuna()
    
    if sensor is None:
        print("Failed to initialize sensor!")
        exit(1)
    
    print("Sensor initialized. Reading distances...")
    print("Press Ctrl+C to stop\n")
    
    try:
        while True:
            distance_mm = read_distance(sensor)
            distance_cm = distance_mm / 10
            distance_m = distance_mm / 1000
            
            if distance_mm > 0:
                flux = sensor.read_flux()
                temp = sensor.read_temperature()
                print(f"Distance: {distance_mm:4d} mm | {distance_cm:6.1f} cm | {distance_m:.2f} m | Flux: {flux} | Temp: {temp:.1f}°C")
            else:
                print("Distance: Out of range")
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nStopping...")
    
    finally:
        terminate_TFLuna(sensor)
        print("Done.")

