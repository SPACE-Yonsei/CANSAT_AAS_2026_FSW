#!/usr/bin/env python3
"""
VL53L1X ToF Distance Sensor Driver
VL53L1CX - Time-of-Flight ranging sensor (up to 4m)

I2C Address: 0x29 (default)
Range: 40mm ~ 4000mm
Accuracy: ±3% (typical)
"""

import time

# I2C Address
VL53L1X_I2C_ADDR = 0x29

# Ranging modes
SHORT_RANGE = 1   # up to 1.3m, better ambient immunity
MEDIUM_RANGE = 2  # up to 3m, default
LONG_RANGE = 3    # up to 4m, max distance

# Timing budget (ms) - affects accuracy
TIMING_BUDGET_MS = 50

# Global sensor instance holder
_sensor = None


def init_VL53L1X(ranging_mode: int = MEDIUM_RANGE):
    """
    Initialize VL53L1X ToF sensor.
    
    Args:
        ranging_mode: SHORT_RANGE, MEDIUM_RANGE, or LONG_RANGE
    
    Returns:
        VL53L1X sensor object or None on failure
    """
    global _sensor
    
    try:
        import board
        import adafruit_vl53l1x
        
        i2c = board.I2C()
        _sensor = adafruit_vl53l1x.VL53L1X(i2c)
        
        # Configure sensor
        _sensor.distance_mode = ranging_mode
        _sensor.timing_budget = TIMING_BUDGET_MS
        
        # Start ranging
        _sensor.start_ranging()
        
        # Wait for first measurement
        time.sleep(0.1)
        
        return _sensor
        
    except Exception as e:
        print(f"VL53L1X init error: {e}")
        return None


def read_distance(sensor=None) -> int:
    """
    Read distance from VL53L1X sensor.
    
    Args:
        sensor: VL53L1X sensor object (optional, uses global if None)
    
    Returns:
        Distance in mm (0 if error or out of range)
    """
    global _sensor
    
    if sensor is None:
        sensor = _sensor
    
    if sensor is None:
        return 0
    
    try:
        if sensor.data_ready:
            distance_cm = sensor.distance  # Returns in cm
            sensor.clear_interrupt()
            
            if distance_cm is not None:
                return int(distance_cm * 10)  # Convert to mm
            else:
                return 0
        else:
            return 0
            
    except Exception as e:
        print(f"VL53L1X read error: {e}")
        return 0


def set_ranging_mode(sensor, mode: int):
    """
    Change ranging mode.
    
    Args:
        sensor: VL53L1X sensor object
        mode: SHORT_RANGE, MEDIUM_RANGE, or LONG_RANGE
    """
    if sensor is None:
        return
    
    try:
        sensor.stop_ranging()
        sensor.distance_mode = mode
        sensor.start_ranging()
    except Exception as e:
        print(f"VL53L1X mode change error: {e}")


def terminate_VL53L1X(sensor=None):
    """
    Stop sensor and cleanup.
    
    Args:
        sensor: VL53L1X sensor object (optional, uses global if None)
    """
    global _sensor
    
    if sensor is None:
        sensor = _sensor
    
    if sensor is not None:
        try:
            sensor.stop_ranging()
        except Exception:
            pass
    
    _sensor = None


# Test function
if __name__ == "__main__":
    print("VL53L1X ToF Sensor Test")
    print("=" * 30)
    
    sensor = init_VL53L1X(MEDIUM_RANGE)
    
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
            
            print(f"Distance: {distance_mm:4d} mm | {distance_cm:6.1f} cm | {distance_m:.2f} m")
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nStopping...")
    
    finally:
        terminate_VL53L1X(sensor)
        print("Done.")

