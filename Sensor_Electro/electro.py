import time
import os
from datetime import datetime

############################################################
# log 데이터 수신
############################################################

log_dir = './sensorlogs'
if not os.path.exists(log_dir):
    os.makedirs(log_dir)

Electrologfile = open(os.path.join(log_dir, 'electro_ina228.txt'), 'a')

def log_Electro(text: str):
    t = datetime.now().isoformat(sep=' ', timespec='milliseconds')
    string_to_write = f"{t},{text}\n"
    Electrologfile.write(string_to_write)
    Electrologfile.flush()


##############################################################
# INA228 초기화
##############################################################
def init_INA228(address=0x40):
    """
    Initialize INA228 current/power monitor.
    
    Args:
        address: I2C address (default 0x40)
    
    Returns:
        I2C device object for INA228
    """
    import board
    import busio
    
    i2c = busio.I2C(board.SCL, board.SDA)
    
    # INA228 register addresses
    class INA228:
        def __init__(self, i2c, address):
            self.i2c = i2c
            self.address = address
            self._shunt_resistor = 0.001  # 1mOhm default, adjust if needed
            
        def _read_register(self, register, length=2):
            """Read register from INA228"""
            while not self.i2c.try_lock():
                pass
            try:
                result = bytearray(length)
                self.i2c.writeto_then_readfrom(
                    self.address, bytes([register]), result
                )
                return int.from_bytes(result, 'big', signed=False)
            finally:
                self.i2c.unlock()
        
        @property
        def bus_voltage(self):
            """Bus voltage in volts"""
            # Register 0x02: VBUS (16-bit, LSB = 195.3125 µV)
            raw = self._read_register(0x02)
            return (raw * 195.3125e-6) / 1000.0  # Convert to volts
        
        @property
        def shunt_voltage(self):
            """Shunt voltage in volts"""
            # Register 0x04: VSHUNT (24-bit, LSB = 312.5 nV)
            raw = self._read_register(0x04, 3)
            # Handle 24-bit signed value
            if raw & 0x800000:
                raw = raw - 0x1000000
            return raw * 312.5e-9
        
        @property
        def current(self):
            """Current in amperes"""
            # Register 0x07: CURRENT (24-bit signed)
            raw = self._read_register(0x07, 3)
            # Handle 24-bit signed value
            if raw & 0x800000:
                raw = raw - 0x1000000
            # Current LSB = 10 nA (default, depends on calibration)
            return raw * 10e-9
        
        @property
        def power(self):
            """Power in watts"""
            # Register 0x08: POWER (24-bit)
            raw = self._read_register(0x08, 3)
            # Power LSB = 3.2 µW (default, depends on calibration)
            return raw * 3.2e-6
    
    ina = INA228(i2c, address)
    return ina


##############################################################
# 측정 함수들
##############################################################
def read_voltage(dev):
    """Bus Voltage [V]"""
    return dev.bus_voltage


def read_current(dev):
    """Current [A]"""
    return dev.current


def read_power(dev):
    """Power [W]"""
    return dev.power


##############################################################
# 단독 테스트용
##############################################################
if __name__ == "__main__":
    ina = init_INA228()

    try:
        while True:
            v = read_voltage(ina)
            i = read_current(ina)
            p = read_power(ina)

            msg = f"V={v:.3f} V, I={i:.6f} A, P={p:.3f} W"
            print(msg)
            log_Electro(msg)

            time.sleep(1.0)

    except KeyboardInterrupt:
        print("Exiting")
