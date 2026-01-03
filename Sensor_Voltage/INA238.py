import time
import os
from datetime import datetime

############################################################
# log 데이터 수신
############################################################

log_dir = './sensorlogs'
if not os.path.exists(log_dir):
    os.makedirs(log_dir)

voltlogfile = open(os.path.join(log_dir, 'voltage_ina238.txt'), 'a')

def log_voltage(text: str):
    t = datetime.now().isoformat(sep=' ', timespec='milliseconds')
    string_to_write = f"{t},{text}\n"
    voltlogfile.write(string_to_write)
    voltlogfile.flush()


##############################################################
# INA238 초기화
##############################################################
def init_INA238():
    import board
    import busio
    import adafruit_ina23x

    i2c = busio.I2C(board.SCL, board.SDA)
    ina = adafruit_ina23x.INA23X(i2c)

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
    ina = init_INA238()

    try:
        while True:
            v = read_voltage(ina)
            i = read_current(ina)
            p = read_power(ina)

            msg = f"V={v:.3f} V, I={i:.6f} A, P={p:.3f} W"
            print(msg)
            log_voltage(msg)

            time.sleep(1.0)

    except KeyboardInterrupt:
        print("Exiting")
