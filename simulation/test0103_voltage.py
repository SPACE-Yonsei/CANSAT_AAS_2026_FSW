import time
import board
import busio
import adafruit_ina23x

# I2C 초기화 (i2c-1, GPIO2/3)
i2c = busio.I2C(board.SCL, board.SDA)

# INA238 = INA23x 계열
ina = adafruit_ina23x.INA23X(i2c, address=0x40)

print("=== INA238 Measurement Start ===")

while True:
    try:
        bus_v = ina.bus_voltage          # V
        shunt_v = ina.shunt_voltage      # V
        current = ina.current            # A
        power = ina.power                # W

        print(
            f"Bus: {bus_v:6.3f} V | "
            f"Shunt: {shunt_v*1000:7.3f} mV | "
            f"Current: {current*1000:7.3f} mA | "
