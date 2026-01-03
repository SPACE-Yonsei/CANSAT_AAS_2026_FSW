import smbus
import time

I2C_ADDR = 0x10
bus = smbus.SMBus(1)

def read_distance_cm():
    data = bus.read_i2c_block_data(I2C_ADDR, 0x00, 2)
    return data[0] + (data[1] << 8)

while True:
    try:
        d = read_distance_cm()
        print(f"Distance: {d} cm")
    except Exception as e:
        print("I2C error:", e)
    time.sleep(0.1)
