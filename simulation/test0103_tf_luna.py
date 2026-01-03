import time
import board
import busio

I2C_ADDR = 0x10

i2c = busio.I2C(board.SCL, board.SDA)

while not i2c.try_lock():
    pass

def read_distance_cm():
    buf = bytearray(2)
    i2c.writeto_then_readfrom(I2C_ADDR, bytes([0x00]), buf)
    return buf[0] | (buf[1] << 8)

try:
    while True:
        d = read_distance_cm()
        print(f"Distance: {d} cm")
        time.sleep(0.1)
finally:
    i2c.unlock()
