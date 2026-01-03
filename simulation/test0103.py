import board
import busio
from adafruit_bno08x import BNO_REPORT_ACCELEROMETER
from adafruit_bno08x.i2c import BNO08X_I2C

i2c = busio.I2C(board.SCL, board.SDA)
bno = BNO08X_I2C(i2c)

# 가속도계 활성화
bno.enable_feature(BNO_REPORT_ACCELEROMETER)

while True:
    accel_x, accel_y, accel_z = bno.acceleration
    print(f"X: {accel_x:.2f}, Y: {accel_y:.2f}, Z: {accel_z:.2f} m/s^2")
