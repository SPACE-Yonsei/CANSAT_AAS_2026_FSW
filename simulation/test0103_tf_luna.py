import serial
import time

ser = serial.Serial('/dev/serial0', 115200, timeout=1)

# I2C 모드로 변경 명령
cmd_set_i2c = bytes([
    0x5A, 0x05, 0x0B, 0x01, 0x00
])

ser.write(cmd_set_i2c)
time.sleep(0.1)

# 설정 저장
cmd_save = bytes([
    0x5A, 0x04, 0x11, 0x6F
])

ser.write(cmd_save)
ser.close()

print("TF-Luna set to I2C mode")
