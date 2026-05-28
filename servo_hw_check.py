#!/usr/bin/env python3
"""
servo_hw_check.py — 서보 하드웨어 진단
  sudo python3 servo_hw_check.py
"""
import pigpio, time, sys

pi = pigpio.pi()
if not pi.connected:
    print("pigpiod 연결 실패. sudo pigpiod 후 재시도")
    sys.exit(1)

LEFT_PIN  = 12
RIGHT_PIN = 13

print(f"GPIO {LEFT_PIN}  현재 모드: {pi.get_mode(LEFT_PIN)}")
print(f"GPIO {RIGHT_PIN} 현재 모드: {pi.get_mode(RIGHT_PIN)}")

def sweep(pin, name):
    print(f"\n── {name} (GPIO {pin}) 스윕 테스트 ──")
    steps = [
        (1500, "중앙  1500µs"),
        (1000, "한쪽끝 1000µs"),
        (2000, "반대끝 2000µs"),
        (1500, "중앙복귀 1500µs"),
    ]
    for pw, label in steps:
        print(f"  {label}  → Enter 누르면 다음", end="", flush=True)
        pi.set_servo_pulsewidth(pin, pw)
        input()
    pi.set_servo_pulsewidth(pin, 0)

sweep(LEFT_PIN,  "LEFT 서보")
sweep(RIGHT_PIN, "RIGHT 서보")

pi.stop()
print("완료")
