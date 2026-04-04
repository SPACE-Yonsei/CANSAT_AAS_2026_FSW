#!/usr/bin/env python3
"""
Solenoid control for Payload-Egg drop
BCM 6: Egg 릴레이/솔레노이드 — 번와이어와 동일 릴레이 보드(relay_levels).
"""

import time
import RPi.GPIO as GPIO

from Sensor_Motor.relay_levels import RELAY_ACTIVATE_LEVEL, RELAY_DEACTIVATE_LEVEL

SOLENOID_PIN = 6
SOLENOID_ACTIVATE_LEVEL = RELAY_ACTIVATE_LEVEL
SOLENOID_DEACTIVATE_LEVEL = RELAY_DEACTIVATE_LEVEL
SOLENOID_DURATION = 0.5  # 솔레노이드 작동 시간 (초)
SOLENOID_REPEAT = 3      # 솔레노이드 반복 횟수

def init_solenoid():
    """Idle = LOW so high-trigger relay stays off (HIGH energizes coil)."""
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SOLENOID_PIN, GPIO.OUT, initial=SOLENOID_DEACTIVATE_LEVEL)


def activate_solenoid():
    """Activate solenoid to drop egg (솔레노이드로 계란 사출)."""
    try:
        GPIO.setup(SOLENOID_PIN, GPIO.OUT, initial=SOLENOID_DEACTIVATE_LEVEL)
        for i in range(SOLENOID_REPEAT):
            GPIO.output(SOLENOID_PIN, SOLENOID_ACTIVATE_LEVEL)
            time.sleep(SOLENOID_DURATION)
            GPIO.output(SOLENOID_PIN, SOLENOID_DEACTIVATE_LEVEL)
            time.sleep(SOLENOID_DURATION)
            print(f"솔레노이드 작동 완료 (반복: {i+1}/{SOLENOID_REPEAT})")
    except Exception as e:
        pass

def terminate_solenoid():
    """Terminate solenoid (cleanup GPIO)."""
    try:
        GPIO.cleanup(SOLENOID_PIN)
    except Exception as e:
        pass

if __name__ == "__main__":
    init_solenoid()
    try:
        print("\n=== Solenoid Test (Egg Drop) ===")
        input("Enter를 눌러 솔레노이드 작동")
        activate_solenoid()
        print("완료. 종료합니다.")

    except KeyboardInterrupt:
        print("\n프로그램 종료 요청 (Ctrl+C)")

    finally:
        terminate_solenoid()
