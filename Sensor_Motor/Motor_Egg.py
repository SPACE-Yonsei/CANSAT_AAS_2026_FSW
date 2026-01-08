#!/usr/bin/env python3
"""
Solenoid control for Payload-Egg drop
GPIO 5: Solenoid (솔레노이드로 계란 사출)
"""

import time
import RPi.GPIO as GPIO 

SOLENOID_PIN = 5
SOLENOID_ACTIVATE_LEVEL = GPIO.HIGH
SOLENOID_DEACTIVATE_LEVEL = GPIO.LOW
SOLENOID_DURATION = 0.5  # 솔레노이드 작동 시간 (초)

def init_solenoid():
    """Initialize solenoid for egg drop (GPIO setup)."""
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SOLENOID_PIN, GPIO.OUT, initial=SOLENOID_DEACTIVATE_LEVEL)

def activate_solenoid():
    """Activate solenoid to drop egg (솔레노이드로 계란 사출)."""
    try:
        GPIO.output(SOLENOID_PIN, SOLENOID_ACTIVATE_LEVEL)        
        time.sleep(SOLENOID_DURATION) 
        GPIO.output(SOLENOID_PIN, SOLENOID_DEACTIVATE_LEVEL)
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
        input("Enter를 눌러 솔레노이드 작동 (0.5초)")
        activate_solenoid()
        print("솔레노이드 작동 완료")
        
        input("Enter를 눌러 종료")

    except KeyboardInterrupt:
        print("\n프로그램 종료 요청 (Ctrl+C)")

    finally:
        terminate_solenoid()
