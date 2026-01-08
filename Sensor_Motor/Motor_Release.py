#!/usr/bin/env python3
"""
Solenoid control for egg drop safety
GPIO 5: Solenoid pin
"""

import time
import RPi.GPIO as GPIO 

SOLENOID_PIN = 5

SOLENOID_ACTIVATE_LEVEL = GPIO.HIGH
SOLENOID_DEACTIVATE_LEVEL = GPIO.LOW

def init_solenoid():
    """Initialize solenoid (GPIO setup)."""
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SOLENOID_PIN, GPIO.OUT, initial=SOLENOID_DEACTIVATE_LEVEL)

def unlock_solenoid():
    """Unlock solenoid (activate for 0.5 seconds then deactivate)."""
    try:
        GPIO.output(SOLENOID_PIN, SOLENOID_ACTIVATE_LEVEL)        
        time.sleep(0.5) 
        GPIO.output(SOLENOID_PIN, SOLENOID_DEACTIVATE_LEVEL)
        
    except Exception as e:
        pass

def terminate_solenoid():
    """Terminate solenoid (cleanup GPIO)."""
    GPIO.cleanup()

if __name__ == "__main__":
    init_solenoid()
    try:
        while True:

            print("\n--- 1. 잠금 해제 상태 ---")
            unlock_solenoid()
            input("Enter를 눌러 순간 잠금 해제 상태로 전환")
            
            print("\n--- 2. 순간 잠금 해제 (100ms) 상태 ---")
            unlock_solenoid()
            input("Enter를 눌러 종료")
            break

    except KeyboardInterrupt:
        print("\n프로그램 종료 요청 (Ctrl+C)")

    finally:
        terminate_solenoid()