#!/usr/bin/env python3
"""
Burnwire control for Container-Payload release
BCM 5: Burnwire (번와이어로 페이로드가 컨테이너에서 탈출)
에그 릴레이와 동일 보드 — 레벨은 relay_levels.py 한 곳에서만 정의 (HIGH=작동, LOW=대기).
"""

import time
import RPi.GPIO as GPIO

from Sensor_Motor.relay_levels import RELAY_ACTIVATE_LEVEL, RELAY_DEACTIVATE_LEVEL

BURNWIRE_PIN = 5
BURNWIRE_ACTIVATE_LEVEL = RELAY_ACTIVATE_LEVEL
BURNWIRE_DEACTIVATE_LEVEL = RELAY_DEACTIVATE_LEVEL
BURNWIRE_DURATION = 3.0  # 번와이어 작동 시간 (초)

def init_burnwire():
    """Hold relay OFF: high-trigger module idle at LOW; OUTPUT avoids floating."""
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BURNWIRE_PIN, GPIO.OUT, initial=BURNWIRE_DEACTIVATE_LEVEL)

def activate_burnwire():
    """Activate burnwire to release payload from container (번와이어로 컨테이너-페이로드 사출)."""
    try:
        GPIO.setup(BURNWIRE_PIN, GPIO.OUT, initial=BURNWIRE_DEACTIVATE_LEVEL)
        GPIO.output(BURNWIRE_PIN, BURNWIRE_ACTIVATE_LEVEL)        
        time.sleep(BURNWIRE_DURATION) 
        GPIO.output(BURNWIRE_PIN, BURNWIRE_DEACTIVATE_LEVEL)
    except Exception as e:
        pass

def terminate_burnwire():
    """Terminate burnwire (cleanup GPIO)."""
    try:
        GPIO.cleanup(BURNWIRE_PIN)
    except Exception as e:
        pass

if __name__ == "__main__":
    init_burnwire()
    try:
        print("\n=== Burnwire Test (Container-Payload Release) ===")
        input("Enter를 눌러 번와이어 작동 (3초)")
        activate_burnwire()
        print("번와이어 작동 완료")
        
        input("Enter를 눌러 종료")

    except KeyboardInterrupt:
        print("\n프로그램 종료 요청 (Ctrl+C)")

    finally:
        terminate_burnwire()
