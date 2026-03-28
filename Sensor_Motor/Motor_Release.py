#!/usr/bin/env python3
"""
Burnwire control for Container-Payload release
BCM 5: Burnwire (번와이어로 페이로드가 컨테이너에서 탈출)
일반적인 저가 릴레이 모듈: IN이 LOW일 때 릴레이 코일 ON (액티브 로우).
"""

import time
import RPi.GPIO as GPIO 

BURNWIRE_PIN = 5
BURNWIRE_ACTIVATE_LEVEL = GPIO.LOW
BURNWIRE_DEACTIVATE_LEVEL = GPIO.HIGH
BURNWIRE_DURATION = 3.0  # 번와이어 작동 시간 (초)

def init_burnwire():
    """Hold relay OFF: active-low modules need IN=HIGH idle; OUTPUT avoids floating."""
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
