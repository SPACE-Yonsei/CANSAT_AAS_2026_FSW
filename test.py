import RPi.GPIO as GPIO
import time

# 제어할 BCM 핀 번호 설정 (GPIO 6)
RELAY_PIN = 6

# 불필요한 GPIO 경고 메시지 숨기기
GPIO.setwarnings(False)

# 핀 번호 참조 방식을 BCM 모드로 설정
GPIO.setmode(GPIO.BCM)

# 6번 핀을 출력(OUTPUT) 모드로 설정
GPIO.setup(RELAY_PIN, GPIO.OUT)

try:
    print("Raspberry Pi Zero 2 W 릴레이 제어 시작 (Ctrl+C로 종료)")
    while True:
        print("릴레이 ON")
        # 핀에 HIGH(1) 신호 보내기
        GPIO.output(RELAY_PIN, GPIO.HIGH) 
        time.sleep(2)  # 2초 대기

        print("릴레이 OFF")
        # 핀에 LOW(0) 신호 보내기
        GPIO.output(RELAY_PIN, GPIO.LOW)
        time.sleep(2)  # 2초 대기

except KeyboardInterrupt:
    # 사용자가 터미널에서 Ctrl+C를 누르면 실행됨
    print("\n프로그램을 안전하게 종료합니다.")

finally:
    # 프로그램 종료 시 GPIO 핀 상태 초기화 (매우 중요)
    # 릴레이가 켜진 채로 프로그램이 종료되는 것을 방지합니다.
    GPIO.cleanup()
