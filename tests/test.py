"""Manual Raspberry Pi relay smoke test.

This file is intentionally skipped during normal unittest discovery unless it is
run directly on a Raspberry Pi with RPi.GPIO installed.
"""

from __future__ import annotations

import time
import unittest

try:
    import RPi.GPIO as GPIO
except ModuleNotFoundError as exc:
    raise unittest.SkipTest("RPi.GPIO is only available on Raspberry Pi hardware") from exc


RELAY_PIN = 5


def run_relay_loop() -> None:
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(RELAY_PIN, GPIO.OUT)
    try:
        print("Raspberry Pi relay smoke test started. Press Ctrl+C to stop.")
        while True:
            print("Relay ON")
            GPIO.output(RELAY_PIN, GPIO.HIGH)
            time.sleep(2)

            print("Relay OFF")
            GPIO.output(RELAY_PIN, GPIO.LOW)
            time.sleep(2)
    finally:
        GPIO.cleanup()


if __name__ == "__main__":
    run_relay_loop()
