"""Egg-drop solenoid GPIO control with safe fallback."""

from __future__ import annotations

import atexit
import os
import time

from lib import config


class _DummyGPIO:
    BCM = "BCM"
    OUT = "OUT"
    HIGH = 1
    LOW = 0

    def __init__(self) -> None:
        self.state = {}

    def setmode(self, _mode) -> None:
        return

    def setup(self, pin, _mode, initial=0) -> None:
        self.state[pin] = initial

    def output(self, pin, val) -> None:
        self.state[pin] = val

    def cleanup(self, pin=None) -> None:
        if pin is None:
            self.state.clear()
        else:
            self.state.pop(pin, None)


GPIO = None
SOLENOID_READY = False
SOLENOID_GPIO = config.EGG_SOLENOID_GPIO
SOLENOID_REPEAT = int(os.environ.get("SOLENOID_REPEAT", "3"))
SOLENOID_ON_SEC = float(os.environ.get("SOLENOID_ON_SEC", "0.5"))
SOLENOID_OFF_SEC = float(os.environ.get("SOLENOID_OFF_SEC", "0.5"))


def _load_gpio():
    global GPIO
    if GPIO is not None:
        return GPIO
    try:
        import RPi.GPIO as real_gpio  # type: ignore

        GPIO = real_gpio
    except Exception:
        GPIO = _DummyGPIO()
    return GPIO


def init_solenoid() -> None:
    global SOLENOID_READY
    gpio = _load_gpio()
    gpio.setmode(gpio.BCM)
    gpio.setup(SOLENOID_GPIO, gpio.OUT, initial=config.RELAY_DEACTIVATE_LEVEL)
    SOLENOID_READY = True
    atexit.register(terminate_solenoid)


def activate_solenoid() -> None:
    if not SOLENOID_READY:
        init_solenoid()
    gpio = _load_gpio()
    for _ in range(max(1, SOLENOID_REPEAT)):
        gpio.output(SOLENOID_GPIO, config.RELAY_ACTIVATE_LEVEL)
        time.sleep(SOLENOID_ON_SEC)
        gpio.output(SOLENOID_GPIO, config.RELAY_DEACTIVATE_LEVEL)
        time.sleep(SOLENOID_OFF_SEC)


def terminate_solenoid() -> None:
    global SOLENOID_READY
    if GPIO is None:
        return
    try:
        GPIO.output(SOLENOID_GPIO, config.RELAY_DEACTIVATE_LEVEL)
        GPIO.cleanup(SOLENOID_GPIO)
    except Exception:
        pass
    SOLENOID_READY = False
