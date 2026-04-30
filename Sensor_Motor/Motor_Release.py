"""Burnwire (release) GPIO control with safe fallback."""

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
BURNWIRE_READY = False
BURNWIRE_GPIO = config.BURNWIRE_GPIO
BURNWIRE_DURATION_SEC = float(os.environ.get("BURNWIRE_DURATION_SEC", "5.0"))


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


def init_burnwire() -> None:
    global BURNWIRE_READY
    gpio = _load_gpio()
    gpio.setmode(gpio.BCM)
    gpio.setup(BURNWIRE_GPIO, gpio.OUT, initial=config.RELAY_DEACTIVATE_LEVEL)
    BURNWIRE_READY = True
    atexit.register(terminate_burnwire)


def activate_burnwire() -> None:
    if not BURNWIRE_READY:
        init_burnwire()
    gpio = _load_gpio()
    gpio.output(BURNWIRE_GPIO, config.RELAY_ACTIVATE_LEVEL)
    time.sleep(BURNWIRE_DURATION_SEC)
    gpio.output(BURNWIRE_GPIO, config.RELAY_DEACTIVATE_LEVEL)


def terminate_burnwire() -> None:
    global BURNWIRE_READY
    if GPIO is None:
        return
    try:
        GPIO.output(BURNWIRE_GPIO, config.RELAY_DEACTIVATE_LEVEL)
        GPIO.cleanup(BURNWIRE_GPIO)
    except Exception:
        pass
    BURNWIRE_READY = False
