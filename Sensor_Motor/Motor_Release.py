"""Burnwire (parafoil release) GPIO control with safe fallback.

Hardware path  : RPi.GPIO -> BURNWIRE_GPIO pin
Fallback path  : _DummyGPIO (no hardware required)

Safety contract:
  - activate_burnwire() uses try/finally to guarantee RELAY_DEACTIVATE_LEVEL
    is written even if an exception occurs mid-burn.
  - terminate_burnwire() is registered with atexit and is idempotent.
"""

from __future__ import annotations

import atexit
import logging
import os
import time

from lib import config

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Dummy GPIO backend (PC / CI fallback)
# ---------------------------------------------------------------------------

class _DummyGPIO:
    BCM = "BCM"
    OUT = "OUT"
    HIGH = 1
    LOW  = 0

    def __init__(self) -> None:
        self.state: dict[int, int] = {}

    def setmode(self, _mode) -> None:
        return

    def setup(self, pin: int, _mode, initial: int = 0) -> None:
        self.state[pin] = initial

    def output(self, pin: int, val: int) -> None:
        self.state[pin] = val

    def cleanup(self, pin=None) -> None:
        if pin is None:
            self.state.clear()
        else:
            self.state.pop(pin, None)


# ---------------------------------------------------------------------------
# Module-level state
# ---------------------------------------------------------------------------

GPIO = None
BURNWIRE_READY: bool = False
BURNWIRE_GPIO:  int  = config.BURNWIRE_GPIO
BURNWIRE_DURATION_SEC: float = float(os.environ.get("BURNWIRE_DURATION_SEC", "5.0"))


# ---------------------------------------------------------------------------
# Internal
# ---------------------------------------------------------------------------

def _load_gpio():
    global GPIO
    if GPIO is not None:
        return GPIO
    try:
        import RPi.GPIO as real_gpio  # type: ignore
        GPIO = real_gpio
        logger.info("Motor_Release: using real RPi.GPIO")
    except Exception:
        GPIO = _DummyGPIO()
        logger.info("Motor_Release: RPi.GPIO unavailable, using dummy backend")
    return GPIO


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def init_burnwire() -> None:
    """Configure GPIO pin and register cleanup handler."""
    global BURNWIRE_READY
    gpio = _load_gpio()
    gpio.setmode(gpio.BCM)
    gpio.setup(BURNWIRE_GPIO, gpio.OUT, initial=config.RELAY_DEACTIVATE_LEVEL)
    BURNWIRE_READY = True
    atexit.register(terminate_burnwire)
    logger.debug("Burnwire init: GPIO %d, deactivate_level=%d", BURNWIRE_GPIO, config.RELAY_DEACTIVATE_LEVEL)


def activate_burnwire() -> None:
    """Fire the burnwire for BURNWIRE_DURATION_SEC seconds.

    Guarantees relay is returned to RELAY_DEACTIVATE_LEVEL via try/finally.
    Safe to call without prior init_burnwire() — will auto-initialise.
    """
    if not BURNWIRE_READY:
        init_burnwire()
    gpio = _load_gpio()
    logger.info("Burnwire ACTIVATE for %.2f s", BURNWIRE_DURATION_SEC)
    try:
        gpio.output(BURNWIRE_GPIO, config.RELAY_ACTIVATE_LEVEL)
        time.sleep(BURNWIRE_DURATION_SEC)
    finally:
        gpio.output(BURNWIRE_GPIO, config.RELAY_DEACTIVATE_LEVEL)
        logger.info("Burnwire DEACTIVATE (relay safe)")


def terminate_burnwire() -> None:
    """Deactivate relay and release GPIO resources. Idempotent."""
    global BURNWIRE_READY
    if GPIO is None:
        return
    try:
        GPIO.output(BURNWIRE_GPIO, config.RELAY_DEACTIVATE_LEVEL)
        GPIO.cleanup(BURNWIRE_GPIO)
    except Exception as exc:
        logger.debug("Burnwire terminate error (safe to ignore): %s", exc)
    BURNWIRE_READY = False
