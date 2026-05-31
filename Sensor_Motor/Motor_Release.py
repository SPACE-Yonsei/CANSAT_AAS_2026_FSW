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
import sys
import time
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

from lib import config
from Sensor_Motor.Motor_Release_Cal import get_burnwire_delay_sec

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
_RELEASE_ACTIVATE_LEVEL = int(
    getattr(config, "RELEASE_RELAY_ACTIVATE_LEVEL", config.RELAY_ACTIVATE_LEVEL)
)
_RELEASE_DEACTIVATE_LEVEL = int(
    getattr(config, "RELEASE_RELAY_DEACTIVATE_LEVEL", config.RELAY_DEACTIVATE_LEVEL)
)
BURNWIRE_DURATION_SEC: float = float(
    os.environ.get("BURNWIRE_DURATION_SEC", str(get_burnwire_delay_sec()))
)


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
    gpio.setup(BURNWIRE_GPIO, gpio.OUT, initial=_RELEASE_DEACTIVATE_LEVEL)
    BURNWIRE_READY = True
    atexit.register(terminate_burnwire)
    logger.debug(
        "Burnwire init: GPIO %d, deactivate_level=%d (release polarity swapped)",
        BURNWIRE_GPIO,
        _RELEASE_DEACTIVATE_LEVEL,
    )


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
        gpio.output(BURNWIRE_GPIO, _RELEASE_ACTIVATE_LEVEL)
        time.sleep(BURNWIRE_DURATION_SEC)
    finally:
        gpio.output(BURNWIRE_GPIO, _RELEASE_DEACTIVATE_LEVEL)
        logger.info("Burnwire DEACTIVATE (relay safe)")


def set_burnwire(active: bool) -> None:
    """GPIO를 즉시 ON/OFF. 타이머 없이 호출한 쪽이 직접 제어."""
    if not BURNWIRE_READY:
        init_burnwire()
    gpio = _load_gpio()
    level = _RELEASE_ACTIVATE_LEVEL if active else _RELEASE_DEACTIVATE_LEVEL
    gpio.output(BURNWIRE_GPIO, level)
    logger.info("Burnwire SET %s (GPIO %d = %d)", "ON" if active else "OFF", BURNWIRE_GPIO, level)


def terminate_burnwire() -> None:
    """Deactivate relay and release GPIO resources. Idempotent."""
    global BURNWIRE_READY
    if GPIO is None:
        return
    try:
        GPIO.output(BURNWIRE_GPIO, _RELEASE_DEACTIVATE_LEVEL)
        GPIO.cleanup(BURNWIRE_GPIO)
    except Exception as exc:
        logger.debug("Burnwire terminate error (safe to ignore): %s", exc)
    BURNWIRE_READY = False



if __name__ == "__main__":
    init_burnwire()
    try:
        print("번와이어 작동 시작")
        activate_burnwire()
        print("번와이어 작동 종료")

    except KeyboardInterrupt:
        print("\n프로그램 종료 요청 (Ctrl+C)")

    finally:
        terminate_burnwire()
