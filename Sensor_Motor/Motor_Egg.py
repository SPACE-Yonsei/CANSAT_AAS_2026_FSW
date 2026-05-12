"""Egg-drop solenoid GPIO control with safe fallback.

Hardware path  : RPi.GPIO -> EGG_SOLENOID_GPIO pin
Fallback path  : _DummyGPIO (no hardware required)

Safety contract:
  - activate_solenoid() uses try/finally to guarantee RELAY_DEACTIVATE_LEVEL
    is written after each pulse even if an exception occurs.
  - terminate_solenoid() is registered with atexit and is idempotent.
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
SOLENOID_READY: bool = False
SOLENOID_GPIO:  int  = config.EGG_SOLENOID_GPIO
_EGG_ACTIVATE_LEVEL = int(
    getattr(config, "EGG_RELAY_ACTIVATE_LEVEL", config.RELAY_ACTIVATE_LEVEL)
)
_EGG_DEACTIVATE_LEVEL = int(
    getattr(config, "EGG_RELAY_DEACTIVATE_LEVEL", config.RELAY_DEACTIVATE_LEVEL)
)
SOLENOID_REPEAT:  int   = int(os.environ.get("SOLENOID_REPEAT",  "3"))
SOLENOID_ON_SEC:  float = float(os.environ.get("SOLENOID_ON_SEC",  "0.5"))
SOLENOID_OFF_SEC: float = float(os.environ.get("SOLENOID_OFF_SEC", "0.5"))


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
        logger.info("Motor_Egg: using real RPi.GPIO")
    except Exception:
        GPIO = _DummyGPIO()
        logger.info("Motor_Egg: RPi.GPIO unavailable, using dummy backend")
    return GPIO


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def init_solenoid() -> None:
    """Configure GPIO pin and register cleanup handler."""
    global SOLENOID_READY
    gpio = _load_gpio()
    gpio.setmode(gpio.BCM)
    gpio.setup(SOLENOID_GPIO, gpio.OUT, initial=_EGG_DEACTIVATE_LEVEL)
    SOLENOID_READY = True
    atexit.register(terminate_solenoid)
    logger.debug("Solenoid init: GPIO %d, deactivate_level=%d", SOLENOID_GPIO, _EGG_DEACTIVATE_LEVEL)


def activate_solenoid() -> None:
    """Fire the solenoid SOLENOID_REPEAT times.

    Each pulse: ACTIVATE for SOLENOID_ON_SEC, then DEACTIVATE for SOLENOID_OFF_SEC.
    try/finally guarantees relay ends at RELAY_DEACTIVATE_LEVEL even on exception.
    Safe to call without prior init_solenoid() — will auto-initialise.
    """
    if not SOLENOID_READY:
        init_solenoid()
    gpio = _load_gpio()
    repeat = max(1, SOLENOID_REPEAT)
    logger.info("Solenoid ACTIVATE: %d pulses (on=%.2fs off=%.2fs)", repeat, SOLENOID_ON_SEC, SOLENOID_OFF_SEC)
    for i in range(repeat):
        try:
            gpio.output(SOLENOID_GPIO, _EGG_ACTIVATE_LEVEL)
            time.sleep(SOLENOID_ON_SEC)
        finally:
            gpio.output(SOLENOID_GPIO, _EGG_DEACTIVATE_LEVEL)
        if i < repeat - 1:
            time.sleep(SOLENOID_OFF_SEC)
    logger.info("Solenoid DEACTIVATE (relay safe)")


def terminate_solenoid() -> None:
    """Deactivate relay and release GPIO resources. Idempotent."""
    global SOLENOID_READY
    if GPIO is None:
        return
    try:
        GPIO.output(SOLENOID_GPIO, _EGG_DEACTIVATE_LEVEL)
        GPIO.cleanup(SOLENOID_GPIO)
    except Exception as exc:
        logger.debug("Solenoid terminate error (safe to ignore): %s", exc)
    SOLENOID_READY = False

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG, format="%(levelname)s  %(message)s")

    print("\n=== Solenoid Test (Egg Drop) ===")
    print(f"  GPIO pin     : {SOLENOID_GPIO}")
    print(f"  Pulses       : {SOLENOID_REPEAT}")
    print(f"  ON duration  : {SOLENOID_ON_SEC:.2f} s")
    print(f"  OFF interval : {SOLENOID_OFF_SEC:.2f} s")
    print(f"  Activate lvl : {_EGG_ACTIVATE_LEVEL}  /  Deactivate lvl : {_EGG_DEACTIVATE_LEVEL}")

    try:
        init_solenoid()
        print(f"  Ready        : {SOLENOID_READY}")

        input("\nEnter를 누르면 솔레노이드를 작동합니다 (Ctrl+C 취소)... ")
        activate_solenoid()
        print(f"  Ready (post) : {SOLENOID_READY}")
        print("완료. 종료합니다.")

    except KeyboardInterrupt:
        print("\n프로그램 종료 요청 (Ctrl+C)")

    finally:
        terminate_solenoid()
        print(f"  Ready (종료) : {SOLENOID_READY}")