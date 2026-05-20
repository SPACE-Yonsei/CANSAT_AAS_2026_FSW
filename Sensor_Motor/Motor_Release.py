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


# ---------------------------------------------------------------------------
# Standalone test entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import argparse
    import sys

    logging.basicConfig(level=logging.DEBUG, format="%(levelname)-8s %(message)s")

    parser = argparse.ArgumentParser(
        description="activate_burnwire() 단독 검사 도구",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
예시:
  python -m Sensor_Motor.Motor_Release              # 기본 설정으로 실행
  python -m Sensor_Motor.Motor_Release --duration 1 # 1초 버닝
  python -m Sensor_Motor.Motor_Release --dry-run    # 초기화만, 실제 동작 없음

환경변수:
  BURNWIRE_DURATION_SEC=2.0  번 지속시간 덮어쓰기 (--duration보다 낮은 우선순위)
""",
    )
    parser.add_argument(
        "--duration", type=float, default=None,
        help="번 지속시간(초). 미설정 시 config 값 사용",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="GPIO 초기화 및 설정 확인만 수행, activate 호출 없음",
    )
    args = parser.parse_args()

    if args.duration is not None:
        BURNWIRE_DURATION_SEC = args.duration

    # ── 설정 출력 ──────────────────────────────────────────────────────────
    print("=" * 52)
    print("  Motor_Release  burnwire 검사")
    print("=" * 52)
    print(f"  GPIO 핀           : {BURNWIRE_GPIO}")
    print(f"  ACTIVATE 레벨     : {_RELEASE_ACTIVATE_LEVEL}  "
          f"({'LOW=0=relay ON' if _RELEASE_ACTIVATE_LEVEL == 0 else 'HIGH=1=relay ON'})")
    print(f"  DEACTIVATE 레벨   : {_RELEASE_DEACTIVATE_LEVEL}")
    print(f"  번 지속시간       : {BURNWIRE_DURATION_SEC:.2f} s")
    print(f"  모드              : {'DRY-RUN (activate 생략)' if args.dry_run else 'LIVE'}")
    print("=" * 52)

    # ── GPIO 초기화 + 상태 추적 ────────────────────────────────────────────
    init_burnwire()
    gpio = _load_gpio()
    backend = "RPi.GPIO (실하드웨어)" if not isinstance(gpio, _DummyGPIO) else "_DummyGPIO (시뮬)"
    print(f"  백엔드            : {backend}")

    def _pin_state_str() -> str:
        if isinstance(gpio, _DummyGPIO):
            level = gpio.state.get(BURNWIRE_GPIO, "?")
            label = "DEACTIVATE(safe)" if level == _RELEASE_DEACTIVATE_LEVEL else "ACTIVATE(LIVE!)"
            return f"GPIO[{BURNWIRE_GPIO}]={level} → {label}"
        return "(실 하드웨어: 멀티미터로 확인)"

    print(f"  초기 상태         : {_pin_state_str()}")
    print()

    if args.dry_run:
        print("[DRY-RUN] activate_burnwire() 호출 생략. 안전 상태 유지.")
        terminate_burnwire()
        sys.exit(0)

    # ── 실행 ───────────────────────────────────────────────────────────────
    print(f"[LIVE] {BURNWIRE_DURATION_SEC:.2f}초 후 자동 해제됩니다. Ctrl+C 로 중단 가능.")
    print("       (중단해도 finally 블록이 DEACTIVATE 보장)")
    print()
    try:
        activate_burnwire()
    except KeyboardInterrupt:
        print("\n[중단됨] KeyboardInterrupt — finally 가 relay를 안전하게 해제했는지 확인:")
    finally:
        print(f"  최종 상태         : {_pin_state_str()}")
        safe = (
            isinstance(gpio, _DummyGPIO)
            and gpio.state.get(BURNWIRE_GPIO) == _RELEASE_DEACTIVATE_LEVEL
        )
        if isinstance(gpio, _DummyGPIO):
            result = "PASS  relay = DEACTIVATE" if safe else "FAIL  relay = ACTIVATE (위험)"
            print(f"  안전 검증         : {result}")
        terminate_burnwire()
        print("  종료 완료")


