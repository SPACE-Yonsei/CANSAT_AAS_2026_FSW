"""UART I/O wrapper for ground-station command and telemetry."""

from __future__ import annotations

from dataclasses import dataclass
import logging
import os
from typing import Optional


logger = logging.getLogger(__name__)


@dataclass
class DummySerial:
    is_open: bool = True
    _rx: list[bytes] = None
    _tx: list[bytes] = None

    def __post_init__(self) -> None:
        if self._rx is None:
            self._rx = []
        if self._tx is None:
            self._tx = []

    def write(self, data: bytes) -> None:
        self._tx.append(data)

    def readline(self) -> bytes:
        if self._rx:
            return self._rx.pop(0)
        return b""

    def close(self) -> None:
        self.is_open = False


def _port_candidates(explicit: str | None) -> list[str]:
    env = os.environ.get("UART_DEVICE", "").strip()
    parts = [p.strip() for p in env.split(",") if p.strip()] if env else []
    defaults = [
        (explicit or "").strip(),
        "/dev/serial0",
        "/dev/ttyAMA0",
        "/dev/ttyS0",
    ]
    out: list[str] = []
    for p in parts + defaults:
        if p and p not in out:
            out.append(p)
    return out


def init_serial(port: str | None = None, baudrate: int = 9600):
    """Initialize serial transport.

    Tries multiple common Raspberry Pi UART device paths unless `UART_DEVICE`
    is set (comma-separated list, highest priority).

    If pyserial is not installed or no port could be opened, returns a dummy
    serial object to keep non-hardware tests runnable.
    """
    try:
        import serial  # type: ignore
    except Exception as exc:
        logger.warning("pyserial unavailable; using DummySerial (%s)", exc)
        return DummySerial()

    candidates = _port_candidates(port or "/dev/serial0")
    last_exc: Exception | None = None
    for cand in candidates:
        try:
            ser = serial.Serial(cand, baudrate, timeout=1)
            logger.info("UART opened %s @ %s", cand, baudrate)
            return ser
        except Exception as exc:
            last_exc = exc
            logger.warning("UART open failed for %s: %s", cand, exc)

    logger.warning("Falling back to DummySerial (last error: %s)", last_exc)
    return DummySerial()


def is_dummy_serial(ser) -> bool:
    return isinstance(ser, DummySerial)


def send_serial_data(ser, string_to_write: str) -> bool:
    if ser is None or not getattr(ser, "is_open", False):
        return False
    try:
        ser.write(string_to_write.encode("utf-8"))
        return True
    except Exception as exc:
        logger.error("send_serial_data failed: %s", exc)
        return False


def receive_serial_data(ser) -> Optional[str]:
    if ser is None or not getattr(ser, "is_open", False):
        return None
    try:
        raw = ser.readline()
        if not raw:
            return None
        return raw.decode("utf-8", errors="ignore").strip()
    except (TypeError, ValueError):
        # Some serial backends may transiently surface invalid timeout state.
        # Treat as "no data" instead of noisy error during shutdown.
        return None
    except Exception as exc:
        logger.error("receive_serial_data failed: %s", exc)
        return None


def terminate_serial(ser) -> None:
    if ser is not None and getattr(ser, "is_open", False):
        ser.close()
