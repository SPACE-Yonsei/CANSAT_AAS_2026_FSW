"""UART I/O wrapper for ground-station command and telemetry."""

from __future__ import annotations

from dataclasses import dataclass
import logging
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


def init_serial(port: str = "/dev/serial0", baudrate: int = 9600):
    """Initialize serial transport.

    If pyserial is not installed, returns a dummy serial object to keep
    non-hardware tests runnable.
    """
    try:
        import serial  # type: ignore

        return serial.Serial(port, baudrate, timeout=1)
    except Exception as exc:
        logger.warning("Falling back to DummySerial: %s", exc)
        return DummySerial()


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
