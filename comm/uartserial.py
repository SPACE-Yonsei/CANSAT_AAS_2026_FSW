"""UART I/O wrapper for ground-station command and telemetry."""

from __future__ import annotations

from dataclasses import dataclass
import logging
import os
import sys
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


def _discover_serial_ports() -> list[str]:
    """Best-effort probe of available serial devices via pyserial tools."""
    try:
        from serial.tools import list_ports  # type: ignore
    except Exception:
        return []

    discovered: list[str] = []
    try:
        for entry in list_ports.comports():
            dev = getattr(entry, "device", "")
            dev = dev.strip() if isinstance(dev, str) else ""
            if dev:
                discovered.append(dev)
    except Exception:
        return []
    return discovered


def _port_candidates(explicit: str | None) -> list[str]:
    env = os.environ.get("UART_DEVICE", "").strip()
    parts = [p.strip() for p in env.split(",") if p.strip()] if env else []
    discovered = _discover_serial_ports()
    explicit_port = (explicit or "").strip()
    if sys.platform.startswith("win"):
        defaults = [f"COM{i}" for i in range(1, 33)]
    else:
        defaults = ["/dev/serial0", "/dev/ttyAMA0", "/dev/ttyAMA10", "/dev/ttyS0", "/dev/ttyUSB0", "/dev/ttyACM0"]

    out: list[str] = []
    for p in parts + [explicit_port] + discovered + defaults:
        if p and p not in out:
            out.append(p)
    return out


def init_serial(port: str | None = None, baudrate: int | None = None):
    """Initialize serial transport.

    Tries multiple common Raspberry Pi UART device paths unless `UART_DEVICE`
    is set (comma-separated list, highest priority).

    Baud rate: pass `baudrate` or set env `UART_BAUD` (default 9600). Match
    the XBee module XCTU "Interface Data Rate" (e.g. BD=5 for 38400). Lower rates
    (9600) saturate the XBee internal RX buffer at our ~430-byte TLM frames
    and produce dropped/merged lines on the GS; higher rates have no effect
    on RF range — they only speed up the host↔XBee chip serial link.

    If pyserial is not installed or no port could be opened, returns a dummy
    serial object to keep non-hardware tests runnable.
    """
    try:
        import serial  # type: ignore
    except Exception as exc:
        logger.warning("pyserial unavailable; using DummySerial (%s)", exc)
        return DummySerial()

    if baudrate is None:
        try:
            baudrate = int(os.environ.get("UART_BAUD", "38400"))
        except ValueError:
            baudrate = 38400

    default_port = "COM3" if sys.platform.startswith("win") else "/dev/serial0"
    candidates = _port_candidates(port or default_port)
    last_exc: Exception | None = None
    for cand in candidates:
        try:
            ser = serial.Serial(cand, baudrate, timeout=1)
            logger.info("UART opened %s @ %s", cand, baudrate)
            return ser
        except Exception as exc:
            last_exc = exc
            logger.warning("UART open failed for %s: %s", cand, exc)

    logger.warning(
        "Falling back to DummySerial after trying %s (last error: %s)",
        ", ".join(candidates),
        last_exc,
    )
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
