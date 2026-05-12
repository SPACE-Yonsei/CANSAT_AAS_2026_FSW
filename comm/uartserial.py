"""UART I/O wrapper for ground-station command and telemetry."""

from __future__ import annotations

from dataclasses import dataclass
import logging
import os
import sys
import time
from typing import Optional

from lib.i2c_bus import i2c_lock


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


class SC16IS750Serial:
    """Minimal line-oriented transport over SC16IS750 (I2C bridge)."""

    # Register addresses (chip register index, not I2C command byte).
    _REG_RHR = 0x00
    _REG_THR = 0x00
    _REG_IER = 0x01
    _REG_FCR = 0x02
    _REG_LCR = 0x03
    _REG_LSR = 0x05
    _REG_DLL = 0x00
    _REG_DLH = 0x01
    _REG_RXLVL = 0x09

    # LCR bit to access divisor latch registers.
    _LCR_DIVISOR_LATCH = 0x80

    # LSR bits.
    _LSR_DATA_READY = 0x01
    _LSR_THR_EMPTY = 0x20

    def __init__(self, bus_id: int, addr: int, baudrate: int, timeout: float = 1.0) -> None:
        from smbus2 import SMBus  # type: ignore

        self._bus = SMBus(bus_id)
        self._addr = addr
        self._buf = bytearray()
        self.timeout = max(0.01, float(timeout))
        self.baudrate = int(baudrate)
        self.port = f"i2c-{bus_id}@0x{addr:02X}"
        self.is_open = True
        self._configure_uart()

    @staticmethod
    def _cmd(reg: int) -> int:
        # SC16IS750 expects register index on [7:3] of command byte.
        return (reg & 0x0F) << 3

    def _read_reg(self, reg: int) -> int:
        with i2c_lock():
            return int(self._bus.read_byte_data(self._addr, self._cmd(reg))) & 0xFF

    def _write_reg(self, reg: int, value: int) -> None:
        with i2c_lock():
            self._bus.write_byte_data(self._addr, self._cmd(reg), int(value) & 0xFF)

    def _configure_uart(self) -> None:
        # Default crystal for SparkFun Explorer I2C is 14.7456 MHz.
        crystal_hz = int(os.environ.get("SC16IS750_CRYSTAL_HZ", "14745600"))
        divisor = max(1, int(round(crystal_hz / (16.0 * float(self.baudrate)))))
        dll = divisor & 0xFF
        dlh = (divisor >> 8) & 0xFF

        # Disable UART interrupts and clear FIFOs during setup.
        self._write_reg(self._REG_IER, 0x00)
        self._write_reg(self._REG_LCR, self._LCR_DIVISOR_LATCH)
        self._write_reg(self._REG_DLL, dll)
        self._write_reg(self._REG_DLH, dlh)
        self._write_reg(self._REG_LCR, 0x03)  # 8 data bits, no parity, 1 stop bit.
        self._write_reg(self._REG_FCR, 0x07)  # FIFO enable + RX/TX reset.

    def write(self, data: bytes) -> None:
        if not self.is_open:
            raise OSError("SC16IS750Serial is closed")
        if not data:
            return
        # Keep SMBus block transfers small for broad adapter compatibility.
        chunk_size = 16
        for idx in range(0, len(data), chunk_size):
            chunk = data[idx : idx + chunk_size]
            # Wait for THR to be ready, then push one chunk.
            deadline = time.time() + self.timeout
            while True:
                lsr = self._read_reg(self._REG_LSR)
                if lsr & self._LSR_THR_EMPTY:
                    break
                if time.time() >= deadline:
                    raise TimeoutError("SC16IS750 THR not ready")
                time.sleep(0.002)
            with i2c_lock():
                self._bus.write_i2c_block_data(self._addr, self._cmd(self._REG_THR), list(chunk))

    def _read_available(self) -> None:
        if not self.is_open:
            return
        lvl = self._read_reg(self._REG_RXLVL)
        if lvl <= 0:
            return
        with i2c_lock():
            for _ in range(lvl):
                b = self._bus.read_byte_data(self._addr, self._cmd(self._REG_RHR))
                self._buf.append(int(b) & 0xFF)

    def readline(self) -> bytes:
        if not self.is_open:
            return b""
        deadline = time.time() + self.timeout
        while time.time() < deadline:
            self._read_available()
            nl = self._buf.find(b"\n")
            if nl >= 0:
                out = bytes(self._buf[: nl + 1])
                del self._buf[: nl + 1]
                return out
            # If some bytes are waiting and HW says no more data, return partial.
            lsr = self._read_reg(self._REG_LSR)
            if self._buf and not (lsr & self._LSR_DATA_READY):
                out = bytes(self._buf)
                self._buf.clear()
                return out
            time.sleep(0.01)
        if self._buf:
            out = bytes(self._buf)
            self._buf.clear()
            return out
        return b""

    def close(self) -> None:
        if self.is_open:
            try:
                self._bus.close()
            except Exception:
                pass
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
    if baudrate is None:
        try:
            baudrate = int(os.environ.get("UART_BAUD", "38400"))
        except ValueError:
            baudrate = 38400

    backend = os.environ.get("UART_BACKEND", "").strip().lower()
    if backend in {"sc16is750", "i2c", "xbee_i2c"}:
        try:
            bus_id = int(os.environ.get("SC16IS750_I2C_BUS", os.environ.get("FSW_I2C_BUS", "1")), 0)
            addr = int(os.environ.get("SC16IS750_I2C_ADDR", "0x48"), 0)
            timeout = float(os.environ.get("SC16IS750_READ_TIMEOUT_SEC", "1.0"))
            ser = SC16IS750Serial(bus_id=bus_id, addr=addr, baudrate=baudrate, timeout=timeout)
            logger.info(
                "UART backend SC16IS750 opened on /dev/i2c-%s addr=0x%02X @ %s",
                bus_id,
                addr,
                baudrate,
            )
            return ser
        except Exception as exc:
            logger.warning("SC16IS750 backend unavailable; falling back to pyserial (%s)", exc)

    try:
        import serial  # type: ignore
    except Exception as exc:
        logger.warning("pyserial unavailable; using DummySerial (%s)", exc)
        return DummySerial()

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
