"""GPS / GNSS for `gpsapp`: **u-blox DDC I2C** NMEA (e.g. GNSS 7 Click / NEO-M9N).

FSW assumes **GNSS on I2C**; the flight radio uses UART (`comm`), not the GPS.

Optional **UART NMEA** only if ``GPS_USE_UART=1`` (e.g. USB dongle), separate from XBee.

I2C: length at 0xFD/0xFE, stream from 0xFF (NEO-M9N integration manual §3.7.2). Default **0x42**.
"""

from __future__ import annotations

import logging
import os
import sys
import time
from pathlib import Path
from typing import Any, Optional

_REPO = Path(__file__).resolve().parents[1]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

logger = logging.getLogger(__name__)


def _port_candidates() -> list[str]:
    env = os.environ.get("GPS_DEVICE", "").strip()
    if env:
        return [p.strip() for p in env.split(",") if p.strip()]
    return ["/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyS0", "/dev/ttyAMA0"]


def _use_uart_gnss() -> bool:
    """Rare: USB/UART NMEA. Default is I2C u-blox only."""
    v = os.environ.get("GPS_USE_UART", "").strip().lower()
    return v in ("1", "true", "yes", "on")


def _parse_coord(raw: str, hemi: str) -> Optional[float]:
    if not raw or raw == "0":
        return None
    try:
        v = float(raw)
    except ValueError:
        return None
    deg = int(v // 100)
    minutes = v - deg * 100
    out = deg + minutes / 60.0
    if hemi.upper() in ("S", "W"):
        out = -out
    return out


def _parse_gga(parts: list[str]) -> Optional[dict]:
    if len(parts) < 10:
        return None
    lat = _parse_coord(parts[2], parts[3])
    lon = _parse_coord(parts[4], parts[5])
    if lat is None or lon is None:
        return None
    try:
        fix_q = int(float(parts[6] or 0))
        sats = int(float(parts[7] or 0))
        alt = float(parts[9] or 0.0)
    except ValueError:
        return None
    return {
        "time": (parts[1] or "000000")[:6],
        "lat": lat,
        "lon": lon,
        "alt": alt,
        "sats": sats,
        "fix_q": fix_q,
    }


def _parse_rmc(parts: list[str]) -> Optional[dict]:
    if len(parts) < 10:
        return None
    status = (parts[2] or "V").upper()
    lat = _parse_coord(parts[3], parts[4])
    lon = _parse_coord(parts[5], parts[6])
    if lat is None or lon is None:
        return None
    try:
        speed_kn = float(parts[7] or 0.0)
        course = float(parts[8] or 0.0)
    except ValueError:
        return None
    return {
        "time": (parts[1] or "000000")[:6],
        "status": status,
        "lat": lat,
        "lon": lon,
        "speed_ms": speed_kn * 0.514444,
        "course": course,
    }


def _ingest_nmea_line(dev: dict, line: str) -> None:
    if len(line) < 6 or line[0] != "$":
        return
    body = line[1:].split("*", 1)[0]
    parts = body.split(",")
    tag = parts[0] if parts else ""
    if tag.endswith("GGA") and len(parts) > 1:
        g = _parse_gga(parts)
        if g:
            dev["gga"] = g
    elif tag.endswith("RMC") and len(parts) > 1:
        r = _parse_rmc(parts)
        if r:
            dev["rmc"] = r


def _nmea_feed_bytes(dev: dict, chunk: bytes) -> None:
    tail: bytes = dev.setdefault("_nmea_tail", b"")
    data = tail + chunk
    lines = data.split(b"\n")
    dev["_nmea_tail"] = lines[-1]
    for raw_line in lines[:-1]:
        try:
            line = raw_line.decode("ascii", errors="ignore").strip()
        except Exception:
            continue
        _ingest_nmea_line(dev, line)


def _gps_build_return(dev: dict) -> Optional[list]:
    gga = dev.get("gga")
    if not gga or int(gga.get("fix_q", 0)) < 1:
        return None

    rmc = dev.get("rmc")
    if rmc:
        status = str(rmc.get("status", "V")).upper()
        speed_ms = float(rmc.get("speed_ms", 0.0))
        course = float(rmc.get("course", 0.0))
    else:
        status = "A"
        speed_ms = 0.0
        course = 0.0

    return [
        gga["time"],
        gga["alt"],
        gga["lat"],
        gga["lon"],
        gga["sats"],
        gga["fix_q"],
        status,
        speed_ms,
        course,
    ]


def _ublox_ddc_bytes_available(i2c: Any, address: int) -> int:
    """RX FIFO length: u-blox uses **separate** regs 0xFD (MSB) and 0xFE (LSB).

    A single writeto+read of 2 bytes assumes subaddress auto-increment; many u-blox
    modules do **not** do that for these regs and return ``EIO`` on Pi/Blinka.
    """
    one = bytearray(1)
    i2c.writeto_then_readfrom(address, bytes([0xFD]), one, out_end=1, in_end=1)
    msb = int(one[0])
    i2c.writeto_then_readfrom(address, bytes([0xFE]), one, out_end=1, in_end=1)
    lsb = int(one[0])
    return (msb << 8) | lsb


def _ublox_read_chunk_max() -> int:
    """SMBus ``read_i2c_block_data`` is often capped at 32; u-blox DDC expects **burst** read from 0xFF."""
    try:
        v = int(os.environ.get("GPS_I2C_READ_CHUNK", "32"), 0)
    except ValueError:
        v = 32
    return max(1, min(v, 128))


def _ublox_ddc_read_stream(i2c: Any, address: int, nbytes: int) -> bytes:
    """Drain u-blox RX FIFO: write reg **0xFF** once per chunk, then read **multiple** FIFO bytes.

    One I2C transaction per byte (Blinka → ``read_i2c_block_data``) often hits ``EIO`` (121) on Pi
    when the queue has tens of bytes; burst read matches u-blox / SparkFun reference code.
    """
    if nbytes <= 0:
        return b""
    nbytes = min(int(nbytes), 512)
    reg = bytes([0xFF])
    chunk_max = _ublox_read_chunk_max()
    out = bytearray()
    remaining = nbytes
    while remaining > 0:
        take = min(chunk_max, remaining)
        buf = bytearray(take)
        try:
            i2c.writeto_then_readfrom(address, reg, buf, out_end=1, in_end=take)
            out.extend(buf)
            remaining -= take
        except OSError:
            if take > 1:
                take = 1
                buf = bytearray(1)
                try:
                    i2c.writeto_then_readfrom(address, reg, buf, out_end=1, in_end=1)
                    out.extend(buf)
                    remaining -= 1
                except OSError:
                    break
            else:
                break
    return bytes(out)


def _init_gps_i2c_ublox() -> Any:
    from lib import i2c_bus

    addr = int(os.environ.get("GPS_I2C_ADDR", "0x42"), 0)
    attempts = max(1, int(os.environ.get("GPS_I2C_INIT_RETRIES", "5"), 0))
    delay = float(os.environ.get("GPS_I2C_INIT_DELAY_SEC", "0.08"))
    last_exc: Exception | None = None
    for i in range(attempts):
        try:
            with i2c_bus.i2c_lock():
                i2c = i2c_bus.get_i2c()
                n = _ublox_ddc_bytes_available(i2c, addr)
            logger.info("GPS u-blox DDC I2C at 0x%02x (rx queue ~%d bytes)", addr, n)
            return {
                "kind": "i2c_ublox",
                "addr": addr,
                "gga": None,
                "rmc": None,
                "_nmea_tail": b"",
            }
        except Exception as exc:
            last_exc = exc
            logger.warning(
                "GPS u-blox I2C probe attempt %d/%d at 0x%02x: %s",
                i + 1,
                attempts,
                addr,
                exc,
            )
            if i + 1 < attempts:
                time.sleep(delay)
    logger.warning("GPS u-blox I2C probe failed at 0x%02x (last: %s)", addr, last_exc)
    return None


def _init_gps_uart() -> Any:
    import serial  # type: ignore

    baud = int(os.environ.get("GPS_BAUD", "9600"))
    last_exc: Exception | None = None
    for port in _port_candidates():
        try:
            ser = serial.Serial(port, baud, timeout=0.2)
            logger.info("GPS UART opened %s @ %s", port, baud)
            return {"kind": "uart", "ser": ser, "gga": None, "rmc": None}
        except Exception as exc:
            last_exc = exc
            logger.warning("GPS open failed for %s: %s", port, exc)
    logger.warning("GPS UART unavailable (last error: %s)", last_exc)
    return None


def init_gps() -> Any:
    """Open GNSS: **I2C u-blox by default**; UART only when ``GPS_USE_UART=1``."""
    if _use_uart_gnss():
        return _init_gps_uart()
    return _init_gps_i2c_ublox()


def _gps_readdata_uart(dev: dict) -> Optional[list]:
    ser = dev["ser"]
    for _ in range(32):
        raw = ser.readline()
        if not raw:
            break
        try:
            line = raw.decode("ascii", errors="ignore").strip()
        except Exception:
            continue
        _ingest_nmea_line(dev, line)
    return _gps_build_return(dev)


def _gps_readdata_i2c(dev: dict) -> Optional[list]:
    from lib import i2c_bus

    addr = int(dev["addr"])
    with i2c_bus.i2c_lock():
        i2c = i2c_bus.get_i2c()
        try:
            n = _ublox_ddc_bytes_available(i2c, addr)
        except OSError as exc:
            logger.debug("GPS I2C bytes_available failed: %s", exc)
            n = 0
        except Exception:
            n = 0
        if n > 0:
            n = min(n, 256)
            try:
                chunk = _ublox_ddc_read_stream(i2c, addr, n)
                _nmea_feed_bytes(dev, chunk)
            except OSError as exc:
                logger.warning("GPS I2C read_stream failed (%s); next poll will retry", exc)
    return _gps_build_return(dev)


def gps_readdata(dev: Optional[dict]) -> Optional[list]:
    if not dev:
        return None
    if dev.get("kind") == "i2c_ublox":
        return _gps_readdata_i2c(dev)
    return _gps_readdata_uart(dev)


def gps_terminate(dev: dict) -> None:
    if not dev or dev.get("kind") == "i2c_ublox":
        return
    try:
        dev["ser"].close()
    except Exception:
        pass


if __name__ == "__main__":
    import time

    from lib.sensor_cli import cli_period_sec

    logging.basicConfig(level=logging.INFO, format="%(message)s")
    if _use_uart_gnss():
        print("GPS: UART mode (GPS_USE_UART=1)", flush=True)
    else:
        print("GPS: I2C u-blox (default). Set GPS_I2C_ADDR if not 0x42.", flush=True)
    dev = init_gps()
    period = cli_period_sec()
    if dev is None:
        print(
            "GPS init failed. I2C: FSW_I2C_BUS, wiring, 0x42. UART: GPS_USE_UART=1 and GPS_DEVICE.",
            flush=True,
        )
        raise SystemExit(1)
    print("GPS: OK, streaming...", flush=True)
    try:
        while True:
            row = gps_readdata(dev)
            if row is None:
                print("fix=no (sky view / baud / I2C / NMEA)", flush=True)
            else:
                gt, alt, lat, lon, sats, fixq, st, spd, crs = row[:9]
                print(
                    f"time={gt} lat={lat:.6f} lon={lon:.6f} alt_m={alt:.1f} "
                    f"sats={sats} fix={fixq} rmc={st} v_ms={spd:.2f} crs={crs:.1f}",
                    flush=True,
                )
            time.sleep(period)
    except KeyboardInterrupt:
        print("", flush=True)
    finally:
        gps_terminate(dev)
