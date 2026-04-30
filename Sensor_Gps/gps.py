"""GPS / GNSS for `gpsapp`: UART NMEA or u-blox DDC (I2C) NMEA (e.g. GNSS 7 Click / NEO-M9N).

UART uses a **separate** serial device from the XBee/comm link.

I2C uses the u-blox register map: length at 0xFD/0xFE, stream bytes from 0xFF
(see NEO-M9N integration manual §3.7.2). Default address **0x42**.
"""

from __future__ import annotations

import logging
import os
import sys
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


def _use_i2c_gnss() -> bool:
    v = os.environ.get("GPS_USE_I2C", "").strip().lower()
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
    buf = bytearray(2)
    i2c.writeto_then_readfrom(address, bytes([0xFD]), buf, out_end=1, in_end=2)
    return int(buf[0]) | (int(buf[1]) << 8)


def _ublox_ddc_read_stream(i2c: Any, address: int, nbytes: int) -> bytes:
    if nbytes <= 0:
        return b""
    reg = bytes([0xFF])
    one = bytearray(1)
    out = bytearray()
    for _ in range(nbytes):
        i2c.writeto_then_readfrom(address, reg, one, out_end=1, in_end=1)
        out.append(one[0])
    return bytes(out)


def _init_gps_i2c_ublox() -> Any:
    from lib import i2c_bus

    addr = int(os.environ.get("GPS_I2C_ADDR", "0x42"), 0)
    try:
        with i2c_bus.i2c_lock():
            i2c = i2c_bus.get_i2c()
            n = _ublox_ddc_bytes_available(i2c, addr)
        logger.info("GPS u-blox DDC I2C at 0x%02x (rx queue ~%d bytes)", addr, n)
    except Exception as exc:
        logger.warning("GPS u-blox I2C probe failed at 0x%02x: %s", addr, exc)
        return None
    return {
        "kind": "i2c_ublox",
        "addr": addr,
        "gga": None,
        "rmc": None,
        "_nmea_tail": b"",
    }


def init_gps() -> Any:
    if _use_i2c_gnss():
        return _init_gps_i2c_ublox()

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
    logger.warning("GPS UART unavailable (last error: %s); gpsapp will use synthetic", last_exc)
    return None


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
        except Exception:
            n = 0
        if n > 0:
            n = min(n, 256)
            chunk = _ublox_ddc_read_stream(i2c, addr, n)
            _nmea_feed_bytes(dev, chunk)
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
    dev = init_gps()
    period = cli_period_sec()
    if dev is None:
        print(
            "GPS init failed. UART: set GPS_DEVICE. I2C u-blox: GPS_USE_I2C=1, GPS_I2C_ADDR=0x42",
            flush=True,
        )
        raise SystemExit(1)
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
