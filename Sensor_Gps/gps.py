"""UART NMEA GPS reader for `gpsapp` (optional hardware path).

Uses a **separate** serial device from the XBee/comm link. Default candidates
prefer USB GPS (`/dev/ttyUSB0`) so comm can keep `/dev/serial0`.
"""

from __future__ import annotations

import logging
import os
from typing import Any, Optional

logger = logging.getLogger(__name__)


def _port_candidates() -> list[str]:
    env = os.environ.get("GPS_DEVICE", "").strip()
    if env:
        return [p.strip() for p in env.split(",") if p.strip()]
    return ["/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyS0", "/dev/ttyAMA0"]


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
    # $GNGGA / $GPGGA: 1=time,2=lat,3=N/S,4=lon,5=E/W,6=fix,7=numSV,8=HDOP,9=alt,10,11=sep
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
    # $GNRMC / $GPRMC: 2=status,3=lat,4,5=lon,6,7=speed(kn),8=course
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


def init_gps() -> Any:
    import serial  # type: ignore

    baud = int(os.environ.get("GPS_BAUD", "9600"))
    last_exc: Exception | None = None
    for port in _port_candidates():
        try:
            ser = serial.Serial(port, baud, timeout=0.2)
            logger.info("GPS UART opened %s @ %s", port, baud)
            return {"ser": ser, "gga": None, "rmc": None}
        except Exception as exc:
            last_exc = exc
            logger.warning("GPS open failed for %s: %s", port, exc)
    logger.warning("GPS UART unavailable (last error: %s); gpsapp will use synthetic", last_exc)
    return None


def gps_readdata(dev: Optional[dict]) -> Optional[list]:
    if not dev:
        return None
    ser = dev["ser"]
    # Drain a few lines to reduce latency
    for _ in range(32):
        raw = ser.readline()
        if not raw:
            break
        try:
            line = raw.decode("ascii", errors="ignore").strip()
        except Exception:
            continue
        if len(line) < 6 or line[0] != "$":
            continue
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


def gps_terminate(dev: dict) -> None:
    try:
        dev["ser"].close()
    except Exception:
        pass
