"""GPS / GNSS: u-blox GNSS 7 Click (NEO-M9N) over SMBus I2C.

Reads NMEA sentences (GGA + RMC) by polling register 0xFF with a timeout loop.
Returns [time_hhmmss, alt_m, lat, lon, sats, fix_quality, rmc_status, speed_ms, course_deg].
"""

from __future__ import annotations

import logging
import math
import os
import sys
import time
from pathlib import Path
from typing import Optional

_REPO = Path(__file__).resolve().parents[1]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

logger = logging.getLogger(__name__)

try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None  # type: ignore

I2C_BUS_NUM = int(os.environ.get("GPS_I2C_BUS", "1"))
GNSS_ADDR = int(os.environ.get("GPS_I2C_ADDR", "0x42"), 0)
READ_SIZE = 32

_read_buffer = b""


# ---------------------------------------------------------------------------
# I2C helpers
# ---------------------------------------------------------------------------

def _i2c_read_block(bus) -> bytes:
    from lib import i2c_bus
    with i2c_bus.i2c_lock():
        try:
            data = bus.read_i2c_block_data(GNSS_ADDR, 0xFF, READ_SIZE)
        except OSError:
            data = bus.read_i2c_block_data(GNSS_ADDR, 0x00, READ_SIZE)
    return bytes(data)


def _read_nmea_lines(bus, timeout: float = 0.2) -> list[bytes]:
    global _read_buffer
    nmea_lines: list[bytes] = []
    got_valid = False
    start = time.time()

    while time.time() - start < timeout:
        try:
            chunk = _i2c_read_block(bus)
        except OSError:
            time.sleep(0.01)
            continue

        if not chunk or not chunk.strip(b"\x00\xff"):
            if got_valid:
                break
            time.sleep(0.01)
            continue

        got_valid = True
        _read_buffer += chunk

        while b"\n" in _read_buffer:
            line, _read_buffer = _read_buffer.split(b"\n", 1)
            line = line + b"\n"
            if b"$" not in line:
                continue
            line = line[line.find(b"$"):]
            nmea_lines.append(line)

    return nmea_lines


# ---------------------------------------------------------------------------
# NMEA parsing
# ---------------------------------------------------------------------------

def _parse_coord(raw: str, hemi: str) -> Optional[float]:
    if not raw:
        return None
    try:
        v = float(raw)
    except ValueError:
        return None
    deg = int(v // 100)
    out = deg + (v - deg * 100) / 60.0
    if hemi.upper() in ("S", "W"):
        out = -out
    return out


def _parse_nmea(nmea_lines: list[bytes]) -> Optional[list]:
    gga: Optional[list[str]] = None
    rmc: Optional[list[str]] = None

    for line in nmea_lines:
        try:
            decoded = line.decode("ascii", errors="ignore").strip()
        except Exception:
            continue

        if decoded.startswith(("$GPGGA", "$GNGGA")):
            parts = decoded.split(",")
            if len(parts) > 7:
                gga = parts
        elif decoded.startswith(("$GPRMC", "$GNRMC")):
            parts = decoded.split(",")
            if len(parts) > 8:
                rmc = parts

    if gga is None:
        return None

    # --- GGA fields ---
    raw_time = gga[1] if len(gga) > 1 else ""
    gps_time = raw_time[:6] if len(raw_time) >= 6 else "000000"

    lat = _parse_coord(gga[2], gga[3]) if len(gga) > 3 else None
    lon = _parse_coord(gga[4], gga[5]) if len(gga) > 5 else None
    lat = lat if lat is not None else 0.0
    lon = lon if lon is not None else 0.0

    try:
        fix_quality = int(gga[6]) if len(gga) > 6 and gga[6] else 0
    except ValueError:
        fix_quality = 0

    try:
        sats = int(gga[7]) if len(gga) > 7 and gga[7] else 0
    except ValueError:
        sats = 0

    try:
        alt = float(gga[9]) if len(gga) > 9 and gga[9] else 0.0
    except ValueError:
        alt = 0.0

    # --- RMC fields ---
    rmc_status = "V"
    speed_ms = 0.0
    course = 0.0

    if rmc is not None and len(rmc) > 8:
        try:
            rmc_status = rmc[2] if rmc[2] else "V"
            if rmc[7]:
                speed_ms = float(rmc[7]) * 0.514444
            if rmc[8]:
                raw_course = float(rmc[8])
                course = raw_course % 360.0
                if not math.isfinite(course):
                    course = 0.0
        except (ValueError, IndexError):
            pass

    return [gps_time, alt, lat, lon, sats, fix_quality, rmc_status, speed_ms, course]


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def init_gps():
    if SMBus is None:
        logger.error("GPS: smbus2 not installed; GPS unavailable")
        return None
    try:
        bus = SMBus(I2C_BUS_NUM)
        logger.info("GPS SMBus(%d) opened, addr=0x%02x", I2C_BUS_NUM, GNSS_ADDR)
        return bus
    except Exception as exc:
        logger.error("GPS SMBus init failed: %s", exc)
        return None


def gps_readdata(bus) -> Optional[list]:
    """Return [time_hhmmss, alt_m, lat, lon, sats, fix_q, rmc_status, speed_ms, course_deg] or None."""
    if bus is None:
        return None
    nmea_lines = _read_nmea_lines(bus, timeout=0.2)
    return _parse_nmea(nmea_lines)


def terminate_gps(bus) -> None:
    try:
        if bus is not None:
            bus.close()
    except Exception:
        pass


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    logging.basicConfig(level=logging.WARNING, format="%(message)s")
    print("GPS monitor  (1s interval, Ctrl-C to quit)", flush=True)
    print("  [GGA/RMC] NMEA only", flush=True)
    print("-" * 80, flush=True)

    bus = init_gps()
    if bus is None:
        print("GPS init failed. Check I2C wiring and GPS_I2C_ADDR (default 0x42).", flush=True)
        raise SystemExit(1)

    try:
        while True:
            row = gps_readdata(bus)
            if row is None:
                print("[ --- ]  no fix", flush=True)
            else:
                gt, alt, lat, lon, sats, fixq, status, spd, crs = row
                print(
                    f"[GGA]  fix={fixq}  status={status}  "
                    f"lat={lat:11.6f}  lon={lon:11.6f}  alt={alt:7.1f}m  "
                    f"sats={sats:2d}  spd={spd:5.2f}m/s  crs={crs:6.1f}deg  t={gt}",
                    flush=True,
                )
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("", flush=True)
    finally:
        terminate_gps(bus)
