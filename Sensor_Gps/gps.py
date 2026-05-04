"""GPS / GNSS for `gpsapp`: **u-blox DDC I2C** NMEA (e.g. GNSS 7 Click / NEO-M9N).

FSW assumes **GNSS on I2C**; the flight radio uses UART (`comm`), not the GPS.

Optional **UART NMEA** only if ``GPS_USE_UART=1`` (e.g. USB dongle), separate from XBee.

I2C: length at 0xFD/0xFE, stream from 0xFF (NEO-M9N integration manual §3.7.2). Default **0x42**.
"""

from __future__ import annotations

import logging
import math
import os
import struct
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
    if not math.isfinite(course) or course < 0.0 or course > 360.0:
        course = 0.0
    return {
        "time": (parts[1] or "000000")[:6],
        "status": status,
        "lat": lat,
        "lon": lon,
        "speed_ms": speed_kn * 0.514444,
        "course": course,
    }


def _nmea_checksum_strict() -> bool:
    v = os.environ.get("GPS_NMEA_STRICT_CHECKSUM", "1").strip().lower()
    return v not in ("0", "false", "no", "off")


def _nmea_checksum_valid(line: str) -> bool:
    """XOR of bytes between ``$`` and ``*`` must match the two hex digits after ``*``."""
    if not _nmea_checksum_strict():
        return True
    star = line.find("*")
    if star < 0:
        return False
    if star + 2 >= len(line):
        return False
    try:
        expect = int(line[star + 1 : star + 3], 16)
    except ValueError:
        return False
    xor = 0
    for ch in line[1:star]:
        xor ^= ord(ch) & 0xFF
    return xor == expect


def _ingest_nmea_line(dev: dict, line: str) -> None:
    line = line.strip()
    if len(line) < 6 or line[0] != "$":
        return
    if not _nmea_checksum_valid(line):
        logger.debug("GPS: dropped NMEA (checksum): %s", line[:96])
        return
    body = line[1:].split("*", 1)[0]
    parts = body.split(",")
    tag = parts[0] if parts else ""
    if tag.endswith("GGA") and len(parts) > 1:
        g = _parse_gga(parts)
        if g:
            g["_seen_ts"] = time.time()
            dev["gga"] = g
    elif tag.endswith("RMC") and len(parts) > 1:
        r = _parse_rmc(parts)
        if r:
            r["_seen_ts"] = time.time()
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


def _ubx_checksum(data: bytes) -> tuple[int, int]:
    ck_a = 0
    ck_b = 0
    for b in data:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def _ubx_frame(msg_class: int, msg_id: int, payload: bytes = b"") -> bytes:
    header = bytes([msg_class & 0xFF, msg_id & 0xFF]) + len(payload).to_bytes(2, "little")
    ck_a, ck_b = _ubx_checksum(header + payload)
    return b"\xB5\x62" + header + payload + bytes([ck_a, ck_b])


def _parse_nav_pvt(payload: bytes) -> Optional[dict]:
    """Parse UBX-NAV-PVT payload (class 0x01, id 0x07)."""
    if len(payload) < 92:
        return None
    try:
        i_tow = struct.unpack_from("<I", payload, 0)[0]
        year = struct.unpack_from("<H", payload, 4)[0]
        month = payload[6]
        day = payload[7]
        hour = payload[8]
        minute = payload[9]
        second = payload[10]
        valid = payload[11]
        fix_type = payload[20]
        flags = payload[21]
        num_sv = payload[23]
        lon_raw = struct.unpack_from("<i", payload, 24)[0]
        lat_raw = struct.unpack_from("<i", payload, 28)[0]
        height_raw = struct.unpack_from("<i", payload, 32)[0]
        h_msl_raw = struct.unpack_from("<i", payload, 36)[0]
        h_acc_raw = struct.unpack_from("<I", payload, 40)[0]
        v_acc_raw = struct.unpack_from("<I", payload, 44)[0]
        vel_n_raw = struct.unpack_from("<i", payload, 48)[0]
        vel_e_raw = struct.unpack_from("<i", payload, 52)[0]
        vel_d_raw = struct.unpack_from("<i", payload, 56)[0]
        g_speed_raw = struct.unpack_from("<i", payload, 60)[0]
        head_mot_raw = struct.unpack_from("<i", payload, 64)[0]
        s_acc_raw = struct.unpack_from("<I", payload, 68)[0]
        head_acc_raw = struct.unpack_from("<I", payload, 72)[0]
    except (IndexError, struct.error):
        return None

    if not (1 <= month <= 12 and 1 <= day <= 31 and 0 <= hour <= 23 and 0 <= minute <= 59 and 0 <= second <= 60):
        time_text = "000000"
    else:
        time_text = f"{hour:02d}{minute:02d}{min(second, 59):02d}"

    return {
        "source": "UBX_NAV_PVT",
        "i_tow": i_tow,
        "date": (year, month, day),
        "time": time_text,
        "valid": valid,
        "fix_type": fix_type,
        "flags": flags,
        "num_sv": num_sv,
        "lat": lat_raw * 1.0e-7,
        "lon": lon_raw * 1.0e-7,
        "alt": h_msl_raw / 1000.0,
        "height": height_raw / 1000.0,
        "h_acc": h_acc_raw / 1000.0,
        "v_acc": v_acc_raw / 1000.0,
        "vel_n": vel_n_raw / 1000.0,
        "vel_e": vel_e_raw / 1000.0,
        "vel_d": vel_d_raw / 1000.0,
        "g_speed": abs(g_speed_raw) / 1000.0,
        "head_mot": (head_mot_raw * 1.0e-5) % 360.0,
        "s_acc": s_acc_raw / 1000.0,
        "head_acc": head_acc_raw * 1.0e-5,
        "_seen_ts": time.time(),
    }


def _ingest_ubx_frame(dev: dict, msg_class: int, msg_id: int, payload: bytes) -> None:
    if msg_class == 0x01 and msg_id == 0x07:
        pvt = _parse_nav_pvt(payload)
        if pvt is not None:
            dev["pvt"] = pvt


def _ubx_feed_bytes(dev: dict, chunk: bytes) -> None:
    data = dev.setdefault("_ubx_tail", b"") + chunk
    idx = 0
    while True:
        start = data.find(b"\xB5\x62", idx)
        if start < 0:
            dev["_ubx_tail"] = data[-1:] if data.endswith(b"\xB5") else b""
            return
        if len(data) - start < 8:
            dev["_ubx_tail"] = data[start:]
            return
        msg_class = data[start + 2]
        msg_id = data[start + 3]
        length = int.from_bytes(data[start + 4 : start + 6], "little")
        end = start + 8 + length
        if len(data) < end:
            dev["_ubx_tail"] = data[start:]
            return
        frame_body = data[start + 2 : start + 6 + length]
        ck_a, ck_b = _ubx_checksum(frame_body)
        if data[start + 6 + length] == ck_a and data[start + 7 + length] == ck_b:
            _ingest_ubx_frame(dev, msg_class, msg_id, data[start + 6 : start + 6 + length])
        idx = end


def _gps_build_return(dev: dict) -> Optional[list]:
    pvt = dev.get("pvt")
    if pvt:
        now = time.time()
        pos_seen = float(pvt.get("_seen_ts", now))
        pos_age = max(0.0, now - pos_seen)
        fix_type = int(pvt.get("fix_type", 0))
        num_sv = int(pvt.get("num_sv", 0))
        status = "A" if fix_type >= 2 else "V"
        motion_valid = 1 if status == "A" else 0
        return [
            pvt["time"],
            pvt["alt"],
            pvt["lat"],
            pvt["lon"],
            num_sv,
            1 if fix_type >= 2 else 0,
            status,
            float(pvt.get("g_speed", 0.0)),
            float(pvt.get("head_mot", 0.0)),
            motion_valid,
            pos_age,
            pos_age,
            "UBX_NAV_PVT",
            fix_type,
            float(pvt.get("h_acc", 1.0e9)),
            float(pvt.get("v_acc", 1.0e9)),
            float(pvt.get("s_acc", 1.0e9)),
            float(pvt.get("head_acc", 1.0e9)),
            float(pvt.get("vel_n", 0.0)),
            float(pvt.get("vel_e", 0.0)),
            int(pvt.get("valid", 0)),
        ]

    gga = dev.get("gga")
    if not gga or int(gga.get("fix_q", 0)) < 1:
        return None

    now = time.time()
    pos_seen = float(gga.get("_seen_ts", now))
    pos_age = max(0.0, now - pos_seen)
    rmc = dev.get("rmc")
    if rmc:
        status = str(rmc.get("status", "V")).upper()
        speed_ms = float(rmc.get("speed_ms", 0.0))
        course = float(rmc.get("course", 0.0))
        motion_seen = float(rmc.get("_seen_ts", now))
        motion_age = max(0.0, now - motion_seen)
        motion_valid = 1 if status == "A" else 0
    else:
        status = "V"
        speed_ms = 0.0
        course = 0.0
        motion_age = 1.0e9
        motion_valid = 0

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
        motion_valid,
        pos_age,
        motion_age,
        "NMEA",
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


def _ublox_enable_nav_pvt_i2c(i2c: Any, address: int) -> None:
    """Request UBX-NAV-PVT output on the DDC/I2C port; harmless if unsupported."""
    payload = bytes([0x01, 0x07, 1, 0, 0, 0, 0, 0])
    frame = _ubx_frame(0x06, 0x01, payload)
    try:
        i2c.writeto(address, frame)
    except Exception as exc:
        logger.debug("GPS: UBX NAV-PVT enable over I2C failed: %s", exc)


def _ublox_enable_nav_pvt_uart(ser: Any) -> None:
    payload = bytes([0x01, 0x07, 0, 1, 0, 0, 0, 0])
    frame = _ubx_frame(0x06, 0x01, payload)
    try:
        ser.write(frame)
    except Exception as exc:
        logger.debug("GPS: UBX NAV-PVT enable over UART failed: %s", exc)


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
                _ublox_enable_nav_pvt_i2c(i2c, addr)
            logger.info("GPS u-blox DDC I2C at 0x%02x (rx queue ~%d bytes)", addr, n)
            return {
                "kind": "i2c_ublox",
                "addr": addr,
                "gga": None,
                "rmc": None,
                "pvt": None,
                "_nmea_tail": b"",
                "_ubx_tail": b"",
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
            _ublox_enable_nav_pvt_uart(ser)
            logger.info("GPS UART opened %s @ %s", port, baud)
            return {"kind": "uart", "ser": ser, "gga": None, "rmc": None, "pvt": None, "_ubx_tail": b""}
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
        _ubx_feed_bytes(dev, raw)
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
                _ubx_feed_bytes(dev, chunk)
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
