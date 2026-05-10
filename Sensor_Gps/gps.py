"""GPS / GNSS for `gpsapp`: **u-blox DDC I2C** NMEA (e.g. GNSS 7 Click / NEO-M9N).

FSW assumes **GNSS on I2C**; the flight radio uses UART (`comm`), not the GPS.

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


def _debug_raw_enabled() -> bool:
    env = os.environ.get("GPS_DEBUG_RAW")
    if env is None:
        return __name__ == "__main__"
    v = env.strip().lower()
    return v not in ("0", "false", "no", "off")


def _gps_cli_period_sec() -> float:
    try:
        return max(0.05, float(os.environ.get("GPS_PRINT_PERIOD_SEC", "1.0")))
    except ValueError:
        return 1.0


def _gps_row_stale_max_sec() -> float:
    try:
        return max(0.1, float(os.environ.get("GPS_ROW_STALE_MAX_SEC", "3.0")))
    except ValueError:
        return 3.0


def _nmea_tail_max_bytes() -> int:
    try:
        return max(32, int(os.environ.get("GPS_NMEA_TAIL_MAX_BYTES", "256"), 0))
    except ValueError:
        return 256


def _gps_i2c_recovery_read_bytes() -> int:
    try:
        return max(0, min(256, int(os.environ.get("GPS_I2C_RECOVERY_READ_BYTES", "64"), 0)))
    except ValueError:
        return 64


def _gps_i2c_max_consecutive_errors() -> int:
    try:
        return max(1, int(os.environ.get("GPS_I2C_MAX_CONSECUTIVE_ERRORS", "3"), 0))
    except ValueError:
        return 3


def _debug_print(message: str) -> None:
    if _debug_raw_enabled():
        print(message, flush=True)


def _bytes_ascii(data: bytes) -> str:
    return "".join(chr(b) if 32 <= b <= 126 else "." for b in data)


def _debug_section(title: str) -> None:
    _debug_print("")
    _debug_print(f"=== {title} ===")


def _debug_value(value: Any) -> str:
    if isinstance(value, float):
        return f"{value:.6f}"
    return str(value)


def _debug_dict(title: str, data: dict, order: list[str] | None = None) -> None:
    _debug_section(title)
    keys = list(order or [])
    keys.extend(k for k in data.keys() if k not in keys)
    now = time.time()
    for key in keys:
        if key not in data:
            continue
        value = data[key]
        if key == "_seen_ts":
            _debug_print(f"{key}: {value:.3f} (age={max(0.0, now - float(value)):.3f}s)")
        else:
            _debug_print(f"{key}: {_debug_value(value)}")


_GPS_ROW_FIELDS = [
    "time_utc_hhmmss",
    "alt_m",
    "lat_deg",
    "lon_deg",
    "satellites",
    "fix_quality",
    "rmc_status",
    "speed_m_s",
    "course_deg",
    "motion_valid",
    "position_age_s",
    "motion_age_s",
    "source",
    "fix_type",
    "h_acc_m",
    "v_acc_m",
    "speed_acc_m_s",
    "heading_acc_deg",
    "vel_n_m_s",
    "vel_e_m_s",
    "valid_flags",
]


def _debug_row(row: Optional[list]) -> None:
    _debug_section("GPS_ROW decoded output")
    if row is None:
        _debug_print("row: None")
        _debug_print("meaning: no valid position row is available yet")
        return
    for i, value in enumerate(row):
        name = _GPS_ROW_FIELDS[i] if i < len(_GPS_ROW_FIELDS) else f"extra_{i}"
        _debug_print(f"{name}: {_debug_value(value)}")


def _debug_latest_measurements(dev: dict) -> None:
    now = time.time()
    pvt = dev.get("pvt")
    gga = dev.get("gga")
    rmc = dev.get("rmc")
    _debug_section("GPS_LATEST parsed values")
    if pvt:
        _debug_print("[UBX_NAV_PVT]")
        _debug_print(f"  time: {pvt.get('time')}")
        _debug_print(f"  fix_type: {pvt.get('fix_type')}  sats: {pvt.get('num_sv')}")
        _debug_print(f"  lat: {_debug_value(pvt.get('lat'))}  lon: {_debug_value(pvt.get('lon'))}")
        _debug_print(f"  alt_m: {_debug_value(pvt.get('alt'))}  height_m: {_debug_value(pvt.get('height'))}")
        _debug_print(f"  speed_m_s: {_debug_value(pvt.get('g_speed'))}  course_deg: {_debug_value(pvt.get('head_mot'))}")
        _debug_print(f"  h_acc_m: {_debug_value(pvt.get('h_acc'))}  v_acc_m: {_debug_value(pvt.get('v_acc'))}")
        _debug_print(f"  age_s: {max(0.0, now - float(pvt.get('_seen_ts', now))):.3f}")
    else:
        _debug_print("[UBX_NAV_PVT] none")

    if gga:
        _debug_print("[NMEA_GGA position]")
        _debug_print(f"  time: {gga.get('time')}")
        _debug_print(f"  fix_quality: {gga.get('fix_q')}  sats: {gga.get('sats')}")
        _debug_print(f"  lat: {_debug_value(gga.get('lat'))}  lon: {_debug_value(gga.get('lon'))}")
        _debug_print(f"  alt_m: {_debug_value(gga.get('alt'))}")
        _debug_print(f"  age_s: {max(0.0, now - float(gga.get('_seen_ts', now))):.3f}")
    else:
        _debug_print("[NMEA_GGA position] none")

    if rmc:
        _debug_print("[NMEA_RMC motion]")
        _debug_print(f"  time: {rmc.get('time')}  status: {rmc.get('status')}")
        _debug_print(f"  lat: {_debug_value(rmc.get('lat'))}  lon: {_debug_value(rmc.get('lon'))}")
        _debug_print(f"  speed_m_s: {_debug_value(rmc.get('speed_ms'))}  course_deg: {_debug_value(rmc.get('course'))}")
        _debug_print(f"  age_s: {max(0.0, now - float(rmc.get('_seen_ts', now))):.3f}")
    else:
        _debug_print("[NMEA_RMC motion] none")


def _debug_runtime_config(dev: dict) -> None:
    _debug_section("GPS_CONFIG")
    _debug_print(f"driver: u-blox DDC/I2C")
    _debug_print(f"kind: {dev.get('kind')}")
    _debug_print(f"i2c_addr: 0x{int(dev.get('addr', 0)):02x}")
    _debug_print(f"i2c_read_chunk_max: {_ublox_read_chunk_max()} bytes")
    _debug_print(f"nmea_strict_checksum: {_nmea_checksum_strict()}")
    _debug_print(f"debug_output: {_debug_raw_enabled()}")


def _debug_i2c_scan(i2c: Any, expected_addr: int) -> None:
    if not _debug_raw_enabled():
        return
    _debug_section("GPS_I2C scan")
    if not hasattr(i2c, "scan"):
        _debug_print("scan: unavailable on this I2C object")
        return
    locked = False
    try:
        if hasattr(i2c, "try_lock"):
            for _ in range(5):
                if i2c.try_lock():
                    locked = True
                    break
                time.sleep(0.01)
            if not locked:
                _debug_print("scan: failed to lock I2C bus")
                return
        addrs = list(i2c.scan())
        _debug_print("addresses: " + (" ".join(f"0x{addr:02x}" for addr in addrs) if addrs else "(none)"))
        _debug_print(f"expected_gps_addr: 0x{expected_addr:02x}")
        _debug_print(f"gps_addr_present: {expected_addr in addrs}")
    except Exception as exc:
        _debug_print(f"scan_error: {exc!r}")
    finally:
        if locked and hasattr(i2c, "unlock"):
            try:
                i2c.unlock()
            except Exception:
                pass


def _debug_dump_bytes(label: str, data: bytes) -> None:
    _debug_section(f"GPS_PACKET raw {label}")
    _debug_print(f"bytes: {len(data)}")
    _debug_print(f"hex: {data.hex(' ')}")
    _debug_print(f"ascii_printable: {_bytes_ascii(data)}")
    text = data.decode("ascii", errors="replace").replace("\r", "\\r")
    if text:
        _debug_print("text_lines:")
        for line in text.split("\n"):
            _debug_print(f"  {line}")


def _debug_flow(dev: dict, stage: str, **fields: Any) -> None:
    details = " ".join(f"{key}={value!r}" for key, value in fields.items())
    _debug_print(f"GPS_FLOW {stage} {details}".rstrip())


def _debug_parser_state(dev: dict) -> None:
    pvt = dev.get("pvt")
    gga = dev.get("gga")
    rmc = dev.get("rmc")
    _debug_section("GPS_PARSER state after this read")
    _debug_print(f"has_ubx_nav_pvt: {bool(pvt)}")
    _debug_print(f"has_nmea_gga: {bool(gga)}")
    _debug_print(f"has_nmea_rmc: {bool(rmc)}")
    _debug_print(f"ubx_tail_bytes_waiting_for_next_read: {len(dev.get('_ubx_tail', b''))}")
    _debug_print(f"nmea_tail_bytes_waiting_for_next_read: {len(dev.get('_nmea_tail', b''))}")
    _debug_print(f"ubx_fix_type: {pvt.get('fix_type') if pvt else None}")
    _debug_print(f"gga_fix_quality: {gga.get('fix_q') if gga else None}")
    _debug_print(f"rmc_status: {rmc.get('status') if rmc else None}")


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
        if line:
            _debug_section("GPS_NMEA ignored")
            _debug_print(f"line: {line}")
            _debug_print("reason: not an NMEA sentence")
        return
    _debug_section("GPS_NMEA sentence")
    _debug_print(f"line: {line}")
    if not _nmea_checksum_valid(line):
        _debug_print("checksum: invalid, sentence dropped")
        logger.debug("GPS: dropped NMEA (checksum): %s", line[:96])
        return
    _debug_print("checksum: valid")
    body = line[1:].split("*", 1)[0]
    parts = body.split(",")
    tag = parts[0] if parts else ""
    _debug_print(f"tag: {tag}")
    _debug_print("fields:")
    for i, part in enumerate(parts):
        _debug_print(f"  [{i}] {part}")
    if tag.endswith("GGA") and len(parts) > 1:
        g = _parse_gga(parts)
        if g:
            g["_seen_ts"] = time.time()
            dev["gga"] = g
            _debug_dict("GPS_PARSE GGA", g, ["time", "lat", "lon", "alt", "sats", "fix_q", "_seen_ts"])
        else:
            _debug_section("GPS_PARSE GGA failed")
            _debug_print("reason: missing fields or invalid numeric/coordinate data")
    elif tag.endswith("RMC") and len(parts) > 1:
        r = _parse_rmc(parts)
        if r:
            r["_seen_ts"] = time.time()
            dev["rmc"] = r
            _debug_dict(
                "GPS_PARSE RMC",
                r,
                ["time", "status", "lat", "lon", "speed_ms", "course", "_seen_ts"],
            )
        else:
            _debug_section("GPS_PARSE RMC failed")
            _debug_print("reason: missing fields or invalid numeric/coordinate data")
    else:
        _debug_print("parser: unsupported sentence type, kept only as raw input")


def _nmea_feed_bytes(dev: dict, chunk: bytes) -> None:
    tail: bytes = dev.setdefault("_nmea_tail", b"")
    data = tail + chunk
    lines = data.split(b"\n")
    dev["_nmea_tail"] = lines[-1]
    tail_max = _nmea_tail_max_bytes()
    if len(dev["_nmea_tail"]) > tail_max:
        candidate = dev["_nmea_tail"][-tail_max:]
        last_start = candidate.rfind(b"$")
        if last_start >= 0:
            dev["_nmea_tail"] = candidate[last_start:]
            action = "kept_last_sentence_start"
        else:
            dev["_nmea_tail"] = b""
            action = "cleared_no_sentence_start"
        _debug_flow(
            dev,
            "nmea_tail_trimmed",
            max_bytes=tail_max,
            action=action,
            new_tail_bytes=len(dev["_nmea_tail"]),
        )
    _debug_flow(
        dev,
        "nmea_feed",
        bytes=len(chunk),
        previous_tail_bytes=len(tail),
        complete_lines=max(0, len(lines) - 1),
        new_tail_bytes=len(dev["_nmea_tail"]),
    )
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
    _debug_section("GPS_UBX frame")
    _debug_print(f"class: 0x{msg_class:02x}")
    _debug_print(f"id: 0x{msg_id:02x}")
    _debug_print(f"payload_bytes: {len(payload)}")
    if msg_class == 0x01 and msg_id == 0x07:
        pvt = _parse_nav_pvt(payload)
        if pvt is not None:
            dev["pvt"] = pvt
            _debug_dict(
                "GPS_PARSE UBX_NAV_PVT",
                pvt,
                [
                    "source",
                    "date",
                    "time",
                    "fix_type",
                    "num_sv",
                    "lat",
                    "lon",
                    "alt",
                    "height",
                    "h_acc",
                    "v_acc",
                    "g_speed",
                    "head_mot",
                    "vel_n",
                    "vel_e",
                    "vel_d",
                    "s_acc",
                    "head_acc",
                    "valid",
                    "flags",
                    "i_tow",
                    "_seen_ts",
                ],
            )
        else:
            _debug_print("parser: UBX_NAV_PVT parse failed")
    else:
        _debug_print("parser: unsupported UBX frame, kept only as raw input")


def _ubx_feed_bytes(dev: dict, chunk: bytes) -> None:
    data = dev.setdefault("_ubx_tail", b"") + chunk
    _debug_flow(
        dev,
        "ubx_feed",
        bytes=len(chunk),
        previous_tail_bytes=len(dev.get("_ubx_tail", b"")),
        buffered_bytes=len(data),
    )
    idx = 0
    while True:
        start = data.find(b"\xB5\x62", idx)
        if start < 0:
            dev["_ubx_tail"] = data[-1:] if data.endswith(b"\xB5") else b""
            if len(data) > idx:
                _debug_flow(dev, "ubx_no_more_frames", skipped_bytes=len(data) - idx, tail_bytes=len(dev["_ubx_tail"]))
            return
        if start > idx:
            _debug_flow(dev, "ubx_skip_prefix", skipped_bytes=start - idx)
        if len(data) - start < 8:
            dev["_ubx_tail"] = data[start:]
            _debug_flow(dev, "ubx_wait_header", tail_bytes=len(dev["_ubx_tail"]))
            return
        msg_class = data[start + 2]
        msg_id = data[start + 3]
        length = int.from_bytes(data[start + 4 : start + 6], "little")
        if length > 1024:
            _debug_flow(
                dev,
                "ubx_drop_bad_length",
                class_hex=f"0x{msg_class:02x}",
                id_hex=f"0x{msg_id:02x}",
                payload_bytes=length,
            )
            idx = start + 2
            continue
        end = start + 8 + length
        if len(data) < end:
            dev["_ubx_tail"] = data[start:]
            _debug_flow(
                dev,
                "ubx_wait_payload",
                class_hex=f"0x{msg_class:02x}",
                id_hex=f"0x{msg_id:02x}",
                payload_bytes=length,
                tail_bytes=len(dev["_ubx_tail"]),
            )
            return
        frame_body = data[start + 2 : start + 6 + length]
        ck_a, ck_b = _ubx_checksum(frame_body)
        if data[start + 6 + length] == ck_a and data[start + 7 + length] == ck_b:
            _ingest_ubx_frame(dev, msg_class, msg_id, data[start + 6 : start + 6 + length])
        else:
            _debug_section("GPS_UBX checksum failed")
            _debug_print(f"class: 0x{msg_class:02x}")
            _debug_print(f"id: 0x{msg_id:02x}")
            _debug_print(f"payload_bytes: {length}")
            _debug_print(f"expected_ck_a: 0x{ck_a:02x}")
            _debug_print(f"expected_ck_b: 0x{ck_b:02x}")
            _debug_print(f"actual_ck_a: 0x{data[start + 6 + length]:02x}")
            _debug_print(f"actual_ck_b: 0x{data[start + 7 + length]:02x}")
        idx = end


def _gps_build_return(dev: dict) -> Optional[list]:
    now = time.time()
    stale_max = _gps_row_stale_max_sec()

    pvt = dev.get("pvt")
    if pvt:
        pos_seen = float(pvt.get("_seen_ts", now))
        pos_age = max(0.0, now - pos_seen)
        if pos_age <= stale_max:
            fix_type = int(pvt.get("fix_type", 0))
            num_sv = int(pvt.get("num_sv", 0))
            status = "A" if fix_type >= 2 else "V"
            motion_valid = 1 if status == "A" else 0
            _debug_flow(dev, "gps_row_select", source="UBX_NAV_PVT", age_s=f"{pos_age:.3f}")
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
        _debug_flow(dev, "gps_row_drop_stale", source="UBX_NAV_PVT", age_s=f"{pos_age:.3f}", max_s=f"{stale_max:.3f}")

    gga = dev.get("gga")
    if not gga or int(gga.get("fix_q", 0)) < 1:
        _debug_flow(dev, "gps_row_select", source="NONE", reason="no_fresh_pvt_or_valid_gga")
        return None

    pos_seen = float(gga.get("_seen_ts", now))
    pos_age = max(0.0, now - pos_seen)
    if pos_age > stale_max:
        _debug_flow(dev, "gps_row_drop_stale", source="NMEA_GGA", age_s=f"{pos_age:.3f}", max_s=f"{stale_max:.3f}")
        return None

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

    _debug_flow(dev, "gps_row_select", source="NMEA", age_s=f"{pos_age:.3f}")
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
    available = (msb << 8) | lsb
    if available >= 4096:
        _debug_flow({}, "i2c_available_invalid", msb=f"0x{msb:02x}", lsb=f"0x{lsb:02x}", bytes=available)
        return 0
    return available


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


def _gps_feed_i2c_chunk(dev: dict, chunk: bytes, label: str = "I2C") -> int:
    _debug_dump_bytes(label, chunk)
    if chunk and all(b == 0xFF for b in chunk):
        _debug_flow(dev, "i2c_read_idle_ff", bytes=len(chunk), action="discard_and_clear_parser_tails")
        dev["_ubx_tail"] = b""
        dev["_nmea_tail"] = b""
        return 0

    _debug_flow(dev, "feed_ubx", bytes=len(chunk))
    _ubx_feed_bytes(dev, chunk)
    _debug_flow(dev, "feed_nmea", bytes=len(chunk))
    _nmea_feed_bytes(dev, chunk)
    return len(chunk)


def _gps_i2c_note_success(dev: dict) -> None:
    dev["_i2c_error_count"] = 0


def _gps_i2c_note_error(dev: dict, i2c_bus: Any, reason: str, exc: Exception | None = None) -> None:
    count = int(dev.get("_i2c_error_count", 0)) + 1
    dev["_i2c_error_count"] = count
    fields: dict[str, Any] = {"reason": reason, "count": count}
    if exc is not None:
        fields["error"] = repr(exc)
    _debug_flow(dev, "i2c_error_count", **fields)

    max_errors = _gps_i2c_max_consecutive_errors()
    if count < max_errors:
        return

    _debug_flow(dev, "i2c_reset_triggered", count=count, max_errors=max_errors)
    try:
        i2c_bus.reset_i2c()
    except Exception as reset_exc:
        _debug_flow(dev, "i2c_reset_error", error=repr(reset_exc))
        logger.warning("GPS I2C reset failed after %d consecutive errors: %s", count, reset_exc)
    finally:
        dev["_i2c_error_count"] = 0


def _ublox_enable_nav_pvt_i2c(i2c: Any, address: int) -> None:
    """Request UBX-NAV-PVT output on the DDC/I2C port; harmless if unsupported."""
    payload = bytes([0x01, 0x07, 1, 0, 0, 0, 0, 0])
    frame = _ubx_frame(0x06, 0x01, payload)
    try:
        i2c.writeto(address, frame)
    except Exception as exc:
        logger.debug("GPS: UBX NAV-PVT enable over I2C failed: %s", exc)


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
                if i == 0:
                    _debug_i2c_scan(i2c, addr)
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
                "_i2c_error_count": 0,
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


def init_gps() -> Any:
    """Open GNSS through u-blox DDC/I2C only."""
    return _init_gps_i2c_ublox()


def _gps_readdata_i2c(dev: dict) -> Optional[list]:
    from lib import i2c_bus

    addr = int(dev["addr"])
    _debug_flow(dev, "poll_start", kind=dev.get("kind"), addr=f"0x{addr:02x}")
    with i2c_bus.i2c_lock():
        i2c = i2c_bus.get_i2c()
        try:
            n = _ublox_ddc_bytes_available(i2c, addr)
            _debug_flow(dev, "i2c_available", bytes=n)
            _gps_i2c_note_success(dev)
        except OSError as exc:
            _debug_flow(dev, "i2c_available_error", error=repr(exc))
            logger.debug("GPS I2C bytes_available failed: %s", exc)
            recovery_n = _gps_i2c_recovery_read_bytes()
            if recovery_n > 0:
                _debug_flow(dev, "i2c_recovery_read_request", bytes=recovery_n)
                chunk = _ublox_ddc_read_stream(i2c, addr, recovery_n)
                consumed = _gps_feed_i2c_chunk(dev, chunk, label="I2C_RECOVERY")
                _debug_flow(dev, "i2c_recovery_read_result", bytes=len(chunk), consumed=consumed)
                if consumed > 0:
                    _gps_i2c_note_success(dev)
                    n = 0
                else:
                    _gps_i2c_note_error(dev, i2c_bus, "available_error_empty_recovery", exc)
                    n = 0
            else:
                _gps_i2c_note_error(dev, i2c_bus, "available_error", exc)
                n = 0
        except Exception as exc:
            _debug_flow(dev, "i2c_available_error", error=repr(exc))
            _gps_i2c_note_error(dev, i2c_bus, "available_error", exc)
            n = 0
        if n > 0:
            n = min(n, 256)
            try:
                _debug_flow(dev, "i2c_read_request", bytes=n)
                chunk = _ublox_ddc_read_stream(i2c, addr, n)
                consumed = _gps_feed_i2c_chunk(dev, chunk)
                if consumed > 0:
                    _gps_i2c_note_success(dev)
                else:
                    _gps_i2c_note_error(dev, i2c_bus, "empty_read_after_available")
            except OSError as exc:
                _debug_flow(dev, "i2c_read_error", error=repr(exc))
                _gps_i2c_note_error(dev, i2c_bus, "read_error", exc)
                logger.warning("GPS I2C read_stream failed (%s); next poll will retry", exc)
        else:
            _debug_flow(dev, "i2c_read_skip", reason="empty_fifo")
    _debug_parser_state(dev)
    _debug_latest_measurements(dev)
    row = _gps_build_return(dev)
    _debug_row(row)
    return row


def gps_readdata(dev: Optional[dict]) -> Optional[list]:
    if not dev:
        return None
    return _gps_readdata_i2c(dev)


def gps_terminate(dev: dict) -> None:
    return


if __name__ == "__main__":
    import time

    logging.basicConfig(level=logging.INFO, format="%(message)s")
    print("GPS: I2C u-blox only. Set GPS_I2C_ADDR if not 0x42.", flush=True)
    dev = init_gps()
    period = _gps_cli_period_sec()
    if dev is None:
        print(
            "GPS init failed. Check FSW_I2C_BUS, wiring, GPS_I2C_ADDR (default 0x42), antenna.",
            flush=True,
        )
        raise SystemExit(1)
    print(
        f"GPS: OK, streaming data flow every {period:.2f}s: "
        "I2C FIFO -> raw bytes -> UBX/NMEA parser -> gps row",
        flush=True,
    )
    _debug_runtime_config(dev)
    poll_count = 0
    try:
        while True:
            poll_count += 1
            _debug_section(f"GPS_POLL {poll_count}")
            row = gps_readdata(dev)
            if row is None:
                print("fix=no (watch GPS_FLOW/GPS_RAW/GPS_PARSE above)", flush=True)
            else:
                gt, alt, lat, lon, sats, fixq, st, spd, crs = row[:9]
                source = row[12] if len(row) >= 13 else "NMEA"
                print(
                    f"time={gt} lat={lat:.6f} lon={lon:.6f} alt_m={alt:.1f} "
                    f"sats={sats} fix={fixq} rmc={st} v_ms={spd:.2f} crs={crs:.1f} source={source}",
                    flush=True,
                )
            time.sleep(period)
    except KeyboardInterrupt:
        print("", flush=True)
    finally:
        gps_terminate(dev)
