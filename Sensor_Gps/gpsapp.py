"""GPS app with separated position and motion health for MotorApp."""

from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque
from math import atan2, cos, degrees, radians, sqrt
from typing import Optional

from lib import appargs, config, msgstructure


logger = logging.getLogger(__name__)
_GPS_SYNTH_WARNED = False


GPSAPP_RUNSTATUS = True
LAT = 0.0
LON = 0.0
ALT = 0.0
VELOCITY = 0.0
DIRECTION = 0.0  # deg, GPS ground-track direction; not IMU yaw.
SATS = 0
FIX_QUALITY = 0
RMC_STATUS = "V"
POS_HEALTH = 0
MOTION_HEALTH = 0

# Backward-compatible alias for tests/telemetry that still read GPS_HEALTH.
GPS_HEALTH = 0

GPS_POS_STALE_TIMEOUT_SEC = 1.5
GPS_MOTION_STALE_TIMEOUT_SEC = 1.5
GPS_DUPLICATE_TIME_MAX = 30
GPS_JUMP_MAX_SPEED = 200.0
GPS_MAX_VALID_SPEED = 40.0
GPS_MIN_COURSE_SPEED = 0.5
GPS_POS_DERIVED_MIN_SPEED = 2.0
GPS_SPEED_POSITION_DIFF_MAX = 12.0
GPS_SPEED_SPIKE_MAX = 25.0
GPS_COURSE_SPIKE_MAX_DEG = 160.0
GPS_LOCAL_RADIUS_M = 5_000.0
GPS_MAX_H_ACC_M = 25.0
GPS_MAX_S_ACC_MPS = 3.0
GPS_MAX_HEAD_ACC_DEG = 60.0
GPS_MIN_POSITION_DELTA_SEC = 0.02

_last_update_ts = 0.0
_last_fix: Optional[dict] = None
_last_good_position: Optional[dict] = None
_last_good_motion: Optional[dict] = None
_anchor_position: Optional[dict] = None
_last_gps_time: Optional[str] = None
_duplicate_gps_time_count = 0
_gps_lock = threading.Lock()
_lat_window = deque(maxlen=5)
_lon_window = deque(maxlen=5)
_speed_window = deque(maxlen=5)

# SIM inject (FlightLogic -> MID_flight_gps_sim): bench / SIM ACTIVATE mode only.
_sim_fix: Optional[dict] = None


def _gps_rate_hz() -> float:
    try:
        return max(0.1, float(config.GPS_RATE_HZ))
    except (TypeError, ValueError):
        return 10.0


def _gps_period_sec() -> float:
    return 1.0 / _gps_rate_hz()


def _comm_tick_interval() -> int:
    # Keep comm GGA downlink near 1 Hz regardless of GPS polling rate.
    return max(1, int(round(_gps_rate_hz())))


def _median(values) -> float:
    arr = sorted(float(v) for v in values)
    n = len(arr)
    if n == 0:
        return 0.0
    mid = n // 2
    if n % 2 == 1:
        return float(arr[mid])
    return float((arr[mid - 1] + arr[mid]) / 2.0)


def _median5_update(value: float, window: deque) -> float:
    window.append(float(value))
    return _median(window)


def _set_sim_fix(payload: Optional[dict]) -> None:
    global _sim_fix
    with _gps_lock:
        _sim_fix = payload


def _get_sim_fix() -> Optional[dict]:
    with _gps_lock:
        return dict(_sim_fix) if _sim_fix is not None else None


def _sim_gps_row() -> Optional[list]:
    """Return a gps_readdata-shaped row when SIM inject is active."""
    fix = _get_sim_fix()
    if fix is None:
        return None
    now_time = time.strftime("%H%M%S", time.gmtime())
    return [
        now_time,
        fix["alt"],
        fix["lat"],
        fix["lon"],
        fix["sats"],
        fix["fix_quality"],
        "A",
        fix["speed"],
        fix["course"],
        1,
        0.0,
        0.0,
        "SIM",
    ]


def _apply_flight_gps_sim(data: str) -> None:
    """Parse `lat,lon,course_deg,speed_m_s[,alt_m]` or CLEAR."""
    global _sim_fix
    text = data.strip()
    if text.upper() == "CLEAR":
        _set_sim_fix(None)
        logger.info("GPS SIM inject cleared (hardware path resumes)")
        return
    parts = [x.strip() for x in text.split(",")]
    if len(parts) not in (4, 5):
        logger.warning("GPS SIM inject: expected lat,lon,course,speed[,alt] got %r", data)
        return
    try:
        lat = float(parts[0])
        lon = float(parts[1])
        course = float(parts[2]) % 360.0
        speed = float(parts[3])
        alt = float(parts[4]) if len(parts) == 5 else ALT
    except ValueError:
        logger.warning("GPS SIM inject: bad numeric fields %r", data)
        return
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return
    if lat == 0.0 and lon == 0.0:
        return
    if speed < 0.0 or speed > GPS_MAX_VALID_SPEED + 1.0:
        logger.warning("GPS SIM inject: speed out of range %s", speed)
        return
    _set_sim_fix(
        {
            "lat": lat,
            "lon": lon,
            "course": course,
            "speed": speed,
            "alt": alt,
            "sats": 8,
            "fix_quality": 1,
        }
    )
    logger.info(
        "GPS SIM inject: lat=%.6f lon=%.6f crs=%.1fdeg V=%.2fm/s alt=%.1fm",
        lat,
        lon,
        course,
        speed,
        alt,
    )


def command_handler(recv_msg: str) -> None:
    global GPSAPP_RUNSTATUS
    unpacked = msgstructure.unpack_msg(recv_msg)
    if unpacked is False:
        return
    if unpacked.msg_id == appargs.MainAppArg.MID_TerminateProcess:
        GPSAPP_RUNSTATUS = False
    elif unpacked.msg_id == appargs.GpsAppArg.MID_flight_gps_sim:
        _apply_flight_gps_sim(unpacked.data or "")


def _wrap_180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def _distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    dlat = (lat2 - lat1) * 111_000.0
    dlon = (lon2 - lon1) * 111_000.0 * cos(radians((lat1 + lat2) / 2.0))
    return sqrt(dlat * dlat + dlon * dlon)


def _bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    d_n = (lat2 - lat1) * 111_000.0
    d_e = (lon2 - lon1) * 111_000.0 * cos(radians((lat1 + lat2) / 2.0))
    return (degrees(atan2(d_e, d_n)) + 360.0) % 360.0


def _is_finite(value: float) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _gps_time_ok(gps_time: str) -> bool:
    text = str(gps_time or "")
    if len(text) != 6 or not text.isdigit():
        return False
    hh = int(text[0:2])
    mm = int(text[2:4])
    ss = int(text[4:6])
    return 0 <= hh <= 23 and 0 <= mm <= 59 and 0 <= ss <= 59


def _gps_time_seconds(gps_time: str) -> Optional[int]:
    if not _gps_time_ok(gps_time):
        return None
    text = str(gps_time)
    return int(text[0:2]) * 3600 + int(text[2:4]) * 60 + int(text[4:6])


def _duplicate_time_ok(gps_time: str) -> bool:
    global _last_gps_time, _duplicate_gps_time_count
    if gps_time == _last_gps_time:
        _duplicate_gps_time_count += 1
    else:
        _last_gps_time = gps_time
        _duplicate_gps_time_count = 0
    return _duplicate_gps_time_count <= GPS_DUPLICATE_TIME_MAX


def _position_basic_ok(lat: float, lon: float, fix_quality: int, sats: int) -> bool:
    return (
        _is_finite(lat)
        and _is_finite(lon)
        and -90.0 <= lat <= 90.0
        and -180.0 <= lon <= 180.0
        and lat != 0.0
        and lon != 0.0
        and fix_quality >= 1
        and sats >= 4
    )


def _is_valid_fix(lat: float, lon: float, fix_quality: int, sats: int, rmc_status: str) -> bool:
    """Legacy helper retained for tests; position health itself no longer depends on RMC."""
    return _position_basic_ok(lat, lon, fix_quality, sats) and str(rmc_status).upper() == "A"


def _is_jump(lat: float, lon: float, now: float) -> bool:
    """Legacy jump helper retained for tests."""
    global _last_fix
    if _last_fix is None:
        _last_fix = {"lat": lat, "lon": lon, "ts": now}
        return False
    dt = max(1e-3, now - _last_fix["ts"])
    dist = _distance_m(_last_fix["lat"], _last_fix["lon"], lat, lon)
    _last_fix = {"lat": lat, "lon": lon, "ts": now}
    return (dist / dt) > GPS_JUMP_MAX_SPEED


def _position_delta(lat: float, lon: float, now: float, gps_time_s: Optional[int]) -> Optional[dict]:
    if _last_good_position is None:
        return None
    prev_gps_time_s = _last_good_position.get("gps_time_s")
    if gps_time_s is not None and prev_gps_time_s is not None:
        dt = float(gps_time_s - int(prev_gps_time_s))
        if dt < -43_200.0:
            dt += 86_400.0
    else:
        dt = now - float(_last_good_position["ts"])
    if dt <= GPS_MIN_POSITION_DELTA_SEC:
        return None
    prev_lat = float(_last_good_position["lat"])
    prev_lon = float(_last_good_position["lon"])
    dist = _distance_m(prev_lat, prev_lon, lat, lon)
    return {
        "dt": dt,
        "dist": dist,
        "speed": dist / dt,
        "course": _bearing_deg(prev_lat, prev_lon, lat, lon),
    }


def _position_health(
    lat: float,
    lon: float,
    fix_quality: int,
    sats: int,
    gps_time: str,
    pos_age: float,
    now: float,
    source: str = "NMEA",
    fix_type: Optional[int] = None,
    h_acc: Optional[float] = None,
) -> tuple[int, Optional[dict]]:
    gps_time_s = _gps_time_seconds(gps_time)
    if not _position_basic_ok(lat, lon, fix_quality, sats):
        return 0, None
    if source == "UBX_NAV_PVT":
        if fix_type is None or int(fix_type) < 2:
            return 0, None
        if h_acc is not None and _is_finite(h_acc) and float(h_acc) > GPS_MAX_H_ACC_M:
            return 0, None
    if str(source).upper() == "SIM":
        if gps_time_s is None:
            return 0, None
        delta = _position_delta(lat, lon, now, gps_time_s)
        return 1, delta
    if gps_time_s is None:
        return 0, None
    if not _duplicate_time_ok(gps_time):
        return 0, None
    if pos_age > GPS_POS_STALE_TIMEOUT_SEC:
        return 0, None
    if _anchor_position is not None:
        anchor_dist = _distance_m(float(_anchor_position["lat"]), float(_anchor_position["lon"]), lat, lon)
        if anchor_dist > GPS_LOCAL_RADIUS_M:
            return 0, None

    delta = _position_delta(lat, lon, now, gps_time_s)
    if delta is not None and float(delta["speed"]) > GPS_JUMP_MAX_SPEED:
        return 0, delta
    return 1, delta


def _motion_health(
    speed: float,
    course: float,
    rmc_status: str,
    motion_present: bool,
    motion_age: float,
    position_delta: Optional[dict],
    source: str = "NMEA",
    s_acc: Optional[float] = None,
    head_acc: Optional[float] = None,
) -> int:
    if not motion_present or str(rmc_status).upper() != "A":
        return 0
    if motion_age > GPS_MOTION_STALE_TIMEOUT_SEC:
        return 0
    if not _is_finite(speed) or not _is_finite(course):
        return 0
    if speed < GPS_MIN_COURSE_SPEED or speed > GPS_MAX_VALID_SPEED:
        return 0
    if course < 0.0 or course > 360.0:
        return 0
    if source == "UBX_NAV_PVT":
        if s_acc is not None and _is_finite(s_acc) and float(s_acc) > GPS_MAX_S_ACC_MPS:
            return 0
        if speed > 2.0 and head_acc is not None and _is_finite(head_acc) and float(head_acc) > GPS_MAX_HEAD_ACC_DEG:
            return 0

    if str(source).upper() == "SIM":
        if str(rmc_status).upper() != "A" or not motion_present:
            return 0
        if not _is_finite(speed) or not _is_finite(course):
            return 0
        if course < 0.0 or course > 360.0:
            return 0
        if speed < 0.0 or speed > GPS_MAX_VALID_SPEED:
            return 0
        return 1

    if position_delta is not None:
        derived_speed = float(position_delta["speed"])
        if derived_speed > GPS_MIN_COURSE_SPEED:
            if abs(speed - derived_speed) > GPS_SPEED_POSITION_DIFF_MAX:
                return 0

    if _last_good_motion is not None:
        prev_speed = float(_last_good_motion["speed"])
        prev_course = float(_last_good_motion["course"])
        if abs(speed - prev_speed) > GPS_SPEED_SPIKE_MAX:
            return 0
        if speed > 2.0 and abs(_wrap_180(course - prev_course)) > GPS_COURSE_SPIKE_MAX_DEG:
            return 0

    return 1


def _synthetic_read():
    """Driver unavailable: zeros / invalid fix (no fake track)."""
    return ["000000", 0.0, 0.0, 0.0, 0, 0, "V", 0.0, 0.0, 0, 0.0, 0.0]


def _read_gps():
    global _GPS_SYNTH_WARNED
    sim_row = _sim_gps_row()
    if sim_row is not None:
        return sim_row
    try:
        from Sensor_Gps import gps as gps_driver  # type: ignore

        if not hasattr(_read_gps, "_inst"):
            _read_gps._inst = gps_driver.init_gps()  # type: ignore[attr-defined]
        if _read_gps._inst is None:  # type: ignore[attr-defined]
            if not _GPS_SYNTH_WARNED:
                logger.warning(
                    "GPS: I2C u-blox init failed; lat/lon/speed forced to 0 (invalid fix). "
                    "Check FSW_I2C_BUS, GPS_I2C_ADDR (default 0x42), antenna."
                )
                _GPS_SYNTH_WARNED = True
            return _synthetic_read()
        data = gps_driver.gps_readdata(_read_gps._inst)  # type: ignore[attr-defined]
        if data is None:
            return None
        return data
    except Exception:
        if not _GPS_SYNTH_WARNED:
            logger.warning("GPS: driver error; lat/lon/speed forced to 0")
            _GPS_SYNTH_WARNED = True
        return _synthetic_read()


def _parse_sample(sample) -> Optional[dict]:
    if sample is None or len(sample) < 9:
        return None
    gps_time, alt, lat, lon, sats, fix_quality, rmc_status, speed, course = sample[:9]
    try:
        out = {
            "gps_time": str(gps_time),
            "alt": float(alt),
            "lat": float(lat),
            "lon": float(lon),
            "sats": int(float(sats)),
            "fix_quality": int(float(fix_quality)),
            "rmc_status": str(rmc_status).upper(),
            "speed": float(speed),
            "course": float(course),
            "motion_present": str(rmc_status).upper() == "A",
            "pos_age": 0.0,
            "motion_age": 0.0,
            "source": "NMEA",
            "fix_type": None,
            "h_acc": None,
            "v_acc": None,
            "s_acc": None,
            "head_acc": None,
            "vel_n": None,
            "vel_e": None,
            "valid_flags": None,
        }
        if len(sample) >= 10:
            out["motion_present"] = int(float(sample[9])) > 0
        if len(sample) >= 11:
            out["pos_age"] = float(sample[10])
        if len(sample) >= 12:
            out["motion_age"] = float(sample[11])
        if len(sample) >= 13:
            out["source"] = str(sample[12])
        if len(sample) >= 14:
            out["fix_type"] = int(float(sample[13]))
        if len(sample) >= 15:
            out["h_acc"] = float(sample[14])
        if len(sample) >= 16:
            out["v_acc"] = float(sample[15])
        if len(sample) >= 17:
            out["s_acc"] = float(sample[16])
        if len(sample) >= 18:
            out["head_acc"] = float(sample[17])
        if len(sample) >= 19:
            out["vel_n"] = float(sample[18])
        if len(sample) >= 20:
            out["vel_e"] = float(sample[19])
        if len(sample) >= 21:
            out["valid_flags"] = int(float(sample[20]))
        return out
    except (TypeError, ValueError):
        return None


def _hold_or_update_position(lat: float, lon: float, pos_health: int, now: float) -> tuple[float, float]:
    global _last_good_position, _anchor_position
    if pos_health:
        gps_time_s = _gps_time_seconds(_last_gps_time or "")
        _last_good_position = {"lat": lat, "lon": lon, "ts": now, "gps_time_s": gps_time_s}
        if _anchor_position is None:
            _anchor_position = {"lat": lat, "lon": lon}
        return lat, lon
    if _last_good_position is not None:
        return float(_last_good_position["lat"]), float(_last_good_position["lon"])
    return LAT, LON


def _hold_or_update_motion(speed: float, course: float, motion_health: int) -> tuple[float, float]:
    global _last_good_motion
    course = course % 360.0
    if motion_health:
        _last_good_motion = {"speed": speed, "course": course}
        return speed, course
    if _last_good_motion is not None:
        return float(_last_good_motion["speed"]), float(_last_good_motion["course"])
    return 0.0, 0.0


def read_and_send_gps_data(main_queue) -> None:
    global LAT, LON, ALT, VELOCITY, DIRECTION, SATS, FIX_QUALITY, RMC_STATUS
    global POS_HEALTH, MOTION_HEALTH, GPS_HEALTH, _last_update_ts
    tick = 0
    period = _gps_period_sec()
    comm_tick_interval = _comm_tick_interval()
    while GPSAPP_RUNSTATUS:
        sample = _parse_sample(_read_gps())
        now = time.time()

        if sample is not None:
            sample["lat"] = _median5_update(sample["lat"], _lat_window)
            sample["lon"] = _median5_update(sample["lon"], _lon_window)
            sample["speed"] = _median5_update(sample["speed"], _speed_window)
            pos_health, position_delta = _position_health(
                sample["lat"],
                sample["lon"],
                sample["fix_quality"],
                sample["sats"],
                sample["gps_time"],
                sample["pos_age"],
                now,
                sample["source"],
                sample["fix_type"],
                sample["h_acc"],
            )
            motion_health = _motion_health(
                sample["speed"],
                sample["course"],
                sample["rmc_status"],
                sample["motion_present"],
                sample["motion_age"],
                position_delta,
                sample["source"],
                sample["s_acc"],
                sample["head_acc"],
            )
            motion_speed = sample["speed"]
            motion_course = sample["course"]
            if (
                motion_health <= 0
                and pos_health > 0
                and position_delta is not None
                and GPS_POS_DERIVED_MIN_SPEED <= float(position_delta["speed"]) <= GPS_MAX_VALID_SPEED
            ):
                motion_speed = float(position_delta["speed"])
                motion_course = float(position_delta["course"])
                motion_health = 1

            lat, lon = _hold_or_update_position(sample["lat"], sample["lon"], pos_health, now)
            velocity, direction = _hold_or_update_motion(motion_speed, motion_course, motion_health)

            with _gps_lock:
                LAT = lat
                LON = lon
                ALT = sample["alt"]
                SATS = sample["sats"]
                FIX_QUALITY = sample["fix_quality"]
                RMC_STATUS = sample["rmc_status"]
                VELOCITY = velocity
                DIRECTION = direction
                POS_HEALTH = pos_health
                MOTION_HEALTH = motion_health
                GPS_HEALTH = POS_HEALTH
                _last_update_ts = now
        else:
            if now - _last_update_ts > GPS_POS_STALE_TIMEOUT_SEC:
                POS_HEALTH = 0
                MOTION_HEALTH = 0
                GPS_HEALTH = 0

        if now - _last_update_ts > GPS_POS_STALE_TIMEOUT_SEC:
            POS_HEALTH = 0
            MOTION_HEALTH = 0
            GPS_HEALTH = 0

        payload_motor = f"{LAT},{LON},{DIRECTION},{VELOCITY},{POS_HEALTH},{MOTION_HEALTH}"
        msgstructure.send_msg(
            main_queue,
            appargs.GpsAppArg.AppID,
            appargs.MotorAppArg.AppID,
            appargs.GpsAppArg.MID_motor_gps,
            payload_motor,
        )
        tick += 1
        if tick >= comm_tick_interval:
            tick = 0
            msgstructure.send_msg(
                main_queue,
                appargs.GpsAppArg.AppID,
                appargs.CommAppArg.AppID,
                appargs.GpsAppArg.MID_comm_gga,
                f"000000,{ALT},{LAT},{LON},{SATS}",
            )
        time.sleep(period)


def gpsapp_main(main_queue, main_pipe) -> None:
    t = threading.Thread(target=read_and_send_gps_data, args=(main_queue,), daemon=True)
    t.start()
    poll_period = _gps_period_sec()
    try:
        while GPSAPP_RUNSTATUS:
            try:
                has_msg = main_pipe.poll(poll_period)
            except (KeyboardInterrupt, EOFError, OSError):
                break
            if has_msg:
                command_handler(main_pipe.recv())
    except KeyboardInterrupt:
        pass
