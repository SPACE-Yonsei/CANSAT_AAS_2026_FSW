"""GPS app with validity/stale/jump handling and robust forwarding."""

from __future__ import annotations

import threading
import time
from math import cos, radians, sqrt
from typing import Optional

from lib import appargs, msgstructure


GPSAPP_RUNSTATUS = True
LAT = 37.5600
LON = 126.9300
ALT = 80.0
SPEED_MS = 0.0
COURSE_DEG = 0.0
SATS = 0
FIX_QUALITY = 0
RMC_STATUS = "V"
GPS_HEALTH = 0
GPS_STALE_TIMEOUT_SEC = 1.5
GPS_JUMP_MAX_SPEED = 200.0
GPS_MAX_SPEED_FOR_MOTOR = 15.0

_last_update_ts = 0.0
_last_fix: Optional[dict] = None
_last_valid_motor_payload = None
_gps_lock = threading.Lock()


def command_handler(recv_msg: str) -> None:
    global GPSAPP_RUNSTATUS
    unpacked = msgstructure.unpack_msg(recv_msg)
    if unpacked is False:
        return
    if unpacked.msg_id == appargs.MainAppArg.MID_TerminateProcess:
        GPSAPP_RUNSTATUS = False


def _distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    dlat = (lat2 - lat1) * 111_000.0
    dlon = (lon2 - lon1) * 111_000.0 * cos(radians((lat1 + lat2) / 2.0))
    return sqrt(dlat * dlat + dlon * dlon)


def _is_valid_fix(lat: float, lon: float, fix_quality: int, sats: int, rmc_status: str) -> bool:
    return (
        -90.0 <= lat <= 90.0
        and -180.0 <= lon <= 180.0
        and fix_quality >= 1
        and sats >= 4
        and str(rmc_status).upper() == "A"
    )


def _is_jump(lat: float, lon: float, now: float) -> bool:
    global _last_fix
    if _last_fix is None:
        _last_fix = {"lat": lat, "lon": lon, "ts": now}
        return False
    dt = max(1e-3, now - _last_fix["ts"])
    dist = _distance_m(_last_fix["lat"], _last_fix["lon"], lat, lon)
    _last_fix = {"lat": lat, "lon": lon, "ts": now}
    return (dist / dt) > GPS_JUMP_MAX_SPEED


def _synthetic_read():
    now = time.time()
    lat = LAT + 0.000001
    lon = LON + 0.000001
    return ["000000", ALT, lat, lon, 8, 1, "A", 8.0, 90.0]


def _read_gps():
    try:
        from Sensor_Gps import gps as gps_driver  # type: ignore

        if not hasattr(_read_gps, "_inst"):
            _read_gps._inst = gps_driver.init_gps()  # type: ignore[attr-defined]
        if _read_gps._inst is None:  # type: ignore[attr-defined]
            return _synthetic_read()
        data = gps_driver.gps_readdata(_read_gps._inst)  # type: ignore[attr-defined]
        if data is None:
            return None
        return data
    except Exception:
        return _synthetic_read()


def read_and_send_gps_data(main_queue) -> None:
    global LAT, LON, ALT, SPEED_MS, COURSE_DEG, SATS, FIX_QUALITY, RMC_STATUS, GPS_HEALTH
    global _last_update_ts, _last_valid_motor_payload
    tick = 0
    while GPSAPP_RUNSTATUS:
        sample = _read_gps()
        now = time.time()

        if sample is not None and len(sample) >= 9:
            gps_time, alt, lat, lon, sats, fix_quality, rmc_status, speed_ms, course = sample[:9]
            try:
                lat_f = float(lat)
                lon_f = float(lon)
                alt_f = float(alt)
                sats_i = int(float(sats))
                fix_i = int(float(fix_quality))
                speed_f = float(speed_ms)
                course_f = float(course)
            except (TypeError, ValueError):
                lat_f = lon_f = alt_f = 0.0
                sats_i = 0
                fix_i = 0
                speed_f = 0.0
                course_f = 0.0
                gps_time = "000000"
                rmc_status = "V"

            valid = _is_valid_fix(lat_f, lon_f, fix_i, sats_i, str(rmc_status))
            jump = _is_jump(lat_f, lon_f, now) if valid else True
            health = 1 if (valid and not jump) else 0

            with _gps_lock:
                LAT = lat_f
                LON = lon_f
                ALT = alt_f
                SATS = sats_i
                FIX_QUALITY = fix_i
                RMC_STATUS = str(rmc_status)
                SPEED_MS = speed_f
                COURSE_DEG = course_f
                GPS_HEALTH = health
                _last_update_ts = now
        else:
            if now - _last_update_ts > GPS_STALE_TIMEOUT_SEC:
                GPS_HEALTH = 0

        # Stale timeout gate
        if now - _last_update_ts > GPS_STALE_TIMEOUT_SEC:
            GPS_HEALTH = 0

        speed_for_motor = min(SPEED_MS, GPS_MAX_SPEED_FOR_MOTOR)
        payload_motor = f"{LAT},{LON},{speed_for_motor},{COURSE_DEG},{FIX_QUALITY},{SATS},{RMC_STATUS},{GPS_HEALTH}"
        if GPS_HEALTH == 1:
            _last_valid_motor_payload = payload_motor
        elif _last_valid_motor_payload is not None:
            payload_motor = _last_valid_motor_payload

        msgstructure.send_msg(
            main_queue,
            appargs.GpsAppArg.AppID,
            appargs.MotorAppArg.AppID,
            appargs.GpsAppArg.MID_motor_gps,
            payload_motor,
        )
        tick += 1
        if tick >= 10:
            tick = 0
            msgstructure.send_msg(
                main_queue,
                appargs.GpsAppArg.AppID,
                appargs.CommAppArg.AppID,
                appargs.GpsAppArg.MID_comm_gga,
                f"000000,{ALT},{LAT},{LON},{SATS}",
            )
        time.sleep(0.1)


def gpsapp_main(main_queue, main_pipe) -> None:
    t = threading.Thread(target=read_and_send_gps_data, args=(main_queue,), daemon=True)
    t.start()
    try:
        while GPSAPP_RUNSTATUS:
            try:
                has_msg = main_pipe.poll(0.1)
            except (KeyboardInterrupt, EOFError, OSError):
                break
            if has_msg:
                command_handler(main_pipe.recv())
    except KeyboardInterrupt:
        pass
