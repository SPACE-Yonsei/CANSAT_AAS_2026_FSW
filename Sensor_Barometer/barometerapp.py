"""Barometer app with filtering, calibration offset, and health/stale checks."""

from __future__ import annotations

import logging
import os
import time
import threading
from collections import deque
from typing import Optional

from lib import appargs, config, msgstructure, prevstate, sensorlog


logger = logging.getLogger(__name__)


BAROMETERAPP_RUNSTATUS = True
ALTITUDE = 0.0
TEMPERATURE = 0.0
PRESSURE = 0.0
BAROMETER_OFFSET = 0.0
BAROMETER_HEALTH = 1
BAROMETER_STALE_TIMEOUT_SEC = 1.0
_last_sample_ts = 0.0
_baro_lock = threading.Lock()
_alt_window = deque(maxlen=5)
_tmp_window = deque(maxlen=5)
_prs_window = deque(maxlen=5)
_baro_hw = None
_last_baro_read_warn_ts = 0.0


def _barometer_rate_hz() -> float:
    try:
        return max(0.1, float(config.BAROMETER_RATE_HZ))
    except (TypeError, ValueError):
        return 10.0


def _barometer_period_sec() -> float:
    return 1.0 / _barometer_rate_hz()


def command_handler(main_queue, recv_msg: str, _barometer_instance=None) -> None:
    global BAROMETERAPP_RUNSTATUS, BAROMETER_OFFSET, ALTITUDE
    unpacked = msgstructure.unpack_msg(recv_msg)
    if unpacked is False:
        return
    if unpacked.msg_id == appargs.MainAppArg.MID_TerminateProcess:
        BAROMETERAPP_RUNSTATUS = False
    elif unpacked.msg_id == appargs.CommAppArg.MID_RouteCmd_CAL:
        # Comm forwards either "<value>" (single token) or "CAL,<value>" (legacy).
        # Accept both: first numeric token is treated as additive offset delta.
        # If no numeric token is present, snap altitude origin to current ALTITUDE.
        parts = [x.strip() for x in (unpacked.data or "").split(",") if x.strip()]
        delta: Optional[float] = None
        for token in parts:
            try:
                delta = float(token)
                break
            except ValueError:
                continue
        if delta is not None:
            BAROMETER_OFFSET += delta
        else:
            BAROMETER_OFFSET = ALTITUDE
        prevstate.update_altcal(BAROMETER_OFFSET)
        msgstructure.send_msg(
            main_queue,
            appargs.BarometerAppArg.AppID,
            appargs.FlightlogicAppArg.AppID,
            appargs.CommAppArg.MID_RouteCmd_CAL,
            "RESET",
        )


def _median(values):
    if not values:
        return 0.0
    arr = sorted(values)
    mid = len(arr) // 2
    if len(arr) % 2 == 1:
        return float(arr[mid])
    return float((arr[mid - 1] + arr[mid]) / 2.0)


def _synthetic_raw():
    """No-hardware / read-failure path: zeros (no fake physics)."""
    return 0.0, 0.0, 0.0


def read_barometer_data() -> None:
    global ALTITUDE, TEMPERATURE, PRESSURE, BAROMETER_HEALTH, _last_sample_ts, _baro_hw
    global _last_baro_read_warn_ts
    period = _barometer_period_sec()
    while BAROMETERAPP_RUNSTATUS:
        try:
            hardware_ok = False
            try:
                from Sensor_Barometer import barometer as baro_driver  # type: ignore

                if _baro_hw is None:
                    try:
                        _baro_hw = baro_driver.init_bmp()
                    except Exception as exc:
                        logger.warning(
                            "Barometer: hardware init failed (%s); pressure/temp/alt forced to 0. "
                            "If i2cdetect shows 0x77 on bus 1 but this fails, try: "
                            "pip install adafruit-extended-bus && export FSW_I2C_BUS=1. "
                            "BARO_I2C_ADDR=0x76 if BMP SDO=GND. (i2c 0x42 is often GNSS Click, not INA.)",
                            exc,
                        )
                        _baro_hw = False
                if _baro_hw is not False:
                    try:
                        prs_raw, tmp_raw, alt_raw = baro_driver.read_bmp(_baro_hw)
                        hardware_ok = True
                    except Exception as exc:
                        now = time.time()
                        if now - _last_baro_read_warn_ts >= 3.0:
                            logger.warning(
                                "Barometer: BMP read failed (%s); zero frame (check I2C/addr %s)",
                                exc,
                                os.environ.get("BARO_I2C_ADDR", "0x77"),
                            )
                            _last_baro_read_warn_ts = now
                        prs_raw, tmp_raw, alt_raw = _synthetic_raw()
                else:
                    prs_raw, tmp_raw, alt_raw = _synthetic_raw()
            except Exception:
                prs_raw, tmp_raw, alt_raw = _synthetic_raw()

            sensorlog.log_barometer_raw(float(prs_raw), float(tmp_raw), float(alt_raw))

            _prs_window.append(float(prs_raw))
            _tmp_window.append(float(tmp_raw))
            _alt_window.append(float(alt_raw))

            prs = _median(_prs_window)
            tmp = _median(_tmp_window)
            alt = _median(_alt_window) - BAROMETER_OFFSET

            with _baro_lock:
                PRESSURE = prs
                TEMPERATURE = tmp
                ALTITUDE = alt
                _last_sample_ts = time.time()
                BAROMETER_HEALTH = 1 if hardware_ok else 0
        except Exception:
            BAROMETER_HEALTH = 0
        time.sleep(period)


def send_barometer_data(main_queue) -> None:
    global BAROMETER_HEALTH
    period = _barometer_period_sec()
    comm_tick_interval = max(1, int(round(_barometer_rate_hz())))
    tick = 0
    while BAROMETERAPP_RUNSTATUS:
        if time.time() - _last_sample_ts > BAROMETER_STALE_TIMEOUT_SEC:
            BAROMETER_HEALTH = 0
        with _baro_lock:
            alt = ALTITUDE
            prs = PRESSURE
            tmp = TEMPERATURE
            health = int(BAROMETER_HEALTH)
        msgstructure.send_msg(
            main_queue,
            appargs.BarometerAppArg.AppID,
            appargs.FlightlogicAppArg.AppID,
            appargs.BarometerAppArg.MID_flight_alt,
            f"{alt},{health}",
        )
        msgstructure.send_msg(
            main_queue,
            appargs.BarometerAppArg.AppID,
            appargs.MotorAppArg.AppID,
            appargs.BarometerAppArg.MID_motor_alt,
            f"{alt},{health}",
        )
        tick += 1
        if tick >= comm_tick_interval:
            tick = 0
            msgstructure.send_msg(
                main_queue,
                appargs.BarometerAppArg.AppID,
                appargs.CommAppArg.AppID,
                appargs.BarometerAppArg.MID_comm_alt,
                f"{prs},{tmp},{alt}",
            )
        time.sleep(period)


def barometerapp_main(main_queue, main_pipe) -> None:
    global BAROMETER_OFFSET
    prevstate.init_prevstate()
    BAROMETER_OFFSET = prevstate.PREV_ALT_CAL
    t1 = threading.Thread(target=read_barometer_data, daemon=True)
    t2 = threading.Thread(target=send_barometer_data, args=(main_queue,), daemon=True)
    t1.start()
    t2.start()
    try:
        while BAROMETERAPP_RUNSTATUS:
            try:
                has_msg = main_pipe.poll(0.1)
            except (KeyboardInterrupt, EOFError, OSError):
                break
            if has_msg:
                recv_msg = main_pipe.recv()
                command_handler(main_queue, recv_msg, None)
    except KeyboardInterrupt:
        pass
