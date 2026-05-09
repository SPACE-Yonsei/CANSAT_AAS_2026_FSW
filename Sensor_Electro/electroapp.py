"""Electro app with sensor read fallback, stale/health handling, and robust send."""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from typing import Optional

from lib import appargs, msgstructure, sensorlog


logger = logging.getLogger(__name__)


ELECTROAPP_RUNSTATUS = True
VOLT = 0.0
CURR = 0.0
PWR = 0.0
ELECTRO_HEALTH = 0
ELECTRO_STALE_TIMEOUT_SEC = 2.5
_last_update_ts = 0.0
_reader = None  # dict (hardware) | False (synthetic-only) | None (not probed yet)
_electro_lock = threading.Lock()
_last_electro_read_warn_ts = 0.0
_volt_window = deque(maxlen=5)
_curr_window = deque(maxlen=5)
_pwr_window = deque(maxlen=5)


def command_handler(recv_msg: str) -> None:
    global ELECTROAPP_RUNSTATUS
    unpacked = msgstructure.unpack_msg(recv_msg)
    if unpacked is False:
        return
    if unpacked.msg_id == appargs.MainAppArg.MID_TerminateProcess:
        ELECTROAPP_RUNSTATUS = False


def _synthetic_read() -> tuple[float, float, float]:
    """INA unavailable or read failure: zeros (no fake power telemetry)."""
    return 0.0, 0.0, 0.0


def _median(values) -> float:
    if not values:
        return 0.0
    arr = sorted(float(v) for v in values)
    n = len(arr)
    mid = n // 2
    if n % 2 == 1:
        return float(arr[mid])
    return float((arr[mid - 1] + arr[mid]) / 2.0)


def _median5_update(value: float, window: deque) -> float:
    window.append(float(value))
    return _median(window)


def _read_sensor() -> Optional[tuple[float, float, float]]:
    global _reader, _last_electro_read_warn_ts
    try:
        from Sensor_Electro import electro as electro_driver  # type: ignore

        if _reader is None:
            try:
                _reader = electro_driver.init_INA228()
            except Exception as exc:
                hint = ""
                if isinstance(exc, ImportError) and "ina228" in str(exc).lower():
                    hint = " Install: pip install adafruit-circuitpython-ina228"
                logger.warning(
                    "Electro: power monitor init failed (%s); V/I/P forced to 0.%s",
                    exc,
                    hint,
                )
                _reader = False
        if _reader is False:
            return _synthetic_read()
        volt, curr, pwr = electro_driver.read_voltage_current_power(_reader)
        return float(volt), float(curr), float(pwr)
    except Exception as exc:
        now = time.time()
        if _reader is not False and now - _last_electro_read_warn_ts >= 3.0:
            logger.warning("Electro: INA228 read failed (%s); V/I/P forced to 0 this cycle", exc)
            _last_electro_read_warn_ts = now
        return _synthetic_read()


def read_electro_data() -> None:
    global VOLT, CURR, PWR, ELECTRO_HEALTH, _last_update_ts
    while ELECTROAPP_RUNSTATUS:
        sample = _read_sensor()
        if sample is None:
            ELECTRO_HEALTH = 0
            time.sleep(1.0)
            continue
        try:
            volt, curr, pwr = sample
            sensorlog.log_electro_raw(float(volt), float(curr), float(pwr))
            volt = _median5_update(volt, _volt_window)
            curr = _median5_update(curr, _curr_window)
            pwr = _median5_update(pwr, _pwr_window)
            with _electro_lock:
                VOLT = float(volt)
                CURR = float(curr)
                PWR = float(pwr)
                _last_update_ts = time.time()
                ELECTRO_HEALTH = 1
        except Exception:
            ELECTRO_HEALTH = 0
        time.sleep(1.0)


def send_electro_data(main_queue) -> None:
    global ELECTRO_HEALTH
    while ELECTROAPP_RUNSTATUS:
        if time.time() - _last_update_ts > ELECTRO_STALE_TIMEOUT_SEC:
            ELECTRO_HEALTH = 0
        with _electro_lock:
            volt, curr, pwr = VOLT, CURR, PWR
        msgstructure.send_msg(
            main_queue,
            appargs.ElectroAppArg.AppID,
            appargs.CommAppArg.AppID,
            appargs.ElectroAppArg.MID_comm_volt,
            f"{volt},{curr},{pwr}",
        )
        time.sleep(1.0)


def electroapp_terminate() -> None:
    global _reader
    try:
        from Sensor_Electro import electro as electro_driver  # type: ignore

        if _reader is not None and _reader is not False:
            electro_driver.terminate_INA228(_reader)
    except Exception:
        pass
    _reader = None


def electroapp_main(main_queue, main_pipe) -> None:
    t1 = threading.Thread(target=read_electro_data, daemon=True)
    t2 = threading.Thread(target=send_electro_data, args=(main_queue,), daemon=True)
    t1.start()
    t2.start()
    try:
        while ELECTROAPP_RUNSTATUS:
            try:
                has_msg = main_pipe.poll(0.1)
            except (KeyboardInterrupt, EOFError, OSError):
                break
            if has_msg:
                command_handler(main_pipe.recv())
    except KeyboardInterrupt:
        pass
    finally:
        electroapp_terminate()
