"""Electro app with sensor read fallback, stale/health handling, and robust send."""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional

from lib import appargs, msgstructure


logger = logging.getLogger(__name__)


ELECTROAPP_RUNSTATUS = True
VOLT = 7.4
CURR = 0.8
PWR = 5.92
ELECTRO_HEALTH = 0
ELECTRO_STALE_TIMEOUT_SEC = 2.5
_last_update_ts = 0.0
_reader = None  # dict (hardware) | False (synthetic-only) | None (not probed yet)
_electro_lock = threading.Lock()


def command_handler(recv_msg: str) -> None:
    global ELECTROAPP_RUNSTATUS
    unpacked = msgstructure.unpack_msg(recv_msg)
    if unpacked is False:
        return
    if unpacked.msg_id == appargs.MainAppArg.MID_TerminateProcess:
        ELECTROAPP_RUNSTATUS = False


def _synthetic_read() -> tuple[float, float, float]:
    # Deterministic low-frequency drift for non-hardware environments.
    phase = int(time.time()) % 4
    volt = 7.35 + (phase * 0.01)
    curr = 0.75 + (phase * 0.02)
    pwr = volt * curr
    return volt, curr, pwr


def _read_sensor() -> Optional[tuple[float, float, float]]:
    global _reader
    try:
        from Sensor_Electro import electro as electro_driver  # type: ignore

        if _reader is None:
            try:
                _reader = electro_driver.init_INA228()
            except Exception as exc:
                logger.warning(
                    "Electro: power monitor init failed (%s); using synthetic V/I/P",
                    exc,
                )
                _reader = False
        if _reader is False:
            return _synthetic_read()
        volt = electro_driver.read_voltage(_reader)
        curr = electro_driver.read_current(_reader)
        pwr = electro_driver.read_power(_reader)
        return float(volt), float(curr), float(pwr)
    except Exception:
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
