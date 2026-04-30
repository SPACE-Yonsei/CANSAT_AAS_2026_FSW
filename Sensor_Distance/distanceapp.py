"""Distance app with range filtering, median smoothing, and stale/health state."""

from __future__ import annotations

import logging
import threading
import time
from collections import deque

from lib import appargs, msgstructure


logger = logging.getLogger(__name__)


DISTANCEAPP_RUNSTATUS = True
DISTANCE_MM = 5000.0
DISTANCE_HEALTH = 1
DISTANCE_STALE_TIMEOUT_SEC = 1.0
DISTANCE_MIN_MM = 200
DISTANCE_MAX_MM = 8000

_last_update_ts = 0.0
_distance_window = deque(maxlen=5)
_distance_lock = threading.Lock()
_dist_hw = None


def command_handler(recv_msg: str) -> None:
    global DISTANCEAPP_RUNSTATUS
    unpacked = msgstructure.unpack_msg(recv_msg)
    if unpacked is False:
        return
    if unpacked.msg_id == appargs.MainAppArg.MID_TerminateProcess:
        DISTANCEAPP_RUNSTATUS = False


def _median_mm(values) -> int:
    if not values:
        return 0
    arr = sorted(int(v) for v in values)
    mid = len(arr) // 2
    if len(arr) % 2 == 1:
        return arr[mid]
    return int(round((arr[mid - 1] + arr[mid]) / 2.0))


def _is_valid_distance(mm: float) -> bool:
    return DISTANCE_MIN_MM <= float(mm) <= DISTANCE_MAX_MM


def _synthetic_read_distance() -> float:
    # Monotonic approach for deterministic local testing.
    return max(300.0, DISTANCE_MM - 7.0)


def read_distance_data() -> None:
    global DISTANCE_MM, DISTANCE_HEALTH, _last_update_ts, _dist_hw
    while DISTANCEAPP_RUNSTATUS:
        try:
            raw: float
            try:
                from Sensor_Distance import distance as dist_driver  # type: ignore

                if _dist_hw is None:
                    try:
                        _dist_hw = dist_driver.init_tfluna()
                    except Exception as exc:
                        logger.warning(
                            "Distance: TF-Luna I2C init failed (%s); using synthetic rangefinder",
                            exc,
                        )
                        _dist_hw = False
                if _dist_hw is not False:
                    try:
                        raw = float(dist_driver.read_range_mm(_dist_hw))
                    except Exception:
                        raw = _synthetic_read_distance()
                else:
                    raw = _synthetic_read_distance()
            except Exception:
                raw = _synthetic_read_distance()

            if _is_valid_distance(raw):
                _distance_window.append(float(raw))
                filtered = _median_mm(_distance_window)
                with _distance_lock:
                    DISTANCE_MM = float(filtered)
                    DISTANCE_HEALTH = 1
                    _last_update_ts = time.time()
            else:
                DISTANCE_HEALTH = 0
        except Exception:
            DISTANCE_HEALTH = 0
        time.sleep(0.1)


def send_distance_data(main_queue) -> None:
    global DISTANCE_HEALTH
    while DISTANCEAPP_RUNSTATUS:
        if time.time() - _last_update_ts > DISTANCE_STALE_TIMEOUT_SEC:
            DISTANCE_HEALTH = 0
        with _distance_lock:
            distance = DISTANCE_MM

        msgstructure.send_msg(
            main_queue,
            appargs.DistanceAppArg.AppID,
            appargs.FlightlogicAppArg.AppID,
            appargs.DistanceAppArg.MID_flight_dis,
            f"{distance}",
        )
        msgstructure.send_msg(
            main_queue,
            appargs.DistanceAppArg.AppID,
            appargs.CommAppArg.AppID,
            appargs.DistanceAppArg.MID_comm_dis,
            f"{distance}",
        )
        time.sleep(0.2)


def distanceapp_main(main_queue, main_pipe) -> None:
    t1 = threading.Thread(target=read_distance_data, daemon=True)
    t2 = threading.Thread(target=send_distance_data, args=(main_queue,), daemon=True)
    t1.start()
    t2.start()
    try:
        while DISTANCEAPP_RUNSTATUS:
            try:
                has_msg = main_pipe.poll(0.1)
            except (KeyboardInterrupt, EOFError, OSError):
                break
            if has_msg:
                command_handler(main_pipe.recv())
    except KeyboardInterrupt:
        pass
