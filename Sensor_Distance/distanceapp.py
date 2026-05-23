"""Distance app with range filtering, median smoothing, and stale/health state."""

from __future__ import annotations

import logging
import os
import threading
import time
from collections import deque

from lib import appargs, config, msgstructure, sensorlog


logger = logging.getLogger(__name__)


DISTANCEAPP_RUNSTATUS = True
DISTANCE_MM = 0.0
DISTANCE_HEALTH = 1
DISTANCE_STALE_TIMEOUT_SEC = 1.0


def _valid_mm_bounds() -> tuple[float, float]:
    """Operational band for rangefinder samples (mm).

    Defaults match TF-Luna useful range (~0.2–8 m). For bench tests closer than
    20 cm, set ``DISTANCE_MIN_MM`` lower (e.g. ``50``) or readings clamp to 0.
    """
    try:
        lo = float(os.environ.get("DISTANCE_MIN_MM", "200"))
    except ValueError:
        lo = 200.0
    try:
        hi = float(os.environ.get("DISTANCE_MAX_MM", "8000"))
    except ValueError:
        hi = 8000.0
    if lo > hi:
        lo, hi = hi, lo
    return lo, hi

_last_update_ts = 0.0
_distance_window = deque(maxlen=5)
_distance_lock = threading.Lock()
_dist_hw = None


def _distance_rate_hz() -> float:
    try:
        return max(0.1, float(config.DISTANCE_RATE_HZ))
    except (TypeError, ValueError):
        return 10.0


def _distance_period_sec() -> float:
    return 1.0 / _distance_rate_hz()


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
    lo, hi = _valid_mm_bounds()
    return lo <= float(mm) <= hi


def _synthetic_read_distance() -> float:
    """No rangefinder / read failure: zero mm (invalid vs operational band)."""
    return 0.0


def read_distance_data() -> None:
    global DISTANCE_MM, DISTANCE_HEALTH, _last_update_ts, _dist_hw
    period = _distance_period_sec()
    while DISTANCEAPP_RUNSTATUS:
        try:
            raw: float
            try:
                from Sensor_Distance import distance as dist_driver  # type: ignore

                if _dist_hw is None:
                    try:
                        _dist_hw = dist_driver.init_tfluna()
                        logger.info(
                            "Distance: TF-Luna init OK; entering read loop"
                        )
                    except Exception as exc:
                        logger.warning(
                            "Distance: TF-Luna I2C init failed (%s); distance forced to 0 mm",
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
            except Exception as exc:
                logger.warning("Distance: outer read loop error (%s)", exc)
                raw = _synthetic_read_distance()

            sensorlog.log_distance_raw(raw)

            if _is_valid_distance(raw):
                _distance_window.append(float(raw))
                filtered = _median_mm(_distance_window)
                with _distance_lock:
                    DISTANCE_MM = float(filtered)
                    DISTANCE_HEALTH = 1
                    _last_update_ts = time.time()
            else:
                with _distance_lock:
                    DISTANCE_MM = 0.0
                DISTANCE_HEALTH = 0
        except Exception as exc:
            logger.warning("Distance: read loop unexpected error (%s)", exc)
            DISTANCE_HEALTH = 0
        time.sleep(period)


def send_distance_data(main_queue) -> None:
    global DISTANCE_HEALTH
    period = _distance_period_sec()
    while DISTANCEAPP_RUNSTATUS:
        if time.time() - _last_update_ts > DISTANCE_STALE_TIMEOUT_SEC:
            DISTANCE_HEALTH = 0
        with _distance_lock:
            distance = DISTANCE_MM
            health = int(DISTANCE_HEALTH)

        msgstructure.send_msg(
            main_queue,
            appargs.DistanceAppArg.AppID,
            appargs.FlightlogicAppArg.AppID,
            appargs.DistanceAppArg.MID_flight_dis,
            f"{distance},{health}",
        )
        msgstructure.send_msg(
            main_queue,
            appargs.DistanceAppArg.AppID,
            appargs.CommAppArg.AppID,
            appargs.DistanceAppArg.MID_comm_dis,
            f"{distance}",
        )
        time.sleep(period)


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
    if _dist_hw:
        try:
            from Sensor_Distance import distance as dist_driver
            dist_driver.terminate_tfluna(_dist_hw)
        except Exception:
            pass
