"""IMU app with filtering, stale/health handling, and retry logic."""

from __future__ import annotations

import threading
import time
from typing import Optional, Tuple

from lib import appargs, msgstructure


IMUAPP_RUNSTATUS = True

# Shared IMU states
ROLL = 0.0
PITCH = 0.0
YAW = 0.0
ACCX = 0.0
ACCY = 0.0
ACCZ = 0.0
MAGX = 0.0
MAGY = 0.0
MAGZ = 0.0
GYRX = 0.0
GYRY = 0.0
GYRZ = 0.0
HEALTH = 1

# Runtime control
IMU_ERROR_COUNT = 0
IMU_MAX_CONSECUTIVE_ERRORS = 50
IMU_STALE_TIMEOUT_SEC = 1.0
_last_sample_ts = 0.0
_imu_lock = threading.Lock()
_imu_instance = None
_i2c_instance = None
_yaw_ema = None
_gyrz_ema = None
EMA_ALPHA = 0.25


def _wrap_deg(deg: float) -> float:
    while deg >= 360.0:
        deg -= 360.0
    while deg < 0.0:
        deg += 360.0
    return deg


def _ema(prev: Optional[float], cur: float, alpha: float = EMA_ALPHA) -> float:
    if prev is None:
        return cur
    return alpha * cur + (1.0 - alpha) * prev


def command_handler(recv_msg: str) -> None:
    global IMUAPP_RUNSTATUS
    unpacked = msgstructure.unpack_msg(recv_msg)
    if unpacked is False:
        return
    if unpacked.msg_id == appargs.MainAppArg.MID_TerminateProcess:
        IMUAPP_RUNSTATUS = False


def _synthetic_sample() -> Tuple[float, float, float, float, float, float, float, float, float, float, float, float]:
    # Fallback when physical IMU is unavailable.
    now = time.time()
    yaw = _wrap_deg((now * 10.0) % 360.0)
    roll = 5.0
    pitch = -2.0
    accx, accy, accz = 0.0, 0.0, 9.81
    magx, magy, magz = 20.0, 1.0, -35.0
    gyrx, gyry, gyrz = 0.1, 0.2, 0.5
    return roll, pitch, yaw, accx, accy, accz, magx, magy, magz, gyrx, gyry, gyrz


def _read_sensor_sample():
    """Read from real imu module if available, else synthetic fallback."""
    global _imu_instance
    try:
        from Sensor_Imu import imu as imu_driver  # type: ignore

        if _imu_instance is None:
            return False
        sample = imu_driver.read_sensor_data(_imu_instance)
        if sample is False:
            return False
        return sample
    except Exception:
        return _synthetic_sample()


def imuapp_init() -> None:
    global _i2c_instance, _imu_instance
    try:
        from Sensor_Imu import imu as imu_driver  # type: ignore

        _i2c_instance, _imu_instance = imu_driver.init_imu()
    except Exception:
        _i2c_instance, _imu_instance = None, None


def _try_reinit() -> None:
    global _i2c_instance, _imu_instance
    try:
        from Sensor_Imu import imu as imu_driver  # type: ignore

        if _i2c_instance is not None or _imu_instance is not None:
            _i2c_instance, _imu_instance = imu_driver.reinit_imu(_i2c_instance, _imu_instance)
        else:
            _i2c_instance, _imu_instance = imu_driver.init_imu()
    except Exception:
        _i2c_instance, _imu_instance = None, None


def read_imu_data() -> None:
    global ROLL, PITCH, YAW, ACCX, ACCY, ACCZ, MAGX, MAGY, MAGZ, GYRX, GYRY, GYRZ
    global HEALTH, IMU_ERROR_COUNT, _last_sample_ts, _yaw_ema, _gyrz_ema
    while IMUAPP_RUNSTATUS:
        sample = _read_sensor_sample()
        if sample is False:
            IMU_ERROR_COUNT += 1
            if IMU_ERROR_COUNT >= IMU_MAX_CONSECUTIVE_ERRORS:
                _try_reinit()
                IMU_ERROR_COUNT = 0
            HEALTH = 0
            time.sleep(0.01)
            continue

        IMU_ERROR_COUNT = 0
        roll, pitch, yaw, accx, accy, accz, magx, magy, magz, gyrx, gyry, gyrz = sample
        _yaw_ema = _ema(_yaw_ema, _wrap_deg(float(yaw)))
        _gyrz_ema = _ema(_gyrz_ema, float(gyrz))

        with _imu_lock:
            ROLL = float(roll)
            PITCH = float(pitch)
            YAW = float(_yaw_ema)
            ACCX, ACCY, ACCZ = float(accx), float(accy), float(accz)
            MAGX, MAGY, MAGZ = float(magx), float(magy), float(magz)
            GYRX, GYRY, GYRZ = float(gyrx), float(gyry), float(_gyrz_ema)
            _last_sample_ts = time.time()

        HEALTH = 1
        time.sleep(0.01)  # 100 Hz


def send_imu_data(main_queue) -> None:
    global HEALTH
    tick = 0
    while IMUAPP_RUNSTATUS:
        now = time.time()
        if now - _last_sample_ts > IMU_STALE_TIMEOUT_SEC:
            HEALTH = 0

        with _imu_lock:
            yaw = YAW
            gyrz = GYRZ
            fr = ROLL
            fp = PITCH
            fy = YAW
            accx, accy, accz = ACCX, ACCY, ACCZ
            magx, magy, magz = MAGX, MAGY, MAGZ
            gyrx, gyry = GYRX, GYRY

        msgstructure.send_msg(
            main_queue,
            appargs.ImuAppArg.AppID,
            appargs.MotorAppArg.AppID,
            appargs.ImuAppArg.MID_motor_imu,
            f"{yaw},{gyrz},{HEALTH}",
        )
        tick += 1
        if tick >= 10:
            tick = 0
            msgstructure.send_msg(
                main_queue,
                appargs.ImuAppArg.AppID,
                appargs.CommAppArg.AppID,
                appargs.ImuAppArg.MID_comm_euler,
                f"{fr},{fp},{fy},{accx},{accy},{accz},{magx},{magy},{magz},{gyrx},{gyry},{gyrz}",
            )
        time.sleep(0.1)


def imuapp_terminate() -> None:
    try:
        from Sensor_Imu import imu as imu_driver  # type: ignore

        if _i2c_instance is not None:
            imu_driver.imu_terminate(_i2c_instance)
    except Exception:
        pass


def imuapp_main(main_queue, main_pipe) -> None:
    imuapp_init()
    t1 = threading.Thread(target=read_imu_data, daemon=True)
    t2 = threading.Thread(target=send_imu_data, args=(main_queue,), daemon=True)
    t1.start()
    t2.start()
    try:
        while IMUAPP_RUNSTATUS:
            try:
                has_msg = main_pipe.poll(0.1)
            except (KeyboardInterrupt, EOFError, OSError):
                break
            if has_msg:
                command_handler(main_pipe.recv())
    except KeyboardInterrupt:
        pass
    finally:
        imuapp_terminate()
