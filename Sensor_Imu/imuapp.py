"""IMU app with filtering, stale/health handling, and retry logic."""

from __future__ import annotations

import logging
import os
import threading
import time
from typing import Optional, Tuple

from lib import appargs, config, msgstructure, prevstate, sensorlog, timebase


logger = logging.getLogger(__name__)


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
IMU_MAX_CONSECUTIVE_ERRORS = 5
IMU_STALE_TIMEOUT_SEC = 1.0


def _env_float(name: str, default: float, lo: float, hi: float) -> float:
    try:
        value = float(os.environ.get(name, str(default)))
    except (TypeError, ValueError):
        value = default
    return max(lo, min(value, hi))


# Stale-sample watchdog: if no fresh sample for ``IMU_STALE_REINIT_SEC`` seconds,
# force a reinit (which pulses the BNO085 RST pin when ``IMU_BNO085_RST_ENABLE=1``).
# Cooldown prevents reinit storms when reinit itself is also failing.
IMU_STALE_REINIT_SEC = _env_float("IMU_STALE_REINIT_SEC", 2.0, 0.5, 30.0)
IMU_REINIT_COOLDOWN_SEC = _env_float("IMU_REINIT_COOLDOWN_SEC", 5.0, 1.0, 60.0)

_last_sample_ts = 0.0
_last_reinit_ts = 0.0
_imu_lock = threading.Lock()
_imu_instance = None
_i2c_instance = None
_acc_norm_window: list[float] = []
_gyro_norm_window: list[float] = []
_startup_yaw_zeroed = False
_startup_yaw_sample_count = 0
# BNO08x heading estimate is unstable for ~1s after init; skip first N samples
_STARTUP_YAW_WARMUP_SAMPLES: int = int(os.environ.get("IMU_YAW_WARMUP_SAMPLES", "10"))

# freefall / tumble 판정 상수 — 환경변수로 오버라이드 가능
FREEFALL_ACC_NORM_THRESHOLD_MPS2 = float(os.environ.get("FREEFALL_ACC_NORM_MPS2", "3.0"))
TUMBLE_GYRO_NORM_THRESHOLD_DEGS  = float(os.environ.get("TUMBLE_GYRO_NORM_DEGS",  "200.0"))
ACC_NORM_WINDOW_SIZE  = 5
GYRO_NORM_WINDOW_SIZE = 5

# freefall / tumble 상태 (send 스레드가 읽음)
FREEFALL = 0  # 1=자유낙하 중, 0=정상(중력 있음)
TUMBLE   = 0  # 1=텀블링 중,  0=안정(정상 선회)

# sample_ts: read 스레드에서 캡처, send 스레드가 읽음
_last_sample_mono_ts: float = 0.0


def _imu_read_period_sec() -> float:
    try:
        rate_hz = float(config.IMU_RATE_HZ)
    except (TypeError, ValueError):
        rate_hz = 10.0
    return 1.0 / max(0.1, rate_hz)


def _wrap_deg(deg: float) -> float:
    while deg >= 360.0:
        deg -= 360.0
    while deg < 0.0:
        deg += 360.0
    return deg


def _apply_yaw_offset(yaw: float) -> float:
    return _wrap_deg(float(yaw) + prevstate.YAW_OFFSET)


def _calibrate_startup_yaw(raw_yaw: float) -> None:
    """Treat the first stable yaw after app start as 0 deg reference.

    Skips the first _STARTUP_YAW_WARMUP_SAMPLES samples because the BNO08x
    heading estimate drifts ~1 deg during the first second of operation.
    """
    global _startup_yaw_zeroed, _startup_yaw_sample_count
    if _startup_yaw_zeroed:
        return
    _startup_yaw_sample_count += 1
    if _startup_yaw_sample_count < _STARTUP_YAW_WARMUP_SAMPLES:
        return
    offset = _wrap_deg(-float(raw_yaw))
    prevstate.PREV_YAW_OFFSET = offset
    prevstate.YAW_OFFSET = offset
    _startup_yaw_zeroed = True
    logger.info(
        "IMU: startup yaw zeroed (raw=%.2f deg, offset=%.2f deg, warmup=%d samples)",
        raw_yaw, offset, _startup_yaw_sample_count,
    )


def command_handler(recv_msg: str) -> None:
    global IMUAPP_RUNSTATUS
    unpacked = msgstructure.unpack_msg(recv_msg)
    if unpacked is False:
        return
    if unpacked.msg_id == appargs.MainAppArg.MID_TerminateProcess:
        IMUAPP_RUNSTATUS = False


def _synthetic_sample() -> Tuple[float, float, float, float, float, float, float, float, float, float, float, float]:
    """Exception path in driver read: all zeros (no fake attitude)."""
    return (0.0,) * 12


def _read_sensor_sample():
    """Read from real IMU if available; on driver exception return zeros."""
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
    global _i2c_instance, _imu_instance, _startup_yaw_zeroed, _startup_yaw_sample_count
    prevstate.refresh_runtime_overrides()
    _startup_yaw_zeroed = False
    _startup_yaw_sample_count = 0
    try:
        from Sensor_Imu import imu as imu_driver  # type: ignore

        _i2c_instance, _imu_instance = imu_driver.init_imu()
    except KeyboardInterrupt:
        raise
    except Exception as exc:
        logger.debug("IMU: hardware init failed (%s); samples will stay at zero until reinit succeeds", exc)
        _i2c_instance, _imu_instance = None, None


def _try_reinit() -> None:
    global _i2c_instance, _imu_instance, _last_reinit_ts, _last_sample_ts
    try:
        from Sensor_Imu import imu as imu_driver  # type: ignore

        if _i2c_instance is not None or _imu_instance is not None:
            _i2c_instance, _imu_instance = imu_driver.reinit_imu(_i2c_instance, _imu_instance)
        else:
            _i2c_instance, _imu_instance = imu_driver.init_imu()
        # Reset sample timestamp so the stale watchdog doesn't fire immediately
        # after a long reinit (reinit duration can exceed IMU_REINIT_COOLDOWN_SEC).
        _last_sample_ts = timebase.wall_now()
    except KeyboardInterrupt:
        raise
    except Exception as exc:
        logger.debug("IMU: reinit failed (%s)", exc)
        _i2c_instance, _imu_instance = None, None
    finally:
        # Measure cooldown from completion, not start, so a slow reinit doesn't
        # cause the watchdog to re-fire the instant it returns.
        _last_reinit_ts = timebase.wall_now()


def _stale_watchdog_check() -> bool:
    """Force reinit when no fresh sample arrived for ``IMU_STALE_REINIT_SEC``.

    Returns ``True`` if a reinit was triggered (so caller can skip the rest of the
    loop iteration). Debounced by ``IMU_REINIT_COOLDOWN_SEC`` to avoid reinit storms.
    """
    now = timebase.wall_now()
    if _last_sample_ts <= 0.0:
        return False
    if (now - _last_sample_ts) <= IMU_STALE_REINIT_SEC:
        return False
    if (now - _last_reinit_ts) <= IMU_REINIT_COOLDOWN_SEC:
        return False
    logger.debug(
        "IMU stale watchdog: %.2fs without fresh sample; forcing reinit "
        "(HW RST pulse if IMU_BNO085_RST_ENABLE=1)",
        now - _last_sample_ts,
    )
    _try_reinit()
    return True


def read_imu_data() -> None:
    global ROLL, PITCH, YAW, ACCX, ACCY, ACCZ, MAGX, MAGY, MAGZ, GYRX, GYRY, GYRZ
    global HEALTH, IMU_ERROR_COUNT, _last_sample_ts, _last_sample_mono_ts
    global _acc_norm_window, _gyro_norm_window
    global FREEFALL, TUMBLE
    import math as _math
    period = _imu_read_period_sec()
    while IMUAPP_RUNSTATUS:
        if _stale_watchdog_check():
            IMU_ERROR_COUNT = 0
            HEALTH = 0
            time.sleep(period)
            continue

        sample = _read_sensor_sample()
        if sample is False:
            IMU_ERROR_COUNT += 1
            if IMU_ERROR_COUNT >= IMU_MAX_CONSECUTIVE_ERRORS:
                _try_reinit()
                IMU_ERROR_COUNT = 0
            HEALTH = 0
            time.sleep(period)
            continue

        sensorlog.log_imu_raw(sample)

        IMU_ERROR_COUNT = 0
        roll, pitch, yaw, accx, accy, accz, magx, magy, magz, gyrx, gyry, gyrz = sample
        _calibrate_startup_yaw(float(yaw))

        # freefall / tumble 판정 (이동평균으로 단발 스파이크 방지)
        acc_norm_raw  = _math.sqrt(float(accx)**2 + float(accy)**2 + float(accz)**2)
        gyro_norm_raw = _math.sqrt(float(gyrx)**2 + float(gyry)**2 + float(gyrz)**2)
        _acc_norm_window.append(acc_norm_raw)
        if len(_acc_norm_window) > ACC_NORM_WINDOW_SIZE:
            _acc_norm_window.pop(0)
        _gyro_norm_window.append(gyro_norm_raw)
        if len(_gyro_norm_window) > GYRO_NORM_WINDOW_SIZE:
            _gyro_norm_window.pop(0)
        acc_norm_avg  = sum(_acc_norm_window)  / len(_acc_norm_window)
        gyro_norm_avg = sum(_gyro_norm_window) / len(_gyro_norm_window)
        freefall_flag = 1 if acc_norm_avg  <  FREEFALL_ACC_NORM_THRESHOLD_MPS2 else 0
        tumble_flag   = 1 if gyro_norm_avg >= TUMBLE_GYRO_NORM_THRESHOLD_DEGS  else 0

        with _imu_lock:
            ROLL = float(roll)
            PITCH = float(pitch)
            YAW = _apply_yaw_offset(float(yaw))
            ACCX, ACCY, ACCZ = float(accx), float(accy), float(accz)
            MAGX, MAGY, MAGZ = float(magx), float(magy), float(magz)
            GYRX, GYRY, GYRZ = float(gyrx), float(gyry), float(gyrz)
            FREEFALL = freefall_flag
            TUMBLE   = tumble_flag
            _last_sample_ts       = timebase.wall_now()
            _last_sample_mono_ts  = timebase.now()

        HEALTH = 1
        time.sleep(period)


def send_imu_data(main_queue) -> None:
    global HEALTH
    tick = 0
    while IMUAPP_RUNSTATUS:
        now = timebase.wall_now()
        if now - _last_sample_ts > IMU_STALE_TIMEOUT_SEC:
            HEALTH = 0

        with _imu_lock:
            fr   = ROLL
            fp   = PITCH
            fy   = YAW
            accx, accy, accz = ACCX, ACCY, ACCZ
            magx, magy, magz = MAGX, MAGY, MAGZ
            gyrx, gyry, gyrz = GYRX, GYRY, GYRZ
            freefall = FREEFALL
            tumble   = TUMBLE
            sample_mono_ts = _last_sample_mono_ts

        msgstructure.send_msg(
            main_queue,
            appargs.ImuAppArg.AppID,
            appargs.MotorAppArg.AppID,
            appargs.ImuAppArg.MID_motor_imu,
            f"{fr},{fp},{fy},{accx},{accy},{accz},{magx},{magy},{magz},{gyrx},{gyry},{gyrz},{sample_mono_ts:.4f},{freefall},{tumble},{int(HEALTH)}",
        )
        tick += 1
        if tick >= 20:
            tick = 0
            msgstructure.send_msg(
                main_queue,
                appargs.ImuAppArg.AppID,
                appargs.CommAppArg.AppID,
                appargs.ImuAppArg.MID_comm_euler,
                f"{fr},{fp},{fy},{accx},{accy},{accz},{magx},{magy},{magz},{gyrx},{gyry},{gyrz}",
            )
        time.sleep(0.05)


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
        t1.join(timeout=2.0)
        t2.join(timeout=2.0)
        imuapp_terminate()
