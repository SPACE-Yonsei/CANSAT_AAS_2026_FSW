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
HEADING_MODE = "unknown"

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
_yaw_ema = None
_gyrz_ema = None
_acc_norm_ema: Optional[float] = None
_gyro_norm_ema: Optional[float] = None
_startup_yaw_zeroed = False
EMA_ALPHA = 0.9
IMU_GAME_VECTOR_HEALTH_OK = os.environ.get("IMU_GAME_VECTOR_HEALTH_OK", "0").strip().lower() in (
    "1",
    "true",
    "yes",
    "on",
)

# freefall / tumble 판정 상수 — 환경변수로 오버라이드 가능
FREEFALL_ACC_NORM_THRESHOLD_MPS2 = float(os.environ.get("FREEFALL_ACC_NORM_MPS2", "3.0"))
TUMBLE_GYRO_NORM_THRESHOLD_DEGS  = float(os.environ.get("TUMBLE_GYRO_NORM_DEGS",  "200.0"))
ACC_NORM_EMA_ALPHA  = 0.3  # 빠른 반응 (freefall 감지)
GYRO_NORM_EMA_ALPHA = 0.3

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


def _wrap_delta_deg(deg: float) -> float:
    return (float(deg) + 180.0) % 360.0 - 180.0


def _apply_yaw_offset(yaw: float) -> float:
    return _wrap_deg(float(yaw) + prevstate.YAW_OFFSET)


def _ema(prev: Optional[float], cur: float, alpha: float = EMA_ALPHA) -> float:
    if prev is None:
        return cur
    return alpha * cur + (1.0 - alpha) * prev


def _ema_angle(prev: Optional[float], cur: float, alpha: float = EMA_ALPHA) -> float:
    cur = _wrap_deg(cur)
    if prev is None:
        return cur
    return _wrap_deg(prev + alpha * _wrap_delta_deg(cur - prev))


def _heading_mode_from_driver() -> str:
    try:
        from Sensor_Imu import imu as imu_driver  # type: ignore

        return imu_driver.heading_mode()
    except Exception:
        return "unknown"


def _heading_health(mode: str) -> int:
    if mode == "game_rotation_vector" and not IMU_GAME_VECTOR_HEALTH_OK:
        return 0
    return 1


def _calibrate_startup_yaw(raw_yaw: float) -> None:
    """Treat the first valid yaw after app start as 0 deg reference."""
    global _startup_yaw_zeroed
    if _startup_yaw_zeroed:
        return
    offset = _wrap_deg(-float(raw_yaw))
    prevstate.PREV_YAW_OFFSET = offset
    prevstate.YAW_OFFSET = offset
    _startup_yaw_zeroed = True
    logger.info("IMU: startup yaw zeroed (raw=%.2f deg, offset=%.2f deg)", raw_yaw, offset)


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
    global _i2c_instance, _imu_instance, _startup_yaw_zeroed
    prevstate.refresh_runtime_overrides()
    _startup_yaw_zeroed = False
    try:
        from Sensor_Imu import imu as imu_driver  # type: ignore

        _i2c_instance, _imu_instance = imu_driver.init_imu()
    except KeyboardInterrupt:
        raise
    except Exception as exc:
        logger.warning("IMU: hardware init failed (%s); samples will stay at zero until reinit succeeds", exc)
        _i2c_instance, _imu_instance = None, None


def _try_reinit() -> None:
    global _i2c_instance, _imu_instance, _last_reinit_ts, HEADING_MODE
    _last_reinit_ts = timebase.wall_now()
    try:
        from Sensor_Imu import imu as imu_driver  # type: ignore

        if _i2c_instance is not None or _imu_instance is not None:
            _i2c_instance, _imu_instance = imu_driver.reinit_imu(_i2c_instance, _imu_instance)
        else:
            _i2c_instance, _imu_instance = imu_driver.init_imu()
        HEADING_MODE = imu_driver.heading_mode()
    except KeyboardInterrupt:
        raise
    except Exception as exc:
        logger.warning("IMU: reinit failed (%s)", exc)
        _i2c_instance, _imu_instance = None, None


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
    logger.warning(
        "IMU stale watchdog: %.2fs without fresh sample; forcing reinit "
        "(HW RST pulse if IMU_BNO085_RST_ENABLE=1)",
        now - _last_sample_ts,
    )
    _try_reinit()
    return True


def read_imu_data() -> None:
    global ROLL, PITCH, YAW, ACCX, ACCY, ACCZ, MAGX, MAGY, MAGZ, GYRX, GYRY, GYRZ
    global HEALTH, IMU_ERROR_COUNT, _last_sample_ts, _last_sample_mono_ts
    global _yaw_ema, _gyrz_ema, _acc_norm_ema, _gyro_norm_ema
    global FREEFALL, TUMBLE, HEADING_MODE
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
        mode = _heading_mode_from_driver()
        _calibrate_startup_yaw(float(yaw))
        _yaw_ema  = _ema_angle(_yaw_ema, _apply_yaw_offset(float(yaw)))
        _gyrz_ema = _ema(_gyrz_ema, float(gyrz))

        # freefall / tumble 판정 (EMA smoothing으로 단발 스파이크 방지)
        acc_norm_raw  = _math.sqrt(float(accx)**2 + float(accy)**2 + float(accz)**2)
        gyro_norm_raw = _math.sqrt(float(gyrx)**2 + float(gyry)**2 + float(gyrz)**2)
        _acc_norm_ema  = _ema(_acc_norm_ema,  acc_norm_raw,  ACC_NORM_EMA_ALPHA)
        _gyro_norm_ema = _ema(_gyro_norm_ema, gyro_norm_raw, GYRO_NORM_EMA_ALPHA)
        freefall_flag = 1 if _acc_norm_ema  <  FREEFALL_ACC_NORM_THRESHOLD_MPS2 else 0
        tumble_flag   = 1 if _gyro_norm_ema >= TUMBLE_GYRO_NORM_THRESHOLD_DEGS  else 0

        with _imu_lock:
            ROLL = float(roll)
            PITCH = float(pitch)
            YAW = float(_yaw_ema)
            ACCX, ACCY, ACCZ = float(accx), float(accy), float(accz)
            MAGX, MAGY, MAGZ = float(magx), float(magy), float(magz)
            GYRX, GYRY, GYRZ = float(gyrx), float(gyry), float(_gyrz_ema)
            FREEFALL = freefall_flag
            TUMBLE   = tumble_flag
            HEADING_MODE = mode
            _last_sample_ts       = timebase.wall_now()
            _last_sample_mono_ts  = timebase.now()

        HEALTH = _heading_health(mode)
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

        # health 포함해서 항상 전송 — motorapp/guidance가 health 값으로 판단
        # freefall(1=자유낙하), tumble(1=텀블링)은 값으로 전달
        msgstructure.send_msg(
            main_queue,
            appargs.ImuAppArg.AppID,
            appargs.MotorAppArg.AppID,
            appargs.ImuAppArg.MID_motor_imu,
            f"{fr},{fp},{fy},{accx},{accy},{accz},{magx},{magy},{magz},{gyrx},{gyry},{gyrz},{sample_mono_ts:.4f},{freefall},{tumble},{int(HEALTH)}",
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
        t1.join(timeout=2.0)
        t2.join(timeout=2.0)
        imuapp_terminate()
