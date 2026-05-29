"""IMU app with filtering, stale/health handling, and retry logic."""

from __future__ import annotations

import logging
import os
import threading
import time
from typing import Optional, Tuple

from lib import appargs, config, msgstructure, prevstate, sensorlog


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
_startup_yaw_zeroed = False
_imuapp_start_time: float = 0.0
# Seconds after imuapp_init() before the first yaw is used as the zero reference.
# Override with IMU_YAW_ZERO_DELAY_SEC env var.
_STARTUP_YAW_ZERO_DELAY_S: float = float(os.environ.get("IMU_YAW_ZERO_DELAY_SEC", "15.0"))


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
    """Zero yaw _STARTUP_YAW_ZERO_DELAY_S seconds after imuapp_init()."""
    global _startup_yaw_zeroed
    if _startup_yaw_zeroed:
        return
    elapsed = time.time() - _imuapp_start_time
    if elapsed < _STARTUP_YAW_ZERO_DELAY_S:
        return
    offset = _wrap_deg(-float(raw_yaw))
    prevstate.update_yaw_offset(offset)
    _startup_yaw_zeroed = True
    logger.info(
        "IMU: startup yaw zeroed (raw=%.2f deg, offset=%.2f deg, elapsed=%.1fs)",
        raw_yaw, offset, elapsed,
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


_read_sample_count = 0

def _read_sensor_sample():
    """Read from real IMU if available; on driver exception return zeros."""
    global _imu_instance, _read_sample_count
    try:
        from Sensor_Imu import imu as imu_driver  # type: ignore

        if _imu_instance is None:
            if _read_sample_count == 0:
                print("[IMU_DEBUG] _read_sensor_sample: _imu_instance가 None → 초기화 실패 상태", flush=True)
            _read_sample_count += 1
            return False
        sample = imu_driver.read_sensor_data(_imu_instance)
        if sample is False:
            return False
        _read_sample_count += 1
        if _read_sample_count <= 3 or _read_sample_count % 50 == 0:
            print(f"[IMU_DEBUG] sample #{_read_sample_count}: roll={sample[0]:.1f} pitch={sample[1]:.1f} yaw={sample[2]:.1f}", flush=True)
        return sample
    except Exception as exc:
        print(f"[IMU_DEBUG] _read_sensor_sample 예외: {type(exc).__name__}: {exc}", flush=True)
        return _synthetic_sample()


def imuapp_init() -> None:
    global _i2c_instance, _imu_instance, _startup_yaw_zeroed, _imuapp_start_time
    prevstate.refresh_runtime_overrides()
    _startup_yaw_zeroed = False
    _imuapp_start_time = time.time()
    print("[IMU_DEBUG] imuapp_init: 시작", flush=True)
    try:
        from Sensor_Imu import imu as imu_driver  # type: ignore

        print("[IMU_DEBUG] imuapp_init: init_imu() 호출 중...", flush=True)
        _i2c_instance, _imu_instance = imu_driver.init_imu()
        print(f"[IMU_DEBUG] imuapp_init: 성공 (i2c={_i2c_instance}, bno={_imu_instance})", flush=True)
    except KeyboardInterrupt:
        raise
    except Exception as exc:
        print(f"[IMU_DEBUG] imuapp_init: 실패! {type(exc).__name__}: {exc}", flush=True)
        logger.warning("IMU: hardware init failed (%s); samples will stay at zero until reinit succeeds", exc)
        _i2c_instance, _imu_instance = None, None


def _try_reinit() -> None:
    global _i2c_instance, _imu_instance, _last_reinit_ts, _last_sample_ts
    global _last_sample_mono_ts  # reinit 완료 후 monotonic ts도 갱신해야 motorapp 쪽 stale 판정을 막을 수 있음
    try:
        from Sensor_Imu import imu as imu_driver  # type: ignore

        if _i2c_instance is not None or _imu_instance is not None:
            _i2c_instance, _imu_instance = imu_driver.reinit_imu(_i2c_instance, _imu_instance)
        else:
            _i2c_instance, _imu_instance = imu_driver.init_imu()
        # Reset sample timestamp so the stale watchdog doesn't fire immediately
        # after a long reinit (reinit duration can exceed IMU_REINIT_COOLDOWN_SEC).
        _last_sample_ts = time.time()
        # monotonic ts도 현재 시각으로 갱신:
        # 갱신하지 않으면 send_imu_data가 reinit 전 구 timestamp를 계속 전송하여
        # motorapp decidefresh가 IMU_FRESH_MAX_AGE_S 초과 직후 stale 판정을 내린다.
        _last_sample_mono_ts = time.monotonic()
    except KeyboardInterrupt:
        raise
    except Exception as exc:
        logger.warning("IMU: reinit failed (%s)", exc)
        _i2c_instance, _imu_instance = None, None
    finally:
        # Measure cooldown from completion, not start, so a slow reinit doesn't
        # cause the watchdog to re-fire the instant it returns.
        _last_reinit_ts = time.time()


def _stale_watchdog_check() -> bool:
    """Force reinit when no fresh sample arrived for ``IMU_STALE_REINIT_SEC``.

    Returns ``True`` if a reinit was triggered (so caller can skip the rest of the
    loop iteration). Debounced by ``IMU_REINIT_COOLDOWN_SEC`` to avoid reinit storms.
    """
    now = time.time()
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

        with _imu_lock:
            ROLL = float(roll)
            PITCH = float(pitch)
            YAW = _apply_yaw_offset(float(yaw))
            ACCX, ACCY, ACCZ = float(accx), float(accy), float(accz)
            MAGX, MAGY, MAGZ = float(magx), float(magy), float(magz)
            GYRX, GYRY, GYRZ = float(gyrx), float(gyry), float(gyrz)
            _last_sample_ts       = time.time()
            _last_sample_mono_ts  = time.monotonic()

        HEALTH = 1
        time.sleep(period)


def send_imu_data(main_queue) -> None:
    global HEALTH
    tick = 0
    while IMUAPP_RUNSTATUS:
        now = time.time()
        if now - _last_sample_ts > IMU_STALE_TIMEOUT_SEC:
            HEALTH = 0

        with _imu_lock:
            fr   = ROLL
            fp   = PITCH
            fy   = YAW
            accx, accy, accz = ACCX, ACCY, ACCZ
            magx, magy, magz = MAGX, MAGY, MAGZ
            gyrx, gyry, gyrz = GYRX, GYRY, GYRZ
            health         = int(HEALTH)
            sample_mono_ts = _last_sample_mono_ts

        # Payload (11 fields): roll,pitch,yaw,ax,ay,az,gyrx,gyry,gyrz,health,sample_ts
        # health=0 → data fields are nan so motorapp skips populating guidance fields
        if health:
            payload = (
                f"{fr},{fp},{fy},"
                f"{accx},{accy},{accz},"
                f"{gyrx},{gyry},{gyrz},"
                f"{health},{sample_mono_ts:.4f}"
            )
        else:
            payload = f"nan,nan,nan,nan,nan,nan,nan,nan,nan,0,{sample_mono_ts:.4f}"

        msgstructure.send_msg(
            main_queue,
            appargs.ImuAppArg.AppID,
            appargs.MotorAppArg.AppID,
            appargs.ImuAppArg.MID_motor_imu,
            payload,
        )
        tick += 1
        if tick >= 20:
            tick = 0
            # commapp expects 12 fields: roll,pitch,yaw,ax,ay,az,mx,my,mz,gyrx,gyry,gyrz
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
    print("[IMU_DEBUG] imuapp_main: 프로세스 시작", flush=True)
    imuapp_init()
    print(f"[IMU_DEBUG] imuapp_main: init 완료, _imu_instance={_imu_instance}", flush=True)
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
