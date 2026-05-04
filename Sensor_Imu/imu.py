"""BNO08x I2C IMU driver for `imuapp` (optional hardware path)."""

from __future__ import annotations

import logging
import math
import os
import sys
import time
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from typing import Any, Optional

_REPO = Path(__file__).resolve().parents[1]
if str(_REPO) not in sys.path:
    sys.path.insert(0, str(_REPO))

from lib import config, i2c_bus

logger = logging.getLogger(__name__)

_BNO_FEATURE_NAMES = {
    0x01: "accelerometer",
    0x02: "gyroscope",
    0x03: "magnetometer",
    0x05: "rotation_vector",
    0x08: "game_rotation_vector",
}


<<<<<<< HEAD
def _init_progress(msg: str) -> None:
    if os.environ.get("IMU_INIT_PROGRESS", "0").strip() == "1":
        print(msg, flush=True)
=======
def _env_int(name: str, default: int, lo: int, hi: int) -> int:
    try:
        value = int(os.environ.get(name, str(default)), 0)
    except ValueError:
        value = default
    return max(lo, min(value, hi))


def _env_float(name: str, default: float, lo: float, hi: float) -> float:
    try:
        value = float(os.environ.get(name, str(default)))
    except ValueError:
        value = default
    return max(lo, min(value, hi))


def _imu_read_rate_hz() -> float:
    return _env_float("IMU_READ_RATE_HZ", float(config.BAROMETER_RATE_HZ), 0.1, 200.0)


REPORT_INTERVAL_US = _env_int("IMU_REPORT_INTERVAL_US", int(1_000_000.0 / _imu_read_rate_hz()), 10000, 1000000)
READ_ATTEMPTS = _env_int("IMU_READ_ATTEMPTS", 4, 1, 12)
BNO085_RST_USE = os.environ.get("IMU_BNO085_RST_ENABLE", "0").strip().lower() not in ("0", "false", "no", "")
BNO085_RST_PIN = os.environ.get("IMU_BNO085_RST_PIN", "D22")
IMU_MOUNTED_ON_BOTTOM = os.environ.get("IMU_MOUNTED_ON_BOTTOM", "1").strip().lower() not in ("0", "false", "no", "")
IMU_FORWARD_AXIS = os.environ.get("IMU_FORWARD_AXIS", "Y").strip().upper()
MAG_FILTER_ALPHA = _env_float("IMU_MAG_FILTER_ALPHA", 0.5, 0.0, 1.0)
MAG_FIELD_MIN = _env_float("IMU_MAG_FIELD_MIN", 5.0, 0.0, 1000.0)
MAG_FIELD_MAX = _env_float("IMU_MAG_FIELD_MAX", 150.0, 1.0, 2000.0)
MAG_NORM_SPIKE_RATIO = _env_float("IMU_MAG_NORM_SPIKE_RATIO", 3.0, 1.1, 100.0)
YAW_CORRECTION_GAIN = _env_float("IMU_YAW_CORRECTION_GAIN", 0.02, 0.0, 1.0)
HAMPEL_WINDOW_SIZE = _env_int("IMU_HAMPEL_WINDOW_SIZE", 3, 3, 31)
HAMPEL_THRESHOLD = _env_float("IMU_HAMPEL_THRESHOLD", 3.0, 0.1, 20.0)
HAMPEL_MIN_MAD = _env_float("IMU_HAMPEL_MIN_MAD", 2.0, 0.0, 180.0)

_ANGLE_WINDOWS: dict[str, list[float]] = {"roll": [], "pitch": [], "yaw": []}
_LAST_VALID = {
    "acc": (0.0, 0.0, 0.0),
    "mag": (0.0, 0.0, 0.0),
    "gyr": (0.0, 0.0, 0.0),
}
_MAG_FILTER_STATE = {"x": 0.0, "y": 0.0, "z": 0.0, "init": False, "norm": None}
>>>>>>> ad06d2286af0c258b80c723ffe82f23fbb1e65a2


def _enable_feature_retry(bno: Any, feature_id: int, attempts: Optional[int] = None) -> None:
    """BNO08x often needs a short settle + retries right after power-up (Blinka / Pi)."""
    if attempts is None:
        try:
            attempts = int(os.environ.get("IMU_ENABLE_FEATURE_ATTEMPTS", "8"), 0)
        except ValueError:
            attempts = 8
        attempts = max(3, min(attempts, 20))
    name = _BNO_FEATURE_NAMES.get(feature_id, f"id={feature_id:#04x}")
    _init_progress(f"IMU: enable {name} …")
    last: Optional[Exception] = None
    for i in range(attempts):
        try:
            if os.environ.get("BNO08X_DEBUG", "").strip() == "1":
                bno.enable_feature(feature_id, REPORT_INTERVAL_US)
            else:
                with open(os.devnull, "w", encoding="utf-8") as devnull:
                    with redirect_stdout(devnull), redirect_stderr(devnull):
                        bno.enable_feature(feature_id, REPORT_INTERVAL_US)
            return
        except Exception as exc:
            last = exc
            time.sleep(0.06 * (i + 1))
    raise RuntimeError(f"BNO08x: enable {name} failed after {attempts} tries: {last}") from last


def _pulse_bno085_reset() -> None:
    """Optionally assert BNO085 nRESET before opening the I2C driver."""
    if not BNO085_RST_USE:
        return
    try:
        import board  # type: ignore
        import digitalio  # type: ignore

        pin = getattr(board, BNO085_RST_PIN)
        rst = digitalio.DigitalInOut(pin)
        rst.direction = digitalio.Direction.OUTPUT
        rst.value = True
        time.sleep(0.002)
        rst.value = False
        time.sleep(0.01)
        rst.value = True
        time.sleep(0.65)
        try:
            rst.deinit()
        except Exception:
            pass
    except Exception as exc:
        logger.warning("IMU: BNO085 reset pulse skipped (%s)", exc)


def _drain_bno_packets(bno: Any, seconds: float) -> None:
    """Optional light drain before ``enable_feature``.

    **Avoid** huge ``max_packets`` loops: if the chip signals data-ready but the bus misbehaves,
    Adafruit's I2C reader can **block indefinitely** inside ``read_header``.
    """
    if seconds <= 0:
        return
    try:
        max_calls = int(os.environ.get("IMU_BOOT_DRAIN_MAX_CALLS", "12"), 0)
    except ValueError:
        max_calls = 12
    max_calls = max(1, min(max_calls, 40))
    per_call_max = int(os.environ.get("IMU_BOOT_DRAIN_MAX_PACKETS", "6"), 0) or 6
    per_call_max = max(1, min(per_call_max, 24))

    deadline = time.monotonic() + max(0.0, seconds)
    for _ in range(max_calls):
        if time.monotonic() >= deadline:
            break
        try:
            with i2c_bus.i2c_lock():
                if hasattr(bno, "_process_available_packets"):
                    bno._process_available_packets(max_packets=per_call_max)  # type: ignore[attr-defined]
        except Exception:
            pass
        time.sleep(0.04)


def _quat_to_euler_deg(qi: float, qj: float, qk: float, qr: float) -> tuple[float, float, float]:
    """Vector part (i,j,k) + real (r) → roll, pitch, yaw in degrees."""
    sinp = 2.0 * (qr * qj - qk * qi)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    roll = math.atan2(2.0 * (qr * qi + qj * qk), 1.0 - 2.0 * (qi * qi + qj * qj))
    yaw = math.atan2(2.0 * (qr * qk + qi * qj), 1.0 - 2.0 * (qj * qj + qk * qk))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def _wrap_360(angle: float) -> float:
    return float(angle) % 360.0


def _wrap_180(angle: float) -> float:
    return (float(angle) + 180.0) % 360.0 - 180.0


def _hampel_angle(name: str, value: float) -> float:
    window = _ANGLE_WINDOWS[name]
    window.append(_wrap_360(value))
    if len(window) > HAMPEL_WINDOW_SIZE:
        window.pop(0)
    if len(window) < 3:
        return _wrap_360(value)

    unwrapped = [window[0]]
    for item in window[1:]:
        unwrapped.append(unwrapped[-1] + _wrap_180(item - unwrapped[-1]))

    sorted_vals = sorted(unwrapped)
    n = len(sorted_vals)
    median = sorted_vals[n // 2] if n % 2 else (sorted_vals[n // 2 - 1] + sorted_vals[n // 2]) / 2.0
    deviations = sorted(abs(x - median) for x in unwrapped)
    mad = deviations[n // 2] if n % 2 else (deviations[n // 2 - 1] + deviations[n // 2]) / 2.0
    threshold = HAMPEL_THRESHOLD * max(mad, HAMPEL_MIN_MAD)
    filtered = [median if abs(x - median) > threshold else x for x in unwrapped]
    return _wrap_360(sum(filtered) / len(filtered))


def _mag_norm_is_valid(norm: float) -> bool:
    if not (MAG_FIELD_MIN <= norm <= MAG_FIELD_MAX):
        return False
    prev = _MAG_FILTER_STATE.get("norm")
    if prev is not None and float(prev) > 0.0:
        prev_f = float(prev)
        if norm > prev_f * MAG_NORM_SPIKE_RATIO or norm < prev_f / MAG_NORM_SPIKE_RATIO:
            return False
    return True


def _filter_mag(mx: float, my: float, mz: float) -> tuple[float, float, float]:
    if not _MAG_FILTER_STATE["init"]:
        _MAG_FILTER_STATE.update({"x": mx, "y": my, "z": mz, "init": True})
        return mx, my, mz
    a = MAG_FILTER_ALPHA
    _MAG_FILTER_STATE["x"] = a * mx + (1.0 - a) * float(_MAG_FILTER_STATE["x"])
    _MAG_FILTER_STATE["y"] = a * my + (1.0 - a) * float(_MAG_FILTER_STATE["y"])
    _MAG_FILTER_STATE["z"] = a * mz + (1.0 - a) * float(_MAG_FILTER_STATE["z"])
    return float(_MAG_FILTER_STATE["x"]), float(_MAG_FILTER_STATE["y"]), float(_MAG_FILTER_STATE["z"])


def _env_axis_sign(name: str, default: float) -> float:
    try:
        value = float(os.environ.get(name, str(default)))
    except ValueError:
        return default
    return -1.0 if value < 0.0 else 1.0


def _yaw_sign() -> float:
    return _env_axis_sign("IMU_YAW_SIGN", 1.0)


def _apply_yaw_convention(yaw_deg: float, gz_deg_s: float) -> tuple[float, float]:
    """Convert BNO yaw/Z-rate to the FSW heading convention."""
    sign = _yaw_sign()
    yaw = float(yaw_deg)
    rate_sign = sign
    if IMU_MOUNTED_ON_BOTTOM:
        yaw = -yaw
        rate_sign *= -1.0
    if IMU_FORWARD_AXIS == "Y":
        yaw += 90.0
    yaw *= sign
    return _wrap_360(yaw), rate_sign * gz_deg_s


def _apply_mag_yaw_correction(yaw_deg: float, mx: float, my: float) -> float:
    try:
        mag_heading = math.degrees(math.atan2(my, mx))
        if IMU_MOUNTED_ON_BOTTOM:
            mag_heading = -mag_heading
        if IMU_FORWARD_AXIS == "Y":
            mag_heading += 90.0
        mag_heading *= _yaw_sign()
        mag_heading = _wrap_360(mag_heading)
        return _wrap_360(yaw_deg + YAW_CORRECTION_GAIN * _wrap_180(mag_heading - yaw_deg))
    except Exception:
        return _wrap_360(yaw_deg)


def _init_imu_once() -> tuple[Any, Any]:
    from adafruit_bno08x import (  # type: ignore
        BNO_REPORT_ACCELEROMETER,
        BNO_REPORT_GAME_ROTATION_VECTOR,
        BNO_REPORT_GYROSCOPE,
        BNO_REPORT_MAGNETOMETER,
        BNO_REPORT_ROTATION_VECTOR,
    )
    from adafruit_bno08x.i2c import BNO08X_I2C  # type: ignore

    env_addr = os.environ.get("IMU_I2C_ADDR", "").strip()
    addr = int(env_addr, 0) if env_addr else 0x4A
    lib_debug = os.environ.get("BNO08X_DEBUG", "").strip() == "1"
    post_open_delay = float(os.environ.get("IMU_POST_OPEN_DELAY_SEC", "0.35"))
    # Default **off**: long drains repeatedly call ``_process_available_packets`` and can hang in I2C read.
    boot_drain = float(os.environ.get("IMU_BOOT_DRAIN_SEC", "0"))
    try:
        ctor_cycles = int(os.environ.get("IMU_OPEN_DRAIN_CYCLES", "2"), 0)
    except ValueError:
        ctor_cycles = 2
    ctor_cycles = max(0, min(ctor_cycles, 6))
    ctor_max_pkt = int(os.environ.get("IMU_OPEN_DRAIN_MAX_PACKETS", "8"), 0) or 8
    ctor_max_pkt = max(1, min(ctor_max_pkt, 24))

<<<<<<< HEAD
    _init_progress("IMU: open I2C + BNO08x …")
=======
    _pulse_bno085_reset()
>>>>>>> ad06d2286af0c258b80c723ffe82f23fbb1e65a2
    with i2c_bus.i2c_lock():
        i2c = i2c_bus.get_i2c()
        if not env_addr and hasattr(i2c, "scan"):
            try:
                addrs = set(i2c.scan())
                if 0x4A in addrs:
                    addr = 0x4A
                elif 0x4B in addrs:
                    addr = 0x4B
            except Exception:
                pass
        bno = BNO08X_I2C(i2c, address=addr, debug=lib_debug)
        if not lib_debug:
            try:
                bno._debug = False  # type: ignore[attr-defined]
            except Exception:
                pass
            def _noop_dbg(*_a: Any, **_k: Any) -> None:
                return None

            bno._dbg = _noop_dbg  # type: ignore[method-assign]
<<<<<<< HEAD
        for _ in range(ctor_cycles):
            if hasattr(bno, "_process_available_packets"):
                bno._process_available_packets(max_packets=ctor_max_pkt)  # type: ignore[attr-defined]
=======
        with open(os.devnull, "w", encoding="utf-8") as devnull:
            with redirect_stdout(devnull), redirect_stderr(devnull):
                for _ in range(4):
                    if hasattr(bno, "_process_available_packets"):
                        bno._process_available_packets(max_packets=24)  # type: ignore[attr-defined]
>>>>>>> ad06d2286af0c258b80c723ffe82f23fbb1e65a2

    if boot_drain > 0:
        _init_progress(f"IMU: boot drain {boot_drain:.2f}s (IMU_BOOT_DRAIN_SEC) …")
    _drain_bno_packets(bno, boot_drain)
    time.sleep(post_open_delay)

    # Accel → gyro → mag, then fusion (Adafruit / Hillcrest bring-up order; gyro-first hung some Pi+I2C setups).
    # If rotation_vector fails (mag / EMI), fall back to game_rotation_vector.
    bno._fsw_use_game_quat = False  # type: ignore[attr-defined]
    for feat in (
        BNO_REPORT_ACCELEROMETER,
        BNO_REPORT_GYROSCOPE,
        BNO_REPORT_MAGNETOMETER,
    ):
        with i2c_bus.i2c_lock():
            _enable_feature_retry(bno, feat)
    with i2c_bus.i2c_lock():
        try:
            _enable_feature_retry(bno, BNO_REPORT_ROTATION_VECTOR)
        except RuntimeError as exc:
            logger.warning(
                "IMU: rotation_vector not available (%s); enabling game_rotation_vector instead",
                exc,
            )
            _enable_feature_retry(bno, BNO_REPORT_GAME_ROTATION_VECTOR)
            bno._fsw_use_game_quat = True  # type: ignore[attr-defined]

    logger.info("IMU BNO08x OK at 0x%02x", addr)
    return i2c, bno


def init_imu() -> tuple[Any, Any]:
    try:
        rounds = int(os.environ.get("IMU_INIT_ROUNDS", "2"), 0)
    except ValueError:
        rounds = 2
    rounds = max(1, min(rounds, 4))
    last_exc: Optional[Exception] = None
    for attempt in range(rounds):
        try:
            return _init_imu_once()
        except RuntimeError as exc:
            last_exc = exc
            logger.warning("IMU: init round %d/%d failed: %s", attempt + 1, rounds, exc)
            if attempt + 1 < rounds:
                i2c_bus.reset_i2c()
                time.sleep(0.25 * (attempt + 1))
    raise RuntimeError(f"BNO08x: init failed after {rounds} round(s): {last_exc}") from last_exc


def read_sensor_data(bno) -> Any:
    """Return 12-tuple for `imuapp`, or False on soft failure."""
    r2d = 180.0 / math.pi
    for _ in range(READ_ATTEMPTS):
        try:
            with i2c_bus.i2c_lock():
                with open(os.devnull, "w", encoding="utf-8") as devnull:
                    with redirect_stdout(devnull), redirect_stderr(devnull):
                        if hasattr(bno, "_process_available_packets"):
                            bno._process_available_packets(max_packets=12)  # type: ignore[attr-defined]
                        quat = bno.game_quaternion if getattr(bno, "_fsw_use_game_quat", False) else bno.quaternion
                        acc = bno.acceleration
                        mag = bno.magnetic
                        gyr = bno.gyro

            if quat is None or any(v is None for v in quat):
                raise RuntimeError("BNO08x quaternion unavailable")
            qi, qj, qk, qr = quat
            roll, pitch, yaw = _quat_to_euler_deg(float(qi), float(qj), float(qk), float(qr))

            if acc is not None and not any(v is None for v in acc):
                ax, ay, az = (float(acc[0]), float(acc[1]), float(acc[2]))
                _LAST_VALID["acc"] = (ax, ay, az)
            else:
                ax, ay, az = _LAST_VALID["acc"]

            if mag is not None and not any(v is None for v in mag):
                raw_mx, raw_my, raw_mz = (float(mag[0]), float(mag[1]), float(mag[2]))
                mag_norm = math.sqrt(raw_mx * raw_mx + raw_my * raw_my + raw_mz * raw_mz)
                if _mag_norm_is_valid(mag_norm):
                    mx, my, mz = _filter_mag(raw_mx, raw_my, raw_mz)
                    _MAG_FILTER_STATE["norm"] = mag_norm
                    _LAST_VALID["mag"] = (mx, my, mz)
                else:
                    mx, my, mz = _LAST_VALID["mag"]
            else:
                mx, my, mz = _LAST_VALID["mag"]

            if gyr is not None and not any(v is None for v in gyr):
                gx, gy, gz = (float(gyr[0]), float(gyr[1]), float(gyr[2]))
                _LAST_VALID["gyr"] = (gx, gy, gz)
            else:
                gx, gy, gz = _LAST_VALID["gyr"]

            gz_deg_s = float(gz) * r2d
            yaw, gz_deg_s = _apply_yaw_convention(yaw, gz_deg_s)
            yaw = _apply_mag_yaw_correction(yaw, mx, my)
            roll = _hampel_angle("roll", roll)
            pitch = _hampel_angle("pitch", pitch)
            yaw = _hampel_angle("yaw", yaw)
            return (
                round(float(roll), 4),
                round(float(pitch), 4),
                round(float(yaw), 4),
                round(float(ax), 4),
                round(float(ay), 4),
                round(float(az), 4),
                round(float(mx), 4),
                round(float(my), 4),
                round(float(mz), 4),
                round(float(gx) * r2d, 4),
                round(float(gy) * r2d, 4),
                round(float(gz_deg_s), 4),
            )
        except Exception:
            time.sleep(0.002)
    return False


def reinit_imu(_i2c_old: Any, _bno_old: Any) -> tuple[Any, Any]:
    """Drop the cached bus so ``init_imu`` opens a fresh handle (``deinit`` alone left a dead singleton)."""
    i2c_bus.reset_i2c()
    for window in _ANGLE_WINDOWS.values():
        window.clear()
    _MAG_FILTER_STATE.update({"x": 0.0, "y": 0.0, "z": 0.0, "init": False, "norm": None})
    return init_imu()


def imu_terminate(_i2c: Any) -> None:
    i2c_bus.reset_i2c()


def _imu_cli_period_sec() -> float:
    try:
        return max(0.05, float(os.environ.get("IMU_PRINT_PERIOD_SEC", "1.0")))
    except ValueError:
        return 1.0


def _imu_read_period_sec() -> float:
    return 1.0 / _imu_read_rate_hz()


def _print_sample_header() -> None:
    print(
        "idx  | roll                 pitch                yaw                  | "
        "ax                    ay                    az                    acc_norm          | "
        "mx                 my                 mz                 mag_norm          | "
        "gx                    gy                    gz                    gyr_norm",
        flush=True,
    )
    print(
        "-----+----------------------------------------------------------------+"
        "--------------------------------------------------------------------------+"
        "----------------------------------------------------------------+"
        "--------------------------------------------------------------------------",
        flush=True,
    )


def _print_sample_line(sample: Any, sample_index: int) -> None:
    r, p, y, ax, ay, az, mx, my, mz, gx, gy, gz = sample
    acc_g = math.sqrt(ax * ax + ay * ay + az * az) / 9.80665
    mag_norm = math.sqrt(mx * mx + my * my + mz * mz)
    gyr_norm = math.sqrt(gx * gx + gy * gy + gz * gz)
    print(
        f"{sample_index:4d} | "
        f"roll={r:8.2f} deg  "
        f"pitch={p:8.2f} deg  "
        f"yaw={y:8.2f} deg | "
        f"ax={ax:9.3f} m/s^2  "
        f"ay={ay:9.3f} m/s^2  "
        f"az={az:9.3f} m/s^2  "
        f"acc={acc_g:6.3f} g | "
        f"mx={mx:8.2f} uT  "
        f"my={my:8.2f} uT  "
        f"mz={mz:8.2f} uT  "
        f"mag={mag_norm:8.2f} uT | "
        f"gx={gx:9.3f} deg/s  "
        f"gy={gy:9.3f} deg/s  "
        f"gz={gz:9.3f} deg/s  "
        f"gyr={gyr_norm:9.3f} deg/s",
        flush=True,
    )


if __name__ == "__main__":
    import logging

    logging.basicConfig(level=logging.INFO, format="%(message)s")
    os.environ.setdefault("IMU_INIT_PROGRESS", "1")
    print(
        "IMU: initializing BNO08x (15–60 s). If this hangs, set IMU_BOOT_DRAIN_SEC=0 (default), "
        "try FSW_I2C_FLOCK=0 for CLI-only tests, I2C 100 kHz, IMU_INIT_PROGRESS=1 shows each step.",
        flush=True,
    )
    try:
        _i2c, bno = init_imu()
    except Exception as exc:
        print(f"IMU: init failed: {exc}", flush=True)
        raise SystemExit(1) from exc
    period = _imu_cli_period_sec()
    read_period = _imu_read_period_sec()
    print(
        f"IMU: OK, reading at {_imu_read_rate_hz():.2f}Hz and printing every {period:.2f}s "
        "(set IMU_READ_RATE_HZ / IMU_PRINT_PERIOD_SEC to override).",
        flush=True,
    )
    print(
        f"IMU: config report_interval_us={REPORT_INTERVAL_US} "
        f"yaw_sign={_yaw_sign():+.0f} mounted_bottom={IMU_MOUNTED_ON_BOTTOM} "
        f"forward_axis={IMU_FORWARD_AXIS} mag_alpha={MAG_FILTER_ALPHA:.2f}",
        flush=True,
    )
    _print_sample_header()
    _last_fail_log = 0.0
    _sample_index = 0
    _latest_sample = None
    _next_print = time.monotonic()
    try:
        while True:
            s = read_sensor_data(bno)
            now = time.monotonic()
            if s is False:
                if now - _last_fail_log >= 2.0:
                    print(
                        "IMU: read failed (move module, check 0x4A/0x4B, BNO08X_DEBUG=1)",
                        flush=True,
                    )
                    _last_fail_log = now
            else:
                _latest_sample = s
            if _latest_sample is not None and now >= _next_print:
                _sample_index += 1
                if _sample_index > 1 and (_sample_index - 1) % 25 == 0:
                    _print_sample_header()
                _print_sample_line(_latest_sample, _sample_index)
                _next_print = now + period
            time.sleep(read_period)
    except KeyboardInterrupt:
        print("", flush=True)
    finally:
        imu_terminate(_i2c)
