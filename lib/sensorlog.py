"""CSV logger for IPC messages (main router) and raw sensor samples (sensor apps)."""

from __future__ import annotations

import csv
import math
import os
import re
import threading
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, Optional, Tuple

from lib import appargs

_writers: Dict[int, Tuple[Any, csv.writer]] = {}
_session_dir: Optional[Path] = None

# Subprocess raw-sample writers (Barometer/IMU/GPS/Distance/Electro apps)
ENV_SENSORLOG_SESSION = "FSW_SENSORLOG_SESSION"
_raw_writers: Dict[str, Tuple[Any, csv.writer]] = {}
_raw_lock = threading.Lock()

_GPS_RAW_FIELDS = [
    "gps_time",
    "alt_m",
    "lat",
    "lon",
    "sats",
    "fix_quality",
    "rmc_status",
    "speed_ms",
    "course_deg",
    "motion_valid",
    "pos_age_s",
    "motion_age_s",
    "source",
    "fix_type",
    "h_acc_m",
    "v_acc_m",
    "s_acc_mps",
    "head_acc_deg",
    "vel_n",
    "vel_e",
    "valid_flags",
]

_APP_NAMES = {
    appargs.MainAppArg.AppID: appargs.MainAppArg.AppName,
    appargs.FlightlogicAppArg.AppID: appargs.FlightlogicAppArg.AppName,
    appargs.CommAppArg.AppID: appargs.CommAppArg.AppName,
    appargs.BarometerAppArg.AppID: appargs.BarometerAppArg.AppName,
    appargs.ImuAppArg.AppID: appargs.ImuAppArg.AppName,
    appargs.GpsAppArg.AppID: appargs.GpsAppArg.AppName,
    appargs.DistanceAppArg.AppID: appargs.DistanceAppArg.AppName,
    appargs.ElectroAppArg.AppID: appargs.ElectroAppArg.AppName,
    appargs.CameraAppArg.AppID: appargs.CameraAppArg.AppName,
    appargs.MotorAppArg.AppID: appargs.MotorAppArg.AppName,
}


def _name(app_id: int) -> str:
    return _APP_NAMES.get(int(app_id), f"App{app_id}")


def _safe_filename(label: str) -> str:
    """ASCII 파일명에 안전하게 줄인 뒤 .csv 접미사용 베이스 이름."""
    s = re.sub(r"[^A-Za-z0-9._-]+", "_", label.strip())
    return s or "unknown"


def _header_row() -> list[str]:
    return [
        "timestamp",
        "sender_app",
        "sender_name",
        "receiver_app",
        "receiver_name",
        "msg_id",
        "data",
    ]


def _get_writer(sender_app: int) -> csv.writer:
    global _writers, _session_dir
    sender_app = int(sender_app)
    if sender_app in _writers:
        return _writers[sender_app][1]
    if _session_dir is None:
        raise RuntimeError("sensorlog not initialized")

    base = _safe_filename(_name(sender_app))
    path = _session_dir / f"{base}.csv"
    fp = path.open("a", encoding="utf-8", newline="")
    writer = csv.writer(fp)
    writer.writerow(_header_row())
    _writers[sender_app] = (fp, writer)
    fp.flush()
    return writer


def init_sensorlog_main_process() -> None:
    """Prepare session directory; CSV files are created per sender on first message."""
    global _session_dir
    if _session_dir is not None:
        return
    root = Path("sensorlogs")
    root.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    _session_dir = root / f"run_{ts}"
    _session_dir.mkdir(parents=True, exist_ok=True)
    os.environ[ENV_SENSORLOG_SESSION] = str(_session_dir.resolve())


def _raw_session_dir() -> Optional[Path]:
    text = os.environ.get(ENV_SENSORLOG_SESSION)
    if not text:
        return None
    path = Path(text)
    return path if path.is_dir() else None


def _get_raw_writer(name: str, header: list[str]) -> Optional[csv.writer]:
    """Lazy CSV writer into ``<session>/raw_<name>.csv`` (sensor subprocess only)."""
    global _raw_writers
    with _raw_lock:
        if name in _raw_writers:
            return _raw_writers[name][1]
        root = _raw_session_dir()
        if root is None:
            return None
        path = root / f"raw_{name}.csv"
        new_file = not path.exists()
        fp = path.open("a", encoding="utf-8", newline="")
        writer = csv.writer(fp)
        if new_file:
            writer.writerow(header)
            fp.flush()
        _raw_writers[name] = (fp, writer)
        return writer


def _flush_raw(name: str) -> None:
    with _raw_lock:
        pair = _raw_writers.get(name)
        if pair:
            try:
                pair[0].flush()
            except Exception:
                pass


def log_barometer_raw(pressure_pa: float, temperature_c: float, altitude_raw_m: float) -> None:
    """One BMP sample as returned by the driver (before median filter / altitude offset)."""
    try:
        w = _get_raw_writer(
            "barometer",
            ["timestamp", "pressure_pa", "temperature_c", "altitude_raw_m"],
        )
        if w is None:
            return
        w.writerow(
            [
                datetime.now().isoformat(timespec="milliseconds"),
                pressure_pa,
                temperature_c,
                altitude_raw_m,
            ]
        )
        _flush_raw("barometer")
    except Exception:
        pass


def log_imu_raw(sample: Iterable[float]) -> None:
    """One IMU frame: roll, pitch, yaw, acc(3), mag(3), gyro(3) from the driver (before EMA)."""
    try:
        vals = tuple(float(x) for x in sample)
        if len(vals) < 12:
            return
        w = _get_raw_writer(
            "imu",
            [
                "timestamp",
                "roll_deg",
                "pitch_deg",
                "yaw_deg",
                "acc_x",
                "acc_y",
                "acc_z",
                "mag_x",
                "mag_y",
                "mag_z",
                "gyr_x",
                "gyr_y",
                "gyr_z",
            ],
        )
        if w is None:
            return
        w.writerow([datetime.now().isoformat(timespec="milliseconds"), *vals[:12]])
        _flush_raw("imu")
    except Exception:
        pass


def log_gps_raw(row: Optional[list]) -> None:
    """GNSS row as returned by ``gps_readdata`` / SIM inject (before median / health logic)."""
    try:
        w = _get_raw_writer("gps", ["timestamp", *_GPS_RAW_FIELDS])
        if w is None:
            return
        ts = datetime.now().isoformat(timespec="milliseconds")
        if row is None:
            w.writerow([ts] + [""] * len(_GPS_RAW_FIELDS))
        else:
            cells = []
            for i in range(len(_GPS_RAW_FIELDS)):
                cells.append(row[i] if i < len(row) else "")
            w.writerow([ts, *cells])
        _flush_raw("gps")
    except Exception:
        pass


def log_distance_raw(range_mm: float) -> None:
    """TF-Luna range read before validity gate / median window."""
    try:
        w = _get_raw_writer("distance", ["timestamp", "range_mm_raw"])
        if w is None:
            return
        w.writerow([datetime.now().isoformat(timespec="milliseconds"), range_mm])
        _flush_raw("distance")
    except Exception:
        pass


_MOTOR_RAW_HEADER = [
    "timestamp",
    "flight_state",
    "motor_enabled",
    "motor_ctrl_mode",
    "control_mode",
    "pos_N",
    "pos_E",
    "target_N",
    "target_E",
    "dist_to_target_m",
    "bearing_to_target_deg",
    "crosstrack_angle_deg",
    "ground_speed_mps",
    "left_pw",
    "right_pw",
    "left_angle_deg",
    "right_angle_deg",
    "delta_ff_deg",
    "delta_pid_deg",
    "delta_arm_deg",
    "angular_velocity_cmd_deg_s",
    "angular_velocity_meas_deg_s",
    "saturated",
    "sensor_valid",
]


def _motor_f(obj, attr, _nan=float("nan")):
    try:
        v = getattr(obj, attr, _nan)
        return float(v) if v is not None else _nan
    except (TypeError, ValueError):
        return float("nan")


def _motor_deg(obj, attr):
    v = _motor_f(obj, attr)
    return math.degrees(v) if math.isfinite(v) else float("nan")


def log_motor_raw(
    flight_state: int,
    motor_enabled: bool,
    motor_ctrl_mode: str,
    ctrl_out,
    l1_out=None,
) -> None:
    """패러포일 제어 사이클 1회 출력 + guidance 상태."""
    try:
        w = _get_raw_writer("motor", _MOTOR_RAW_HEADER)
        if w is None:
            return
        w.writerow([
            datetime.now().isoformat(timespec="milliseconds"),
            int(flight_state),
            int(bool(motor_enabled)),
            str(motor_ctrl_mode),
            str(getattr(ctrl_out, "control_mode", "")),
            _motor_f(l1_out, "pos_N"),
            _motor_f(l1_out, "pos_E"),
            _motor_f(l1_out, "target_N"),
            _motor_f(l1_out, "target_E"),
            _motor_f(l1_out, "distance_to_target"),
            _motor_deg(l1_out, "target_bearing"),
            _motor_deg(l1_out, "nu"),
            _motor_f(l1_out, "ground_speed_mps"),
            _motor_f(ctrl_out, "left_pw"),
            _motor_f(ctrl_out, "right_pw"),
            _motor_f(ctrl_out, "left_angle_deg"),
            _motor_f(ctrl_out, "right_angle_deg"),
            _motor_f(ctrl_out, "delta_ff_deg"),
            _motor_f(ctrl_out, "delta_pid_deg"),
            _motor_f(ctrl_out, "delta_arm_deg"),
            _motor_f(ctrl_out, "angular_velocity_cmd_deg_s"),
            _motor_f(ctrl_out, "angular_velocity_meas_deg_s"),
            int(bool(getattr(ctrl_out, "saturated", False))),
            int(bool(getattr(ctrl_out, "sensor_valid", False))),
        ])
        _flush_raw("motor")
    except Exception:
        pass


def log_electro_raw(voltage_v: float, current_a: float, power_w: float) -> None:
    """INA228 sample before median smoothing."""
    try:
        w = _get_raw_writer(
            "electro",
            ["timestamp", "voltage_v", "current_a", "power_w"],
        )
        if w is None:
            return
        w.writerow(
            [
                datetime.now().isoformat(timespec="milliseconds"),
                voltage_v,
                current_a,
                power_w,
            ]
        )
        _flush_raw("electro")
    except Exception:
        pass


def log_bus_message(msg) -> None:
    """Append one unpacked bus message into the CSV for msg.sender_app."""
    if _session_dir is None:
        return
    writer = _get_writer(msg.sender_app)
    writer.writerow(
        [
            datetime.now().isoformat(timespec="milliseconds"),
            int(msg.sender_app),
            _name(msg.sender_app),
            int(msg.receiver_app),
            _name(msg.receiver_app),
            int(msg.msg_id),
            str(msg.data),
        ]
    )
    fp = _writers[int(msg.sender_app)][0]
    fp.flush()


def shutdown_sensorlog() -> None:
    global _writers, _session_dir, _raw_writers
    for fp, _ in list(_writers.values()):
        try:
            fp.flush()
            fp.close()
        except Exception:
            pass
    _writers.clear()
    with _raw_lock:
        for fp, _ in list(_raw_writers.values()):
            try:
                fp.flush()
                fp.close()
            except Exception:
                pass
        _raw_writers.clear()
    _session_dir = None
