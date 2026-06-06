"""CSV logger for IPC messages (main router) and raw sensor samples (sensor apps)."""

from __future__ import annotations

import csv
import math
import os
import re
import threading
import time
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
_motor_control_writer: Optional[Tuple[Any, csv.writer]] = None

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
    "monotonic_s",
    "event",
    "flight_state",
    "motor_enabled",
    "motor_ctrl_mode",
    "control_mode",
    "output_valid",
    "l1_valid",
    "l1_nominal",
    "l1_reason",
    "dr_method",
    "confidence_scale",
    "pid_enabled",
    "yaw_rate_limit_dps",
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
    "angular_velocity_error_deg_s",
    "motor_cmd",
    "saturated",
    "sensor_valid",
    "gyro_rejected",
    "gps_lat",
    "gps_lon",
    "gps_course_deg",
    "gps_speed_mps",
    "gps_pos_health",
    "gps_motion_health",
    "gps_pos_age_s",
    "gps_motion_age_s",
    "imu_roll_deg",
    "imu_pitch_deg",
    "imu_yaw_deg",
    "imu_gyrz_deg_s",
    "imu_health",
    "imu_age_s",
    "lin_acc_x_mps2",
    "lin_acc_y_mps2",
    "lin_acc_z_mps2",
    "lin_acc_valid",
    "lin_acc_reject_reason",
    "raw_acc_norm_mps2",
    "gravity_body_x_mps2",
    "gravity_body_y_mps2",
    "gravity_body_z_mps2",
    "lin_acc_xy_mag_mps2",
    "imu_sample_age_s",
    "baro_alt_m",
    "baro_sink_rate_mps",
    "baro_health",
    "baro_age_s",
    "target_lat",
    "target_lon",
    # ── guidance/control mode + reason 체인 (확장) ──
    "mode_reason",
    "l1input_valid",
    "l1input_reason",
    "l1output_valid",
    "l1output_reason",
    "ctrl_valid",
    "ctrl_reason",
    # ── sensor fresh flags ──
    "gps_pos_fresh",
    "gps_motion_fresh",
    "imu_gyrz_fresh",
    "imu_yaw_fresh",
    "baro_sink_fresh",
    "acc_fresh",
    # ── DR validity ──
    "dr_anchor_valid",
    "dr_current_valid",
    "dr_confidence",
    "dr_age_s",
    # ── navigation ──
    "nav_E",
    "nav_N",
    "nav_V",
    "nav_course_deg",
    "nav_vE",
    "nav_vN",
    # ── DR current state ──
    "dr_current_E",
    "dr_current_N",
    "dr_current_V",
    "dr_current_course_deg",
    # ── L1 / control 추가 ──
    "yaw_rate_cmd_dps",
    "kp_used",
    "ff_scale",
    "d_total",
    # ── DR 조향 원인 추적 (강화: nu/FF/course 분해; 뒤에만 추가) ──
    "nu_clamped_deg",
    "sin_nu_eff",
    "yaw_rate_cmd_pre_conf_dps",
    "course_gyro_deg",
    "course_yaw_deg",
    "course_selected_deg",
    "course_source_reason",
    "ff_deadband_dps",
    "ff_ref_dps",
    "dr_pid_active",
    "delta_ff_pre_cap_deg",
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


def _motor_i(obj, attr, default: int = 0) -> int:
    try:
        return int(getattr(obj, attr, default))
    except (TypeError, ValueError):
        return default


def _motor_bool(obj, attr) -> int:
    return int(bool(getattr(obj, attr, False)))


def _motor_age(now: float, ts) -> float:
    try:
        ts_f = float(ts)
    except (TypeError, ValueError):
        return float("nan")
    return now - ts_f if math.isfinite(ts_f) else float("nan")


def _motor_session_stamp() -> str:
    root = _raw_session_dir()
    if root is not None and root.name.startswith("run_"):
        return root.name[4:]
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def _get_motor_control_writer(header: list[str]) -> Optional[csv.writer]:
    """Dedicated motor-control CSV in motorlogs/, independent of sensorlogs."""
    global _motor_control_writer
    with _raw_lock:
        if _motor_control_writer is not None:
            return _motor_control_writer[1]
        root = Path("motorlogs")
        root.mkdir(parents=True, exist_ok=True)
        path = root / f"motor_control_{_motor_session_stamp()}.csv"
        fp = path.open("a", encoding="utf-8", newline="")
        writer = csv.writer(fp)
        writer.writerow(header)
        fp.flush()
        _motor_control_writer = (fp, writer)
        return writer


def _flush_motor_control() -> None:
    with _raw_lock:
        pair = _motor_control_writer
        if pair:
            try:
                pair[0].flush()
            except Exception:
                pass


def log_motor_raw(
    flight_state: int,
    motor_enabled: bool,
    motor_ctrl_mode: str,
    ctrl_out,
    l1_out=None,
    snap=None,
    event: str = "",
    l1_in=None,
) -> None:
    """패러포일 제어 사이클 1회 출력 + guidance 상태."""
    try:
        raw_w = _get_raw_writer("motor", _MOTOR_RAW_HEADER)
        motor_w = _get_motor_control_writer(_MOTOR_RAW_HEADER)
        if raw_w is None and motor_w is None:
            return

        now = time.monotonic()
        gps = getattr(snap, "latest_gps", None)
        imu = getattr(snap, "latest_imu", None)
        baro = getattr(snap, "latest_baro", None)

        # guidance 내부 상태(flags/dr/nav)는 로깅 편의를 위해 지연 임포트로 읽는다.
        # guidance는 sensorlog를 임포트하지 않으므로 순환 임포트가 없다.
        flags = dr = nav = None
        try:
            from Sensor_Motor import guidance as _g
            _st = _g._STATE_t
            flags, dr, nav = _st.flags, _st.dr, _st.nav
        except Exception:
            pass
        try:
            _anchor_t = float(getattr(dr, "anchor_time", float("nan")))
            dr_age_s = now - _anchor_t if math.isfinite(_anchor_t) else float("nan")
        except (TypeError, ValueError):
            dr_age_s = float("nan")

        row = [
            datetime.now().isoformat(timespec="milliseconds"),
            _motor_f(ctrl_out, "timestamp", now),
            str(event),
            int(flight_state),
            int(bool(motor_enabled)),
            str(motor_ctrl_mode),
            str(getattr(ctrl_out, "control_mode", "")),
            _motor_bool(ctrl_out, "valid"),
            _motor_bool(l1_out, "control_valid"),
            _motor_bool(l1_out, "nominal"),
            str(getattr(l1_out, "reason", "")),
            str(getattr(l1_out, "dr_method", "")),
            _motor_f(l1_out, "confidence"),
            _motor_bool(l1_out, "pid_enabled"),
            _motor_f(l1_out, "yaw_rate_limit_dps"),
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
            _motor_f(ctrl_out, "angular_velocity_error_deg_s"),
            _motor_f(ctrl_out, "motor_cmd"),
            int(bool(getattr(ctrl_out, "saturated", False))),
            int(bool(getattr(ctrl_out, "sensor_valid", False))),
            int(bool(getattr(ctrl_out, "gyro_rejected", False))),
            _motor_f(gps, "lat"),
            _motor_f(gps, "lon"),
            _motor_deg(gps, "course_rad"),
            _motor_f(gps, "speed_mps"),
            _motor_i(gps, "pos_health"),
            _motor_i(gps, "motion_health"),
            _motor_age(now, getattr(gps, "pos_ts", None)),
            _motor_age(now, getattr(gps, "motion_ts", None)),
            _motor_deg(imu, "roll_rad"),
            _motor_deg(imu, "pitch_rad"),
            _motor_deg(imu, "yaw_rad"),
            _motor_deg(imu, "gyrz_rad_s"),
            _motor_i(imu, "health"),
            _motor_age(now, getattr(imu, "ts", None)),
            _motor_f(imu, "lin_acc_x"),
            _motor_f(imu, "lin_acc_y"),
            _motor_f(imu, "lin_acc_z"),
            int(bool(getattr(imu, "lin_acc_valid", False))),
            getattr(imu, "lin_acc_reject_reason", ""),
            _motor_f(imu, "raw_acc_norm_mps2"),
            _motor_f(imu, "gravity_body_x_mps2"),
            _motor_f(imu, "gravity_body_y_mps2"),
            _motor_f(imu, "gravity_body_z_mps2"),
            _motor_f(imu, "lin_acc_xy_mag_mps2"),
            _motor_age(now, getattr(imu, "ts", None)),
            _motor_f(baro, "alt_m"),
            _motor_f(baro, "sink_rate"),
            _motor_i(baro, "health"),
            _motor_age(now, getattr(baro, "rx_ts", None)),
            _motor_f(snap, "target_lat"),
            _motor_f(snap, "target_lon"),
            # ── guidance/control mode + reason 체인 ──
            str(getattr(nav, "fail_reason", "")),
            _motor_bool(l1_in, "valid"),
            str(getattr(l1_in, "reason", "")),
            _motor_bool(l1_out, "valid"),
            str(getattr(l1_out, "reason", "")),
            _motor_bool(ctrl_out, "valid"),
            str(getattr(ctrl_out, "reason", "")),
            # ── sensor fresh flags ──
            _motor_bool(flags, "gps_pos_fresh"),
            _motor_bool(flags, "gps_motion_fresh"),
            _motor_bool(flags, "imu_gyrz_fresh"),
            _motor_bool(flags, "imu_yaw_fresh"),
            _motor_bool(flags, "baro_sink_fresh"),
            _motor_bool(flags, "acc_fresh"),
            # ── DR validity ──
            _motor_bool(flags, "dr_anchor_valid"),
            _motor_bool(flags, "dr_current_valid"),
            _motor_f(dr, "confidence"),
            dr_age_s,
            # ── navigation ──
            _motor_f(nav, "E"),
            _motor_f(nav, "N"),
            _motor_f(nav, "V"),
            _motor_deg(nav, "course"),
            _motor_f(nav, "vE"),
            _motor_f(nav, "vN"),
            # ── DR current state ──
            _motor_f(dr, "current_E"),
            _motor_f(dr, "current_N"),
            _motor_f(dr, "current_V"),
            _motor_deg(dr, "current_course"),
            # ── L1 / control 추가 ──
            _motor_deg(l1_out, "yaw_rate_cmd"),
            _motor_f(ctrl_out, "kp_used"),
            _motor_f(ctrl_out, "ff_scale"),
            _motor_f(ctrl_out, "delta_total_deg"),
            # ── DR 조향 원인 추적 (강화) ──
            _motor_deg(l1_out, "nu_clamped"),
            _motor_f(l1_out, "sin_nu_eff"),
            _motor_deg(l1_out, "yaw_rate_cmd_pre_conf"),
            _motor_deg(dr, "dbg_course_gyro"),
            _motor_deg(dr, "dbg_course_yaw"),
            _motor_deg(dr, "dbg_course_selected"),
            str(getattr(dr, "dbg_course_reason", "")),
            _motor_f(ctrl_out, "ff_deadband_dps"),
            _motor_f(ctrl_out, "ff_ref_dps"),
            int(bool(getattr(ctrl_out, "dr_pid_active", False))),
            _motor_f(ctrl_out, "delta_ff_pre_cap_deg"),
        ]

        if raw_w is not None:
            raw_w.writerow(row)
            _flush_raw("motor")
        if motor_w is not None:
            motor_w.writerow(row)
            _flush_motor_control()
        return

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
    global _writers, _session_dir, _raw_writers, _motor_control_writer
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
        if _motor_control_writer is not None:
            try:
                _motor_control_writer[0].flush()
                _motor_control_writer[0].close()
            except Exception:
                pass
            _motor_control_writer = None
    _session_dir = None
