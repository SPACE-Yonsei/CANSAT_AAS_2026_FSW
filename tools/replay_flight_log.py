from __future__ import annotations

import argparse
import csv
import importlib
import math
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from Sensor_Motor import control, guidance, motorapp
from lib import config


DEFAULT_RAW_MOTOR = ROOT / "motorlogs" / "motor_control_20260604_161105.csv"
DEFAULT_OUT = ROOT / "replay_control_output.csv"
DEFAULT_TARGET_OFFSET_N_M = 30.0
DEFAULT_TARGET_OFFSET_E_M = 0.0
ALLOWED_FAIL_REASONS = {
    "NO_ORIGIN",
    "NO_TARGET",
    "NO_GUIDANCE_SOURCE",
    "DR_TIMEOUT",
    "FAIL",
}

OUTPUT_COLUMNS = [
    "time_s",
    "flight_state",
    "baro_alt_m",
    "mode",
    "fail_reason",
    "gps_pos_fresh",
    "gps_motion_fresh",
    "imu_gyrz_fresh",
    "imu_yaw_fresh",
    "baro_sink_fresh",
    "acc_fresh",
    "l1_valid",
    "l1_reason",
    "yaw_rate_cmd_dps",
    "yaw_rate_limit_dps",
    "gyrz_meas_dps",
    "delta_ff_deg",
    "delta_pid_deg",
    "delta_arm_deg",
    "left_angle_deg",
    "right_angle_deg",
    "left_pw",
    "right_pw",
    "saturated",
    "gyro_rejected",
    "baro_sink_spike",
    "dr_confidence",
    "dr_age_s",
    "distance_to_target",
    "nu_deg",
]


@dataclass
class Event:
    time_s: float
    kind: str
    payload: str
    state: int | None = None
    motor_enabled: bool | None = None


class FakePi:
    def __init__(self):
        self.calls = []

    def set_servo_pulsewidth(self, pin, pulsewidth):
        self.calls.append((pin, pulsewidth))


def _f(row, key, default=math.nan) -> float:
    value = row.get(key, "")
    if value in ("", "nan", "None", None):
        return default
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return out if math.isfinite(out) else default


def _i(row, key, default=0) -> int:
    value = _f(row, key, default)
    return int(value) if math.isfinite(value) else default


def _b(value) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _mode_value(mode) -> str:
    raw = getattr(mode, "value", mode)
    text = str(raw)
    return text.replace("ControlMode.", "")


def _csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as f:
        return list(csv.DictReader(f))


def _origin_target_from_offset(
    lat: float, lon: float, north_m: float, east_m: float
) -> tuple[float, float]:
    target_lat = lat + math.degrees(north_m / guidance.EARTH_RADIUS_M)
    target_lon = lon + math.degrees(
        east_m / (guidance.EARTH_RADIUS_M * math.cos(math.radians(lat)))
    )
    return target_lat, target_lon


def _reconstruct_raw_acc(row: dict[str, str]) -> tuple[float, float, float]:
    vals = []
    for lin_key, gravity_key in (
        ("lin_acc_x_mps2", "gravity_body_x_mps2"),
        ("lin_acc_y_mps2", "gravity_body_y_mps2"),
        ("lin_acc_z_mps2", "gravity_body_z_mps2"),
    ):
        lin = _f(row, lin_key)
        gravity = _f(row, gravity_key)
        if math.isfinite(lin) and math.isfinite(gravity):
            vals.append(lin + gravity)
        else:
            vals.append(math.nan)
    if all(math.isfinite(v) for v in vals):
        return tuple(vals)
    return (0.0, 0.0, 9.81)


def _embedded_motor_events(path: Path) -> list[Event]:
    events: list[Event] = []
    for row in _csv_rows(path):
        t = _f(row, "monotonic_s")
        if not math.isfinite(t):
            continue
        state = _i(row, "flight_state", 4)
        motor_enabled = _b(row.get("motor_enabled", "1"))

        gps_lat = _f(row, "gps_lat")
        gps_lon = _f(row, "gps_lon")
        gps_pos_health = _i(row, "gps_pos_health", 0)
        gps_motion_health = _i(row, "gps_motion_health", 0)
        gps_course_deg = _f(row, "gps_course_deg", 0.0)
        gps_speed_mps = _f(row, "gps_speed_mps", 0.0)
        if math.isfinite(gps_lat) and math.isfinite(gps_lon):
            events.append(
                Event(
                    t,
                    "gps",
                    f"{gps_lat},{gps_lon},{gps_pos_health},{t},"
                    f"{gps_course_deg},{gps_speed_mps},{gps_motion_health},{t}",
                    state,
                    motor_enabled,
                )
            )

        imu_health = _i(row, "imu_health", 0)
        roll = _f(row, "imu_roll_deg", 0.0)
        pitch = _f(row, "imu_pitch_deg", 0.0)
        yaw = _f(row, "imu_yaw_deg", 0.0)
        nav_gyrz = _f(row, "imu_gyrz_deg_s", 0.0)
        raw_gyrz = -nav_gyrz if math.isfinite(nav_gyrz) else math.nan
        accx, accy, accz = _reconstruct_raw_acc(row)
        if imu_health:
            events.append(
                Event(
                    t,
                    "imu",
                    f"{roll},{pitch},{yaw},{accx},{accy},{accz},0.0,0.0,"
                    f"{raw_gyrz},{imu_health},{t},0.0",
                    state,
                    motor_enabled,
                )
            )

        baro_health = _i(row, "baro_health", 0)
        baro_alt = _f(row, "baro_alt_m")
        baro_sink = _f(row, "baro_sink_rate_mps")
        if math.isfinite(baro_alt) or baro_health:
            events.append(
                Event(
                    t,
                    "baro",
                    f"{baro_alt},{baro_sink},{baro_health}",
                    state,
                    motor_enabled,
                )
            )

        events.append(Event(t, "cycle", "", state, motor_enabled))
    return sorted(events, key=lambda e: (e.time_s, e.kind == "cycle"))


def _message_log_events(log_dir: Path) -> list[Event]:
    events: list[Event] = []
    mapping = {
        "GPS.csv": "gps",
        "Gps.csv": "gps",
        "IMU.csv": "imu",
        "Barometer.csv": "baro",
    }
    for filename, kind in mapping.items():
        path = log_dir / filename
        if not path.exists():
            continue
        rows = _csv_rows(path)
        if not rows:
            continue
        t0 = None
        for index, row in enumerate(rows):
            data = row.get("data", "")
            if not data:
                continue
            # Prefer messages sent to Motor, but keep direct raw fixtures too.
            receiver = row.get("receiver_name", "")
            receiver_app = row.get("receiver_app", "")
            if receiver and receiver != "Motor" and receiver_app != "19":
                continue
            timestamp = row.get("timestamp", "")
            t = _f(row, "monotonic_s")
            if not math.isfinite(t):
                t = float(index)
            if t0 is None:
                t0 = t
            events.append(Event(t - t0, kind, data))
    for event in list(events):
        events.append(Event(event.time_s, "cycle", ""))
    return sorted(events, key=lambda e: (e.time_s, e.kind == "cycle"))


def load_events(raw_motor: Path | None = None, log_dir: Path | None = None) -> list[Event]:
    if raw_motor is not None:
        return _embedded_motor_events(raw_motor)
    if log_dir is not None:
        return _message_log_events(log_dir)
    return _embedded_motor_events(DEFAULT_RAW_MOTOR)


def _first_valid_gps(events: list[Event]) -> tuple[float, float] | None:
    for event in events:
        if event.kind != "gps":
            continue
        parts = event.payload.split(",")
        if len(parts) < 3:
            continue
        try:
            lat = float(parts[0])
            lon = float(parts[1])
            health = int(float(parts[2]))
        except ValueError:
            continue
        if health and math.isfinite(lat) and math.isfinite(lon):
            return lat, lon
    return None


def reset_for_replay(
    events: list[Event],
    *,
    target_lat: float | None = None,
    target_lon: float | None = None,
    target_offset_n_m: float = DEFAULT_TARGET_OFFSET_N_M,
    target_offset_e_m: float = DEFAULT_TARGET_OFFSET_E_M,
) -> FakePi:
    global control, guidance, motorapp
    guidance = importlib.reload(guidance)
    control = importlib.reload(control)
    motorapp = importlib.reload(motorapp)
    control.reset()

    first_gps = _first_valid_gps(events)
    if first_gps is not None:
        guidance.set_origin_point(*first_gps)
        motorapp._ORIGIN_LOCKED = True
        if target_lat is None or target_lon is None:
            target_lat, target_lon = _origin_target_from_offset(
                first_gps[0], first_gps[1], target_offset_n_m, target_offset_e_m
            )
    if target_lat is not None and target_lon is not None:
        guidance.set_target_point(target_lat, target_lon)

    fake_pi = FakePi()
    motorapp.PI = fake_pi
    motorapp.STATE = 4
    motorapp.MOTOR_ENABLED = True
    motorapp._STEER_MODE = ""
    return fake_pi


def _row_from_cycle(now: float, ctrl_out, captured: dict) -> dict[str, object]:
    st = guidance._STATE_t
    flags = st.flags
    l1_in = captured.get("l1_in")
    l1_out = captured.get("l1_out")
    snap = captured.get("snap")
    baro_alt = getattr(getattr(snap, "latest_baro", SimpleNamespace()), "alt_m", math.nan)
    imu_gyrz = getattr(getattr(snap, "latest_imu", SimpleNamespace()), "gyrz_rad_s", math.nan)
    dr_age = now - st.dr.anchor_time if math.isfinite(st.dr.anchor_time) else math.nan
    l1_valid = bool(getattr(l1_out, "valid", False) or getattr(l1_in, "valid", False))
    l1_reason = (
        getattr(l1_out, "reason", "")
        or getattr(l1_in, "reason", "")
        or getattr(ctrl_out, "reason", "")
    )
    return {
        "time_s": now,
        "flight_state": motorapp.STATE,
        "baro_alt_m": baro_alt,
        "mode": _mode_value(st.nav.control_mode),
        "fail_reason": st.nav.fail_reason,
        "gps_pos_fresh": flags.gps_pos_fresh,
        "gps_motion_fresh": flags.gps_motion_fresh,
        "imu_gyrz_fresh": flags.imu_gyrz_fresh,
        "imu_yaw_fresh": flags.imu_yaw_fresh,
        "baro_sink_fresh": flags.baro_sink_fresh,
        "acc_fresh": flags.acc_fresh,
        "l1_valid": l1_valid,
        "l1_reason": l1_reason,
        "yaw_rate_cmd_dps": math.degrees(getattr(l1_out, "yaw_rate_cmd", 0.0) or 0.0)
        if l1_out is not None
        else 0.0,
        "yaw_rate_limit_dps": getattr(l1_out, "yaw_rate_limit_dps", 0.0)
        if l1_out is not None
        else 0.0,
        "gyrz_meas_dps": math.degrees(float(imu_gyrz))
        if math.isfinite(float(imu_gyrz or math.nan))
        else math.nan,
        "delta_ff_deg": getattr(ctrl_out, "delta_ff_deg", 0.0),
        "delta_pid_deg": getattr(ctrl_out, "delta_pid_deg", 0.0),
        "delta_arm_deg": getattr(ctrl_out, "delta_arm_deg", 0.0),
        "left_angle_deg": getattr(ctrl_out, "left_angle_deg", control.NEUTRAL_ARM_DEG),
        "right_angle_deg": getattr(ctrl_out, "right_angle_deg", control.NEUTRAL_ARM_DEG),
        "left_pw": getattr(ctrl_out, "left_pw", control.LEFT_NEUTRAL),
        "right_pw": getattr(ctrl_out, "right_pw", control.RIGHT_NEUTRAL),
        "saturated": getattr(ctrl_out, "saturated", False),
        "gyro_rejected": getattr(ctrl_out, "gyro_rejected", False),
        "baro_sink_spike": st.nav.baro_sink_spike,
        "dr_confidence": st.dr.confidence,
        "dr_age_s": dr_age,
        "distance_to_target": getattr(l1_out, "distance_to_target", math.nan)
        if l1_out is not None
        else math.nan,
        "nu_deg": math.degrees(getattr(l1_out, "nu", math.nan))
        if l1_out is not None and math.isfinite(getattr(l1_out, "nu", math.nan))
        else math.nan,
    }


def replay_events(
    events: list[Event],
    *,
    target_lat: float | None = None,
    target_lon: float | None = None,
    target_offset_n_m: float = DEFAULT_TARGET_OFFSET_N_M,
    target_offset_e_m: float = DEFAULT_TARGET_OFFSET_E_M,
) -> list[dict[str, object]]:
    reset_for_replay(
        events,
        target_lat=target_lat,
        target_lon=target_lon,
        target_offset_n_m=target_offset_n_m,
        target_offset_e_m=target_offset_e_m,
    )
    rows: list[dict[str, object]] = []
    captured: dict = {}

    def capture_log(state, motor_enabled, motor_ctrl_mode, ctrl_out, l1_out=None, **kwargs):
        captured.clear()
        captured.update(
            ctrl_out=ctrl_out,
            l1_out=l1_out,
            l1_in=kwargs.get("l1_in"),
            snap=kwargs.get("snap"),
        )

    with (
        patch.object(motorapp.sensorlog, "log_motor_raw", side_effect=capture_log),
        patch.object(motorapp.prevstate, "update_start_point", return_value=None),
        patch.object(motorapp.prevstate, "update_motor_enabled", return_value=None),
    ):
        for event in events:
            now = event.time_s
            if event.state is not None:
                motorapp.STATE = int(event.state)
            if event.motor_enabled is not None:
                motorapp.MOTOR_ENABLED = bool(event.motor_enabled)
            with patch.object(motorapp.time, "monotonic", return_value=now):
                if event.kind == "gps":
                    motorapp.handle_gps(event.payload)
                elif event.kind == "imu":
                    motorapp.handle_imu(event.payload)
                elif event.kind == "baro":
                    motorapp.handle_barometer(event.payload)
                elif event.kind == "state":
                    motorapp.handle_flight_state(event.payload)
                elif event.kind == "cycle":
                    captured.clear()
                    ctrl_out = motorapp._ctrl_cycle(None, now)
                    if ctrl_out is None:
                        continue
                    captured.setdefault("ctrl_out", ctrl_out)
                    rows.append(_row_from_cycle(now, ctrl_out, captured))
    return rows


def write_output(rows: list[dict[str, object]], path: Path) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=OUTPUT_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)


def summarize(rows: list[dict[str, object]]) -> dict[str, object]:
    total = len(rows)
    mode_counts = Counter(str(row["mode"]) for row in rows)
    fail_counts = Counter(
        str(row["fail_reason"]) for row in rows if str(row["mode"]) == "FAIL"
    )
    saturated = sum(1 for row in rows if _b(row["saturated"]))
    gyro_rejected = sum(1 for row in rows if _b(row["gyro_rejected"]))
    baro_spike = sum(1 for row in rows if _b(row["baro_sink_spike"]))
    state4_under50_valid = [
        row
        for row in rows
        if int(row["flight_state"]) == 4
        and math.isfinite(float(row["distance_to_target"]))
        and float(row["distance_to_target"]) <= 50.0
        and _b(row["l1_valid"])
    ]
    return {
        "total": total,
        "state4_under50_valid": len(state4_under50_valid),
        "mode_counts": mode_counts,
        "fail_counts": fail_counts,
        "saturation_ratio": saturated / total if total else 0.0,
        "gyro_rejected_ratio": gyro_rejected / total if total else 0.0,
        "baro_sink_spike_ratio": baro_spike / total if total else 0.0,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw-motor", type=Path, default=DEFAULT_RAW_MOTOR)
    parser.add_argument("--log-dir", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--target-lat", type=float, default=None)
    parser.add_argument("--target-lon", type=float, default=None)
    parser.add_argument("--target-offset-n-m", type=float, default=DEFAULT_TARGET_OFFSET_N_M)
    parser.add_argument("--target-offset-e-m", type=float, default=DEFAULT_TARGET_OFFSET_E_M)
    args = parser.parse_args(argv)

    events = load_events(None if args.log_dir else args.raw_motor, args.log_dir)
    rows = replay_events(
        events,
        target_lat=args.target_lat,
        target_lon=args.target_lon,
        target_offset_n_m=args.target_offset_n_m,
        target_offset_e_m=args.target_offset_e_m,
    )
    write_output(rows, args.output)
    summary = summarize(rows)
    print(f"cycles: {summary['total']}")
    print(f"state4_under50_control_valid: {summary['state4_under50_valid']}")
    print(f"saturation_ratio: {summary['saturation_ratio']:.3f}")
    print(f"gyro_rejected_ratio: {summary['gyro_rejected_ratio']:.3f}")
    print(f"baro_sink_spike_ratio: {summary['baro_sink_spike_ratio']:.3f}")
    print("mode_counts:")
    for mode, count in sorted(summary["mode_counts"].items()):
        print(f"  {mode}: {count}")
    print("fail_reasons:")
    for reason, count in sorted(summary["fail_counts"].items()):
        print(f"  {reason}: {count}")
    print(f"csv: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
