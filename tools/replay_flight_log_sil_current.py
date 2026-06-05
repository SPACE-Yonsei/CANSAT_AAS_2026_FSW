from __future__ import annotations

import argparse
import csv
import importlib
import math
import sys
from collections import Counter
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from Sensor_Motor import control, guidance, motorapp  # noqa: E402
from lib import config  # noqa: E402


DEFAULT_RAW_MOTOR = ROOT / "motorlogs" / "motor_control_20260604_161105.csv"
DEFAULT_OUT = ROOT / "flight_log_replay_sil_current.csv"
DEFAULT_TARGET_OFFSET_N_M = 30.0
DEFAULT_TARGET_OFFSET_E_M = 0.0

ALLOWED_FAIL_REASONS = {
    "",
    "FAIL",
    "NO_ORIGIN",
    "NO_TARGET",
    "NO_COURSE_SOURCE",
    "NO_GUIDANCE_SOURCE",
    "NO_SPEED_SOURCE",
    "DR_TIMEOUT",
    "DR_POSITION_JUMP",
    "GPS_POS_NAN",
    "NAV_INVALID",
    "V_TOO_SMALL",
    "LOW_DR_CONFIDENCE",
    "NAN_NAV_STATE",
}

OUTPUT_COLUMNS = [
    "time_s",
    "flight_state",
    "baro_alt_m",
    "mode",
    "fail_reason",
    "nav_valid",
    "l1_valid",
    "l1_reason",
    "l1out_valid",
    "l1out_reason",
    "yaw_rate_cmd_dps",
    "yaw_rate_limit_dps",
    "delta_arm_deg",
    "left_pw",
    "right_pw",
    "dr_age_s",
    "dr_confidence",
    "dr_current_E",
    "dr_current_N",
    "nu_deg",
    "distance_to_target",
    "saturated",
    "gyro_rejected",
    "baro_sink_spike",
    "speed_clamped",
    "dr_speed_source",
    "decide_call_count",
    "produce_l1input_call_count",
    "dr_pm_double_update",
    "exception",
]


@dataclass(frozen=True)
class Event:
    time_s: float
    kind: str
    payload: str = ""
    state: int | None = None
    motor_enabled: bool | None = None


class FakePi:
    def __init__(self):
        self.calls: list[tuple[int, int]] = []

    def set_servo_pulsewidth(self, pin, pulsewidth):
        self.calls.append((int(pin), int(pulsewidth)))


def _f(row: dict[str, str], key: str, default: float = math.nan) -> float:
    value = row.get(key, "")
    if value in ("", "nan", "None", None):
        return default
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return out if math.isfinite(out) else default


def _i(row: dict[str, str], key: str, default: int = 0) -> int:
    value = _f(row, key, float(default))
    return int(value) if math.isfinite(value) else default


def _b(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _mode_value(mode: Any) -> str:
    text = str(getattr(mode, "value", mode))
    return text.replace("ControlMode.", "")


def _csv_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8-sig") as f:
        return list(csv.DictReader(f))


def _parse_iso_seconds(text: str) -> float:
    try:
        return datetime.fromisoformat(str(text).strip()).timestamp()
    except (TypeError, ValueError):
        return math.nan


def _time_from_row(row: dict[str, str], *, prefer_wall: bool = False) -> float:
    if prefer_wall:
        t_wall = _parse_iso_seconds(row.get("timestamp", ""))
        if math.isfinite(t_wall):
            return t_wall
    t = _f(row, "monotonic_s")
    if math.isfinite(t):
        return t
    return _parse_iso_seconds(row.get("timestamp", ""))


def _origin_target_from_offset(
    lat: float, lon: float, north_m: float, east_m: float
) -> tuple[float, float]:
    target_lat = lat + math.degrees(north_m / guidance.EARTH_RADIUS_M)
    target_lon = lon + math.degrees(
        east_m / (guidance.EARTH_RADIUS_M * math.cos(math.radians(lat)))
    )
    return target_lat, target_lon


def _reconstruct_raw_acc(row: dict[str, str]) -> tuple[float, float, float]:
    vals: list[float] = []
    for lin_key, gravity_key in (
        ("lin_acc_x_mps2", "gravity_body_x_mps2"),
        ("lin_acc_y_mps2", "gravity_body_y_mps2"),
        ("lin_acc_z_mps2", "gravity_body_z_mps2"),
    ):
        lin = _f(row, lin_key)
        gravity = _f(row, gravity_key)
        vals.append(lin + gravity if math.isfinite(lin) and math.isfinite(gravity) else math.nan)
    if all(math.isfinite(v) for v in vals):
        return vals[0], vals[1], vals[2]
    return 0.0, 0.0, 9.81


def _embedded_motor_events(path: Path) -> list[Event]:
    events: list[Event] = []
    for row in _csv_rows(path):
        t = _time_from_row(row)
        if not math.isfinite(t):
            continue
        state = _i(row, "flight_state", 4)
        motor_enabled = _b(row.get("motor_enabled", "1"))

        gps_lat = _f(row, "gps_lat")
        gps_lon = _f(row, "gps_lon")
        if math.isfinite(gps_lat) and math.isfinite(gps_lon):
            events.append(Event(
                t,
                "gps",
                f"{gps_lat},{gps_lon},{_i(row, 'gps_pos_health', 0)},{t},"
                f"{_f(row, 'gps_course_deg', 0.0)},{_f(row, 'gps_speed_mps', 0.0)},"
                f"{_i(row, 'gps_motion_health', 0)},{t}",
                state,
                motor_enabled,
            ))

        imu_health = _i(row, "imu_health", 0)
        if imu_health:
            accx, accy, accz = _reconstruct_raw_acc(row)
            nav_gyrz = _f(row, "imu_gyrz_deg_s", 0.0)
            raw_gyrz = -nav_gyrz if math.isfinite(nav_gyrz) else math.nan
            events.append(Event(
                t,
                "imu",
                f"{_f(row, 'imu_roll_deg', 0.0)},{_f(row, 'imu_pitch_deg', 0.0)},"
                f"{_f(row, 'imu_yaw_deg', 0.0)},{accx},{accy},{accz},0.0,0.0,"
                f"{raw_gyrz},{imu_health},{t},0.0",
                state,
                motor_enabled,
            ))

        baro_alt = _f(row, "baro_alt_m")
        baro_sink = _f(row, "baro_sink_rate_mps")
        baro_health = _i(row, "baro_health", 0)
        if baro_health or math.isfinite(baro_alt):
            events.append(Event(
                t,
                "baro",
                f"{baro_alt},{baro_sink},{baro_health}",
                state,
                motor_enabled,
            ))

        events.append(Event(t, "cycle", state=state, motor_enabled=motor_enabled))
    return sorted(events, key=lambda e: (e.time_s, e.kind == "cycle"))


def _raw_sensor_events(log_dir: Path) -> list[Event]:
    events: list[Event] = []

    for row in _csv_rows(log_dir / "raw_gps.csv"):
        t = _time_from_row(row, prefer_wall=True)
        lat = _f(row, "lat")
        lon = _f(row, "lon")
        fix_quality = _i(row, "fix_quality", 0)
        rmc_status = str(row.get("rmc_status", "")).strip().upper()
        motion_valid = _b(row.get("motion_valid", "0")) or rmc_status == "A"
        pos_health = 1 if fix_quality > 0 and math.isfinite(lat) and math.isfinite(lon) else 0
        motion_health = 1 if motion_valid else 0
        if math.isfinite(t):
            events.append(Event(
                t,
                "gps",
                f"{lat},{lon},{pos_health},{t},{_f(row, 'course_deg', 0.0)},"
                f"{_f(row, 'speed_ms', 0.0)},{motion_health},{t}",
            ))

    for row in _csv_rows(log_dir / "raw_imu.csv"):
        t = _time_from_row(row, prefer_wall=True)
        vals = [_f(row, key) for key in (
            "roll_deg", "pitch_deg", "yaw_deg", "acc_x", "acc_y", "acc_z",
            "gyr_x", "gyr_y", "gyr_z",
        )]
        health = 1 if math.isfinite(t) and all(math.isfinite(v) for v in vals) else 0
        if math.isfinite(t):
            events.append(Event(t, "imu", ",".join(str(v) for v in vals) + f",{health},{t},0.0"))

    prev_alt: float | None = None
    prev_t: float | None = None
    for row in _csv_rows(log_dir / "raw_barometer.csv"):
        t = _time_from_row(row, prefer_wall=True)
        alt = _f(row, "altitude_raw_m")
        sink = math.nan
        if prev_alt is not None and prev_t is not None and math.isfinite(t) and t > prev_t:
            sink = -(alt - prev_alt) / (t - prev_t)
        if math.isfinite(t) and math.isfinite(alt):
            events.append(Event(t, "baro", f"{alt},{sink if math.isfinite(sink) else 0.0},1"))
            prev_alt, prev_t = alt, t

    raw_motor = log_dir / "raw_motor.csv"
    if not raw_motor.exists():
        raw_motor = log_dir / "motor_control.csv"
    for row in _csv_rows(raw_motor):
        t = _time_from_row(row, prefer_wall=True)
        if math.isfinite(t):
            events.append(Event(
                t,
                "cycle",
                state=_i(row, "flight_state", 4),
                motor_enabled=_b(row.get("motor_enabled", "1")),
            ))

    if events:
        t0 = min(e.time_s for e in events if math.isfinite(e.time_s))
        events = [Event(e.time_s - t0, e.kind, e.payload, e.state, e.motor_enabled) for e in events]
    return sorted(events, key=lambda e: (e.time_s, e.kind == "cycle"))


def load_events(raw_motor: Path | None = DEFAULT_RAW_MOTOR, log_dir: Path | None = None) -> list[Event]:
    if log_dir is not None:
        events = _raw_sensor_events(log_dir)
        if events:
            return events
    if raw_motor is None:
        raw_motor = DEFAULT_RAW_MOTOR
    return _embedded_motor_events(raw_motor)


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
    motorapp.MOTOR_CTRL_MODE = config.MOTOR_CTRL_MODE_GPS_GUIDED
    motorapp._STEER_MODE = ""
    return fake_pi


def _finite_or_nan(value: Any) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return math.nan
    return out if math.isfinite(out) else math.nan


def _row_from_cycle(now: float, ctrl_out: Any, captured: dict[str, Any]) -> dict[str, Any]:
    st = guidance._STATE_t
    nav = st.nav
    dr = st.dr
    l1_in = captured.get("l1_in")
    l1_out = captured.get("l1_out")
    snap = captured.get("snap")
    baro_alt = getattr(getattr(snap, "latest_baro", SimpleNamespace()), "alt_m", math.nan)
    dr_age = now - dr.anchor_time if math.isfinite(dr.anchor_time) else math.nan
    yaw_cmd = math.degrees(_finite_or_nan(getattr(l1_out, "yaw_rate_cmd", 0.0))) if l1_out else 0.0
    return {
        "time_s": now,
        "flight_state": motorapp.STATE,
        "baro_alt_m": _finite_or_nan(baro_alt),
        "mode": _mode_value(nav.control_mode),
        "fail_reason": nav.fail_reason,
        "nav_valid": bool(nav.valid),
        "l1_valid": bool(getattr(l1_in, "valid", False)),
        "l1_reason": str(getattr(l1_in, "reason", "")),
        "l1out_valid": bool(getattr(l1_out, "valid", False)),
        "l1out_reason": str(getattr(l1_out, "reason", "")),
        "yaw_rate_cmd_dps": yaw_cmd,
        "yaw_rate_limit_dps": _finite_or_nan(getattr(l1_out, "yaw_rate_limit_dps", 0.0)) if l1_out else 0.0,
        "delta_arm_deg": _finite_or_nan(getattr(ctrl_out, "delta_arm_deg", 0.0)),
        "left_pw": int(getattr(ctrl_out, "left_pw", control.LEFT_NEUTRAL)),
        "right_pw": int(getattr(ctrl_out, "right_pw", control.RIGHT_NEUTRAL)),
        "dr_age_s": dr_age,
        "dr_confidence": _finite_or_nan(dr.confidence),
        "dr_current_E": _finite_or_nan(dr.current_E),
        "dr_current_N": _finite_or_nan(dr.current_N),
        "nu_deg": math.degrees(_finite_or_nan(getattr(l1_out, "nu", math.nan))) if l1_out else math.nan,
        "distance_to_target": _finite_or_nan(getattr(l1_out, "distance_to_target", math.nan)) if l1_out else math.nan,
        "saturated": bool(getattr(ctrl_out, "saturated", False)),
        "gyro_rejected": bool(getattr(ctrl_out, "gyro_rejected", False)),
        "baro_sink_spike": bool(nav.baro_sink_spike),
        "speed_clamped": bool(nav.speed_clamped),
        "dr_speed_source": nav.dr_speed_source,
        "decide_call_count": captured.get("decide_call_count", 0),
        "produce_l1input_call_count": captured.get("produce_l1input_call_count", 0),
        "dr_pm_double_update": bool(captured.get("dr_pm_double_update", False)),
        "exception": "",
    }


def replay_events(
    events: list[Event],
    *,
    target_lat: float | None = None,
    target_lon: float | None = None,
    target_offset_n_m: float = DEFAULT_TARGET_OFFSET_N_M,
    target_offset_e_m: float = DEFAULT_TARGET_OFFSET_E_M,
) -> list[dict[str, Any]]:
    reset_for_replay(
        events,
        target_lat=target_lat,
        target_lon=target_lon,
        target_offset_n_m=target_offset_n_m,
        target_offset_e_m=target_offset_e_m,
    )
    rows: list[dict[str, Any]] = []
    captured: dict[str, Any] = {}

    orig_decide = motorapp.guidance.DecideControlMode
    orig_l1input = motorapp.guidance.ProduceL1Input

    def capture_log(state, motor_enabled, motor_ctrl_mode, ctrl_out, l1_out=None, **kwargs):
        captured["ctrl_out"] = ctrl_out
        captured["l1_out"] = l1_out
        captured["l1_in"] = kwargs.get("l1_in")
        captured["snap"] = kwargs.get("snap")

    def capture_decide(gps=None, imu=None, baro=None, now=None):
        captured["decide_call_count"] = captured.get("decide_call_count", 0) + 1
        captured["dr_before_decide"] = (
            guidance._STATE_t.dr.current_E,
            guidance._STATE_t.dr.current_N,
        )
        mode = orig_decide(gps, imu, baro, now)
        captured["dr_after_decide"] = (
            guidance._STATE_t.dr.current_E,
            guidance._STATE_t.dr.current_N,
        )
        return mode

    def capture_l1input(now):
        captured["produce_l1input_call_count"] = captured.get("produce_l1input_call_count", 0) + 1
        before = (guidance._STATE_t.dr.current_E, guidance._STATE_t.dr.current_N)
        l1_in = orig_l1input(now)
        after = (guidance._STATE_t.dr.current_E, guidance._STATE_t.dr.current_N)
        captured["l1_in"] = l1_in
        captured["dr_after_l1input"] = after
        mode_value = _mode_value(guidance._STATE_t.nav.control_mode)
        if mode_value.startswith("DR_PM"):
            captured["dr_pm_double_update"] = before != after
        return l1_in

    with (
        mock.patch.object(motorapp.sensorlog, "log_motor_raw", side_effect=capture_log),
        mock.patch.object(motorapp.prevstate, "update_start_point", return_value=None),
        mock.patch.object(motorapp.prevstate, "update_motor_enabled", return_value=None),
        mock.patch.object(motorapp, "_publish_motor_diag", return_value=None),
        mock.patch.object(motorapp.guidance, "DecideControlMode", side_effect=capture_decide),
        mock.patch.object(motorapp.guidance, "ProduceL1Input", side_effect=capture_l1input),
    ):
        for event in events:
            now = event.time_s
            if event.state is not None:
                motorapp.STATE = int(event.state)
            if event.motor_enabled is not None:
                motorapp.MOTOR_ENABLED = bool(event.motor_enabled)
            with mock.patch.object(motorapp.time, "monotonic", return_value=now):
                try:
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
                        if ctrl_out is not None:
                            captured.setdefault("ctrl_out", ctrl_out)
                            rows.append(_row_from_cycle(now, ctrl_out, captured))
                except Exception as exc:
                    rows.append({
                        **{key: "" for key in OUTPUT_COLUMNS},
                        "time_s": now,
                        "flight_state": motorapp.STATE,
                        "exception": repr(exc),
                    })
    return rows


def write_output(rows: list[dict[str, Any]], path: Path = DEFAULT_OUT) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=OUTPUT_COLUMNS, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in OUTPUT_COLUMNS})


def summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
    total = len(rows)
    exceptions = [row for row in rows if str(row.get("exception", ""))]
    saturation_count = sum(1 for row in rows if _b(row.get("saturated", False)))
    state4_under50_valid = [
        row for row in rows
        if int(row.get("flight_state", 0) or 0) == 4
        and math.isfinite(_finite_or_nan(row.get("distance_to_target")))
        and _finite_or_nan(row.get("distance_to_target")) <= 50.0
        and _b(row.get("l1out_valid", False))
    ]
    return {
        "total": total,
        "exceptions": len(exceptions),
        "state4_under50_valid": len(state4_under50_valid),
        "saturation_ratio": saturation_count / total if total else 0.0,
        "mode_counts": Counter(str(row.get("mode", "")) for row in rows),
        "fail_counts": Counter(
            str(row.get("fail_reason", ""))
            for row in rows if str(row.get("mode", "")) == "FAIL"
        ),
        "double_update_count": sum(1 for row in rows if _b(row.get("dr_pm_double_update", False))),
    }


def assert_replay_contract(rows: list[dict[str, Any]]) -> None:
    assert rows
    assert not [row for row in rows if str(row.get("exception", ""))]
    assert any(
        int(row["flight_state"]) == 4
        and math.isfinite(_finite_or_nan(row["distance_to_target"]))
        and _finite_or_nan(row["distance_to_target"]) <= 50.0
        and _b(row["l1out_valid"])
        for row in rows
    )
    for row in rows:
        if int(row["flight_state"]) >= 4:
            assert int(row["decide_call_count"]) == 1
            assert int(row["produce_l1input_call_count"]) == 1
        limit = _finite_or_nan(row["yaw_rate_limit_dps"])
        cmd = _finite_or_nan(row["yaw_rate_cmd_dps"])
        if math.isfinite(limit) and limit > 0.0:
            assert abs(cmd) <= limit + 1.0e-9
        assert abs(_finite_or_nan(row["delta_arm_deg"])) <= control.DELTA_ARM_MAX_DEG + 1.0e-9
        if str(row["mode"]) == "FAIL":
            assert str(row["fail_reason"]) in ALLOWED_FAIL_REASONS
        assert not _b(row["dr_pm_double_update"])
    saturation_ratio = (
        sum(1 for row in rows if _b(row["saturated"])) / len(rows)
        if rows else 0.0
    )
    assert saturation_ratio < 0.30


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

    events = load_events(raw_motor=None if args.log_dir else args.raw_motor, log_dir=args.log_dir)
    rows = replay_events(
        events,
        target_lat=args.target_lat,
        target_lon=args.target_lon,
        target_offset_n_m=args.target_offset_n_m,
        target_offset_e_m=args.target_offset_e_m,
    )
    write_output(rows, args.output)
    summary = summarize(rows)
    print(f"events: {len(events)}")
    print(f"cycles: {summary['total']}")
    print(f"exceptions: {summary['exceptions']}")
    print(f"state4_under50_control_valid: {summary['state4_under50_valid']}")
    print(f"saturation_ratio: {summary['saturation_ratio']:.3f}")
    print(f"dr_pm_double_update_count: {summary['double_update_count']}")
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
