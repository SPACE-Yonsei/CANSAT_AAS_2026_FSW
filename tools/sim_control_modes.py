from __future__ import annotations

import csv
import importlib
import math
import sys
from collections import Counter
from itertools import product
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from lib import config
from Sensor_Motor import control, guidance

try:
    from Sensor_Motor import motorapp
except Exception:  # pragma: no cover - optional integration import
    motorapp = None


NOW = 1000.0
ORIGIN_LAT = 37.0
ORIGIN_LON = 127.0
TARGET_E_M = 100.0
GPS_E_M = 20.0
GPS_N_M = 0.0
BASE_COURSE_RAD = math.radians(90.0)
BASE_SPEED_MPS = 5.0

TUNED_GAINS = {
    "KP_GPS_CLOSED": 0.53,
    "KI_GPS_CLOSED": 1.06,
    "KD_GPS_CLOSED": 0.0,
    "KP_DR_M_CLOSED": 0.40,
    "KI_DR_M_CLOSED": 0.79,
    "KD_DR_M_CLOSED": 0.0,
    "KP_DR_PM_CLOSED": 0.30,
    "KI_DR_PM_CLOSED": 0.60,
    "KD_DR_PM_CLOSED": 0.0,
}

CASE_KEYS = (
    "origin_ready",
    "target_ready",
    "gps_pos",
    "gps_motion",
    "imu_gyrz",
    "imu_yaw",
    "baro_sink",
    "acc",
    "dr_ready",
    "dr_old",
    "gyro_spike",
    "baro_spike",
)


def _latlon_from_ne(north_m: float, east_m: float) -> tuple[float, float]:
    lat = ORIGIN_LAT + math.degrees(north_m / guidance.EARTH_RADIUS_M)
    lon = ORIGIN_LON + math.degrees(
        east_m / (guidance.EARTH_RADIUS_M * math.cos(math.radians(ORIGIN_LAT)))
    )
    return lat, lon


def apply_tuned_config(*, overwrite_existing: bool = True) -> dict[str, float]:
    applied = {}
    for name, value in TUNED_GAINS.items():
        if overwrite_existing or not hasattr(config, name):
            setattr(config, name, value)
        applied[name] = float(getattr(config, name))
    return applied


def reload_under_test(*, reload_motorapp: bool = False) -> None:
    global control, guidance, motorapp
    guidance = importlib.reload(guidance)
    control = importlib.reload(control)
    control.reset()
    if reload_motorapp and motorapp is not None:
        with patch.object(control, "init_control", return_value=None):
            motorapp = importlib.reload(motorapp)
    for name, expected in TUNED_GAINS.items():
        actual = float(getattr(config, name))
        assert math.isclose(actual, expected, rel_tol=0.0, abs_tol=1.0e-12), (
            f"{name}={actual} was not applied before reload"
        )


def make_gps(pos: bool, motion: bool, now: float = NOW) -> SimpleNamespace:
    lat, lon = _latlon_from_ne(GPS_N_M, GPS_E_M)
    return SimpleNamespace(
        lat=lat,
        lon=lon,
        pos_ts=now if pos else now - 999.0,
        pos_health=1 if pos else 0,
        course_rad=BASE_COURSE_RAD,
        speed_mps=BASE_SPEED_MPS,
        motion_ts=now if motion else now - 999.0,
        motion_health=1 if motion else 0,
    )


def make_imu(
    yaw: bool,
    gyrz: bool,
    acc: bool,
    gyro_spike: bool,
    now: float = NOW,
) -> SimpleNamespace:
    limit = float(getattr(config, "DR_MAX_YAW_RATE_DPS_FOR_CONTROL", 120.0))
    gyrz_dps = limit + 20.0 if gyro_spike else 5.0
    return SimpleNamespace(
        health=1 if (yaw or gyrz or acc) else 0,
        ts=now,
        yaw_rad=BASE_COURSE_RAD if yaw else None,
        gyrz_rad_s=math.radians(gyrz_dps) if gyrz else None,
        lin_acc_x=0.05 if acc else None,
        lin_acc_y=0.0 if acc else None,
        lin_acc_valid=bool(acc),
    )


def make_baro(
    baro: bool,
    baro_spike: bool,
    now: float = NOW,
) -> SimpleNamespace:
    max_sink = float(getattr(config, "DR_BARO_SINK_MAX_MPS", 6.0))
    sink = max_sink + 2.0 if baro_spike else 3.0
    return SimpleNamespace(
        health=1 if baro else 0,
        rx_ts=now,
        alt_m=120.0,
        sink_rate=sink if baro else None,
    )


def reset_world(
    origin_ready: bool,
    target_ready: bool,
    dr_ready: bool,
    dr_old: bool,
    now: float = NOW,
) -> None:
    reload_under_test()
    if origin_ready:
        guidance.set_origin_point(ORIGIN_LAT, ORIGIN_LON)
    if target_ready:
        target_lat, target_lon = _latlon_from_ne(0.0, TARGET_E_M)
        guidance.set_target_point(target_lat, target_lon)
    if dr_ready:
        anchor_time = now - config.DR_MAX_AGE_S - 5.0 if dr_old else now - 2.0
        guidance.dr_lock(
            guidance._STATE_t.dr,
            E=GPS_E_M,
            N=GPS_N_M,
            V=BASE_SPEED_MPS,
            course=BASE_COURSE_RAD,
            yaw=BASE_COURSE_RAD,
            now=anchor_time,
        )
        guidance._STATE_t.dr.current_time = now - 0.1
        guidance._STATE_t.dr.last_step_time = now - 0.1


def generate_cases() -> list[dict[str, bool]]:
    cases = []
    for values in product((False, True), repeat=len(CASE_KEYS)):
        case = dict(zip(CASE_KEYS, values))
        if case["dr_old"] and not case["dr_ready"]:
            continue
        if case["gyro_spike"] and not case["imu_gyrz"]:
            continue
        if case["baro_spike"] and not case["baro_sink"]:
            continue
        cases.append(case)
    return cases


def _mode_name(mode) -> str:
    return getattr(mode, "value", str(mode))


def _finite_or_nan(value) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return float("nan")
    return out if math.isfinite(out) else float("nan")


def run_one_case(case: dict[str, bool], now: float = NOW) -> dict[str, object]:
    reset_world(
        case["origin_ready"],
        case["target_ready"],
        case["dr_ready"],
        case["dr_old"],
        now,
    )
    gps = make_gps(case["gps_pos"], case["gps_motion"], now)
    imu = make_imu(
        case["imu_yaw"],
        case["imu_gyrz"],
        case["acc"],
        case["gyro_spike"],
        now,
    )
    baro = make_baro(case["baro_sink"], case["baro_spike"], now)

    mode = guidance.DecideControlMode(gps, imu, baro, now)
    l1_in = guidance.ProduceL1Input(now)
    l1_out = guidance.ProduceL1Output(l1_in) if l1_in.valid else None

    ctrl_in = None
    if l1_out is not None and l1_out.valid:
        ctrl_in = control.ProduceCtrlInput(l1_out, now)
        gz_rad_s = _finite_or_nan(guidance._STATE_t.imu.gyr_z)
        gyro_meas_deg_s = math.degrees(gz_rad_s) if math.isfinite(gz_rad_s) else float("nan")
        ctrl_out = control.ProduceCtrlOutput(ctrl_in, gyro_meas_deg_s, now)
    else:
        ctrl_out = control.WriteNeutral(now, mode)
        if l1_in.valid and l1_out is not None:
            ctrl_out.reason = l1_out.reason
        else:
            ctrl_out.reason = l1_in.reason

    flags = guidance._STATE_t.flags
    nav = guidance._STATE_t.nav

    row = dict(case)
    row.update(
        mode=_mode_name(mode),
        fail_reason=nav.fail_reason,
        gps_pos_fresh=flags.gps_pos_fresh,
        gps_motion_fresh=flags.gps_motion_fresh,
        imu_gyrz_fresh=flags.imu_gyrz_fresh,
        imu_yaw_fresh=flags.imu_yaw_fresh,
        baro_sink_fresh=flags.baro_sink_fresh,
        acc_fresh=flags.acc_fresh,
        dr_anchor_valid=flags.dr_anchor_valid,
        dr_current_valid=flags.dr_current_valid,
        nav_valid=nav.valid,
        l1_in_valid=l1_in.valid,
        l1_in_reason=l1_in.reason,
        l1_out_valid=bool(l1_out and l1_out.valid),
        l1_out_reason=getattr(l1_out, "reason", ""),
        yaw_rate_cmd_dps=math.degrees(getattr(l1_out, "yaw_rate_cmd", 0.0) or 0.0)
        if l1_out is not None
        else 0.0,
        yaw_rate_limit_dps=getattr(l1_out, "yaw_rate_limit_dps", 0.0)
        if l1_out is not None
        else 0.0,
        ctrl_valid=ctrl_out.valid,
        ctrl_reason=ctrl_out.reason,
        delta_arm_deg=ctrl_out.delta_arm_deg,
        delta_ff_deg=ctrl_out.delta_ff_deg,
        delta_pid_deg=ctrl_out.delta_pid_deg,
        kp_used=ctrl_out.kp_used,
        ff_scale=ctrl_out.ff_scale,
        left_pw=ctrl_out.left_pw,
        right_pw=ctrl_out.right_pw,
        saturated=ctrl_out.saturated,
        gyro_rejected=ctrl_out.gyro_rejected,
        baro_sink_spike=nav.baro_sink_spike,
        speed_clamped=nav.speed_clamped,
        dr_speed_source=nav.dr_speed_source,
    )
    if ctrl_in is not None:
        row["pid_enabled"] = ctrl_in.pid_enabled
    else:
        row["pid_enabled"] = False
    return row


def run_all_cases(now: float = NOW) -> list[dict[str, object]]:
    return [run_one_case(case, now) for case in generate_cases()]


def write_csv(rows: list[dict[str, object]], path: Path) -> None:
    fieldnames = list(CASE_KEYS) + [
        "mode",
        "fail_reason",
        "gps_pos_fresh",
        "gps_motion_fresh",
        "imu_gyrz_fresh",
        "imu_yaw_fresh",
        "baro_sink_fresh",
        "acc_fresh",
        "dr_anchor_valid",
        "dr_current_valid",
        "nav_valid",
        "l1_in_valid",
        "l1_in_reason",
        "l1_out_valid",
        "l1_out_reason",
        "yaw_rate_cmd_dps",
        "yaw_rate_limit_dps",
        "ctrl_valid",
        "ctrl_reason",
        "delta_arm_deg",
        "delta_ff_deg",
        "delta_pid_deg",
        "kp_used",
        "ff_scale",
        "left_pw",
        "right_pw",
        "saturated",
        "gyro_rejected",
        "baro_sink_spike",
        "speed_clamped",
        "dr_speed_source",
        "pid_enabled",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def summarize(rows: list[dict[str, object]]) -> dict[str, Counter]:
    return {
        "modes": Counter(str(row["mode"]) for row in rows),
        "fail_reasons": Counter(
            str(row["fail_reason"]) for row in rows if row["mode"] == "FAIL"
        ),
    }


def main() -> int:
    apply_tuned_config()
    rows = run_all_cases(NOW)
    csv_path = ROOT / "control_mode_matrix.csv"
    write_csv(rows, csv_path)
    summary = summarize(rows)
    mode_counts = summary["modes"]
    missing = [
        mode.value for mode in guidance.ControlMode if mode_counts.get(mode.value, 0) == 0
    ]

    print(f"cases: {len(rows)}")
    print("mode counts:")
    for mode, count in sorted(mode_counts.items()):
        print(f"  {mode}: {count}")
    print("enum reachability:")
    for mode in guidance.ControlMode:
        print(f"  {mode.value}: {mode_counts.get(mode.value, 0)}")
    if missing:
        print("UNREACHABLE_MODES:")
        for mode in missing:
            print(f"  {mode}")
    else:
        print("UNREACHABLE_MODES: none")
    print("FAIL reasons:")
    for reason, count in sorted(summary["fail_reasons"].items()):
        print(f"  {reason}: {count}")
    print(f"csv: {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
