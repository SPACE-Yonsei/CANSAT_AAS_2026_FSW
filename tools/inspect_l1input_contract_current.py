from __future__ import annotations

import csv
import importlib
import math
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from Sensor_Motor import control, guidance  # noqa: E402
from Sensor_Motor.sensor_types import _BaroFromApp, _GpsFromApp, _ImuFromApp  # noqa: E402
from lib import config  # noqa: E402


NOW = 1000.0
ORIGIN_LAT = 37.0
ORIGIN_LON = 127.0
TARGET_E_M = 80.0
TARGET_N_M = 20.0
COURSE_EAST = math.radians(90.0)
COURSE_NORTH = 0.0
ANCHOR_TIME = NOW - config.GPS_FRESH_MAX_AGE_S - 1.0

OUTPUT_CSV = ROOT / "l1input_contract_current.csv"

CSV_COLUMNS = [
    "case",
    "mode",
    "nav_valid",
    "nav_E",
    "nav_N",
    "nav_V",
    "nav_course_deg",
    "nav_confidence",
    "l1_valid",
    "l1_reason",
    "l1_E",
    "l1_N",
    "l1_V",
    "l1_course_deg",
    "l1_vE",
    "l1_vN",
    "l1_target_E",
    "l1_target_N",
    "l1_confidence",
    "l1_dr_method",
    "l1out_valid",
    "l1out_reason",
    "distance_to_target",
    "nu_deg",
    "target_bearing_deg",
    "yaw_rate_cmd_dps",
    "yaw_rate_limit_dps",
    "ctrl_valid",
    "delta_arm_deg",
]


def reload_modules():
    global guidance, control
    guidance = importlib.reload(guidance)
    control = importlib.reload(control)
    control.reset()
    return guidance, control


def latlon_from_ne(n_m: float, e_m: float) -> tuple[float, float]:
    lat = ORIGIN_LAT + math.degrees(n_m / guidance.EARTH_RADIUS_M)
    lon = ORIGIN_LON + math.degrees(
        e_m / (guidance.EARTH_RADIUS_M * math.cos(math.radians(ORIGIN_LAT)))
    )
    return lat, lon


def configure_origin() -> None:
    guidance.set_origin_point(ORIGIN_LAT, ORIGIN_LON)


def configure_target() -> None:
    target_lat, target_lon = latlon_from_ne(TARGET_N_M, TARGET_E_M)
    guidance.set_target_point(target_lat, target_lon)


def configure_mission(*, origin: bool = True, target: bool = True) -> None:
    guidance.reset()
    if origin:
        configure_origin()
    if target:
        configure_target()


def gps_at(
    now: float,
    *,
    n_m: float = 0.0,
    e_m: float = 0.0,
    pos: bool = True,
    motion: bool = True,
    course: float = COURSE_EAST,
    speed: float = 4.0,
) -> _GpsFromApp:
    lat, lon = latlon_from_ne(n_m, e_m)
    return _GpsFromApp(
        lat=lat,
        lon=lon,
        course_rad=course if motion else None,
        speed_mps=speed if motion else None,
        pos_ts=now if pos else None,
        motion_ts=now if motion else None,
        pos_health=1 if pos else 0,
        motion_health=1 if motion else 0,
    )


def imu_at(
    now: float,
    *,
    gyrz_dps: float | None = 5.0,
    yaw: float | None = None,
    acc: bool = False,
    health: int = 1,
) -> _ImuFromApp:
    return _ImuFromApp(
        yaw_rad=yaw,
        gyrz_rad_s=math.radians(gyrz_dps) if gyrz_dps is not None else None,
        ts=now,
        lin_acc_x=0.2 if acc else None,
        lin_acc_y=0.0 if acc else None,
        lin_acc_valid=acc,
        health=health,
    )


def baro_at(now: float, *, sink: float | None = 2.0, health: int = 1) -> _BaroFromApp:
    return _BaroFromApp(
        alt_m=120.0,
        sink_rate=sink,
        rx_ts=now,
        health=health if sink is not None else 0,
    )


def lock_dr_anchor_with_gps() -> None:
    mode = guidance.DecideControlMode(
        gps_at(
            ANCHOR_TIME,
            n_m=0.0,
            e_m=0.0,
            pos=True,
            motion=True,
            course=COURSE_EAST,
            speed=4.0,
        ),
        imu_at(ANCHOR_TIME, gyrz_dps=0.0, yaw=COURSE_EAST),
        baro_at(ANCHOR_TIME, sink=2.0),
        ANCHOR_TIME,
    )
    if mode not in (
        guidance.ControlMode.GPS_TRACKING_CLOSED,
        guidance.ControlMode.GPS_TRACKING_OPEN,
    ):
        raise AssertionError(f"anchor lock failed: {mode}")


def case_sensors(case_name: str) -> tuple[Any, Any, Any]:
    if case_name == "A_NO_ORIGIN":
        configure_mission(origin=False, target=True)
        return (
            gps_at(NOW, pos=True, motion=True, course=COURSE_EAST, speed=4.0),
            imu_at(NOW, gyrz_dps=5.0),
            baro_at(NOW),
        )

    if case_name == "B_NO_TARGET":
        configure_mission(origin=True, target=False)
        return (
            gps_at(NOW, pos=True, motion=True, course=COURSE_EAST, speed=4.0),
            imu_at(NOW, gyrz_dps=5.0),
            baro_at(NOW),
        )

    if case_name == "C_GPS_TRACKING_CLOSED":
        configure_mission()
        return (
            gps_at(NOW, n_m=2.0, e_m=3.0, pos=True, motion=True, course=COURSE_EAST, speed=4.0),
            imu_at(NOW, gyrz_dps=5.0),
            baro_at(NOW),
        )

    if case_name == "D_GPS_TRACKING_OPEN":
        configure_mission()
        return (
            gps_at(NOW, n_m=2.0, e_m=3.0, pos=True, motion=True, course=COURSE_NORTH, speed=4.0),
            imu_at(NOW, gyrz_dps=None, yaw=COURSE_NORTH),
            baro_at(NOW),
        )

    if case_name == "E_DR_M_GBA":
        configure_mission()
        lock_dr_anchor_with_gps()
        return (
            gps_at(NOW, n_m=6.0, e_m=10.0, pos=True, motion=False),
            imu_at(NOW, gyrz_dps=10.0, yaw=COURSE_EAST, acc=True),
            baro_at(NOW, sink=2.0),
        )

    if case_name == "F_DR_PM_GBA":
        configure_mission()
        lock_dr_anchor_with_gps()
        return (
            None,
            imu_at(NOW, gyrz_dps=10.0, yaw=COURSE_EAST, acc=True),
            baro_at(NOW, sink=2.0),
        )

    if case_name == "G_NAV_INVALID":
        configure_mission()
        return (
            gps_at(
                NOW,
                n_m=2.0,
                e_m=3.0,
                pos=True,
                motion=True,
                course=COURSE_EAST,
                speed=config.V_MIN_MPS * 0.5,
            ),
            imu_at(NOW, gyrz_dps=5.0),
            baro_at(NOW),
        )

    if case_name == "H_L1INPUT_TO_L1OUTPUT":
        configure_mission()
        return (
            gps_at(NOW, n_m=2.0, e_m=3.0, pos=True, motion=True, course=COURSE_EAST, speed=4.0),
            imu_at(NOW, gyrz_dps=5.0),
            baro_at(NOW),
        )

    if case_name == "I_L1OUTPUT_TO_CONTROL":
        configure_mission()
        return (
            gps_at(NOW, n_m=2.0, e_m=3.0, pos=True, motion=True, course=COURSE_NORTH, speed=4.0),
            imu_at(NOW, gyrz_dps=5.0),
            baro_at(NOW),
        )

    raise KeyError(case_name)


def finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def deg(value: Any) -> float:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return float("nan")
    return math.degrees(v) if math.isfinite(v) else float("nan")


def enum_value(value: Any) -> str:
    return str(getattr(value, "value", value))


def run_contract_case(case_name: str) -> dict[str, Any]:
    reload_modules()
    gps, imu, baro = case_sensors(case_name)

    decide_calls = 0
    mode = guidance.DecideControlMode(gps, imu, baro, NOW)
    decide_calls += 1

    nav = guidance._STATE_t.nav
    dr = guidance._STATE_t.dr
    l1_in = guidance.ProduceL1Input(NOW)
    l1_out = guidance.ProduceL1Output(l1_in)

    ctrl_in = None
    if l1_out.valid:
        ctrl_in = control.ProduceCtrlInput(l1_out, NOW)
        gyrz = guidance._STATE_t.imu.gyr_z
        gyrz_meas_dps = math.degrees(gyrz) if finite(gyrz) else float("nan")
        ctrl_out = control.ProduceCtrlOutput(ctrl_in, gyrz_meas_dps, NOW)
    else:
        ctrl_out = control.WriteNeutral(NOW, mode)

    row = {
        "case": case_name,
        "mode": enum_value(mode),
        "nav_valid": nav.valid,
        "nav_E": nav.E,
        "nav_N": nav.N,
        "nav_V": nav.V,
        "nav_course_deg": deg(nav.course),
        "nav_confidence": nav.confidence,
        "l1_valid": l1_in.valid,
        "l1_reason": l1_in.reason,
        "l1_E": l1_in.E,
        "l1_N": l1_in.N,
        "l1_V": l1_in.V,
        "l1_course_deg": deg(l1_in.course),
        "l1_vE": l1_in.vE,
        "l1_vN": l1_in.vN,
        "l1_target_E": l1_in.target_E,
        "l1_target_N": l1_in.target_N,
        "l1_confidence": l1_in.confidence,
        "l1_dr_method": enum_value(l1_in.dr_method),
        "l1out_valid": l1_out.valid,
        "l1out_reason": l1_out.reason,
        "distance_to_target": l1_out.distance_to_target,
        "nu_deg": deg(l1_out.nu),
        "target_bearing_deg": deg(l1_out.target_bearing),
        "yaw_rate_cmd_dps": math.degrees(l1_out.yaw_rate_cmd),
        "yaw_rate_limit_dps": l1_out.yaw_rate_limit_dps,
        "ctrl_valid": ctrl_out.valid,
        "delta_arm_deg": ctrl_out.delta_arm_deg,
    }

    return {
        "case": case_name,
        "mode": mode,
        "nav": nav,
        "dr": dr,
        "l1_in": l1_in,
        "l1_out": l1_out,
        "ctrl_in": ctrl_in,
        "ctrl_out": ctrl_out,
        "row": row,
        "decide_calls": decide_calls,
        "guidance": guidance,
        "control": control,
    }


def run_all_cases() -> list[dict[str, Any]]:
    return [run_contract_case(case_name) for case_name in CASE_NAMES]


def write_csv(results: list[dict[str, Any]], path: Path = OUTPUT_CSV) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS, lineterminator="\n")
        writer.writeheader()
        for result in results:
            writer.writerow(result["row"])


CASE_NAMES = [
    "A_NO_ORIGIN",
    "B_NO_TARGET",
    "C_GPS_TRACKING_CLOSED",
    "D_GPS_TRACKING_OPEN",
    "E_DR_M_GBA",
    "F_DR_PM_GBA",
    "G_NAV_INVALID",
    "H_L1INPUT_TO_L1OUTPUT",
    "I_L1OUTPUT_TO_CONTROL",
]


def main() -> int:
    results = run_all_cases()
    write_csv(results)
    for result in results:
        row = result["row"]
        print(
            f"{row['case']}: mode={row['mode']} "
            f"l1_valid={row['l1_valid']} l1_reason={row['l1_reason']} "
            f"l1out_valid={row['l1out_valid']} ctrl_valid={row['ctrl_valid']}"
        )
    print(f"csv: {OUTPUT_CSV}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
