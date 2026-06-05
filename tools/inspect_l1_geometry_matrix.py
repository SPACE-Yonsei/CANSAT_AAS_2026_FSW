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
DEFAULT_DIST_M = 100.0
DEFAULT_SPEED_MPS = 5.0
CSV_PATH = ROOT / "l1_geometry_matrix_current.csv"

CSV_COLUMNS = [
    "case",
    "source",
    "mode",
    "valid_input",
    "l1_valid",
    "l1out_valid",
    "control_valid",
    "course_deg",
    "target_bearing_deg",
    "nu_deg",
    "confidence",
    "yaw_rate_cmd_dps",
    "yaw_rate_limit_dps",
    "distance_to_target",
    "nav_E",
    "nav_N",
    "l1_E",
    "l1_N",
    "l1_V",
    "out_V",
    "out_course_deg",
    "reason",
]


def reload_modules() -> None:
    global guidance, control
    guidance = importlib.reload(guidance)
    control = importlib.reload(control)
    control.reset()


def wrap_deg(angle_deg: float) -> float:
    return math.degrees(math.atan2(math.sin(math.radians(angle_deg)), math.cos(math.radians(angle_deg))))


def deg(rad: Any) -> float:
    try:
        value = float(rad)
    except (TypeError, ValueError):
        return float("nan")
    return math.degrees(value) if math.isfinite(value) else float("nan")


def finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def latlon_from_ne(n_m: float, e_m: float) -> tuple[float, float]:
    lat = ORIGIN_LAT + math.degrees(n_m / guidance.EARTH_RADIUS_M)
    lon = ORIGIN_LON + math.degrees(
        e_m / (guidance.EARTH_RADIUS_M * math.cos(math.radians(ORIGIN_LAT)))
    )
    return lat, lon


def target_from_bearing(bearing_deg: float, distance_m: float = DEFAULT_DIST_M) -> tuple[float, float]:
    bearing = math.radians(bearing_deg)
    return distance_m * math.sin(bearing), distance_m * math.cos(bearing)


def make_l1input(
    *,
    case: str,
    mode: guidance.ControlMode,
    course_deg: float,
    nu_deg: float,
    speed_mps: float = DEFAULT_SPEED_MPS,
    confidence: float = 1.0,
    valid: bool = True,
) -> guidance.L1Input:
    target_bearing_deg = course_deg + nu_deg
    target_e, target_n = target_from_bearing(target_bearing_deg)
    course = math.radians(course_deg)
    return guidance.L1Input(
        valid=valid,
        reason=case,
        control_mode=mode,
        dr_method=guidance.DRMethod.GYRO_INTEGRATION if mode.value.startswith("DR_") else guidance.DRMethod.NONE,
        confidence=confidence,
        E=0.0,
        N=0.0,
        V=speed_mps,
        course=course,
        vE=speed_mps * math.sin(course),
        vN=speed_mps * math.cos(course),
        target_E=target_e,
        target_N=target_n,
    )


def row_from_l1(case: str, source: str, l1_in: guidance.L1Input, l1_out: guidance.L1Output) -> dict[str, Any]:
    return {
        "case": case,
        "source": source,
        "mode": l1_in.control_mode.value,
        "valid_input": l1_in.valid,
        "l1_valid": l1_in.valid,
        "l1out_valid": l1_out.valid,
        "control_valid": l1_out.control_valid,
        "course_deg": deg(l1_in.course),
        "target_bearing_deg": deg(l1_out.target_bearing),
        "nu_deg": deg(l1_out.nu),
        "confidence": l1_in.confidence,
        "yaw_rate_cmd_dps": math.degrees(l1_out.yaw_rate_cmd),
        "yaw_rate_limit_dps": l1_out.yaw_rate_limit_dps,
        "distance_to_target": l1_out.distance_to_target,
        "nav_E": l1_out.nav_E,
        "nav_N": l1_out.nav_N,
        "l1_E": l1_in.E,
        "l1_N": l1_in.N,
        "l1_V": l1_in.V,
        "out_V": l1_out.V,
        "out_course_deg": deg(l1_out.course),
        "reason": l1_out.reason,
    }


def run_direct_case(
    case: str,
    *,
    mode: guidance.ControlMode = guidance.ControlMode.GPS_TRACKING_CLOSED,
    course_deg: float = 0.0,
    nu_deg: float = 30.0,
    speed_mps: float = DEFAULT_SPEED_MPS,
    confidence: float = 1.0,
    valid: bool = True,
) -> dict[str, Any]:
    reload_modules()
    l1_in = make_l1input(
        case=case,
        mode=mode,
        course_deg=course_deg,
        nu_deg=nu_deg,
        speed_mps=speed_mps,
        confidence=confidence,
        valid=valid,
    )
    l1_out = guidance.ProduceL1Output(l1_in)
    return {"case": case, "source": "direct", "l1_in": l1_in, "l1_out": l1_out, "row": row_from_l1(case, "direct", l1_in, l1_out)}


def gps_at(now: float, *, course_deg: float, speed_mps: float = DEFAULT_SPEED_MPS) -> _GpsFromApp:
    lat, lon = latlon_from_ne(0.0, 0.0)
    return _GpsFromApp(
        lat=lat,
        lon=lon,
        course_rad=math.radians(course_deg),
        speed_mps=speed_mps,
        pos_ts=now,
        motion_ts=now,
        pos_health=1,
        motion_health=1,
    )


def imu_at(now: float) -> _ImuFromApp:
    return _ImuFromApp(gyrz_rad_s=math.radians(5.0), ts=now, health=1)


def baro_at(now: float) -> _BaroFromApp:
    return _BaroFromApp(alt_m=120.0, sink_rate=2.0, rx_ts=now, health=1)


def run_pipeline_case(
    case: str,
    *,
    course_deg: float,
    nu_deg: float,
    speed_mps: float = DEFAULT_SPEED_MPS,
) -> dict[str, Any]:
    reload_modules()
    guidance.reset()
    guidance.set_origin_point(ORIGIN_LAT, ORIGIN_LON)
    target_bearing_deg = course_deg + nu_deg
    target_e, target_n = target_from_bearing(target_bearing_deg)
    target_lat, target_lon = latlon_from_ne(target_n, target_e)
    guidance.set_target_point(target_lat, target_lon)

    mode = guidance.DecideControlMode(
        gps_at(NOW, course_deg=course_deg, speed_mps=speed_mps),
        imu_at(NOW),
        baro_at(NOW),
        NOW,
    )
    l1_in = guidance.ProduceL1Input(NOW)
    l1_out = guidance.ProduceL1Output(l1_in)
    result = {
        "case": case,
        "source": "pipeline",
        "mode": mode,
        "l1_in": l1_in,
        "l1_out": l1_out,
        "row": row_from_l1(case, "pipeline", l1_in, l1_out),
    }
    result["row"]["mode"] = mode.value
    return result


def direct_cases() -> list[dict[str, Any]]:
    return [
        run_direct_case("direct_positive_nu", course_deg=0.0, nu_deg=30.0),
        run_direct_case("direct_negative_nu", course_deg=0.0, nu_deg=-30.0),
        run_direct_case("direct_deadband_nu", course_deg=0.0, nu_deg=config.NU_DEADBAND_DEG * 0.5),
        run_direct_case("direct_dr_conf_full", mode=guidance.ControlMode.DR_M_G_CLOSED, course_deg=0.0, nu_deg=20.0, confidence=1.0),
        run_direct_case("direct_dr_conf_half", mode=guidance.ControlMode.DR_M_G_CLOSED, course_deg=0.0, nu_deg=20.0, confidence=0.5),
        run_direct_case("direct_invalid_input", course_deg=0.0, nu_deg=30.0, valid=False),
    ]


def pipeline_cases() -> list[dict[str, Any]]:
    return [
        run_pipeline_case("pipeline_positive_nu", course_deg=0.0, nu_deg=30.0),
        run_pipeline_case("pipeline_negative_nu", course_deg=0.0, nu_deg=-30.0),
        run_pipeline_case("pipeline_deadband_nu", course_deg=0.0, nu_deg=config.NU_DEADBAND_DEG * 0.5),
    ]


def run_all_cases() -> list[dict[str, Any]]:
    return direct_cases() + pipeline_cases()


def write_csv(results: list[dict[str, Any]], path: Path = CSV_PATH) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS, lineterminator="\n")
        writer.writeheader()
        for result in results:
            writer.writerow(result["row"])


def main() -> int:
    results = run_all_cases()
    write_csv(results)
    for result in results:
        row = result["row"]
        print(
            f"{row['case']}: source={row['source']} mode={row['mode']} "
            f"valid={row['l1out_valid']} nu={row['nu_deg']:.3f} "
            f"yaw_rate={row['yaw_rate_cmd_dps']:.3f}dps limit={row['yaw_rate_limit_dps']:.3f}dps"
        )
    print(f"csv: {CSV_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
