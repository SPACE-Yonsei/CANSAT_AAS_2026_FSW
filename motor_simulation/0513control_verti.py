"""Longitudinal slice simulator for the current parafoil guidance/control stack.

Default scenario
----------------
- Start point : (E=0 m, N=0 m)
- Target point: (E=0 m, N=200 m)
- Vehicle     : E=50 m is fixed while N is swept by dy

The production guidance module is stateful, so each sweep point resets and
configures the mission frame before feeding one fresh GPS/IMU sample through
the same guidance/control functions used by motorapp.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from Sensor_Motor import control, guidance  # noqa: E402
from Sensor_Motor.sensor_types import _BaroFromApp, _GpsFromApp, _ImuFromApp  # noqa: E402


DEFAULT_ORIGIN_LAT = 37.0
DEFAULT_ORIGIN_LON = 127.0
DEFAULT_TARGET_E_M = 0.0
DEFAULT_TARGET_N_M = 200.0


@dataclass
class SimulationCase:
    e_m: float
    n_m: float
    course_deg: float
    speed_mps: float
    gyrz_deg_s: float
    dt_s: float


@dataclass
class PathMetrics:
    cross_track_m: float
    along_track_m: float


@dataclass
class SimulationResult:
    case: SimulationCase
    metrics: PathMetrics
    control_mode: guidance.ControlMode
    l1_input: guidance.L1Input
    l1_output: guidance.L1Output
    ctrl_input: control.CtrlInput
    ctrl_output: control.CtrlOutput


def ne_to_latlon(
    north_m: float,
    east_m: float,
    origin_lat: float,
    origin_lon: float,
) -> tuple[float, float]:
    """Convert local N/E offsets into latitude/longitude."""
    lat = origin_lat + math.degrees(north_m / guidance.EARTH_RADIUS_M)
    cos_lat = math.cos(math.radians(origin_lat))
    if abs(cos_lat) < 1.0e-9:
        raise ValueError("origin latitude is too close to the pole")
    lon = origin_lon + math.degrees(east_m / (guidance.EARTH_RADIUS_M * cos_lat))
    return lat, lon


def configure_mission(origin_lat: float, origin_lon: float,
                      target_e_m: float, target_n_m: float) -> None:
    """Reset guidance and pin a deterministic origin/target for one slice."""
    guidance.reset()
    mission = guidance._MISSION_t
    mission.origin_lat = float(origin_lat)
    mission.origin_lon = float(origin_lon)
    mission.origin_ready = True
    mission._raw_lat = float(origin_lat)
    mission._raw_lon = float(origin_lon)

    target_lat, target_lon = ne_to_latlon(
        target_n_m, target_e_m, origin_lat, origin_lon
    )
    guidance.set_target(target_lat, target_lon)


def build_gps(case: SimulationCase, now: float,
              origin_lat: float, origin_lon: float) -> _GpsFromApp:
    lat, lon = ne_to_latlon(case.n_m, case.e_m, origin_lat, origin_lon)
    return _GpsFromApp(
        lat=lat,
        lon=lon,
        course_rad=math.radians(case.course_deg),
        speed_mps=case.speed_mps,
        pos_ts=now,
        motion_ts=now,
        pos_health=1,
        motion_health=1,
    )


def build_imu(case: SimulationCase, now: float) -> _ImuFromApp:
    return _ImuFromApp(
        yaw_rad=math.radians(case.course_deg),
        gyrz_rad_s=math.radians(case.gyrz_deg_s),
        ts=now,
        health=1,
    )


def path_metrics(e_m: float, n_m: float,
                 target_e_m: float, target_n_m: float) -> PathMetrics:
    length = math.hypot(target_e_m, target_n_m)
    if length <= 1.0e-9:
        return PathMetrics(cross_track_m=math.nan, along_track_m=math.nan)
    unit_e = target_e_m / length
    unit_n = target_n_m / length
    along = e_m * unit_e + n_m * unit_n
    cross = e_m * unit_n - n_m * unit_e
    return PathMetrics(cross_track_m=cross, along_track_m=along)


def y_sweep_values(y_start_m: float, y_end_m: float, dy_m: float) -> Iterable[float]:
    if dy_m <= 0.0:
        raise ValueError("dy must be positive")

    y = y_start_m
    eps = abs(dy_m) * 1.0e-9
    while y <= y_end_m + eps:
        yield round(y, 10)
        y += dy_m


def run_single_case(case: SimulationCase, origin_lat: float, origin_lon: float,
                    target_e_m: float, target_n_m: float) -> SimulationResult:
    now = time.monotonic()
    configure_mission(origin_lat, origin_lon, target_e_m, target_n_m)

    gps = build_gps(case, now, origin_lat, origin_lon)
    imu = build_imu(case, now)
    baro = _BaroFromApp()

    guidance.UpdateRaws(gps, imu, baro, now)
    mode = guidance.DecideControlMode(now)
    l1_input = guidance.ProduceL1Input(now)
    l1_output = guidance.ProduceL1Output(l1_input)

    ctrl_input = control.ProduceCtrlInput(l1_output, now)
    ctrler = control.MakeCtrler()
    ctrler.pid.prev_time = now - max(0.0, case.dt_s)
    ctrl_output = control.ProduceCtrlOutput(
        ctrler,
        ctrl_input,
        angular_velocity_meas_deg_s=case.gyrz_deg_s,
        now=now,
    )

    return SimulationResult(
        case=case,
        metrics=path_metrics(case.e_m, case.n_m, target_e_m, target_n_m),
        control_mode=mode,
        l1_input=l1_input,
        l1_output=l1_output,
        ctrl_input=ctrl_input,
        ctrl_output=ctrl_output,
    )


def fmt_deg(rad: float) -> str:
    return "nan" if not math.isfinite(rad) else f"{math.degrees(rad):+7.2f}"


def print_header(args: argparse.Namespace) -> None:
    print("=" * 108)
    print("0513control_verti.py - current guidance/control longitudinal slice")
    print("=" * 108)
    print("Frame      : E positive east/right, N positive north/forward")
    print("Start      : E=0.0 m, N=0.0 m")
    print(f"Target     : E={args.target_e_m:.1f} m, N={args.target_n_m:.1f} m")
    print(f"Vehicle E  : {args.e_m:.1f} m")
    print(
        "N sweep    : "
        f"{args.n_start_m:.1f} m -> {args.n_end_m:.1f} m "
        f"(dN={args.dn_m:.1f} m)"
    )
    print(f"Course     : {args.course_deg:.2f} deg (North=0, East=+90)")
    print(f"Speed      : {args.speed_mps:.2f} m/s")
    print(f"gyrz_nav   : {args.gyrz_deg_s:.2f} deg/s (right turn positive)")
    print(f"ctrl dt    : {args.dt_s:.3f} s")
    print()


def print_summary_row(result: SimulationResult) -> None:
    g = result.l1_output
    c = result.ctrl_output
    print(
        f"N={result.case.n_m:+7.1f} m | "
        f"cross={result.metrics.cross_track_m:+8.2f} m | "
        f"along={result.metrics.along_track_m:+8.2f} m | "
        f"dist={g.distance_to_target:7.2f} m | "
        f"nu={fmt_deg(g.nu)} deg | "
        f"cmd={math.degrees(g.yaw_rate_cmd):+7.2f} deg/s | "
        f"delta={c.delta_arm_deg:+7.2f} deg | "
        f"L={c.left_angle_deg:6.2f} deg | "
        f"R={c.right_angle_deg:6.2f} deg | "
        f"{result.control_mode.value}/{c.mode}/{c.fallback_mode}"
    )


def print_full_outputs(result: SimulationResult) -> None:
    print()
    print("-" * 108)
    print(
        f"Case E={result.case.e_m:+.1f} m, N={result.case.n_m:+.1f} m, "
        f"course={result.case.course_deg:.2f} deg, "
        f"speed={result.case.speed_mps:.2f} m/s, "
        f"gyrz={result.case.gyrz_deg_s:.2f} deg/s"
    )
    print("-" * 108)
    print(f"ControlMode: {result.control_mode}")
    print(f"L1Input    : {result.l1_input}")
    print(f"L1Output   : {result.l1_output}")
    print(f"CtrlInput  : {result.ctrl_input}")
    print(f"CtrlOutput : {result.ctrl_output}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Sweep N at fixed E and print current L1/control outputs."
    )
    parser.add_argument("--course-deg", type=float, default=0.0)
    parser.add_argument("--speed-mps", type=float, default=5.0)
    parser.add_argument("--gyrz-deg-s", type=float, default=0.0)
    parser.add_argument("--dt-s", type=float, default=0.05)
    parser.add_argument("--e-m", type=float, default=50.0)
    parser.add_argument("--n-start-m", type=float, default=-80.0)
    parser.add_argument("--n-end-m", type=float, default=300.0)
    parser.add_argument("--dn-m", type=float, default=40.0)
    parser.add_argument("--target-e-m", type=float, default=DEFAULT_TARGET_E_M)
    parser.add_argument("--target-n-m", type=float, default=DEFAULT_TARGET_N_M)
    parser.add_argument("--origin-lat", type=float, default=DEFAULT_ORIGIN_LAT)
    parser.add_argument("--origin-lon", type=float, default=DEFAULT_ORIGIN_LON)
    parser.add_argument(
        "--full",
        action="store_true",
        help="Print full L1Input/L1Output/CtrlInput/CtrlOutput structures.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    print_header(args)

    results: list[SimulationResult] = []
    for n_m in y_sweep_values(args.n_start_m, args.n_end_m, args.dn_m):
        case = SimulationCase(
            e_m=args.e_m,
            n_m=n_m,
            course_deg=args.course_deg,
            speed_mps=args.speed_mps,
            gyrz_deg_s=args.gyrz_deg_s,
            dt_s=args.dt_s,
        )
        result = run_single_case(
            case,
            origin_lat=args.origin_lat,
            origin_lon=args.origin_lon,
            target_e_m=args.target_e_m,
            target_n_m=args.target_n_m,
        )
        results.append(result)
        print_summary_row(result)

    if args.full:
        for result in results:
            print_full_outputs(result)


if __name__ == "__main__":
    main()
