"""Longitudinal L1 guidance/control slice simulator for the 2026 CanSat parafoil.

Default scenario
----------------
- Start point : (x=0 m, y=0 m)
- Target point: (x=0 m, y=200 m)
- Vehicle     : x=50 m is fixed while y is swept from -80 m to 300 m

Coordinate mapping used by the flight software:
- x -> East  (E)
- y -> North (N)

This script intentionally reuses the production guidance/control pipeline:

    guidance.ProduceL1Input
    guidance.ProduceL1Output
    control.ProduceCtrlInput
    control.ProduceCtrlOutput
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from typing import Iterable, Optional

from Sensor_Motor import control, guidance


EARTH_R_M = 6_371_000.0
DEFAULT_ORIGIN_LAT = 37.0
DEFAULT_ORIGIN_LON = 127.0


@dataclass
class MockGPS:
    lat: Optional[float]
    lon: Optional[float]
    course_rad: Optional[float]
    speed_mps: Optional[float]
    pos_ts: Optional[float]
    motion_ts: Optional[float]
    pos_health: bool = True
    motion_health: bool = True


@dataclass
class MockIMU:
    gyrz_rad_s: Optional[float]
    ts: Optional[float]


@dataclass
class SimulationCase:
    x_m: float
    y_m: float
    heading_deg: float
    speed_mps: float
    gyrz_deg_s: float


@dataclass
class SimulationResult:
    case: SimulationCase
    l1_input: guidance.L1Input
    control_mode: guidance.ControlMode
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
    lat = origin_lat + math.degrees(north_m / EARTH_R_M)
    lon = origin_lon + math.degrees(
        east_m / (EARTH_R_M * math.cos(math.radians(origin_lat)))
    )
    return lat, lon


def build_gps(case: SimulationCase, now: float) -> MockGPS:
    lat, lon = ne_to_latlon(
        north_m=case.y_m,
        east_m=case.x_m,
        origin_lat=DEFAULT_ORIGIN_LAT,
        origin_lon=DEFAULT_ORIGIN_LON,
    )
    return MockGPS(
        lat=lat,
        lon=lon,
        course_rad=math.radians(case.heading_deg),
        speed_mps=case.speed_mps,
        pos_ts=now,
        motion_ts=now,
        pos_health=True,
        motion_health=True,
    )


def build_imu(case: SimulationCase, now: float) -> MockIMU:
    return MockIMU(
        gyrz_rad_s=math.radians(case.gyrz_deg_s),
        ts=now,
    )


def y_sweep_values(y_start_m: float, y_end_m: float, dy_m: float) -> Iterable[float]:
    if dy_m <= 0.0:
        raise ValueError("dy must be positive.")

    y = y_start_m
    eps = abs(dy_m) * 1e-9
    while y <= y_end_m + eps:
        yield round(y, 10)
        y += dy_m


def run_single_case(case: SimulationCase) -> SimulationResult:
    now = time.monotonic()

    target_lat, target_lon = ne_to_latlon(
        north_m=200.0,
        east_m=0.0,
        origin_lat=DEFAULT_ORIGIN_LAT,
        origin_lon=DEFAULT_ORIGIN_LON,
    )

    gps = build_gps(case, now)
    imu = build_imu(case, now)

    l1_input, mode = guidance.ProduceL1Input(
        gps=gps,
        imu=imu,
        baro=None,
        origin_lat=DEFAULT_ORIGIN_LAT,
        origin_lon=DEFAULT_ORIGIN_LON,
        target_lat=target_lat,
        target_lon=target_lon,
        now=now,
    )
    l1_output = guidance.ProduceL1Output(
        l1_input=l1_input,
        mode=mode,
        origin_lat=DEFAULT_ORIGIN_LAT,
        origin_lon=DEFAULT_ORIGIN_LON,
        target_lat=target_lat,
        target_lon=target_lon,
        now=now,
    )

    ctrl_input = control.ProduceCtrlInput(l1_output, now)
    ctrler = control.MakeCtrler()
    ctrl_output = control.ProduceCtrlOutput(
        ctrler,
        ctrl_input,
        angular_velocity_meas_deg_s=case.gyrz_deg_s,
        now=now,
    )

    return SimulationResult(
        case=case,
        l1_input=l1_input,
        control_mode=mode,
        l1_output=l1_output,
        ctrl_input=ctrl_input,
        ctrl_output=ctrl_output,
    )


def print_header(args: argparse.Namespace) -> None:
    print("=" * 92)
    print("0513control_lon.py - L1 guidance/control longitudinal slice simulator")
    print("=" * 92)
    print("Path frame : x -> East, y -> North")
    print("Start      : (x=0.0 m, y=0.0 m)")
    print("Target     : (x=0.0 m, y=200.0 m)")
    print(f"Vehicle x  : {args.x_m:.1f} m")
    print(
        "Y sweep    : "
        f"{args.y_start_m:.1f} m -> {args.y_end_m:.1f} m "
        f"(dy={args.dy_m:.1f} m)"
    )
    print(f"Heading    : {args.heading_deg:.2f} deg")
    print(f"Speed      : {args.speed_mps:.2f} m/s")
    print(f"gyrz       : {args.gyrz_deg_s:.2f} deg/s")
    print()


def print_summary_row(result: SimulationResult) -> None:
    g = result.l1_output
    c = result.ctrl_output
    print(
        f"y={result.case.y_m:+7.1f} m | "
        f"cross={g.crossTrack:+8.2f} m | "
        f"along={g.alongTrack:+8.2f} m | "
        f"yaw_cmd={g.yaw_rate_cmd:+7.4f} rad/s | "
        f"lat_acc={g.lat_acc_cmd_mps2:+7.3f} m/s^2 | "
        f"L={c.left_angle_deg:6.2f} deg | "
        f"R={c.right_angle_deg:6.2f} deg | "
        f"mode={c.mode}"
    )


def print_full_outputs(result: SimulationResult) -> None:
    print()
    print("-" * 92)
    print(
        f"Case x={result.case.x_m:+.1f} m, y={result.case.y_m:+.1f} m, "
        f"heading={result.case.heading_deg:.2f} deg, "
        f"speed={result.case.speed_mps:.2f} m/s, "
        f"gyrz={result.case.gyrz_deg_s:.2f} deg/s"
    )
    print("-" * 92)
    print(f"ControlMode: {result.control_mode}")
    print(f"L1Input    : {result.l1_input}")
    print(f"L1Output   : {result.l1_output}")
    print(f"CtrlInput  : {result.ctrl_input}")
    print(f"CtrlOutput : {result.ctrl_output}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Sweep y at fixed x=50 m and print production L1/control outputs."
    )
    parser.add_argument(
        "--heading-deg",
        type=float,
        default=0.0,
        help="Vehicle ground-course azimuth in degrees. 0 deg = North.",
    )
    parser.add_argument(
        "--speed-mps",
        type=float,
        default=5.0,
        help="Vehicle ground speed in m/s.",
    )
    parser.add_argument(
        "--gyrz-deg-s",
        type=float,
        default=0.0,
        help="Measured yaw rate in deg/s.",
    )
    parser.add_argument(
        "--x-m",
        type=float,
        default=50.0,
        help="Fixed vehicle x/East position in meters.",
    )
    parser.add_argument(
        "--y-start-m",
        type=float,
        default=-80.0,
        help="Sweep start y/North position in meters.",
    )
    parser.add_argument(
        "--y-end-m",
        type=float,
        default=300.0,
        help="Sweep end y/North position in meters.",
    )
    parser.add_argument(
        "--dy-m",
        type=float,
        default=40.0,
        help="Sweep increment in meters.",
    )
    parser.add_argument(
        "--full",
        action="store_true",
        help="Print full L1Input/L1Output/CtrlInput/CtrlOutput structures for every point.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    print_header(args)

    results: list[SimulationResult] = []
    for y_m in y_sweep_values(args.y_start_m, args.y_end_m, args.dy_m):
        case = SimulationCase(
            x_m=args.x_m,
            y_m=y_m,
            heading_deg=args.heading_deg,
            speed_mps=args.speed_mps,
            gyrz_deg_s=args.gyrz_deg_s,
        )
        result = run_single_case(case)
        results.append(result)
        print_summary_row(result)

    if args.full:
        for result in results:
            print_full_outputs(result)


if __name__ == "__main__":
    main()
