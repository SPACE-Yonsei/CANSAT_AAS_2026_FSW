"""
Offline motor control scenario sweep — no hardware, no IPC required.

Directly calls guidance.ProduceL1Output + control.ProduceCtrlOutput with
synthetic sensor values to map how control output changes across flight situations.
"""
from __future__ import annotations

import csv
import math
import sys
import argparse
import dataclasses
from dataclasses import dataclass
from typing import Optional
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from Sensor_Motor import guidance, control
from Sensor_Motor.guidance import L1Input, SensorQuality, ControlMode


# ── Canonical geometry ─────────────────────────────────────────────────────────
# Track runs due north from ORIGIN; target is TRACK_DEFAULT_LEN_M away.
_ORIGIN_LAT = 37.55
_ORIGIN_LON  = 126.96
_EARTH_R     = 6_371_000.0


def _target_latlon(track_length_m: float) -> tuple[float, float]:
    lat = _ORIGIN_LAT + math.degrees(track_length_m / _EARTH_R)
    return lat, _ORIGIN_LON


# ── Data structures ────────────────────────────────────────────────────────────

@dataclass
class ScenarioParams:
    """Defines a single flight situation to evaluate."""
    bearing_error_deg: float = 0.0
    """course − north. 0 = flying straight toward target, +90 = flying east."""
    cross_track_m: float = 0.0
    """Lateral offset from the start→target line. Left of track = negative E."""
    along_track_ratio: float = 0.5
    """Position along the track: 0.0 = start, 1.0 = target."""
    ground_speed_mps: float = 8.0
    gyrz_deg_s: float = 0.0
    """Current yaw rate in deg/s (positive = turning right)."""
    pos_quality: str = "FRESH"
    """FRESH | FRESHED | STALE"""
    gyrz_quality: str = "FRESH"
    control_mode_override: Optional[str] = None
    """Force a specific ControlMode string. None = auto-decide."""
    track_length_m: float = 1000.0


@dataclass
class ScenarioResult:
    # Layer A — inputs
    bearing_error_deg: float
    cross_track_m: float
    along_track_ratio: float
    ground_speed_mps: float
    gyrz_deg_s: float
    pos_quality: str
    gyrz_quality: str
    track_length_m: float
    # Layer B — guidance intermediates
    control_mode: str
    fail_reason: str
    nu_deg: float
    cross_track_calc_m: float
    L1_distance_m: float
    lat_acc_cmd_mps2: float
    angular_velocity_cmd_deg_s: float
    # Layer C — control outputs
    delta_ff_deg: float
    delta_pid_deg: float
    delta_arm_deg: float
    left_pw: int
    right_pw: int


# ── Core ───────────────────────────────────────────────────────────────────────

def _build_l1_input(params: ScenarioParams) -> tuple[L1Input, ControlMode]:
    target_lat, target_lon = _target_latlon(params.track_length_m)
    pq = SensorQuality[params.pos_quality]
    gq = SensorQuality[params.gyrz_quality]

    l1 = L1Input(
        pos_N=params.along_track_ratio * params.track_length_m,
        pos_E=params.cross_track_m,
        course=math.radians(params.bearing_error_deg),
        ground_speed_mps=params.ground_speed_mps,
        gyrz=math.radians(params.gyrz_deg_s),
        gyrx=0.0,
        gyry=0.0,
        alt=200.0,
        pos_quality=pq,
        motion_quality=pq,
        gyrz_quality=gq,
        alt_quality=SensorQuality.FRESH,
        origin_lat=_ORIGIN_LAT,
        origin_lon=_ORIGIN_LON,
        target_lat=target_lat,
        target_lon=target_lon,
        dr_valid=False,
        freefall=0,
        tumble=0,
    )

    if params.control_mode_override is not None:
        mode = ControlMode(params.control_mode_override)
    else:
        mode, _ = guidance.DecideControlMode(l1)

    return l1, mode


def run_scenario(params: ScenarioParams) -> ScenarioResult:
    """Run the full guidance + control pipeline for one scenario."""
    target_lat, target_lon = _target_latlon(params.track_length_m)
    l1_input, mode = _build_l1_input(params)
    now = 0.0

    g_out = guidance.ProduceL1Output(
        l1_input, mode,
        _ORIGIN_LAT, _ORIGIN_LON,
        target_lat, target_lon,
        now, l1_state=None,
    )

    ctl = control.MakeCtrler()
    ctrl_in = control.ProduceCtrlInput(g_out, now)
    cmd = control.ProduceCtrlOutput(
        ctl, ctrl_in, math.radians(params.gyrz_deg_s), now
    )

    return ScenarioResult(
        bearing_error_deg=params.bearing_error_deg,
        cross_track_m=params.cross_track_m,
        along_track_ratio=params.along_track_ratio,
        ground_speed_mps=params.ground_speed_mps,
        gyrz_deg_s=params.gyrz_deg_s,
        pos_quality=params.pos_quality,
        gyrz_quality=params.gyrz_quality,
        track_length_m=params.track_length_m,
        control_mode=g_out.reason,
        fail_reason=g_out.fail_reason,
        nu_deg=math.degrees(g_out.nu),
        cross_track_calc_m=g_out.crossTrack,
        L1_distance_m=g_out.L1_distance,
        lat_acc_cmd_mps2=g_out.lat_acc_cmd_mps2,
        angular_velocity_cmd_deg_s=math.degrees(g_out.angular_velocity_cmd_rad_s),
        delta_ff_deg=cmd.delta_ff_deg,
        delta_pid_deg=cmd.delta_pid_deg,
        delta_arm_deg=cmd.delta_arm_deg,
        left_pw=cmd.left_pw,
        right_pw=cmd.right_pw,
    )


def sweep(param_grid: list[ScenarioParams]) -> list[ScenarioResult]:
    return [run_scenario(p) for p in param_grid]


# ── Preset sweeps ──────────────────────────────────────────────────────────────

def sweep_bearing_error(
    cross_track_m: float = 0.0,
    ground_speed_mps: float = 8.0,
    gyrz_deg_s: float = 0.0,
    step_deg: float = 5.0,
    **kw,
) -> list[ScenarioResult]:
    """bearing_error를 −180 ~ +180 범위로 스윕."""
    return sweep([
        ScenarioParams(
            bearing_error_deg=b,
            cross_track_m=cross_track_m,
            ground_speed_mps=ground_speed_mps,
            gyrz_deg_s=gyrz_deg_s,
            **kw,
        )
        for b in _frange(-180.0, 180.0, step_deg)
    ])


def sweep_cross_track(
    bearing_error_deg: float = 0.0,
    ground_speed_mps: float = 8.0,
    cross_track_range_m: float = 300.0,
    step_m: float = 10.0,
    **kw,
) -> list[ScenarioResult]:
    """cross_track을 ±range 범위로 스윕."""
    return sweep([
        ScenarioParams(
            bearing_error_deg=bearing_error_deg,
            cross_track_m=c,
            ground_speed_mps=ground_speed_mps,
            **kw,
        )
        for c in _frange(-cross_track_range_m, cross_track_range_m, step_m)
    ])


def sweep_speed(
    bearing_error_deg: float = 30.0,
    cross_track_m: float = 0.0,
    speed_min: float = 3.0,
    speed_max: float = 15.0,
    step: float = 0.5,
    **kw,
) -> list[ScenarioResult]:
    """ground_speed를 speed_min ~ speed_max 범위로 스윕."""
    return sweep([
        ScenarioParams(
            bearing_error_deg=bearing_error_deg,
            cross_track_m=cross_track_m,
            ground_speed_mps=s,
            **kw,
        )
        for s in _frange(speed_min, speed_max, step)
    ])


def _frange(start: float, stop: float, step: float) -> list[float]:
    vals, v = [], start
    while v <= stop + step * 1e-6:
        vals.append(round(v, 9))
        v += step
    return vals


# ── Output helpers ─────────────────────────────────────────────────────────────

def results_to_csv(results: list[ScenarioResult], out=None) -> None:
    if not results:
        return
    fields = [f.name for f in dataclasses.fields(ScenarioResult)]
    dest = out or sys.stdout
    w = csv.DictWriter(dest, fieldnames=fields, lineterminator="\n")
    w.writeheader()
    for r in results:
        row = dataclasses.asdict(r)
        for k, v in row.items():
            if isinstance(v, float):
                row[k] = f"{v:.4f}"
        w.writerow(row)


def print_single(r: ScenarioResult) -> None:
    print("=== Scenario Result ===")
    print("  [Input]")
    print(f"    bearing_error     : {r.bearing_error_deg:+.1f} deg")
    print(f"    cross_track       : {r.cross_track_m:+.1f} m")
    print(f"    along_track_ratio : {r.along_track_ratio:.2f}")
    print(f"    ground_speed      : {r.ground_speed_mps:.1f} m/s")
    print(f"    gyrz              : {r.gyrz_deg_s:+.1f} deg/s")
    print(f"    pos_quality       : {r.pos_quality}")
    print(f"    gyrz_quality      : {r.gyrz_quality}")
    print("  [Guidance]")
    print(f"    control_mode      : {r.control_mode}")
    print(f"    fail_reason       : {r.fail_reason}")
    print(f"    nu                : {r.nu_deg:+.2f} deg")
    print(f"    cross_track_calc  : {r.cross_track_calc_m:+.2f} m")
    print(f"    L1_distance       : {r.L1_distance_m:.2f} m")
    print(f"    lat_acc_cmd       : {r.lat_acc_cmd_mps2:+.4f} m/s²")
    print(f"    angular_vel_cmd   : {r.angular_velocity_cmd_deg_s:+.2f} deg/s")
    print("  [Control Output]")
    print(f"    delta_ff          : {r.delta_ff_deg:+.2f} deg")
    print(f"    delta_pid         : {r.delta_pid_deg:+.2f} deg")
    print(f"    delta_arm         : {r.delta_arm_deg:+.2f} deg")
    print(f"    left_pw           : {r.left_pw} μs")
    print(f"    right_pw          : {r.right_pw} μs")


# ── CLI ────────────────────────────────────────────────────────────────────────

def _add_quality_args(p):
    p.add_argument("--pos-quality",  default="FRESH", choices=["FRESH", "FRESHED", "STALE"])
    p.add_argument("--gyrz-quality", default="FRESH", choices=["FRESH", "FRESHED", "STALE"])


def _parse_args():
    root = argparse.ArgumentParser(
        description="Offline motor control scenario sweep (no hardware required)"
    )
    sub = root.add_subparsers(dest="mode", required=True)

    # single ─────────────────────────────────────────────────────────────────
    s = sub.add_parser("single", help="Run one scenario, print detailed output")
    s.add_argument("--bearing-error", type=float, default=0.0, metavar="DEG")
    s.add_argument("--cross-track",   type=float, default=0.0, metavar="M")
    s.add_argument("--along-ratio",   type=float, default=0.5, metavar="0-1")
    s.add_argument("--speed",         type=float, default=8.0, metavar="MPS")
    s.add_argument("--gyrz",          type=float, default=0.0, metavar="DEG_S")
    s.add_argument("--mode-override", default=None)
    s.add_argument("--track-length",  type=float, default=1000.0, metavar="M")
    _add_quality_args(s)

    # bearing ─────────────────────────────────────────────────────────────────
    b = sub.add_parser("bearing", help="Sweep bearing_error −180 → +180")
    b.add_argument("--cross-track", type=float, default=0.0, metavar="M")
    b.add_argument("--speed",       type=float, default=8.0,  metavar="MPS")
    b.add_argument("--gyrz",        type=float, default=0.0,  metavar="DEG_S")
    b.add_argument("--step",        type=float, default=5.0,  metavar="DEG")
    b.add_argument("--out",         default=None, metavar="FILE")
    _add_quality_args(b)

    # cross-track ─────────────────────────────────────────────────────────────
    ct = sub.add_parser("cross-track", help="Sweep cross_track ±range")
    ct.add_argument("--bearing-error", type=float, default=0.0,   metavar="DEG")
    ct.add_argument("--speed",         type=float, default=8.0,   metavar="MPS")
    ct.add_argument("--range",         type=float, default=300.0, metavar="M")
    ct.add_argument("--step",          type=float, default=10.0,  metavar="M")
    ct.add_argument("--out",           default=None, metavar="FILE")
    _add_quality_args(ct)

    # speed ───────────────────────────────────────────────────────────────────
    sp = sub.add_parser("speed", help="Sweep ground speed")
    sp.add_argument("--bearing-error", type=float, default=30.0, metavar="DEG")
    sp.add_argument("--cross-track",   type=float, default=0.0,  metavar="M")
    sp.add_argument("--speed-min",     type=float, default=3.0,  metavar="MPS")
    sp.add_argument("--speed-max",     type=float, default=15.0, metavar="MPS")
    sp.add_argument("--step",          type=float, default=0.5,  metavar="MPS")
    sp.add_argument("--out",           default=None, metavar="FILE")
    _add_quality_args(sp)

    return root.parse_args()


def _write_output(results: list[ScenarioResult], out_path: Optional[str]) -> None:
    if out_path:
        with open(out_path, "w", newline="") as f:
            results_to_csv(results, f)
        print(f"Saved {len(results)} rows → {out_path}", file=sys.stderr)
    else:
        results_to_csv(results)


def main() -> None:
    args = _parse_args()

    if args.mode == "single":
        params = ScenarioParams(
            bearing_error_deg=args.bearing_error,
            cross_track_m=args.cross_track,
            along_track_ratio=args.along_ratio,
            ground_speed_mps=args.speed,
            gyrz_deg_s=args.gyrz,
            pos_quality=args.pos_quality,
            gyrz_quality=args.gyrz_quality,
            control_mode_override=args.mode_override,
            track_length_m=args.track_length,
        )
        print_single(run_scenario(params))

    elif args.mode == "bearing":
        results = sweep_bearing_error(
            cross_track_m=args.cross_track,
            ground_speed_mps=args.speed,
            gyrz_deg_s=args.gyrz,
            step_deg=args.step,
            pos_quality=args.pos_quality,
            gyrz_quality=args.gyrz_quality,
        )
        _write_output(results, args.out)

    elif args.mode == "cross-track":
        results = sweep_cross_track(
            bearing_error_deg=args.bearing_error,
            ground_speed_mps=args.speed,
            cross_track_range_m=args.range,
            step_m=args.step,
            pos_quality=args.pos_quality,
            gyrz_quality=args.gyrz_quality,
        )
        _write_output(results, args.out)

    elif args.mode == "speed":
        results = sweep_speed(
            bearing_error_deg=args.bearing_error,
            cross_track_m=args.cross_track,
            speed_min=args.speed_min,
            speed_max=args.speed_max,
            step=args.step,
            pos_quality=args.pos_quality,
            gyrz_quality=args.gyrz_quality,
        )
        _write_output(results, args.out)


if __name__ == "__main__":
    main()
