"""
Offline mag-guidance scenario sweep — no hardware, no IPC required.

GPS-free branch: directly calls mag_guidance.ProduceMagGuidance +
control.ProduceCtrlOutput with synthetic IMU/baro values to map how
control output changes across heading errors, yaw rates, and altitudes.

Usage examples
--------------
# single scenario
python tests/scenario_sweep.py single --heading-error 30 --gyrz 0

# sweep heading error
python tests/scenario_sweep.py bearing --step 5

# sweep yaw rate (damping effect)
python tests/scenario_sweep.py gyrz --heading-error 30 --step 2

# closed-loop heading convergence + position trace
python tests/scenario_sweep.py flight --heading-error 30 --alt 200
python tests/scenario_sweep.py flight --multi 0:200 30:150 -60:200 90:300

# save to CSV
python tests/scenario_sweep.py bearing --out bearing_sweep.csv
"""
from __future__ import annotations

import csv
import math
import sys
import argparse
import dataclasses
import random
from dataclasses import dataclass
from typing import Optional
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from Sensor_Motor import control
from Sensor_Motor.mag_guidance import (
    MagGuidanceConfig,
    MagGuidanceInput,
    ProduceMagGuidance,
    MODE_NOMINAL,
    MODE_DEGRADED,
    MODE_NO_BEARING,
    MODE_IMU_FAIL,
)

# ── Default parameters ─────────────────────────────────────────────────────────
_DEFAULT_TARGET_BEARING_DEG = 0.0    # 정북으로 타겟이 있다고 가정
_DEFAULT_ALT_M              = 200.0
_DEFAULT_SINK_RATE_MPS      = 3.0
_DEFAULT_AIRSPEED_MPS       = 8.0
_MAG_CFG                    = MagGuidanceConfig()  # Kp=1.2, Kd=0.6, max=45 deg/s


# ── Data structures ────────────────────────────────────────────────────────────
@dataclass
class ScenarioParams:
    """Defines one flight situation to evaluate."""
    heading_error_deg: float = 0.0
    """current_yaw − target_bearing. 0 = aligned, +30 = 30° clockwise of target."""
    gyrz_deg_s: float = 0.0
    """Current yaw rate (deg/s, CW = positive)."""
    alt_m: float = _DEFAULT_ALT_M
    target_bearing_deg: float = _DEFAULT_TARGET_BEARING_DEG
    imu_health: int = 1
    baro_health: int = 1
    imu_age_s: float = 0.05
    baro_age_s: float = 0.10
    cfg: MagGuidanceConfig = dataclasses.field(default_factory=MagGuidanceConfig)


@dataclass
class ScenarioResult:
    # ── inputs ──────────────────────────────────────────────────────────────
    heading_error_deg: float
    gyrz_deg_s: float
    alt_m: float
    target_bearing_deg: float
    current_yaw_deg: float
    # ── guidance ─────────────────────────────────────────────────────────────
    mode: str
    angular_velocity_cmd_deg_s: float
    # ── control ──────────────────────────────────────────────────────────────
    delta_ff_deg: float
    delta_pid_deg: float
    delta_arm_deg: float
    left_angle_deg: float
    right_angle_deg: float
    left_pw: int
    right_pw: int


@dataclass
class FlightResult:
    """One timestep of closed-loop heading-convergence simulation."""
    t_s: float
    heading_deg: float
    heading_error_deg: float
    gyrz_cmd_deg_s: float
    alt_m: float
    pos_n_m: float
    pos_e_m: float
    cross_track_m: float    # lateral deviation from target-bearing line
    along_track_m: float    # distance advanced along target-bearing line
    mode: str
    delta_arm_deg: float
    left_pw: int
    right_pw: int


# ── Core single-scenario runner ────────────────────────────────────────────────
def run_scenario(params: ScenarioParams) -> ScenarioResult:
    now = 100.0  # fixed reference time
    yaw_deg = params.target_bearing_deg + params.heading_error_deg

    inp = MagGuidanceInput(
        yaw_rad            = math.radians(yaw_deg),
        gyrz_rad_s         = math.radians(params.gyrz_deg_s),
        alt_m              = params.alt_m,
        target_bearing_rad = math.radians(params.target_bearing_deg),
        timestamp          = now,
        imu_ts             = now - params.imu_age_s,
        baro_ts            = now - params.baro_age_s,
        imu_health         = params.imu_health,
        baro_health        = params.baro_health,
    )
    g_out = ProduceMagGuidance(inp, params.cfg)

    ctl = control.MakeCtrler()
    gcmd = control.CtrlInput(
        angular_velocity_cmd_deg_s = g_out.angular_velocity_cmd_deg_s,
        valid                      = g_out.valid,
        timestamp                  = now,
    )
    # warm slew limiter
    cmd_warm = control.ProduceCtrlOutput(ctl, gcmd, params.gyrz_deg_s, now)
    _, _, ld, rd, _ = control.ConnectRoMo(cmd_warm.delta_arm_deg)
    ctl.prev_left_angle_deg  = ld
    ctl.prev_right_angle_deg = rd
    cmd = control.ProduceCtrlOutput(ctl, gcmd, params.gyrz_deg_s, now + 0.05)

    return ScenarioResult(
        heading_error_deg          = params.heading_error_deg,
        gyrz_deg_s                 = params.gyrz_deg_s,
        alt_m                      = params.alt_m,
        target_bearing_deg         = params.target_bearing_deg,
        current_yaw_deg            = yaw_deg,
        mode                       = g_out.mode,
        angular_velocity_cmd_deg_s = g_out.angular_velocity_cmd_deg_s,
        delta_ff_deg               = cmd.delta_ff_deg,
        delta_pid_deg              = cmd.delta_pid_deg,
        delta_arm_deg              = cmd.delta_arm_deg,
        left_angle_deg             = cmd.left_angle_deg,
        right_angle_deg            = cmd.right_angle_deg,
        left_pw                    = cmd.left_pw,
        right_pw                   = cmd.right_pw,
    )


def sweep(param_grid: list[ScenarioParams]) -> list[ScenarioResult]:
    return [run_scenario(p) for p in param_grid]


# ── Preset sweeps ──────────────────────────────────────────────────────────────
def sweep_bearing_error(
    gyrz_deg_s: float = 0.0,
    alt_m: float = _DEFAULT_ALT_M,
    step_deg: float = 5.0,
    target_bearing_deg: float = _DEFAULT_TARGET_BEARING_DEG,
    **kw,
) -> list[ScenarioResult]:
    """heading_error를 −180 ~ +180 범위로 스윕."""
    return sweep([
        ScenarioParams(
            heading_error_deg  = e,
            gyrz_deg_s         = gyrz_deg_s,
            alt_m              = alt_m,
            target_bearing_deg = target_bearing_deg,
            **kw,
        )
        for e in _frange(-180.0, 180.0, step_deg)
    ])


def sweep_gyrz(
    heading_error_deg: float = 30.0,
    alt_m: float = _DEFAULT_ALT_M,
    gyrz_min: float = -60.0,
    gyrz_max: float = 60.0,
    step_deg_s: float = 2.0,
    **kw,
) -> list[ScenarioResult]:
    """yaw rate를 스윕하여 damping 효과 확인."""
    return sweep([
        ScenarioParams(
            heading_error_deg = heading_error_deg,
            gyrz_deg_s        = g,
            alt_m             = alt_m,
            **kw,
        )
        for g in _frange(gyrz_min, gyrz_max, step_deg_s)
    ])


def sweep_altitude(
    heading_error_deg: float = 30.0,
    gyrz_deg_s: float = 0.0,
    alt_min: float = 0.0,
    alt_max: float = 400.0,
    step_m: float = 20.0,
    **kw,
) -> list[ScenarioResult]:
    """고도를 스윕 (제어 출력은 고도에 무관해야 함 — 검증용)."""
    return sweep([
        ScenarioParams(
            heading_error_deg = heading_error_deg,
            gyrz_deg_s        = gyrz_deg_s,
            alt_m             = a,
            **kw,
        )
        for a in _frange(alt_min, alt_max, step_m)
    ])


# ── Closed-loop trajectory simulation ─────────────────────────────────────────
def simulate_flight(
    heading_error_deg: float = 30.0,
    alt_m: float = _DEFAULT_ALT_M,
    airspeed_mps: float = _DEFAULT_AIRSPEED_MPS,
    sink_rate_mps: float = _DEFAULT_SINK_RATE_MPS,
    target_bearing_deg: float = _DEFAULT_TARGET_BEARING_DEG,
    dt: float = 1.0,
    max_steps: int = 500,
    wind_n_mps: float = 0.0,
    wind_e_mps: float = 0.0,
    gust_sigma_deg_s: float = 0.0,
    gust_seed: int = 42,
    cfg: MagGuidanceConfig = MagGuidanceConfig(),
) -> list[FlightResult]:
    """폐루프 방위각 수렴 + 낙하 위치 시뮬레이션.

    매 dt초마다:
      1. mag_guidance 실행 → angular_velocity_cmd
      2. heading += angular_velocity_cmd * dt  (+ 선택적 돌풍)
      3. 위치 = airspeed * heading 벡터 + 바람
      4. alt  -= sink_rate * dt
    cross_track: 타겟 방향 선(bearing line)에서 수직 편차
    along_track: 타겟 방향으로의 전진 거리
    종료: alt ≤ 0 또는 max_steps
    """
    rng = random.Random(gust_seed)
    heading_deg = target_bearing_deg + heading_error_deg
    pos_n = 0.0
    pos_e = 0.0
    current_alt = float(alt_m)
    results: list[FlightResult] = []

    # bearing unit vector (N, E)
    bear_rad = math.radians(target_bearing_deg)
    bear_n = math.cos(bear_rad)
    bear_e = math.sin(bear_rad)

    ctl = control.MakeCtrler()

    for step in range(max_steps):
        t = step * dt
        if current_alt <= 0.0:
            break

        gust = rng.gauss(0.0, gust_sigma_deg_s) if gust_sigma_deg_s > 0.0 else 0.0
        now_fake = 100.0 + t

        inp = MagGuidanceInput(
            yaw_rad            = math.radians(heading_deg),
            gyrz_rad_s         = math.radians(0.0),  # perfect rate sensor (no lag model)
            alt_m              = current_alt,
            target_bearing_rad = math.radians(target_bearing_deg),
            timestamp          = now_fake,
            imu_ts             = now_fake - 0.05,
            baro_ts            = now_fake - 0.10,
            imu_health         = 1,
            baro_health        = 1,
        )
        g_out = ProduceMagGuidance(inp, cfg)

        gcmd = control.CtrlInput(
            angular_velocity_cmd_deg_s = g_out.angular_velocity_cmd_deg_s,
            valid                      = g_out.valid,
            timestamp                  = now_fake,
        )
        cmd = control.ProduceCtrlOutput(
            ctl, gcmd, float("nan"), now_fake
        )

        # along/cross track
        along = pos_n * bear_n + pos_e * bear_e
        cross = pos_e * bear_n - pos_n * bear_e   # + = right of bearing line

        results.append(FlightResult(
            t_s               = t,
            heading_deg       = heading_deg,
            heading_error_deg = (heading_deg - target_bearing_deg + 180) % 360 - 180,
            gyrz_cmd_deg_s    = g_out.angular_velocity_cmd_deg_s,
            alt_m             = current_alt,
            pos_n_m           = pos_n,
            pos_e_m           = pos_e,
            cross_track_m     = cross,
            along_track_m     = along,
            mode              = g_out.mode,
            delta_arm_deg     = cmd.delta_arm_deg,
            left_pw           = cmd.left_pw,
            right_pw          = cmd.right_pw,
        ))

        # dynamics update
        heading_deg += g_out.angular_velocity_cmd_deg_s * dt + gust
        heading_deg = (heading_deg + 180.0) % 360.0 - 180.0
        heading_rad = math.radians(heading_deg)

        vn = airspeed_mps * math.cos(heading_rad) + wind_n_mps
        ve = airspeed_mps * math.sin(heading_rad) + wind_e_mps
        pos_n      += vn * dt
        pos_e      += ve * dt
        current_alt -= sink_rate_mps * dt

    return results


# ── Helpers ────────────────────────────────────────────────────────────────────
def _frange(start: float, stop: float, step: float) -> list[float]:
    vals, v = [], start
    while v <= stop + step * 1e-6:
        vals.append(round(v, 9))
        v += step
    return vals


# ── Output ────────────────────────────────────────────────────────────────────
def results_to_csv(results, out=None) -> None:
    if not results:
        return
    fields = [f.name for f in dataclasses.fields(results[0].__class__)]
    dest = out or sys.stdout
    w = csv.DictWriter(dest, fieldnames=fields, lineterminator="\n")
    w.writeheader()
    for r in results:
        row = dataclasses.asdict(r)
        for k, v in row.items():
            if isinstance(v, float):
                row[k] = f"{v:.4f}"
        w.writerow(row)


_SCENARIO_HDR = (
    f"{'herr':>7}  {'gyrz':>6}  {'alt':>6}  {'mode':<12}  "
    f"{'w/s':>7}  {'darm':>6}  {'L-arm':>6}  {'R-arm':>6}  {'Lpw':>5}  {'Rpw':>5}"
)
_FLIGHT_HDR = (
    f"{'t(s)':>6}  {'yaw':>7}  {'herr':>7}  {'alt':>6}  "
    f"{'along':>8}  {'cross':>8}  {'mode':<12}  "
    f"{'w/s':>7}  {'darm':>6}  {'Lpw':>5}  {'Rpw':>5}"
)
_SEP = "-" * 90

_ARM_LEGEND = (
    "arm-angle convention (control.ConnectRoMo):\n"
    "  neutral=80 deg  |  range 0~160  |  delta>0 → RIGHT  delta<0 → LEFT\n"
    "  RIGHT-turn: L-arm<80, R-arm>80  |  LEFT-turn: L-arm>80, R-arm<80\n"
    "  Lpw = LEFT_ZERO(2480) - L-arm*11.11   Rpw = RIGHT_ZERO(636) + R-arm*11.11"
)


def print_scenario_table(results: list[ScenarioResult], title: str = "") -> None:
    if title:
        print(f"\n{_SEP}\n{title}")
        print("  herr=heading-error(deg)  gyrz=yaw-rate(deg/s)  alt=altitude(m)")
        print("  w/s=ang-vel-cmd(deg/s)   darm=delta-arm(deg)")
    print(_SCENARIO_HDR)
    print(_SEP)
    for r in results:
        print(
            f"{r.heading_error_deg:+7.1f}  {r.gyrz_deg_s:+6.1f}  {r.alt_m:6.1f}  "
            f"{r.mode:<12}  {r.angular_velocity_cmd_deg_s:+7.2f}  "
            f"{r.delta_arm_deg:+6.2f}  {r.left_angle_deg:6.2f}  {r.right_angle_deg:6.2f}  "
            f"{r.left_pw:5d}  {r.right_pw:5d}"
        )
    print(_SEP)
    print(_ARM_LEGEND)


def print_flight_table(results: list[FlightResult], title: str = "", dt: float = 1.0) -> None:
    if title:
        print(f"\n{_SEP}\n{title}")
        print("  herr=heading-error(deg)  along/cross=track(m)  w/s=cmd-yaw-rate(deg/s)")
    print(_FLIGHT_HDR)
    print(_SEP)
    for r in results:
        print(
            f"{r.t_s:6.1f}  {r.heading_deg:+7.1f}  {r.heading_error_deg:+7.1f}  "
            f"{r.alt_m:6.1f}  {r.along_track_m:+8.1f}  {r.cross_track_m:+8.1f}  "
            f"{r.mode:<12}  {r.gyrz_cmd_deg_s:+7.2f}  {r.delta_arm_deg:+6.2f}  "
            f"{r.left_pw:5d}  {r.right_pw:5d}"
        )
    print(_SEP)
    if results:
        last = results[-1]
        hdg_conv = abs(last.heading_error_deg) < 5.0
        print(f"  Landing: alt={last.alt_m:.1f}m  along={last.along_track_m:+.1f}m  "
              f"cross={last.cross_track_m:+.1f}m  "
              f"heading_err={last.heading_error_deg:+.1f}°  "
              f"{'[OK] CONVERGED' if hdg_conv else '[!!] NOT CONVERGED'}")
    print(_ARM_LEGEND)


def print_single(r: ScenarioResult) -> None:
    print("=== Mag-Guidance Scenario Result ===")
    print("  [Input]")
    print(f"    heading_error     : {r.heading_error_deg:+.1f} deg")
    print(f"    current_yaw       : {r.current_yaw_deg:+.1f} deg")
    print(f"    target_bearing    : {r.target_bearing_deg:+.1f} deg")
    print(f"    gyrz              : {r.gyrz_deg_s:+.1f} deg/s")
    print(f"    alt               : {r.alt_m:.1f} m")
    print("  [Guidance]")
    print(f"    mode              : {r.mode}")
    print(f"    angular_vel_cmd   : {r.angular_velocity_cmd_deg_s:+.2f} deg/s")
    print("  [Control Output]")
    print(f"    delta_ff          : {r.delta_ff_deg:+.2f} deg")
    print(f"    delta_pid         : {r.delta_pid_deg:+.2f} deg")
    print(f"    delta_arm         : {r.delta_arm_deg:+.2f} deg")
    print(f"    left_angle        : {r.left_angle_deg:.2f} deg  (neutral=80)")
    print(f"    right_angle       : {r.right_angle_deg:.2f} deg  (neutral=80)")
    print(f"    left_pw           : {r.left_pw} us")
    print(f"    right_pw          : {r.right_pw} us")


# ── CLI ────────────────────────────────────────────────────────────────────────
def _parse_args():
    root = argparse.ArgumentParser(
        description="Mag-guidance offline scenario sweep (GPS-free, no hardware)"
    )
    sub = root.add_subparsers(dest="mode", required=True)

    # single ──────────────────────────────────────────────────────────────────
    s = sub.add_parser("single", help="Run one scenario, print detailed output")
    s.add_argument("--heading-error", type=float, default=0.0,   metavar="DEG",
                   help="current_yaw − target_bearing (deg)")
    s.add_argument("--gyrz",          type=float, default=0.0,   metavar="DEG_S")
    s.add_argument("--alt",           type=float, default=200.0, metavar="M")
    s.add_argument("--target-bearing",type=float, default=0.0,   metavar="DEG")

    # bearing sweep ───────────────────────────────────────────────────────────
    b = sub.add_parser("bearing", help="Sweep heading_error −180 → +180")
    b.add_argument("--gyrz",          type=float, default=0.0,  metavar="DEG_S")
    b.add_argument("--alt",           type=float, default=200.0,metavar="M")
    b.add_argument("--step",          type=float, default=5.0,  metavar="DEG")
    b.add_argument("--target-bearing",type=float, default=0.0,  metavar="DEG")
    b.add_argument("--out",           default=None, metavar="FILE")

    # gyrz sweep ──────────────────────────────────────────────────────────────
    g = sub.add_parser("gyrz", help="Sweep yaw rate (damping effect)")
    g.add_argument("--heading-error", type=float, default=30.0, metavar="DEG")
    g.add_argument("--alt",           type=float, default=200.0,metavar="M")
    g.add_argument("--gyrz-min",      type=float, default=-60.0,metavar="DEG_S")
    g.add_argument("--gyrz-max",      type=float, default=60.0, metavar="DEG_S")
    g.add_argument("--step",          type=float, default=2.0,  metavar="DEG_S")
    g.add_argument("--out",           default=None, metavar="FILE")

    # altitude sweep ──────────────────────────────────────────────────────────
    a = sub.add_parser("altitude", help="Sweep altitude (control should be independent)")
    a.add_argument("--heading-error", type=float, default=30.0, metavar="DEG")
    a.add_argument("--gyrz",          type=float, default=0.0,  metavar="DEG_S")
    a.add_argument("--alt-min",       type=float, default=0.0,  metavar="M")
    a.add_argument("--alt-max",       type=float, default=400.0,metavar="M")
    a.add_argument("--step",          type=float, default=20.0, metavar="M")
    a.add_argument("--out",           default=None, metavar="FILE")

    # closed-loop flight ──────────────────────────────────────────────────────
    fl = sub.add_parser(
        "flight",
        help="폐루프 방위각 수렴 + 낙하 위치 시뮬레이션",
    )
    fl.add_argument("--heading-error",  type=float, default=30.0,  metavar="DEG",
                    help="초기 heading error (deg)")
    fl.add_argument("--alt",            type=float, default=200.0, metavar="M",
                    help="초기 고도 (m)")
    fl.add_argument("--speed",          type=float, default=8.0,   metavar="MPS",
                    help="대기속도 (m/s)")
    fl.add_argument("--sink-rate",      type=float, default=3.0,   metavar="MPS",
                    help="하강율 (m/s)")
    fl.add_argument("--target-bearing", type=float, default=0.0,   metavar="DEG",
                    help="타겟 자기 방위각 (deg, 0=N)")
    fl.add_argument("--dt",             type=float, default=1.0,   metavar="SEC")
    fl.add_argument("--wind-n",         type=float, default=0.0,   metavar="MPS")
    fl.add_argument("--wind-e",         type=float, default=0.0,   metavar="MPS")
    fl.add_argument("--gust-sigma",     type=float, default=0.0,   metavar="DEG_S",
                    help="돌풍 표준편차 (deg/s)")
    fl.add_argument("--gust-seed",      type=int,   default=42)
    fl.add_argument(
        "--multi",
        nargs="*",
        metavar="HERR:ALT",
        help="여러 초기조건. 예: --multi 0:200 30:200 -60:300 90:150",
    )
    fl.add_argument("--out", default=None, metavar="FILE")

    return root.parse_args()


def _write_output(results, out_path: Optional[str]) -> None:
    if out_path:
        with open(out_path, "w", newline="") as f:
            results_to_csv(results, f)
        print(f"Saved {len(results)} rows → {out_path}", file=sys.stderr)
    else:
        results_to_csv(results)


def main() -> None:
    args = _parse_args()

    if args.mode == "single":
        r = run_scenario(ScenarioParams(
            heading_error_deg  = args.heading_error,
            gyrz_deg_s         = args.gyrz,
            alt_m              = args.alt,
            target_bearing_deg = args.target_bearing,
        ))
        print_single(r)

    elif args.mode == "bearing":
        results = sweep_bearing_error(
            gyrz_deg_s         = args.gyrz,
            alt_m              = args.alt,
            step_deg           = args.step,
            target_bearing_deg = args.target_bearing,
        )
        if args.out:
            _write_output(results, args.out)
        else:
            print_scenario_table(results,
                title=f"Bearing sweep  gyrz={args.gyrz:+.1f}°/s  alt={args.alt:.0f}m  step={args.step}°")

    elif args.mode == "gyrz":
        results = sweep_gyrz(
            heading_error_deg = args.heading_error,
            alt_m             = args.alt,
            gyrz_min          = args.gyrz_min,
            gyrz_max          = args.gyrz_max,
            step_deg_s        = args.step,
        )
        if args.out:
            _write_output(results, args.out)
        else:
            print_scenario_table(results,
                title=f"Gyrz sweep  herr={args.heading_error:+.1f}°  alt={args.alt:.0f}m")

    elif args.mode == "altitude":
        results = sweep_altitude(
            heading_error_deg = args.heading_error,
            gyrz_deg_s        = args.gyrz,
            alt_min           = args.alt_min,
            alt_max           = args.alt_max,
            step_m            = args.step,
        )
        if args.out:
            _write_output(results, args.out)
        else:
            print_scenario_table(results,
                title=f"Altitude sweep  herr={args.heading_error:+.1f}°  gyrz={args.gyrz:+.1f}°/s")

    elif args.mode == "flight":
        sim_kw = dict(
            airspeed_mps       = args.speed,
            sink_rate_mps      = args.sink_rate,
            target_bearing_deg = args.target_bearing,
            dt                 = args.dt,
            wind_n_mps         = args.wind_n,
            wind_e_mps         = args.wind_e,
            gust_sigma_deg_s   = args.gust_sigma,
            gust_seed          = args.gust_seed,
        )
        wind_str = (
            f"  wind=({args.wind_n:+.1f}N,{args.wind_e:+.1f}E)m/s"
            f"  gust_sigma={args.gust_sigma}°/s"
        ) if (args.wind_n or args.wind_e or args.gust_sigma) else ""

        if args.multi is not None:
            scenarios = args.multi if args.multi else ["0:200", "30:200", "-60:200", "90:200"]
            parsed = [(float(t.split(":")[0]), float(t.split(":")[1])) for t in scenarios]
            if args.out:
                all_results = []
                for herr, alt in parsed:
                    all_results.extend(simulate_flight(herr, alt, **sim_kw))
                _write_output(all_results, args.out)
            else:
                for herr, alt in parsed:
                    results = simulate_flight(herr, alt, **sim_kw)
                    print_flight_table(
                        results,
                        title=f"herr0={herr:+.0f}°  alt0={alt:.0f}m"
                              f"  bearing={args.target_bearing:.0f}°  "
                              f"airspeed={args.speed}m/s  sink={args.sink_rate}m/s  dt={args.dt}s{wind_str}",
                        dt=args.dt,
                    )
        else:
            results = simulate_flight(
                heading_error_deg = args.heading_error,
                alt_m             = args.alt,
                **sim_kw,
            )
            title = (f"herr0={args.heading_error:+.0f}°  alt0={args.alt:.0f}m"
                     f"  bearing={args.target_bearing:.0f}°  "
                     f"airspeed={args.speed}m/s  sink={args.sink_rate}m/s  dt={args.dt}s{wind_str}")
            if args.out:
                _write_output(results, args.out)
            else:
                print_flight_table(results, title=title, dt=args.dt)


if __name__ == "__main__":
    main()
