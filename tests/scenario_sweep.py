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
    dr_sim: bool = False
    """True = simulate dead-reckoning active: STALE pos_quality is treated as FRESHED
    (mirrors motorapp FillFreshed upgrading GPS-STALE to FRESHED via DR position)."""


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
    left_angle_deg: float
    right_angle_deg: float
    left_pw: int
    right_pw: int


# ── Core ───────────────────────────────────────────────────────────────────────

def _build_l1_input(params: ScenarioParams) -> tuple[L1Input, ControlMode]:
    target_lat, target_lon = _target_latlon(params.track_length_m)
    pq = SensorQuality[params.pos_quality]
    gq = SensorQuality[params.gyrz_quality]

    # DR simulation: motorapp의 FillFreshed()는 GPS가 STALE일 때 DR 위치로
    # pos_quality를 FRESHED로 승격시킨다. --dr-valid 플래그가 켜져 있으면
    # 여기서 동일한 승격을 수행해 실제 비행의 DR 경로를 재현한다.
    if params.dr_sim and pq == SensorQuality.STALE:
        pq = SensorQuality.FRESHED

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
        dr_valid=params.dr_sim,
        freefall=0,
        tumble=0,
    )

    if params.control_mode_override is not None:
        mode = ControlMode(params.control_mode_override)
    else:
        mode, _ = guidance.DecideControlMode(l1)

    return l1, mode


def run_scenario(params: ScenarioParams) -> ScenarioResult:
    """Run the full guidance + control pipeline for one scenario.

    슬루 리미터는 이전 arm 위치에서 얼마나 이동했는지를 보기 때문에
    첫 프레임만 실행하면 항상 neutral(80°)에서 10° 이동으로 제한된다.
    "한 번 입력하면 한 프레임 완료" 원칙에 맞게:
      1단계: guidance 출력으로 원하는 arm 각도를 구한다.
      2단계: arm이 이미 그 위치에 있다고 가정하고 (슬루 상태 주입) 실제 프레임을 실행한다.
    """
    target_lat, target_lon = _target_latlon(params.track_length_m)
    l1_input, mode = _build_l1_input(params)
    now = 0.0
    gyrz_meas = params.gyrz_deg_s

    g_out = guidance.ProduceL1Output(
        l1_input, mode,
        _ORIGIN_LAT, _ORIGIN_LON,
        target_lat, target_lon,
        now,
    )

    ctl = control.MakeCtrler()
    ctrl_in = control.ProduceCtrlInput(g_out, now)

    # 1단계: 원하는 arm 각도 계산
    cmd = control.ProduceCtrlOutput(ctl, ctrl_in, gyrz_meas, now)
    _, _, left_des, right_des, _ = control.ConnectRoMo(cmd.delta_arm_deg)
    ctl.prev_left_angle_deg = left_des
    ctl.prev_right_angle_deg = right_des

    # 2단계: arm이 원하는 위치에서 출발하는 정상 프레임
    cmd = control.ProduceCtrlOutput(
        ctl, ctrl_in, gyrz_meas, now + 1.0 / 20.0
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
        nu_deg=math.degrees(g_out.angle_to_turn),
        cross_track_calc_m=g_out.crossTrack,
        L1_distance_m=g_out.L1_distance,
        lat_acc_cmd_mps2=g_out.lat_acc_cmd_mps2,
        angular_velocity_cmd_deg_s=math.degrees(g_out.yaw_rate_cmd),
        delta_ff_deg=cmd.delta_ff_deg,
        delta_pid_deg=cmd.delta_pid_deg,
        delta_arm_deg=cmd.delta_arm_deg,
        left_angle_deg=cmd.left_angle_deg,
        right_angle_deg=cmd.right_angle_deg,
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


def simulate_trajectory(
    bearing_error_deg: float = 30.0,
    cross_track_m: float = 0.0,
    airspeed_mps: float = 8.0,
    dt: float = 1.0,
    track_length_m: float = 1000.0,
    max_steps: int = 2000,
    wind_n_mps: float = 0.0,
    wind_e_mps: float = 0.0,
    gust_sigma_mps: float = 0.0,
    gust_seed: int = 42,
    pos_quality: str = "FRESH",
    gyrz_quality: str = "FRESH",
    dr_sim: bool = False,
) -> list[ScenarioResult]:
    """항공기 운동학 + 바람 외란을 적용한 폐루프 궤적 시뮬레이션.

    매 dt초마다:
      1. 기체 heading + 바람벡터 → GPS course/groundspeed 계산
      2. guidance+control 실행 → angular_velocity_cmd 획득
      3. yaw rate로 heading 업데이트
      4. 새 ground velocity로 pos_N/pos_E 이동

    wind_n/e_mps: 정상 바람 (북/동 방향 m/s).
    gust_sigma_mps: 매 스텝마다 N(0,σ)으로 두 축에 독립 추가되는 돌풍.
    """
    import random
    rng = random.Random(gust_seed)

    heading_rad = math.radians(bearing_error_deg)
    pos_n = 0.0
    pos_e = cross_track_m
    results = []
    prev_yaw_rate_rad_s = 0.0

    for _ in range(max_steps):
        along = pos_n / track_length_m
        if along >= 0.99:
            break

        # 돌풍: 매 스텝 독립 Gaussian 노이즈
        gust_n = rng.gauss(0.0, gust_sigma_mps) if gust_sigma_mps > 0.0 else 0.0
        gust_e = rng.gauss(0.0, gust_sigma_mps) if gust_sigma_mps > 0.0 else 0.0

        # 대기속도 벡터 (heading 방향) + 바람 → 지상 속도 벡터
        vas_n = airspeed_mps * math.cos(heading_rad)
        vas_e = airspeed_mps * math.sin(heading_rad)
        gnd_n = vas_n + wind_n_mps + gust_n
        gnd_e = vas_e + wind_e_mps + gust_e
        ground_speed = math.hypot(gnd_n, gnd_e)
        course_rad = math.atan2(gnd_e, gnd_n)  # GPS가 보는 course

        params = ScenarioParams(
            bearing_error_deg=math.degrees(course_rad),
            cross_track_m=pos_e,
            along_track_ratio=max(0.0, min(along, 0.999)),
            ground_speed_mps=max(ground_speed, 0.1),
            gyrz_deg_s=math.degrees(prev_yaw_rate_rad_s),
            pos_quality=pos_quality,
            gyrz_quality=gyrz_quality,
            track_length_m=track_length_m,
            dr_sim=dr_sim,
        )
        r = run_scenario(params)
        results.append(r)

        yaw_rate_rad_s = math.radians(r.angular_velocity_cmd_deg_s)
        heading_rad += yaw_rate_rad_s * dt
        heading_rad = (heading_rad + math.pi) % (2 * math.pi) - math.pi

        # 다음 스텝 이동 (바람 포함)
        nxt_n = airspeed_mps * math.cos(heading_rad) + wind_n_mps + gust_n
        nxt_e = airspeed_mps * math.sin(heading_rad) + wind_e_mps + gust_e
        pos_n += nxt_n * dt
        pos_e += nxt_e * dt
        prev_yaw_rate_rad_s = yaw_rate_rad_s

    return results


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


_TABLE_HDR = (
    f"{'t(s)':>5}  {'ratio':>5}  {'bear':>6}  {'xtrack':>8}  "
    f"{'mode':<20}  {'nu':>6}  {'w/s':>6}  "
    f"{'darm':>6}  {'L-arm':>6}  {'R-arm':>6}  {'Lpw':>5}  {'Rpw':>5}"
)
_TABLE_SEP = "-" * len(_TABLE_HDR)

# arm angle convention from control.py ConnectRoMo():
#   left_angle  = NEUTRAL(80) - delta/2   right_angle = NEUTRAL(80) + delta/2
#   delta > 0 (right-turn): L-arm < 80, R-arm > 80
#   delta < 0 (left-turn) : L-arm > 80, R-arm < 80
_ARM_LEGEND = (
    "arm-angle convention (control.ConnectRoMo):\n"
    "  neutral = 80 deg  |  range 0 ~ 160 deg  |  delta = R-arm - L-arm  (= 2 * (R-arm - 80))\n"
    "  RIGHT-turn (delta>0): L-arm < 80  R-arm > 80  |  L-arm decreases  R-arm increases\n"
    "  LEFT-turn  (delta<0): L-arm > 80  R-arm < 80  |  L-arm increases  R-arm decreases\n"
    "  PWM: Lpw = LEFT_ZERO(2480) - L-arm * 11.11   Rpw = RIGHT_ZERO(636) + R-arm * 11.11"
)


def print_table_row(r: ScenarioResult, t_s: float = 0.0) -> None:
    mode_short = r.control_mode.replace("NOMINAL_", "N_").replace("DEGRADED_", "D_")
    print(
        f"{t_s:5.1f}  {r.along_track_ratio:5.2f}  {r.bearing_error_deg:+6.1f}  {r.cross_track_calc_m:+8.1f}  "
        f"{mode_short:<20}  {r.nu_deg:+6.1f}  {r.angular_velocity_cmd_deg_s:+6.1f}  "
        f"{r.delta_arm_deg:+6.1f}  {r.left_angle_deg:6.1f}  {r.right_angle_deg:6.1f}  "
        f"{r.left_pw:5d}  {r.right_pw:5d}"
    )


def print_table(results: list[ScenarioResult], title: str = "", dt: float = 1.0) -> None:
    if title:
        print(f"\n{'-'*len(_TABLE_HDR)}")
        print(title)
        print(f"  col: t=elapsed(s)  ratio=along-track  bear=bearing-err(deg)  xtrack=cross-track(m)")
        print(f"       w/s=ang-vel-cmd(deg/s)  darm=delta-arm(deg)  L/R-arm=servo-angle(deg)  Lpw/Rpw=servo-pw(us)")
    print(_TABLE_HDR)
    print(_TABLE_SEP)
    for i, r in enumerate(results):
        print_table_row(r, t_s=i * dt)
    print(_TABLE_SEP)
    print(_ARM_LEGEND)


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
    print(f"    lat_acc_cmd       : {r.lat_acc_cmd_mps2:+.4f} m/s^2")
    print(f"    angular_vel_cmd   : {r.angular_velocity_cmd_deg_s:+.2f} deg/s")
    print("  [Control Output]")
    print(f"    delta_ff          : {r.delta_ff_deg:+.2f} deg")
    print(f"    delta_pid         : {r.delta_pid_deg:+.2f} deg")
    print(f"    delta_arm         : {r.delta_arm_deg:+.2f} deg")
    print(f"    left_angle        : {r.left_angle_deg:.2f} deg  (neutral=80 deg, range 0~160 deg)")
    print(f"    right_angle       : {r.right_angle_deg:.2f} deg  (neutral=80 deg, range 0~160 deg)")
    print(f"    left_pw           : {r.left_pw} us")
    print(f"    right_pw          : {r.right_pw} us")


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
    s.add_argument("--dr-valid", action="store_true",
                   help="GPS STALE 상태에서 DR이 위치를 복원하는 실제 비행 경로 시뮬레이션 (STALE→FRESHED)")
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

    # flight ──────────────────────────────────────────────────────────────────
    fl = sub.add_parser(
        "flight",
        help="폐루프 궤적 시뮬레이션 (분리점→목표). 바람/돌풍 외란 지원.",
    )
    fl.add_argument("--bearing-error", type=float, default=30.0, metavar="DEG",
                    help="초기 bearing 오차 (deg)")
    fl.add_argument("--cross-track",   type=float, default=0.0,  metavar="M",
                    help="초기 cross-track 오차 (m)")
    fl.add_argument("--speed",         type=float, default=8.0,  metavar="MPS",
                    help="대기속도 (m/s)")
    fl.add_argument("--dt",            type=float, default=1.0,  metavar="SEC",
                    help="시뮬레이션 타임스텝 (s, 기본 1s)")
    fl.add_argument("--track-length",  type=float, default=1000.0, metavar="M")
    fl.add_argument("--wind-n",        type=float, default=0.0,  metavar="MPS",
                    help="정상 바람 북향 성분 (m/s, 양수=북)")
    fl.add_argument("--wind-e",        type=float, default=0.0,  metavar="MPS",
                    help="정상 바람 동향 성분 (m/s, 양수=동)")
    fl.add_argument("--gust-sigma",    type=float, default=0.0,  metavar="MPS",
                    help="돌풍 표준편차 (m/s). 매 스텝 N(0,σ) 노이즈 추가")
    fl.add_argument("--gust-seed",     type=int,   default=42,   metavar="INT")
    fl.add_argument(
        "--multi",
        nargs="*",
        metavar="BEAR:XTRACK",
        help="여러 초기조건을 순서대로 출력. 예: --multi 0:0 30:0 -30:100",
    )
    fl.add_argument("--out", default=None, metavar="FILE")
    fl.add_argument("--dr-valid", action="store_true",
                   help="GPS STALE 상태에서 DR이 위치를 복원하는 실제 비행 경로 시뮬레이션 (STALE→FRESHED)")
    _add_quality_args(fl)

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
            dr_sim=args.dr_valid,
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

    elif args.mode == "flight":
        sim_kw = dict(
            airspeed_mps=args.speed,
            dt=args.dt,
            track_length_m=args.track_length,
            wind_n_mps=args.wind_n,
            wind_e_mps=args.wind_e,
            gust_sigma_mps=args.gust_sigma,
            gust_seed=args.gust_seed,
            pos_quality=args.pos_quality,
            gyrz_quality=args.gyrz_quality,
            dr_sim=args.dr_valid,
        )
        if args.multi is not None:
            scenarios = args.multi if args.multi else ["0:0", "30:0", "30:100", "-30:-100"]
            parsed = [(float(t.split(":")[0]), float(t.split(":")[1])) for t in scenarios]
            if args.out:
                all_results = []
                for bear, xtrack in parsed:
                    all_results.extend(simulate_trajectory(bear, xtrack, **sim_kw))
                with open(args.out, "w", newline="") as f:
                    results_to_csv(all_results, f)
                print(f"Saved {len(all_results)} rows → {args.out}", file=sys.stderr)
            else:
                for bear, xtrack in parsed:
                    results = simulate_trajectory(bear, xtrack, **sim_kw)
                    wind_str = (f"  wind=({args.wind_n:+.1f}N,{args.wind_e:+.1f}E) m/s"
                                f"  gust_sigma={args.gust_sigma} m/s") if (args.wind_n or args.wind_e or args.gust_sigma) else ""
                    print_table(
                        results,
                        title=(f"bear0={bear:+.0f} deg  xtrack0={xtrack:+.0f} m"
                               f"  airspeed={args.speed} m/s  dt={args.dt}s{wind_str}"),
                        dt=args.dt,
                    )
        else:
            results = simulate_trajectory(
                bearing_error_deg=args.bearing_error,
                cross_track_m=args.cross_track,
                **sim_kw,
            )
            wind_str = (f"  wind=({args.wind_n:+.1f}N,{args.wind_e:+.1f}E) m/s"
                        f"  gust_sigma={args.gust_sigma} m/s") if (args.wind_n or args.wind_e or args.gust_sigma) else ""
            title = (f"bear0={args.bearing_error:+.0f} deg  xtrack0={args.cross_track:+.0f} m"
                     f"  airspeed={args.speed} m/s  dt={args.dt}s{wind_str}")
            if args.out:
                with open(args.out, "w", newline="") as f:
                    results_to_csv(results, f)
                print(f"Saved {len(results)} rows → {args.out}", file=sys.stderr)
            else:
                print_table(results, title=title, dt=args.dt)


if __name__ == "__main__":
    main()
