from __future__ import annotations

import csv
import importlib
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from Sensor_Motor import control, guidance  # noqa: E402
from Sensor_Motor.sensor_types import _BaroFromApp, _GpsFromApp, _ImuFromApp  # noqa: E402
from lib import config  # noqa: E402


ORIGIN_LAT = 37.0
ORIGIN_LON = 127.0
TARGET_E = 80.0
TARGET_N = 20.0
BASE_T = 1000.0
ANCHOR_T = 1000.0
DR_START_T = 1005.2
DT = 0.2
TRUTH_V = 4.0
TRUTH_COURSE = math.radians(90.0)
CSV_PATH = ROOT / "trace_dr_accumulation_current.csv"
DOC_PATH = ROOT / "docs" / "dr_accumulation_trace_current.md"


CSV_COLUMNS = [
    "scenario",
    "cycle",
    "t",
    "mode",
    "fail_reason",
    "gps_pos_fresh",
    "gps_motion_fresh",
    "imu_gyrz_fresh",
    "imu_yaw_fresh",
    "baro_sink_fresh",
    "acc_fresh",
    "nav_valid",
    "nav_E",
    "nav_N",
    "nav_V",
    "nav_course_deg",
    "dr_current_E",
    "dr_current_N",
    "dr_confidence",
    "dr_age_s",
    "dr_method",
    "l1_valid",
    "l1_reason",
    "l1out_valid",
    "l1out_reason",
    "yaw_rate_cmd_dps",
    "yaw_rate_limit_dps",
    "ctrl_valid",
    "ctrl_reason",
    "delta_arm_deg",
    "produce_l1input_mutated_dr",
    "decide_calls",
    "truth_E",
    "truth_N",
    "truth_course_deg",
    "truth_V",
    "est_distance_to_target",
    "truth_distance_to_target",
    "distance_error_m",
    "target_bearing_est_deg",
    "target_bearing_truth_deg",
    "nu_est_deg",
    "nu_truth_deg",
    "abs_nu_est_deg",
    "abs_nu_truth_deg",
    "closing_rate_est_mps",
    "closing_rate_truth_mps",
    "distance_decreasing_est",
    "distance_decreasing_truth",
    "yaw_cmd_sign_correct_est",
    "yaw_cmd_sign_correct_truth",
    "target_pointing_ok",
    "baro_sink_spike",
    "speed_clamped",
    "dr_speed_source",
]


@dataclass
class TruthState:
    E: float = 0.0
    N: float = 0.0
    course: float = TRUTH_COURSE
    V: float = TRUTH_V

    def step(self, dt: float) -> None:
        self.E += self.V * math.sin(self.course) * dt
        self.N += self.V * math.cos(self.course) * dt


def reload_modules() -> None:
    global guidance, control
    guidance = importlib.reload(guidance)
    control = importlib.reload(control)
    control.reset()


def finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def deg(rad: Any) -> float:
    try:
        value = float(rad)
    except (TypeError, ValueError):
        return float("nan")
    return math.degrees(value) if math.isfinite(value) else float("nan")


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def latlon_from_ne(n_m: float, e_m: float) -> tuple[float, float]:
    lat = ORIGIN_LAT + math.degrees(n_m / guidance.EARTH_RADIUS_M)
    lon = ORIGIN_LON + math.degrees(
        e_m / (guidance.EARTH_RADIUS_M * math.cos(math.radians(ORIGIN_LAT)))
    )
    return lat, lon


def configure_mission() -> None:
    guidance.reset()
    guidance.set_origin_point(ORIGIN_LAT, ORIGIN_LON)
    target_lat, target_lon = latlon_from_ne(TARGET_N, TARGET_E)
    guidance.set_target_point(target_lat, target_lon)


def gps_at(
    now: float,
    truth: TruthState,
    *,
    pos: bool,
    motion: bool,
    speed: float | None = None,
    course: float | None = None,
) -> _GpsFromApp:
    lat, lon = latlon_from_ne(truth.N, truth.E)
    return _GpsFromApp(
        lat=lat,
        lon=lon,
        course_rad=truth.course if course is None else course,
        speed_mps=truth.V if speed is None else speed,
        pos_ts=now if pos else None,
        motion_ts=now if motion else None,
        pos_health=1 if pos else 0,
        motion_health=1 if motion else 0,
    )


def imu_at(
    now: float,
    *,
    gyro: bool = True,
    yaw: bool = True,
    acc: bool = True,
    gyro_spike: bool = False,
    yaw_value: float = TRUTH_COURSE,
) -> _ImuFromApp:
    limit = getattr(config, "DR_MAX_YAW_RATE_DPS_FOR_CONTROL", 120.0)
    gyrz_dps = limit + 30.0 if gyro_spike else 0.0
    return _ImuFromApp(
        yaw_rad=yaw_value if yaw else None,
        gyrz_rad_s=math.radians(gyrz_dps) if gyro else None,
        ts=now,
        lin_acc_x=0.0 if acc else None,
        lin_acc_y=0.0 if acc else None,
        lin_acc_valid=acc,
        health=1 if (gyro or yaw or acc) else 0,
    )


def baro_at(now: float, *, sink: float | None = 2.0, spike: bool = False) -> _BaroFromApp:
    if sink is None:
        return _BaroFromApp(alt_m=120.0, sink_rate=None, rx_ts=now, health=0)
    raw_sink = getattr(config, "DR_BARO_SINK_MAX_MPS", 6.0) + 2.0 if spike else sink
    return _BaroFromApp(alt_m=120.0, sink_rate=raw_sink, rx_ts=now, health=1)


def seed_anchor(truth: TruthState | None = None, *, t: float = ANCHOR_T, old: bool = False) -> None:
    if truth is None:
        truth = TruthState()
    anchor_t = t - config.DR_MAX_AGE_S - 1.0 if old else t
    mode = guidance.DecideControlMode(
        gps_at(anchor_t, truth, pos=True, motion=True, speed=truth.V, course=truth.course),
        imu_at(anchor_t, gyro=True, yaw=True, acc=False, yaw_value=truth.course),
        baro_at(anchor_t, sink=2.0),
        anchor_t,
    )
    if not old and mode != guidance.ControlMode.GPS_TRACKING_CLOSED:
        raise AssertionError(f"GPS anchor lock failed: {mode}")


def distance_metrics(E: float, N: float, course: float, V: float) -> dict[str, float]:
    if not (finite(E) and finite(N) and finite(course) and finite(V)):
        return {
            "distance": float("nan"),
            "bearing": float("nan"),
            "nu": float("nan"),
            "closing_rate": float("nan"),
        }
    dE = TARGET_E - E
    dN = TARGET_N - N
    distance = math.hypot(dE, dN)
    bearing = math.atan2(dE, dN)
    nu = wrap_pi(bearing - course)
    if distance > 1.0e-9:
        vE = V * math.sin(course)
        vN = V * math.cos(course)
        closing = (vE * dE + vN * dN) / distance
    else:
        closing = 0.0
    return {"distance": distance, "bearing": bearing, "nu": nu, "closing_rate": closing}


def sign_correct(cmd_rad_s: float, nu: float) -> bool:
    if not finite(cmd_rad_s) or not finite(nu):
        return False
    if abs(math.degrees(nu)) < config.NU_DEADBAND_DEG:
        return abs(cmd_rad_s) <= 1.0e-12
    if abs(cmd_rad_s) <= 1.0e-12:
        return False
    return math.copysign(1.0, cmd_rad_s) == math.copysign(1.0, nu)


def cycle(
    scenario: str,
    cycle_idx: int,
    now: float,
    truth: TruthState,
    *,
    gps: _GpsFromApp | None,
    imu: _ImuFromApp | None,
    baro: _BaroFromApp | None,
    prev_est_dist: float | None,
    prev_truth_dist: float | None,
) -> tuple[dict[str, Any], float, float]:
    decide_calls = 0
    mode = guidance.DecideControlMode(gps, imu, baro, now)
    decide_calls += 1
    dr_before_l1 = (guidance._STATE_t.dr.current_E, guidance._STATE_t.dr.current_N)
    l1_in = guidance.ProduceL1Input(now)
    dr_after_l1 = (guidance._STATE_t.dr.current_E, guidance._STATE_t.dr.current_N)
    l1_out = guidance.ProduceL1Output(l1_in)
    if l1_out.valid:
        ctrl_in = control.ProduceCtrlInput(l1_out, now)
        gyrz = guidance._STATE_t.imu.gyr_z
        gyrz_dps = math.degrees(gyrz) if finite(gyrz) else float("nan")
        ctrl_out = control.ProduceCtrlOutput(ctrl_in, gyrz_dps, now)
    else:
        ctrl_out = control.WriteNeutral(now, mode)
        ctrl_out.reason = l1_out.reason

    st = guidance._STATE_t
    nav = st.nav
    dr = st.dr
    est = distance_metrics(nav.E, nav.N, nav.course, nav.V)
    truth_m = distance_metrics(truth.E, truth.N, truth.course, truth.V)
    est_dist = est["distance"]
    truth_dist = truth_m["distance"]
    row = {
        "scenario": scenario,
        "cycle": cycle_idx,
        "t": now,
        "mode": mode.value,
        "fail_reason": nav.fail_reason,
        "gps_pos_fresh": st.flags.gps_pos_fresh,
        "gps_motion_fresh": st.flags.gps_motion_fresh,
        "imu_gyrz_fresh": st.flags.imu_gyrz_fresh,
        "imu_yaw_fresh": st.flags.imu_yaw_fresh,
        "baro_sink_fresh": st.flags.baro_sink_fresh,
        "acc_fresh": st.flags.acc_fresh,
        "nav_valid": nav.valid,
        "nav_E": nav.E,
        "nav_N": nav.N,
        "nav_V": nav.V,
        "nav_course_deg": deg(nav.course),
        "dr_current_E": dr.current_E,
        "dr_current_N": dr.current_N,
        "dr_confidence": dr.confidence,
        "dr_age_s": now - dr.anchor_time if finite(dr.anchor_time) else float("nan"),
        "dr_method": dr.method.value,
        "l1_valid": l1_in.valid,
        "l1_reason": l1_in.reason,
        "l1out_valid": l1_out.valid,
        "l1out_reason": l1_out.reason,
        "yaw_rate_cmd_dps": math.degrees(l1_out.yaw_rate_cmd),
        "yaw_rate_limit_dps": l1_out.yaw_rate_limit_dps,
        "ctrl_valid": ctrl_out.valid,
        "ctrl_reason": ctrl_out.reason,
        "delta_arm_deg": ctrl_out.delta_arm_deg,
        "produce_l1input_mutated_dr": dr_after_l1 != dr_before_l1,
        "decide_calls": decide_calls,
        "truth_E": truth.E,
        "truth_N": truth.N,
        "truth_course_deg": deg(truth.course),
        "truth_V": truth.V,
        "est_distance_to_target": est_dist,
        "truth_distance_to_target": truth_dist,
        "distance_error_m": est_dist - truth_dist if finite(est_dist) and finite(truth_dist) else float("nan"),
        "target_bearing_est_deg": deg(est["bearing"]),
        "target_bearing_truth_deg": deg(truth_m["bearing"]),
        "nu_est_deg": deg(est["nu"]),
        "nu_truth_deg": deg(truth_m["nu"]),
        "abs_nu_est_deg": abs(deg(est["nu"])) if finite(est["nu"]) else float("nan"),
        "abs_nu_truth_deg": abs(deg(truth_m["nu"])) if finite(truth_m["nu"]) else float("nan"),
        "closing_rate_est_mps": est["closing_rate"],
        "closing_rate_truth_mps": truth_m["closing_rate"],
        "distance_decreasing_est": bool(prev_est_dist is not None and finite(est_dist) and est_dist < prev_est_dist),
        "distance_decreasing_truth": bool(prev_truth_dist is not None and finite(truth_dist) and truth_dist < prev_truth_dist),
        "yaw_cmd_sign_correct_est": sign_correct(l1_out.yaw_rate_cmd, est["nu"]),
        "yaw_cmd_sign_correct_truth": sign_correct(l1_out.yaw_rate_cmd, truth_m["nu"]),
        "target_pointing_ok": bool(not finite(truth_m["nu"]) or abs(deg(truth_m["nu"])) <= 120.0),
        "baro_sink_spike": nav.baro_sink_spike,
        "speed_clamped": nav.speed_clamped,
        "dr_speed_source": nav.dr_speed_source,
    }
    return row, est_dist, truth_dist


def start_scenario() -> TruthState:
    reload_modules()
    configure_mission()
    truth = TruthState()
    seed_anchor(truth)
    return truth


def run_a_dr_m_gba() -> list[dict[str, Any]]:
    truth = start_scenario()
    truth.step(DR_START_T - ANCHOR_T)
    row, _, _ = cycle(
        "A_GPS_TO_DR_M_GBA",
        0,
        DR_START_T,
        truth,
        gps=gps_at(DR_START_T, truth, pos=True, motion=False),
        imu=imu_at(DR_START_T, gyro=True, yaw=True, acc=True),
        baro=baro_at(DR_START_T, sink=2.0),
        prev_est_dist=None,
        prev_truth_dist=None,
    )
    return [row]


def run_dr_pm_series(
    scenario: str,
    *,
    gyro: bool,
    yaw: bool,
    baro_sink: float | None,
    acc: bool,
    gyro_spike: bool = False,
    steps: int = 8,
) -> list[dict[str, Any]]:
    truth = start_scenario()
    rows = []
    prev_est = prev_truth = None
    for i in range(steps):
        now = DR_START_T + i * DT
        if i == 0:
            truth.step(DR_START_T - ANCHOR_T)
        elif i > 0:
            truth.step(DT)
        row, prev_est, prev_truth = cycle(
            scenario,
            i,
            now,
            truth,
            gps=None,
            imu=imu_at(now, gyro=gyro, yaw=yaw, acc=acc, gyro_spike=gyro_spike),
            baro=baro_at(now, sink=baro_sink),
            prev_est_dist=prev_est,
            prev_truth_dist=prev_truth,
        )
        rows.append(row)
    return rows


def run_e_timeout() -> list[dict[str, Any]]:
    reload_modules()
    configure_mission()
    truth = TruthState()
    seed_anchor(truth, t=DR_START_T, old=True)
    row, _, _ = cycle(
        "E_DR_TIMEOUT",
        0,
        DR_START_T,
        truth,
        gps=None,
        imu=imu_at(DR_START_T, gyro=True, yaw=True, acc=True),
        baro=baro_at(DR_START_T, sink=2.0),
        prev_est_dist=None,
        prev_truth_dist=None,
    )
    return [row]


def run_f_position_jump() -> list[dict[str, Any]]:
    truth = start_scenario()
    before = (guidance._STATE_t.dr.current_E, guidance._STATE_t.dr.current_N)
    truth.step(DR_START_T - ANCHOR_T)
    row, _, _ = cycle(
        "F_DR_POSITION_JUMP",
        0,
        DR_START_T,
        truth,
        gps=None,
        imu=imu_at(DR_START_T, gyro=True, yaw=False, acc=False),
        baro=baro_at(DR_START_T, sink=3.0),
        prev_est_dist=None,
        prev_truth_dist=None,
    )
    after = (guidance._STATE_t.dr.current_E, guidance._STATE_t.dr.current_N)
    row["jump_current_unchanged"] = before == after
    return [row]


def run_g_baro_spike() -> list[dict[str, Any]]:
    truth = start_scenario()
    truth.step(DR_START_T - ANCHOR_T)
    row, _, _ = cycle(
        "G_BARO_SPIKE_DEGRADATION",
        0,
        DR_START_T,
        truth,
        gps=gps_at(DR_START_T, truth, pos=True, motion=False),
        imu=imu_at(DR_START_T, gyro=True, yaw=True, acc=True),
        baro=baro_at(DR_START_T, sink=2.0, spike=True),
        prev_est_dist=None,
        prev_truth_dist=None,
    )
    return [row]


def run_h_gyro_spike() -> list[dict[str, Any]]:
    return run_dr_pm_series(
        "H_GYRO_SPIKE_DEGRADATION",
        gyro=True,
        yaw=True,
        baro_sink=2.0,
        acc=True,
        gyro_spike=True,
        steps=1,
    )


def run_i_target_pointing_split() -> list[dict[str, Any]]:
    return run_dr_pm_series(
        "I_TARGET_POINTING_TRUTH_NAV_SPLIT",
        gyro=True,
        yaw=True,
        baro_sink=2.0,
        acc=True,
        steps=51,
    )


def run_all_scenarios() -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    rows.extend(run_a_dr_m_gba())
    rows.extend(run_dr_pm_series("B_GPS_TO_DR_PM_GBA", gyro=True, yaw=True, baro_sink=2.0, acc=True, steps=8))
    rows.extend(run_dr_pm_series("C_DR_PM_G_FALLBACK", gyro=True, yaw=False, baro_sink=None, acc=False, steps=8))
    rows.extend(run_dr_pm_series("D_DR_PM_YBA_OPEN", gyro=False, yaw=True, baro_sink=2.0, acc=True, steps=8))
    rows.extend(run_e_timeout())
    rows.extend(run_f_position_jump())
    rows.extend(run_g_baro_spike())
    rows.extend(run_h_gyro_spike())
    rows.extend(run_i_target_pointing_split())
    return rows


def write_csv(rows: list[dict[str, Any]], path: Path = CSV_PATH) -> None:
    extras = sorted({key for row in rows for key in row.keys()} - set(CSV_COLUMNS))
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS + extras, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def scenario_rows(rows: list[dict[str, Any]], scenario: str) -> list[dict[str, Any]]:
    return [row for row in rows if row["scenario"] == scenario]


def ratio(rows: list[dict[str, Any]], key: str) -> float:
    if not rows:
        return float("nan")
    return sum(1 for row in rows if row[key]) / len(rows)


def write_doc(rows: list[dict[str, Any]], path: Path = DOC_PATH) -> None:
    scenario_names = list(dict.fromkeys(row["scenario"] for row in rows))
    lines = [
        "# DR accumulation trace current",
        "",
        "Generated by `python tools/trace_dr_accumulation_current.py`.",
        "",
        "Each cycle uses the current guidance pipeline:",
        "",
        "1. `DecideControlMode(gps, imu, baro, now)` exactly once",
        "2. `ProduceL1Input(now)` as a read/convert step",
        "3. `ProduceL1Output(l1_in)`",
        "4. control output or neutral",
        "",
        "The trace records truth position separately from `guidance._STATE_t.nav`",
        "so target pointing can be compared in estimated and truth frames.",
        "",
        "## Scenario Summary",
        "",
        "| Scenario | Rows | Modes |",
        "| --- | ---: | --- |",
    ]
    for name in scenario_names:
        subset = scenario_rows(rows, name)
        modes = ", ".join(sorted({row["mode"] for row in subset}))
        lines.append(f"| `{name}` | {len(subset)} | `{modes}` |")

    pointing = scenario_rows(rows, "I_TARGET_POINTING_TRUTH_NAV_SPLIT")
    early_5s = [row for row in pointing if row["t"] - DR_START_T <= 5.0 and row["cycle"] > 0]
    early_10s = [row for row in pointing if row["t"] - DR_START_T <= 10.0]
    lines.extend([
        "",
        "## Target Pointing Metrics",
        "",
        f"- Truth distance decreasing ratio, initial 0-5 s: {ratio(early_5s, 'distance_decreasing_truth'):.3f}",
        f"- Truth yaw command sign-correct ratio, initial 0-10 s: {ratio(early_10s, 'yaw_cmd_sign_correct_truth'):.3f}",
        f"- Any `abs_nu_truth_deg > 120` rows: {sum(1 for row in pointing if row['abs_nu_truth_deg'] > 120.0)}",
        "",
        f"CSV: `{CSV_PATH.name}`",
        "",
    ])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    rows = run_all_scenarios()
    write_csv(rows)
    write_doc(rows)
    print(f"rows: {len(rows)}")
    for scenario in dict.fromkeys(row["scenario"] for row in rows):
        subset = scenario_rows(rows, scenario)
        modes = ", ".join(sorted({row["mode"] for row in subset}))
        print(f"{scenario}: rows={len(subset)} modes={modes}")
    print(f"csv: {CSV_PATH}")
    print(f"docs: {DOC_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
