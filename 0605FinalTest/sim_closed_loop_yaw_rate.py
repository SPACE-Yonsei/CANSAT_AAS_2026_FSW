from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
ARTIFACT_DIR = Path(__file__).resolve().parent

from Sensor_Motor import control
from Sensor_Motor.guidance import ControlMode
from lib import config


K_CASES = [0.32, 0.63, 1.0]
TAU_CASES = [0.3, 0.5, 0.66]
DISTURBANCE_CASES = [-15.0, 0.0, 15.0]
COMMAND_CASES = [-30.0, -15.0, 15.0, 30.0]
MODE_CASES = [
    ControlMode.GPS_TRACKING_CLOSED,
    ControlMode.DR_M_G_CLOSED,
    ControlMode.DR_PM_G_CLOSED,
]
DT = 0.05
DURATION = 10.0
DEFAULT_OUT = ARTIFACT_DIR / "closed_loop_yaw_rate_results.csv"

FIELDNAMES = [
    "mode",
    "K",
    "tau",
    "disturbance_dps",
    "command_dps",
    "final_yaw_rate_dps",
    "final_error_dps",
    "final_error_ratio",
    "error_ratio_1s",
    "response_time_63_s",
    "max_abs_yaw_rate_dps",
    "saturation_ratio",
    "settling_time_s",
    "diverged",
    "opposite_sign",
    "pass",
    "fail_reason",
]


def _limit_for_mode(mode: ControlMode) -> float:
    if mode == ControlMode.DR_M_G_CLOSED:
        return config.DR_M_G_YAW_RATE_LIMIT_DPS
    if mode == ControlMode.DR_PM_G_CLOSED:
        return config.DR_PM_G_YAW_RATE_LIMIT_DPS
    return config.GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS


def _pid_enabled(mode: ControlMode) -> bool:
    return mode in {
        ControlMode.GPS_TRACKING_CLOSED,
        ControlMode.DR_M_G_CLOSED,
        ControlMode.DR_PM_G_CLOSED,
    }


def _ctrl_input(mode: ControlMode, command_dps: float) -> control.CtrlInput:
    return control.CtrlInput(
        angular_velocity_cmd_deg_s=command_dps,
        ground_speed_mps=5.0,
        valid=True,
        timestamp=0.0,
        pid_enabled=_pid_enabled(mode),
        control_mode=mode,
        yaw_rate_limit_dps=_limit_for_mode(mode),
    )


def simulate_case(
    *,
    mode: ControlMode,
    K: float,
    tau: float,
    disturbance_dps: float,
    command_dps: float,
    dt: float = DT,
    duration: float = DURATION,
) -> dict[str, object]:
    control.reset()
    w = 0.0
    max_abs_w = 0.0
    saturated_steps = 0
    diverged = False
    settling_time = math.nan
    settled_band = max(2.0, abs(command_dps) * 0.10)
    steps = int(duration / dt)
    history: list[tuple[float, float]] = []
    error_ratio_1s = math.nan
    response_time_63 = math.nan

    for step in range(steps):
        now = (step + 1) * dt
        out = control.ProduceCtrlOutput(_ctrl_input(mode, command_dps), w, now)
        u = out.delta_arm_deg
        if out.saturated or abs(u) >= control.DELTA_ARM_MAX_DEG - 1.0e-9:
            saturated_steps += 1
        w_dot = (K * u + disturbance_dps - w) / tau
        w += w_dot * dt
        max_abs_w = max(max_abs_w, abs(w))
        history.append((now, w))
        if not math.isfinite(error_ratio_1s) and now >= 1.0:
            error_ratio_1s = abs(command_dps - w) / max(abs(command_dps), 1.0)
        if (
            not math.isfinite(response_time_63)
            and math.copysign(1.0, w) == math.copysign(1.0, command_dps)
            and abs(w) >= 0.63 * abs(command_dps)
        ):
            response_time_63 = now
        if not math.isfinite(w) or abs(w) > 1000.0:
            diverged = True
            break

    final_error = command_dps - w
    final_error_ratio = abs(final_error) / max(abs(command_dps), 1.0)
    if not math.isfinite(error_ratio_1s):
        error_ratio_1s = final_error_ratio
    saturation_ratio = saturated_steps / max(len(history), 1)
    if history:
        for index, (t, value) in enumerate(history):
            tail = history[index:]
            if all(abs(command_dps - v) <= settled_band for _, v in tail):
                settling_time = t
                break

    opposite_sign = (
        abs(command_dps) > 1.0e-9
        and abs(w) > 1.0
        and math.copysign(1.0, w) != math.copysign(1.0, command_dps)
    )
    fail_reasons = []
    if diverged:
        fail_reasons.append("DIVERGED")
    if final_error_ratio >= 0.40:
        fail_reasons.append("FINAL_ERROR")
    if saturation_ratio >= 0.50:
        fail_reasons.append("SATURATION")
    if opposite_sign:
        fail_reasons.append("OPPOSITE_SIGN")

    return {
        "mode": mode.value,
        "K": K,
        "tau": tau,
        "disturbance_dps": disturbance_dps,
        "command_dps": command_dps,
        "final_yaw_rate_dps": w,
        "final_error_dps": final_error,
        "final_error_ratio": final_error_ratio,
        "error_ratio_1s": error_ratio_1s,
        "response_time_63_s": response_time_63,
        "max_abs_yaw_rate_dps": max_abs_w,
        "saturation_ratio": saturation_ratio,
        "settling_time_s": settling_time,
        "diverged": diverged,
        "opposite_sign": opposite_sign,
        "pass": not fail_reasons,
        "fail_reason": "|".join(fail_reasons),
    }


def run_all_cases() -> list[dict[str, object]]:
    rows = []
    for mode in MODE_CASES:
        for K in K_CASES:
            for tau in TAU_CASES:
                for disturbance in DISTURBANCE_CASES:
                    for command in COMMAND_CASES:
                        rows.append(
                            simulate_case(
                                mode=mode,
                                K=K,
                                tau=tau,
                                disturbance_dps=disturbance,
                                command_dps=command,
                            )
                        )
    return rows


def gps_faster_than_dr_pm(rows: list[dict[str, object]]) -> bool:
    grouped: dict[tuple[float, float, float, float], dict[str, float]] = defaultdict(dict)
    for row in rows:
        if float(row["disturbance_dps"]) != 0.0:
            continue
        key = (
            float(row["K"]),
            float(row["tau"]),
            float(row["disturbance_dps"]),
            float(row["command_dps"]),
        )
        response_time = float(row["response_time_63_s"])
        if not math.isfinite(response_time):
            response_time = DURATION
        grouped[key][str(row["mode"])] = response_time
    comparable = [
        values
        for values in grouped.values()
        if ControlMode.GPS_TRACKING_CLOSED.value in values
        and ControlMode.DR_PM_G_CLOSED.value in values
    ]
    return bool(comparable) and all(
        values[ControlMode.GPS_TRACKING_CLOSED.value]
        <= values[ControlMode.DR_PM_G_CLOSED.value] + 1.0e-9
        for values in comparable
    )


def write_csv(rows: list[dict[str, object]], path: Path) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


def summarize(rows: list[dict[str, object]]) -> dict[str, dict[str, int]]:
    summary: dict[str, dict[str, int]] = {}
    for mode in MODE_CASES:
        mode_rows = [row for row in rows if row["mode"] == mode.value]
        summary[mode.value] = {
            "total": len(mode_rows),
            "pass": sum(1 for row in mode_rows if row["pass"]),
            "fail": sum(1 for row in mode_rows if not row["pass"]),
        }
    return summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args(argv)

    rows = run_all_cases()
    write_csv(rows, args.output)
    summary = summarize(rows)
    all_pass = all(row["pass"] for row in rows)
    gps_fast = gps_faster_than_dr_pm(rows)
    print(f"cases: {len(rows)}")
    for mode, counts in summary.items():
        print(f"{mode}: pass={counts['pass']} fail={counts['fail']} total={counts['total']}")
    print(f"gps_faster_than_dr_pm: {gps_fast}")
    print(f"all_pass: {all_pass and gps_fast}")
    print(f"csv: {args.output}")
    return 0 if all_pass and gps_fast else 1


if __name__ == "__main__":
    raise SystemExit(main())
