from __future__ import annotations

import argparse
import csv
import importlib
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from Sensor_Motor import control, guidance  # noqa: E402
from lib import config  # noqa: E402


K_CASES = [0.32, 0.63, 1.0]
TAU_CASES = [0.3, 0.5, 0.66]
DISTURBANCE_CASES = [-15.0, 0.0, 15.0]
COMMAND_CASES = [-30.0, -15.0, 15.0, 30.0]
MODE_CASES = [
    guidance.ControlMode.GPS_TRACKING_CLOSED,
    guidance.ControlMode.DR_M_G_CLOSED,
    guidance.ControlMode.DR_PM_G_CLOSED,
]

DT = 0.05
DURATION = 10.0
V_FOR_GEOMETRY_MPS = 6.5
TARGET_DISTANCE_M = 100.0
DEFAULT_OUT = ROOT / "closed_loop_yaw_rate_current.csv"

FIELDNAMES = [
    "mode",
    "K",
    "tau",
    "disturbance_dps",
    "command_dps",
    "l1_yaw_rate_cmd_dps",
    "l1_yaw_rate_limit_dps",
    "ctrl_in_yaw_rate_limit_dps",
    "ctrl_out_yaw_rate_limit_dps",
    "limit_transferred",
    "delta_ff_deg",
    "expected_delta_ff_deg",
    "ff_uses_mode_limit",
    "ff_scale",
    "final_yaw_rate_dps",
    "final_error_dps",
    "final_error_abs_limit_dps",
    "final_error_ratio",
    "max_abs_yaw_rate_dps",
    "saturation_ratio",
    "response_time_63_s",
    "settling_time_s",
    "diverged",
    "opposite_sign",
    "pass",
    "fail_reason",
]


def _yaw_rate_limit_for_mode(mode: guidance.ControlMode) -> float:
    l1 = guidance.L1Input(
        valid=False,
        reason="LIMIT_PROBE",
        control_mode=mode,
    )
    return guidance.ProduceL1Output(l1).yaw_rate_limit_dps


def _pid_enabled(mode: guidance.ControlMode) -> bool:
    return mode.value.endswith("_CLOSED")


def _target_for_command(command_dps: float, speed_mps: float) -> tuple[float, float, float]:
    cmd_rad = math.radians(command_dps)
    denom = 2.0 * speed_mps / config.L_GAIN_M
    ratio = max(-1.0, min(1.0, cmd_rad / denom))
    nu = math.asin(ratio)
    target_E = TARGET_DISTANCE_M * math.sin(nu)
    target_N = TARGET_DISTANCE_M * math.cos(nu)
    return target_E, target_N, nu


def make_l1input(mode: guidance.ControlMode, command_dps: float) -> guidance.L1Input:
    target_E, target_N, _ = _target_for_command(command_dps, V_FOR_GEOMETRY_MPS)
    return guidance.L1Input(
        valid=True,
        reason="GPS_NAV" if mode == guidance.ControlMode.GPS_TRACKING_CLOSED else mode.value,
        control_mode=mode,
        dr_method=guidance.DRMethod.NONE
        if mode == guidance.ControlMode.GPS_TRACKING_CLOSED
        else guidance.DRMethod.GYRO_INTEGRATION,
        confidence=1.0,
        E=0.0,
        N=0.0,
        V=V_FOR_GEOMETRY_MPS,
        course=0.0,
        vE=0.0,
        vN=V_FOR_GEOMETRY_MPS,
        target_E=target_E,
        target_N=target_N,
    )


def _ff_scale_for_mode(mode: guidance.ControlMode) -> float:
    if mode == guidance.ControlMode.DR_PM_G_CLOSED:
        return getattr(config, "DR_PM_FF_SCALE", 0.6)
    if mode == guidance.ControlMode.DR_M_G_CLOSED:
        return getattr(config, "DR_M_FF_SCALE", 0.8)
    return 1.0


def _is_close(a: float, b: float, tol: float = 1.0e-9) -> bool:
    return math.isclose(float(a), float(b), rel_tol=0.0, abs_tol=tol)


def simulate_case(
    *,
    mode: guidance.ControlMode,
    K: float,
    tau: float,
    disturbance_dps: float,
    command_dps: float,
    dt: float = DT,
    duration: float = DURATION,
) -> dict[str, Any]:
    control.reset()
    l1_in = make_l1input(mode, command_dps)
    l1_out = guidance.ProduceL1Output(l1_in)
    ctrl_in_probe = control.ProduceCtrlInput(l1_out, 0.0)

    expected_limit = _yaw_rate_limit_for_mode(mode)
    limit_transferred = (
        _is_close(l1_out.yaw_rate_limit_dps, expected_limit)
        and _is_close(ctrl_in_probe.yaw_rate_limit_dps, l1_out.yaw_rate_limit_dps)
    )

    w = 0.0
    max_abs_w = 0.0
    saturated_steps = 0
    diverged = False
    response_time_63 = math.nan
    settling_time = math.nan
    settled_band = max(2.0, abs(command_dps) * 0.10)
    history: list[tuple[float, float]] = []
    first_out: control.CtrlOutput | None = None

    for step in range(int(duration / dt)):
        now = (step + 1) * dt
        l1_out = guidance.ProduceL1Output(l1_in)
        ctrl_in = control.ProduceCtrlInput(l1_out, now)
        ctrl_out = control.ProduceCtrlOutput(ctrl_in, w, now)
        if first_out is None:
            first_out = ctrl_out

        u = ctrl_out.delta_arm_deg
        if ctrl_out.saturated or abs(u) >= control.DELTA_ARM_MAX_DEG - 1.0e-9:
            saturated_steps += 1

        w_dot = (K * u + disturbance_dps - w) / tau
        w += w_dot * dt
        max_abs_w = max(max_abs_w, abs(w))
        history.append((now, w))

        if (
            not math.isfinite(response_time_63)
            and abs(command_dps) > 1.0e-9
            and math.copysign(1.0, w) == math.copysign(1.0, command_dps)
            and abs(w) >= 0.63 * abs(command_dps)
        ):
            response_time_63 = now
        if not math.isfinite(w) or abs(w) > 1000.0:
            diverged = True
            break

    final_error = command_dps - w
    final_error_ratio = abs(final_error) / max(abs(command_dps), 1.0)
    final_error_abs_limit = max(20.0, 2.0 * abs(command_dps))
    saturation_ratio = saturated_steps / max(len(history), 1)

    if history:
        for index, (t, _) in enumerate(history):
            if all(abs(command_dps - value) <= settled_band for _, value in history[index:]):
                settling_time = t
                break

    opposite_sign = (
        abs(command_dps) > 1.0e-9
        and abs(w) > 1.0
        and math.copysign(1.0, w) != math.copysign(1.0, command_dps)
    )

    ctrl_out_probe = first_out if first_out is not None else control.CtrlOutput()
    expected_delta_ff = (
        control.angular_velocity_to_delta_ff(command_dps, expected_limit)
        * _ff_scale_for_mode(mode)
    )
    gps_fallback_delta_ff = (
        control.angular_velocity_to_delta_ff(
            command_dps,
            config.GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS,
        )
        * _ff_scale_for_mode(mode)
    )
    ff_uses_mode_limit = _is_close(ctrl_out_probe.delta_ff_deg, expected_delta_ff, tol=1.0e-6)
    if mode != guidance.ControlMode.GPS_TRACKING_CLOSED and not _is_close(expected_delta_ff, gps_fallback_delta_ff):
        ff_uses_mode_limit = ff_uses_mode_limit and not _is_close(
            ctrl_out_probe.delta_ff_deg,
            gps_fallback_delta_ff,
            tol=1.0e-6,
        )

    fail_reasons: list[str] = []
    if not l1_out.valid:
        fail_reasons.append(f"L1_INVALID:{l1_out.reason}")
    if not limit_transferred:
        fail_reasons.append("LIMIT_NOT_TRANSFERRED")
    if not _is_close(ctrl_out_probe.yaw_rate_limit_dps, l1_out.yaw_rate_limit_dps):
        fail_reasons.append("CTRL_OUT_LIMIT_MISMATCH")
    if not ff_uses_mode_limit:
        fail_reasons.append("FF_LIMIT_NORMALIZATION")
    if diverged:
        fail_reasons.append("DIVERGED")
    if opposite_sign:
        fail_reasons.append("OPPOSITE_SIGN")
    if abs(final_error) > final_error_abs_limit:
        fail_reasons.append("FINAL_ERROR")
    if saturation_ratio >= 0.50:
        fail_reasons.append("SATURATION")

    return {
        "mode": mode.value,
        "K": K,
        "tau": tau,
        "disturbance_dps": disturbance_dps,
        "command_dps": command_dps,
        "l1_yaw_rate_cmd_dps": math.degrees(l1_out.yaw_rate_cmd),
        "l1_yaw_rate_limit_dps": l1_out.yaw_rate_limit_dps,
        "ctrl_in_yaw_rate_limit_dps": ctrl_in_probe.yaw_rate_limit_dps,
        "ctrl_out_yaw_rate_limit_dps": ctrl_out_probe.yaw_rate_limit_dps,
        "limit_transferred": limit_transferred and _is_close(ctrl_out_probe.yaw_rate_limit_dps, l1_out.yaw_rate_limit_dps),
        "delta_ff_deg": ctrl_out_probe.delta_ff_deg,
        "expected_delta_ff_deg": expected_delta_ff,
        "ff_uses_mode_limit": ff_uses_mode_limit,
        "ff_scale": ctrl_out_probe.ff_scale,
        "final_yaw_rate_dps": w,
        "final_error_dps": final_error,
        "final_error_abs_limit_dps": final_error_abs_limit,
        "final_error_ratio": final_error_ratio,
        "max_abs_yaw_rate_dps": max_abs_w,
        "saturation_ratio": saturation_ratio,
        "response_time_63_s": response_time_63,
        "settling_time_s": settling_time,
        "diverged": diverged,
        "opposite_sign": opposite_sign,
        "pass": not fail_reasons,
        "fail_reason": "|".join(fail_reasons),
    }


def run_all_cases() -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
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


def gps_not_slower_than_dr_pm(rows: list[dict[str, Any]]) -> bool:
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
        rt = float(row["response_time_63_s"])
        if not math.isfinite(rt):
            rt = DURATION
        grouped[key][str(row["mode"])] = rt
    comparable = [
        values for values in grouped.values()
        if guidance.ControlMode.GPS_TRACKING_CLOSED.value in values
        and guidance.ControlMode.DR_PM_G_CLOSED.value in values
    ]
    return bool(comparable) and all(
        values[guidance.ControlMode.GPS_TRACKING_CLOSED.value]
        <= values[guidance.ControlMode.DR_PM_G_CLOSED.value] + 1.0e-9
        for values in comparable
    )


def write_csv(rows: list[dict[str, Any]], path: Path = DEFAULT_OUT) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "total": len(rows),
        "pass": sum(1 for row in rows if row["pass"]),
        "fail": sum(1 for row in rows if not row["pass"]),
        "fail_reasons": Counter(
            reason
            for row in rows
            for reason in str(row["fail_reason"]).split("|")
            if reason
        ),
        "mode_counts": {
            mode.value: {
                "total": sum(1 for row in rows if row["mode"] == mode.value),
                "pass": sum(1 for row in rows if row["mode"] == mode.value and row["pass"]),
            }
            for mode in MODE_CASES
        },
    }


def reload_modules() -> None:
    global control, guidance
    guidance = importlib.reload(guidance)
    control = importlib.reload(control)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args(argv)

    reload_modules()
    rows = run_all_cases()
    write_csv(rows, args.output)
    summary = summarize(rows)
    print(f"cases: {summary['total']}")
    for mode, counts in summary["mode_counts"].items():
        print(f"{mode}: pass={counts['pass']} total={counts['total']}")
    print(f"fail: {summary['fail']}")
    print(f"fail_reasons: {dict(summary['fail_reasons'])}")
    print(f"all_pass: {summary['fail'] == 0}")
    print(f"csv: {args.output}")
    return 0 if summary["fail"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
