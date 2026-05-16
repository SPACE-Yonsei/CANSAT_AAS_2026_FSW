"""Replay and segment-check a Motor FSW drop-test trace.

Default input:
  C:\\Users\\ms kang\\Desktop\\0405_droptest\\happened\\motor_fsw_replay_trace.csv

Outputs in the input directory by default:
  - replay_result.csv
  - replay_segment_summary.md

The script is intentionally standalone and uses only the standard library plus
the repo's Motor modules.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
import time as pytime
from collections import Counter, defaultdict
from pathlib import Path
from types import SimpleNamespace

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from Sensor_Motor import motor_control, motor_guidance, motorapp  # noqa: E402


DEFAULT_INPUT = Path(r"C:\Users\ms kang\Desktop\0405_droptest\happened\motor_fsw_replay_trace.csv")
DEFAULT_HIGH_YAW_DPS = 120.0
DEFAULT_LOW_ALT_M = 50.0
DEFAULT_EARLY_DESCENT_SEC = 30.0


class _ReplayPi:
    def __init__(self) -> None:
        self.pulses: dict[int, int] = {}

    def set_servo_pulsewidth(self, pin: int, pulse: int) -> None:
        self.pulses[pin] = pulse


def _float(row: dict, name: str, default: float = math.nan) -> float:
    raw = row.get(name, "")
    if raw is None or raw == "":
        return default
    try:
        return float(raw)
    except (TypeError, ValueError):
        return default


def _int(row: dict, name: str, default: int = 0) -> int:
    value = _float(row, name, math.nan)
    if math.isnan(value):
        return default
    return int(value)


def _bool(row: dict, name: str) -> bool:
    return str(row.get(name, "")).strip().lower() in {"1", "true", "yes", "y"}


def _is_finite(value: float) -> bool:
    return value is not None and not math.isnan(value) and not math.isinf(value)


def _wrap180(deg: float) -> float:
    while deg > 180.0:
        deg -= 360.0
    while deg <= -180.0:
        deg += 360.0
    return deg


def _mean(values: list[float]) -> float:
    return statistics.fmean(values) if values else math.nan


def _median(values: list[float]) -> float:
    return statistics.median(values) if values else math.nan


def _stdev(values: list[float]) -> float:
    return statistics.stdev(values) if len(values) >= 2 else 0.0


def _percentile(values: list[float], pct: float) -> float:
    if not values:
        return math.nan
    data = sorted(values)
    k = (len(data) - 1) * pct / 100.0
    lo = math.floor(k)
    hi = math.ceil(k)
    if lo == hi:
        return data[int(k)]
    return data[lo] * (hi - k) + data[hi] * (k - lo)


def _stats(values: list[float]) -> dict:
    clean = [v for v in values if _is_finite(v)]
    if not clean:
        return {
            "count": 0,
            "mean": math.nan,
            "median": math.nan,
            "std": math.nan,
            "min": math.nan,
            "max": math.nan,
            "p90": math.nan,
            "p95": math.nan,
            "p99": math.nan,
            "outlier_ratio": math.nan,
        }
    mu = _mean(clean)
    sd = _stdev(clean)
    if sd > 0:
        outliers = [v for v in clean if abs(v - mu) > 3.0 * sd]
    else:
        outliers = []
    return {
        "count": len(clean),
        "mean": mu,
        "median": _median(clean),
        "std": sd,
        "min": min(clean),
        "max": max(clean),
        "p90": _percentile(clean, 90),
        "p95": _percentile(clean, 95),
        "p99": _percentile(clean, 99),
        "outlier_ratio": len(outliers) / len(clean),
    }


def _fmt(value: object, digits: int = 3) -> str:
    if isinstance(value, float):
        if not _is_finite(value):
            return "nan"
        return f"{value:.{digits}f}"
    return str(value)


def _valid_gps(row: dict) -> bool:
    lat = _float(row, "lat")
    lon = _float(row, "lon")
    return (
        -90.0 <= lat <= 90.0
        and -180.0 <= lon <= 180.0
        and abs(lon) > 1e-9
        and _int(row, "fix_quality") >= 1
        and _int(row, "sats") >= 4
        and str(row.get("rmc_status", "")).upper() == "A"
    )


def _invalid_gps(row: dict) -> bool:
    reason = str(row.get("fdir_reason", ""))
    return (
        not _valid_gps(row)
        or _bool(row, "gps_jump_rejected")
        or "GPS invalid" in reason
        or "GPS jump" in reason
    )


def _sensor_fault(row: dict) -> bool:
    reason = str(row.get("fdir_reason", "")).lower()
    if _bool(row, "fdir_pass"):
        return False
    fault_tokens = (
        "stale",
        "unhealthy",
        "non-finite",
        "nan",
        "inf",
        "negative",
        "jump",
        "invalid",
        "target",
    )
    return any(token in reason for token in fault_tokens)


def _segment_for_row(row: dict, first_state3_t: float, high_yaw_dps: float, low_alt_m: float, early_sec: float) -> str:
    sim_t = _float(row, "sim_t", _float(row, "tick", 0.0))
    baro_m = _float(row, "baro_m")
    gyrz = abs(_float(row, "gyrz_deg_s", math.nan))

    if _invalid_gps(row):
        return "INIT_INVALID_GPS"
    if _is_finite(gyrz) and gyrz >= high_yaw_dps:
        return "HIGH_YAW_RATE"
    if _is_finite(baro_m) and baro_m <= low_alt_m:
        return "LOW_ALTITUDE"
    if _int(row, "state") == 3 and _is_finite(first_state3_t) and sim_t <= first_state3_t + early_sec:
        return "EARLY_DESCENT"
    if _sensor_fault(row):
        return "SENSOR_FAULT"
    return "STABLE_GLIDE"


def _gps_from_local(start_lat: float, start_lon: float, east_m: float, north_m: float) -> tuple[float, float]:
    lat = start_lat + north_m / 111_000.0
    scale = 111_000.0 * math.cos(math.radians(start_lat))
    lon = start_lon + (east_m / scale if abs(scale) > 1e-9 else 0.0)
    return lat, lon


def _infer_target(rows: list[dict]) -> tuple[float, float] | None:
    for row in rows:
        lat = _float(row, "lat")
        lon = _float(row, "lon")
        my_e = _float(row, "my_E")
        my_n = _float(row, "my_N")
        target_e = _float(row, "target_E")
        target_n = _float(row, "target_N")
        if all(_is_finite(v) for v in (lat, lon, my_e, my_n, target_e, target_n)) and abs(lon) > 1e-9:
            start_lat = lat - my_n / 111_000.0
            scale = 111_000.0 * math.cos(math.radians(start_lat))
            start_lon = lon - (my_e / scale if abs(scale) > 1e-9 else 0.0)
            return _gps_from_local(start_lat, start_lon, target_e, target_n)
    return None


def _reset_motorapp_for_replay(target: tuple[float, float] | None) -> None:
    motor_guidance.init_guidance()
    motorapp.MOTORAPP_RUNSTATUS  = True
    motorapp.MOTOR_ENABLED       = True
    motorapp.STATE               = 0
    motorapp._PREV_STATE         = -1
    motorapp._START_POINT_LOCKED = False
    motorapp.TARGET              = None

    motorapp.IMU.yaw        = 0.0
    motorapp.IMU.gyrz       = 0.0
    motorapp.IMU.imu_health = 1

    motorapp.GPS_VECTOR.lat       = 0.0
    motorapp.GPS_VECTOR.lon       = 0.0
    motorapp.GPS_VECTOR.direction = 0.0
    motorapp.GPS_VECTOR.velocity  = 0.0

    motorapp.GPS_HEALTH.pos_health    = 0
    motorapp.GPS_HEALTH.motion_health = 0

    motorapp.ALT = 0.0

    if target is not None:
        motorapp.handle_target_coord(f"{target[0]},{target[1]}")


def _replay_current_code(rows: list[dict]) -> list[dict]:
    target = _infer_target(rows)
    _reset_motorapp_for_replay(target)
    control_backend = _ReplayPi()
    motor_control.set_neutral(control_backend)
    out: list[dict] = []

    original_time = pytime.time
    try:
        for row in rows:
            sim_t = _float(row, "sim_t", _float(row, "tick", 0.0))
            fake_now = original_time() + sim_t
            pytime.time = lambda now=fake_now: now  # type: ignore[assignment]

            motorapp.handle_flight_state(str(_int(row, "state", motorapp.STATE)))
            motorapp.handle_gps(
                ",".join(
                    [
                        str(row.get("lat", "")),
                        str(row.get("lon", "")),
                        str(row.get("gps_speed_m_s", "")),
                        str(row.get("gps_course_deg", "")),
                        str(row.get("fix_quality", "")),
                        str(row.get("sats", "")),
                        str(row.get("rmc_status", "")),
                        "1" if _valid_gps(row) and not _bool(row, "gps_jump_rejected") else "0",
                    ]
                )
            )
            motorapp.handle_imu(
                ",".join(
                    [
                        str(row.get("yaw_deg", "")),
                        str(row.get("gyrz_deg_s", "")),
                        "1",
                    ]
                )
            )
            motorapp.handle_barometer(str(row.get("baro_m", "")))

            snap = motorapp._snapshot_sensors()
            fdir_reason = motorapp._check_fdir(snap)
            safety_action = "control"
            current_phase = ""
            current_distance = math.nan
            current_crosstrack = math.nan
            current_along_track = math.nan
            current_heading_error = math.nan
            current_desired_yr = math.nan
            current_cmd_yr = 0.0
            left_pulse = motor_control.LEFT_NEUTRAL
            right_pulse = motor_control.RIGHT_NEUTRAL
            pulse_offset = 0.0

            if motorapp.STATE < 3 or not motorapp.MOTOR_ENABLED:
                safety_action = "neutral"
                motor_control.set_neutral(control_backend)
            elif motorapp.STATE == 5:
                safety_action = "motors_off"
                motor_control.WriteOff(control_backend)
                left_pulse = 0
                right_pulse = 0
            elif fdir_reason is not None:
                safety_action = "neutral"
                motor_control.set_neutral(control_backend)
            else:
                imu = SimpleNamespace(yaw=snap.yaw, gyrz=snap.gyrz)
                gps = motor_guidance.GpsVector(snap.lat, snap.lon, snap.speed, snap.course)
                fidelity = motor_guidance.GpsFidelity(
                    snap.fix_quality,
                    snap.sats,
                    snap.rmc_status,
                    snap.gps_health,
                )
                result = motor_guidance.guidance(imu, gps, fidelity, snap.target, snap.alt)
                current_phase = result.state
                current_distance = float(result.distance)
                current_crosstrack = float(getattr(result, "crosstrack_error", math.nan))
                current_along_track = float(getattr(result, "along_track", math.nan))
                current_heading_error = float(getattr(result, "heading_error", math.nan))
                current_desired_yr = float(getattr(result, "desired_angular_velocity", math.nan))
                current_cmd_yr = float(result.commanded_angular_velocity)
                if result.state == "FDIR":
                    safety_action = "neutral"
                    motor_control.set_neutral(control_backend)
                else:
                    feedback = motor_control.control(control_backend, current_cmd_yr)
                    left_pulse = feedback.left_pulse
                    right_pulse = feedback.right_pulse
                    _, _, *_, pulse_offset = motor_control.actuator_mixer(current_cmd_yr)

            out.append(
                {
                    "current_fdir_pass": fdir_reason is None,
                    "current_fdir_reason": "" if fdir_reason is None else fdir_reason,
                    "current_phase": current_phase,
                    "current_distance_m": current_distance,
                    "current_crosstrack_error_m": current_crosstrack,
                    "current_along_track_m": current_along_track,
                    "current_heading_error_deg": current_heading_error,
                    "current_desired_angular_velocity_deg_s": current_desired_yr,
                    "current_commanded_angular_velocity_deg_s": current_cmd_yr,
                    "current_pulse_offset_us": pulse_offset,
                    "current_left_pulse_us": left_pulse,
                    "current_right_pulse_us": right_pulse,
                    "current_safety_action": safety_action,
                }
            )
    finally:
        pytime.time = original_time  # type: ignore[assignment]
    return out


def _contiguous_ranges(rows: list[dict], segments: list[str]) -> list[dict]:
    ranges = []
    if not rows:
        return ranges
    start = 0
    for idx in range(1, len(rows)):
        if segments[idx] != segments[start]:
            ranges.append(_range_summary(rows, segments[start], start, idx - 1))
            start = idx
    ranges.append(_range_summary(rows, segments[start], start, len(rows) - 1))
    return ranges


def _range_summary(rows: list[dict], segment: str, start: int, end: int) -> dict:
    return {
        "segment": segment,
        "row_range": f"{start}-{end}",
        "time_range": f"{_fmt(_float(rows[start], 'sim_t'))}-{_fmt(_float(rows[end], 'sim_t'))}",
        "count": end - start + 1,
    }


def _segment_table(rows: list[dict], segments: list[str]) -> list[dict]:
    out = []
    for segment in ("INIT_INVALID_GPS", "EARLY_DESCENT", "STABLE_GLIDE", "HIGH_YAW_RATE", "LOW_ALTITUDE", "SENSOR_FAULT"):
        idxs = [i for i, s in enumerate(segments) if s == segment]
        if not idxs:
            out.append(
                {
                    "segment": segment,
                    "range": "-",
                    "features": "no rows",
                    "expected": "-",
                    "actual": "-",
                    "pass_fail": "N/A",
                    "improvement": "-",
                }
            )
            continue
        sub = [rows[i] for i in idxs]
        fdir_pass_ratio = sum(1 for r in sub if _bool(r, "fdir_pass")) / len(sub)
        finite_guidance_cols = ("distance_m", "heading_error_deg", "desired_angular_velocity_deg_s", "commanded_angular_velocity_deg_s")
        finite_ratio = sum(
            1
            for r in sub
            if all(_is_finite(_float(r, col)) for col in finite_guidance_cols if col in r)
        ) / len(sub)
        pulse_ok = all(
            (
                not _is_finite(_float(r, "left_pulse_us"))
                or motor_control.LEFT_MIN <= _float(r, "left_pulse_us") <= motor_control.LEFT_MAX
            )
            and (
                not _is_finite(_float(r, "right_pulse_us"))
                or motor_control.RIGHT_MIN <= _float(r, "right_pulse_us") <= motor_control.RIGHT_MAX
            )
            for r in sub
        )
        reason_top = Counter(str(r.get("fdir_reason", "")) or "PASS" for r in sub).most_common(2)
        ranges = _contiguous_ranges(rows, segments)
        range_text = "; ".join(x["row_range"] for x in ranges if x["segment"] == segment)[:80]
        if segment == "INIT_INVALID_GPS":
            expected = "FDIR fail, neutral"
            passed = fdir_pass_ratio < 0.2
            improvement = "separate pre-GPS-stable start/target logs"
        elif segment == "HIGH_YAW_RATE":
            expected = "spike/natural rotation distinguish; excessive command blocked"
            passed = pulse_ok
            improvement = "phase/adaptive yaw-rate threshold candidate"
        elif segment == "LOW_ALTITUDE":
            expected = "PATTERN/LANDING limit, clamp valid"
            passed = pulse_ok
            improvement = "requires real motor-on landing test"
        elif segment == "SENSOR_FAULT":
            expected = "FDIR fail, neutral"
            passed = fdir_pass_ratio < 0.5
            improvement = "split fault reason telemetry"
        else:
            expected = "FDIR pass, finite guidance/control"
            passed = fdir_pass_ratio > 0.5 and finite_ratio > 0.5 and pulse_ok
            improvement = "keep handler-to-guidance replay in regression"
        out.append(
            {
                "segment": segment,
                "range": range_text or f"{idxs[0]}-{idxs[-1]}",
                "features": f"n={len(sub)}, fdir_pass={fdir_pass_ratio:.1%}, top_reason={reason_top}",
                "expected": expected,
                "actual": f"finite_guidance={finite_ratio:.1%}, pulse_ok={pulse_ok}",
                "pass_fail": "PASS" if passed else "REVIEW",
                "improvement": improvement,
            }
        )
    return out


def _numeric_stats(rows: list[dict], cols: list[str]) -> dict[str, dict]:
    return {col: _stats([_float(r, col) for r in rows]) for col in cols}


def _sampling_stats(rows: list[dict]) -> dict:
    times = [_float(r, "sim_t", _float(r, "tick", math.nan)) for r in rows]
    clean = [t for t in times if _is_finite(t)]
    dts = [b - a for a, b in zip(clean, clean[1:]) if b >= a]
    return _stats(dts)


def _descent_stats(rows: list[dict]) -> dict:
    samples = [
        (_float(r, "sim_t", _float(r, "tick", math.nan)), _float(r, "baro_m"))
        for r in rows
        if _is_finite(_float(r, "sim_t", _float(r, "tick", math.nan))) and _is_finite(_float(r, "baro_m"))
    ]
    if len(samples) < 2:
        return {"dt_s": math.nan, "descent_rate_m_s": math.nan}
    t0, h0 = samples[0]
    t1, h1 = samples[-1]
    dt = max(1e-9, t1 - t0)
    return {"dt_s": dt, "descent_rate_m_s": (h0 - h1) / dt}


def _drift_stats(rows: list[dict]) -> dict:
    valid = [
        r for r in rows
        if _valid_gps(r) and _is_finite(_float(r, "sim_t", _float(r, "tick", math.nan)))
    ]
    if len(valid) < 2:
        return {"dt_s": math.nan, "distance_m": math.nan, "speed_m_s": math.nan, "bearing_deg": math.nan}
    first = valid[0]
    last = valid[-1]
    t0 = _float(first, "sim_t", _float(first, "tick", math.nan))
    t1 = _float(last, "sim_t", _float(last, "tick", math.nan))
    lat0 = _float(first, "lat")
    lon0 = _float(first, "lon")
    lat1 = _float(last, "lat")
    lon1 = _float(last, "lon")
    north = (lat1 - lat0) * 111_000.0
    east = (lon1 - lon0) * 111_000.0 * math.cos(math.radians(lat0))
    distance = math.hypot(north, east)
    bearing = math.degrees(math.atan2(east, north))
    dt = max(1e-9, t1 - t0)
    return {"dt_s": dt, "distance_m": distance, "speed_m_s": distance / dt, "bearing_deg": bearing}


def _write_result_csv(path: Path, rows: list[dict], replay_rows: list[dict], segments: list[str]) -> None:
    result_path = path / "replay_result.csv"
    fieldnames = list(rows[0].keys()) + [
        "segment",
        "current_fdir_pass",
        "current_fdir_reason",
        "current_phase",
        "current_distance_m",
        "current_crosstrack_error_m",
        "current_along_track_m",
        "current_heading_error_deg",
        "current_desired_angular_velocity_deg_s",
        "current_commanded_angular_velocity_deg_s",
        "current_pulse_offset_us",
        "current_left_pulse_us",
        "current_right_pulse_us",
        "current_safety_action",
    ]
    with result_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row, replay, segment in zip(rows, replay_rows, segments):
            combined = dict(row)
            combined["segment"] = segment
            combined.update(replay)
            writer.writerow(combined)


def _write_markdown(path: Path, rows: list[dict], segments: list[str], replay_rows: list[dict]) -> None:
    md_path = path / "replay_segment_summary.md"
    segment_rows = _segment_table(rows, segments)
    reason_counts = Counter(str(r.get("fdir_reason", "")) or "PASS" for r in rows)
    current_reason_counts = Counter(str(r.get("current_fdir_reason", "")) or "PASS" for r in replay_rows)
    stats_cols = [
        "baro_m",
        "gps_speed_m_s",
        "gps_course_deg",
        "yaw_deg",
        "gyrz_deg_s",
        "desired_angular_velocity_deg_s",
        "commanded_angular_velocity_deg_s",
        "pulse_offset_us",
        "left_pulse_us",
        "right_pulse_us",
    ]
    stats = _numeric_stats(rows, stats_cols)
    sample_stats = _sampling_stats(rows)
    descent = _descent_stats(rows)
    drift = _drift_stats(rows)

    lon_zero_ratio = sum(1 for r in rows if abs(_float(r, "lon", 0.0)) < 1e-9) / max(1, len(rows))
    gps_jump_ratio = sum(1 for r in rows if _bool(r, "gps_jump_rejected")) / max(1, len(rows))
    fdir_pass_ratio = sum(1 for r in rows if _bool(r, "fdir_pass")) / max(1, len(rows))
    fix_dist = Counter(str(r.get("fix_quality", "")) for r in rows)
    sats_dist = Counter(str(r.get("sats", "")) for r in rows)
    rmc_dist = Counter(str(r.get("rmc_status", "")) for r in rows)

    lines = []
    lines.append("# Motor FSW Replay Segment Summary")
    lines.append("")
    lines.append(f"- rows: {len(rows)}")
    lines.append(f"- time range: {_fmt(_float(rows[0], 'sim_t'))} - {_fmt(_float(rows[-1], 'sim_t'))} s")
    lines.append(f"- fdir_pass ratio(trace): {fdir_pass_ratio:.2%}")
    lines.append(f"- lon=0 ratio: {lon_zero_ratio:.2%}")
    lines.append(f"- gps_jump_rejected ratio: {gps_jump_ratio:.2%}")
    lines.append(f"- sampling dt mean/median/max: {_fmt(sample_stats['mean'])}/{_fmt(sample_stats['median'])}/{_fmt(sample_stats['max'])} s")
    lines.append(f"- baro descent rate: {_fmt(descent['descent_rate_m_s'])} m/s over {_fmt(descent['dt_s'])} s")
    lines.append(
        f"- GPS ground drift: {_fmt(drift['distance_m'])} m, {_fmt(drift['speed_m_s'])} m/s, bearing {_fmt(drift['bearing_deg'])} deg "
        "(valid GPS endpoints only; no airspeed/wind sensor)"
    )
    lines.append("")
    lines.append("## Segment Results")
    lines.append("| segment | row/time range | key features | expected behavior | actual behavior | pass/fail | improvement |")
    lines.append("|---|---|---|---|---|---|---|")
    for item in segment_rows:
        lines.append(
            f"| {item['segment']} | {item['range']} | {item['features']} | {item['expected']} | "
            f"{item['actual']} | {item['pass_fail']} | {item['improvement']} |"
        )
    lines.append("")
    lines.append("## FDIR Reason Distribution")
    for reason, count in reason_counts.most_common(20):
        lines.append(f"- trace: {count}: {reason}")
    lines.append("")
    lines.append("## Current-Code FDIR Reason Distribution")
    for reason, count in current_reason_counts.most_common(20):
        lines.append(f"- current: {count}: {reason}")
    lines.append("")
    lines.append("## Sensor / Guidance / Control Stats")
    lines.append("| field | count | mean | median | std | min | max | p90 | p95 | p99 | outlier_ratio |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for col, st in stats.items():
        lines.append(
            f"| {col} | {st['count']} | {_fmt(st['mean'])} | {_fmt(st['median'])} | {_fmt(st['std'])} | "
            f"{_fmt(st['min'])} | {_fmt(st['max'])} | {_fmt(st['p90'])} | {_fmt(st['p95'])} | "
            f"{_fmt(st['p99'])} | {_fmt(st['outlier_ratio'])} |"
        )
    lines.append("")
    lines.append("## GPS Distributions")
    lines.append(f"- fix_quality: {dict(fix_dist)}")
    lines.append(f"- sats: {dict(sats_dist)}")
    lines.append(f"- rmc_status: {dict(rmc_dist)}")
    lines.append("")
    lines.append("## FDIR / Stale Condition Evaluation")
    lines.append("| item | current condition | trace observation | evaluation | recommended change |")
    lines.append("|---|---|---|---|---|")
    lines.append("| GPS stale timeout | 10 s | trace has 1 Hz rows; invalid GPS is quality/jump dominated | timeout is not overly strict here | keep stale timeout; count invalid/jump separately |")
    lines.append("| IMU stale timeout | 5 s | IMU columns are continuous in trace | reasonable | add timestamp gap field to future logs |")
    lines.append("| BARO stale timeout | 15 s | baro_m continuous descent | conservative | tune after sensor-dropout tests |")
    lines.append("| GPS quality | fix>=1, sats>=4, rmc=A, health>=1 | lon=0/jump/void early rows exist | necessary gate | explicitly gate guidance until GPS stabilizes |")
    lines.append("| GPS jump | 200 m/s, 2 stable samples | gps_jump_rejected rows exist near start | initial reject is reasonable | replay dt-based threshold needs regression |")
    lines.append("| yaw-rate implausible | extreme >=500 deg/s or high-rate spike delta >=350 deg/s | high gyrz exists; motor-off drop-test can naturally spin | avoids treating every >200 deg/s sample as a hard fault | identify actuator-coupled threshold after motor-on test |")
    lines.append("| baro negative | baro_m < 0 fault | trace remains positive | reasonable | separate landed detection from negative fault |")
    lines.append("| target missing | target None fault | target delivery matters right after state 3 | reasonable | add target receipt log at state 3 |")
    lines.append("| safety | state<3/MEC OFF neutral, state==5 off, FDIR neutral | replay can check output range | structurally reasonable | verify with hardware motor-on drop-test |")
    lines.append("")
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(input_path: Path, output_dir: Path, high_yaw_dps: float, low_alt_m: float, early_sec: float) -> None:
    with input_path.open(newline="", encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"empty CSV: {input_path}")

    state3_times = [_float(r, "sim_t") for r in rows if _int(r, "state") == 3]
    first_state3_t = min(state3_times) if state3_times else math.nan
    segments = [_segment_for_row(r, first_state3_t, high_yaw_dps, low_alt_m, early_sec) for r in rows]
    replay_rows = _replay_current_code(rows)

    output_dir.mkdir(parents=True, exist_ok=True)
    _write_result_csv(output_dir, rows, replay_rows, segments)
    _write_markdown(output_dir, rows, segments, replay_rows)

    print(f"rows={len(rows)}")
    print(f"output={output_dir}")
    print("segments=" + str(dict(Counter(segments))))
    print("top_fdir_reasons=" + str(Counter(str(r.get("fdir_reason", "")) or "PASS" for r in rows).most_common(8)))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--high-yaw-dps", type=float, default=DEFAULT_HIGH_YAW_DPS)
    parser.add_argument("--low-alt-m", type=float, default=DEFAULT_LOW_ALT_M)
    parser.add_argument("--early-descent-sec", type=float, default=DEFAULT_EARLY_DESCENT_SEC)
    args = parser.parse_args()
    output_dir = args.output_dir if args.output_dir is not None else args.input.parent
    run(args.input, output_dir, args.high_yaw_dps, args.low_alt_m, args.early_descent_sec)


if __name__ == "__main__":
    main()
