#!/usr/bin/env python3
"""Replay supplied drop-test logs through the current motor guidance/control code.

The script intentionally treats the external logs as inputs and does not modify
mission persistent state. It adapts older log formats into the current
Sensor_Motor.guidance/control data classes, then reports whether target/origin
availability, freshness gates, and control outputs would allow guidance to run.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import statistics
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass
from datetime import datetime, timedelta
from pathlib import Path
from typing import Iterable, Optional

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from Sensor_Motor import control, guidance, motorapp  # noqa: E402
from lib import config  # noqa: E402


DEFAULT_0405_LOG = Path(r"C:\Users\ms kang\Desktop\0405_droptest\0405_droptest_log.txt")
DEFAULT_0510_DIR = Path(r"C:\logs\sensorlogs\run_20260510_191040")
DEFAULT_OUT = Path(r"C:\tmp\motorapp_guidance_replay")

STATE_NAME_TO_INT = {
    "LAUNCH_PAD": 0,
    "ASCENT": 1,
    "APOGEE": 2,
    "RELEASE": 3,
    "EGG": 4,
    "LANDED": 5,
}


@dataclass
class Sample:
    t: float
    state: int = 3
    lat: Optional[float] = None
    lon: Optional[float] = None
    course_deg: Optional[float] = None
    speed_mps: Optional[float] = None
    gps_pos_ok: bool = False
    gps_motion_ok: bool = False
    roll_deg: Optional[float] = None
    pitch_deg: Optional[float] = None
    yaw_deg: Optional[float] = None
    accx: float = 0.0
    accy: float = 0.0
    accz: float = 9.8
    magx: float = 0.0
    magy: float = 0.0
    magz: float = 0.0
    gyrx_deg_s: Optional[float] = None
    gyry_deg_s: Optional[float] = None
    gyrz_deg_s: Optional[float] = None
    imu_ok: bool = False
    alt_m: Optional[float] = None
    sink_rate_mps: Optional[float] = None
    baro_ok: bool = False


def _finite(value) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _to_float(value, default: Optional[float] = None) -> Optional[float]:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return out if math.isfinite(out) else default


def _valid_latlon(lat: Optional[float], lon: Optional[float]) -> bool:
    return (
        lat is not None
        and lon is not None
        and -90.0 <= lat <= 90.0
        and -180.0 <= lon <= 180.0
        and not (abs(lat) <= 1.0e-4 and abs(lon) <= 1.0e-4)
    )


def _gps_motion_sane(course_deg: Optional[float], speed_mps: Optional[float]) -> bool:
    return (
        course_deg is not None
        and speed_mps is not None
        and math.isfinite(course_deg)
        and math.isfinite(speed_mps)
        and 0.0 <= course_deg < 360.0
        and 0.5 <= speed_mps <= float(config.GPS_MAX_VALID_SPEED_MPS)
    )


def _distance_course_speed(prev: Sample, cur: Sample) -> tuple[Optional[float], Optional[float]]:
    if not (prev.gps_pos_ok and cur.gps_pos_ok and cur.t > prev.t):
        return None, None
    dn, de = guidance.latlon_to_ne(cur.lat, cur.lon, prev.lat, prev.lon)
    dt = cur.t - prev.t
    speed = math.hypot(dn, de) / dt
    course = math.degrees(math.atan2(de, dn)) % 360.0
    return course, speed


def _fill_derived_motion(samples: list[Sample]) -> None:
    prev_valid: Optional[Sample] = None
    for sample in samples:
        if sample.gps_pos_ok:
            if sample.course_deg is None or sample.speed_mps is None:
                if prev_valid is not None:
                    sample.course_deg, sample.speed_mps = _distance_course_speed(prev_valid, sample)
            sample.gps_motion_ok = sample.gps_pos_ok and _gps_motion_sane(sample.course_deg, sample.speed_mps)
            prev_valid = sample


def parse_0405_comm_log(path: Path) -> list[Sample]:
    ts_re = re.compile(r"^\[(?P<ts>[^]]+)\] INFO \| Communication")
    state_re = re.compile(r"STATE : (?P<state>[A-Z_]+)")
    baro_re = re.compile(r"Barometer :\s*([-+0-9.]+)")
    imu_re = re.compile(
        r"IMU : Gyro\((?P<gx>[-+0-9.]+),\s*(?P<gy>[-+0-9.]+),\s*(?P<gz>[-+0-9.]+)\), "
        r"Accel\((?P<ax>[-+0-9.]+),\s*(?P<ay>[-+0-9.]+),\s*(?P<az>[-+0-9.]+)\), "
        r"Mag\((?P<mx>[-+0-9.]+),\s*(?P<my>[-+0-9.]+),\s*(?P<mz>[-+0-9.]+)\)"
    )
    euler_re = re.compile(r"Euler angle\((?P<roll>[-+0-9.]+),\s*(?P<pitch>[-+0-9.]+),\s*(?P<yaw>[-+0-9.]+)\)")
    gps_re = re.compile(
        r"GPS : Lat\((?P<lat>[-+0-9.]+)\), Lon\((?P<lon>[-+0-9.]+)\), "
        r"Alt\((?P<alt>[-+0-9.]+)\), Time\((?P<time>[^)]+)\), Sats\((?P<sats>[-+0-9.]+)\)"
    )
    samples: list[Sample] = []
    cur: Optional[Sample] = None
    t0: Optional[datetime] = None
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        m = ts_re.match(raw)
        if m:
            if cur is not None:
                samples.append(cur)
            ts = datetime.strptime(m.group("ts"), "%Y-%m-%d %H:%M:%S.%f")
            if t0 is None:
                t0 = ts
            cur = Sample(t=(ts - t0).total_seconds(), state=0)
            continue
        if cur is None:
            continue
        if m := state_re.search(raw):
            cur.state = STATE_NAME_TO_INT.get(m.group("state"), cur.state)
        elif m := baro_re.search(raw):
            cur.alt_m = _to_float(m.group(1))
            cur.baro_ok = cur.alt_m is not None and cur.alt_m > 0.0
        elif m := imu_re.search(raw):
            cur.gyrx_deg_s = _to_float(m.group("gx"))
            cur.gyry_deg_s = _to_float(m.group("gy"))
            cur.gyrz_deg_s = _to_float(m.group("gz"))
            cur.accx = float(m.group("ax"))
            cur.accy = float(m.group("ay"))
            cur.accz = float(m.group("az"))
            cur.magx = float(m.group("mx"))
            cur.magy = float(m.group("my"))
            cur.magz = float(m.group("mz"))
            cur.imu_ok = True
        elif m := euler_re.search(raw):
            cur.roll_deg = _to_float(m.group("roll"))
            cur.pitch_deg = _to_float(m.group("pitch"))
            cur.yaw_deg = _to_float(m.group("yaw"))
        elif m := gps_re.search(raw):
            cur.lat = _to_float(m.group("lat"))
            cur.lon = _to_float(m.group("lon"))
            sats = int(_to_float(m.group("sats"), 0) or 0)
            cur.gps_pos_ok = _valid_latlon(cur.lat, cur.lon) and sats >= int(config.GPS_MIN_SATS)
    if cur is not None:
        samples.append(cur)
    _fill_derived_motion(samples)
    return samples


def _read_sensor_csv(path: Path, msg_id: str) -> list[dict]:
    rows = []
    with path.open(newline="", encoding="utf-8-sig", errors="replace") as f:
        for row in csv.DictReader(f):
            if row.get("receiver_name") == "Motor" and row.get("msg_id") == msg_id:
                rows.append(row)
    return rows


def parse_0510_event_logs(root: Path) -> list[Sample]:
    events: list[tuple[float, str, list[str]]] = []
    t0: Optional[datetime] = None

    def add_events(path: Path, msg_id: str, kind: str) -> None:
        nonlocal t0
        for row in _read_sensor_csv(path, msg_id):
            ts = datetime.fromisoformat(row["timestamp"])
            if t0 is None:
                t0 = ts
            events.append(((ts - t0).total_seconds(), kind, [x.strip() for x in row["data"].split(",")]))

    add_events(root / "GPS.csv", "1501901", "gps")
    add_events(root / "IMU.csv", "1401901", "imu")
    add_events(root / "Barometer.csv", "1301901", "baro")
    events.sort(key=lambda x: x[0])

    latest = Sample(t=0.0, state=3)
    out: list[Sample] = []
    last_alt: Optional[tuple[float, float]] = None
    for t, kind, fields in events:
        sample = Sample(**{**latest.__dict__, "t": t, "state": 3})
        if kind == "gps" and len(fields) == 7:
            lat = _to_float(fields[0])
            lon = _to_float(fields[1])
            spd = _to_float(fields[2])
            crs = _to_float(fields[3])
            fix = int(_to_float(fields[4], 0) or 0)
            sats = int(_to_float(fields[5], 0) or 0)
            rmc = fields[6].upper()
            sample.lat = lat
            sample.lon = lon
            sample.course_deg = crs
            sample.speed_mps = spd
            sample.gps_pos_ok = _valid_latlon(lat, lon) and fix >= 1 and sats >= int(config.GPS_MIN_SATS)
            sample.gps_motion_ok = sample.gps_pos_ok and rmc == "A" and _gps_motion_sane(crs, spd)
        elif kind == "imu" and len(fields) == 14:
            sample.roll_deg = _to_float(fields[0])
            sample.pitch_deg = _to_float(fields[1])
            sample.yaw_deg = _to_float(fields[2])
            sample.accx = float(_to_float(fields[3], 0.0) or 0.0)
            sample.accy = float(_to_float(fields[4], 0.0) or 0.0)
            sample.accz = float(_to_float(fields[5], 0.0) or 0.0)
            sample.magx = float(_to_float(fields[6], 0.0) or 0.0)
            sample.magy = float(_to_float(fields[7], 0.0) or 0.0)
            sample.magz = float(_to_float(fields[8], 0.0) or 0.0)
            sample.gyrx_deg_s = _to_float(fields[9])
            sample.gyry_deg_s = _to_float(fields[10])
            sample.gyrz_deg_s = _to_float(fields[11])
            sample.imu_ok = bool(int(_to_float(fields[12], 0) or 0))
        elif kind == "baro" and len(fields) == 3:
            alt = _to_float(fields[0])
            health = bool(int(_to_float(fields[1], 0) or 0))
            sample.alt_m = alt
            sample.baro_ok = health and alt is not None and alt > 0.0
            if sample.baro_ok and last_alt is not None and t > last_alt[0]:
                sample.sink_rate_mps = (last_alt[1] - float(alt)) / (t - last_alt[0])
            if sample.baro_ok:
                last_alt = (t, float(alt))
        latest = sample
        out.append(sample)
    return out


def parse_0510_logged_target(root: Path) -> Optional[tuple[float, float]]:
    path = root / "FlightLogic.csv"
    if not path.exists():
        return None
    with path.open(newline="", encoding="utf-8-sig", errors="replace") as f:
        for row in csv.DictReader(f):
            if row.get("receiver_name") != "Motor" or row.get("msg_id") != "1101901":
                continue
            parts = [x.strip() for x in row.get("data", "").split(",")]
            if len(parts) != 2:
                continue
            lat = _to_float(parts[0])
            lon = _to_float(parts[1])
            if _valid_latlon(lat, lon):
                return float(lat), float(lon)
    return None


def _make_gps(sample: Sample, now: float) -> motorapp._GpsFromApp:
    return motorapp._GpsFromApp(
        lat=sample.lat if sample.gps_pos_ok else None,
        lon=sample.lon if sample.gps_pos_ok else None,
        course_rad=math.radians(sample.course_deg) if sample.gps_motion_ok else None,
        speed_mps=sample.speed_mps if sample.gps_motion_ok else None,
        pos_ts=now if sample.gps_pos_ok else None,
        motion_ts=now if sample.gps_motion_ok else None,
        rx_ts=now,
        pos_health=int(sample.gps_pos_ok),
        motion_health=int(sample.gps_motion_ok),
    )


def _make_imu(sample: Sample, now: float) -> motorapp._ImuFromApp:
    ok = sample.imu_ok and sample.gyrz_deg_s is not None
    roll_deg  = sample.roll_deg  or 0.0
    pitch_deg = sample.pitch_deg or 0.0
    if ok and sample.accx is not None and sample.accy is not None and sample.accz is not None:
        try:
            import math as _m
            cp = _m.cos(_m.radians(pitch_deg))
            gx_ = -_m.sin(_m.radians(pitch_deg)) * 9.81
            gy_ =  cp * _m.sin(_m.radians(roll_deg))  * 9.81
            gz_ =  cp * _m.cos(_m.radians(roll_deg))  * 9.81
            lax = sample.accx - gx_
            lay = sample.accy - gy_
            laz = sample.accz - gz_
            lin_valid = True
        except Exception:
            lax = lay = laz = 0.0
            lin_valid = False
    else:
        lax = lay = laz = 0.0
        lin_valid = False

    return motorapp._ImuFromApp(
        roll_rad=math.radians(roll_deg) if sample.roll_deg is not None else None,
        pitch_rad=math.radians(pitch_deg) if sample.pitch_deg is not None else None,
        yaw_rad=math.radians(sample.yaw_deg or 0.0) if sample.yaw_deg is not None else None,
        accx_mps2=sample.accx,
        accy_mps2=sample.accy,
        accz_mps2=sample.accz,
        gyrx_rad_s=math.radians(sample.gyrx_deg_s or 0.0) if ok else None,
        gyry_rad_s=math.radians(sample.gyry_deg_s or 0.0) if ok else None,
        gyrz_rad_s=math.radians(-(sample.gyrz_deg_s or 0.0)) if ok else None,
        ts=now if ok else None,
        rx_ts=now,
        lin_acc_x=lax if lin_valid else None,
        lin_acc_y=lay if lin_valid else None,
        lin_acc_z=laz if lin_valid else None,
        lin_acc_valid=lin_valid,
        health=1 if ok else 0,
    )


def _make_baro(sample: Sample, now: float) -> motorapp._BaroFromApp:
    return motorapp._BaroFromApp(
        alt_m=sample.alt_m if sample.baro_ok else None,
        sink_rate=sample.sink_rate_mps if sample.baro_ok else None,
        rx_ts=now,
        health=1 if sample.baro_ok else 0,
    )


def _copy_cache(cache: motorapp._Cache) -> motorapp._Cache:
    """로컬 _Cache 객체의 센서 필드를 독립적으로 복사한다."""
    return motorapp._Cache(
        latest_gps=motorapp._GpsFromApp(**vars(cache.latest_gps)),
        latest_imu=motorapp._ImuFromApp(**vars(cache.latest_imu)),
        latest_baro=motorapp._BaroFromApp(**vars(cache.latest_baro)),
    )


def _valid_final_target(samples: Iterable[Sample]) -> Optional[tuple[float, float]]:
    for sample in reversed(list(samples)):
        if sample.gps_pos_ok:
            return float(sample.lat), float(sample.lon)
    return None


def replay(samples: list[Sample], target: Optional[tuple[float, float]], name: str) -> tuple[dict, list[dict]]:
    if not samples:
        return {"name": name, "rows": 0}, []
    cache = motorapp._Cache()
    control.reset()
    rows: list[dict] = []
    pending = list(samples)
    idx = 0
    start_t = samples[0].t
    end_t = samples[-1].t
    now = start_t
    dt = 1.0 / float(config.MOTOR_RATE_HZ)
    state = samples[0].state
    target_lat: Optional[float] = target[0] if target else None
    target_lon: Optional[float] = target[1] if target else None
    origin_lat: Optional[float] = None
    origin_lon: Optional[float] = None
    origin_lock_time: Optional[float] = None

    while now <= end_t + 1.0e-9:
        while idx < len(pending) and pending[idx].t <= now + 1.0e-9:
            sample = pending[idx]
            state = sample.state
            cache.latest_gps = _make_gps(sample, now)
            cache.latest_imu = _make_imu(sample, now)
            cache.latest_baro = _make_baro(sample, now)
            if origin_lat is None and state >= 3 and cache.latest_gps.pos_health:
                origin_lat = cache.latest_gps.lat
                origin_lon = cache.latest_gps.lon
                origin_lock_time = now
            idx += 1

        if state < 3:
            rows.append({"t": now, "state": state, "diag": "IDLE", "mode": "", "reason": "IDLE"})
            now += dt
            continue
        if state == 5:
            rows.append({"t": now, "state": state, "diag": "LANDED", "mode": "", "reason": "LANDED"})
            now += dt
            continue

        snap = _copy_cache(cache)
        l1_input, mode = guidance.ProduceL1Input(
            gps=snap.latest_gps,
            imu=snap.latest_imu,
            baro=snap.latest_baro,
            origin_lat=origin_lat,
            origin_lon=origin_lon,
            target_lat=target_lat,
            target_lon=target_lon,
            now=now,
        )
        g_out = guidance.ProduceL1Output(
            l1_input=l1_input,
            mode=mode,
            origin_lat=origin_lat,
            origin_lon=origin_lon,
            target_lat=target_lat,
            target_lon=target_lon,
            now=now,
        )
        if g_out.control_valid:
            meas = float("nan")
            if l1_input.gyrz is not None and l1_input.gyrz_quality in (
                guidance.SensorQuality.FRESH,
                guidance.SensorQuality.FRESHED,
            ):
                meas = math.degrees(float(l1_input.gyrz))
            cmd = control.ProduceCtrlOutput(control.ProduceCtrlInput(g_out, now), meas, now)
        else:
            cmd = control.WriteNeutral(now, g_out.reason)
        rows.append(
            {
                "t": now,
                "state": state,
                "diag": "ACTIVE" if g_out.control_valid and not g_out.degraded else "DEGRADED" if g_out.control_valid else g_out.reason,
                "mode": mode.value if hasattr(mode, "value") else str(mode),
                "reason": g_out.reason,
                "fail_reason": g_out.fail_reason,
                "pos_q": l1_input.pos_quality.value,
                "motion_q": l1_input.motion_quality.value,
                "gyrz_q": l1_input.gyrz_quality.value,
                "alt_q": l1_input.alt_quality.value,
                "valid": int(cmd.valid),
                "left": cmd.left_angle_deg,
                "right": cmd.right_angle_deg,
                "delta": cmd.delta_arm_deg,
                "cmd_dps": cmd.angular_velocity_cmd_deg_s,
                "xtrack": g_out.crossTrack,
                "along": g_out.alongTrack,
                "start_lat": origin_lat,
                "start_lon": origin_lon,
                "target_lat": target_lat,
                "target_lon": target_lon,
            }
        )
        now += dt

    active_rows = [r for r in rows if r.get("state", 0) >= 3 and r.get("state", 0) < 5]
    control_valid = [r for r in active_rows if r.get("valid") == 1]
    reasons = Counter(str(r.get("reason", "")) for r in active_rows)
    modes = Counter(str(r.get("mode", "")) for r in active_rows)
    fail_reasons = Counter(str(r.get("fail_reason", "")) for r in active_rows)
    deltas = [float(r["delta"]) for r in control_valid if _finite(r.get("delta"))]
    summary = {
        "name": name,
        "rows": len(rows),
        "active_ticks": len(active_rows),
        "control_valid_ticks": len(control_valid),
        "control_valid_pct": 100.0 * len(control_valid) / max(1, len(active_rows)),
        "origin_locked": origin_lock_time is not None,
        "origin_lock_t": origin_lock_time,
        "origin": (cache.start_lat, cache.start_lon),
        "target": target,
        "top_reasons": reasons.most_common(6),
        "top_modes": modes.most_common(6),
        "top_fail_reasons": fail_reasons.most_common(6),
        "delta_abs_p95": percentile([abs(v) for v in deltas], 95),
        "delta_abs_max": max([abs(v) for v in deltas], default=math.nan),
    }
    return summary, rows


def percentile(values: list[float], pct: float) -> float:
    values = sorted(v for v in values if _finite(v))
    if not values:
        return math.nan
    k = (len(values) - 1) * pct / 100.0
    lo = math.floor(k)
    hi = math.ceil(k)
    if lo == hi:
        return values[lo]
    return values[lo] * (hi - k) + values[hi] * (k - lo)


def freshness_stats(samples: list[Sample]) -> dict[str, dict]:
    tracks = {
        "gps_pos": [s.t for s in samples if s.gps_pos_ok],
        "gps_motion": [s.t for s in samples if s.gps_motion_ok],
        "imu": [s.t for s in samples if s.imu_ok],
        "baro": [s.t for s in samples if s.baro_ok],
    }
    max_age = {
        "gps_pos": guidance.POS_FRESH_AGE,
        "gps_motion": guidance.MOTION_FRESH_AGE,
        "imu": guidance.GYRZ_FRESH_AGE,
        "baro": guidance.ALT_FRESH_AGE,
    }
    out = {}
    for name, times in tracks.items():
        gaps = [b - a for a, b in zip(times, times[1:]) if b >= a]
        out[name] = {
            "count": len(times),
            "mean_gap": statistics.fmean(gaps) if gaps else math.nan,
            "p95_gap": percentile(gaps, 95),
            "max_gap": max(gaps, default=math.nan),
            "fresh_age": max_age[name],
            "gap_over_fresh": sum(1 for g in gaps if g > max_age[name]),
        }
    return out


def write_rows(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames: list[str] = []
    for row in rows:
        for key in row.keys():
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_report(path: Path, summaries: list[dict], fresh: dict[str, dict[str, dict]]) -> None:
    lines = ["# Motorapp Guidance/Control Replay", ""]
    lines.append("## Replay Summary")
    lines.append("| case | active ticks | valid control | origin | target | top mode | top reason/fail | |delta| p95/max |")
    lines.append("|---|---:|---:|---|---|---|---|---:|")
    for s in summaries:
        lines.append(
            f"| {s['name']} | {s['active_ticks']} | {s['control_valid_pct']:.1f}% | "
            f"{s['origin']} | {s['target']} | {s['top_modes'][:2]} | "
            f"{s['top_reasons'][:2]} / {s['top_fail_reasons'][:2]} | "
            f"{s['delta_abs_p95']:.2f}/{s['delta_abs_max']:.2f} |"
        )
    lines.append("")
    lines.append("## Freshness Observations")
    lines.append("| log | sensor | valid samples | mean gap | p95 gap | max gap | fresh age | gaps over fresh |")
    lines.append("|---|---|---:|---:|---:|---:|---:|---:|")
    for log_name, stats in fresh.items():
        for sensor, st in stats.items():
            lines.append(
                f"| {log_name} | {sensor} | {st['count']} | {st['mean_gap']:.3f} | "
                f"{st['p95_gap']:.3f} | {st['max_gap']:.3f} | {st['fresh_age']:.3f} | "
                f"{st['gap_over_fresh']} |"
            )
    lines.append("")
    lines.append("## Notes")
    lines.append("- Cases ending in no_target replay the logs without injecting target coordinates.")
    lines.append("- Cases ending in final_target inject the final valid GPS coordinate as a synthetic target so the current L1/control path can be exercised.")
    lines.append("- 0405 Communication telemetry is only 1 Hz, so it is stricter than the production 10 Hz GPS/baro and 10 Hz IMU sender behavior.")
    lines.append("- 0510 logs use older Motor message formats; this replay adapts those fields to the current data classes.")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--log0405", type=Path, default=DEFAULT_0405_LOG)
    parser.add_argument("--dir0510", type=Path, default=DEFAULT_0510_DIR)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()

    samples0405 = parse_0405_comm_log(args.log0405)
    samples0510 = parse_0510_event_logs(args.dir0510)
    target0405 = _valid_final_target(samples0405)
    target0510_logged = parse_0510_logged_target(args.dir0510)
    target0510_final = _valid_final_target(samples0510)

    runs = [
        ("0405_no_target", samples0405, None),
        ("0405_final_target", samples0405, target0405),
        ("0510_no_target", samples0510, None),
        ("0510_logged_target", samples0510, target0510_logged),
        ("0510_final_target", samples0510, target0510_final),
    ]
    summaries = []
    for name, samples, target in runs:
        summary, rows = replay(samples, target, name)
        summaries.append(summary)
        write_rows(args.out / f"{name}.csv", rows)

    fresh = {
        "0405": freshness_stats(samples0405),
        "0510": freshness_stats(samples0510),
    }
    write_report(args.out / "summary.md", summaries, fresh)

    for s in summaries:
        print(
            f"{s['name']}: active={s['active_ticks']} valid={s['control_valid_pct']:.1f}% "
            f"origin={s['origin']} target={s['target']} modes={s['top_modes'][:3]} reasons={s['top_reasons'][:3]}"
        )
    print(f"report={args.out / 'summary.md'}")


if __name__ == "__main__":
    main()
