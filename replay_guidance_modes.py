"""Offline replay of the guidance pipeline against a recorded raw_motor log.

Drives guidance.DecideControlMode / ProduceL1Input / ProduceL1Output and an
estimated control.step on the *recorded sensor snapshots*, with no hardware.
The logged ``control_mode`` column is the BEFORE result (whatever guidance ran
during the flight); the recomputed mode is the AFTER result (current guidance).

Usage:
    python replay_guidance_modes.py [INPUT.xlsx|.csv] [--out result.csv] [--summary summary.md]

Default input: the 3차 flight log (additional working directory).
Outputs: replay_guidance_result.csv, replay_summary.md
Sensors are reconstructed from the raw_motor snapshot columns; *_age_s columns
recover each sample's original monotonic timestamp (ts = monotonic_s - age_s).
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from collections import Counter, defaultdict

# Repo root on path (script lives at repo root).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from lib import config
from Sensor_Motor import control, guidance
from Sensor_Motor.sensor_types import _BaroFromApp, _GpsFromApp, _ImuFromApp

ControlMode = guidance.ControlMode

DEFAULT_INPUT = r"C:\Users\ms kang\Desktop\닭\3차\raw_motor_part.xlsx"
DT_MAX_S = 1.0   # gap clamp when attributing duration to a row's mode


# ── 입력 로딩 ────────────────────────────────────────────────────────────────

def _load_rows(path: str):
    """Return (header list, list-of-dicts) from .xlsx or .csv."""
    if path.lower().endswith(".xlsx"):
        import openpyxl
        wb = openpyxl.load_workbook(path, read_only=True, data_only=True)
        ws = wb.active
        it = ws.iter_rows(values_only=True)
        header = [str(h) for h in next(it)]
        rows = [dict(zip(header, r)) for r in it]
        wb.close()
        return header, rows
    with open(path, newline="", encoding="utf-8", errors="replace") as f:
        rdr = csv.DictReader(f)
        return rdr.fieldnames, list(rdr)


def _f(row, key):
    v = row.get(key)
    try:
        return float(v)
    except (TypeError, ValueError):
        return float("nan")


def _i(row, key, default=0):
    v = row.get(key)
    try:
        return int(float(v))
    except (TypeError, ValueError):
        return default


def _fin(x):
    return isinstance(x, float) and math.isfinite(x)


# ── 센서 복원 ────────────────────────────────────────────────────────────────

def _build_gps(row, now):
    pos_health = _i(row, "gps_pos_health")
    motion_health = _i(row, "gps_motion_health")
    lat, lon = _f(row, "gps_lat"), _f(row, "gps_lon")
    crs = _f(row, "gps_course_deg")
    spd = _f(row, "gps_speed_mps")
    pos_age = _f(row, "gps_pos_age_s")
    mot_age = _f(row, "gps_motion_age_s")
    if not (pos_health and _fin(lat) and _fin(lon)):
        pos_health = 0
    pos_ts = (now - pos_age) if (pos_health and _fin(pos_age)) else (now if pos_health else float("nan"))
    mot_ts = (now - mot_age) if (motion_health and _fin(mot_age)) else (now if motion_health else float("nan"))
    return _GpsFromApp(
        lat=lat if pos_health else None,
        lon=lon if pos_health else None,
        pos_ts=pos_ts if pos_health else None,
        course_rad=math.radians(crs) if (motion_health and _fin(crs)) else None,
        speed_mps=spd if (motion_health and _fin(spd)) else None,
        motion_ts=mot_ts if motion_health else None,
        rx_ts=now,
        pos_health=pos_health,
        motion_health=motion_health,
    )


def _build_imu(row, now):
    health = _i(row, "imu_health")
    yaw = _f(row, "imu_yaw_deg")
    gyrz = _f(row, "imu_gyrz_deg_s")
    lax, lay = _f(row, "lin_acc_x_mps2"), _f(row, "lin_acc_y_mps2")
    lin_valid = bool(_i(row, "lin_acc_valid"))
    age = _f(row, "imu_age_s")
    ts = (now - age) if _fin(age) else now
    return _ImuFromApp(
        yaw_rad=math.radians(yaw) if _fin(yaw) else None,
        gyrz_rad_s=math.radians(gyrz) if _fin(gyrz) else None,
        ts=ts,
        lin_acc_x=lax if (lin_valid and _fin(lax)) else None,
        lin_acc_y=lay if (lin_valid and _fin(lay)) else None,
        lin_acc_valid=lin_valid and _fin(lax) and _fin(lay),
        health=health if health else (1 if (_fin(yaw) or _fin(gyrz)) else 0),
    )


def _build_baro(row, now):
    health = _i(row, "baro_health")
    alt = _f(row, "baro_alt_m")
    sink = _f(row, "baro_sink_rate_mps")
    age = _f(row, "baro_age_s")
    ts = (now - age) if _fin(age) else now
    return _BaroFromApp(
        alt_m=alt if _fin(alt) else None,
        sink_rate=sink if _fin(sink) else None,
        rx_ts=ts,
        health=health if health else (1 if _fin(sink) else 0),
    )


# ── replay 본체 ──────────────────────────────────────────────────────────────

RESULT_FIELDS = [
    "time", "state", "control_mode", "l1input_valid", "l1output_valid",
    "fail_reason", "gps_pos_fresh", "gps_motion_fresh", "imu_gyrz_fresh",
    "imu_yaw_fresh", "baro_sink_fresh", "acc_fresh", "dr_current_valid",
    "dr_confidence", "nav_E", "nav_N", "nav_V", "nav_course_deg",
    "yaw_rate_cmd_dps", "left_pwm_est", "right_pwm_est",
    # 진단 추가
    "origin_lock_source", "yaw_saturated", "servo_saturated", "before_mode",
]


def _should_detumble(gyrz_dps):
    if not getattr(config, "DETUMBLE_ENABLE", False):
        return False
    return _fin(gyrz_dps) and abs(gyrz_dps) > config.DETUMBLE_GYRZ_THRESHOLD_DPS


def replay(rows):
    # 시간순 정렬
    rows = sorted(rows, key=lambda r: (_f(r, "monotonic_s") if _fin(_f(r, "monotonic_s")) else 0.0))
    guidance.reset()
    control.reset()
    prev_state = None
    target_set = False
    results = []

    for row in rows:
        now = _f(row, "monotonic_s")
        if not _fin(now):
            continue
        state = _i(row, "flight_state", default=0)

        # 상태 전이 처리 (motorapp 미러)
        if prev_state is None or state != prev_state:
            if state < 4 and prev_state is not None and prev_state != state:
                guidance.reset()
                control.reset()
                target_set = False
        prev_state = state

        # target lock (로그 target_lat/lon)
        if not target_set:
            tlat, tlon = _f(row, "target_lat"), _f(row, "target_lon")
            if _fin(tlat) and _fin(tlon) and abs(tlat) > 1e-9 and abs(tlon) > 1e-9:
                guidance.set_target(tlat, tlon)
                target_set = True

        gps = _build_gps(row, now)
        imu = _build_imu(row, now)
        baro = _build_baro(row, now)

        # origin lock (motorapp.handle_gps 미러): STATE>=4 첫 유효 GPS를 origin으로.
        if (state >= 4 and not guidance._MISSION_t.origin_ready
                and gps.pos_health and _fin(_f(row, "gps_lat"))):
            guidance.lock_origin(_f(row, "gps_lat"), _f(row, "gps_lon"),
                                 source="STATE4_FIRST_GPS")

        gyrz_dps = _f(row, "imu_gyrz_deg_s")

        # DETUMBLING 선점 (motorapp 정책 미러)
        if state >= 3 and _should_detumble(gyrz_dps):
            guidance.UpdateRaw(gps, imu, baro, now)
            guidance.ComputeFreshFlags(now)
            guidance._STATE_t.nav.control_mode = ControlMode.DETUMBLING
            mode = ControlMode.DETUMBLING
            l1in = guidance.L1Input(valid=False, reason="DETUMBLING", control_mode=mode)
            l1out = guidance.ProduceL1Output(l1in)
        else:
            mode = guidance.DecideControlMode(gps, imu, baro, now)
            l1in = guidance.ProduceL1Input(now)
            l1out = guidance.ProduceL1Output(l1in)

        # 서보 PWM 추정 (하드웨어 없음)
        ctrl_in = control.ProduceCtrlInput(l1out, now)
        ctrl_out = control.ProduceCtrlOutput(ctrl_in, gyrz_dps if _fin(gyrz_dps) else float("nan"), now)

        fl = guidance._STATE_t.flags
        nav = guidance._STATE_t.nav
        dr = guidance._STATE_t.dr
        reason = (l1in.reason if not l1in.valid
                  else (l1out.reason if not l1out.valid else "OK"))

        results.append({
            "time": round(now, 3),
            "state": state,
            "control_mode": mode.value,
            "l1input_valid": int(bool(l1in.valid)),
            "l1output_valid": int(bool(getattr(l1out, "valid", False))),
            "fail_reason": reason,
            "gps_pos_fresh": int(fl.gps_pos_fresh),
            "gps_motion_fresh": int(fl.gps_motion_fresh),
            "imu_gyrz_fresh": int(fl.imu_gyrz_fresh),
            "imu_yaw_fresh": int(fl.imu_yaw_fresh),
            "baro_sink_fresh": int(fl.baro_sink_fresh),
            "acc_fresh": int(fl.acc_fresh),
            "dr_current_valid": int(fl.dr_current_valid),
            "dr_confidence": round(dr.confidence, 4),
            "nav_E": round(nav.E, 3) if _fin(nav.E) else "",
            "nav_N": round(nav.N, 3) if _fin(nav.N) else "",
            "nav_V": round(nav.V, 3) if _fin(nav.V) else "",
            "nav_course_deg": round(math.degrees(nav.course), 2) if _fin(nav.course) else "",
            "yaw_rate_cmd_dps": round(ctrl_in.angular_velocity_cmd_deg_s, 3),
            "left_pwm_est": ctrl_out.left_pw,
            "right_pwm_est": ctrl_out.right_pw,
            "origin_lock_source": guidance._MISSION_t.origin_lock_source,
            "yaw_saturated": int(bool(getattr(ctrl_out, "saturated", False))),
            "servo_saturated": int(bool(getattr(ctrl_out, "saturated", False))),
            "before_mode": str(row.get("control_mode", "")).replace("ControlMode.", ""),
        })
    return rows, results


# ── 분석/요약 ────────────────────────────────────────────────────────────────

def _row_dts(times):
    """각 행이 차지하는 시간(다음 행까지, DT_MAX_S로 clamp)."""
    dts = []
    for i in range(len(times)):
        if i + 1 < len(times):
            dt = times[i + 1] - times[i]
            dt = dt if (0.0 <= dt <= DT_MAX_S) else min(max(dt, 0.0), DT_MAX_S)
        else:
            dt = 0.0
        dts.append(dt)
    return dts


def _norm_before(m):
    """Old mode 이름을 새 카테고리로 매핑 (비교용)."""
    m = m.replace("ControlMode.", "")
    if m.startswith("GPS_TRACKING"):
        return m
    if m.startswith("DR_TRACKING") or m.startswith("DR_"):
        return "DR_(legacy/any)"
    return m


def summarize(results, raw_rows, out_md):
    times = [r["time"] for r in results]
    dts = _row_dts(times)
    in34 = [i for i, r in enumerate(results) if r["state"] in (3, 4)]
    total34 = sum(dts[i] for i in in34)

    after_dur = defaultdict(float)
    before_dur = defaultdict(float)
    failreason_dur = defaultdict(float)
    cat_dur = defaultdict(float)   # FAIL / GPS_TRACKING / DR_M / DR_PM / DETUMBLING
    l1in_valid = 0.0
    l1out_valid = 0.0
    yaw_sat = 0.0
    servo_sat = 0.0

    def cat(mode):
        if mode.startswith("GPS_TRACKING"):
            return "GPS_TRACKING"
        if mode.startswith("DR_PM_"):
            return "DR_PM"
        if mode.startswith("DR_M_"):
            return "DR_M"
        return mode  # FAIL / DETUMBLING

    for i in in34:
        r = results[i]
        dt = dts[i]
        after_dur[r["control_mode"]] += dt
        before_dur[_norm_before(r["before_mode"])] += dt
        cat_dur[cat(r["control_mode"])] += dt
        if not r["l1input_valid"]:
            failreason_dur[r["fail_reason"]] += dt
        l1in_valid += dt if r["l1input_valid"] else 0.0
        l1out_valid += dt if r["l1output_valid"] else 0.0
        yaw_sat += dt if r["yaw_saturated"] else 0.0
        servo_sat += dt if r["servo_saturated"] else 0.0

    # before FAIL/GPS durations from logged column
    before_cat = defaultdict(float)
    for i in in34:
        before_cat[_norm_before(results[i]["before_mode"])] += dts[i]

    origin_lock_time = next((r["time"] for r in results if r["origin_lock_source"]), None)
    origin_lock_src = next((r["origin_lock_source"] for r in results if r["origin_lock_source"]), "")
    dr_start_time = next((r["time"] for r in results
                          if r["control_mode"].startswith("DR_")), None)

    def pct(x):
        return (100.0 * x / total34) if total34 > 0 else 0.0

    lines = []
    lines.append("# Guidance Replay Summary\n")
    lines.append(f"- Input rows: {len(results)}  (state 3+4: {len(in34)} rows)")
    lines.append(f"- **State 3+4 duration**: {total34:.1f} s")
    lines.append(f"- Origin lock: t={origin_lock_time}  source={origin_lock_src or '(none)'}")
    lines.append(f"- First DR mode (replay): t={dr_start_time}\n")

    lines.append("## 1. Mode duration — BEFORE (logged) vs AFTER (replay), state 3+4\n")
    lines.append("| Category | BEFORE s (%) | AFTER s (%) |")
    lines.append("|---|---|---|")
    cats = ["FAIL", "GPS_TRACKING", "DR_M", "DR_PM", "DETUMBLING"]
    before_grouped = defaultdict(float)
    for k, v in before_cat.items():
        if k.startswith("GPS_TRACKING"):
            before_grouped["GPS_TRACKING"] += v
        elif k.startswith("DR_"):
            before_grouped["DR_(legacy)"] += v
        else:
            before_grouped[k] += v
    for c in cats:
        b = before_grouped.get(c, 0.0) + (before_grouped.get("DR_(legacy)", 0.0) if c == "DR_PM" else 0.0) * 0
        a = cat_dur.get(c, 0.0)
        lines.append(f"| {c} | {before_grouped.get(c,0.0):.1f} ({pct(before_grouped.get(c,0.0)):.1f}%) | {a:.1f} ({pct(a):.1f}%) |")
    lines.append(f"| DR_(legacy, before only) | {before_grouped.get('DR_(legacy)',0.0):.1f} ({pct(before_grouped.get('DR_(legacy)',0.0)):.1f}%) | — |\n")

    lines.append("## 2. AFTER mode breakdown (state 3+4)\n")
    lines.append("| Mode | duration s | % |")
    lines.append("|---|---|---|")
    for m, d in sorted(after_dur.items(), key=lambda kv: -kv[1]):
        lines.append(f"| {m} | {d:.1f} | {pct(d):.1f}% |")
    lines.append("")

    lines.append("## 3. FAIL / invalid reason duration (state 3+4)\n")
    lines.append("| reason | duration s | % |")
    lines.append("|---|---|---|")
    for rr, d in sorted(failreason_dur.items(), key=lambda kv: -kv[1]):
        lines.append(f"| {rr} | {d:.1f} | {pct(d):.1f}% |")
    lines.append("")

    lines.append("## 4. L1 valid ratio (state 3+4)\n")
    lines.append(f"- L1Input valid: {l1in_valid:.1f} s ({pct(l1in_valid):.1f}%)")
    lines.append(f"- L1Output valid: {l1out_valid:.1f} s ({pct(l1out_valid):.1f}%)\n")

    lines.append("## 5. Saturation (state 3+4)\n")
    lines.append(f"- yaw_rate_cmd saturated: {pct(yaw_sat):.1f}%")
    lines.append(f"- servo command saturated: {pct(servo_sat):.1f}%\n")

    lines.append("## 6. Notes / remaining issues\n")
    fail_before = before_grouped.get("FAIL", 0.0)
    fail_after = cat_dur.get("FAIL", 0.0)
    lines.append(f"- FAIL duration: BEFORE {fail_before:.1f}s → AFTER {fail_after:.1f}s "
                 f"(Δ {fail_after - fail_before:+.1f}s)")
    dr_after = cat_dur.get("DR_M", 0.0) + cat_dur.get("DR_PM", 0.0)
    lines.append(f"- DR (DR_M+DR_PM) duration AFTER: {dr_after:.1f}s ({pct(dr_after):.1f}%)")
    lines.append(f"- See replay_guidance_result.csv for per-cycle detail.")

    with open(out_md, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    return {
        "total34": total34, "fail_before": fail_before, "fail_after": fail_after,
        "dr_after": dr_after, "l1in_pct": pct(l1in_valid), "yaw_sat_pct": pct(yaw_sat),
        "servo_sat_pct": pct(servo_sat), "origin_lock_time": origin_lock_time,
        "origin_lock_src": origin_lock_src, "dr_start_time": dr_start_time,
        "after_dur": dict(after_dur), "failreason": dict(failreason_dur),
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description="Replay guidance modes from a raw_motor log.")
    ap.add_argument("input", nargs="?", default=DEFAULT_INPUT)
    ap.add_argument("--out", default="replay_guidance_result.csv")
    ap.add_argument("--summary", default="replay_summary.md")
    args = ap.parse_args(argv)

    if not os.path.exists(args.input):
        print(f"[replay] input not found: {args.input}", file=sys.stderr)
        return 2

    print(f"[replay] loading {args.input}")
    _, rows = _load_rows(args.input)
    print(f"[replay] {len(rows)} rows")
    raw_rows, results = replay(rows)

    with open(args.out, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=RESULT_FIELDS)
        w.writeheader()
        w.writerows(results)
    print(f"[replay] wrote {args.out} ({len(results)} rows)")

    stats = summarize(results, raw_rows, args.summary)
    print(f"[replay] wrote {args.summary}")
    print(f"[replay] FAIL(state3+4): before {stats['fail_before']:.1f}s "
          f"-> after {stats['fail_after']:.1f}s | DR after {stats['dr_after']:.1f}s "
          f"| L1Input valid {stats['l1in_pct']:.1f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
