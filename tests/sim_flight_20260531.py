"""2026-05-31 비행 로그 재현 시뮬레이션.

raw_motor.csv의 센서 입력을 현재 guidance/control 코드에 그대로 주입하여
수정된 코드(Fix 1~4)가 적용됐을 때 어떤 제어가 이루어졌을지 재현한다.

출력:
  tests/sim_result_20260531.csv        — 사이클별 상세 결과
  tests/sim_summary_20260531.md        — 요약 리포트
"""
from __future__ import annotations

import csv
import math
import sys
import time as pytime
from pathlib import Path
from types import SimpleNamespace

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from Sensor_Motor import guidance, control  # noqa: E402
from lib import config  # noqa: E402

INPUT_CSV   = Path(r"C:\Users\ms kang\Documents\카카오톡 받은 파일\raw_motor.csv")
OUT_CSV     = Path(__file__).parent / "sim_result_20260531.csv"
OUT_MD      = Path(__file__).parent / "sim_summary_20260531.md"

TARGET_LAT  = 37.52678
TARGET_LON  = 126.618569


# ── 헬퍼 ─────────────────────────────────────────────────────────────────────

def _f(row, col, default=math.nan):
    v = row.get(col, "")
    if v in ("", "nan", "None", None):
        return default
    try:
        return float(v)
    except (ValueError, TypeError):
        return default


def _b(row, col):
    return str(row.get(col, "0")).strip() in ("1", "True", "true")


def _haversine_m(lat1, lon1, lat2, lon2):
    R = 6_371_000.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1))*math.cos(math.radians(lat2))*math.sin(dlon/2)**2
    return 2*R*math.asin(math.sqrt(a))


# ── GPS duck-type 객체 ────────────────────────────────────────────────────────

def _make_gps(row):
    g = SimpleNamespace()
    g.lat          = _f(row, "gps_lat")
    g.lon          = _f(row, "gps_lon")
    raw_course     = _f(row, "gps_course_deg")
    g.course_rad   = math.radians(raw_course) if math.isfinite(raw_course) else math.nan
    g.speed_mps    = _f(row, "gps_speed_mps")
    g.pos_health   = int(_f(row, "gps_pos_health",  0))
    g.motion_health= int(_f(row, "gps_motion_health", 0))
    g.pos_ts       = _f(row, "monotonic_s")
    g.motion_ts    = _f(row, "monotonic_s") if g.motion_health else math.nan
    return g


def _make_imu(row):
    i = SimpleNamespace()
    roll  = _f(row, "imu_roll_deg")
    pitch = _f(row, "imu_pitch_deg")
    yaw   = _f(row, "imu_yaw_deg")
    gyrz  = _f(row, "imu_gyrz_deg_s")
    i.roll_rad    = math.radians(roll)  if math.isfinite(roll)  else math.nan
    i.pitch_rad   = math.radians(pitch) if math.isfinite(pitch) else math.nan
    i.yaw_rad     = math.radians(yaw)   if math.isfinite(yaw)   else math.nan
    i.gyrz_rad_s  = math.radians(gyrz)  if math.isfinite(gyrz)  else math.nan
    i.health      = int(_f(row, "imu_health", 0))
    i.ts          = _f(row, "monotonic_s")
    lax, lay, laz = _f(row,"lin_acc_x_mps2"), _f(row,"lin_acc_y_mps2"), _f(row,"lin_acc_z_mps2")
    i.lin_acc_x     = lax
    i.lin_acc_y     = lay
    i.lin_acc_z     = laz
    i.lin_acc_valid = bool(_b(row, "lin_acc_valid"))
    i.yaw_offset_deg = 0.0
    return i


def _make_baro(row):
    b = SimpleNamespace()
    b.alt_m     = _f(row, "baro_alt_m")
    b.sink_rate = _f(row, "baro_sink_rate_mps")
    b.health    = int(_f(row, "baro_health", 0))
    b.rx_ts     = _f(row, "monotonic_s")
    b.valid     = b.health == 1 and math.isfinite(b.alt_m)
    return b


# ── 시뮬레이션 메인 ───────────────────────────────────────────────────────────

def run():
    guidance.reset()
    guidance.set_target(TARGET_LAT, TARGET_LON)

    rows_in = []
    with INPUT_CSV.open(newline="", encoding="utf-8-sig") as f:
        for r in csv.DictReader(f):
            rows_in.append(r)

    print(f"입력 행 수: {len(rows_in)}")

    results = []
    mode_counts: dict[str, int] = {}
    detumble_exits = 0
    detumble_active = False

    # DETUMBLING 내부 상태 직접 관리 (모듈 변수 참조)
    guidance._DETUMBLE_ACTIVE      = False
    guidance._DETUMBLE_EXIT_START  = math.nan

    for row in rows_in:
        mono  = _f(row, "monotonic_s")
        state = int(_f(row, "flight_state", 0))
        gps   = _make_gps(row)
        imu   = _make_imu(row)
        baro  = _make_baro(row)

        guidance.UpdateRaws(gps, imu, baro, mono)
        guidance.TryInitStateFromPosOnly(mono)

        # DETUMBLING 판정
        gz_dps = math.degrees(imu.gyrz_rad_s) if math.isfinite(imu.gyrz_rad_s) else math.nan
        should_detumble = False
        if math.isfinite(gz_dps):
            abs_gz = abs(gz_dps)
            if guidance._DETUMBLE_ACTIVE:
                if abs_gz <= config.DETUMBLE_EXIT_THRESHOLD_DPS:
                    if not math.isfinite(guidance._DETUMBLE_EXIT_START):
                        guidance._DETUMBLE_EXIT_START = mono
                    if mono - guidance._DETUMBLE_EXIT_START < config.DETUMBLE_EXIT_HOLD_S:
                        should_detumble = True
                    else:
                        guidance._DETUMBLE_ACTIVE = False
                        guidance._DETUMBLE_EXIT_START = math.nan
                        detumble_exits += 1
                else:
                    guidance._DETUMBLE_EXIT_START = math.nan
                    should_detumble = True
            elif abs_gz >= config.DETUMBLE_GYRZ_THRESHOLD_DPS:
                guidance._DETUMBLE_ACTIVE = True
                guidance._DETUMBLE_EXIT_START = math.nan
                should_detumble = True

        if should_detumble:
            ctrl_out = control.ProduceDetumbleOutput(mono, gz_dps)
            sim_mode = "DETUMBLING"
            l1_valid = False
            dist_m   = math.nan
            nu_deg   = math.nan
        else:
            mode = guidance.DecideControlMode(mono)
            sim_mode = mode.value

            if mode in (guidance.ControlMode.GPS_TRACKING_CLOSED,
                        guidance.ControlMode.GPS_TRACKING_OPEN,
                        guidance.ControlMode.DR_TRACKING_CLOSED,
                        guidance.ControlMode.DR_TRACKING_OPEN):
                l1_in  = guidance.ProduceL1Input(mono)
                l1_out = guidance.ProduceL1Output(l1_in)
                l1_valid = l1_out.control_valid
                dist_m   = l1_out.distance_to_target if l1_valid else math.nan
                nu_deg   = math.degrees(l1_out.nu) if l1_valid and math.isfinite(l1_out.nu) else math.nan
                # 제어 출력 (단순화: yaw_rate_cmd → delta 변환)
                delta_cmd = math.degrees(l1_out.yaw_rate_cmd) * 0.1 if l1_valid else 0.0
                delta_cmd = max(-control.DELTA_ARM_MAX_DEG, min(control.DELTA_ARM_MAX_DEG, delta_cmd))
                lp, rp, la, ra, da = control.ConnectRoMo(delta_cmd)
                ctrl_out = control.CtrlOutput(
                    timestamp=mono, left_pw=lp, right_pw=rp,
                    left_angle_deg=la, right_angle_deg=ra,
                    delta_arm_deg=da, valid=l1_valid,
                )
            else:
                l1_valid = False
                dist_m   = math.nan
                nu_deg   = math.nan
                ctrl_out = control.CtrlOutput(timestamp=mono, valid=False)

        mode_counts[sim_mode] = mode_counts.get(sim_mode, 0) + 1

        # 실제 비행과의 비교
        actual_mode = row.get("control_mode", "")
        origin_ready = guidance._MISSION_t.origin_ready
        dr_valid     = guidance.dr_is_valid(guidance._STATE_t.dr)

        results.append({
            "timestamp":     row.get("timestamp", ""),
            "mono":          f"{mono:.3f}",
            "flight_state":  state,
            "baro_alt_m":    f"{baro.alt_m:.2f}" if math.isfinite(baro.alt_m) else "nan",
            "actual_mode":   actual_mode,
            "sim_mode":      sim_mode,
            "mode_changed":  "YES" if sim_mode != actual_mode.replace("ControlMode.", "") else "",
            "origin_ready":  "1" if origin_ready else "0",
            "dr_valid":      "1" if dr_valid else "0",
            "l1_valid":      "1" if l1_valid else "0",
            "dist_to_tgt_m": f"{dist_m:.1f}" if math.isfinite(dist_m) else "nan",
            "nu_deg":        f"{nu_deg:.1f}" if math.isfinite(nu_deg) else "nan",
            "left_pw":       ctrl_out.left_pw,
            "right_pw":      ctrl_out.right_pw,
            "delta_arm_deg": f"{ctrl_out.delta_arm_deg:.1f}",
            "gyrz_dps":      f"{gz_dps:.2f}" if math.isfinite(gz_dps) else "nan",
        })

    # ── CSV 저장 ──────────────────────────────────────────────────────────────
    if results:
        with OUT_CSV.open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(results[0].keys()))
            w.writeheader()
            w.writerows(results)
        print(f"결과 저장: {OUT_CSV}")

    # ── 요약 통계 ─────────────────────────────────────────────────────────────
    total = len(results)
    homing_modes = {"GPS_TRACKING_CLOSED","GPS_TRACKING_OPEN",
                    "DR_TRACKING_CLOSED","DR_TRACKING_OPEN"}
    homing_rows  = [r for r in results if r["sim_mode"] in homing_modes]
    detumble_rows= [r for r in results if r["sim_mode"] == "DETUMBLING"]
    fail_rows    = [r for r in results if r["sim_mode"] == "FAIL"]
    origin_set_t = next((r["timestamp"] for r in results if r["origin_ready"]=="1"), "미설정")
    dr_set_t     = next((r["timestamp"] for r in results if r["dr_valid"]=="1"), "미설정")

    # 첫 homing 진입 시각
    homing_first_t = homing_rows[0]["timestamp"] if homing_rows else "미진입"
    homing_first_alt = homing_rows[0]["baro_alt_m"] if homing_rows else "—"

    # 마지막 거리
    dist_vals = [float(r["dist_to_tgt_m"]) for r in homing_rows
                 if r["dist_to_tgt_m"] != "nan"]
    final_dist = f"{dist_vals[-1]:.1f} m" if dist_vals else "—"
    min_dist   = f"{min(dist_vals):.1f} m" if dist_vals else "—"

    # detumble 탈출 횟수
    # cross-track = nu_deg 기반 (L1 geometry: xte ≈ dist * sin(nu))
    xte_vals = []
    for r in homing_rows:
        d = r["dist_to_tgt_m"]
        nu = r["nu_deg"]
        if d != "nan" and nu != "nan":
            xte = float(d) * math.sin(math.radians(float(nu)))
            xte_vals.append(abs(xte))
    xte_rms = math.sqrt(sum(x**2 for x in xte_vals)/len(xte_vals)) if xte_vals else math.nan

    # mode 비율
    def pct(n):
        return f"{100*n/total:.1f}%" if total else "0%"

    lines = [
        "# 2026-05-31 비행 시뮬레이션 결과 (Fix 1~4 적용)",
        "",
        "## 제어 모드 분포 (전체 사이클)",
        "",
        f"| 모드 | 사이클 수 | 비율 |",
        f"|------|-----------|------|",
    ]
    for m, cnt in sorted(mode_counts.items(), key=lambda x: -x[1]):
        lines.append(f"| {m} | {cnt} | {pct(cnt)} |")

    lines += [
        "",
        "## 핵심 이벤트 타임라인",
        "",
        f"| 이벤트 | 시각 | 비고 |",
        f"|--------|------|------|",
        f"| Origin 설정 | {origin_set_t} | Fix 1 효과 |",
        f"| DR anchor 설정 | {dr_set_t} | Fix 1 효과 |",
        f"| HOMING 첫 진입 | {homing_first_t} | 고도 {homing_first_alt} m |",
        f"| DETUMBLING 탈출 횟수 | {detumble_exits}회 | Fix 3·4 효과 |",
        "",
        "## 제어 성능 (HOMING 구간)",
        "",
        f"| 지표 | 값 | 비고 |",
        f"|------|-----|------|",
        f"| HOMING 진입 사이클 | {len(homing_rows)} | 전체의 {pct(len(homing_rows))} |",
        f"| 최종 목표 거리 | {final_dist} | |",
        f"| 최근접 거리 | {min_dist} | |",
        f"| Cross-track RMS | {f'{xte_rms:.1f} m' if math.isfinite(xte_rms) else '—'} | |",
        "",
        "## 구 코드 vs 수정 코드 비교",
        "",
        f"| 항목 | 구 코드 (실제 비행) | 수정 코드 (시뮬레이션) |",
        f"|------|-------------------|----------------------|",
        f"| Origin 설정 | 미설정 (전 구간 nan) | {origin_set_t} |",
        f"| DR anchor | 미설정 | {dr_set_t} |",
        f"| HOMING 진입 | 0회 | {len(homing_rows)} 사이클 |",
        f"| DETUMBLING 탈출 | 0회 | {detumble_exits}회 |",
        f"| FAIL 사이클 | {len(rows_in)} ({pct(len(rows_in))}) | {len(fail_rows)} ({pct(len(fail_rows))}) |",
    ]

    md = "\n".join(lines)
    OUT_MD.write_text(md, encoding="utf-8")
    print(f"요약 저장: {OUT_MD}")
    print()
    print(md)


if __name__ == "__main__":
    run()
