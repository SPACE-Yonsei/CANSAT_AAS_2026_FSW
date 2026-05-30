#!/usr/bin/env python3
"""
tests/gps_motor_test.py
GPS 8필드 페이로드 → handle_gps() → guidance → control 전체 파이프라인 검증.

검증 대상 입력 필드:
  lat, lon, pos_health, pos_ts,
  course_deg (= 사용자 표기상 course_seg), speed_mps,
  motion_health, motion_ts

이 값들이 handle_gps() → guidance.UpdateRaws() → DecideControlMode()
→ ProduceL1Input() → ProduceL1Output() → ProduceCtrlOutput() 파이프라인을
거쳐 모터 방향(좌/우/중립 PWM)으로 올바르게 변환되는지 확인한다.

사용법:
  python tests/gps_motor_test.py
"""
from __future__ import annotations

import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

import Sensor_Motor.motorapp as motorapp
from Sensor_Motor import control, guidance
from Sensor_Motor.sensor_types import _GpsFromApp, _ImuFromApp, _BaroFromApp

# ══════════════════════════════════════════════════════════════════════
# 상수
# ══════════════════════════════════════════════════════════════════════
_EARTH_R = 6_371_000.0

ORIGIN_LAT = 37.558658   # 실제 현재 위치
ORIGIN_LON = 126.945271
TARGET_LAT = ORIGIN_LAT + math.degrees(500.0 / _EARTH_R)   # 북쪽 500 m
TARGET_LON = ORIGIN_LON

GROUND_SPEED_MPS = 5.0

# 200 m 이탈 → nu ≈ 21.8°  > NU_DEADBAND(15°) 이므로 모터 작동 보장
# (500m 거리에서 deadband 최소 crosstrack = 134m → 200m 사용)
_LON_PER_200M = math.degrees(
    200.0 / (_EARTH_R * math.cos(math.radians(ORIGIN_LAT)))
)

W = 72

# ══════════════════════════════════════════════════════════════════════
# 시나리오 정의
# ══════════════════════════════════════════════════════════════════════
SCENARIOS = [
    {
        "name": "STRAIGHT",
        "desc": "경로 정중앙, 북향 진행 → 선회 없음 (nu < deadband)",
        "lat": ORIGIN_LAT,
        "lon": ORIGIN_LON,
        "course_deg": 0.0,
        "speed_mps": GROUND_SPEED_MPS,
        "pos_health": 1,
        "motion_health": 1,
        "expected": "NEUTRAL",
    },
    {
        "name": "TURN_RIGHT",
        "desc": "서쪽 200 m 이탈, 북향 → 우선회 (target이 오른쪽)",
        "lat": ORIGIN_LAT,
        "lon": ORIGIN_LON - _LON_PER_200M,
        "course_deg": 0.0,
        "speed_mps": GROUND_SPEED_MPS,
        "pos_health": 1,
        "motion_health": 1,
        "expected": "RIGHT",
    },
    {
        "name": "TURN_LEFT",
        "desc": "동쪽 200 m 이탈, 북향 → 좌선회 (target이 왼쪽)",
        "lat": ORIGIN_LAT,
        "lon": ORIGIN_LON + _LON_PER_200M,
        "course_deg": 0.0,
        "speed_mps": GROUND_SPEED_MPS,
        "pos_health": 1,
        "motion_health": 1,
        "expected": "LEFT",
    },
    {
        "name": "HEADING_ERR_RIGHT",
        "desc": "경로 위, 서향(-90°) 진행 → 우선회 (목표가 오른쪽)",
        "lat": ORIGIN_LAT,
        "lon": ORIGIN_LON,
        "course_deg": -90.0,
        "speed_mps": GROUND_SPEED_MPS,
        "pos_health": 1,
        "motion_health": 1,
        "expected": "RIGHT",
    },
    {
        "name": "HEADING_ERR_LEFT",
        "desc": "경로 위, 동향(+90°) 진행 → 좌선회 (목표가 왼쪽)",
        "lat": ORIGIN_LAT,
        "lon": ORIGIN_LON,
        "course_deg": 90.0,
        "speed_mps": GROUND_SPEED_MPS,
        "pos_health": 1,
        "motion_health": 1,
        "expected": "LEFT",
    },
    {
        "name": "POS_HEALTH_0",
        "desc": "pos_health=0 → 위치 무효 → mode=FAIL → 모터 중립",
        "lat": ORIGIN_LAT,
        "lon": ORIGIN_LON,
        "course_deg": 90.0,
        "speed_mps": GROUND_SPEED_MPS,
        "pos_health": 0,
        "motion_health": 1,
        "expected": "NEUTRAL",
    },
    {
        "name": "MOTION_HEALTH_0",
        "desc": "motion_health=0 → 속도/방향 무효 → GPS_TRACKING 불가 → FAIL → 중립",
        "lat": ORIGIN_LAT,
        "lon": ORIGIN_LON,
        "course_deg": 0.0,
        "speed_mps": GROUND_SPEED_MPS,
        "pos_health": 1,
        "motion_health": 0,
        "expected": "NEUTRAL",
    },
]


# ══════════════════════════════════════════════════════════════════════
# 헬퍼
# ══════════════════════════════════════════════════════════════════════

def _reset_for_scenario() -> None:
    """guidance + motorapp 캐시 초기화 후 origin / target 수동 설정."""
    guidance.reset()
    mi = guidance._MISSION_t
    mi.origin_lat   = ORIGIN_LAT
    mi.origin_lon   = ORIGIN_LON
    mi.origin_ready = True
    guidance.set_target(TARGET_LAT, TARGET_LON)
    motorapp._CACHE_t.latest_gps  = _GpsFromApp()
    motorapp._CACHE_t.latest_imu  = _ImuFromApp()
    motorapp._CACHE_t.latest_baro = _BaroFromApp()


def _make_gps_payload(sc: dict, ts: float) -> str:
    """GPS IPC 페이로드 문자열 생성.
    형식: lat,lon,pos_health,pos_ts,course_deg,speed_mps,motion_health,motion_ts
    """
    return (
        f"{sc['lat']:.8f},{sc['lon']:.8f},"
        f"{sc['pos_health']},{ts:.4f},"
        f"{sc['course_deg']:.2f},{sc['speed_mps']:.2f},"
        f"{sc['motion_health']},{ts:.4f}"
    )


def _turn_label(delta: float, tol: float = 0.5) -> str:
    if delta > tol:
        return "RIGHT"
    if delta < -tol:
        return "LEFT"
    return "NEUTRAL"


def _pass_fail(actual: str, expected: str) -> str:
    return "[PASS]" if actual == expected else f"[FAIL]  expected={expected}, actual={actual}"


def _arm_note(angle: float) -> str:
    n = control.NEUTRAL_ARM_DEG
    if angle < n - 0.5:
        return "^ 올라감 (제동 해제)"
    if angle > n + 0.5:
        return "v 내려감 (제동 증가)"
    return "-- 중립"


def _hdr(t: str) -> None:
    print()
    print("=" * W)
    print(f"  {t}")
    print("=" * W)


def _row(k: str, v: str) -> None:
    print(f"  {k:<36} {v}")


def _sep(label: str = "") -> None:
    inner = f"-- {label} " + "-" * max(0, W - 8 - len(label)) if label else "-" * (W - 4)
    print(f"  {inner}")


# ══════════════════════════════════════════════════════════════════════
# 메인 실행
# ══════════════════════════════════════════════════════════════════════

def run() -> None:
    _hdr("GPS Motor Pipeline Test  —  GPS 8필드 → 모터 방향 검증")
    print(f"  Origin : ({ORIGIN_LAT:.4f}°N, {ORIGIN_LON:.4f}°E)")
    print(f"  Target : ({TARGET_LAT:.6f}°N, {TARGET_LON:.4f}°E)  [origin 북쪽 500 m]")
    print()
    print(f"  GPS 페이로드 필드 순서:")
    print(f"    lat | lon | pos_health | pos_ts | course_deg | speed_mps | motion_health | motion_ts")

    total = 0
    passed = 0

    for sc in SCENARIOS:
        _reset_for_scenario()
        now = time.monotonic()

        # ── 1. GPS 페이로드 → handle_gps() ───────────────────────────────────
        payload = _make_gps_payload(sc, ts=now)
        motorapp.handle_gps(payload)

        # ── 2. Guidance 파이프라인 ────────────────────────────────────────────
        snap   = motorapp._cache_snapshot()
        guidance.UpdateRaws(snap.latest_gps, snap.latest_imu, snap.latest_baro, now)
        mode   = guidance.DecideControlMode(now)
        l1_in  = guidance.ProduceL1Input(now)
        l1_out = guidance.ProduceL1Output(l1_in)

        # ── 3. Control 파이프라인 ─────────────────────────────────────────────
        ctl      = control.MakeCtrler()
        ctrl_in  = control.ProduceCtrlInput(l1_out, now)
        cmd      = control.ProduceCtrlOutput(ctl, ctrl_in, math.nan, now)

        actual  = _turn_label(cmd.delta_arm_deg)
        verdict = _pass_fail(actual, sc["expected"])
        total  += 1
        if "PASS" in verdict:
            passed += 1

        # ── 출력 ─────────────────────────────────────────────────────────────
        print()
        name_w = max(0, W - 15 - len(sc["name"]))
        print(f"+-- SCENARIO: {sc['name']}  {'─' * name_w}+")

        _row("설명", sc["desc"])

        _sep("GPS 입력 (IPC 페이로드 문자열)")
        print(f"  payload = {payload}")
        _row("lat", f"{sc['lat']:.8f}°")
        e_offset_m = (sc['lon'] - ORIGIN_LON) * math.radians(1) * _EARTH_R * math.cos(math.radians(ORIGIN_LAT))
        _row("lon", f"{sc['lon']:.8f}°  ({e_offset_m:+.0f} m  E/W from origin)")
        _row("pos_health", str(sc["pos_health"]))
        _row("pos_ts", f"(현재 monotonic 사용)  freshness OK")
        _row("course_deg  [= course_seg]", f"{sc['course_deg']:+.1f}°  (0=북, +90=동, -90=서)")
        _row("speed_mps", f"{sc['speed_mps']:.1f} m/s")
        _row("motion_health", str(sc["motion_health"]))
        _row("motion_ts", f"(현재 monotonic 사용)  freshness OK")

        _sep("Guidance 파이프라인 결과")
        _row("DecideControlMode()", mode.value)
        _row("ProduceL1Input().valid", str(l1_in.valid))
        _row("ProduceL1Input().reason", l1_in.reason)
        if l1_in.valid:
            _row("  현재 위치 N (북, m)", f"{l1_in.N:+.2f}")
            _row("  현재 위치 E (동, m)", f"{l1_in.E:+.2f}")
            _row("  목표 위치 N (북, m)", f"{l1_in.target_N:+.2f}")
            _row("  목표 위치 E (동, m)", f"{l1_in.target_E:+.2f}")
        _row("ProduceL1Output().nominal", str(l1_out.nominal))
        if l1_out.nominal:
            _row("  target_bearing", f"{math.degrees(l1_out.target_bearing):+.2f}°")
            _row("  nu (선회 오차 각도)", f"{math.degrees(l1_out.nu):+.2f}°")
            _row("  distance_to_target", f"{l1_out.distance_to_target:.1f} m")
            _row("  yaw_rate_cmd", f"{math.degrees(l1_out.yaw_rate_cmd):+.3f} deg/s")
        else:
            _row("  reason", l1_out.reason)

        _sep("Control 출력 (모터 명령)")
        _row("ctrl_in.valid", str(ctrl_in.valid))
        _row("delta_arm_deg", f"{cmd.delta_arm_deg:+.3f}°  →  방향: {actual}")
        _row("left_angle_deg", f"{cmd.left_angle_deg:.2f}°   {_arm_note(cmd.left_angle_deg)}")
        _row("right_angle_deg", f"{cmd.right_angle_deg:.2f}°   {_arm_note(cmd.right_angle_deg)}")
        _row(f"left_pw  (GPIO {control.PARAFOIL_LEFT_MOTOR_PIN})", f"{cmd.left_pw} µs")
        _row(f"right_pw (GPIO {control.PARAFOIL_RIGHT_MOTOR_PIN})", f"{cmd.right_pw} µs")
        _row("saturated", str(cmd.saturated))

        _sep("판정")
        _row(f"예상: {sc['expected']:<10}  실제: {actual:<10}", verdict)
        print(f"+{'─' * (W - 1)}+")

    print()
    print(f"  결과: {passed}/{total} PASS")
    print()
    print("═" * W)
    print("  검증 완료")
    print("═" * W)
    print()


if __name__ == "__main__":
    run()
