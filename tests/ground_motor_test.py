#!/usr/bin/env python3
"""
tests/ground_motor_test.py
지상 정지 상태에서 모터 방향 및 L1 파이프라인 검증.

Section A  ConnectRoMo 직접 sweep  — angular_velocity 입력 대비 PWM/각도 출력
Section B  L1 파이프라인 시나리오  — 하드코딩 위치 → guidance → control → PWM
Section C  실물 서보 sweep          — pigpio 연결 시 실제 암 움직임 (--live)

사용법:
  python tests/ground_motor_test.py           # dry-run (pigpio 불필요)
  python tests/ground_motor_test.py --live    # 실물 서보 구동
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

# Windows cp949 터미널에서도 출력 가능하도록 UTF-8 강제
if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

from Sensor_Motor import control, guidance

# ══════════════════════════════════════════════════════════════════════
# 하드코딩 상수
# ══════════════════════════════════════════════════════════════════════
ORIGIN_LAT = 37.5000
ORIGIN_LON = 127.0000

# 목표: origin에서 북쪽 500m (경로 방향 북향 고정)
_EARTH_R = 6_371_000.0
TARGET_LAT = ORIGIN_LAT + math.degrees(500.0 / _EARTH_R)   # ≈ 37.50450°
TARGET_LON = ORIGIN_LON                                     # 동서 이탈 없음

GROUND_SPEED_MPS = 5.0   # 가상 속도 — L1_distance 분모 보호, 정지 상태 대체
GYRZ_RAD_S = 0.0         # 정지 상태
ALT_M = 100.0            # guidance.ALT_FRESH_AGE 충족용 더미 값

STEP_SEC = 3.0           # Section C 각 단계 지속 시간 (초)

# ══════════════════════════════════════════════════════════════════════
# L1 시나리오 정의
# ══════════════════════════════════════════════════════════════════════
# 경로 북향 고정 (pos_E 조작으로 이탈 방향 제어)
#
# 좌표 관계:
#   cross = unit_N * pos_E − unit_E * pos_N
#   cross > 0  →  오른쪽 이탈  →  nu < 0  →  좌선회
#   cross < 0  →  왼쪽 이탈   →  nu > 0  →  우선회
#
# 헤딩 오차 관계:
#   course 북향(0) 기준, nu > 0 → 우선회
SCENARIOS = [
    {
        "name": "STRAIGHT",
        "desc": "경로 정중앙, 북향 진행  →  선회 없음",
        "pos_N": 0.0,
        "pos_E": 0.0,
        "course_rad": 0.0,
        "expected": "NEUTRAL",
    },
    {
        "name": "TURN_RIGHT",
        "desc": "경로 왼쪽 50m 이탈, 북향 진행  →  우선회 (오른쪽으로 복귀)",
        "pos_N": 0.0,
        "pos_E": -50.0,
        "course_rad": 0.0,
        "expected": "RIGHT",
    },
    {
        "name": "TURN_LEFT",
        "desc": "경로 오른쪽 50m 이탈, 북향 진행  →  좌선회 (왼쪽으로 복귀)",
        "pos_N": 0.0,
        "pos_E": +50.0,
        "course_rad": 0.0,
        "expected": "LEFT",
    },
    {
        "name": "HEADING_ERROR_RIGHT",
        "desc": "경로 위, 서향(−90°) 진행  →  우선회 (목표가 오른쪽)",
        "pos_N": 0.0,
        "pos_E": 0.0,
        "course_rad": -math.pi / 2,
        "expected": "RIGHT",
    },
    {
        "name": "HEADING_ERROR_LEFT",
        "desc": "경로 위, 동향(+90°) 진행  →  좌선회 (목표가 왼쪽)",
        "pos_N": 0.0,
        "pos_E": 0.0,
        "course_rad": +math.pi / 2,
        "expected": "LEFT",
    },
]

# Section C 서보 sweep 단계
LIVE_STEPS = [
    ("NEUTRAL",     0.0,   "중립 — 양쪽 암 80°"),
    ("RIGHT_TURN", +60.0, "우선회 — 오른쪽 암 내려감 / 왼쪽 암 올라감"),
    ("NEUTRAL",     0.0,   "중립 복귀"),
    ("LEFT_TURN",  -60.0, "좌선회 — 왼쪽 암 내려감 / 오른쪽 암 올라감"),
    ("NEUTRAL",     0.0,   "중립 복귀"),
]

# ══════════════════════════════════════════════════════════════════════
# 출력 헬퍼
# ══════════════════════════════════════════════════════════════════════
W = 66


def _header(title: str) -> None:
    print()
    print("=" * W)
    print(f"  {title}")
    print("=" * W)


def _block_open(label: str) -> None:
    fill = max(0, W - 5 - len(label))
    print(f"\n+-- {label} {'-' * fill}+")


def _row(key: str, val: str) -> None:
    print(f"|  {key:<30} {val}")


def _sep(label: str = "") -> None:
    if label:
        inner = f"-- {label} " + "-" * max(0, W - 9 - len(label))
    else:
        inner = "-" * (W - 4)
    print(f"|  {inner}")


def _block_close() -> None:
    print(f"+{'-' * (W - 1)}+")


def _arm_note(angle: float) -> str:
    n = control.NEUTRAL_ARM_DEG
    if angle < n - 0.5:
        return "^ 올라감 (브레이크 해제)"
    if angle > n + 0.5:
        return "v 내려감 (브레이크 증가)"
    return "-- 중립"


def _turn_label(delta: float, tol: float = 0.5) -> str:
    if delta > tol:
        return "RIGHT"
    if delta < -tol:
        return "LEFT"
    return "NEUTRAL"


def _pass_fail(actual: str, expected: str) -> str:
    ok = (actual == expected) if expected != "NEUTRAL" else (actual == "NEUTRAL")
    return "  [PASS]" if ok else f"  [FAIL]  ← 예상: {expected}, 실제: {actual}"


# ══════════════════════════════════════════════════════════════════════
# Section A — ConnectRoMo 직접 sweep
# ══════════════════════════════════════════════════════════════════════
def run_section_a() -> None:
    _header("SECTION A  ConnectRoMo 직접 sweep")
    print(f"  부호 규약: angular_velocity > 0  →  delta_arm > 0  →  우선회")
    print(f"             angular_velocity < 0  →  delta_arm < 0  →  좌선회")
    print(f"  암 규약  : angle > {control.NEUTRAL_ARM_DEG}°  →  내려감(제동)  |  angle < {control.NEUTRAL_ARM_DEG}°  →  올라감(해제)")

    sweep = [
        (0.0,    "중립"),
        (+20.0,  "우선회 약"),
        (-20.0,  "좌선회 약"),
        (+80.0,  "우선회 강"),
        (-80.0,  "좌선회 강"),
        (+160.0, "우선회 최대 (포화 경계)"),
        (-160.0, "좌선회 최대 (포화 경계)"),
    ]

    for yr, label in sweep:
        lp, rp, la, ra, delta = control.ConnectRoMo(yr)
        _block_open(f"angular_velocity = {yr:+.1f} deg/s  [{label}]")
        _row("delta_arm_deg",    f"{delta:+.2f}°")
        _row("left_angle_deg",   f"{la:.2f}°   {_arm_note(la)}")
        _row("right_angle_deg",  f"{ra:.2f}°   {_arm_note(ra)}")
        _row("left_pw",          f"{lp} us   (GPIO {control.PARAFOIL_LEFT_MOTOR_PIN})")
        _row("right_pw",         f"{rp} us   (GPIO {control.PARAFOIL_RIGHT_MOTOR_PIN})")
        _block_close()


# ══════════════════════════════════════════════════════════════════════
# Section B — L1 파이프라인 시나리오
# ══════════════════════════════════════════════════════════════════════
def _make_l1_input(pos_N: float, pos_E: float, course_rad: float) -> guidance.L1Input:
    """하드코딩된 상태로 L1Input 생성. 모든 품질 FRESH 강제."""
    inp = guidance.L1Input()
    inp.N = pos_N
    inp.E = pos_E
    inp.course = course_rad
    inp.V = GROUND_SPEED_MPS
    inp.gyrz = GYRZ_RAD_S
    inp.alt = ALT_M
    inp.pos_quality    = guidance.SensorQuality.FRESH
    inp.motion_quality = guidance.SensorQuality.FRESH
    inp.gyrz_quality   = guidance.SensorQuality.FRESH
    inp.alt_quality    = guidance.SensorQuality.FRESH
    inp.origin_lat  = ORIGIN_LAT
    inp.origin_lon  = ORIGIN_LON
    inp.target_lat  = TARGET_LAT
    inp.target_lon  = TARGET_LON
    inp.control_mode = guidance.ControlMode.NOMINAL_FEEDFORWARD
    return inp


def run_section_b() -> None:
    _header("SECTION B  L1 파이프라인 시나리오")
    print(f"  Origin : ({ORIGIN_LAT:.4f}, {ORIGIN_LON:.4f})")
    print(f"  Target : ({TARGET_LAT:.6f}, {TARGET_LON:.4f})  [북쪽 500m 고정]")
    print(f"  Speed  : {GROUND_SPEED_MPS} m/s (가상)  |  gyrz = {GYRZ_RAD_S} rad/s")
    print(f"  Mode   : NOMINAL_FEEDFORWARD (GPS only, gyro 피드백 없음)")

    total = 0
    passed = 0

    for sc in SCENARIOS:
        now = time.monotonic()

        # L1 guidance
        l1_in = _make_l1_input(sc["pos_N"], sc["pos_E"], sc["course_rad"])
        g_out = guidance.ProduceL1Output(
            l1_in,
            guidance.ControlMode.NOMINAL_FEEDFORWARD,
            ORIGIN_LAT, ORIGIN_LON,
            TARGET_LAT, TARGET_LON,
            now,
        )

        # Control (fresh controller per scenario to avoid PID accumulation)
        ctl = control.MakeCtrler()
        ctrl_in = control.ProduceCtrlInput(g_out, now)
        cmd = control.ProduceCtrlOutput(ctl, ctrl_in, math.nan, now)

        actual = _turn_label(cmd.delta_arm_deg)
        result = _pass_fail(actual, sc["expected"])
        total += 1
        if "PASS" in result:
            passed += 1

        _block_open(f"SCENARIO: {sc['name']}")

        _row("설명", sc["desc"])
        _row("입력 pos_N",  f"{sc['pos_N']:+.1f} m")
        easting_note = (
            "(경로 오른쪽 이탈)" if sc["pos_E"] > 0
            else "(경로 왼쪽 이탈)" if sc["pos_E"] < 0
            else "(경로 중앙)"
        )
        _row("입력 pos_E",  f"{sc['pos_E']:+.1f} m  {easting_note}")
        _row("입력 course", f"{math.degrees(sc['course_rad']):+.1f}°")

        _sep("L1 Output")
        _row("nominal",         str(g_out.nominal))
        _row("nu_deg",          f"{math.degrees(g_out.angle_to_turn):+.3f}°")
        _row("  nu1 (xtrack)",  f"{math.degrees(g_out.nu1):+.3f}°  crosstrack 기여")
        _row("  nu2 (heading)", f"{math.degrees(g_out.nu2):+.3f}°  heading error 기여")
        _row("crossTrack_m",    f"{g_out.crossTrack:+.2f} m")
        _row("alongTrack_m",    f"{g_out.alongTrack:+.2f} m")
        _row("L1_distance_m",   f"{g_out.L1_distance:.2f} m")
        _row("angular_velocity_cmd", f"{g_out.yaw_rate_cmd:+.4f} rad/s  "
                                    f"({math.degrees(g_out.yaw_rate_cmd):+.2f} deg/s)")
        _row("lat_acc_cmd",     f"{g_out.lat_acc_cmd_mps2:+.4f} m/s²")

        _sep("Control Output")
        _row("delta_ff_deg",    f"{cmd.delta_ff_deg:+.3f}°  (K_FF × angular_velocity_cmd)")
        _row("delta_pid_deg",   f"{cmd.delta_pid_deg:+.3f}°  (현재 K_P=K_I=K_D=0)")
        _row("delta_arm_deg",   f"{cmd.delta_arm_deg:+.3f}°  (ff + pid)")
        _row("left_angle_deg",  f"{cmd.left_angle_deg:.2f}°   {_arm_note(cmd.left_angle_deg)}")
        _row("right_angle_deg", f"{cmd.right_angle_deg:.2f}°   {_arm_note(cmd.right_angle_deg)}")
        _row("left_pw",         f"{cmd.left_pw} us")
        _row("right_pw",        f"{cmd.right_pw} us")
        _row("saturated",       str(cmd.saturated))
        _row("mode",            cmd.mode)

        _sep("판정")
        _row(f"예상: {sc['expected']:<8}  실제: {actual:<8}", result)
        _block_close()

    print(f"\n  결과: {passed}/{total} PASS")


# ══════════════════════════════════════════════════════════════════════
# Section C — 실물 서보 sweep
# ══════════════════════════════════════════════════════════════════════
def run_section_c(pi) -> None:
    _header("SECTION C  실물 서보 sweep  [LIVE]")
    print(f"  각 단계 지속 시간 : {STEP_SEC}s")
    print(f"  GPIO  Left={control.PARAFOIL_LEFT_MOTOR_PIN}  Right={control.PARAFOIL_RIGHT_MOTOR_PIN}")
    print(f"  부호 규약: angular_velocity > 0 → 우선회 / angular_velocity < 0 → 좌선회")

    ctl = control.MakeCtrler()

    for step_name, yr, step_desc in LIVE_STEPS:
        now = time.monotonic()
        ctrl_in = control.CtrlInput(
            angular_velocity_cmd_deg_s=yr,
            lat_acc_cmd_mps2=0.0,
            ground_speed_mps=GROUND_SPEED_MPS,
            valid=True,
            timestamp=now,
        )
        cmd = control.ProduceCtrlOutput(ctl, ctrl_in, math.nan, now)

        _block_open(f"STEP: {step_name}")
        _row("설명",            step_desc)
        _row("angular_velocity_cmd_deg_s", f"{yr:+.1f} deg/s")
        _row("delta_arm_deg",  f"{cmd.delta_arm_deg:+.2f}°")
        _row("left_angle_deg", f"{cmd.left_angle_deg:.2f}°   {_arm_note(cmd.left_angle_deg)}")
        _row("right_angle_deg",f"{cmd.right_angle_deg:.2f}°   {_arm_note(cmd.right_angle_deg)}")
        _row("left_pw",        f"{cmd.left_pw} us  →  GPIO {control.PARAFOIL_LEFT_MOTOR_PIN}")
        _row("right_pw",       f"{cmd.right_pw} us  →  GPIO {control.PARAFOIL_RIGHT_MOTOR_PIN}")
        _block_close()

        control.ProducePulse(pi, cmd)

        for remaining in range(int(STEP_SEC), 0, -1):
            print(f"\r  실행 중... {remaining}s 남음  ", end="", flush=True)
            time.sleep(1.0)
        print()

    print("\n  sweep 완료. 서보 중립 유지.")


# ══════════════════════════════════════════════════════════════════════
# main
# ══════════════════════════════════════════════════════════════════════
def main() -> None:
    parser = argparse.ArgumentParser(
        description="지상 정지 상태 모터 방향 검증 스크립트"
    )
    parser.add_argument(
        "--live",
        action="store_true",
        help="pigpio 연결 후 실물 서보 sweep 실행",
    )
    args = parser.parse_args()

    border = "=" * W
    print()
    print(f"+{border}+")
    print(f"|{'CANSAT Ground Motor Direction Test':^{W}}|")
    print(f"+{border}+")

    run_section_a()
    run_section_b()

    if args.live:
        pi = control.init_control()
        if pi is None:
            print(f"\n  [WARNING] pigpio 연결 실패 — Section C 생략.")
            print(f"            pigpio 데몬 실행 여부 확인: sudo pigpiod")
        else:
            input(f"\n  [Enter] 키를 누르면 실물 서보 sweep 시작합니다... ")
            run_section_c(pi)
    else:
        print()
        print(f"  [dry-run]  Section C 생략. 실물 서보 구동 시 --live 옵션 사용.")

    print()
    print("═" * W)
    print("  검증 완료")
    print("═" * W)
    print()


if __name__ == "__main__":
    main()
