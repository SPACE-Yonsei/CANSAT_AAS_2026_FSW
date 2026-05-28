#!/usr/bin/env python3
"""
bench_motor_test.py — 모터 알고리즘 직접 검증 스크립트
====================================================
main.py / 멀티프로세싱 없이 guidance→control 파이프라인을 직접 실행하고
실제 서보를 구동하여 알고리즘을 검증합니다.

사전 준비 (Raspberry Pi):
  sudo pigpiod              # pigpio 데몬 기동
  cd /workspace/CANSAT_AAS_2026_FSW
  python3 bench_motor_test.py

각 케이스: Enter → 다음 케이스 / q+Enter → 종료
Ctrl+C → 서보 중립 복귀 후 종료
"""
import sys
import os
import time
import math

# ── FSW 경로 설정 (스크립트 위치 = FSW 루트) ─────────────────────────────────
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from Sensor_Motor import guidance, control
from Sensor_Motor.guidance import L1Input, ControlMode, DRMethod
from lib import config

# ── pigpio 초기화 ─────────────────────────────────────────────────────────────
try:
    import pigpio
    pi = pigpio.pi()
    if pi.connected:
        print("[OK ] pigpio 연결 성공")
    else:
        print("[WARN] pigpio 연결 실패 — 계산만 표시 (서보 미구동)")
        pi = None
except ImportError:
    print("[WARN] pigpio 미설치 — 계산만 표시 (서보 미구동)")
    pi = None

# ── guidance origin / target 설정 ─────────────────────────────────────────────
# prevstate.json 과 동일하게 맞춰야 main.py 실행과 일치함
ORIGIN_LAT, ORIGIN_LON = 37.5000, 127.0000
TARGET_LAT, TARGET_LON = 37.5000, 127.0010   # 동쪽 88.2m

# origin 직접 주입 (GPS 잠금 없이 사용)
mi = guidance._MISSION_t
mi.origin_lat   = ORIGIN_LAT
mi.origin_lon   = ORIGIN_LON
mi.origin_ready = True
mi._raw_lat     = ORIGIN_LAT
mi._raw_lon     = ORIGIN_LON

# target 설정 + ENU 투영
guidance.set_target(TARGET_LAT, TARGET_LON)
tN, tE = guidance.latlon_to_ne(TARGET_LAT, TARGET_LON, ORIGIN_LAT, ORIGIN_LON)
mi.target_E = tE
mi.target_N = tN
mi.target_ready = True

print(f"[SETUP] Origin ({ORIGIN_LAT}, {ORIGIN_LON})  →  (0, 0) m")
print(f"[SETUP] Target ({TARGET_LAT}, {TARGET_LON})  →  (E={tE:.1f}m, N={tN:.1f}m)\n")

# ── 컨트롤러 인스턴스 ─────────────────────────────────────────────────────────
ctrler = control.MakeCtrler()

# ── pigpio 서보 초기화 ────────────────────────────────────────────────────────
if pi is not None:
    pi.set_servo_pulsewidth(control.PARAFOIL_LEFT_MOTOR_PIN,  control.LEFT_NEUTRAL)
    pi.set_servo_pulsewidth(control.PARAFOIL_RIGHT_MOTOR_PIN, control.RIGHT_NEUTRAL)
    print(f"[SERVO] 중립 설정: LEFT={control.LEFT_NEUTRAL}µs  RIGHT={control.RIGHT_NEUTRAL}µs\n")


# ── 헬퍼 ──────────────────────────────────────────────────────────────────────

def _nu_deg(pos_E, pos_N, course_deg):
    """주어진 위치/방향에서의 nu(횡거 오차각) 계산 (deg)."""
    dE = tE - pos_E
    dN = tN - pos_N
    if math.hypot(dE, dN) < config.TARGET_RADIUS_M:
        return 0.0
    bearing = math.degrees(math.atan2(dE, dN))
    nu = bearing - course_deg
    # wrap [-180, 180]
    while nu >  180: nu -= 360
    while nu < -180: nu += 360
    return nu


def _run_case(name, pos_E, pos_N, course_deg, speed_mps, gyrz_dps, mode):
    """
    L1Input 직접 구성 → ProduceL1Output → ProduceCtrlInput → ProduceCtrlOutput
    → MoveServo 순으로 실행하고 결과를 출력.
    """
    now = time.monotonic()
    course_rad = math.radians(course_deg)

    # 케이스마다 이전 케이스 서보 위치 영향을 차단 (슬루율 기준점 리셋)
    control.controller_reset(ctrler)

    l1in = L1Input(
        valid        = True,
        reason       = "BENCH",
        control_mode = mode,
        dr_method    = DRMethod.NONE,
        confidence   = 1.0,
        E            = pos_E,
        N            = pos_N,
        V            = speed_mps,
        course       = course_rad,
        target_E     = tE,
        target_N     = tN,
    )

    l1out  = guidance.ProduceL1Output(l1in)
    ctrl_in  = control.ProduceCtrlInput(l1out, now)
    ctrl_out = control.ProduceCtrlOutput(ctrler, ctrl_in, gyrz_dps, now)

    nu_deg = _nu_deg(pos_E, pos_N, course_deg)
    dist   = math.hypot(tE - pos_E, tN - pos_N)

    print(f"  위치 : E={pos_E:+7.1f}m  N={pos_N:+7.1f}m  (타겟까지 {dist:.1f}m)")
    print(f"  비행 : course={course_deg:.1f}°  speed={speed_mps:.1f}m/s  gyrz={gyrz_dps:+.1f}dps")
    print(f"  L1   : nu={nu_deg:+.1f}°  bearing={math.degrees(l1out.target_bearing) if math.isfinite(l1out.target_bearing) else 'N/A':.1f}°  reason={l1out.reason}")
    print(f"  CMD  : yaw_rate={math.degrees(l1out.yaw_rate_cmd):+.2f}dps  ctrl_mode={ctrl_out.mode}")
    print(f"  SERVO: left={ctrl_out.left_angle_deg:.1f}°({ctrl_out.left_pw}µs)  "
          f"right={ctrl_out.right_angle_deg:.1f}°({ctrl_out.right_pw}µs)  "
          f"delta={ctrl_out.delta_arm_deg:+.1f}°")

    if pi is not None:
        control.MoveServo(pi, ctrl_out)
        print(f"  [HW ] 서보 구동 완료")
    else:
        print(f"  [SIM] (pigpio 없음 — 실제 구동 없음)")


def _run_detumble(name, gyrz_dps):
    """
    DETUMBLING 브레이크 경로:
      ProduceCtrlOutput(DETUMBLING, gyrz) → delta = -sign(gyrz) × 80°
    """
    now = time.monotonic()

    control.controller_reset(ctrler)
    ctrl_in_dtb = control.CtrlInput(
        angular_velocity_cmd_deg_s = 0.0,
        valid        = True,
        pid_enabled  = False,
        control_mode = config.CONTROL_MODE_DETUMBLING,
    )
    out_brake = control.ProduceCtrlOutput(ctrler, ctrl_in_dtb, gyrz_dps, now)
    print(f"  gyrz    : {gyrz_dps:+.1f} dps  →  delta = {out_brake.delta_arm_deg:+.1f}°  (브레이크)")
    print(f"  SERVO   : left={out_brake.left_angle_deg:.1f}°({out_brake.left_pw}µs)  "
          f"right={out_brake.right_angle_deg:.1f}°({out_brake.right_pw}µs)")
    if pi is not None:
        control.MoveServo(pi, out_brake)
        print(f"  [HW ] 서보 구동 완료")
    else:
        print(f"  [SIM] (pigpio 없음)")


# ── 테스트 케이스 정의 ────────────────────────────────────────────────────────
#
# 좌표평면 (origin=0,0, target=(+88.2, 0)):
#
#       N
#       ↑
# +111  │  ★ CASE2 (E=0,N=111)
#       │   ↓ course=90°→
#       │    \
#       │     \ nu=+52°
#       │      ↘
#   0   ├────────────◎─────────────── E
#       │  CASE3→ (0,0)  TARGET(88,0)
#       │  (E=200,N=0,course=0°↑)
#       │  nu=-90°
#

CASES = [
    # ── (label, run_func, args) ───────────────────────────────────────────────

    ("CASE 1  │ 타겟 정면 직진  (nu≈0°, 데드밴드 내 → 중립)",
     "gps", (tE - 20.0, 0.0, 90.0, 5.0, 0.0, ControlMode.GPS_TRACKING_CLOSED)),

    ("CASE 2  │ 북쪽 111m, 동쪽 비행 → 오른쪽 선회  (nu=+52° → delta≈+101°)",
     "gps", (0.0, 111.3, 90.0, 5.0, 0.0, ControlMode.GPS_TRACKING_CLOSED)),

    ("CASE 3  │ 동쪽 200m, 북쪽 비행 → 왼쪽 선회  (nu=-90° → delta≈-132°)",
     "gps", (200.0, 0.0, 0.0, 5.0, 0.0, ControlMode.GPS_TRACKING_CLOSED)),

    ("CASE 4  │ 북서쪽 50m, 동쪽 비행 → 약한 오른쪽  (nu≈+30° → delta≈+64°)",
     "gps", (0.0, 50.0, 90.0, 5.0, 0.0, ControlMode.GPS_TRACKING_CLOSED)),

    ("CASE 5  │ 타겟 도달 (dist < 5m → nu=0, 중립)",
     "gps", (tE - 2.0, 0.0, 90.0, 5.0, 0.0, ControlMode.GPS_TRACKING_CLOSED)),

    ("CASE 6  │ DETUMBLING — 시계방향 +250dps",
     "dtb", (250.0,)),

    ("CASE 7  │ DETUMBLING — 반시계방향 -250dps",
     "dtb", (-250.0,)),
]


# ── 메인 루프 ─────────────────────────────────────────────────────────────────

def main():
    print("=" * 65)
    print(" bench_motor_test.py  —  모터 알고리즘 직접 검증")
    print("=" * 65)
    print(f" 중립 참조: left={control.NEUTRAL_ARM_DEG}°({control.LEFT_NEUTRAL}µs)  "
          f"right={control.NEUTRAL_ARM_DEG}°({control.RIGHT_NEUTRAL}µs)")
    print(f" DELTA_ARM_MAX = {control.DELTA_ARM_MAX_DEG:.0f}°   "
          f"NU_DEADBAND = {config.NU_DEADBAND_DEG}°")
    print("=" * 65)
    print(" Enter: 다음 케이스 │ q: 종료\n")

    try:
        for i, (label, kind, args) in enumerate(CASES):
            print(f"\n{'─'*65}")
            print(f" {label}")
            print(f"{'─'*65}")

            if kind == "gps":
                _run_case(label, *args)
            elif kind == "dtb":
                _run_detumble(label, *args)

            user = input("\n  [Enter=다음  q=종료] > ").strip().lower()
            if user == "q":
                break

    except KeyboardInterrupt:
        pass
    finally:
        print("\n[EXIT] 서보 중립 복귀 후 종료")
        if pi is not None:
            pi.set_servo_pulsewidth(control.PARAFOIL_LEFT_MOTOR_PIN,  control.LEFT_NEUTRAL)
            pi.set_servo_pulsewidth(control.PARAFOIL_RIGHT_MOTOR_PIN, control.RIGHT_NEUTRAL)
            time.sleep(0.3)
            pi.set_servo_pulsewidth(control.PARAFOIL_LEFT_MOTOR_PIN,  0)
            pi.set_servo_pulsewidth(control.PARAFOIL_RIGHT_MOTOR_PIN, 0)
            pi.stop()


if __name__ == "__main__":
    main()
