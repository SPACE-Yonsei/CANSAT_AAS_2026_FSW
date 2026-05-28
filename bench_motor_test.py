#!/usr/bin/env python3
"""
bench_motor_test.py - motor guidance/control one-shot bench
===========================================================

main.py 없이 guidance/control 파이프라인을 직접 실행하고, pigpio가 연결된
환경에서는 실제 서보까지 구동한다.

현재 제어 기준:
  - 타겟 거리 기반 "도달 시 중립" 특수 처리는 없다.
  - bearing은 현재 위치에서 타겟을 보는 절대 방위각이다.
  - nu는 bearing - course로 계산되는 방향 오차다.
  - detumbling은 guidance/L1/PID를 거치지 않고 ProduceDetumbleOutput()을 직접 쓴다.

사용:
  sudo pigpiod
  python3 bench_motor_test.py

Enter: 다음 케이스 / q+Enter: 종료
Ctrl+C: 서보 중립 복귀 후 종료
"""

import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from Sensor_Motor import control, guidance
from Sensor_Motor.guidance import ControlMode, DRMethod, L1Input
from lib import config


ORIGIN_LAT, ORIGIN_LON = 37.5000, 127.0000
TARGET_LAT, TARGET_LON = 37.5000, 127.0010


try:
    import pigpio

    pi = pigpio.pi()
    if pi.connected:
        print("[OK ] pigpio connected")
    else:
        print("[WARN] pigpio connection failed. Calculation-only mode.")
        pi = None
except ImportError:
    print("[WARN] pigpio is not installed. Calculation-only mode.")
    pi = None


def _wrap_deg(angle_deg: float) -> float:
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg


def _deg_text(value_rad: float) -> str:
    if not math.isfinite(value_rad):
        return "N/A"
    return f"{math.degrees(value_rad):+.1f}deg"


def _configure_mission() -> tuple[float, float]:
    guidance.reset()
    mi = guidance._MISSION_t
    mi.origin_lat = ORIGIN_LAT
    mi.origin_lon = ORIGIN_LON
    mi.origin_ready = True
    mi._raw_lat = ORIGIN_LAT
    mi._raw_lon = ORIGIN_LON

    guidance.set_target(TARGET_LAT, TARGET_LON)
    target_N, target_E = guidance.latlon_to_ne(
        TARGET_LAT,
        TARGET_LON,
        ORIGIN_LAT,
        ORIGIN_LON,
    )
    mi.target_E = target_E
    mi.target_N = target_N
    mi.target_ready = True
    return target_E, target_N


tE, tN = _configure_mission()
ctrler = control.MakeCtrler()


if pi is not None:
    pi.set_servo_pulsewidth(control.PARAFOIL_LEFT_MOTOR_PIN, control.LEFT_NEUTRAL)
    pi.set_servo_pulsewidth(control.PARAFOIL_RIGHT_MOTOR_PIN, control.RIGHT_NEUTRAL)
    print(
        "[SERVO] neutral: "
        f"LEFT={control.LEFT_NEUTRAL}us RIGHT={control.RIGHT_NEUTRAL}us"
    )


def _bearing_nu_deg(pos_E: float, pos_N: float, course_deg: float) -> tuple[float, float]:
    dE = tE - pos_E
    dN = tN - pos_N
    bearing = math.degrees(math.atan2(dE, dN))
    nu = _wrap_deg(bearing - course_deg)
    return bearing, nu


def _applied_delta(out: control.CtrlOutput) -> float:
    return out.right_angle_deg - out.left_angle_deg


def _move_or_print(out: control.CtrlOutput) -> None:
    if pi is not None:
        control.MoveServo(pi, out)
        print("  [HW ] servo moved")
    else:
        print("  [SIM] pigpio unavailable, servo not moved")


def _run_tracking_case(
    pos_E: float,
    pos_N: float,
    course_deg: float,
    speed_mps: float,
    gyrz_dps: float,
    mode: ControlMode,
) -> None:
    now = time.monotonic()
    control.controller_reset(ctrler)

    l1_in = L1Input(
        valid=True,
        reason="BENCH",
        control_mode=mode,
        dr_method=DRMethod.NONE,
        confidence=1.0,
        E=pos_E,
        N=pos_N,
        V=speed_mps,
        course=math.radians(course_deg),
        target_E=tE,
        target_N=tN,
    )

    l1_out = guidance.ProduceL1Output(l1_in)
    ctrl_in = control.ProduceCtrlInput(l1_out, now)
    ctrl_out = control.ProduceCtrlOutput(ctrler, ctrl_in, gyrz_dps, now)

    bearing_deg, nu_deg = _bearing_nu_deg(pos_E, pos_N, course_deg)
    dist = math.hypot(tE - pos_E, tN - pos_N)
    applied_delta = _applied_delta(ctrl_out)
    slew_limited = abs(ctrl_out.delta_arm_deg - applied_delta) > 0.5

    print(f"  pos    : E={pos_E:+7.1f}m  N={pos_N:+7.1f}m  dist={dist:.1f}m")
    print(
        f"  flight : course={course_deg:+.1f}deg  "
        f"speed={speed_mps:.1f}m/s  gyrz={gyrz_dps:+.1f}dps"
    )
    print(
        f"  L1     : bearing={bearing_deg:+.1f}deg  "
        f"nu={nu_deg:+.1f}deg  out_bearing={_deg_text(l1_out.target_bearing)}  "
        f"reason={l1_out.reason}"
    )
    print(
        f"  CMD    : yaw_rate={math.degrees(l1_out.yaw_rate_cmd):+.2f}dps  "
        f"ctrl_mode={ctrl_out.mode}  fallback={ctrl_out.fallback_mode}"
    )
    print(
        f"  SERVO  : left={ctrl_out.left_angle_deg:.1f}deg({ctrl_out.left_pw}us)  "
        f"right={ctrl_out.right_angle_deg:.1f}deg({ctrl_out.right_pw}us)"
    )
    print(
        f"  DELTA  : target={ctrl_out.delta_arm_deg:+.1f}deg  "
        f"applied={applied_delta:+.1f}deg"
        + ("  (slew limited)" if slew_limited else "")
    )
    _move_or_print(ctrl_out)


def _run_detumble_case(gyrz_dps: float) -> None:
    now = time.monotonic()
    control.controller_reset(ctrler)

    out = control.ProduceDetumbleOutput(now, gyrz_dps)
    print(
        f"  gyrz   : {gyrz_dps:+.1f}dps  "
        f"delta={out.delta_arm_deg:+.1f}deg  mode={out.mode}"
    )
    print(
        f"  SERVO  : left={out.left_angle_deg:.1f}deg({out.left_pw}us)  "
        f"right={out.right_angle_deg:.1f}deg({out.right_pw}us)"
    )
    _move_or_print(out)


CASES = [
    (
        "CASE 1 | target ahead, straight",
        "gps",
        (tE - 20.0, tN, 90.0, 5.0, 0.0, ControlMode.GPS_TRACKING_CLOSED),
    ),
    (
        "CASE 2 | north of path, eastbound -> right turn",
        "gps",
        (0.0, tN + 111.3, 90.0, 5.0, 0.0, ControlMode.GPS_TRACKING_CLOSED),
    ),
    (
        "CASE 3 | east of target, northbound -> left turn",
        "gps",
        (tE + 111.8, tN, 0.0, 5.0, 0.0, ControlMode.GPS_TRACKING_CLOSED),
    ),
    (
        "CASE 4 | weak right command",
        "gps",
        (0.0, tN + 50.0, 90.0, 5.0, 0.0, ControlMode.GPS_TRACKING_CLOSED),
    ),
    (
        "CASE 5 | near target, northbound -> still commands turn",
        "gps",
        (tE - 2.0, tN, 0.0, 5.0, 0.0, ControlMode.GPS_TRACKING_CLOSED),
    ),
    (
        "CASE 6 | DETUMBLING, right rotation +250dps",
        "dtb",
        (250.0,),
    ),
    (
        "CASE 7 | DETUMBLING, left rotation -250dps",
        "dtb",
        (-250.0,),
    ),
]


def main() -> None:
    print("=" * 70)
    print(" bench_motor_test.py - current motor control bench")
    print("=" * 70)
    print(f" Origin: ({ORIGIN_LAT:.7f}, {ORIGIN_LON:.7f}) -> E=0.0m, N=0.0m")
    print(
        f" Target: ({TARGET_LAT:.7f}, {TARGET_LON:.7f}) "
        f"-> E={tE:.1f}m, N={tN:.1f}m"
    )
    print(
        f" Neutral: left={control.NEUTRAL_ARM_DEG:.1f}deg({control.LEFT_NEUTRAL}us)  "
        f"right={control.NEUTRAL_ARM_DEG:.1f}deg({control.RIGHT_NEUTRAL}us)"
    )
    print(
        f" Limits : arm=[{control.ARM_MIN_DEG:.0f}, {control.ARM_MAX_DEG:.0f}]deg  "
        f"delta_max={control.DELTA_ARM_MAX_DEG:.0f}deg  "
        f"nu_deadband={config.NU_DEADBAND_DEG:.1f}deg"
    )
    print("=" * 70)
    print("Enter: next case / q: quit\n")

    try:
        for label, kind, args in CASES:
            print("\n" + "-" * 70)
            print(f" {label}")
            print("-" * 70)

            if kind == "gps":
                _run_tracking_case(*args)
            elif kind == "dtb":
                _run_detumble_case(*args)

            user = input("\n  [Enter=next  q=quit] > ").strip().lower()
            if user == "q":
                break
    except KeyboardInterrupt:
        pass
    finally:
        print("\n[EXIT] neutral/off and stop")
        if pi is not None:
            pi.set_servo_pulsewidth(control.PARAFOIL_LEFT_MOTOR_PIN, control.LEFT_NEUTRAL)
            pi.set_servo_pulsewidth(control.PARAFOIL_RIGHT_MOTOR_PIN, control.RIGHT_NEUTRAL)
            time.sleep(0.3)
            pi.set_servo_pulsewidth(control.PARAFOIL_LEFT_MOTOR_PIN, 0)
            pi.set_servo_pulsewidth(control.PARAFOIL_RIGHT_MOTOR_PIN, 0)
            pi.stop()


if __name__ == "__main__":
    main()
