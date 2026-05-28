#!/usr/bin/env python3
"""
bench_motor_test.py - motor guidance/control one-shot bench
===========================================================

_ctrl_cycle 파이프라인을 직접 재현한다.

ControlMode별 파이프라인 (motorapp._ctrl_cycle 기준):

  DETUMBLING:
    _should_detumble → ProduceDetumbleOutput(now, gz_meas) → MoveServo

  GPS_TRACKING_CLOSED / GPS_TRACKING_OPEN /
  DR_TRACKING_CLOSED  / DR_TRACKING_OPEN:
    ProduceL1Input → ProduceL1Output
      → ProduceCtrlInput → ProduceCtrlOutput(ctrler, ctrl_in, gz_meas) → MoveServo

  FAIL:
    WriteOff

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


# ── GPS_TRACKING_CLOSED / GPS_TRACKING_OPEN /
#    DR_TRACKING_CLOSED  / DR_TRACKING_OPEN ───────────────────────────────────
#
# motorapp._ctrl_cycle 파이프라인:
#   ProduceL1Input → ProduceL1Output
#   → ProduceCtrlInput → ProduceCtrlOutput(ctrler, ctrl_in, gz_meas) → MoveServo
#
# bench에서는 L1Input을 직접 주입한다 (실제 센서 없음).

def _run_tracking_case(
    pos_E: float,
    pos_N: float,
    course_deg: float,
    speed_mps: float,
    gyrz_dps: float,
    control_mode: ControlMode,
) -> None:
    now = time.monotonic()
    control.controller_reset(ctrler)

    l1_in = L1Input(
        valid=True,
        reason="BENCH",
        control_mode=control_mode,
        dr_method=DRMethod.NONE,
        confidence=1.0,
        E=pos_E,
        N=pos_N,
        V=speed_mps,
        course=math.radians(course_deg),
        target_E=tE,
        target_N=tN,
    )

    # _ctrl_cycle 파이프라인 재현
    l1_out   = guidance.ProduceL1Output(l1_in)
    ctrl_in  = control.ProduceCtrlInput(l1_out, now)
    ctrl_out = control.ProduceCtrlOutput(ctrler, ctrl_in, gyrz_dps, now)

    bearing_deg, nu_deg = _bearing_nu_deg(pos_E, pos_N, course_deg)
    dist = math.hypot(tE - pos_E, tN - pos_N)
    applied_delta = _applied_delta(ctrl_out)
    slew_limited = abs(ctrl_out.delta_arm_deg - applied_delta) > 0.5

    pid_label = "ON" if ctrl_in.pid_enabled else "OFF"

    print(f"  pos          : E={pos_E:+7.1f}m  N={pos_N:+7.1f}m  dist={dist:.1f}m")
    print(
        f"  flight       : course={course_deg:+.1f}deg  "
        f"speed={speed_mps:.1f}m/s  gyrz={gyrz_dps:+.1f}dps"
    )
    print(
        f"  L1           : bearing={bearing_deg:+.1f}deg  nu={nu_deg:+.1f}deg  "
        f"yaw_rate_cmd={math.degrees(l1_out.yaw_rate_cmd):+.2f}dps"
    )
    print(
        f"  control_mode : {ctrl_out.control_mode}  PID={pid_label}"
    )
    print(
        f"  delta        : FF={ctrl_out.delta_ff_deg:+.1f}deg  "
        f"PID={ctrl_out.delta_pid_deg:+.1f}deg  "
        f"target={ctrl_out.delta_arm_deg:+.1f}deg  "
        f"applied={applied_delta:+.1f}deg"
        + ("  (slew limited)" if slew_limited else "")
    )
    print(
        f"  SERVO        : left={ctrl_out.left_angle_deg:.1f}deg({ctrl_out.left_pw}us)  "
        f"right={ctrl_out.right_angle_deg:.1f}deg({ctrl_out.right_pw}us)"
    )
    _move_or_print(ctrl_out)


# ── DETUMBLING ────────────────────────────────────────────────────────────────
#
# motorapp._ctrl_cycle 파이프라인:
#   _should_detumble → ProduceDetumbleOutput(now, gz_meas) → MoveServo
#
# ProduceCtrlOutput을 거치지 않는다.

def _run_detumble_case(gyrz_dps: float) -> None:
    now = time.monotonic()
    control.controller_reset(ctrler)

    # _ctrl_cycle에서 ProduceDetumbleOutput을 직접 호출하는 것과 동일
    out = control.ProduceDetumbleOutput(now, gyrz_dps)

    applied_delta = _applied_delta(out)
    print(
        f"  control_mode : {out.control_mode}"
    )
    print(
        f"  gyrz         : {gyrz_dps:+.1f}dps  "
        f"delta={out.delta_arm_deg:+.1f}deg  applied={applied_delta:+.1f}deg"
    )
    print(
        f"  SERVO        : left={out.left_angle_deg:.1f}deg({out.left_pw}us)  "
        f"right={out.right_angle_deg:.1f}deg({out.right_pw}us)"
    )
    _move_or_print(out)


# ── 감도 테이블 ───────────────────────────────────────────────────────────────
#
# nu → yaw_rate_cmd → delta_ff → arm 각도
#
# L1 공식: yaw_rate_cmd = 2·V / L_GAIN_M · sin(nu) · confidence
# expo 곡선: delta_ff = DELTA_MIN_EFFECTIVE + (DELTA_FF_MAX - DELTA_MIN_EFFECTIVE) · (|cmd|/cmd_max)^EXPO
# 믹서: left = NEUTRAL - delta/2 / right = NEUTRAL + delta/2

_NU_SWEEP_DEG = [-180, -150, -120, -90, -60, -45, -30, -15, 0,
                  15, 30, 45, 60, 90, 120, 150, 180]


def _nu_to_yaw_rate_dps(nu_deg: float, speed_mps: float, confidence: float = 1.0) -> float:
    """L1 공식. yaw_rate_cmd (deg/s)."""
    nu_rad = math.radians(nu_deg)
    sin_nu = max(-1.0, min(1.0, math.sin(nu_rad)))
    rate_rad_s = 2.0 * speed_mps / config.L_GAIN_M * sin_nu * confidence
    return math.degrees(rate_rad_s)


def _print_sensitivity_table(speed_mps: float, confidence: float = 1.0) -> None:
    """nu → yaw_rate_cmd → delta_ff → arm 각도 감도 테이블 출력."""
    cfg = control.ControlConfig()

    header = (
        f"{'nu':>7}  {'yaw_rate':>10}  {'delta_ff':>10}  "
        f"{'left_arm':>10}  {'right_arm':>10}  {'left_pw':>8}  {'right_pw':>8}"
    )
    unit = (
        f"{'(deg)':>7}  {'(dps)':>10}  {'(deg)':>10}  "
        f"{'(deg)':>10}  {'(deg)':>10}  {'(us)':>8}  {'(us)':>8}"
    )
    sep = "-" * len(header)

    print(f"\n  V={speed_mps:.1f}m/s  L_GAIN={config.L_GAIN_M:.1f}m  confidence={confidence:.2f}")
    print(f"  cmd_max={cfg.ANGULAR_VELOCITY_CMD_MAX_DEG_S:.1f}dps  "
          f"deadband={cfg.ANGULAR_VELOCITY_DEADBAND_DEG_S:.1f}dps  "
          f"expo={cfg.EXPO:.2f}  "
          f"delta_ff_max={cfg.DELTA_FF_MAX_DEG:.0f}deg  "
          f"delta_min_eff={cfg.DELTA_MIN_EFFECTIVE_DEG:.1f}deg")
    print(f"  {sep}")
    print(f"  {header}")
    print(f"  {unit}")
    print(f"  {sep}")

    for nu_deg in _NU_SWEEP_DEG:
        yaw_rate_dps = _nu_to_yaw_rate_dps(nu_deg, speed_mps, confidence)
        delta_ff = control.angular_velocity_to_delta_ff(yaw_rate_dps, cfg)
        left_pw, right_pw, left_angle, right_angle, _ = control.ConnectRoMo(delta_ff)
        clamped = abs(yaw_rate_dps) > cfg.ANGULAR_VELOCITY_CMD_MAX_DEG_S
        clamp_mark = "*" if clamped else " "
        print(
            f"  {nu_deg:>+6.0f}°  {yaw_rate_dps:>+9.2f}{clamp_mark}  "
            f"{delta_ff:>+10.1f}  "
            f"{left_angle:>10.1f}  {right_angle:>10.1f}  "
            f"{left_pw:>8}  {right_pw:>8}"
        )

    print(f"  {sep}")
    print("  * : yaw_rate_cmd가 cmd_max에 클램핑됨")


def _print_sensitivity_section() -> None:
    print("\n" + "=" * 70)
    print(" 감도 테이블: nu → yaw_rate_cmd → delta_ff → arm 각도")
    print("=" * 70)
    for v in (5.0, 8.0):
        _print_sensitivity_table(speed_mps=v)


CASES = [
    (
        "CASE 1 | GPS_TRACKING_CLOSED | target ahead, straight",
        "tracking",
        (tE - 20.0, tN, 90.0, 5.0, 0.0, ControlMode.GPS_TRACKING_CLOSED),
    ),
    (
        "CASE 2 | GPS_TRACKING_CLOSED | north of path, eastbound -> right turn",
        "tracking",
        (0.0, tN + 111.3, 90.0, 5.0, 0.0, ControlMode.GPS_TRACKING_CLOSED),
    ),
    (
        "CASE 3 | GPS_TRACKING_CLOSED | east of target, northbound -> left turn",
        "tracking",
        (tE + 111.8, tN, 0.0, 5.0, 0.0, ControlMode.GPS_TRACKING_CLOSED),
    ),
    (
        "CASE 4 | GPS_TRACKING_CLOSED | weak right command",
        "tracking",
        (0.0, tN + 50.0, 90.0, 5.0, 0.0, ControlMode.GPS_TRACKING_CLOSED),
    ),
    (
        "CASE 5 | GPS_TRACKING_OPEN | east of target, northbound -> left turn (FF only, no PID)",
        "tracking",
        (tE + 111.8, tN, 0.0, 5.0, 0.0, ControlMode.GPS_TRACKING_OPEN),
    ),
    (
        "CASE 6 | DETUMBLING | right rotation +250dps",
        "detumble",
        (250.0,),
    ),
    (
        "CASE 7 | DETUMBLING | left rotation -250dps",
        "detumble",
        (-250.0,),
    ),
]


def main() -> None:
    print("=" * 70)
    print(" bench_motor_test.py - _ctrl_cycle 파이프라인 재현")
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
    _print_sensitivity_section()
    print("\n" + "=" * 70)
    print("Enter: next case / q: quit\n")

    try:
        for label, kind, args in CASES:
            print("\n" + "-" * 70)
            print(f" {label}")
            print("-" * 70)

            if kind == "tracking":
                _run_tracking_case(*args)
            elif kind == "detumble":
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
