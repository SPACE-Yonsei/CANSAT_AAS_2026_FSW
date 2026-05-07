# -*- coding: utf-8 -*-
"""ex1.py -- LCSG sensor value input -> left/right motor angle calculator

Calls the exact same classes, methods, and variables in the same order
as described in the step-by-step example:

  GuidanceInputResolver  ->  L1Guidance  ->  ParafoilBrakeController

Default values match the worked example:
  Start  : N=0 m, E=0 m  (origin)
  Current: N=10 m, E=4 m  (course=20 deg, speed=5 m/s, gz=3 deg/s)
  Target : N=100 m, E=20 m

Usage:
  .venv/Scripts/python.exe ex1.py               # use defaults
  .venv/Scripts/python.exe ex1.py --interactive  # enter values manually
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from Sensor_Motor.motor_guidance import (
    GuidanceInputResolver,
    L1Guidance,
    L1Config,
    GuidanceMode,
    ne_to_ll,
    _ll_to_ne,
)
from Sensor_Motor.motor_control import (
    ParafoilBrakeController,
    ControlConfig,
    GuidanceCommand,
    BrakeCommand,
    NEUTRAL_ARM_DEG,
)

_LINE  = "-" * 68
_LINE2 = "=" * 68


def _banner(title):
    print("\n" + _LINE2)
    print("  " + title)
    print(_LINE2)


def _section(title):
    print("\n" + _LINE)
    print("  " + title)
    print(_LINE)


# ---- input collection --------------------------------------------------------

def collect_inputs(interactive):
    defaults = dict(
        pos_N_m    = 10.0,    # L: current North [m] from origin
        pos_E_m    =  4.0,    # L: current East  [m] from origin
        course_deg = 20.0,    # C: GPS ground-track [deg]
        speed_mps  =  5.0,    # S: GPS ground-speed [m/s]
        gz_degs    =  3.0,    # G: IMU gz [deg/s]
        tgt_N_m    = 100.0,   # target North [m]
        tgt_E_m    =  20.0,   # target East  [m]
        has_L      = True,
        has_C      = True,
        has_S      = True,
        has_G      = True,
    )

    if not interactive:
        return defaults

    print("\n[LCSG input]  press Enter to keep default")

    def _f(prompt, default):
        raw = input("  %s [%s]: " % (prompt, default)).strip()
        return float(raw) if raw else default

    def _b(prompt, default):
        raw = input("  %s [%s]: " % (prompt, "Y" if default else "N")).strip().upper()
        if raw in ("Y", "1", "TRUE"):  return True
        if raw in ("N", "0", "FALSE"): return False
        return default

    print("\n  -- L: position --")
    defaults["has_L"]    = _b("posHealth (Y/N)", True)
    defaults["pos_N_m"]  = _f("pos_N [m]", 10.0)
    defaults["pos_E_m"]  = _f("pos_E [m]",  4.0)

    print("\n  -- C: course --")
    defaults["has_C"]      = _b("course valid (Y/N)", True)
    defaults["course_deg"] = _f("course_deg", 20.0)

    print("\n  -- S: speed --")
    defaults["has_S"]     = _b("speed valid (Y/N)", True)
    defaults["speed_mps"] = _f("speed [m/s]", 5.0)

    print("\n  -- G: gyro --")
    defaults["has_G"]   = _b("gz valid (Y/N)", True)
    defaults["gz_degs"] = _f("gz [deg/s]", 3.0)

    print("\n  -- target --")
    defaults["tgt_N_m"] = _f("target North [m]", 100.0)
    defaults["tgt_E_m"] = _f("target East  [m]",  20.0)

    return defaults


# ---- main simulator ----------------------------------------------------------

def simulate(p):
    _ORIGIN_LAT = 37.0
    _ORIGIN_LON = 127.0
    NOW = 1.0   # single-step simulation; avoid timestamp=0.0 (falsy -> age=inf)

    lcsg_str = (
        ("L" if p["has_L"] else "-") +
        ("C" if p["has_C"] else "-") +
        ("S" if p["has_S"] else "-") +
        ("G" if p["has_G"] else "-")
    )

    _banner("Parafoil guidance/control  single-step sim   (LCSG = %s)" % lcsg_str)

    print("\n  Input sensor values:")
    print("    L  pos_N=%.2f m,  pos_E=%.2f m   (posHealth=%s)"
          % (p["pos_N_m"], p["pos_E_m"], p["has_L"]))
    print("    C  course=%.2f deg   (valid=%s)" % (p["course_deg"], p["has_C"]))
    print("    S  speed=%.2f m/s   (valid=%s)"  % (p["speed_mps"],  p["has_S"]))
    print("    G  gz=%.3f deg/s   (valid=%s)"   % (p["gz_degs"],    p["has_G"]))
    print("\n  Target: N=%.1f m,  E=%.1f m" % (p["tgt_N_m"], p["tgt_E_m"]))

    # ==========================================================================
    # STEP 1  GuidanceInputResolver -- register sensor values
    # ==========================================================================
    _section("STEP 1  GuidanceInputResolver  -- sensor registration")

    # __init__: all _FieldRecord fields = None
    resolver = GuidanceInputResolver()

    # set_origin: _origin_lat, _origin_lon
    resolver.set_origin(_ORIGIN_LAT, _ORIGIN_LON)
    print("  resolver.set_origin(%.4f, %.4f)" % (_ORIGIN_LAT, _ORIGIN_LON))
    print("    _origin_lat = %s" % resolver.origin_lat)
    print("    _origin_lon = %s" % resolver.origin_lon)

    # ne_to_ll: reverse-map N/E -> lat/lon for update_gnss
    lat, lon = ne_to_ll(p["pos_N_m"], p["pos_E_m"], _ORIGIN_LAT, _ORIGIN_LON)
    course_rad = math.radians(p["course_deg"])
    motion_ok  = p["has_C"] and p["has_S"]

    # update_gnss: posHealth -> _lat, _lon
    #              motionHealth -> _course [rad], _groundSpeed
    resolver.update_gnss(
        lat          = lat               if p["has_L"] else None,
        lon          = lon               if p["has_L"] else None,
        course_rad   = course_rad        if p["has_C"] else None,
        groundSpeed  = p["speed_mps"]    if p["has_S"] else None,
        posHealth    = p["has_L"],
        motionHealth = motion_ok,
        ts           = NOW,
    )
    print("\n  resolver.update_gnss()")
    print("    lat=%.6f, lon=%.6f  (ne_to_ll inverse)" % (lat, lon))
    print("    course_rad=%.4f rad  (%.2f deg)" % (course_rad, p["course_deg"]))
    print("    groundSpeed=%.2f m/s" % p["speed_mps"])
    print("    posHealth=%s, motionHealth=%s" % (p["has_L"], motion_ok))
    print("    -> _lat   = %s" % (resolver._lat.value  if resolver._lat  else None))
    print("    -> _lon   = %s" % (resolver._lon.value  if resolver._lon  else None))
    if resolver._course:
        print("    -> _course= %.4f rad" % resolver._course.value)
    else:
        print("    -> _course= None")
    if resolver._groundSpeed:
        print("    -> _groundSpeed= %.2f m/s" % resolver._groundSpeed.value)
    else:
        print("    -> _groundSpeed= None")

    # update_imu: gz [deg/s] -> _gz
    if p["has_G"]:
        resolver.update_imu(gz=p["gz_degs"], ts=NOW)
        print("\n  resolver.update_imu(gz=%.3f deg/s)" % p["gz_degs"])
        print("    -> _gz= %.3f deg/s" % (resolver._gz.value if resolver._gz else 0))
    else:
        print("\n  resolver.update_imu() -- skipped (has_G=False)")

    # ==========================================================================
    # STEP 2  GuidanceInputResolver.resolve() -- build GuidanceInput
    # ==========================================================================
    _section("STEP 2  GuidanceInputResolver.resolve()  -- build GuidanceInput")

    # resolve() calls internally:
    #   _fill_position, _fill_motion, _fill_yaw_rate, _fill_altitude,
    #   _fill_attitude, _fill_tilt_rate, _fill_lcsg_case, _classify_guidance_mode
    g_input = resolver.resolve(NOW)

    print("  _fill_position():")
    print("    pos_N       = %s" % g_input.pos_N)
    print("    pos_E       = %s" % g_input.pos_E)
    print("    pos_status  = %s" % g_input.pos_status)

    print("\n  _fill_motion():")
    if g_input.course is not None:
        print("    course      = %.4f rad  (%.4f deg)"
              % (g_input.course, math.degrees(g_input.course)))
    else:
        print("    course      = None")
    print("    groundSpeed = %s" % g_input.groundSpeed)
    if g_input.vel_N is not None:
        print("    vel_N       = %.4f m/s" % g_input.vel_N)
        print("    vel_E       = %.4f m/s" % g_input.vel_E)
    print("    motion_health = %s" % g_input.motion_health)

    print("\n  _fill_yaw_rate():")
    if g_input.gyrz is not None:
        print("    gz_degs -> gyrz = %.6f rad/s" % g_input.gyrz)
    else:
        print("    gyrz = None")
    print("    gyrz_health = %s" % g_input.gyrz_health)

    print("\n  _fill_lcsg_case():")
    print('    lcsg_case    = "%s"' % g_input.lcsg_case)
    print('    input_policy = "%s"' % g_input.input_policy)

    print("\n  _classify_guidance_mode():")
    print("    guidance_mode = %s" % g_input.guidance_mode)
    if g_input.reason:
        print('    reason        = "%s"' % g_input.reason)

    # ==========================================================================
    # STEP 3  L1Guidance.update() -- GuidanceOutput
    # ==========================================================================
    _section("STEP 3  L1Guidance.update()  -- L1 path following")

    cfg     = L1Config()
    guidance = L1Guidance(cfg)
    guidance.set_start(0.0, 0.0)          # start_N=0, start_E=0 (origin)
    guidance.set_target(p["tgt_N_m"], p["tgt_E_m"])

    g_out = guidance.update(g_input, NOW)

    if not g_out.active:
        print('\n  INACTIVE -- reason: "%s"' % g_out.reason)
        print("  -> controller receives invalid command -> neutral output")
    else:
        # reproduce internal variables for display
        A_N, A_E = guidance._start_N,  guidance._start_E
        B_N, B_E = guidance._target_N, guidance._target_E
        AB_N = B_N - A_N;  AB_E = B_E - A_E
        AB_len = math.hypot(AB_N, AB_E)
        e_N = AB_N / AB_len;  e_E = AB_E / AB_len
        pos_N = g_input.pos_N or 0.0
        pos_E = g_input.pos_E or 0.0

        gs     = g_input.groundSpeed or 0.0
        course = g_input.course      or 0.0
        vel_N_ = gs * math.cos(course)
        vel_E_ = gs * math.sin(course)
        ltrackVel = vel_N_ * e_N + vel_E_ * e_E
        xtrackVel = vel_N_ * e_E - vel_E_ * e_N

        gs_eff  = max(gs, cfg.V_MIN)
        L1_dist = max(cfg.damping * cfg.period / math.pi * gs_eff, cfg.L1_MIN)

        print("\n  Path A->B:")
        print("    A=(N=%.1f, E=%.1f)  B=(N=%.1f, E=%.1f)"
              % (A_N, A_E, B_N, B_E))
        print("    AB_len=%.2f m" % AB_len)
        print("    e_N=%.4f,  e_E=%.4f" % (e_N, e_E))

        print("\n  Position projection:")
        print("    crossTrack  = %.3f m  (%s)"
              % (g_out.crossTrack,
                 "left of path" if g_out.crossTrack > 0 else "right of path"))
        print("    alongTrack  = %.3f m" % g_out.alongTrack)

        print("\n  Velocity projection:")
        print("    ltrackVel   = %.3f m/s  (along path)" % ltrackVel)
        print("    xtrackVel   = %.3f m/s  (%s)"
              % (xtrackVel,
                 "drifting left" if xtrackVel > 0 else "drifting right"))

        print("\n  L1 distance:")
        print("    gs_eff  = max(%.2f, %.1f) = %.2f m/s"
              % (gs, cfg.V_MIN, gs_eff))
        print("    L1_dist = max(0.75x8/pi x %.2f, %.1f)" % (gs_eff, cfg.L1_MIN))
        print("            = max(%.3f, %.1f) = %.3f m"
              % (cfg.damping * cfg.period / math.pi * gs_eff, cfg.L1_MIN, g_out.L1_dist))

        sine_nu1_raw = g_out.crossTrack / g_out.L1_dist
        print("\n  Nu calculation:")
        print("    sine_Nu1 = clamp(%.3f/%.3f, +-0.7071) = clamp(%.4f, +-0.7071)"
              % (g_out.crossTrack, g_out.L1_dist, sine_nu1_raw))
        print("    Nu1 = %.3f deg  (%.5f rad)" % (math.degrees(g_out.Nu1), g_out.Nu1))
        print("    Nu2 = %.3f deg  (%.5f rad)" % (math.degrees(g_out.Nu2), g_out.Nu2))
        print("    Nu  = %.3f deg  (%.5f rad)" % (math.degrees(g_out.Nu),  g_out.Nu))

        K_L1 = 4.0 * cfg.damping ** 2
        print("\n  Acceleration and course-rate command:")
        print("    K_L1         = 4 x %.2f^2 = %.4f" % (cfg.damping, K_L1))
        print("    latAccDem    = clamp(%.4f x %.2f^2 / %.3f x sin(%.3f deg), +- %.1f)"
              % (K_L1, gs, g_out.L1_dist, math.degrees(g_out.Nu), cfg.LAT_ACC_MAX))
        print("                 = %.4f m/s^2" % g_out.latAccDem)
        print("    courseRateCmd= clamp(%.4f / %.2f, +- %.1f)"
              % (g_out.latAccDem, gs_eff, cfg.COURSE_RATE_MAX))
        print("                 = %.5f rad/s  (%.4f deg/s)  [%s]"
              % (g_out.courseRateCmd,
                 math.degrees(g_out.courseRateCmd),
                 "left turn" if g_out.courseRateCmd < 0 else "right turn"))

    # ==========================================================================
    # STEP 4  GuidanceCommand packaging  [motorapp.ctrl_paragldr]
    # ==========================================================================
    _section("STEP 4  GuidanceCommand packaging")

    if g_out.active:
        # rad/s -> deg/s conversion (done in ctrl_paragldr)
        yaw_rate_cmd_degs  = math.degrees(g_out.courseRateCmd)
        yaw_rate_meas_degs = (math.degrees(g_input.gyrz)
                              if g_input.gyrz is not None else None)

        guidance_cmd = GuidanceCommand(
            yaw_rate_cmd_deg_s = yaw_rate_cmd_degs,
            lat_acc_cmd_mps2   = g_out.latAccDem,
            ground_speed_mps   = g_out.groundSpeed,
            valid              = True,
            timestamp          = NOW,
        )
        print("  g_out.courseRateCmd = %.5f rad/s" % g_out.courseRateCmd)
        print("  -> math.degrees() -> yaw_rate_cmd_deg_s = %.4f deg/s"
              % yaw_rate_cmd_degs)
        if g_input.gyrz is not None:
            print("\n  g_input.gyrz = %.6f rad/s" % g_input.gyrz)
            print("  -> math.degrees() -> yaw_rate_meas_deg_s = %.4f deg/s"
                  % (yaw_rate_meas_degs or 0))
        else:
            print("\n  g_input.gyrz = None -> yaw_rate_meas_deg_s = None")
        print("\n  GuidanceCommand(")
        print("      yaw_rate_cmd_deg_s = %.4f," % guidance_cmd.yaw_rate_cmd_deg_s)
        print("      lat_acc_cmd_mps2   = %.4f," % guidance_cmd.lat_acc_cmd_mps2)
        print("      ground_speed_mps   = %.4f," % guidance_cmd.ground_speed_mps)
        print("      valid              = %s,"   % guidance_cmd.valid)
        print("      timestamp          = %s,"   % guidance_cmd.timestamp)
        print("  )")
    else:
        yaw_rate_meas_degs = None
        guidance_cmd = GuidanceCommand(valid=False, timestamp=NOW)
        print("  guidance inactive -> GuidanceCommand(valid=False)")

    # ==========================================================================
    # STEP 5  ParafoilBrakeController.update() -- BrakeCommand
    # ==========================================================================
    _section("STEP 5  ParafoilBrakeController.update()  -- PID + Mixer")

    controller = ParafoilBrakeController(ControlConfig())
    if not g_out.active:
        controller.reset()

    cmd = controller.update(guidance_cmd, yaw_rate_meas_degs, NOW)
    cfg_c = controller.cfg

    if g_out.active and cmd.valid:
        yaw_cmd  = cmd.yaw_rate_cmd_deg_s
        yaw_meas = cmd.yaw_rate_meas_deg_s
        error    = cmd.yaw_rate_error_deg_s

        print("  _tick() -> dt=0.1 s  (first tick default)")
        print("\n  _resolve_yaw_rate_cmd():")
        print("    yaw_cmd = yaw_rate_cmd_deg_s = %.4f deg/s" % yaw_cmd)
        print("\n  error:")
        print("    yaw_meas = %.4f deg/s" % yaw_meas)
        print("    error    = yaw_cmd - yaw_meas = %.4f - %.4f = %.4f deg/s"
              % (yaw_cmd, yaw_meas, error))
        print("\n  Feedforward:")
        print("    delta_ff = K_FF x yaw_cmd = %.2f x %.4f = %.4f deg"
              % (cfg_c.K_FF, yaw_cmd, cmd.delta_ff_deg))
        print("\n  PID (mode=%s):" % cmd.mode)
        print("    p = K_P x error = %.2f x %.4f = %.4f deg"
              % (cfg_c.K_P, error, cmd.pid_p))
        print("    i = %.4f deg  (K_I=%.1f)" % (cmd.pid_i, cfg_c.K_I))
        print("    d = %.4f deg  (K_D=%.1f)" % (cmd.pid_d, cfg_c.K_D))
        print("    delta_pid = %.4f deg"     % cmd.delta_pid_deg)

        raw_delta     = cmd.delta_ff_deg + cmd.delta_pid_deg
        delta_clamped = max(-cfg_c.DELTA_ARM_MAX_DEG,
                            min(cfg_c.DELTA_ARM_MAX_DEG, raw_delta))
        left_target   = cfg_c.NEUTRAL_ARM_DEG + delta_clamped / 2.0
        right_target  = cfg_c.NEUTRAL_ARM_DEG - delta_clamped / 2.0
        step          = cfg_c.MAX_ARM_RATE_DEG_S * 0.1
        # post-slew actual delta
        actual_delta  = cmd.left_angle_deg - cmd.right_angle_deg

        print("\n  Differential mixer:")
        print("    raw_delta    = %.4f + %.4f = %.4f deg"
              % (cmd.delta_ff_deg, cmd.delta_pid_deg, raw_delta))
        print("    delta_clamped= clamp(%.4f, +-%d) = %.4f deg"
              % (raw_delta, cfg_c.DELTA_ARM_MAX_DEG, delta_clamped))
        print("    left_target  = %.0f + %.4f/2 = %.4f deg"
              % (cfg_c.NEUTRAL_ARM_DEG, delta_clamped, left_target))
        print("    right_target = %.0f - %.4f/2 = %.4f deg"
              % (cfg_c.NEUTRAL_ARM_DEG, delta_clamped, right_target))
        print("\n  Slew rate limit (first tick, prev=NEUTRAL=%.0f deg):"
              % cfg_c.NEUTRAL_ARM_DEG)
        print("    max_step     = %.0f deg/s x 0.1 s = %.1f deg"
              % (cfg_c.MAX_ARM_RATE_DEG_S, step))
        print("    left_angle   = %.4f deg  (slewed from %.0f)"
              % (cmd.left_angle_deg,  cfg_c.NEUTRAL_ARM_DEG))
        print("    right_angle  = %.4f deg  (slewed from %.0f)"
              % (cmd.right_angle_deg, cfg_c.NEUTRAL_ARM_DEG))
        print("    delta_arm    = left - right = %.4f - %.4f = %.4f deg"
              % (cmd.left_angle_deg, cmd.right_angle_deg, actual_delta))
    else:
        print('  -> fallback: mode="%s"' % cmd.mode)

    # ==========================================================================
    # STEP 6  angles_to_pwm -- PWM conversion
    # ==========================================================================
    _section("STEP 6  angles_to_pwm()  -- PWM conversion")

    PULSE_PER_DEG = 2000.0 / 180.0
    LEFT_ZERO, RIGHT_ZERO = 600, 2500
    LEFT_NEUTRAL  = int(LEFT_ZERO  + NEUTRAL_ARM_DEG * PULSE_PER_DEG)
    RIGHT_NEUTRAL = int(RIGHT_ZERO - NEUTRAL_ARM_DEG * PULSE_PER_DEG)

    print("  PULSE_PER_DEG = 2000/180 = %.4f us/deg" % PULSE_PER_DEG)
    print("  left_pw  = %d + %.4f x %.4f = %d us"
          % (LEFT_ZERO, cmd.left_angle_deg,  PULSE_PER_DEG, cmd.left_pw))
    print("  right_pw = %d - %.4f x %.4f = %d us"
          % (RIGHT_ZERO, cmd.right_angle_deg, PULSE_PER_DEG, cmd.right_pw))

    # ==========================================================================
    # Final result
    # ==========================================================================
    _banner("RESULT")

    if cmd.delta_arm_deg < -0.01:
        turn_dir = "left turn"
    elif cmd.delta_arm_deg > 0.01:
        turn_dir = "right turn"
    else:
        turn_dir = "straight"

    print("  LCSG case     : %s" % g_input.lcsg_case)
    print("  GuidanceMode  : %s" % g_input.guidance_mode.value)
    print("  ctrl_mode     : %s" % cmd.mode)
    print()
    print("  +--------------------------------------------------+")
    print("  |  LEFT  arm : %7.3f deg   (PWM %4d us)        |"
          % (cmd.left_angle_deg,  cmd.left_pw))
    print("  |  RIGHT arm : %7.3f deg   (PWM %4d us)        |"
          % (cmd.right_angle_deg, cmd.right_pw))
    print("  |  delta_arm : %+7.3f deg  -> %-14s       |"
          % (cmd.delta_arm_deg, turn_dir))
    print("  |  neutral   :  %2.0f deg      L_neu=%d us  R_neu=%d us  |"
          % (NEUTRAL_ARM_DEG, LEFT_NEUTRAL, RIGHT_NEUTRAL))
    print("  +--------------------------------------------------+")

    if g_out.active:
        dist = math.hypot(p["tgt_N_m"] - (g_input.pos_N or 0.0),
                          p["tgt_E_m"] - (g_input.pos_E or 0.0))
        xtrk_dir = "left of path" if g_out.crossTrack > 0 else "right of path"
        print("\n  crossTrack   : %+.3f m  (%s)" % (g_out.crossTrack, xtrk_dir))
        print("  Nu           : %+.3f deg" % math.degrees(g_out.Nu))
        print("  dist to target: %.2f m" % dist)
    print()


# ---- entry point -------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--interactive", "-i", action="store_true",
        help="enter sensor values manually (default: use worked-example defaults)",
    )
    args = parser.parse_args()
    params = collect_inputs(args.interactive)
    simulate(params)


if __name__ == "__main__":
    main()
