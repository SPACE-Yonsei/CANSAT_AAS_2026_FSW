"""Closed-loop homing sim over EVERY control mode — text output (fast).

Unlike sim_s_curve_gps_tracking.py (which *imposes* a fixed S path and only
watches the commands), here the trajectory is *emergent*: the real FSW pipeline
(motorapp structs + guidance + control) commands a servo differential, a simple
parafoil turn model converts that differential into an actual yaw rate + motion,
and the resulting true state is fed back as GPS/IMU/baro for the next cycle.

So this answers the closed-loop question: starting off-line-of-sight, does each
mode actually steer the vehicle to the target — including the DR modes where GPS
position/motion is withheld and guidance must dead-reckon?

Vehicle / turn model (deliberately simple, documented assumptions):
  * constant ground speed V = SPEED_MPS (parafoil under canopy)
  * yaw rate produced by the realized servo differential:
        omega_cmd = K_TURN_DPS_PER_ARMDEG * (right_angle - left_angle)
    first-order lag (TURN_TAU_S) toward omega_cmd, clamped to +-OMEGA_MAX_DPS.
  * course integrates omega; position integrates V along course.
  * The true omega is fed back as IMU gyrz, so a non-zero PID would regulate it.

Local NE frame: +N north, +E east. Target is due north of the origin line.
Servo zero (deg=0): right=RIGHT_SERVO_ZERO_US us, left=LEFT_SERVO_ZERO_US us.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from lib import config
from Sensor_Motor import control, guidance, motorapp
from Sensor_Motor.guidance import ControlMode

# ── scenario ──────────────────────────────────────────────────────────────────
ORIGIN_LAT      = 37.0
ORIGIN_LON      = 127.0
TARGET_N_M      = 100.0          # target due north of the origin
TARGET_E_M      = 0.0
START_E_M       = 35.0           # vehicle starts off to the east (lateral offset)
START_N_M       = 0.0
START_COURSE    = 0.0            # initially heading north -> must steer left to home
SPEED_MPS       = 5.0

# turn model
K_TURN_DPS_PER_ARMDEG = 0.25     # deg/s of yaw per deg of (R-L) arm differential
OMEGA_MAX_DPS         = 45.0     # physical parafoil yaw-rate ceiling
TURN_TAU_S            = 0.4      # actuator + canopy response lag

DT          = 0.1
T0          = 1000.0
N_WARMUP    = 5
T_MAX_S     = 80.0
R_ARRIVE_M  = 5.0                # stop when within this of the target
N_SAMPLES   = 10

EARTH_RADIUS_M  = guidance.EARTH_RADIUS_M
_STALE_AGE      = 2.0 * config.GPS_FRESH_MAX_AGE_S
_SINK_FOR_SPEED = SPEED_MPS / max(config.DR_SINK_TO_HSPEED_GAIN, 1e-6)

ALL_MODES = [m for m in ControlMode if m is not ControlMode.FAIL]


def ne_to_latlon(N: float, E: float) -> tuple[float, float]:
    dlat = math.degrees(N / EARTH_RADIUS_M)
    dlon = math.degrees(E / (EARTH_RADIUS_M * math.cos(math.radians(ORIGIN_LAT))))
    return ORIGIN_LAT + dlat, ORIGIN_LON + dlon


TARGET_LAT, TARGET_LON = ne_to_latlon(TARGET_N_M, TARGET_E_M)


# ── per-mode sensor-freshness recipe (reuses guidance's own decoders) ────────
def _freshness_for(mode: ControlMode) -> dict:
    name = mode.value
    is_gps      = guidance._is_gps_tracking_mode(mode)
    is_gps_open = mode is ControlMode.GPS_TRACKING_OPEN
    is_dr       = guidance._is_dr_mode(mode)
    is_dr_pm    = name.startswith("DR_PM_")
    return dict(
        gps_pos    = is_gps or (is_dr and not is_dr_pm),
        gps_motion = is_gps,
        gyrz       = (is_gps and not is_gps_open) or (is_dr and guidance._mode_uses_gyro(mode)),
        yaw        = is_dr,
        baro       = is_dr and guidance._mode_uses_baro(mode),
        acc        = is_dr and guidance._mode_uses_acc(mode),
    )


def _make_sensors(mode, e, n, course, omega_dps, now):
    """Build (gps, imu, baro) from the TRUE vehicle state under `mode`'s recipe."""
    fr = _freshness_for(mode)
    lat, lon = ne_to_latlon(n, e)
    gps = motorapp._GpsFromApp(
        lat=lat, lon=lon,
        pos_ts=now if fr["gps_pos"] else now - _STALE_AGE, pos_health=1,
        course_rad=course, speed_mps=SPEED_MPS,
        motion_ts=now if fr["gps_motion"] else now - _STALE_AGE, motion_health=1,
        rx_ts=now,
    )
    imu = motorapp._ImuFromApp(
        yaw_rad=course if fr["yaw"] else None,
        gyrz_rad_s=math.radians(omega_dps) if fr["gyrz"] else None,  # nav sign
        lin_acc_x=0.0 if fr["acc"] else None,
        lin_acc_y=0.0 if fr["acc"] else None,
        lin_acc_valid=bool(fr["acc"]),
        ts=now, rx_ts=now, health=1,
    )
    baro = motorapp._BaroFromApp(
        alt_m=500.0 - SPEED_MPS * (now - T0),
        sink_rate=_SINK_FOR_SPEED,
        rx_ts=now if fr["baro"] else now - _STALE_AGE, health=1,
    )
    return gps, imu, baro


def _warmup_sensors(e, n, course, now):
    lat, lon = ne_to_latlon(n, e)
    gps = motorapp._GpsFromApp(
        lat=lat, lon=lon, pos_ts=now, pos_health=1,
        course_rad=course, speed_mps=SPEED_MPS, motion_ts=now, motion_health=1,
        rx_ts=now,
    )
    imu = motorapp._ImuFromApp(
        yaw_rad=course, gyrz_rad_s=0.0, lin_acc_x=0.0, lin_acc_y=0.0,
        lin_acc_valid=True, ts=now, rx_ts=now, health=1,
    )
    baro = motorapp._BaroFromApp(alt_m=500.0, sink_rate=_SINK_FOR_SPEED, rx_ts=now, health=1)
    return gps, imu, baro


def _run_cycle(gps, imu, baro, now):
    motorapp._CACHE_t.latest_gps  = gps
    motorapp._CACHE_t.latest_imu  = imu
    motorapp._CACHE_t.latest_baro = baro
    snap = motorapp._cache_snapshot()
    mode  = guidance.DecideControlMode(snap.latest_gps, snap.latest_imu, snap.latest_baro, now)
    l1in  = guidance.ProduceL1Input(now)
    l1out = guidance.ProduceL1Output(l1in)
    gz = math.degrees(float(snap.latest_imu.gyrz_rad_s or 0.0))
    ci = control.ProduceCtrlInput(l1out, now)
    co = control.ProduceCtrlOutput(ci, gz, now)
    return mode, l1out, co


def run_mode(mode: ControlMode) -> dict:
    """Fly the vehicle under `mode` from the offset start until it reaches the
    target or T_MAX_S. Returns per-step true trajectory + commands."""
    guidance.reset()
    control.reset()
    guidance.set_origin_point(ORIGIN_LAT, ORIGIN_LON)
    guidance.set_target_point(TARGET_LAT, TARGET_LON)

    # true vehicle state
    e, n, course, omega = START_E_M, START_N_M, START_COURSE, 0.0

    now = T0
    for _ in range(N_WARMUP):                # lock DR anchor at the start point
        _run_cycle(*_warmup_sensors(e, n, course, now), now)
        now += DT

    rec = {k: [] for k in ("t", "E", "N", "dist", "nu", "av", "arm", "mode", "valid")}
    arrived = False
    n_steps = int(T_MAX_S / DT)
    for _ in range(n_steps):
        mode_sel, l1out, co = _run_cycle(
            *_make_sensors(mode, e, n, course, omega, now), now)

        dist = math.hypot(TARGET_E_M - e, TARGET_N_M - n)
        rec["t"].append(now - T0)
        rec["E"].append(e); rec["N"].append(n); rec["dist"].append(dist)
        rec["nu"].append(math.degrees(l1out.nu) if math.isfinite(l1out.nu) else float("nan"))
        rec["av"].append(co.angular_velocity_cmd_deg_s)
        rec["arm"].append(co.right_angle_deg - co.left_angle_deg)
        rec["mode"].append(mode_sel.value)
        rec["valid"].append(bool(co.valid))

        if dist <= R_ARRIVE_M:
            arrived = True
            break

        # ── turn model: realized arm differential -> yaw rate -> motion ──────
        arm = co.right_angle_deg - co.left_angle_deg
        omega_cmd = max(-OMEGA_MAX_DPS, min(OMEGA_MAX_DPS, K_TURN_DPS_PER_ARMDEG * arm))
        omega += (omega_cmd - omega) * (DT / TURN_TAU_S)          # first-order lag
        course = (course + math.radians(omega) * DT + math.pi) % (2 * math.pi) - math.pi
        e += SPEED_MPS * math.sin(course) * DT
        n += SPEED_MPS * math.cos(course) * DT
        now += DT

    rec["arrived"] = arrived
    return rec


# ── output ────────────────────────────────────────────────────────────────────
def _sample_idx(n: int) -> list[int]:
    if n <= N_SAMPLES:
        return list(range(n))
    return [int(round(k * (n - 1) / (N_SAMPLES - 1))) for k in range(N_SAMPLES)]


def print_table(rec: dict) -> None:
    idx = _sample_idx(len(rec["t"]))
    seen = sorted(set(rec["mode"]))
    res = (f"ARRIVED in {rec['t'][-1]:.1f}s (final dist {rec['dist'][-1]:.1f} m)"
           if rec["arrived"]
           else f"NOT reached: ran {rec['t'][-1]:.1f}s, final dist {rec['dist'][-1]:.1f} m")
    print(f"  {res}")
    print(f"  modes seen: {seen}")
    print(f"  {'#':>2} {'t(s)':>5} {'E(m)':>7} {'N(m)':>7} {'dist':>6} | "
          f"{'nu(deg)':>8} {'av(d/s)':>8} {'R-L(deg)':>9}")
    print("  " + "-" * 66)
    for k, i in enumerate(idx):
        print(f"  {k:>2} {rec['t'][i]:>5.1f} {rec['E'][i]:>7.2f} {rec['N'][i]:>7.2f} "
              f"{rec['dist'][i]:>6.1f} | {rec['nu'][i]:>8.2f} {rec['av'][i]:>8.2f} "
              f"{rec['arm'][i]:>9.2f}")


def draw_diagram(rec: dict) -> None:
    H, W = 19, 40
    e_lim = max(abs(START_E_M), 10.0) * 1.4
    n_top, n_bot = TARGET_N_M + 5.0, -5.0

    def rc(E, N):
        row = int(round((n_top - N) / (n_top - n_bot) * (H - 1)))
        col = int(round((E + e_lim) / (2 * e_lim) * W))
        return max(0, min(H - 1, row)), max(0, min(W, col))

    grid = [[" "] * (W + 1) for _ in range(H)]
    axis_col = rc(0.0, 0.0)[1]
    for r in range(H):
        grid[r][axis_col] = "."
    # full emergent trajectory as '*', then numbered samples on top
    for e, n in zip(rec["E"], rec["N"]):
        r, c = rc(e, n)
        if grid[r][c] == " " or grid[r][c] == ".":
            grid[r][c] = "*"
    for k, i in enumerate(_sample_idx(len(rec["t"]))):
        r, c = rc(rec["E"][i], rec["N"][i])
        grid[r][c] = str(k % 10)
    r_t, c_t = rc(TARGET_E_M, TARGET_N_M); grid[r_t][c_t] = "T"
    r_s, c_s = rc(START_E_M, START_N_M);   grid[r_s][c_s] = "S"

    print("  trajectory  (T=target, S=start, *=path, digits=samples; x=East, y=North)")
    for r in range(H):
        n_here = n_top - r / (H - 1) * (n_top - n_bot)
        label = f"{n_here:5.0f} " if r % 3 == 0 else "      "
        print(label + "".join(grid[r]))
    print("       " + "-" * (W + 1))
    print(f"       E(m): {-e_lim:+.0f}" + " " * 13 + "0" + " " * 13 + f"{e_lim:+.0f}")


def main() -> None:
    print("=" * 72)
    print("CLOSED-LOOP HOMING SWEEP OVER ALL CONTROL MODES")
    print(f"start=({START_E_M:.0f},{START_N_M:.0f}) course={math.degrees(START_COURSE):.0f}deg, "
          f"target=(0,{TARGET_N_M:.0f}), v={SPEED_MPS:.0f} m/s")
    print(f"turn model: omega={K_TURN_DPS_PER_ARMDEG}*(R-L) dps, "
          f"|omega|<={OMEGA_MAX_DPS}, tau={TURN_TAU_S}s; arrive R={R_ARRIVE_M} m")
    print("servo zero: right=%d us, left=%d us  (R-L arm>0 => right turn)"
          % (control.RIGHT_ZERO_PULSE, control.LEFT_ZERO_PULSE))
    print("=" * 72)
    for mode in ALL_MODES:
        rec = run_mode(mode)
        print()
        print(f"### {mode.value}")
        print_table(rec)
        draw_diagram(rec)


if __name__ == "__main__":
    main()
