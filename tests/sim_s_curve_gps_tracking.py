"""S-curve sweep over EVERY control mode — text output (fast, no plotting).

A fixed S-shaped path joins origin (bottom of the S) to target (top). The path
(position + course) is prescribed externally — as if a giant carried the payload
along it — and we only observe what guidance/control *command* at each point.

Local NE frame (origin = (0,0)):
  y-axis = North = the origin->target line (target is due north of origin)
  x-axis = East  = perpendicular to that line  (S bulges +E then -E)

For each of guidance.ControlMode's non-FAIL modes we drive the *real* FSW
pipeline (motorapp structs/handlers + guidance + control, no monkeypatched math)
and, for ~10 samples along the path, print 4 quantities:

  1. coordinate (E, N)  metres   — the position guidance actually navigates on
                                   (true path for GPS / DR_M, dead-reckoned for DR_PM)
  2. nu                  deg      — L1Output.nu, target-bearing error
  3. av_cmd              deg/s    — CtrlOutput.angular_velocity_cmd_deg_s
  4. R-L arm             deg      — right_angle_deg - left_angle_deg (realized servo
                                   differential, post slew-rate limit)

Each mode is forced by feeding the sensor-freshness pattern that
guidance.DecideControlMode/FillNav requires for it (GPS pos/motion, gyrz, yaw, baro
sink, lin-acc). The actually-selected mode is verified per step.

Servo zero calibration (deg=0): right pulse = RIGHT_SERVO_ZERO_US (636 us),
left pulse = LEFT_SERVO_ZERO_US (2480 us). R-L arm > 0 commands a right turn.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from lib import config
from Sensor_Motor import control, guidance, motorapp
from Sensor_Motor.guidance import ControlMode

# ── scenario parameters ───────────────────────────────────────────────────────
ORIGIN_LAT      = 37.0
ORIGIN_LON      = 127.0
TARGET_DIST_N_M = 100.0          # target due north of origin
TARGET_DIST_E_M = 0.0
SPEED_MPS       = 5.0
S_AMPLITUDE_M   = 30.0           # east bulge of the S
DT              = 0.1            # 10 Hz, matches config.MOTOR_RATE_HZ
# N_STEPS is derived below from the arc length so ground speed = SPEED_MPS.
T0              = 1000.0         # synthetic monotonic clock start
N_WARMUP        = 5             # full-GPS cycles to lock the DR anchor + settle baro
N_SAMPLES       = 10            # rows printed / markers drawn per mode

EARTH_RADIUS_M  = guidance.EARTH_RADIUS_M

# baro sink that yields ~SPEED_MPS through the DR sink->hspeed gain
_SINK_FOR_SPEED = SPEED_MPS / max(config.DR_SINK_TO_HSPEED_GAIN, 1e-6)
# age (s) stamped on a "stale" GPS channel so it reads stale *immediately*,
# overriding any lingering fresh timestamp left by the warm-up cycles.
_STALE_AGE = 2.0 * config.GPS_FRESH_MAX_AGE_S

# all modes worth exercising, in selection-priority order
ALL_MODES = [m for m in ControlMode if m is not ControlMode.FAIL]


# ── geometry ──────────────────────────────────────────────────────────────────
def ne_to_latlon(N: float, E: float,
                 origin_lat: float, origin_lon: float) -> tuple[float, float]:
    dlat = math.degrees(N / EARTH_RADIUS_M)
    dlon = math.degrees(E / (EARTH_RADIUS_M * math.cos(math.radians(origin_lat))))
    return origin_lat + dlat, origin_lon + dlon


# Shape: E(t) = A * sin(2*pi*t) * sin^2(pi*t)  -> dE/dt = 0 at t=0 and t=1
#   => course = north at origin & target => nu ~ 0 and arm ~ 0 at the endpoints.
# We then RE-SAMPLE the shape at equal ARC-LENGTH steps of ds = SPEED*DT so the
# ground speed is genuinely constant SPEED_MPS everywhere. Without this the raw
# t-parametrization runs 5..10 m/s along the curve while we feed a flat 5 m/s,
# which makes the dead-reckoned DR_PM modes under-progress in N (a sim artifact,
# not an FSW fault).
_dense  = np.linspace(0.0, 1.0, 20001)
_dE     = S_AMPLITUDE_M * np.sin(2.0 * math.pi * _dense) * np.sin(math.pi * _dense) ** 2
_dN     = TARGET_DIST_N_M * _dense
_s      = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(_dE), np.diff(_dN)))])
_ds     = SPEED_MPS * DT
N_STEPS = int(_s[-1] // _ds) + 1
_s_samp = np.arange(N_STEPS) * _ds
E_PATH  = np.interp(_s_samp, _s, _dE)
N_PATH  = np.interp(_s_samp, _s, _dN)
COURSE  = np.arctan2(np.gradient(E_PATH), np.gradient(N_PATH))
# nav-convention yaw rate (rad/s, right turn = +) along the real-time clock
GYRZ    = np.gradient(np.unwrap(COURSE), DT)

TARGET_LAT, TARGET_LON = ne_to_latlon(TARGET_DIST_N_M, TARGET_DIST_E_M,
                                      ORIGIN_LAT, ORIGIN_LON)


# ── per-mode sensor-freshness recipe ─────────────────────────────────────────
def _freshness_for(mode: ControlMode) -> dict:
    """Which sensor channels must be fresh for FillNav to pick `mode`.

    Uses guidance's own mode-decoders so the recipe tracks the source code.
    """
    name = mode.value
    is_gps      = guidance._is_gps_tracking_mode(mode)
    is_gps_open = mode is ControlMode.GPS_TRACKING_OPEN
    is_dr       = guidance._is_dr_mode(mode)
    is_dr_pm    = name.startswith("DR_PM_")

    return dict(
        # GPS position fresh for GPS tracking and DR_M_*; stale for DR_PM_*.
        gps_pos    = is_gps or (is_dr and not is_dr_pm),
        # GPS motion (course/speed) fresh only while truly GPS-tracking.
        gps_motion = is_gps,
        # gyro: closed GPS + any G-source DR mode; OPEN GPS forces it stale.
        gyrz       = (is_gps and not is_gps_open) or (is_dr and guidance._mode_uses_gyro(mode)),
        # yaw is always supplied for DR (bootstrap / Y-source modes need it).
        yaw        = is_dr,
        baro       = is_dr and guidance._mode_uses_baro(mode),
        acc        = is_dr and guidance._mode_uses_acc(mode),
    )


def _make_sensors(mode: ControlMode, i: int, now: float):
    """Build (gps, imu, baro) raw-sensor structs for path step i under `mode`.

    Channels that must be stale are sent with health=0 / value None so
    guidance.UpdateRaw leaves their last-good timestamp old (=> stale).
    """
    fr = _freshness_for(mode)
    E_pos  = float(E_PATH[i]); N_pos = float(N_PATH[i])
    course = float(COURSE[i]); gyrz  = float(GYRZ[i])
    lat, lon = ne_to_latlon(N_pos, E_pos, ORIGIN_LAT, ORIGIN_LON)

    # Always health=1 with valid values; freshness is decided purely by the
    # timestamp (now = fresh, now-_STALE_AGE = stale). Using health=0 would
    # instead leave the warm-up's last-good timestamp in place, which stays
    # "fresh" for GPS_FRESH_MAX_AGE_S and would mis-select GPS tracking.
    gps = motorapp._GpsFromApp(
        lat=lat, lon=lon,
        pos_ts=now if fr["gps_pos"] else now - _STALE_AGE,
        pos_health=1,
        course_rad=course, speed_mps=SPEED_MPS,
        motion_ts=now if fr["gps_motion"] else now - _STALE_AGE,
        motion_health=1,
        rx_ts=now,
    )

    imu = motorapp._ImuFromApp(
        yaw_rad=course if fr["yaw"] else None,
        gyrz_rad_s=gyrz if fr["gyrz"] else None,   # nav sign (right = +)
        lin_acc_x=0.0 if fr["acc"] else None,
        lin_acc_y=0.0 if fr["acc"] else None,
        lin_acc_valid=bool(fr["acc"]),
        ts=now,
        rx_ts=now,
        health=1,
    )

    # Same fresh-vs-stale-by-timestamp trick as GPS: a stale baro must carry an
    # old rx_ts (not health=0) or the warm-up's baro lingers fresh for
    # BARO_FRESH_MAX_AGE_S and pulls a G/Y mode into its *B variant.
    baro = motorapp._BaroFromApp(
        alt_m=500.0 - SPEED_MPS * (now - T0),
        sink_rate=_SINK_FOR_SPEED,
        rx_ts=now if fr["baro"] else now - _STALE_AGE,
        health=1,
    )
    return gps, imu, baro


def _warmup_sensors(now: float):
    """Full GPS-fresh cycle at the origin (course = north) — locks DR anchor."""
    lat, lon = ORIGIN_LAT, ORIGIN_LON
    gps = motorapp._GpsFromApp(
        lat=lat, lon=lon, pos_ts=now, pos_health=1,
        course_rad=0.0, speed_mps=SPEED_MPS, motion_ts=now, motion_health=1,
        rx_ts=now,
    )
    imu = motorapp._ImuFromApp(
        yaw_rad=0.0, gyrz_rad_s=0.0, lin_acc_x=0.0, lin_acc_y=0.0,
        lin_acc_valid=True, ts=now, rx_ts=now, health=1,
    )
    baro = motorapp._BaroFromApp(
        alt_m=500.0, sink_rate=_SINK_FOR_SPEED, rx_ts=now, health=1,
    )
    return gps, imu, baro


# ── one full pipeline cycle (mirrors motorapp._ctrl_cycle's core) ────────────
def _run_cycle(gps, imu, baro, now: float):
    """DecideControlMode -> ProduceL1Input/Output -> control I/O. Returns (mode, l1out, co)."""
    # publish into motorapp's shared cache, then snapshot exactly like _ctrl_cycle.
    motorapp._CACHE_t.latest_gps  = gps
    motorapp._CACHE_t.latest_imu  = imu
    motorapp._CACHE_t.latest_baro = baro
    snap = motorapp._cache_snapshot()

    mode  = guidance.DecideControlMode(snap.latest_gps, snap.latest_imu,
                                       snap.latest_baro, now)
    l1in  = guidance.ProduceL1Input(now)
    l1out = guidance.ProduceL1Output(l1in)

    gz_meas = math.degrees(float(snap.latest_imu.gyrz_rad_s or 0.0))
    ci = control.ProduceCtrlInput(l1out, now)
    co = control.ProduceCtrlOutput(ci, gz_meas, now)
    return mode, l1out, co


def run_mode(mode: ControlMode) -> dict:
    """Carry the payload along the S-curve while forcing `mode`. Returns arrays."""
    # fresh frame each mode (reset() preserves only target lat/lon)
    guidance.reset()
    control.reset()
    guidance.set_origin_point(ORIGIN_LAT, ORIGIN_LON)
    guidance.set_target_point(TARGET_LAT, TARGET_LON)

    now = T0
    for _ in range(N_WARMUP):
        _run_cycle(*_warmup_sensors(now), now)
        now += DT

    rec = {k: [] for k in
           ("E", "N", "nu", "av", "arm", "mode", "valid")}
    for i in range(N_STEPS):
        m, l1out, co = _run_cycle(*_make_sensors(mode, i, now), now)
        # coordinate guidance navigates on (true path or DR estimate)
        E = l1out.pos_E if math.isfinite(l1out.pos_E) else float(E_PATH[i])
        N = l1out.pos_N if math.isfinite(l1out.pos_N) else float(N_PATH[i])
        rec["E"].append(E)
        rec["N"].append(N)
        rec["nu"].append(math.degrees(l1out.nu) if math.isfinite(l1out.nu) else float("nan"))
        rec["av"].append(co.angular_velocity_cmd_deg_s)
        rec["arm"].append(co.right_angle_deg - co.left_angle_deg)
        rec["mode"].append(m.value if hasattr(m, "value") else str(m))
        rec["valid"].append(bool(co.valid))
        now += DT
    return rec


# ── sampling ──────────────────────────────────────────────────────────────────
SAMPLE_IDX = [int(round(k * (N_STEPS - 1) / (N_SAMPLES - 1))) for k in range(N_SAMPLES)]


# ── ASCII diagram: origin -> target arrow as the y-axis ──────────────────────
def draw_diagram(rec: dict) -> None:
    H = 19
    e_lim = max(S_AMPLITUDE_M, 5.0) * 1.2
    n_top, n_bot = TARGET_DIST_N_M, 0.0
    W = 40

    def rc(E, N):
        row = int(round((n_top - N) / (n_top - n_bot) * (H - 1)))
        col = int(round((E + e_lim) / (2 * e_lim) * W))
        return max(0, min(H - 1, row)), max(0, min(W, col))

    grid = [[" "] * (W + 1) for _ in range(H)]
    axis_col = rc(0.0, 0.0)[1]
    for r in range(H):                       # y-axis = origin->target line
        grid[r][axis_col] = "|"
    grid[0][axis_col] = "^"                  # arrowhead at target

    for k, idx in enumerate(SAMPLE_IDX):
        r, c = rc(rec["E"][idx], rec["N"][idx])
        grid[r][c] = str(k % 10)
    r_t, c_t = rc(TARGET_DIST_E_M, TARGET_DIST_N_M); grid[r_t][c_t] = "T"
    r_o, c_o = rc(0.0, 0.0);                          grid[r_o][c_o] = "O"

    print("  diagram  (y = North = origin->target arrow, x = East; digits = samples)")
    for r in range(H):
        n_here = n_top - r / (H - 1) * (n_top - n_bot)
        label = f"{n_here:5.0f} " if r % 3 == 0 else "      "
        print(label + "".join(grid[r]))
    print("       " + "-" * (W + 1))
    print(f"       E(m): {-e_lim:+.0f}" + " " * 13 + "0" + " " * 13 + f"{e_lim:+.0f}")


# ── per-mode 4-value table ───────────────────────────────────────────────────
def print_table(rec: dict) -> None:
    seen = sorted(set(rec["mode"]))
    held = all(m == rec["mode"][0] for m in rec["mode"])
    note = "" if held else "  <-- mode varied along path!"
    print(f"  selected modes seen: {seen}{note}")
    print(f"  {'#':>2} {'E(m)':>7} {'N(m)':>7} | {'nu(deg)':>8} "
          f"{'av_cmd(d/s)':>11} {'R-L arm(deg)':>12}  {'ok':>3}")
    print("  " + "-" * 62)
    for k, idx in enumerate(SAMPLE_IDX):
        print(f"  {k:>2} {rec['E'][idx]:>7.2f} {rec['N'][idx]:>7.2f} | "
              f"{rec['nu'][idx]:>8.2f} {rec['av'][idx]:>11.2f} "
              f"{rec['arm'][idx]:>12.2f}  {('Y' if rec['valid'][idx] else '-'):>3}")


# ── main ──────────────────────────────────────────────────────────────────────
def main() -> None:
    print("=" * 72)
    print("S-CURVE SWEEP OVER ALL CONTROL MODES")
    print(f"origin=(0,0) bottom of S, target=(0,{TARGET_DIST_N_M:.0f}) top, "
          f"|S|={S_AMPLITUDE_M:.0f} m E, v={SPEED_MPS:.0f} m/s, "
          f"{N_STEPS} steps @ {DT:.2f}s")
    print("servo zero: right=%d us, left=%d us  (R-L arm>0 => right turn)"
          % (control.RIGHT_ZERO_PULSE, control.LEFT_ZERO_PULSE))
    print("4 outputs per sample: (E,N) coord | nu | av_cmd | R-L arm")
    print("=" * 72)

    for mode in ALL_MODES:
        rec = run_mode(mode)
        print()
        print(f"### {mode.value}")
        print_table(rec)
        draw_diagram(rec)


if __name__ == "__main__":
    main()
