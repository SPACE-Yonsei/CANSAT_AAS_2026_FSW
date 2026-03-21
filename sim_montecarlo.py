#!/usr/bin/env python3
"""
CanSat Parafoil — Monte-Carlo Boundary Analysis
=================================================
풍속별 × 다수 시드로 시뮬레이션을 반복 실행하여,
5m / 10m / 50m 착지 성공 확률의 경계를 탐색한다.

Run:  python sim_montecarlo.py
"""

# ── stdout/stderr encoding ───────────────────────────────────────────────────
import io, sys
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

import math, os, types as _t
from collections import deque
from types import SimpleNamespace

import numpy as np

# ══════════════════════════════════════════════════════════════════════════════
# Hardware / lib mocks (same as sim_hifi)
# ══════════════════════════════════════════════════════════════════════════════

class _MockPi:
    def __init__(self): self._p = {}
    def set_servo_pulsewidth(self, pin, pw): self._p[pin] = pw
    def stop(self): pass

sys.modules['pigpio'] = _t.ModuleType('pigpio')
sys.modules['pigpio'].pi = _MockPi

class _GPIO:
    BCM = 11; OUT = 0; HIGH = 1; LOW = 0
    @staticmethod
    def setmode(m): pass
    @staticmethod
    def setup(pin, mode, initial=0): pass
    @staticmethod
    def output(pin, val): pass
    @staticmethod
    def cleanup(pin=None): pass

_rpi = _t.ModuleType('RPi'); _rpi.GPIO = _GPIO; sys.modules['RPi'] = _rpi
_rpigpio = _t.ModuleType('RPi.GPIO')
for _a in ('BCM', 'OUT', 'HIGH', 'LOW', 'setmode', 'setup', 'output', 'cleanup'):
    setattr(_rpigpio, _a, getattr(_GPIO, _a))
sys.modules['RPi.GPIO'] = _rpigpio

_lib = _t.ModuleType('lib'); sys.modules['lib'] = _lib
_appargs = _t.ModuleType('lib.appargs')
class _NS:
    def __init__(self, **kw): self.__dict__.update(kw)
_appargs.MainAppArg        = _NS(AppID=10, AppName='Main', MID_TerminateProcess=1001001)
_appargs.MotorAppArg       = _NS(AppID=19, AppName='Motor')
_appargs.FlightlogicAppArg = _NS(MID_motor_TargetCor=1101901, MID_motor_state=1101902,
                                  MID_motor_burnwire=1101903, MID_motor_EggDrop=1101904,
                                  MID_motor_PullArms=1101905)
_appargs.GpsAppArg         = _NS(MID_motor_gps=1501901)
_appargs.ImuAppArg         = _NS(MID_motor_imu=1401901)
_appargs.BarometerAppArg   = _NS(MID_motor_alt=1301901)
_appargs.CommAppArg        = _NS(MID_RouteCmd_MEC=1607)
sys.modules['lib.appargs'] = _appargs; _lib.appargs = _appargs

_events = _t.ModuleType('lib.events')
_events.EventType = _NS(error=0, info=1, debug=2, warning=3)
_events.LogEvent  = lambda *a, **k: None
sys.modules['lib.events'] = _events; _lib.events = _events

_msgmod = _t.ModuleType('lib.msgstructure')
sys.modules['lib.msgstructure'] = _msgmod; _lib.msgstructure = _msgmod

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from Sensor_Motor import motor_guidance, motor_control

import time as _real_time
_sim_clock = [0.0]

class _SimTimeMod:
    @staticmethod
    def time() -> float:
        return _sim_clock[0]
    @staticmethod
    def sleep(s):
        pass
    def __getattr__(self, name):
        return getattr(_real_time, name)

motor_guidance.time = _SimTimeMod()

# ══════════════════════════════════════════════════════════════════════════════
# Coordinate helpers
# ══════════════════════════════════════════════════════════════════════════════

REF_LAT  = 35.0950
REF_LON  = 127.0950
LAT2M    = 111320.0
COS_LAT  = math.cos(math.radians(REF_LAT))

def en_to_latlon(E_m, N_m):
    return REF_LAT + N_m / LAT2M, REF_LON + E_m / (LAT2M * COS_LAT)

# ══════════════════════════════════════════════════════════════════════════════
# Fixed constants (extreme scenario from sim_hifi)
# ══════════════════════════════════════════════════════════════════════════════

VA_BASE      = 8.0
DESCENT_BASE = 3.5
WIND_Z_REF   = 600.0
WIND_ALPHA   = 0.30
L_HOR        = 150.0
L_VER        = 30.0
SIG_HOR      = 4.0
SIG_VER      = 2.5
OMEGA_N      = math.sqrt(9.81 / 1.5)
ZETA         = 0.08
K_COUPLE     = 0.10
SLEW_RATE_DEG_S = 200.0
DEADBAND_MECH   = 5.0
GPS_NOISE_M  = 3.5
GPS_DELAY    = 4
GPS_WARMUP   = 12
YAW_NOISE_DEG    = 4.0
GYRZ_NOISE_RPS   = math.radians(1.5)
BIAS_DRIFT_RATE  = math.radians(0.06)
K_MAG_PEND       = 0.8
BARO_NOISE_M = 3.0
DT           = 0.1
MAX_STEPS    = 3000

# ══════════════════════════════════════════════════════════════════════════════
# Physics model functions (identical to sim_hifi)
# ══════════════════════════════════════════════════════════════════════════════

# --- module-level state (reset per run) ---
_turb_u = 0.0; _turb_v = 0.0; _turb_w = 0.0
_servo_delta = 0.0; _pend_phi = 0.0; _pend_phidot = 0.0; _imu_bias = 0.0
_E_buf = deque(maxlen=GPS_DELAY); _N_buf = deque(maxlen=GPS_DELAY)
_spd_buf = deque(maxlen=GPS_DELAY); _crs_buf = deque(maxlen=GPS_DELAY)
_rng = None  # set per run
_WIND_UNIT_E = 0.0; _WIND_UNIT_N = 0.0; _WIND_U_REF = 5.0

def _reset_state(rng, wind_u_ref, wind_dir_met):
    global _turb_u, _turb_v, _turb_w, _servo_delta
    global _pend_phi, _pend_phidot, _imu_bias
    global _E_buf, _N_buf, _spd_buf, _crs_buf
    global _rng, _WIND_UNIT_E, _WIND_UNIT_N, _WIND_U_REF
    _turb_u = _turb_v = _turb_w = 0.0
    _servo_delta = _pend_phi = _pend_phidot = _imu_bias = 0.0
    _E_buf = deque(maxlen=GPS_DELAY); _N_buf = deque(maxlen=GPS_DELAY)
    _spd_buf = deque(maxlen=GPS_DELAY); _crs_buf = deque(maxlen=GPS_DELAY)
    _rng = rng; _WIND_U_REF = wind_u_ref
    wr = math.radians(wind_dir_met + 180.0)
    _WIND_UNIT_E = math.sin(wr); _WIND_UNIT_N = math.cos(wr)

def wind_at_alt(alt_m):
    z = max(alt_m, 1.0)
    U = _WIND_U_REF * (z / WIND_Z_REF) ** WIND_ALPHA
    return U * _WIND_UNIT_E, U * _WIND_UNIT_N

def step_turbulence(Va, dt):
    global _turb_u, _turb_v, _turb_w
    rho_h = math.exp(-Va * dt / L_HOR)
    rho_v = math.exp(-Va * dt / L_VER)
    sh = SIG_HOR * math.sqrt(max(0.0, 1.0 - rho_h**2))
    sv = SIG_VER * math.sqrt(max(0.0, 1.0 - rho_v**2))
    _turb_u = rho_h * _turb_u + sh * _rng.standard_normal()
    _turb_v = rho_h * _turb_v + sh * _rng.standard_normal()
    _turb_w = rho_v * _turb_w + sv * _rng.standard_normal()

def step_servo(delta_target, dt):
    global _servo_delta
    ms = SLEW_RATE_DEG_S * dt
    _servo_delta += max(-ms, min(ms, delta_target - _servo_delta))
    return math.copysign(max(0.0, abs(_servo_delta) - DEADBAND_MECH), _servo_delta)

def step_pendulum(yr, dt):
    global _pend_phi, _pend_phidot
    ddphi = (-2*ZETA*OMEGA_N*math.radians(_pend_phidot)
             - OMEGA_N**2*math.radians(_pend_phi)
             + math.radians(K_COUPLE*yr))
    _pend_phidot += math.degrees(ddphi) * dt
    _pend_phi += _pend_phidot * dt

def sensor_gps(E, N, VE, VN, step):
    En = E + _rng.normal(0, GPS_NOISE_M); Nn = N + _rng.normal(0, GPS_NOISE_M)
    spd = math.hypot(VE, VN); crs = math.degrees(math.atan2(VE, VN)) % 360
    _E_buf.append(En); _N_buf.append(Nn); _spd_buf.append(spd); _crs_buf.append(crs)
    warm = step >= GPS_WARMUP and len(_E_buf) == GPS_DELAY
    if warm:
        lat, lon = en_to_latlon(_E_buf[0], _N_buf[0])
        return SimpleNamespace(lat=lat, lon=lon, speed=_spd_buf[0], course=_crs_buf[0]), \
               SimpleNamespace(fix_quality=1, sats=8, rmc_status="A")
    lat, lon = en_to_latlon(0, 0)
    return SimpleNamespace(lat=lat, lon=lon, speed=0, course=0), \
           SimpleNamespace(fix_quality=0, sats=0, rmc_status="V")

def sensor_imu(hdg_true, yr_true):
    global _imu_bias
    yaw = hdg_true + K_MAG_PEND * _pend_phi + _rng.normal(0, YAW_NOISE_DEG)
    _imu_bias += _rng.normal(0, BIAS_DRIFT_RATE * math.sqrt(DT))
    _imu_bias = max(-0.2, min(0.2, _imu_bias))
    gyrz = math.radians(yr_true) + _imu_bias + _rng.normal(0, GYRZ_NOISE_RPS)
    return SimpleNamespace(yaw=yaw, gyrz=gyrz)

def sensor_baro(alt_true):
    return alt_true + _rng.normal(0, BARO_NOISE_M)


# ══════════════════════════════════════════════════════════════════════════════
# Single run function
# ══════════════════════════════════════════════════════════════════════════════

def run_single(seed, wind_u_ref, target_dist=None, target_bearing=None):
    """
    Returns SimpleNamespace with:
      final_dist, wind_spd, wind_dir, bearing, distance,
      init_heading, gps_inv_pct, pattern_steps, total_steps
    """
    rng = np.random.default_rng(seed)

    bearing = target_bearing if target_bearing is not None else float(rng.uniform(30, 70))
    dist    = target_dist if target_dist is not None else float(rng.uniform(550, 700))
    w_dir   = float(rng.uniform(250, 360))

    tgt_E = dist * math.sin(math.radians(bearing))
    tgt_N = dist * math.cos(math.radians(bearing))
    tgt_lat, tgt_lon = en_to_latlon(tgt_E, tgt_N)

    _reset_state(rng, wind_u_ref, w_dir)

    # FSW init
    motor_guidance.init_guidance()
    motor_guidance.set_start_coordinates(REF_LAT, REF_LON)
    motor_guidance.set_target_coord(tgt_lat, tgt_lon)
    motor_guidance.GPS_JUMP_MAX_SPEED = 200.0

    # Suppress debug prints
    orig_dbg = motor_guidance.DEBUG_GUIDANCE
    motor_guidance.DEBUG_GUIDANCE = False

    pi = _MockPi()
    E = N = 0.0; alt = 600.0
    heading = float(rng.uniform(0, 360))
    init_heading = heading
    yr_phy = 0.0; Va = VA_BASE; fstate = 3
    gps_inv = 0; pat_steps = 0

    for step in range(MAX_STEPS):
        t = step * DT
        _sim_clock[0] = t

        wE, wN = wind_at_alt(alt)
        step_turbulence(Va, DT)

        hr = math.radians(heading)
        Va_fwd = max(VA_BASE - 0.06 * abs(_servo_delta), 4.0)
        wEt = wE + _turb_u; wNt = wN + _turb_v
        VE = Va_fwd * math.sin(hr) + wEt
        VN = Va_fwd * math.cos(hr) + wNt

        gv, gf = sensor_gps(E, N, VE, VN, step)
        im = sensor_imu(heading, yr_phy)
        bm = sensor_baro(alt)

        patterned = (fstate == 4) and (bm > 10.0)

        tgt_ns = SimpleNamespace(lat=tgt_lat, lon=tgt_lon)
        gr = motor_guidance.guidance(im, gv, gf, tgt_ns, baro_m=bm, patterned=patterned)
        mr = motor_control.control(pi, gr.commanded_yaw_rate)
        eff = step_servo(mr.actual_delta_deg, DT)

        Va_fwd = max(VA_BASE - 0.03 * abs(eff), 4.0)
        dr = max(DESCENT_BASE + 0.001 * eff**2 + _turb_w * 0.3, 1.0)
        yr_phy = eff * (Va_fwd / VA_BASE)

        step_pendulum(yr_phy, DT)
        heading = (heading + yr_phy * DT) % 360
        hr = math.radians(heading)
        E += (Va_fwd * math.sin(hr) + wEt) * DT
        N += (Va_fwd * math.cos(hr) + wNt) * DT
        alt -= dr * DT
        Va = Va_fwd

        if fstate == 3 and alt < 50: fstate = 4
        if alt <= 0: fstate = 5

        if gr.state == 'GPS_INVALID': gps_inv += 1
        if gr.state == 'PATTERN': pat_steps += 1
        if fstate == 5: break

    motor_guidance.DEBUG_GUIDANCE = orig_dbg

    fd = math.hypot(tgt_E - E, tgt_N - N)
    return SimpleNamespace(
        final_dist=fd, wind_spd=wind_u_ref, wind_dir=w_dir,
        bearing=bearing, distance=dist, init_heading=init_heading,
        gps_inv_pct=100.0 * gps_inv / max(step + 1, 1),
        pattern_steps=pat_steps, total_steps=step + 1,
        seed=seed
    )


# ══════════════════════════════════════════════════════════════════════════════
# Monte-Carlo sweep
# ══════════════════════════════════════════════════════════════════════════════

if __name__ == '__main__':
    SEEDS_PER_WIND = 50
    WIND_SPEEDS = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]

    print()
    print("=" * 90)
    print("  CanSat Parafoil — Monte-Carlo Boundary Analysis")
    print(f"  {SEEDS_PER_WIND} seeds × {len(WIND_SPEEDS)} wind speeds = "
          f"{SEEDS_PER_WIND * len(WIND_SPEEDS)} runs")
    print("  Extreme params: turb=4.0, GPS_noise=3.5m, IMU_noise=4°, "
          "servo_deadband=5°, descent=3.5m/s")
    print("=" * 90)

    header = (f"  {'Wind':>6}  {'< 5m':>6}  {'< 10m':>6}  {'< 25m':>6}  "
              f"{'< 50m':>6}  {'>=150':>6}  {'Mean':>7}  {'Med':>7}  "
              f"{'P90':>7}  {'Best':>7}  {'Worst':>7}  {'GPS_INV':>7}")
    print(header)
    print("  " + "-" * 86)

    all_results = {}

    for wind_spd in WIND_SPEEDS:
        results = []
        for i in range(SEEDS_PER_WIND):
            seed = 1000 + i
            r = run_single(seed, wind_spd)
            results.append(r)

        dists = np.array([r.final_dist for r in results])
        gps_pcts = np.array([r.gps_inv_pct for r in results])

        n5  = np.sum(dists < 5)
        n10 = np.sum(dists < 10)
        n25 = np.sum(dists < 25)
        n50 = np.sum(dists < 50)
        nf  = np.sum(dists >= 150)
        mn  = np.mean(dists)
        md  = np.median(dists)
        p90 = np.percentile(dists, 90)
        best = np.min(dists)
        worst = np.max(dists)
        gps_avg = np.mean(gps_pcts)

        pct = lambda n: f"{100*n/len(dists):5.1f}%"

        print(f"  {wind_spd:5.1f}m  {pct(n5):>6}  {pct(n10):>6}  {pct(n25):>6}  "
              f"{pct(n50):>6}  {pct(nf):>6}  {mn:6.1f}m  {md:6.1f}m  "
              f"{p90:6.1f}m  {best:6.1f}m  {worst:6.1f}m  {gps_avg:5.1f}%")

        all_results[wind_spd] = (dists, results)

    # ── Summary ──
    print()
    print("=" * 90)
    print("  BOUNDARY ANALYSIS SUMMARY")
    print("=" * 90)

    # Find boundary: highest wind where P(< 10m) >= 50%
    for threshold, label in [(5, "5m"), (10, "10m"), (25, "25m"), (50, "50m")]:
        boundary_wind = None
        for wind_spd in WIND_SPEEDS:
            dists, _ = all_results[wind_spd]
            if np.sum(dists < threshold) / len(dists) >= 0.50:
                boundary_wind = wind_spd
        if boundary_wind is not None:
            print(f"  P(landing < {label:>3}) >= 50%  →  wind <= {boundary_wind:.1f} m/s")
        else:
            print(f"  P(landing < {label:>3}) >= 50%  →  NOT ACHIEVABLE in tested range")

    print()
    # Best 3 and worst 3 individual runs across all winds
    flat = []
    for w, (_, rs) in all_results.items():
        flat.extend(rs)
    flat.sort(key=lambda r: r.final_dist)

    print("  Top 5 landings:")
    for r in flat[:5]:
        print(f"    {r.final_dist:6.1f}m  wind={r.wind_spd:.0f}m/s dir={r.wind_dir:.0f}°  "
              f"tgt_bear={r.bearing:.0f}° dist={r.distance:.0f}m  "
              f"hdg0={r.init_heading:.0f}°  gps_inv={r.gps_inv_pct:.1f}%  seed={r.seed}")

    print("\n  Worst 5 landings:")
    for r in flat[-5:]:
        print(f"    {r.final_dist:6.1f}m  wind={r.wind_spd:.0f}m/s dir={r.wind_dir:.0f}°  "
              f"tgt_bear={r.bearing:.0f}° dist={r.distance:.0f}m  "
              f"hdg0={r.init_heading:.0f}°  gps_inv={r.gps_inv_pct:.1f}%  seed={r.seed}")

    print()
    print("=" * 90)
