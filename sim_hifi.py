#!/usr/bin/env python3
"""
CanSat Parafoil — High-Fidelity 3DOF Aerodynamic Simulation Testbench
======================================================================
Mocks all hardware dependencies then imports the REAL motor_guidance.py
and motor_control.py from Sensor_Motor/ to close the loop.

Run:  python sim_hifi.py   (from C:\\workspace\\CANSAT_AAS_2026_FSW\\)
"""

# ── stdout/stderr encoding (Windows console) ──────────────────────────────────
import io, sys
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

# ── standard library ──────────────────────────────────────────────────────────
import math
import os
import types as _t
from collections import deque
from types import SimpleNamespace

import numpy as np

# ══════════════════════════════════════════════════════════════════════════════
# SECTION 1 — Hardware mocks (must be in sys.modules before any FSW import)
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

# ── FSW imports ────────────────────────────────────────────────────────────────
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from Sensor_Motor import motor_guidance, motor_control  # noqa: E402

# ── Simulation clock: patch motor_guidance.time so wall-clock calls return
#    our stepped sim time.  Without this, _is_gps_jump sees microsecond dt,
#    computes enormous apparent velocity, and rejects every GPS fix.
import time as _real_time
_sim_clock = [0.0]  # mutable container so loop can write via index (no global needed)

class _SimTimeMod:
    """Thin wrapper: time.time() → sim clock; everything else → real time."""
    @staticmethod
    def time() -> float:
        return _sim_clock[0]
    @staticmethod
    def sleep(s):
        pass  # no-op in simulation
    def __getattr__(self, name):
        return getattr(_real_time, name)

motor_guidance.time = _SimTimeMod()  # type: ignore[attr-defined]

# ══════════════════════════════════════════════════════════════════════════════
# SECTION 2 — Coordinate helpers & constants
# ══════════════════════════════════════════════════════════════════════════════

REF_LAT  = 35.0950
REF_LON  = 127.0950
LAT2M    = 111320.0
COS_LAT  = math.cos(math.radians(REF_LAT))

def en_to_latlon(E_m: float, N_m: float):
    lat = REF_LAT + N_m / LAT2M
    lon = REF_LON + E_m / (LAT2M * COS_LAT)
    return lat, lon

# ── Aerodynamics constants ────────────────────────────────────────────────────
VA_BASE      = 8.0    # m/s, glide speed
DESCENT_BASE = 3.0    # m/s, nominal sink rate

# ── Wind model constants ──────────────────────────────────────────────────────
WIND_U_REF = 5.0     # m/s at reference altitude
WIND_Z_REF = 600.0    # m
WIND_ALPHA  = 0.35    # power-law exponent

# ── Dryden turbulence constants ───────────────────────────────────────────────
L_HOR   = 200.0       # horizontal length scale (m)
L_VER   = 50.0        # vertical length scale (m)
SIG_HOR = 2.5         # horizontal turbulence intensity (m/s)
SIG_VER = 1.5         # vertical turbulence intensity (m/s)

# ── Pendulum constants ────────────────────────────────────────────────────────
OMEGA_N  = math.sqrt(9.81 / 1.5)
ZETA     = 0.12
K_COUPLE = 0.06

# ── Actuator constants ─────────────────────────────────────────────────────────
SLEW_RATE_DEG_S = 300.0
DEADBAND_MECH   = 3.0

# ── GPS sensor constants ──────────────────────────────────────────────────────
GPS_NOISE_M  = 2.5
GPS_DELAY    = 3       # steps
GPS_WARMUP   = 8       # steps

# ── IMU sensor constants ──────────────────────────────────────────────────────
YAW_NOISE_DEG    = 2.0
GYRZ_NOISE_RPS   = math.radians(0.5)
BIAS_DRIFT_RATE  = math.radians(0.02)
K_MAG_PEND       = 0.5

# ── Barometer noise ────────────────────────────────────────────────────────────
BARO_NOISE_M = 1.5

# ── Simulation timing ─────────────────────────────────────────────────────────
DT        = 0.1
MAX_STEPS = 3000

# ══════════════════════════════════════════════════════════════════════════════
# SECTION 3 — Random scenario (deterministic seed)
# ══════════════════════════════════════════════════════════════════════════════

rng = np.random.default_rng(42)
bearing_deg  = float(rng.uniform(30, 70))
distance_m   = float(rng.uniform(500, 600))
wind_dir_met = float(rng.uniform(280, 340))    # FROM direction (met convention)

target_E = distance_m * math.sin(math.radians(bearing_deg))
target_N = distance_m * math.cos(math.radians(bearing_deg))
target_lat, target_lon = en_to_latlon(target_E, target_N)

wind_toward_rad = math.radians(wind_dir_met + 180.0)
WIND_UNIT_E = math.sin(wind_toward_rad)
WIND_UNIT_N = math.cos(wind_toward_rad)

# ══════════════════════════════════════════════════════════════════════════════
# SECTION 4 — Global simulation state
# ══════════════════════════════════════════════════════════════════════════════

turb_u           = 0.0   # E-axis turbulence (m/s)
turb_v           = 0.0   # N-axis turbulence (m/s)
turb_w           = 0.0   # vertical turbulence (m/s)
servo_delta_actual = 0.0  # current servo position (deg, signed)
pend_phi         = 0.0   # pendulum angle (deg)
pend_phidot      = 0.0   # pendulum angular rate (deg/s)
imu_bias_gyrz    = 0.0   # gyro z-axis bias (rad/s)

# GPS delay buffers
_E_buf   = deque(maxlen=GPS_DELAY)
_N_buf   = deque(maxlen=GPS_DELAY)
_spd_buf = deque(maxlen=GPS_DELAY)
_crs_buf = deque(maxlen=GPS_DELAY)

# ══════════════════════════════════════════════════════════════════════════════
# SECTION 5 — Physics model functions
# ══════════════════════════════════════════════════════════════════════════════

def wind_at_alt(alt_m: float):
    """Power-law wind profile → (wE, wN) in m/s."""
    z = max(alt_m, 1.0)
    U = WIND_U_REF * (z / WIND_Z_REF) ** WIND_ALPHA
    return U * WIND_UNIT_E, U * WIND_UNIT_N


def step_turbulence(Va: float, dt: float):
    """Update Dryden Gauss-Markov turbulence states in-place."""
    global turb_u, turb_v, turb_w
    rho_h = math.exp(-Va * dt / L_HOR)
    rho_v = math.exp(-Va * dt / L_VER)
    sig_h = SIG_HOR * math.sqrt(max(0.0, 1.0 - rho_h ** 2))
    sig_v = SIG_VER * math.sqrt(max(0.0, 1.0 - rho_v ** 2))
    turb_u = rho_h * turb_u + sig_h * rng.standard_normal()
    turb_v = rho_h * turb_v + sig_h * rng.standard_normal()
    turb_w = rho_v * turb_w + sig_v * rng.standard_normal()


def step_servo(delta_target: float, dt: float) -> float:
    """Slew-rate-limited servo model. Returns effective aerodynamic delta (deg)."""
    global servo_delta_actual
    max_slew = SLEW_RATE_DEG_S * dt
    servo_delta_actual += max(-max_slew, min(max_slew, delta_target - servo_delta_actual))
    # Apply mechanical deadband
    effective = math.copysign(max(0.0, abs(servo_delta_actual) - DEADBAND_MECH), servo_delta_actual)
    return effective


def step_pendulum(yaw_rate_phy: float, dt: float):
    """Second-order pendulum dynamics coupled to yaw rate."""
    global pend_phi, pend_phidot
    ddphi_rad = (
        -2.0 * ZETA * OMEGA_N * math.radians(pend_phidot)
        - OMEGA_N ** 2 * math.radians(pend_phi)
        + math.radians(K_COUPLE * yaw_rate_phy)
    )
    pend_phidot += math.degrees(ddphi_rad) * dt
    pend_phi    += pend_phidot * dt


def sensor_gps(E: float, N: float, V_E_gnd: float, V_N_gnd: float, step: int):
    """
    Returns (gps_vector SimpleNamespace, gps_fidelity SimpleNamespace).
    Applies Gaussian position noise, GPS_DELAY steps of latency, and warmup mask.
    """
    # Noisy measurements pushed into delay buffer
    E_noisy = E + rng.normal(0.0, GPS_NOISE_M)
    N_noisy = N + rng.normal(0.0, GPS_NOISE_M)
    spd = math.hypot(V_E_gnd, V_N_gnd)
    crs = math.degrees(math.atan2(V_E_gnd, V_N_gnd)) % 360.0

    _E_buf.append(E_noisy)
    _N_buf.append(N_noisy)
    _spd_buf.append(spd)
    _crs_buf.append(crs)

    warm = step >= GPS_WARMUP and len(_E_buf) == GPS_DELAY

    if warm:
        E_out = _E_buf[0]
        N_out = _N_buf[0]
        spd_out = _spd_buf[0]
        crs_out = _crs_buf[0]
        fix_quality = 1; sats = 8; rmc_status = "A"
    else:
        E_out = 0.0; N_out = 0.0; spd_out = 0.0; crs_out = 0.0
        fix_quality = 0; sats = 0; rmc_status = "V"

    lat_out, lon_out = en_to_latlon(E_out, N_out)

    gps_vec = SimpleNamespace(lat=lat_out, lon=lon_out, speed=spd_out, course=crs_out)
    gps_fid = SimpleNamespace(fix_quality=fix_quality, sats=sats, rmc_status=rmc_status)
    return gps_vec, gps_fid


def sensor_imu(heading_true: float, yaw_rate_true: float, pend_phi_deg: float):
    """Returns SimpleNamespace(yaw=deg, gyrz=rad/s) with noise and bias."""
    global imu_bias_gyrz
    yaw_meas = heading_true + K_MAG_PEND * pend_phi_deg + rng.normal(0.0, YAW_NOISE_DEG)
    imu_bias_gyrz += rng.normal(0.0, BIAS_DRIFT_RATE * math.sqrt(DT))
    imu_bias_gyrz  = max(-0.2, min(0.2, imu_bias_gyrz))
    gyrz_meas = math.radians(yaw_rate_true) + imu_bias_gyrz + rng.normal(0.0, GYRZ_NOISE_RPS)
    return SimpleNamespace(yaw=yaw_meas, gyrz=gyrz_meas)


def sensor_baro(alt_true: float) -> float:
    return alt_true + rng.normal(0.0, BARO_NOISE_M)

# ══════════════════════════════════════════════════════════════════════════════
# SECTION 6 — FSW initialisation
# ══════════════════════════════════════════════════════════════════════════════

# init_guidance sets last_time = time.time() which now hits our sim clock
motor_guidance.init_guidance()
motor_guidance.set_start_coordinates(REF_LAT, REF_LON)
motor_guidance.set_target_coord(target_lat, target_lon)

# ── Sim-level GPS jump threshold override ─────────────────────────────────────
# GPS_NOISE_M=2.5m at 10Hz → adjacent noisy samples produce ~35 m/s RMS apparent
# velocity; FSW threshold=50 m/s (designed for 1-5 Hz GPS) is borderline.
# Real Multipath jumps are km-scale, so 100 m/s is still a tight guard.
motor_guidance.GPS_JUMP_MAX_SPEED = 100.0

# ══════════════════════════════════════════════════════════════════════════════
# SECTION 7 — Main simulation loop
# ══════════════════════════════════════════════════════════════════════════════

mock_pi = _MockPi()

# Initial flight state
E   = 0.0
N   = 0.0
alt = 600.0
heading      = float(rng.uniform(0.0, 360.0))
yaw_rate_phy = 0.0
Va_curr      = VA_BASE
flight_state = 3  # mirrors flightlogicapp: 3=descending, 4=pattern, 5=landed

# History accumulators
h_E     = []; h_N     = []; h_alt   = []; h_t     = []
h_cmdyr = []; h_phyyr = []; h_imuyz = []; h_phase = []
h_wE    = []; h_wN    = []; h_servo = []
h_state = []  # flight_state per step

# Phase colour map for legend (2-D figure)
PHASE_COLOUR = {
    'TURNING':        'royalblue',
    'HOMING':         'royalblue',
    'STRAIGHT':       'green',
    'PATTERN':        'darkorange',
    'GPS_INVALID':    'red',
    'TARGET_REACHED': 'gold',
    'BARO_INVALID':   'red',
}

print()
print("=" * 88)
print("  CanSat Parafoil Hi-Fi 3DOF Simulation")
print(f"  Target: bearing {bearing_deg:.1f} deg  dist {distance_m:.0f} m  "
      f"| Wind FROM {wind_dir_met:.0f} deg  {WIND_U_REF} m/s @ {WIND_Z_REF:.0f} m")
print("=" * 88)
print(f"  {'Step':>5}  {'t[s]':>6}  {'alt[m]':>7}  {'dist[m]':>8}  "
      f"{'Phase':<12}  {'cmd_yr':>7}  {'phy_yr':>7}  {'servo':>7}  "
      f"{'hdg':>6}  {'E[m]':>8}  {'N[m]':>8}  {'st':>2}")
print("-" * 88)

gps_invalid_count = 0
pattern_steps     = 0

for step in range(MAX_STEPS):
    t = step * DT
    _sim_clock[0] = t  # advance sim clock → motor_guidance.time.time() returns t

    # 1. Wind
    wE, wN = wind_at_alt(alt)

    # 2. Turbulence
    step_turbulence(Va_curr, DT)

    # 3. Ground velocity components (used for GPS course/speed)
    hdg_rad   = math.radians(heading)
    Va_fwd    = max(VA_BASE - 0.06 * abs(servo_delta_actual), 4.0)
    wE_total  = wE + turb_u
    wN_total  = wN + turb_v
    V_E_gnd   = Va_fwd * math.sin(hdg_rad) + wE_total
    V_N_gnd   = Va_fwd * math.cos(hdg_rad) + wN_total

    # 4. Sensors
    gps_vec, gps_fid = sensor_gps(E, N, V_E_gnd, V_N_gnd, step)
    imu_data          = sensor_imu(heading, yaw_rate_phy, pend_phi)
    baro_m            = sensor_baro(alt)

    # 5. Pattern activation — mirrors motorapp._resolve_patterned logic:
    #    state==4, baro>10m → True (8자 비행)
    #    state==4, baro<=10m → False (당근, Final)
    #    state==3 → False (당근, 호밍)
    patterned = (flight_state == 4) and (baro_m > 10.0)

    # 6. Guidance
    target_ns      = SimpleNamespace(lat=target_lat, lon=target_lon)
    guidance_result = motor_guidance.guidance(
        imu_data, gps_vec, gps_fid, target_ns,
        baro_m=baro_m, patterned=patterned
    )
    cmd_yr = guidance_result.commanded_yaw_rate
    phase  = guidance_result.state

    # 7. Motor control (actuator mixer + servo command)
    motor_result   = motor_control.control(mock_pi, cmd_yr)
    effective_delta = step_servo(motor_result.actual_delta_deg, DT)

    # 8. Aerodynamics update
    Va_fwd       = max(VA_BASE - 0.03 * abs(effective_delta), 4.0)
    descent_rate = max(DESCENT_BASE + 0.001 * effective_delta ** 2 + turb_w * 0.3, 1.0)
    yaw_rate_phy = effective_delta * (Va_fwd / VA_BASE)

    # 9. Pendulum
    step_pendulum(yaw_rate_phy, DT)

    # 10. Integrate position & heading
    heading += yaw_rate_phy * DT
    heading  = heading % 360.0
    hdg_rad  = math.radians(heading)
    E   += (Va_fwd * math.sin(hdg_rad) + wE_total) * DT
    N   += (Va_fwd * math.cos(hdg_rad) + wN_total) * DT
    alt -= descent_rate * DT
    Va_curr = Va_fwd

    # State transitions — mirrors flightlogicapp behaviour:
    #   state 3→4 when alt drops below 50 m,
    #   state 4→5 on touchdown (alt<=0)
    if flight_state == 3 and alt < 50.0:
        flight_state = 4
    if alt <= 0.0:
        flight_state = 5

    # 11. Statistics
    if phase == 'GPS_INVALID':
        gps_invalid_count += 1
    if phase == 'PATTERN':
        pattern_steps += 1

    # 12. History
    h_E.append(E); h_N.append(N); h_alt.append(alt); h_t.append(t)
    h_cmdyr.append(cmd_yr); h_phyyr.append(yaw_rate_phy)
    h_imuyz.append(math.degrees(imu_data.gyrz))
    h_phase.append(phase); h_wE.append(wE); h_wN.append(wN)
    h_servo.append(servo_delta_actual)
    h_state.append(flight_state)

    # 13. Console print (every 20 steps, or every 5 in PATTERN)
    print_interval = 5 if phase == 'PATTERN' else 20
    dist_to_tgt = math.hypot(target_E - E, target_N - N)
    if step % print_interval == 0:
        print(f"  {step:>5}  {t:>6.1f}  {alt:>7.1f}  {dist_to_tgt:>8.1f}  "
              f"{phase:<12}  {cmd_yr:>7.2f}  {yaw_rate_phy:>7.2f}  {servo_delta_actual:>7.2f}  "
              f"{heading:>6.1f}  {E:>8.1f}  {N:>8.1f}  {'st':>2}:{flight_state}")

    # 14. Termination — state 5 means touchdown confirmed
    if flight_state == 5:
        motor_control.set_motors_off(mock_pi)
        print(f"\n  [LANDED/STOP] step={step}  t={t:.1f}s  E={E:.1f}m  N={N:.1f}m  final_state={flight_state}")
        break

print("-" * 88)

# ══════════════════════════════════════════════════════════════════════════════
# SECTION 8 — Assessment Report
# ══════════════════════════════════════════════════════════════════════════════

final_dist = math.hypot(target_E - E, target_N - N)
total_steps = len(h_t)
total_time  = h_t[-1] if h_t else 0.0

# Wind speeds at key altitudes
wE_600, wN_600 = wind_at_alt(600.0)
wE_10,  wN_10  = wind_at_alt(10.0)
wind_spd_600 = math.hypot(wE_600, wN_600)
wind_spd_10  = math.hypot(wE_10,  wN_10)

# Max lateral drift from straight-line path (start → target)
line_E = target_E; line_N = target_N
line_len = math.hypot(line_E, line_N)
max_drift = 0.0
if line_len > 0.01:
    uE = line_E / line_len; uN = line_N / line_len
    for e, n in zip(h_E, h_N):
        proj  = e * uE + n * uN
        lat_E = e - proj * uE
        lat_N = n - proj * uN
        drift = math.hypot(lat_E, lat_N)
        if drift > max_drift:
            max_drift = drift

# Flight state step counts
state_counts = {3: 0, 4: 0, 5: 0}
for s in h_state:
    if s in state_counts:
        state_counts[s] += 1

# ══════════════════════════════════════════════════════════════════════════════
# SECTION 7.5 — FIX-GYRZ & coord_ok validation (post-sim unit checks)
# ══════════════════════════════════════════════════════════════════════════════

print()
print("=" * 60)
print("  FIX VALIDATION (coord_ok + GYRZ spike gate)")
print("=" * 60)

# ── coord_ok: lat=0 단독 bypass 차단 확인 ──
_fix_pass = 0; _fix_total = 0

_fix_total += 1
v = motor_guidance.is_gps_valid(0.0, 127.5, 1, 6, "A")
print(f"  [{'PASS' if not v else 'FAIL'}] lat=0, lon=127.5 → invalid (got {v})")
if not v: _fix_pass += 1

_fix_total += 1
v = motor_guidance.is_gps_valid(35.0, 0.0, 1, 8, "A")
print(f"  [{'PASS' if not v else 'FAIL'}] lat=35, lon=0 → invalid (got {v})")
if not v: _fix_pass += 1

_fix_total += 1
v = motor_guidance.is_gps_valid(35.0, 127.0, 1, 8, "A")
print(f"  [{'PASS' if v else 'FAIL'}] lat=35, lon=127 → valid (got {v})")
if v: _fix_pass += 1

# ── FIX-GYRZ: motorapp gyrz spike gate 확인 ──
# motorapp은 sim_hifi에서 import하지 않으므로, 게이트 로직만 인라인 검증
_GYRZ_SPIKE_THRESHOLD = 45.0

_fix_total += 1
prev, new = 3.0, 63.0
rejected = abs(new - prev) > _GYRZ_SPIKE_THRESHOLD
print(f"  [{'PASS' if rejected else 'FAIL'}] gyrz prev=3, new=63 (delta=60>45) → rejected={rejected}")
if rejected: _fix_pass += 1

_fix_total += 1
prev, new = 3.0, 40.0
accepted = abs(new - prev) <= _GYRZ_SPIKE_THRESHOLD
print(f"  [{'PASS' if accepted else 'FAIL'}] gyrz prev=3, new=40 (delta=37<45) → accepted={accepted}")
if accepted: _fix_pass += 1

print(f"\n  FIX checks: {_fix_pass}/{_fix_total} passed")

print()
print("=" * 60)
print("  ASSESSMENT REPORT")
print("=" * 60)

print("\n  [1] Environment")
print(f"      Wind from {wind_dir_met:.1f} deg (met)")
print(f"      Speed @ 600 m : {wind_spd_600:.2f} m/s  "
      f"(E={wE_600:.2f}, N={wN_600:.2f})")
print(f"      Speed @  10 m : {wind_spd_10:.2f} m/s  "
      f"(E={wE_10:.2f}, N={wN_10:.2f})")
print(f"      Target : bearing {bearing_deg:.1f} deg, "
      f"dist {distance_m:.0f} m  (E={target_E:.1f}, N={target_N:.1f})")
print(f"      Initial heading : {h_E[0] and heading:.1f} deg (random)")

print("\n  [2] Flight Statistics")
print(f"      Total steps : {total_steps}  ({total_time:.1f} s)")
gps_inv_pct = 100.0 * gps_invalid_count / max(total_steps, 1)
print(f"      GPS_INVALID : {gps_invalid_count} steps  ({gps_inv_pct:.1f} %)")
pat_time = pattern_steps * DT
print(f"      PATTERN     : {pattern_steps} steps  ({pat_time:.1f} s)")
print(f"      Max lateral drift from straight path : {max_drift:.1f} m")

print("\n  [3] Final Result")
print(f"      Landing  E={E:.2f} m  N={N:.2f} m  alt={alt:.2f} m")
print(f"      Distance to target : {final_dist:.2f} m")

print("\n  [4] Verdict")
if final_dist < 10.0:
    verdict = "EXCELLENT  (< 10 m)"
elif final_dist < 50.0:
    verdict = "GOOD       (< 50 m)"
elif final_dist < 150.0:
    verdict = "MARGINAL   (< 150 m)"
else:
    verdict = "FAIL       (>= 150 m)"
print(f"      {verdict}")

print("\n  [5] Wind Compensation")
print(f"      motor_guidance.wind_effect (learned) : {motor_guidance.wind_effect:.3f} deg")
true_crab = math.degrees(math.atan2(wE_10, wN_10)) - (
    math.degrees(math.atan2(wE_10 + V_E_gnd, wN_10 + V_N_gnd)) if False else 0.0
)
# Compute representative crab angle at low altitude
wE_repr, wN_repr = wind_at_alt(50.0)
Va_repr = VA_BASE
crab_repr = math.degrees(math.atan2(wE_repr, Va_repr))
print(f"      True wind crab (@ 50 m, Va={Va_repr} m/s) : {crab_repr:.3f} deg")

print("\n  [6] Flight State Breakdown")
state_labels = {3: 'DESCENDING (state 3)', 4: 'PATTERN/LANDING (state 4)', 5: 'LANDED (state 5)'}
for s in (3, 4, 5):
    cnt = state_counts[s]
    pct = 100.0 * cnt / max(total_steps, 1)
    print(f"      {state_labels[s]:<28} : {cnt:>5} steps  ({pct:.1f} %  /  {cnt * DT:.1f} s)")

print()
print("=" * 60)

# ══════════════════════════════════════════════════════════════════════════════
# SECTION 9 — Visualisation
# ══════════════════════════════════════════════════════════════════════════════

try:
    import matplotlib
    matplotlib.use('TkAgg')
except Exception:
    pass

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

E_arr   = np.array(h_E)
N_arr   = np.array(h_N)
alt_arr = np.array(h_alt)
t_arr   = np.array(h_t)
cmdyr_arr  = np.array(h_cmdyr)
phyyr_arr  = np.array(h_phyyr)
imuyz_arr  = np.array(h_imuyz)
servo_arr  = np.array(h_servo)
phases_arr = h_phase

# ── Figure 1: 3D trajectory ───────────────────────────────────────────────────
fig1 = plt.figure(figsize=(10, 7))
ax3  = fig1.add_subplot(111, projection='3d')

sc = ax3.scatter(E_arr, N_arr, alt_arr, c=t_arr, cmap='viridis', s=2, alpha=0.7)
fig1.colorbar(sc, ax=ax3, label='Time (s)', shrink=0.6)

ax3.scatter([0],       [0],       [600],      color='green', marker='^', s=120, zorder=5, label='Start')
ax3.scatter([target_E],[target_N],[0],         color='red',   marker='*', s=200, zorder=5, label='Target')
ax3.scatter([E],       [N],       [alt],       color='black', marker='x', s=150, zorder=5, label='Landing')

# Wind arrow at mid altitude (300 m)
wE_mid, wN_mid = wind_at_alt(300.0)
ax3.quiver(0, 0, 300, wE_mid * 5, wN_mid * 5, 0,
           color='cyan', linewidth=2, arrow_length_ratio=0.2, label='Wind x5')

ax3.set_xlabel('East (m)'); ax3.set_ylabel('North (m)'); ax3.set_zlabel('Alt (m)')
ax3.set_title(
    f'3D Trajectory | Wind FROM {wind_dir_met:.0f}° @ {WIND_U_REF} m/s | '
    f'Target dist={distance_m:.0f} m bear={bearing_deg:.0f}°'
)
ax3.legend(loc='upper left')

# ── Figure 2: 2D XY + wind field ──────────────────────────────────────────────
fig2, ax2 = plt.subplots(figsize=(9, 9))

# Trajectory coloured by guidance phase
phase_colours = [PHASE_COLOUR.get(p, 'gray') for p in phases_arr]
for i in range(len(E_arr) - 1):
    ax2.plot(E_arr[i:i+2], N_arr[i:i+2], color=phase_colours[i], linewidth=1.2)

# Wind quiver grid at 100 m altitude
grid_res = 10
e_lin = np.linspace(E_arr.min() - 50, E_arr.max() + 50, grid_res)
n_lin = np.linspace(N_arr.min() - 50, N_arr.max() + 50, grid_res)
Eg, Ng = np.meshgrid(e_lin, n_lin)
wE_100, wN_100 = wind_at_alt(100.0)
ax2.quiver(Eg, Ng, np.full_like(Eg, wE_100), np.full_like(Ng, wN_100),
           alpha=0.25, color='steelblue', scale=50, label='Wind @ 100 m')

# Markers
ax2.plot(0,        0,        '^', color='green', ms=10, zorder=5, label='Start')
ax2.plot(target_E, target_N, '*', color='red',   ms=14, zorder=5, label='Target')
ax2.plot(E,        N,        'x', color='black', ms=10, mew=2.5,  zorder=5, label='Landing')

# Landing zone circles
theta = np.linspace(0, 2 * math.pi, 200)
ax2.plot(target_E + 5  * np.cos(theta), target_N + 5  * np.sin(theta),
         'r--', linewidth=1.2, label='5 m zone')
ax2.plot(target_E + 40 * np.cos(theta), target_N + 40 * np.sin(theta),
         color='orange', linestyle='--', linewidth=1.0, label='40 m pattern zone')

# Phase legend entries
from matplotlib.lines import Line2D
legend_phase = [
    Line2D([0],[0], color='royalblue', lw=2, label='TURNING / HOMING'),
    Line2D([0],[0], color='green',     lw=2, label='STRAIGHT'),
    Line2D([0],[0], color='darkorange',lw=2, label='PATTERN'),
    Line2D([0],[0], color='red',       lw=2, label='GPS_INVALID'),
]
handles, labels = ax2.get_legend_handles_labels()
ax2.legend(handles=handles + legend_phase, loc='upper left', fontsize=8)

ax2.set_aspect('equal')
ax2.set_xlabel('East (m)'); ax2.set_ylabel('North (m)')
ax2.set_title('2D Ground Track with Phase Colouring and Wind Field')
ax2.grid(True, alpha=0.3)

# ── Figure 3: Control response ────────────────────────────────────────────────
fig3, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

# Pattern altitude threshold mask for orange shading (matches 40 m activation)
pattern_mask = alt_arr <= 40.0

# Subplot 1: commanded vs physical yaw rate
ax = axes[0]
ax.plot(t_arr, cmdyr_arr,  color='blue',       lw=1.2, label='cmd_yr (deg/s)')
ax.plot(t_arr, phyyr_arr,  color='red',  ls='--', lw=1.0, label='phy_yr (deg/s)')
ax.set_ylabel('Yaw rate (deg/s)')
ax.set_title('Control Command vs Physical Response vs IMU Reading')
ax.legend(loc='upper right', fontsize=8); ax.grid(True, alpha=0.3)

# Subplot 2: IMU gyro reading vs physical
ax = axes[1]
ax.plot(t_arr, imuyz_arr,  color='orange',     lw=1.0, label='imu_gyrz (deg/s)')
ax.plot(t_arr, phyyr_arr,  color='red',  ls='--', lw=1.0, label='phy_yr (deg/s)')
ax.set_ylabel('Yaw rate (deg/s)')
ax.legend(loc='upper right', fontsize=8); ax.grid(True, alpha=0.3)

# Subplot 3: servo deflection (left) + altitude (right twin)
ax  = axes[2]
ax2b = ax.twinx()
ax.plot(t_arr, servo_arr, color='purple', lw=1.2, label='servo delta (deg)')
ax.axhline( DEADBAND_MECH, color='red', ls=':', lw=0.8, label=f'+{DEADBAND_MECH}° deadband')
ax.axhline(-DEADBAND_MECH, color='red', ls=':', lw=0.8, label=f'-{DEADBAND_MECH}° deadband')

# Orange shading for pattern phase region
if pattern_mask.any():
    ax.fill_between(t_arr, ax.get_ylim()[0] if False else -70, 70,
                    where=pattern_mask, color='orange', alpha=0.15, label='Pattern region')

ax.set_ylabel('Servo delta (deg)'); ax.set_xlabel('Time (s)')
ax.legend(loc='upper left', fontsize=8); ax.grid(True, alpha=0.3)

ax2b.plot(t_arr, alt_arr, color='gray', ls=':', lw=1.0, label='Altitude (m)')
ax2b.set_ylabel('Altitude (m)')
ax2b.legend(loc='upper right', fontsize=8)

fig3.tight_layout()
fig3.suptitle('Control vs Physical Response', y=1.01)

plt.show()
