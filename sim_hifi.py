#!/usr/bin/env python3
"""
CanSat Parafoil — Fault-Injection Integrated High-Fidelity Simulation
======================================================================
정밀 3DOF 물리 엔진 + 비행 중 고장 주입(Fault Injection) 통합 테스트
하드웨어 없이 로컬 PC에서 실행 가능.

Run:  python sim_integrated.py
"""

import io, sys
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

import math
import os
import types as _t
from collections import deque
from types import SimpleNamespace
import numpy as np

# ══════════════════════════════════════════════════════════════════════════════
# SECTION 1 — Hardware & Library Mocks
# ══════════════════════════════════════════════════════════════════════════════

class _MockPi:
    def __init__(self): self._p = {}
    def set_servo_pulsewidth(self, pin, pw): self._p[pin] = pw
    def stop(self): pass
    def get_pulse(self, pin): return self._p.get(pin, 0)

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

# ── FSW imports ──
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from Sensor_Motor import motor_guidance, motor_control

# ── Simulation clock patch ──
import time as _real_time
_sim_clock = [0.0]
class _SimTimeMod:
    @staticmethod
    def time() -> float: return _sim_clock[0]
    @staticmethod
    def sleep(s): pass
    def __getattr__(self, name): return getattr(_real_time, name)
motor_guidance.time = _SimTimeMod()

# ══════════════════════════════════════════════════════════════════════════════
# SECTION 2 — Physics & Constants
# ══════════════════════════════════════════════════════════════════════════════

REF_LAT, REF_LON = 35.0950, 127.0950
LAT2M, COS_LAT = 111320.0, math.cos(math.radians(REF_LAT))

def en_to_latlon(E_m: float, N_m: float):
    return REF_LAT + N_m / LAT2M, REF_LON + E_m / (LAT2M * COS_LAT)

VA_BASE, DESCENT_BASE = 8.0, 3.5
WIND_U_REF, WIND_Z_REF, WIND_ALPHA = 8.0, 600.0, 0.30
L_HOR, L_VER, SIG_HOR, SIG_VER = 150.0, 30.0, 4.0, 2.5
OMEGA_N, ZETA, K_COUPLE = math.sqrt(9.81 / 1.5), 0.08, 0.10
SLEW_RATE_DEG_S, DEADBAND_MECH = 200.0, 5.0
GPS_NOISE_M, GPS_DELAY, GPS_WARMUP = 3.5, 4, 12
YAW_NOISE_DEG, GYRZ_NOISE_RPS, BIAS_DRIFT_RATE, K_MAG_PEND = 4.0, math.radians(1.5), math.radians(0.06), 0.8
BARO_NOISE_M = 3.0
DT, MAX_STEPS = 0.1, 3000

rng = np.random.default_rng(99)
bearing_deg = float(rng.uniform(30, 70))
distance_m = float(rng.uniform(550, 700))
wind_dir_met = float(rng.uniform(250, 360))
target_E, target_N = distance_m * math.sin(math.radians(bearing_deg)), distance_m * math.cos(math.radians(bearing_deg))
target_lat, target_lon = en_to_latlon(target_E, target_N)
WIND_UNIT_E, WIND_UNIT_N = math.sin(math.radians(wind_dir_met + 180.0)), math.cos(math.radians(wind_dir_met + 180.0))

turb_u = turb_v = turb_w = servo_delta_actual = pend_phi = pend_phidot = imu_bias_gyrz = 0.0
_E_buf, _N_buf, _spd_buf, _crs_buf = deque(maxlen=GPS_DELAY), deque(maxlen=GPS_DELAY), deque(maxlen=GPS_DELAY), deque(maxlen=GPS_DELAY)

def wind_at_alt(alt_m: float):
    U = WIND_U_REF * (max(alt_m, 1.0) / WIND_Z_REF) ** WIND_ALPHA
    return U * WIND_UNIT_E, U * WIND_UNIT_N

def step_turbulence(Va: float, dt: float):
    global turb_u, turb_v, turb_w
    rho_h, rho_v = math.exp(-Va * dt / L_HOR), math.exp(-Va * dt / L_VER)
    sig_h, sig_v = SIG_HOR * math.sqrt(max(0.0, 1.0 - rho_h ** 2)), SIG_VER * math.sqrt(max(0.0, 1.0 - rho_v ** 2))
    turb_u, turb_v, turb_w = rho_h * turb_u + sig_h * rng.standard_normal(), rho_h * turb_v + sig_h * rng.standard_normal(), rho_v * turb_w + sig_v * rng.standard_normal()

def step_servo(delta_target: float, dt: float) -> float:
    global servo_delta_actual
    max_slew = SLEW_RATE_DEG_S * dt
    servo_delta_actual += max(-max_slew, min(max_slew, delta_target - servo_delta_actual))
    return math.copysign(max(0.0, abs(servo_delta_actual) - DEADBAND_MECH), servo_delta_actual)

def step_pendulum(yaw_rate_phy: float, dt: float):
    global pend_phi, pend_phidot
    ddphi_rad = -2.0 * ZETA * OMEGA_N * math.radians(pend_phidot) - OMEGA_N ** 2 * math.radians(pend_phi) + math.radians(K_COUPLE * yaw_rate_phy)
    pend_phidot += math.degrees(ddphi_rad) * dt
    pend_phi += pend_phidot * dt

def sensor_gps(E: float, N: float, V_E_gnd: float, V_N_gnd: float, step: int):
    _E_buf.append(E + rng.normal(0.0, GPS_NOISE_M))
    _N_buf.append(N + rng.normal(0.0, GPS_NOISE_M))
    _spd_buf.append(math.hypot(V_E_gnd, V_N_gnd))
    _crs_buf.append(math.degrees(math.atan2(V_E_gnd, V_N_gnd)) % 360.0)
    warm = step >= GPS_WARMUP and len(_E_buf) == GPS_DELAY
    E_out, N_out, spd_out, crs_out = (_E_buf[0], _N_buf[0], _spd_buf[0], _crs_buf[0]) if warm else (0.0, 0.0, 0.0, 0.0)
    lat_out, lon_out = en_to_latlon(E_out, N_out)
    return SimpleNamespace(lat=lat_out, lon=lon_out, speed=spd_out, course=crs_out), SimpleNamespace(fix_quality=1 if warm else 0, sats=8 if warm else 0, rmc_status="A" if warm else "V")

def sensor_imu(heading_true: float, yaw_rate_true: float, pend_phi_deg: float):
    global imu_bias_gyrz
    imu_bias_gyrz = max(-0.2, min(0.2, imu_bias_gyrz + rng.normal(0.0, BIAS_DRIFT_RATE * math.sqrt(DT))))
    return SimpleNamespace(yaw=heading_true + K_MAG_PEND * pend_phi_deg + rng.normal(0.0, YAW_NOISE_DEG), gyrz=math.radians(yaw_rate_true) + imu_bias_gyrz + rng.normal(0.0, GYRZ_NOISE_RPS))

def sensor_baro(alt_true: float) -> float:
    return alt_true + rng.normal(0.0, BARO_NOISE_M)

# ══════════════════════════════════════════════════════════════════════════════
# SECTION 3 — Initialization & Fault Injection Setup
# ══════════════════════════════════════════════════════════════════════════════

motor_guidance.init_guidance()
motor_guidance.set_start_coordinates(REF_LAT, REF_LON)
motor_guidance.set_target_coord(target_lat, target_lon)
motor_guidance.GPS_JUMP_MAX_SPEED = 200.0

mock_pi = _MockPi()
E, N, alt, heading, yaw_rate_phy, Va_curr, flight_state = 0.0, 0.0, 600.0, float(rng.uniform(0.0, 360.0)), 0.0, VA_BASE, 3

h_E, h_N, h_alt, h_t, h_cmdyr, h_phyyr, h_imuyz, h_phase, h_wE, h_wN, h_servo, h_state = ([] for _ in range(12))
gps_invalid_count, pattern_steps = 0, 0

fault_flags = {"gps_jump": False, "gyrz_spike": False, "baro_zero": False}

print("\n" + "=" * 88)
print("  🚀 CanSat Parafoil Integrated Simulation (w/ Fault Injection)")
print("=" * 88)

# ══════════════════════════════════════════════════════════════════════════════
# SECTION 4 — Main Simulation Loop
# ══════════════════════════════════════════════════════════════════════════════

## ── Real-time plot setup ──
try:
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    import numpy as np

    plt.ion()
    fig, axes = plt.subplots(1, 2, figsize=(16, 7))
    ax1, ax2 = axes

    # 2D 궤적
    ax1.plot(0, 0, '^', color='green', ms=12, label='Start (0,0)')
    ax1.plot(target_E, target_N, '*', color='red', ms=18, label='Target')
    theta = np.linspace(0, 2 * math.pi, 100)
    ax1.plot(target_E + 5 * np.cos(theta), target_N + 5 * np.sin(theta), 'r--', label='5m Zone')
    trail_line, = ax1.plot([], [], 'b-', linewidth=1.2, label='Track')
    pos_dot, = ax1.plot([], [], 'ko', ms=6)
    ax1.set_title("2D Ground Track"); ax1.set_xlabel("East (m)"); ax1.set_ylabel("North (m)")
    ax1.grid(True, alpha=0.3); ax1.legend(); ax1.axis('equal')

    # 제어 반응
    cmd_line, = ax2.plot([], [], 'b-', label='Cmd Yaw Rate', alpha=0.7)
    phy_line, = ax2.plot([], [], 'r--', label='Phy Yaw Rate')
    ax2.set_title("Control Response"); ax2.set_xlabel("Time (s)"); ax2.set_ylabel("Yaw Rate (deg/s)")
    ax2.grid(True, alpha=0.3); ax2.legend()

    PLOT_LIVE = True
    PLOT_INTERVAL = 5  # 5 step마다 갱신
except ImportError:
    PLOT_LIVE = False

PHASE_COLOUR = {'TURNING': 'royalblue', 'HOMING': 'royalblue', 'STRAIGHT': 'green',
                'PATTERN': 'darkorange', 'GPS_INVALID': 'red', 'TARGET_REACHED': 'gold', 'BARO_INVALID': 'red'}

for step in range(MAX_STEPS):
    t = step * DT
    _sim_clock[0] = t

    # 1. 환경 및 물리 업데이트
    wE, wN = wind_at_alt(alt)
    step_turbulence(Va_curr, DT)
    hdg_rad = math.radians(heading)
    Va_fwd = max(VA_BASE - 0.06 * abs(servo_delta_actual), 4.0)
    wE_total, wN_total = wE + turb_u, wN + turb_v
    V_E_gnd, V_N_gnd = Va_fwd * math.sin(hdg_rad) + wE_total, Va_fwd * math.cos(hdg_rad) + wN_total

    # 2. 센서 리딩
    gps_vec, gps_fid = sensor_gps(E, N, V_E_gnd, V_N_gnd, step)
    imu_data = sensor_imu(heading, yaw_rate_phy, pend_phi)
    baro_m = sensor_baro(alt)

    # 🚨 3. 고장 주입 (Fault Injection) 🚨
    # 고장 1: 고도 450m 부근에서 GPS 1km 튐 (3초 지속)
    if 450.0 >= alt > 430.0:
        if not fault_flags["gps_jump"]:
            print(f"\n[💥 FAULT] 고도 {alt:.1f}m: GPS 1km Multipath 점프 발생! (Guidance가 거부해야 함)")
            fault_flags["gps_jump"] = True
        gps_vec.lat += 0.01
        gps_vec.lon += 0.01

    # 고장 2: 고도 300m 부근에서 자이로센서 스파이크
    if 300.0 >= alt > 298.0:
        if not fault_flags["gyrz_spike"]:
            print(f"\n[💥 FAULT] 고도 {alt:.1f}m: IMU 자이로 스파이크 발생 (150 deg/s)!")
            fault_flags["gyrz_spike"] = True
        imu_data.gyrz = math.radians(150.0)

    # 고장 3: 고도 150m 부근에서 기압계 0.0 에러 (2초 지속)
    if 150.0 >= alt > 140.0:
        if not fault_flags["baro_zero"]:
            print(f"\n[💥 FAULT] 고도 {alt:.1f}m: Barometer 센서 0.0m 출력 발생!")
            fault_flags["baro_zero"] = True
        baro_m = 0.0

    # 4. 제어 로직 실행 (Motor Guidance & Control)
    patterned = (flight_state == 4) and (baro_m > 10.0)
    target_ns = SimpleNamespace(lat=target_lat, lon=target_lon)
    
    guidance_result = motor_guidance.guidance(imu_data, gps_vec, gps_fid, target_ns, baro_m=baro_m)
    cmd_yr = guidance_result.commanded_yaw_rate
    phase = guidance_result.state

    motor_result = motor_control.control(mock_pi, cmd_yr)
    effective_delta = step_servo(motor_result.actual_delta_deg, DT)

    # 5. 비행체 동역학 업데이트
    Va_fwd = max(VA_BASE - 0.03 * abs(effective_delta), 4.0)
    descent_rate = max(DESCENT_BASE + 0.001 * effective_delta ** 2 + turb_w * 0.3, 1.0)
    
    # effective_delta 음수 → 우선회(+) → 부호 반전
    # 회전 민감도: 모터 1° 차이 = 기체 1°/s 회전 (하드웨어 실측 기반)
    yaw_rate_phy = -effective_delta * 1.0 * (Va_fwd / VA_BASE)

    step_pendulum(yaw_rate_phy, DT)

    heading = (heading + yaw_rate_phy * DT) % 360.0
    hdg_rad = math.radians(heading)
    E += (Va_fwd * math.sin(hdg_rad) + wE_total) * DT
    N += (Va_fwd * math.cos(hdg_rad) + wN_total) * DT
    alt -= descent_rate * DT
    Va_curr = Va_fwd

    # 상태 전이
    if flight_state == 3 and alt < 50.0: flight_state = 4
    if alt <= 0.0: flight_state = 5

    # 6. 통계 및 로깅
    if phase == 'GPS_INVALID': gps_invalid_count += 1
    if phase == 'PATTERN': pattern_steps += 1

    h_E.append(E); h_N.append(N); h_alt.append(alt); h_t.append(t)
    h_cmdyr.append(cmd_yr); h_phyyr.append(yaw_rate_phy); h_imuyz.append(math.degrees(imu_data.gyrz))
    h_phase.append(phase); h_wE.append(wE); h_wN.append(wN); h_servo.append(servo_delta_actual); h_state.append(flight_state)

    if step % (5 if phase == 'PATTERN' else 20) == 0:
        print(f"  {step:>5}  {t:>6.1f}s  {alt:>7.1f}m  | Phase: {phase:<12} | Cmd_Yr: {cmd_yr:>6.1f}  Phy_Yr: {yaw_rate_phy:>6.1f} | Servo: {servo_delta_actual:>5.1f}")

    # ── Real-time plot update ──
    if PLOT_LIVE and step % PLOT_INTERVAL == 0:
        trail_line.set_data(h_E, h_N)
        pos_dot.set_data([E], [N])
        all_E = h_E + [target_E, 0.0]
        all_N = h_N + [target_N, 0.0]
        margin = 50
        ax1.set_xlim(min(all_E) - margin, max(all_E) + margin)
        ax1.set_ylim(min(all_N) - margin, max(all_N) + margin)

        cmd_line.set_data(h_t, h_cmdyr)
        phy_line.set_data(h_t, h_phyyr)
        ax2.set_xlim(0, t + 5)
        if h_cmdyr:
            yr_min = min(min(h_cmdyr), min(h_phyyr)) - 10
            yr_max = max(max(h_cmdyr), max(h_phyyr)) + 10
            ax2.set_ylim(yr_min, yr_max)

        fig.canvas.draw_idle()
        fig.canvas.flush_events()

    if flight_state == 5:
        motor_control.set_motors_off(mock_pi)
        print(f"\n  🛬 [LANDED] 비행 종료! (시간: {t:.1f}s, 최종 고도: {alt:.1f}m)")
        break

# ══════════════════════════════════════════════════════════════════════════════
# SECTION 5 — Plotting & Results
# ══════════════════════════════════════════════════════════════════════════════

if PLOT_LIVE:
    # 최종 궤적: phase별 색상으로 다시 그리기
    trail_line.set_visible(False)
    pos_dot.set_visible(False)
    for i in range(len(h_E) - 1):
        ax1.plot(h_E[i:i+2], h_N[i:i+2], color=PHASE_COLOUR.get(h_phase[i], 'gray'), linewidth=1.5)
    ax1.plot(E, N, 'X', color='black', ms=12, label='Landing')
    ax1.legend()

    # 제어 그래프에 IMU + Fault 영역 추가
    ax2.plot(h_t, h_imuyz, 'orange', label='IMU Gyro', alpha=0.5)
    ax2.fill_between(h_t, -100, 100, where=(np.array(h_alt) <= 450) & (np.array(h_alt) > 430), color='red', alpha=0.2, label='GPS Fault')
    ax2.fill_between(h_t, -100, 100, where=(np.array(h_alt) <= 150) & (np.array(h_alt) > 140), color='gray', alpha=0.2, label='Baro Fault')
    ax2.legend()

    plt.ioff()
    plt.tight_layout()
    plt.show()
else:
    print("\n[알림] matplotlib이 설치되어 있지 않아 그래프를 생략합니다.")

print("\n🚀 통합 시뮬레이션 및 검증 완료!")