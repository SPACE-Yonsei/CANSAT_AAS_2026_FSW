#!/usr/bin/env python3
"""
sim_hifi.py
===========

High-fidelity local parafoil simulation for the uploaded CanSat motor/guidance stack.

What is integrated
------------------
1) sim_verify.py style sensor/hardware mocks
2) Real motor_guidance.py + motor_control.py in-the-loop
3) Physics upgrades discussed in review:
   - per-servo asymmetry (gain / deadband / slew / delay)
   - nonlinear brake-line -> yaw response
   - turn-induced sink increase and forward-speed decrease
   - simple bank + pendulum coupling
   - wind profile + Dryden-like turbulence
   - GPS/IMU/Baro delay + noise + bias drift
   - landing / pattern phases preserved from motor_guidance.py
4) Lightweight connection audit for motor sign conventions

Run:
    python sim_hifi.py

Optional examples:
    python sim_hifi.py --runs 40 --target-bearing 60 --target-distance 650 --plot
    python sim_hifi.py --single --wind-speed 6 --wind-dir 250 --init-heading 180
"""

import io
import os
import sys
import math
import time as _real_time
import types as _t
import argparse
import importlib.util
from dataclasses import dataclass, field
from pathlib import Path
from datetime import datetime
from types import SimpleNamespace
from collections import deque

try:
    import numpy as np
except ImportError as e:
    raise SystemExit("numpy is required to run sim_hifi.py") from e

# ------------------------------------------------------------------------------
# UTF-8 console
# ------------------------------------------------------------------------------
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8", errors="replace")

# ------------------------------------------------------------------------------
# Section 1 — hardware mocks
# ------------------------------------------------------------------------------
class _MockPi:
    def __init__(self):
        self._p = {}

    def set_servo_pulsewidth(self, pin, pw):
        self._p[pin] = pw

    def stop(self):
        pass

    def get_pulse(self, pin):
        return self._p.get(pin, 0)


sys.modules["pigpio"] = _t.ModuleType("pigpio")
sys.modules["pigpio"].pi = _MockPi


class _GPIO:
    BCM = 11
    OUT = 0
    HIGH = 1
    LOW = 0

    @staticmethod
    def setmode(m):
        return None

    @staticmethod
    def setup(pin, mode, initial=0):
        return None

    @staticmethod
    def output(pin, val):
        return None

    @staticmethod
    def cleanup(pin=None):
        return None


_rpi = _t.ModuleType("RPi")
_rpi.GPIO = _GPIO
sys.modules["RPi"] = _rpi
_rpigpio = _t.ModuleType("RPi.GPIO")
for _a in ("BCM", "OUT", "HIGH", "LOW", "setmode", "setup", "output", "cleanup"):
    setattr(_rpigpio, _a, getattr(_GPIO, _a))
sys.modules["RPi.GPIO"] = _rpigpio

# ------------------------------------------------------------------------------
# Section 2 — minimal lib mocks required by motor modules
# ------------------------------------------------------------------------------
_lib = _t.ModuleType("lib")
sys.modules["lib"] = _lib

_appargs = _t.ModuleType("lib.appargs")
class _NS:
    def __init__(self, **kw):
        self.__dict__.update(kw)

_appargs.MainAppArg        = _NS(AppID=10, AppName="Main", MID_TerminateProcess=1001001)
_appargs.MotorAppArg       = _NS(AppID=19, AppName="Motor")
_appargs.FlightlogicAppArg = _NS(
    MID_motor_TargetCor=1101901, MID_motor_state=1101902,
    MID_motor_burnwire=1101903, MID_motor_EggDrop=1101904,
    MID_motor_PullArms=1101905)
_appargs.GpsAppArg         = _NS(MID_motor_gps=1501901)
_appargs.ImuAppArg         = _NS(MID_motor_imu=1401901)
_appargs.BarometerAppArg   = _NS(MID_motor_alt=1301901)
_appargs.CommAppArg        = _NS(MID_RouteCmd_MEC=1607)
sys.modules["lib.appargs"] = _appargs
_lib.appargs = _appargs

_events = _t.ModuleType("lib.events")
_events.EventType = _NS(error=0, info=1, debug=2, warning=3)
_events.LogEvent = lambda *a, **k: None
sys.modules["lib.events"] = _events
_lib.events = _events

_msgmod = _t.ModuleType("lib.msgstructure")
sys.modules["lib.msgstructure"] = _msgmod
_lib.msgstructure = _msgmod

# ------------------------------------------------------------------------------
# Section 3 — import local motor_guidance.py + motor_control.py into fake package
# ------------------------------------------------------------------------------
HERE = Path(__file__).resolve().parent

def _load_local_module(mod_name: str, filename: str):
    spec = importlib.util.spec_from_file_location(mod_name, HERE / filename)
    if spec is None or spec.loader is None:
        raise ImportError(f"Failed to load {filename}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[mod_name] = mod
    spec.loader.exec_module(mod)
    return mod

_sensor_motor = _t.ModuleType("Sensor_Motor")
sys.modules["Sensor_Motor"] = _sensor_motor

motor_control = _load_local_module("Sensor_Motor.motor_control", "motor_control.py")
motor_guidance = _load_local_module("Sensor_Motor.motor_guidance", "motor_guidance.py")
_sensor_motor.motor_control = motor_control
_sensor_motor.motor_guidance = motor_guidance

# Silence module logging for Monte Carlo speed
motor_guidance.DEBUG_GUIDANCE = False
motor_control.DEBUG_CONTROL = False
motor_guidance._dbg = lambda line: None
motor_control._dbg = lambda line: None

# ------------------------------------------------------------------------------
# Section 4 — simulated clock
# ------------------------------------------------------------------------------
_sim_clock = [0.0]

class _SimTimeMod:
    @staticmethod
    def time() -> float:
        return _sim_clock[0]

    @staticmethod
    def sleep(s: float) -> None:
        return None

    def __getattr__(self, name):
        return getattr(_real_time, name)

motor_guidance.time = _SimTimeMod()

# ------------------------------------------------------------------------------
# Section 5 — environment / coordinate constants
# ------------------------------------------------------------------------------
REF_LAT, REF_LON = 35.0950, 127.0950
LAT2M = 111320.0
COS_LAT = math.cos(math.radians(REF_LAT))
DT = 0.1
MAX_STEPS = 3500

def en_to_latlon(E_m: float, N_m: float):
    return REF_LAT + N_m / LAT2M, REF_LON + E_m / (LAT2M * COS_LAT)

def bearing_distance_to_en(bearing_deg: float, distance_m: float):
    rad = math.radians(bearing_deg)
    return distance_m * math.sin(rad), distance_m * math.cos(rad)

def wrap_180(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0

# ------------------------------------------------------------------------------
# Section 6 — tunable high-fidelity model parameters
# ------------------------------------------------------------------------------
@dataclass
class ServoSideModel:
    gain: float
    deadband_deg: float
    slew_deg_s: float
    delay_steps: int
    neutral_deg: float = 60.0
    max_deg: float = 120.0

@dataclass
class HiFiConfig:
    va_trim: float = 5.5
    descent_trim: float = 5.0
    mass_kg: float = 0.42

    # sensor models
    gps_noise_m: float = 3.5
    gps_delay: int = 4
    gps_warmup: int = 12
    yaw_noise_deg: float = 4.0
    gyrz_noise_rps: float = math.radians(1.5)
    bias_drift_rate: float = math.radians(0.06)
    mag_pend_gain: float = 0.8
    baro_noise_m: float = 3.0

    # atmosphere
    wind_z_ref: float = 600.0
    wind_alpha: float = 0.30
    l_hor: float = 150.0
    l_ver: float = 30.0
    sig_hor: float = 4.0
    sig_ver: float = 2.5

    # response model
    base_yaw_gain: float = 0.95          # deg/s per effective brake delta unit
    yaw_exp: float = 1.35                # nonlinear delta exponent
    yaw_deadband_deg: float = 4.0        # global line deadband
    speed_loss_per_brake: float = 0.020  # m/s per total brake deg
    speed_loss_turn: float = 0.010       # m/s per |delta| deg
    sink_quad_total: float = 0.00045     # sink increase from common brake
    sink_quad_diff: float = 0.00020      # sink increase from differential brake
    sink_bank_gain: float = 0.020        # sink increase from bank angle

    # bank + pendulum
    bank_tau_s: float = 0.8
    bank_gain_deg_per_yr: float = 0.85
    max_bank_deg: float = 40.0
    pend_length_m: float = 1.5
    pend_zeta: float = 0.08
    pend_couple: float = 0.14

    # left/right asymmetry
    left: ServoSideModel = field(default_factory=lambda: ServoSideModel(
        gain=0.90, deadband_deg=3.0, slew_deg_s=180.0, delay_steps=1))
    right: ServoSideModel = field(default_factory=lambda: ServoSideModel(
        gain=1.05, deadband_deg=4.0, slew_deg_s=220.0, delay_steps=2))

DEFAULT_CFG = HiFiConfig()

# ------------------------------------------------------------------------------
# Section 7 — per-run stateful physics
# ------------------------------------------------------------------------------
class PhysicsState:
    def __init__(self, rng: np.random.Generator, cfg: HiFiConfig):
        self.rng = rng
        self.cfg = cfg

        # turbulence
        self.turb_u = 0.0
        self.turb_v = 0.0
        self.turb_w = 0.0

        # left/right actual brake-line engagement in "equivalent brake degrees"
        # 0 = neutral, + = deeper brake pull.
        self.left_actual_brake_deg = 0.0
        self.right_actual_brake_deg = 0.0
        self.left_queue = deque([0.0] * (cfg.left.delay_steps + 1),
                                maxlen=cfg.left.delay_steps + 1)
        self.right_queue = deque([0.0] * (cfg.right.delay_steps + 1),
                                 maxlen=cfg.right.delay_steps + 1)

        # flight attitude coupling
        self.bank_deg = 0.0
        self.pend_phi_deg = 0.0
        self.pend_phidot_deg_s = 0.0

        # imu drift
        self.imu_bias_gyrz = 0.0

        # gps delay buffers
        self._E_buf = deque(maxlen=cfg.gps_delay)
        self._N_buf = deque(maxlen=cfg.gps_delay)
        self._spd_buf = deque(maxlen=cfg.gps_delay)
        self._crs_buf = deque(maxlen=cfg.gps_delay)

    def wind_at_alt(self, alt_m: float, wind_u_ref: float, wind_unit_E: float, wind_unit_N: float):
        U = wind_u_ref * (max(alt_m, 1.0) / self.cfg.wind_z_ref) ** self.cfg.wind_alpha
        return U * wind_unit_E, U * wind_unit_N

    def step_turbulence(self, Va: float, dt: float):
        rho_h = math.exp(-Va * dt / self.cfg.l_hor)
        rho_v = math.exp(-Va * dt / self.cfg.l_ver)
        sig_h = self.cfg.sig_hor * math.sqrt(max(0.0, 1.0 - rho_h ** 2))
        sig_v = self.cfg.sig_ver * math.sqrt(max(0.0, 1.0 - rho_v ** 2))
        self.turb_u = rho_h * self.turb_u + sig_h * self.rng.standard_normal()
        self.turb_v = rho_h * self.turb_v + sig_h * self.rng.standard_normal()
        self.turb_w = rho_v * self.turb_w + sig_v * self.rng.standard_normal()

    @staticmethod
    def _slew(curr: float, target: float, slew_deg_s: float, dt: float) -> float:
        max_delta = slew_deg_s * dt
        return curr + max(-max_delta, min(max_delta, target - curr))

    @staticmethod
    def _effective_brake(brake_cmd_deg: float, side: ServoSideModel) -> float:
        raw = brake_cmd_deg * side.gain
        return max(0.0, raw - side.deadband_deg)

    def step_servos(self, left_brake_cmd_deg: float, right_brake_cmd_deg: float, dt: float):
        self.left_queue.append(left_brake_cmd_deg)
        self.right_queue.append(right_brake_cmd_deg)
        left_delayed = self.left_queue[0]
        right_delayed = self.right_queue[0]

        self.left_actual_brake_deg = self._slew(self.left_actual_brake_deg, left_delayed, self.cfg.left.slew_deg_s, dt)
        self.right_actual_brake_deg = self._slew(self.right_actual_brake_deg, right_delayed, self.cfg.right.slew_deg_s, dt)

        left_eff = self._effective_brake(self.left_actual_brake_deg, self.cfg.left)
        right_eff = self._effective_brake(self.right_actual_brake_deg, self.cfg.right)
        return left_eff, right_eff

    def step_bank(self, yaw_rate_phy_deg_s: float, dt: float):
        bank_target = max(-self.cfg.max_bank_deg,
                          min(self.cfg.max_bank_deg, self.cfg.bank_gain_deg_per_yr * yaw_rate_phy_deg_s))
        alpha = max(0.0, min(1.0, dt / self.cfg.bank_tau_s))
        self.bank_deg = (1.0 - alpha) * self.bank_deg + alpha * bank_target

    def step_pendulum(self, yaw_rate_phy_deg_s: float, dt: float):
        omega_n = math.sqrt(9.81 / self.cfg.pend_length_m)
        ddphi_rad = (
            -2.0 * self.cfg.pend_zeta * omega_n * math.radians(self.pend_phidot_deg_s)
            - omega_n ** 2 * math.radians(self.pend_phi_deg)
            + math.radians(self.cfg.pend_couple * yaw_rate_phy_deg_s)
        )
        self.pend_phidot_deg_s += math.degrees(ddphi_rad) * dt
        self.pend_phi_deg += self.pend_phidot_deg_s * dt

    def sensor_gps(self, E: float, N: float, V_E_gnd: float, V_N_gnd: float, step: int):
        self._E_buf.append(E + self.rng.normal(0.0, self.cfg.gps_noise_m))
        self._N_buf.append(N + self.rng.normal(0.0, self.cfg.gps_noise_m))
        self._spd_buf.append(math.hypot(V_E_gnd, V_N_gnd))
        self._crs_buf.append(math.degrees(math.atan2(V_E_gnd, V_N_gnd)) % 360.0)

        warm = step >= self.cfg.gps_warmup and len(self._E_buf) == self.cfg.gps_delay
        if warm:
            E_out = self._E_buf[0]
            N_out = self._N_buf[0]
            spd_out = self._spd_buf[0]
            crs_out = self._crs_buf[0]
        else:
            E_out = N_out = spd_out = crs_out = 0.0

        lat_out, lon_out = en_to_latlon(E_out, N_out)
        gps_vec = SimpleNamespace(lat=lat_out, lon=lon_out, speed=spd_out, course=crs_out)
        gps_fid = SimpleNamespace(
            fix_quality=1 if warm else 0,
            sats=8 if warm else 0,
            rmc_status="A" if warm else "V",
        )
        return gps_vec, gps_fid

    def sensor_imu(self, heading_true_deg: float, yaw_rate_true_deg_s: float):
        drift = self.rng.normal(0.0, self.cfg.bias_drift_rate * math.sqrt(DT))
        self.imu_bias_gyrz = max(-0.2, min(0.2, self.imu_bias_gyrz + drift))
        yaw = heading_true_deg + self.cfg.mag_pend_gain * self.pend_phi_deg + self.rng.normal(0.0, self.cfg.yaw_noise_deg)
        gyrz = math.radians(yaw_rate_true_deg_s) + self.imu_bias_gyrz + self.rng.normal(0.0, self.cfg.gyrz_noise_rps)
        return SimpleNamespace(yaw=yaw % 360.0, gyrz=gyrz)

    def sensor_baro(self, alt_true: float) -> float:
        return max(0.1, alt_true + self.rng.normal(0.0, self.cfg.baro_noise_m))

# ------------------------------------------------------------------------------
# Section 8 — helpers for actuator convention audit
# ------------------------------------------------------------------------------
def actuator_convention_audit():
    mock_pi = _MockPi()
    res_pos = motor_control.control(mock_pi, +30.0)
    res_neg = motor_control.control(mock_pi, -30.0)

    # In the current mixer, "right brake deeper" is represented by a *larger right pulse*,
    # not by a larger right_cmd_deg (because right servo angle decreases as pulse increases).
    ok_pos = res_pos.right_pulse > motor_control.RIGHT_NEUTRAL and res_pos.left_pulse >= motor_control.LEFT_NEUTRAL
    ok_neg = res_neg.left_pulse < motor_control.LEFT_NEUTRAL and res_neg.right_pulse <= motor_control.RIGHT_NEUTRAL
    same_pin = motor_control.PARAFOIL_LEFT_MOTOR_PIN == motor_control.PARAFOIL_RIGHT_MOTOR_PIN

    return SimpleNamespace(
        pass_all=(ok_pos and ok_neg and not same_pin),
        ok_pos=ok_pos,
        ok_neg=ok_neg,
        same_pin=same_pin,
        left_pin=motor_control.PARAFOIL_LEFT_MOTOR_PIN,
        right_pin=motor_control.PARAFOIL_RIGHT_MOTOR_PIN,
        res_pos=res_pos,
        res_neg=res_neg,
    )

# ------------------------------------------------------------------------------
# Section 9 — core high-fidelity run
# ------------------------------------------------------------------------------
def run_hifi_once(
    *,
    init_heading: float,
    target_E: float,
    target_N: float,
    wind_speed: float,
    wind_dir_met: float,
    seed: int,
    cfg: HiFiConfig = DEFAULT_CFG,
    start_alt: float = 600.0,
    enable_turbulence: bool = True,
):
    rng = np.random.default_rng(seed)
    phys = PhysicsState(rng, cfg)
    mock_pi = _MockPi()

    wind_unit_E = math.sin(math.radians(wind_dir_met + 180.0))
    wind_unit_N = math.cos(math.radians(wind_dir_met + 180.0))

    _sim_clock[0] = 0.0
    motor_guidance.init_guidance()
    motor_guidance.DEBUG_GUIDANCE = False
    motor_guidance.GPS_JUMP_MAX_SPEED = 200.0

    target_lat, target_lon = en_to_latlon(target_E, target_N)
    motor_guidance.set_start_coordinates(REF_LAT, REF_LON)
    motor_guidance.set_target_coord(target_lat, target_lon)
    target_ns = SimpleNamespace(lat=target_lat, lon=target_lon)

    E = 0.0
    N = 0.0
    alt = start_alt
    heading = init_heading % 360.0
    yaw_rate_phy = 0.0

    hist = {
        "t": [],
        "E": [],
        "N": [],
        "alt": [],
        "heading": [],
        "cmd_yr": [],
        "phase": [],
        "hdg_err": [],
        "left_cmd_deg": [],
        "right_cmd_deg": [],
        "left_eff": [],
        "right_eff": [],
        "yaw_rate_phy": [],
        "bank_deg": [],
        "pend_deg": [],
        "gnd_speed": [],
    }

    for step in range(MAX_STEPS):
        _sim_clock[0] = step * DT

        # Propagate using current heading / bank / turbulence
        hdg_rad = math.radians(heading)
        total_brake = phys.left_actual_brake_deg + phys.right_actual_brake_deg

        Va_fwd = cfg.va_trim - cfg.speed_loss_per_brake * total_brake - cfg.speed_loss_turn * abs(yaw_rate_phy)
        Va_fwd = max(Va_fwd, 3.8)

        wE, wN = phys.wind_at_alt(alt, wind_speed, wind_unit_E, wind_unit_N)
        if enable_turbulence:
            phys.step_turbulence(Va_fwd, DT)
            wE += phys.turb_u
            wN += phys.turb_v

        # crude banked-turn slip model
        slip_E = 0.18 * math.sin(math.radians(phys.bank_deg + phys.pend_phi_deg))
        slip_N = -0.18 * math.cos(math.radians(heading)) * math.sin(math.radians(phys.bank_deg))
        V_E_gnd = Va_fwd * math.sin(hdg_rad) + wE + slip_E
        V_N_gnd = Va_fwd * math.cos(hdg_rad) + wN + slip_N

        gps_vec, gps_fid = phys.sensor_gps(E, N, V_E_gnd, V_N_gnd, step)
        imu_data = phys.sensor_imu(heading, yaw_rate_phy)
        baro_m = phys.sensor_baro(alt)

        g_result = motor_guidance.guidance(imu_data, gps_vec, gps_fid, target_ns, baro_m=baro_m)
        cmd_yr = float(g_result.commanded_yaw_rate)
        phase = g_result.state

        m_result = motor_control.control(mock_pi, cmd_yr)

        left_brake_cmd = max(0.0, (m_result.left_pulse - motor_control.LEFT_NEUTRAL) / motor_control.PULSE_PER_DEG)
        right_brake_cmd = max(0.0, (m_result.right_pulse - motor_control.RIGHT_NEUTRAL) / motor_control.PULSE_PER_DEG)

        left_eff, right_eff = phys.step_servos(left_brake_cmd, right_brake_cmd, DT)

        # Important sign convention:
        # +angl_to_turn -> +commanded_yaw_rate -> right side more braked -> right turn.
        # In current motor_control.py, actual right brake dominance means:
        delta_brake = right_eff - left_eff   # positive => right-brake-dominant => right turn
        common_brake = 0.5 * (abs(left_eff) + abs(right_eff))

        # nonlinear turn effectiveness after combined deadband
        delta_mag = max(0.0, abs(delta_brake) - cfg.yaw_deadband_deg)
        nonlinear_delta = math.copysign(delta_mag ** cfg.yaw_exp, delta_brake)

        # Positive right-brake dominance should produce positive/right yaw rate in this sim.
        yaw_rate_phy = cfg.base_yaw_gain * nonlinear_delta * (Va_fwd / cfg.va_trim)

        phys.step_bank(yaw_rate_phy, DT)
        phys.step_pendulum(yaw_rate_phy, DT)

        # Apply turn-induced speed/sink penalties
        Va_fwd = cfg.va_trim - cfg.speed_loss_per_brake * (2.0 * common_brake) - cfg.speed_loss_turn * abs(delta_brake)
        Va_fwd = max(Va_fwd, 3.6)
        descent_rate = (
            cfg.descent_trim
            + cfg.sink_quad_total * (2.0 * common_brake) ** 2
            + cfg.sink_quad_diff * (delta_brake ** 2)
            + cfg.sink_bank_gain * abs(math.sin(math.radians(phys.bank_deg)))
        )
        if enable_turbulence:
            descent_rate = max(1.0, descent_rate + 0.25 * phys.turb_w)
        else:
            descent_rate = max(1.0, descent_rate)

        heading = (heading + yaw_rate_phy * DT) % 360.0
        hdg_rad = math.radians(heading)
        V_E_gnd = Va_fwd * math.sin(hdg_rad) + wE + slip_E
        V_N_gnd = Va_fwd * math.cos(hdg_rad) + wN + slip_N
        E += V_E_gnd * DT
        N += V_N_gnd * DT
        alt -= descent_rate * DT

        direct_bearing = math.degrees(math.atan2(target_E - E, target_N - N)) % 360.0
        hdg_err = wrap_180(heading - direct_bearing)

        hist["t"].append(step * DT)
        hist["E"].append(E)
        hist["N"].append(N)
        hist["alt"].append(max(0.0, alt))
        hist["heading"].append(heading)
        hist["cmd_yr"].append(cmd_yr)
        hist["phase"].append(phase)
        hist["hdg_err"].append(hdg_err)
        hist["left_cmd_deg"].append(m_result.left_cmd_deg)
        hist["right_cmd_deg"].append(m_result.right_cmd_deg)
        hist["left_eff"].append(left_eff)
        hist["right_eff"].append(right_eff)
        hist["yaw_rate_phy"].append(yaw_rate_phy)
        hist["bank_deg"].append(phys.bank_deg)
        hist["pend_deg"].append(phys.pend_phi_deg)
        hist["gnd_speed"].append(math.hypot(V_E_gnd, V_N_gnd))

        if alt <= 0.0:
            break

    final_dist = math.hypot(E - target_E, N - target_N)
    return SimpleNamespace(
        final_dist=final_dist,
        landed=(alt <= 0.0),
        steps=step + 1,
        final_E=E,
        final_N=N,
        final_alt=max(0.0, alt),
        hist=hist,
    )

# ------------------------------------------------------------------------------
# Section 10 — Monte Carlo wrapper
# ------------------------------------------------------------------------------
def run_monte_carlo(
    *,
    runs: int,
    target_E: float,
    target_N: float,
    wind_speed: float,
    wind_dir_met: float,
    cfg: HiFiConfig = DEFAULT_CFG,
    init_heading_mean: float = 0.0,
):
    results = []
    rng = np.random.default_rng(12345)

    for i in range(runs):
        init_heading = (init_heading_mean + rng.normal(0.0, 35.0)) % 360.0
        run = run_hifi_once(
            init_heading=init_heading,
            target_E=target_E,
            target_N=target_N,
            wind_speed=max(0.0, wind_speed + rng.normal(0.0, 1.0)),
            wind_dir_met=(wind_dir_met + rng.normal(0.0, 12.0)) % 360.0,
            seed=int(rng.integers(0, 2**31 - 1)),
            cfg=cfg,
            start_alt=max(420.0, 600.0 + rng.normal(0.0, 20.0)),
            enable_turbulence=True,
        )
        results.append(run)
    return results

# ------------------------------------------------------------------------------
# Section 11 — reporting helpers
# ------------------------------------------------------------------------------
PASS = "\033[92mPASS\033[0m"
FAIL = "\033[91mFAIL\033[0m"
WARN = "\033[93mWARN\033[0m"

def _pctl(arr, q):
    return float(np.percentile(np.asarray(arr, dtype=float), q))

def print_connection_audit():
    audit = actuator_convention_audit()
    print("\n[Connection audit]")
    print(f"  pins: left={audit.left_pin}, right={audit.right_pin}")
    print(f"  +30°/s -> L={audit.res_pos.left_cmd_deg:.1f}°, R={audit.res_pos.right_cmd_deg:.1f}°")
    print(f"  -30°/s -> L={audit.res_neg.left_cmd_deg:.1f}°, R={audit.res_neg.right_cmd_deg:.1f}°")
    print(f"  [{PASS if audit.pass_all else FAIL}] sign convention / pin sanity")
    return audit

def summarize_single(run):
    phases = set(run.hist["phase"])
    max_bank = max(map(abs, run.hist["bank_deg"])) if run.hist["bank_deg"] else 0.0
    max_pend = max(map(abs, run.hist["pend_deg"])) if run.hist["pend_deg"] else 0.0
    cmd_rms = float(np.sqrt(np.mean(np.square(run.hist["cmd_yr"])))) if run.hist["cmd_yr"] else 0.0
    print("\n[Single-run summary]")
    print(f"  landed      : {run.landed}")
    print(f"  final_dist  : {run.final_dist:.1f} m")
    print(f"  steps       : {run.steps}")
    print(f"  phases      : {sorted(phases)}")
    print(f"  max bank    : {max_bank:.1f} deg")
    print(f"  max pend    : {max_pend:.1f} deg")
    print(f"  cmd RMS     : {cmd_rms:.1f} deg/s")

def summarize_mc(results):
    dists = [r.final_dist for r in results]
    landed = sum(1 for r in results if r.landed)
    within_25 = sum(d <= 25.0 for d in dists)
    within_50 = sum(d <= 50.0 for d in dists)
    within_100 = sum(d <= 100.0 for d in dists)
    print("\n[Monte Carlo summary]")
    print(f"  runs        : {len(results)}")
    print(f"  landed      : {landed}/{len(results)}")
    print(f"  mean dist   : {np.mean(dists):.1f} m")
    print(f"  p50 / p90   : {_pctl(dists, 50):.1f} / {_pctl(dists, 90):.1f} m")
    print(f"  within 25m  : {within_25}/{len(results)}")
    print(f"  within 50m  : {within_50}/{len(results)}")
    print(f"  within 100m : {within_100}/{len(results)}")

    label = PASS if _pctl(dists, 90) <= 100.0 else (WARN if _pctl(dists, 90) <= 150.0 else FAIL)
    print(f"  [{label}] p90 landing dispersion check")
    return dists

def try_plot(single_run, mc_dists):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return

    if single_run is not None:
        fig, ax = plt.subplots(figsize=(7, 7))
        ax.plot(single_run.hist["E"], single_run.hist["N"], linewidth=1.5)
        ax.scatter([single_run.hist["E"][0]], [single_run.hist["N"][0]], marker="o")
        ax.scatter([single_run.hist["E"][-1]], [single_run.hist["N"][-1]], marker="x")
        ax.set_xlabel("East [m]")
        ax.set_ylabel("North [m]")
        ax.set_title("sim_hifi — single-run trajectory")
        ax.axis("equal")
        ax.grid(True, alpha=0.3)
        out = HERE / "sim_hifi_trajectory.png"
        fig.tight_layout()
        fig.savefig(out, dpi=130)
        plt.close(fig)
        print(f"\n  plot saved: {out}")

    if mc_dists:
        fig, ax = plt.subplots(figsize=(7, 4.5))
        xs = np.sort(np.asarray(mc_dists))
        ys = np.arange(1, len(xs) + 1) / len(xs) * 100.0
        ax.plot(xs, ys, linewidth=1.8)
        ax.axvline(25, linestyle="--")
        ax.axvline(50, linestyle="--")
        ax.axvline(100, linestyle="--")
        ax.set_xlabel("Landing distance to target [m]")
        ax.set_ylabel("CDF [%]")
        ax.set_title("sim_hifi — landing distance CDF")
        ax.grid(True, alpha=0.3)
        out = HERE / "sim_hifi_cdf.png"
        fig.tight_layout()
        fig.savefig(out, dpi=130)
        plt.close(fig)
        print(f"  plot saved: {out}")

# ------------------------------------------------------------------------------
# Section 12 — CLI
# ------------------------------------------------------------------------------
def parse_args():
    ap = argparse.ArgumentParser(description="High-fidelity parafoil simulation")
    ap.add_argument("--single", action="store_true", help="run one nominal trajectory")
    ap.add_argument("--runs", type=int, default=30, help="Monte Carlo run count")
    ap.add_argument("--target-bearing", type=float, default=45.0, help="target bearing from release point [deg]")
    ap.add_argument("--target-distance", type=float, default=700.0, help="target distance from release point [m]")
    ap.add_argument("--wind-speed", type=float, default=4.5, help="reference wind speed at z_ref [m/s]")
    ap.add_argument("--wind-dir", type=float, default=240.0, help="meteorological wind FROM direction [deg]")
    ap.add_argument("--init-heading", type=float, default=180.0, help="single-run initial heading [deg]")
    ap.add_argument("--plot", action="store_true", help="save png plots")
    return ap.parse_args()

def main():
    args = parse_args()
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    print("=" * 68)
    print("  CANSAT PARAFOIL HIGH-FIDELITY SIMULATION")
    print(f"  sim_hifi.py  |  {timestamp}")
    print("=" * 68)

    target_E, target_N = bearing_distance_to_en(args.target_bearing, args.target_distance)

    audit = print_connection_audit()

    single_run = run_hifi_once(
        init_heading=args.init_heading,
        target_E=target_E,
        target_N=target_N,
        wind_speed=args.wind_speed,
        wind_dir_met=args.wind_dir,
        seed=7,
        cfg=DEFAULT_CFG,
        start_alt=600.0,
        enable_turbulence=True,
    )
    summarize_single(single_run)

    mc_results = run_monte_carlo(
        runs=max(1, args.runs),
        target_E=target_E,
        target_N=target_N,
        wind_speed=args.wind_speed,
        wind_dir_met=args.wind_dir,
        cfg=DEFAULT_CFG,
        init_heading_mean=args.init_heading,
    )
    dists = summarize_mc(mc_results)

    print("\n[Integrated upgrades]")
    print("  - real motor_guidance + motor_control in loop")
    print("  - left/right servo asymmetry, delay, deadband, slew")
    print("  - nonlinear brake-to-yaw mapping")
    print("  - turn-induced sink / speed penalty")
    print("  - bank + pendulum coupling")
    print("  - noisy delayed GPS / IMU / BARO")
    print("  - wind profile + turbulence")

    if not audit.pass_all:
        print("\n[!] Connection audit failed. Check motor pin mapping and sign convention.")

    if args.plot:
        try_plot(single_run, dists)

if __name__ == "__main__":
    main()
