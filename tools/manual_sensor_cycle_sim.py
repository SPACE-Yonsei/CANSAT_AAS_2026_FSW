"""Manual sensor-cycle simulator for the parafoil guidance/control pipeline.

This is a Software-In-the-Loop (SIL) harness. It feeds *sensor* inputs into the
production guidance/control pipeline and lets the pipeline decide everything
downstream.  The operator never selects a control mode: control modes are an
*output* of `guidance.DecideControlMode`, derived from sensor freshness.

Two input layers (see docs/manual_sensor_cycle_sim.md):

  A. Premise / simulation setup  — origin, target, initial payload pose, dt,
     cycle count.  Constant for the whole run unless explicitly changed.
  B. Per-cycle sensor inputs     — health (on/off/stale) + values for GPS pos,
     GPS motion, IMU yaw/gyro/acc, and baro.  Editable every cycle; a blank
     interactive line keeps the previous cycle's inputs.

One cycle runs the production pipeline exactly once::

    mode  = guidance.DecideControlMode(gps, imu, baro, now)   # exactly 1x
    l1in  = guidance.ProduceL1Input(now)                      # read-only on DR
    l1out = guidance.ProduceL1Output(l1in)  (only if l1in.valid)
    ctrl  = control.ProduceCtrlInput/ProduceCtrlOutput  (only if l1out.valid)
            else control.WriteNeutral(now, mode)

Hard rules honoured here (do NOT change without re-reading guidance.py):
  * `DecideControlMode` is the only mode source; it is called once per cycle.
  * `_STATE_t.nav.control_mode` is never assigned by this tool.
  * `SelectControlMode` / `FillNav` are never called directly.
  * `ProduceL1Input` must not mutate `dr.current_E/N` — we record both sides.
  * No pigpio / servo / file side effects beyond the trace CSV.
  * No randomness, no time.sleep.

Units: distance=m, angle=deg at the CLI / rad inside guidance, time=s.
Frame: local NE, +N north, +E east; course=0 deg north, course=90 deg east.

NOTE on off vs stale freshness (important, read before interpreting modes):
  guidance freshness is *age-based* on the last accepted sample.  A real
  health=0 dropout leaves the previous (recent) timestamp in place, so the
  channel would keep reading "fresh" for up to its max-age window of *simulated*
  time.  Because this harness compresses time (dt defaults to 0.05 s), an
  instantaneous dropout would not register within a 10-cycle run.  To make a
  dropout observable in the same cycle the operator requests it, both "off" and
  "stale" present an *aged* sample (health=1, finite value, timestamp older than
  the freshness window).  guidance makes no decision that distinguishes a
  powered-off sensor from a stale one, so the resulting control mode is
  identical either way; the literal operator intent ("on"/"off"/"stale") is
  still recorded verbatim in the trace.
"""
from __future__ import annotations

import argparse
import csv
import importlib
import math
import sys
from dataclasses import dataclass, field, replace
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Callable, Optional


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from lib import config  # noqa: E402
from Sensor_Motor import control, guidance  # noqa: E402


# ── Defaults (CLI premise) ───────────────────────────────────────────────────
DEFAULT_ORIGIN_LAT = 38.000000
DEFAULT_ORIGIN_LON = -79.000000
DEFAULT_TARGET_E = 80.0
DEFAULT_TARGET_N = 20.0
DEFAULT_PAYLOAD_E = 0.0
DEFAULT_PAYLOAD_N = 0.0
DEFAULT_COURSE_DEG = 0.0
DEFAULT_SPEED_MPS = 4.0
DEFAULT_YAW_DEG = 0.0
DEFAULT_GYRZ_DPS = 0.0
DEFAULT_BARO_ALT_M = 40.0
DEFAULT_BARO_SINK_MPS = 2.0
DEFAULT_DT = 0.05
DEFAULT_CYCLES = 10
DEFAULT_SAMPLE_CYCLES = "1,2,3,10"
DEFAULT_OUTPUT = "manual_sensor_cycle_trace.csv"
DEFAULT_PRESET = "GPS_DROPOUT_SEQUENCE_RIGHT"
DEFAULT_NOW = 1000.0
DEFAULT_WARMUP_CYCLES = 40

VALID_STATES = ("on", "off", "stale")
SENSOR_FIELDS = ("gps_pos", "gps_motion", "yaw", "gyrz", "baro", "acc")

# Freshness windows per sensor group (used to age out-of-window samples).
GPS_MAX_AGE = float(getattr(config, "GPS_FRESH_MAX_AGE_S", 5.0))
IMU_MAX_AGE = float(getattr(config, "IMU_FRESH_MAX_AGE_S", 3.0))
BARO_MAX_AGE = float(getattr(config, "BARO_FRESH_MAX_AGE_S", 2.0))


# ── Small helpers ─────────────────────────────────────────────────────────────

def finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def deg(rad: Any) -> float:
    if not finite(rad):
        return float("nan")
    return math.degrees(float(rad))


def ne_to_latlon(N: float, E: float, origin_lat: float, origin_lon: float) -> tuple[float, float]:
    """Inverse of guidance.latlon_to_ne for the local NE frame.

    N is metres north, E is metres east of (origin_lat, origin_lon).
    """
    R = guidance.EARTH_RADIUS_M
    lat = origin_lat + math.degrees(N / R)
    lon = origin_lon + math.degrees(E / (R * math.cos(math.radians(origin_lat))))
    return lat, lon


def reload_modules() -> None:
    """Reload guidance + control and clear control PID state.

    Called once at the start of every run so module-level _STATE_t / PID state
    never leak between simulations.
    """
    global guidance, control
    guidance = importlib.reload(guidance)
    control = importlib.reload(control)
    control.reset()


# ── Premise (layer A) ─────────────────────────────────────────────────────────

@dataclass
class Premise:
    origin_lat: float = DEFAULT_ORIGIN_LAT
    origin_lon: float = DEFAULT_ORIGIN_LON
    target_E: float = DEFAULT_TARGET_E
    target_N: float = DEFAULT_TARGET_N
    payload_E: float = DEFAULT_PAYLOAD_E
    payload_N: float = DEFAULT_PAYLOAD_N
    course_deg: float = DEFAULT_COURSE_DEG
    speed_mps: float = DEFAULT_SPEED_MPS
    yaw_deg: float = DEFAULT_YAW_DEG
    gyrz_dps: float = DEFAULT_GYRZ_DPS
    baro_alt_m: float = DEFAULT_BARO_ALT_M
    baro_sink_mps: float = DEFAULT_BARO_SINK_MPS
    dt: float = DEFAULT_DT
    cycles: int = DEFAULT_CYCLES
    now0: float = DEFAULT_NOW
    warmup_cycles: int = 0


# ── Per-cycle sensor input (layer B) ──────────────────────────────────────────

@dataclass
class CycleInput:
    # health states
    gps_pos: str = "on"
    gps_motion: str = "on"
    yaw: str = "on"
    gyrz: str = "on"
    baro: str = "on"
    acc: str = "on"
    # values
    payload_E: float = DEFAULT_PAYLOAD_E
    payload_N: float = DEFAULT_PAYLOAD_N
    course_deg: float = DEFAULT_COURSE_DEG
    speed_mps: float = DEFAULT_SPEED_MPS
    yaw_deg: float = DEFAULT_YAW_DEG
    gyrz_dps: float = DEFAULT_GYRZ_DPS
    baro_alt_m: float = DEFAULT_BARO_ALT_M
    baro_sink_mps: float = DEFAULT_BARO_SINK_MPS
    acc_x: float = 0.0
    acc_y: float = 0.0
    # bookkeeping (not sent to guidance directly)
    state: str = "DESCENT"
    motor_enabled: bool = True

    def copy(self) -> "CycleInput":
        return replace(self)


def base_cycle_from_premise(premise: Premise) -> CycleInput:
    return CycleInput(
        payload_E=premise.payload_E,
        payload_N=premise.payload_N,
        course_deg=premise.course_deg,
        speed_mps=premise.speed_mps,
        yaw_deg=premise.yaw_deg,
        gyrz_dps=premise.gyrz_dps,
        baro_alt_m=premise.baro_alt_m,
        baro_sink_mps=premise.baro_sink_mps,
    )


# ── Input parsing (interactive + scripted CSV share this) ─────────────────────

_NUMERIC_CYCLE_KEYS = {
    "payload_e": "payload_E",
    "payload_n": "payload_N",
    "course_deg": "course_deg",
    "speed_mps": "speed_mps",
    "yaw_deg": "yaw_deg",
    "gyrz_dps": "gyrz_dps",
    "baro_alt": "baro_alt_m",
    "baro_alt_m": "baro_alt_m",
    "baro_sink": "baro_sink_mps",
    "baro_sink_mps": "baro_sink_mps",
    "acc_x": "acc_x",
    "acc_y": "acc_y",
}
_STATE_KEYS = set(SENSOR_FIELDS)
_PREMISE_TARGET_KEYS = {"target_e": "target_E", "target_n": "target_N"}


def _parse_bool(token: str) -> bool:
    low = token.strip().lower()
    if low in ("1", "true", "on", "yes", "y"):
        return True
    if low in ("0", "false", "off", "no", "n"):
        return False
    raise ValueError(f"cannot parse bool from {token!r}")


def apply_overrides(cycle: CycleInput, premise: Premise, overrides: dict[str, Any]) -> None:
    """Mutate `cycle` (and premise.target on target_* keys) from a key→value map.

    Unknown keys raise ValueError (the spec forbids silently ignoring them).
    State keys must be on/off/stale; numeric keys must parse to float.
    """
    for raw_key, raw_val in overrides.items():
        key = str(raw_key).strip().lower()
        if key == "":
            continue
        if key in _STATE_KEYS:
            val = str(raw_val).strip().lower()
            if val not in VALID_STATES:
                raise ValueError(f"{key} must be one of {VALID_STATES}, got {raw_val!r}")
            setattr(cycle, key, val)
        elif key in _NUMERIC_CYCLE_KEYS:
            setattr(cycle, _NUMERIC_CYCLE_KEYS[key], float(raw_val))
        elif key in _PREMISE_TARGET_KEYS:
            setattr(premise, _PREMISE_TARGET_KEYS[key], float(raw_val))
        elif key == "state":
            cycle.state = str(raw_val).strip()
        elif key == "motor_enabled":
            cycle.motor_enabled = _parse_bool(str(raw_val))
        else:
            raise ValueError(f"unknown input key {raw_key!r}")


def parse_kv_line(line: str) -> dict[str, str]:
    """Parse a `k=v k2=v2` line into a dict. Empty line → empty dict."""
    out: dict[str, str] = {}
    for token in line.replace(",", " ").split():
        if "=" not in token:
            raise ValueError(f"expected key=value, got {token!r}")
        k, v = token.split("=", 1)
        if k.strip() == "":
            raise ValueError(f"empty key in {token!r}")
        out[k.strip()] = v.strip()
    return out


# ── Sensor mock construction (production duck-typed SimpleNamespace) ───────────

def _sample(state: str, now: float, max_age: float) -> tuple[int, float, bool]:
    """Return (health, ts, value_present) for a sensor channel state.

    on    → health=1, ts=now, value present (fresh)
    stale → health=1, ts=now-max_age-1, value present (aged → not fresh)
    off   → health=1, ts=now-max_age-1, value present (aged → not fresh)

    See the module docstring: off and stale both present an aged sample so an
    operator-requested dropout registers in the same compressed-time cycle.
    """
    if state == "on":
        return 1, now, True
    if state in ("off", "stale"):
        return 1, now - max_age - 1.0, True
    raise ValueError(f"bad sensor state {state!r}")


def make_gps(cycle: CycleInput, premise: Premise, now: float) -> SimpleNamespace:
    pos_health, pos_ts, _ = _sample(cycle.gps_pos, now, GPS_MAX_AGE)
    mot_health, mot_ts, _ = _sample(cycle.gps_motion, now, GPS_MAX_AGE)
    lat, lon = ne_to_latlon(cycle.payload_N, cycle.payload_E,
                            premise.origin_lat, premise.origin_lon)
    return SimpleNamespace(
        lat=lat,
        lon=lon,
        pos_health=pos_health,
        pos_ts=pos_ts,
        course_rad=math.radians(cycle.course_deg),
        speed_mps=cycle.speed_mps,
        motion_health=mot_health,
        motion_ts=mot_ts,
    )


def make_imu(cycle: CycleInput, now: float) -> SimpleNamespace:
    """Single IMU timestamp governs yaw/gyro/acc freshness (one rx per sample).

    If any channel is "on" the sample is fresh (ts=now); else if any is aged
    (off/stale) the sample is aged; if every channel is off-with-no-value the
    IMU reads unhealthy.  Per-channel validity still follows each channel's
    state, so e.g. yaw=on + gyrz=off yields a fresh yaw and a stale gyro.
    """
    states = (cycle.yaw, cycle.gyrz, cycle.acc)
    if any(s == "on" for s in states):
        health, ts = 1, now
    else:
        # all channels aged → present the IMU as a single aged sample
        health, ts = 1, now - IMU_MAX_AGE - 1.0

    def chan(state: str, value: float) -> Optional[float]:
        # "on" forces fresh; an aged channel paired with a fresh IMU sample
        # still carries its value but will read fresh (shared timestamp). To
        # keep an individual channel non-fresh while another is on, the value is
        # suppressed so its *_valid flag drops in guidance.UpdateRaw.
        if state == "on":
            return value
        if ts == now:
            # IMU sample is fresh because another channel is on → suppress this
            # channel's value so guidance marks it invalid (not fresh).
            return None
        return value

    return SimpleNamespace(
        health=health,
        ts=ts,
        yaw_rad=(math.radians(cycle.yaw_deg) if chan(cycle.yaw, cycle.yaw_deg) is not None else None),
        gyrz_rad_s=(math.radians(cycle.gyrz_dps) if chan(cycle.gyrz, cycle.gyrz_dps) is not None else None),
        lin_acc_x=(cycle.acc_x if chan(cycle.acc, cycle.acc_x) is not None else None),
        lin_acc_y=(cycle.acc_y if chan(cycle.acc, cycle.acc_y) is not None else None),
        lin_acc_valid=bool(chan(cycle.acc, cycle.acc_x) is not None),
    )


def make_baro(cycle: CycleInput, now: float) -> SimpleNamespace:
    health, ts, _ = _sample(cycle.baro, now, BARO_MAX_AGE)
    return SimpleNamespace(
        health=health,
        rx_ts=ts,
        alt_m=cycle.baro_alt_m,
        sink_rate=cycle.baro_sink_mps,
    )


# ── Side classification + judgement ───────────────────────────────────────────

def side_of(value: float, deadband: float = 0.0) -> str:
    if not finite(value):
        return "CENTER"
    if value > deadband:
        return "RIGHT"
    if value < -deadband:
        return "LEFT"
    return "CENTER"


def target_pointing_ok(valid: bool, target_side: str, cmd_side: str, arm_side: str) -> bool:
    if valid:
        return target_side == cmd_side == arm_side
    # FAIL / invalid: command + arm must be centred (no motion).
    return cmd_side == "CENTER" and arm_side == "CENTER"


# ── One cycle of the production pipeline ──────────────────────────────────────

CSV_COLUMNS = [
    "cycle", "t", "dt", "phase", "preset",
    "input_target_E", "input_target_N",
    "input_payload_E", "input_payload_N",
    "input_course_deg", "input_speed_mps",
    "input_yaw_deg", "input_gyrz_dps",
    "input_baro_alt_m", "input_baro_sink_mps",
    "input_gps_pos_state", "input_gps_motion_state",
    "input_yaw_state", "input_gyrz_state",
    "input_baro_state", "input_acc_state",
    "mode", "fail_reason",
    "gps_pos_fresh", "gps_motion_fresh", "imu_gyrz_fresh", "imu_yaw_fresh",
    "baro_sink_fresh", "acc_fresh",
    "nav_valid", "nav_E", "nav_N", "nav_V", "nav_course_deg", "nav_confidence",
    "dr_anchor_valid", "dr_current_valid", "dr_age_s", "dr_confidence", "dr_method",
    "dr_current_E_before_l1input", "dr_current_N_before_l1input",
    "dr_current_E_after_l1input", "dr_current_N_after_l1input",
    "dr_double_integrated_by_l1input", "dr_speed_source", "baro_sink_spike", "speed_clamped",
    "l1_valid", "l1_reason", "l1out_valid", "l1out_reason",
    "target_distance_m", "nu_deg", "abs_nu_deg", "target_bearing_deg",
    "yaw_rate_cmd_dps", "yaw_rate_limit_dps", "pid_enabled",
    "ctrl_valid", "ctrl_reason",
    "delta_ff_deg", "delta_pid_deg", "delta_arm_deg",
    "left_angle_deg", "right_angle_deg", "right_minus_left_angle_deg",
    "left_pw", "right_pw", "saturated",
    "target_side", "cmd_side", "arm_side", "target_pointing_ok",
]


def run_cycle(cycle: CycleInput, premise: Premise, now: float, dt: float,
              cycle_idx: int, phase: str, preset_name: str) -> dict[str, Any]:
    """Execute the production pipeline exactly once and return a trace row.

    The control mode is produced by guidance — never injected here.
    """
    gps = make_gps(cycle, premise, now)
    imu = make_imu(cycle, now)
    baro = make_baro(cycle, now)

    # 2. exactly one DecideControlMode call (UpdateRaw → ComputeFreshFlags → FillNav)
    mode = guidance.DecideControlMode(gps, imu, baro, now)

    # 3/5. dr.current must be unchanged by ProduceL1Input
    dr_before = (guidance._STATE_t.dr.current_E, guidance._STATE_t.dr.current_N)
    l1_in = guidance.ProduceL1Input(now)
    dr_after = (guidance._STATE_t.dr.current_E, guidance._STATE_t.dr.current_N)

    # 6. L1 output + control output / neutral
    l1_out = None
    if not l1_in.valid:
        ctrl_out = control.WriteNeutral(now, mode)
        ctrl_out.reason = l1_in.reason
    else:
        l1_out = guidance.ProduceL1Output(l1_in)
        if not l1_out.valid:
            ctrl_out = control.WriteNeutral(now, mode)
            ctrl_out.reason = l1_out.reason
        else:
            ctrl_in = control.ProduceCtrlInput(l1_out, now)
            flags_now = guidance._STATE_t.flags
            gyrz_meas_deg_s = (cycle.gyrz_dps
                               if (flags_now.imu_gyrz_fresh and finite(cycle.gyrz_dps))
                               else 0.0)
            ctrl_out = control.ProduceCtrlOutput(ctrl_in, gyrz_meas_deg_s, now)

    st = guidance._STATE_t
    nav = st.nav
    dr = st.dr
    flags = st.flags

    # derived guidance values
    target_distance_m = (l1_out.distance_to_target
                         if (l1_out is not None and finite(l1_out.distance_to_target))
                         else float("nan"))
    nu_deg = deg(l1_out.nu) if (l1_out is not None and finite(l1_out.nu)) else float("nan")
    target_bearing_deg = (deg(l1_out.target_bearing)
                          if (l1_out is not None and finite(l1_out.target_bearing))
                          else float("nan"))
    yaw_rate_cmd_dps = deg(l1_out.yaw_rate_cmd) if l1_out is not None else 0.0
    yaw_rate_limit_dps = (l1_out.yaw_rate_limit_dps if l1_out is not None
                          else float("nan"))
    pid_enabled = bool(l1_out.pid_enabled) if l1_out is not None else False

    right_minus_left = ctrl_out.right_angle_deg - ctrl_out.left_angle_deg

    l1out_valid = bool(l1_out.valid) if l1_out is not None else False
    valid_for_pointing = l1out_valid and ctrl_out.valid

    target_side = side_of(nu_deg, config.NU_DEADBAND_DEG)
    cmd_side = side_of(yaw_rate_cmd_dps, 0.0)
    arm_side = side_of(right_minus_left, 0.0)
    pointing_ok = target_pointing_ok(valid_for_pointing, target_side, cmd_side, arm_side)

    dr_age_s = now - dr.anchor_time if finite(dr.anchor_time) else float("nan")

    row: dict[str, Any] = {
        "cycle": cycle_idx,
        "t": now,
        "dt": dt,
        "phase": phase,
        "preset": preset_name,
        "input_target_E": premise.target_E,
        "input_target_N": premise.target_N,
        "input_payload_E": cycle.payload_E,
        "input_payload_N": cycle.payload_N,
        "input_course_deg": cycle.course_deg,
        "input_speed_mps": cycle.speed_mps,
        "input_yaw_deg": cycle.yaw_deg,
        "input_gyrz_dps": cycle.gyrz_dps,
        "input_baro_alt_m": cycle.baro_alt_m,
        "input_baro_sink_mps": cycle.baro_sink_mps,
        "input_gps_pos_state": cycle.gps_pos,
        "input_gps_motion_state": cycle.gps_motion,
        "input_yaw_state": cycle.yaw,
        "input_gyrz_state": cycle.gyrz,
        "input_baro_state": cycle.baro,
        "input_acc_state": cycle.acc,
        "mode": mode.value,
        "fail_reason": nav.fail_reason,
        "gps_pos_fresh": flags.gps_pos_fresh,
        "gps_motion_fresh": flags.gps_motion_fresh,
        "imu_gyrz_fresh": flags.imu_gyrz_fresh,
        "imu_yaw_fresh": flags.imu_yaw_fresh,
        "baro_sink_fresh": flags.baro_sink_fresh,
        "acc_fresh": flags.acc_fresh,
        "nav_valid": nav.valid,
        "nav_E": nav.E,
        "nav_N": nav.N,
        "nav_V": nav.V,
        "nav_course_deg": deg(nav.course),
        "nav_confidence": nav.confidence,
        "dr_anchor_valid": flags.dr_anchor_valid,
        "dr_current_valid": flags.dr_current_valid,
        "dr_age_s": dr_age_s,
        "dr_confidence": dr.confidence,
        "dr_method": dr.method.value,
        "dr_current_E_before_l1input": dr_before[0],
        "dr_current_N_before_l1input": dr_before[1],
        "dr_current_E_after_l1input": dr_after[0],
        "dr_current_N_after_l1input": dr_after[1],
        "dr_double_integrated_by_l1input": dr_after != dr_before,
        "dr_speed_source": nav.dr_speed_source,
        "baro_sink_spike": nav.baro_sink_spike,
        "speed_clamped": nav.speed_clamped,
        "l1_valid": l1_in.valid,
        "l1_reason": l1_in.reason,
        "l1out_valid": l1out_valid,
        "l1out_reason": (l1_out.reason if l1_out is not None else l1_in.reason),
        "target_distance_m": target_distance_m,
        "nu_deg": nu_deg,
        "abs_nu_deg": abs(nu_deg) if finite(nu_deg) else float("nan"),
        "target_bearing_deg": target_bearing_deg,
        "yaw_rate_cmd_dps": yaw_rate_cmd_dps,
        "yaw_rate_limit_dps": yaw_rate_limit_dps,
        "pid_enabled": pid_enabled,
        "ctrl_valid": ctrl_out.valid,
        "ctrl_reason": ctrl_out.reason,
        "delta_ff_deg": ctrl_out.delta_ff_deg,
        "delta_pid_deg": ctrl_out.delta_pid_deg,
        "delta_arm_deg": ctrl_out.delta_arm_deg,
        "left_angle_deg": ctrl_out.left_angle_deg,
        "right_angle_deg": ctrl_out.right_angle_deg,
        "right_minus_left_angle_deg": right_minus_left,
        "left_pw": ctrl_out.left_pw,
        "right_pw": ctrl_out.right_pw,
        "saturated": ctrl_out.saturated,
        "target_side": target_side,
        "cmd_side": cmd_side,
        "arm_side": arm_side,
        "target_pointing_ok": pointing_ok,
    }
    return row


# ── Presets (layer A overrides + per-cycle layer B sequence) ──────────────────

@dataclass
class Preset:
    name: str
    description: str
    premise: dict[str, Any] = field(default_factory=dict)
    cycle_fn: Callable[[int], dict[str, Any]] = field(default=lambda c: {})
    warmup_cycles: int = 0
    dt: Optional[float] = None
    cycles: Optional[int] = None


def _dropout_seq(c: int) -> dict[str, Any]:
    if c <= 2:
        return {}
    if c <= 5:
        return {"gps_motion": "off"}
    return {"gps_pos": "off", "gps_motion": "off"}


def _const(overrides: dict[str, Any]) -> Callable[[int], dict[str, Any]]:
    return lambda c: dict(overrides)


def _baro_spike(c: int) -> dict[str, Any]:
    base = {"gps_pos": "off", "gps_motion": "off"}
    if c == 5:
        base["baro_sink"] = 8.0
    return base


def _gyro_spike(c: int) -> dict[str, Any]:
    base = {"gps_pos": "off", "gps_motion": "off"}
    if c == 5:
        base["gyrz_dps"] = 150.0
    return base


PRESETS: dict[str, Preset] = {
    "GPS_CLOSED_RIGHT": Preset(
        "GPS_CLOSED_RIGHT",
        "GPS pos+motion+gyro fresh, target to the right → GPS_TRACKING_CLOSED.",
        premise={"target_E": 80.0, "target_N": 20.0},
    ),
    "GPS_CLOSED_LEFT": Preset(
        "GPS_CLOSED_LEFT",
        "Same as GPS_CLOSED_RIGHT but target to the left.",
        premise={"target_E": -80.0, "target_N": 20.0},
    ),
    "GPS_OPEN_RIGHT": Preset(
        "GPS_OPEN_RIGHT",
        "GPS fresh but gyro off → GPS_TRACKING_OPEN (pid disabled).",
        premise={"target_E": 80.0, "target_N": 20.0},
        cycle_fn=_const({"gyrz": "off"}),
    ),
    "GPS_DROPOUT_SEQUENCE_RIGHT": Preset(
        "GPS_DROPOUT_SEQUENCE_RIGHT",
        "GPS closed → motion drop (DR_M) → position drop (DR_PM).",
        premise={"target_E": 80.0, "target_N": 20.0},
        cycle_fn=_dropout_seq,
    ),
    "DR_M_GBA_RIGHT": Preset(
        "DR_M_GBA_RIGHT",
        "Warmup GPS lock, then motion drop with gyro+baro+acc → DR_M_GBA_CLOSED.",
        premise={"target_E": 80.0, "target_N": 20.0},
        warmup_cycles=DEFAULT_WARMUP_CYCLES,
        cycle_fn=_const({"gps_motion": "off"}),
    ),
    "DR_M_G_RIGHT": Preset(
        "DR_M_G_RIGHT",
        "Motion+baro+acc drop, gyro only → DR_M_G_CLOSED (SPEED_LASTV).",
        premise={"target_E": 80.0, "target_N": 20.0},
        warmup_cycles=DEFAULT_WARMUP_CYCLES,
        cycle_fn=_const({"gps_motion": "off", "baro": "off", "acc": "off"}),
    ),
    "DR_M_YBA_RIGHT": Preset(
        "DR_M_YBA_RIGHT",
        "Motion+gyro drop, yaw+baro+acc → DR_M_YBA_OPEN (pid disabled).",
        premise={"target_E": 80.0, "target_N": 20.0},
        warmup_cycles=DEFAULT_WARMUP_CYCLES,
        cycle_fn=_const({"gps_motion": "off", "gyrz": "off"}),
    ),
    "DR_PM_GBA_RIGHT": Preset(
        "DR_PM_GBA_RIGHT",
        "Position+motion drop, gyro+baro+acc → DR_PM_GBA_CLOSED (accumulates).",
        premise={"target_E": 80.0, "target_N": 20.0},
        warmup_cycles=DEFAULT_WARMUP_CYCLES,
        cycle_fn=_const({"gps_pos": "off", "gps_motion": "off"}),
    ),
    "DR_PM_G_RIGHT": Preset(
        "DR_PM_G_RIGHT",
        "Position+motion+baro+acc drop, gyro only → DR_PM_G_CLOSED (SPEED_LASTV).",
        premise={"target_E": 80.0, "target_N": 20.0},
        warmup_cycles=DEFAULT_WARMUP_CYCLES,
        cycle_fn=_const({"gps_pos": "off", "gps_motion": "off", "baro": "off", "acc": "off"}),
    ),
    "DR_PM_YBA_RIGHT": Preset(
        "DR_PM_YBA_RIGHT",
        "Position+motion+gyro drop, yaw+baro+acc → DR_PM_YBA_OPEN (pid disabled).",
        premise={"target_E": 80.0, "target_N": 20.0},
        warmup_cycles=DEFAULT_WARMUP_CYCLES,
        cycle_fn=_const({"gps_pos": "off", "gps_motion": "off", "gyrz": "off"}),
    ),
    "DR_TIMEOUT_RIGHT": Preset(
        "DR_TIMEOUT_RIGHT",
        "DR_PM that ages past DR_MAX_AGE_S → FAIL/DR_TIMEOUT (large dt).",
        premise={"target_E": 80.0, "target_N": 20.0},
        warmup_cycles=DEFAULT_WARMUP_CYCLES,
        cycle_fn=_const({"gps_pos": "off", "gps_motion": "off"}),
        dt=6.0,
        cycles=10,
    ),
    "BARO_SPIKE_RIGHT": Preset(
        "BARO_SPIKE_RIGHT",
        "DR_PM with a baro sink spike at cycle 5 → baro_sink_spike, no blow-up.",
        premise={"target_E": 80.0, "target_N": 20.0},
        warmup_cycles=DEFAULT_WARMUP_CYCLES,
        cycle_fn=_baro_spike,
    ),
    "GYRO_SPIKE_RIGHT": Preset(
        "GYRO_SPIKE_RIGHT",
        "DR_PM with a gyro spike at cycle 5 → OPEN transition, spike not in PID.",
        premise={"target_E": 80.0, "target_N": 20.0},
        warmup_cycles=DEFAULT_WARMUP_CYCLES,
        cycle_fn=_gyro_spike,
    ),
}


# ── Mission setup ─────────────────────────────────────────────────────────────

def configure_mission(premise: Premise) -> None:
    guidance.reset()
    guidance.set_origin_point(premise.origin_lat, premise.origin_lon)
    target_lat, target_lon = ne_to_latlon(premise.target_N, premise.target_E,
                                          premise.origin_lat, premise.origin_lon)
    guidance.set_target_point(target_lat, target_lon)


def _set_target(premise: Premise) -> None:
    target_lat, target_lon = ne_to_latlon(premise.target_N, premise.target_E,
                                          premise.origin_lat, premise.origin_lon)
    guidance.set_target_point(target_lat, target_lon)


def run_warmup(premise: Premise, warmup_cycles: int, dt: float, now0: float) -> float:
    """Run GPS-closed cycles to lock the DR anchor. Returns the next `now`.

    Every warmup cycle is full GPS tracking, so the anchor re-locks each cycle
    and ends fresh. Not recorded to the trace.
    """
    now = now0
    base = base_cycle_from_premise(premise)
    for _ in range(warmup_cycles):
        cyc = base.copy()  # all sensors on, premise values
        guidance.DecideControlMode(make_gps(cyc, premise, now),
                                   make_imu(cyc, now),
                                   make_baro(cyc, now),
                                   now)
        now += dt
    return now


# ── Top-level run drivers ─────────────────────────────────────────────────────

def run_preset(name: str, *, cycles: Optional[int] = None, dt: Optional[float] = None,
               sample_cycles: Optional[set[int]] = None,
               premise_overrides: Optional[dict[str, Any]] = None,
               verbose: bool = False) -> list[dict[str, Any]]:
    """Run a named preset end to end and return the trace rows.

    Precedence for dt/cycles: explicit arg > preset value > module default.
    """
    if name not in PRESETS:
        raise KeyError(f"unknown preset {name!r}; choose from {sorted(PRESETS)}")
    preset = PRESETS[name]

    premise = Premise()
    for k, v in preset.premise.items():
        setattr(premise, k, v)
    if premise_overrides:
        for k, v in premise_overrides.items():
            setattr(premise, k, v)
    premise.dt = dt if dt is not None else (preset.dt if preset.dt is not None else premise.dt)
    premise.cycles = (cycles if cycles is not None
                      else (preset.cycles if preset.cycles is not None else premise.cycles))
    premise.warmup_cycles = preset.warmup_cycles

    reload_modules()
    configure_mission(premise)

    now = premise.now0
    if preset.warmup_cycles > 0:
        now = run_warmup(premise, preset.warmup_cycles, premise.dt, now)

    base = base_cycle_from_premise(premise)
    rows: list[dict[str, Any]] = []
    for i in range(1, premise.cycles + 1):
        cyc = base.copy()
        overrides = preset.cycle_fn(i)
        apply_overrides(cyc, premise, overrides)
        _set_target(premise)  # premise.target may have been overridden
        row = run_cycle(cyc, premise, now, premise.dt, i, "main", preset.name)
        rows.append(row)
        if verbose and (sample_cycles is None or i in sample_cycles):
            print_sample(row)
        now += premise.dt
    return rows


def run_script(path: Path, premise: Premise, *, sample_cycles: Optional[set[int]] = None,
               cycles: Optional[int] = None, verbose: bool = False) -> list[dict[str, Any]]:
    """Run cycles from a scripted CSV (columns documented in the .md)."""
    reload_modules()
    configure_mission(premise)

    with path.open(newline="", encoding="utf-8") as f:
        records = list(csv.DictReader(f))
    if cycles is not None:
        records = records[:cycles]

    now = premise.now0
    base = base_cycle_from_premise(premise)
    rows: list[dict[str, Any]] = []
    prev = base.copy()
    for idx, rec in enumerate(records, start=1):
        cyc = prev.copy()
        overrides = {k: v for k, v in rec.items()
                     if k and k.lower() != "cycle" and str(v).strip() != ""}
        apply_overrides(cyc, premise, overrides)
        _set_target(premise)
        row = run_cycle(cyc, premise, now, premise.dt, idx, "main", f"SCRIPT:{path.name}")
        rows.append(row)
        if verbose and (sample_cycles is None or idx in sample_cycles):
            print_sample(row)
        prev = cyc
        now += premise.dt
    return rows


def run_interactive(premise: Premise, *, sample_cycles: Optional[set[int]] = None,
                    cycles: Optional[int] = None) -> list[dict[str, Any]]:
    """Interactive REPL: edit per-cycle inputs; blank line keeps previous."""
    reload_modules()
    configure_mission(premise)

    total = cycles if cycles is not None else premise.cycles
    now = premise.now0
    prev = base_cycle_from_premise(premise)
    rows: list[dict[str, Any]] = []
    print(f"Interactive run: {total} cycles, dt={premise.dt}, preset base values.")
    print("Enter `key=value ...` to edit; blank line keeps the previous cycle.")
    print(f"State keys: {', '.join(SENSOR_FIELDS)} = on/off/stale")
    print("Numeric keys: payload_E payload_N course_deg speed_mps yaw_deg "
          "gyrz_dps baro_alt baro_sink acc_x acc_y target_E target_N")
    for i in range(1, total + 1):
        cyc = prev.copy()
        while True:
            try:
                line = input(f"cycle {i} input: ")
            except EOFError:
                line = ""
            try:
                overrides = parse_kv_line(line)
                apply_overrides(cyc, premise, overrides)
                break
            except ValueError as exc:
                print(f"  ! {exc}; please re-enter cycle {i}.")
                cyc = prev.copy()
        _set_target(premise)
        row = run_cycle(cyc, premise, now, premise.dt, i, "main", "INTERACTIVE")
        rows.append(row)
        if sample_cycles is None or i in sample_cycles:
            print_sample(row)
        prev = cyc
        now += premise.dt
    return rows


# ── Output ────────────────────────────────────────────────────────────────────

def print_sample(row: dict[str, Any]) -> None:
    def f(key: str, fmt: str = "{:.2f}") -> str:
        v = row[key]
        return fmt.format(v) if finite(v) else "nan"

    judgement = (
        f"TARGET {row['target_side']} -> CMD {row['cmd_side']} "
        f"-> ARM {row['arm_side']}: "
        f"{'OK' if row['target_pointing_ok'] else 'FAIL'}"
    )
    print(f"[cycle {row['cycle']}]")
    print(f"mode = {row['mode']}")
    print(f"target_distance_m = {f('target_distance_m')}")
    print(f"nu_deg = {f('nu_deg')}")
    print(f"yaw_rate_cmd_dps = {f('yaw_rate_cmd_dps')}")
    print(f"right_minus_left_angle_deg = {f('right_minus_left_angle_deg')}")
    print(f"delta_arm_deg = {f('delta_arm_deg')}")
    print(f"dr_age_s = {f('dr_age_s')}")
    print(f"dr_confidence = {f('dr_confidence')}")
    print(f"target_side = {row['target_side']}")
    print(f"cmd_side = {row['cmd_side']}")
    print(f"arm_side = {row['arm_side']}")
    print(f"judgement = {judgement}")
    print()


def write_csv(rows: list[dict[str, Any]], path: Path) -> None:
    extras = sorted({k for row in rows for k in row} - set(CSV_COLUMNS))
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS + extras, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


# ── CLI ───────────────────────────────────────────────────────────────────────

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Manual sensor-cycle simulator for the parafoil guidance/control pipeline.",
    )
    p.add_argument("--origin-lat", type=float, default=DEFAULT_ORIGIN_LAT)
    p.add_argument("--origin-lon", type=float, default=DEFAULT_ORIGIN_LON)
    p.add_argument("--target-e", type=float, default=DEFAULT_TARGET_E)
    p.add_argument("--target-n", type=float, default=DEFAULT_TARGET_N)
    p.add_argument("--payload-e", type=float, default=DEFAULT_PAYLOAD_E)
    p.add_argument("--payload-n", type=float, default=DEFAULT_PAYLOAD_N)
    p.add_argument("--initial-course-deg", type=float, default=DEFAULT_COURSE_DEG)
    p.add_argument("--initial-speed-mps", type=float, default=DEFAULT_SPEED_MPS)
    p.add_argument("--initial-yaw-deg", type=float, default=DEFAULT_YAW_DEG)
    p.add_argument("--initial-gyrz-dps", type=float, default=DEFAULT_GYRZ_DPS)
    p.add_argument("--initial-baro-alt-m", type=float, default=DEFAULT_BARO_ALT_M)
    p.add_argument("--initial-baro-sink-mps", type=float, default=DEFAULT_BARO_SINK_MPS)
    p.add_argument("--dt", type=float, default=None,
                   help=f"cycle dt (s); default {DEFAULT_DT} or preset-specific")
    p.add_argument("--cycles", type=int, default=None,
                   help=f"number of cycles; default {DEFAULT_CYCLES} or preset-specific")
    p.add_argument("--sample-cycles", type=str, default=DEFAULT_SAMPLE_CYCLES)
    p.add_argument("--output", type=str, default=DEFAULT_OUTPUT)
    p.add_argument("--preset", type=str, default=DEFAULT_PRESET,
                   help=f"one of {sorted(PRESETS)}")
    p.add_argument("--now", type=float, default=DEFAULT_NOW)
    p.add_argument("--interactive", action="store_true")
    p.add_argument("--script", type=str, default=None)
    p.add_argument("--list-presets", action="store_true")
    return p


def parse_sample_cycles(text: str) -> set[int]:
    out: set[int] = set()
    for tok in text.replace(",", " ").split():
        out.add(int(tok))
    return out


def premise_from_args(args: argparse.Namespace) -> Premise:
    return Premise(
        origin_lat=args.origin_lat,
        origin_lon=args.origin_lon,
        target_E=args.target_e,
        target_N=args.target_n,
        payload_E=args.payload_e,
        payload_N=args.payload_n,
        course_deg=args.initial_course_deg,
        speed_mps=args.initial_speed_mps,
        yaw_deg=args.initial_yaw_deg,
        gyrz_dps=args.initial_gyrz_dps,
        baro_alt_m=args.initial_baro_alt_m,
        baro_sink_mps=args.initial_baro_sink_mps,
        dt=args.dt if args.dt is not None else DEFAULT_DT,
        cycles=args.cycles if args.cycles is not None else DEFAULT_CYCLES,
        now0=args.now,
    )


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)

    if args.list_presets:
        for name in sorted(PRESETS):
            print(f"{name}: {PRESETS[name].description}")
        return 0

    sample_cycles = parse_sample_cycles(args.sample_cycles)
    output_path = Path(args.output)
    if not output_path.is_absolute():
        output_path = ROOT / output_path

    if args.interactive:
        premise = premise_from_args(args)
        rows = run_interactive(premise, sample_cycles=sample_cycles, cycles=args.cycles)
    elif args.script:
        premise = premise_from_args(args)
        script_path = Path(args.script)
        if not script_path.is_absolute():
            script_path = ROOT / script_path
        rows = run_script(script_path, premise, sample_cycles=sample_cycles,
                          cycles=args.cycles, verbose=True)
    else:
        premise_overrides = {
            "origin_lat": args.origin_lat,
            "origin_lon": args.origin_lon,
            "payload_E": args.payload_e,
            "payload_N": args.payload_n,
            "course_deg": args.initial_course_deg,
            "speed_mps": args.initial_speed_mps,
            "yaw_deg": args.initial_yaw_deg,
            "gyrz_dps": args.initial_gyrz_dps,
            "baro_alt_m": args.initial_baro_alt_m,
            "baro_sink_mps": args.initial_baro_sink_mps,
            "now0": args.now,
        }
        rows = run_preset(args.preset, cycles=args.cycles, dt=args.dt,
                          sample_cycles=sample_cycles,
                          premise_overrides=premise_overrides, verbose=True)

    write_csv(rows, output_path)
    print(f"rows: {len(rows)}")
    print(f"csv: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
