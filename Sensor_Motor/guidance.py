"""Guidance module: L1 guidance and motor output computation.

Implements ArduPilot L1 navigation controller logic ported to Python.
Reference: libraries/AP_L1_Control/AP_L1_Control.cpp :: update_waypoint()

Sign convention: commanded_angular_velocity > 0 = RIGHT turn, < 0 = LEFT turn.
Units: _deg / _rad / _m / _ms / _mps suffixes throughout.
"""

from __future__ import annotations
from collections import deque
from dataclasses import dataclass, field
import enum
import math
from typing import Optional

from lib import config, timebase

EARTH_RADIUS_M = 6_371_000.0


def latlon_to_ne(
    lat: float,
    lon: float,
    origin_lat: float,
    origin_lon: float,
) -> tuple[float, float]:
    d_n = math.radians(float(lat) - float(origin_lat)) * EARTH_RADIUS_M
    d_e = (
        math.radians(float(lon) - float(origin_lon))
        * EARTH_RADIUS_M
        * math.cos(math.radians(float(origin_lat)))
    )
    return d_n, d_e


def ne_to_latlon(
    pos_n: float,
    pos_e: float,
    origin_lat: float,
    origin_lon: float,
) -> tuple[float, float]:
    lat = float(origin_lat) + math.degrees(float(pos_n) / EARTH_RADIUS_M)
    cos_lat = max(1.0e-6, abs(math.cos(math.radians(float(origin_lat)))))
    lon = float(origin_lon) + math.degrees(float(pos_e) / (EARTH_RADIUS_M * cos_lat))
    return lat, lon


def project_from_origin(
    lat: float,
    lon: float,
    origin_lat: float,
    origin_lon: float,
    max_range_m: Optional[float] = None,
) -> Optional[tuple[float, float]]:
    try:
        pos_n, pos_e = latlon_to_ne(lat, lon, origin_lat, origin_lon)
    except (TypeError, ValueError):
        return None
    values = (float(lat), float(lon), float(origin_lat), float(origin_lon), pos_n, pos_e)
    if not all(math.isfinite(v) for v in values):
        return None
    if max_range_m is not None and math.hypot(pos_n, pos_e) > float(max_range_m):
        return None
    return pos_n, pos_e


class SensorQuality(enum.Enum):
    FRESH = config.SENSOR_QUALITY_FRESH
    STALE = config.SENSOR_QUALITY_STALE

class ControlMode(enum.Enum):
    FAIL                = config.CONTROL_MODE_FAIL
    GPS_TRACKING_CLOSED = config.CONTROL_MODE_GPS_TRACKING_CLOSED
    GPS_TRACKING_OPEN   = config.CONTROL_MODE_GPS_TRACKING_OPEN
    DR_TRACKING_CLOSED  = config.CONTROL_MODE_DR_TRACKING_CLOSED
    DR_TRACKING_OPEN    = config.CONTROL_MODE_DR_TRACKING_OPEN
    DETUMBLING          = config.CONTROL_MODE_DETUMBLING


class DRMethod(enum.Enum):
    NONE                  = config.DR_METHOD_NONE
    GYRO_INTEGRATION      = config.DR_METHOD_GYRO_INTEGRATION
    ACC_DOUBLE_INTEGRATION = config.DR_METHOD_ACC_DOUBLE_INTEGRATION
    GYRO_ACC_BLEND        = config.DR_METHOD_GYRO_ACC_BLEND




@dataclass
class FreshResult:
    """Per-iteration sensor freshness snapshot tied to a single 'now' timestamp.

    decidefresh() populates this each cycle.  Nothing here decides control mode
    or computes yaw_rate_cmd — that is done downstream.
    """
    # POINT (GPS lat/lon → local EN)
    point_fresh: bool = False
    point_age_s: float = math.inf
    # VELOCITY (GPS course + speed)
    velocity_fresh: bool = False
    velocity_age_s: float = math.inf
    # IMU overall
    imu_fresh: bool = False
    imu_age_s: float = math.inf
    # IMU per-component (same timestamp, validity per channel)
    imu_gyrz_fresh: bool = False
    imu_yaw_fresh: bool = False
    imu_acc_fresh: bool = False
    imu_linear_acc_fresh: bool = False
    # BAROMETER
    barometer_fresh: bool = False
    baro_age_s: float = math.inf
    # ── backward-compat aliases ────────────────────────────────────────────────
    gyrz_fresh: bool = False   # == imu_gyrz_fresh
    gyrz_age_s: float = math.inf
    baro_fresh: bool = False   # == barometer_fresh


@dataclass
class PointSample:
    lat: float
    lon: float
    point_E: float    # math.nan when origin not set at append time
    point_N: float    # math.nan when origin not set at append time
    timestamp: float
    valid: bool = True


@dataclass
class VelocitySample:
    course: float     # rad
    speed: float      # m/s
    timestamp: float
    valid: bool = True


@dataclass
class ImuSample:
    roll: float       # rad
    pitch: float      # rad
    yaw: float        # rad
    acc_x: float      # m/s²
    acc_y: float      # m/s²
    acc_z: float      # m/s²
    gyr_x: float      # rad/s
    gyr_y: float      # rad/s
    gyr_z: float      # rad/s
    mag_x: float      # µT
    mag_y: float      # µT
    mag_z: float      # µT
    timestamp: float
    lin_acc_x: Optional[float] = None
    lin_acc_y: Optional[float] = None
    lin_acc_z: Optional[float] = None
    quat_w: Optional[float] = None
    quat_x: Optional[float] = None
    quat_y: Optional[float] = None
    quat_z: Optional[float] = None
    gyrz_valid: bool = True
    yaw_valid: bool = True
    acc_valid: bool = True
    lin_acc_valid: bool = False


@dataclass
class BarometerSample:
    altitude: float   # m
    timestamp: float
    pressure: Optional[float] = None
    valid: bool = True


@dataclass
class GuidanceState:
    """Persistent guidance state maintained across control-loop iterations."""
    # Origin — set once after release_state == 3, never changed after that
    origin_lat: Optional[float] = None
    origin_lon: Optional[float] = None
    origin_ready: bool = False
    # Target — raw lat/lon stored on receipt; E/N computed after origin is known
    target_lat: Optional[float] = None
    target_lon: Optional[float] = None
    target_N: Optional[float] = None
    target_E: Optional[float] = None
    target_ready: bool = False
    # Last fresh GPS snapshot used as DR initial condition
    last_fresh_pos_N: Optional[float] = None
    last_fresh_pos_E: Optional[float] = None
    last_fresh_pos_ts: Optional[float] = None
    last_fresh_course: Optional[float] = None
    last_fresh_speed_mps: Optional[float] = None
    last_fresh_motion_ts: Optional[float] = None
    # DR integrated state
    dr_pos_N: Optional[float] = None
    dr_pos_E: Optional[float] = None
    dr_course: Optional[float] = None
    dr_speed_mps: Optional[float] = None
    dr_method: str = config.DR_METHOD_NONE
    dr_confidence: float = 0.0
    # 8-second history queues (timestamp-pruned deques, no maxlen)
    point_history: deque = field(default_factory=deque)
    velocity_history: deque = field(default_factory=deque)
    imu_history: deque = field(default_factory=deque)
    baro_history: deque = field(default_factory=deque)
    # ── Homing-architecture nav state ─────────────────────────────────────────
    nav_E: Optional[float] = None
    nav_N: Optional[float] = None
    nav_course: Optional[float] = None
    nav_V: Optional[float] = None
    nav_vE: Optional[float] = None
    nav_vN: Optional[float] = None
    nav_confidence: float = 0.0
    nav_dr_age: float = 0.0
    nav_control_mode: str = config.CONTROL_MODE_FAIL
    nav_dr_method: str = config.DR_METHOD_NONE
    # DR anchor (saved at each GPS update; survives history pruning)
    dr_start_E: Optional[float] = None
    dr_start_N: Optional[float] = None
    dr_start_vE: Optional[float] = None
    dr_start_vN: Optional[float] = None
    dr_start_V: Optional[float] = None
    dr_start_course: Optional[float] = None
    dr_start_time: Optional[float] = None
    # GPS dropout course reference
    course_at_dropout: Optional[float] = None
    yaw_at_dropout: Optional[float] = None
    gyro_integral_since_dropout: float = 0.0
    # Unified last-valid-GPS timestamp
    last_valid_gps_time: Optional[float] = None
    # DR incremental integration timing
    last_dr_update_time: Optional[float] = None
    # DETUMBLING exit hysteresis timer
    detumble_exit_start: Optional[float] = None



@dataclass
class L1Input:
    """Unified navigation state passed from produceL1input → produceL1output.

    Field naming follows the spec convention (E/N/V) as primary.
    Legacy names (pos_E/pos_N/ground_speed_mps) are kept as backward-compat
    properties so existing callers continue to work unchanged.
    """
    valid: bool = False
    reason: str = ""
    control_mode: Optional[ControlMode] = None
    dr_method: str = config.DR_METHOD_NONE
    confidence: float = 0.0

    # ── Spec-primary nav state ─────────────────────────────────────────────────
    E: Optional[float] = None        # East position [m] in local EN frame
    N: Optional[float] = None        # North position [m] in local EN frame
    vE: Optional[float] = None       # East velocity [m/s]
    vN: Optional[float] = None       # North velocity [m/s]
    V: Optional[float] = None        # Horizontal speed [m/s]
    course: Optional[float] = None   # Course angle [rad], 0=N, +CW
    origin_E: float = 0.0            # Always 0 (origin is local-EN reference)
    origin_N: float = 0.0            # Always 0

    # ── Target ────────────────────────────────────────────────────────────────
    target_E: Optional[float] = None
    target_N: Optional[float] = None
    target_lat: Optional[float] = None   # for diag / telemetry
    target_lon: Optional[float] = None   # for diag / telemetry

    # ── Sensor ages [s] ───────────────────────────────────────────────────────
    point_age: float = math.inf
    velocity_age: float = math.inf
    imu_age: float = math.inf
    barometer_age: float = math.inf
    dr_age: float = 0.0

    # ── Backward-compat properties (read/write via E/N/V) ─────────────────────
    @property
    def pos_E(self) -> Optional[float]:
        return self.E

    @pos_E.setter
    def pos_E(self, v: Optional[float]) -> None:
        self.E = v

    @property
    def pos_N(self) -> Optional[float]:
        return self.N

    @pos_N.setter
    def pos_N(self, v: Optional[float]) -> None:
        self.N = v

    @property
    def ground_speed_mps(self) -> Optional[float]:
        return self.V

    @ground_speed_mps.setter
    def ground_speed_mps(self, v: Optional[float]) -> None:
        self.V = v

def _wrap_pi(angle_rad: float) -> float:
    return (float(angle_rad) + math.pi) % (2.0 * math.pi) - math.pi


def _try_float(v) -> Optional[float]:
    """Return float(v) if finite, else None. Absorbs None / TypeError / ValueError."""
    if v is None:
        return None
    try:
        f = float(v)
        return f if math.isfinite(f) else None
    except (TypeError, ValueError):
        return None


def _first_attr(obj, *names):
    """Return the first non-None attribute from obj among names."""
    for n in names:
        v = getattr(obj, n, None)
        if v is not None:
            return v
    return None


# ── Public helpers ─────────────────────────────────────────────────────────────

def append_history(history: deque, sample) -> None:
    """Append a sample to a history deque."""
    history.append(sample)


def prune_history(history: deque, now: float, window_s: float) -> None:
    """Remove samples older than window_s from the left end of history."""
    cutoff = float(now) - float(window_s)
    while history and history[0].timestamp < cutoff:
        history.popleft()


def wrap_pi(angle_rad: float) -> float:
    return _wrap_pi(float(angle_rad))


def clamp(x: float, lo: float, hi: float) -> float:
    return max(float(lo), min(float(hi), float(x)))


def safe_isfinite(x) -> bool:
    try:
        return math.isfinite(float(x))
    except (TypeError, ValueError):
        return False


def angle_blend(a: float, b: float, weight_b: float) -> float:
    """Interpolate between two angles [rad] with given weight on b."""
    return _wrap_pi(float(a) + float(weight_b) * _wrap_pi(float(b) - float(a)))


def saturated_sin(nu: float) -> float:
    """sign(nu) * sin(min(|nu|, π/2)) — prevents cmd reversal for large heading errors."""
    nu_f = float(nu)
    s = 1.0 if nu_f >= 0.0 else -1.0
    return s * math.sin(min(abs(nu_f), math.pi / 2.0))


def compute_dr_confidence(dr_age: float) -> float:
    """Piecewise-linear DR confidence based on time since last fresh GPS."""
    age = float(dr_age)
    if age < config.DR_CONF_AGE_1_S:
        return 1.0
    if age < config.DR_CONF_AGE_2_S:
        return 0.7
    if age < config.DR_CONF_AGE_3_S:
        return 0.4
    return 0.0


def choose_yaw_rate_limit(control_mode, dr_method=None) -> float:
    """Return yaw-rate authority limit [deg/s] for the given mode and DR method."""
    mode_val = getattr(control_mode, "value", str(control_mode))
    dm_val = (
        getattr(dr_method, "value", str(dr_method))
        if dr_method is not None
        else config.DR_METHOD_NONE
    )
    if mode_val == config.CONTROL_MODE_GPS_TRACKING_CLOSED:
        return config.GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS
    if mode_val == config.CONTROL_MODE_GPS_TRACKING_OPEN:
        return config.GPS_TRACKING_OPEN_YAW_RATE_LIMIT_DPS
    if mode_val in (config.CONTROL_MODE_DR_TRACKING_CLOSED, config.CONTROL_MODE_DR_TRACKING_OPEN):
        base = (
            config.DR_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS
            if mode_val == config.CONTROL_MODE_DR_TRACKING_CLOSED
            else config.DR_TRACKING_OPEN_YAW_RATE_LIMIT_DPS
        )
        if dm_val == config.DR_METHOD_GYRO_ACC_BLEND:
            return base * 0.85
        if dm_val == config.DR_METHOD_ACC_DOUBLE_INTEGRATION:
            return base * 0.70
        return base
    if mode_val == config.CONTROL_MODE_DETUMBLING:
        return config.DETUMBLING_YAW_RATE_LIMIT_DPS
    return config.FAIL_YAW_RATE_LIMIT_DPS


@dataclass
class L1Output:
    """Result of produceL1output — consumed by motorapp → control pipeline.

    Spec-primary fields: target_bearing, nu, distance_to_target, yaw_rate_cmd.
    Legacy fields (angular_velocity_cmd_rad_s, nu2, etc.) are kept for
    backward compatibility with control.py / motorapp.py / telemetry.
    """
    timestamp: float = 0.0
    nominal: bool = False
    control_valid: bool = False
    reason: str = config.MOTOR_REASON_INIT

    # ── Spec-primary output ────────────────────────────────────────────────────
    target_bearing: float = 0.0         # atan2(dE, dN) [rad]
    nu: float = 0.0                     # target_bearing - course [rad]
    distance_to_target: float = 0.0     # |target - pos| [m]
    yaw_rate_cmd: float = 0.0           # final yaw-rate command [rad/s]

    # ── Control authority limits (set by _homing_fill_ctrl_params) ────────────
    angular_velocity_cmd_max_deg_s: float = 0.0
    delta_ff_max_deg: float = 0.0
    delta_pid_max_deg: float = 0.0
    delta_total_max_deg: float = 0.0
    max_arm_rate_deg_s: float = 0.0
    pid_enabled: bool = False

    # ── Backward-compat / telemetry fields ────────────────────────────────────
    angular_velocity_cmd_rad_s: float = 0.0  # == yaw_rate_cmd (kept for control.py)
    ground_speed_mps: float = 0.0
    nu2: float = 0.0              # == nu (legacy alias)
    angle_to_turn: float = 0.0   # == nu (legacy alias)
    crossTrack: float = 0.0
    alongTrack: float = 0.0      # == distance_to_target
    pos_N: float = 0.0
    pos_E: float = 0.0
    target_N: float = 0.0
    target_E: float = 0.0
    carrot_N: float = 0.0
    carrot_E: float = 0.0
    target_lat: Optional[float] = None
    target_lon: Optional[float] = None
    carrot_lat: Optional[float] = None
    carrot_lon: Optional[float] = None
    current_heading_rad: float = 0.0
    yaw_rate_limit_dps: float = 0.0
    dr_confidence: float = 0.0
    dr_method: str = config.DR_METHOD_NONE
    kp_override: Optional[float] = None


def decidefresh(
    gps,
    imu,
    baro,
    state: GuidanceState,
    now: float,
) -> FreshResult:
    """Update history queues and compute per-sensor freshness flags.

    Does NOT decide control mode or compute yaw_rate_cmd.
    Stale samples are kept in history for HISTORY_WINDOW_S seconds.
    """
    result = FreshResult()
    _nan = math.nan

    # ── GPS POINT ──────────────────────────────────────────────────────────────
    gps_pos_ts = getattr(gps, "pos_ts", None) if gps is not None else None
    lat_f = _try_float(getattr(gps, "lat", None)) if gps is not None else None
    lon_f = _try_float(getattr(gps, "lon", None)) if gps is not None else None

    if gps_pos_ts is not None and lat_f is not None and lon_f is not None:
        if state.origin_ready and state.origin_lat is not None and state.origin_lon is not None:
            try:
                pt_N, pt_E = latlon_to_ne(lat_f, lon_f, state.origin_lat, state.origin_lon)
                if not (math.isfinite(pt_N) and math.isfinite(pt_E)):
                    pt_N = pt_E = _nan
            except Exception:
                pt_N = pt_E = _nan
        else:
            pt_N = pt_E = _nan
        append_history(
            state.point_history,
            PointSample(
                lat=lat_f, lon=lon_f,
                point_E=pt_E, point_N=pt_N,
                timestamp=float(gps_pos_ts),
                valid=bool(getattr(gps, "pos_health", True)),
            ),
        )

    prune_history(state.point_history, now, config.HISTORY_WINDOW_S)

    if state.point_history:
        latest = state.point_history[-1]
        age = max(0.0, now - latest.timestamp)
        result.point_age_s = age
        result.point_fresh = age <= config.GPS_FRESH_MAX_AGE_S and latest.valid
        if result.point_fresh and math.isfinite(latest.point_N) and math.isfinite(latest.point_E):
            state.last_fresh_pos_N  = latest.point_N
            state.last_fresh_pos_E  = latest.point_E
            state.last_fresh_pos_ts = latest.timestamp

    # ── GPS VELOCITY ───────────────────────────────────────────────────────────
    gps_motion_ts = getattr(gps, "motion_ts", None) if gps is not None else None
    course_f = _try_float(_first_attr(gps, "course_rad", "course")) if gps is not None else None
    speed_f  = _try_float(_first_attr(gps, "speed_mps",  "speed"))  if gps is not None else None

    if gps_motion_ts is not None and course_f is not None and speed_f is not None:
        append_history(
            state.velocity_history,
            VelocitySample(
                course=course_f, speed=speed_f,
                timestamp=float(gps_motion_ts),
                valid=bool(getattr(gps, "motion_health", True)),
            ),
        )

    prune_history(state.velocity_history, now, config.HISTORY_WINDOW_S)

    if state.velocity_history:
        latest = state.velocity_history[-1]
        age = max(0.0, now - latest.timestamp)
        result.velocity_age_s = age
        result.velocity_fresh = age <= config.GPS_FRESH_MAX_AGE_S and latest.valid
        if result.velocity_fresh:
            state.last_fresh_course    = latest.course
            state.last_fresh_speed_mps = latest.speed
            state.last_fresh_motion_ts = latest.timestamp

    # ── IMU ────────────────────────────────────────────────────────────────────
    imu_ts = getattr(imu, "ts", None) if imu is not None else None

    if imu_ts is not None and imu is not None:
        roll  = _try_float(_first_attr(imu, "roll_rad",  "roll"))
        pitch = _try_float(_first_attr(imu, "pitch_rad", "pitch"))
        yaw   = _try_float(_first_attr(imu, "yaw_rad",   "yaw"))
        ax    = _try_float(_first_attr(imu, "accx_mps2", "acc_x"))
        ay    = _try_float(_first_attr(imu, "accy_mps2", "acc_y"))
        az    = _try_float(_first_attr(imu, "accz_mps2", "acc_z"))
        gx    = _try_float(_first_attr(imu, "gyrx_rad_s", "gyrx"))
        gy    = _try_float(_first_attr(imu, "gyry_rad_s", "gyry"))
        gz    = _try_float(_first_attr(imu, "gyrz_rad_s", "gyrz"))
        mx    = _try_float(_first_attr(imu, "magx_uT", "mag_x"))
        my    = _try_float(_first_attr(imu, "magy_uT", "mag_y"))
        mz    = _try_float(_first_attr(imu, "magz_uT", "mag_z"))
        lin_ax = _try_float(_first_attr(imu, "lin_acc_x", "linear_ax"))
        lin_ay = _try_float(_first_attr(imu, "lin_acc_y", "linear_ay"))
        lin_az = _try_float(_first_attr(imu, "lin_acc_z", "linear_az"))
        imu_lin_acc_valid = bool(getattr(imu, "lin_acc_valid", False))
        lin_valid = imu_lin_acc_valid and (lin_ax is not None and lin_ay is not None and lin_az is not None)
        def _f(v): return v if v is not None else _nan
        append_history(
            state.imu_history,
            ImuSample(
                roll=_f(roll), pitch=_f(pitch), yaw=_f(yaw),
                acc_x=_f(ax), acc_y=_f(ay), acc_z=_f(az),
                gyr_x=_f(gx), gyr_y=_f(gy), gyr_z=_f(gz),
                mag_x=_f(mx), mag_y=_f(my), mag_z=_f(mz),
                timestamp=float(imu_ts),
                gyrz_valid=gz is not None,
                yaw_valid=yaw is not None,
                acc_valid=(ax is not None and ay is not None and az is not None),
                lin_acc_x=lin_ax,
                lin_acc_y=lin_ay,
                lin_acc_z=lin_az,
                lin_acc_valid=lin_valid,
            ),
        )

    prune_history(state.imu_history, now, config.HISTORY_WINDOW_S)

    if state.imu_history:
        latest = state.imu_history[-1]
        age = max(0.0, now - latest.timestamp)
        result.imu_age_s         = age
        result.imu_fresh         = age <= config.IMU_FRESH_MAX_AGE_S
        result.imu_gyrz_fresh    = result.imu_fresh and latest.gyrz_valid
        result.imu_yaw_fresh     = result.imu_fresh and latest.yaw_valid
        result.imu_acc_fresh     = result.imu_fresh and latest.acc_valid
        result.imu_linear_acc_fresh = result.imu_fresh and latest.lin_acc_valid
        result.gyrz_fresh        = result.imu_gyrz_fresh   # backward compat
        result.gyrz_age_s        = age                     # backward compat

    # ── BAROMETER ──────────────────────────────────────────────────────────────
    baro_ts  = getattr(baro, "ts",    None) if baro is not None else None
    alt_f    = _try_float(_first_attr(baro, "alt_m", "alt")) if baro is not None else None

    if baro_ts is not None and alt_f is not None:
        append_history(
            state.baro_history,
            BarometerSample(altitude=alt_f, timestamp=float(baro_ts)),
        )

    prune_history(state.baro_history, now, config.HISTORY_WINDOW_S)

    if state.baro_history:
        latest = state.baro_history[-1]
        age = max(0.0, now - latest.timestamp)
        result.baro_age_s      = age
        result.barometer_fresh = age <= config.BARO_FRESH_MAX_AGE_S and latest.valid
        result.baro_fresh      = result.barometer_fresh   # backward compat

    return result


# ── Origin / target / local-EN helpers ────────────────────────────────────────

def convert_latlon_to_local_en(
    lat: float,
    lon: float,
    origin_lat: float,
    origin_lon: float,
) -> tuple[float, float]:
    """Return local (E, N) [m] relative to origin.

    Convention: N = North [m], E = East [m].
    Bearing from North clockwise: target_bearing = atan2(dE, dN).
    """
    N, E = latlon_to_ne(lat, lon, origin_lat, origin_lon)
    return E, N


def convert_target_to_local_en_if_possible(state: GuidanceState) -> bool:
    """Project state.target_lat/lon to state.target_E/N if origin is ready.

    Idempotent — safe to call every cycle; only re-projects when target_ready
    is False (e.g. target arrived before origin was set).
    Returns True if target_ready afterwards.
    """
    if state.target_ready:
        return True
    if not state.origin_ready:
        return False
    t_lat = _try_float(state.target_lat)
    t_lon = _try_float(state.target_lon)
    if t_lat is None or t_lon is None:
        return False
    try:
        E, N = convert_latlon_to_local_en(t_lat, t_lon, state.origin_lat, state.origin_lon)
    except Exception:
        return False
    if not (math.isfinite(E) and math.isfinite(N)):
        return False
    state.target_E = E
    state.target_N = N
    state.target_ready = True
    return True


def gps_point_to_en_if_origin_ready(
    state: GuidanceState,
    lat: float,
    lon: float,
) -> Optional[tuple[float, float]]:
    """Convert GPS lat/lon to local (E, N) [m].  Returns None if origin not set."""
    if not state.origin_ready or state.origin_lat is None or state.origin_lon is None:
        return None
    lat_f = _try_float(lat)
    lon_f = _try_float(lon)
    if lat_f is None or lon_f is None:
        return None
    try:
        E, N = convert_latlon_to_local_en(lat_f, lon_f, state.origin_lat, state.origin_lon)
    except Exception:
        return None
    if not (math.isfinite(E) and math.isfinite(N)):
        return None
    return E, N


def set_origin_from_current_gps(
    state: GuidanceState,
    sample: PointSample,
) -> bool:
    """Set origin from sample.lat/lon if not yet set.  Call only when
    release_state == 3 and fresh.point_fresh is True.

    After origin is set:
    - all existing point_history samples are backfilled with their E/N.
    - last_fresh_pos_N/E is updated from the most-recent history sample.
    - convert_target_to_local_en_if_possible() is called.

    Returns True on the call that actually set the origin, False otherwise.
    """
    if state.origin_ready:
        return False   # immutable once set
    lat_f = _try_float(sample.lat)
    lon_f = _try_float(sample.lon)
    if lat_f is None or lon_f is None:
        return False

    state.origin_lat   = lat_f
    state.origin_lon   = lon_f
    state.origin_ready = True

    # Backfill E/N for every history sample that has valid lat/lon
    for s in state.point_history:
        result = gps_point_to_en_if_origin_ready(state, s.lat, s.lon)
        if result is not None:
            s.point_E = result[0]
            s.point_N = result[1]

    # Update DR initial condition from the freshest history sample with valid EN
    for s in reversed(state.point_history):
        if math.isfinite(s.point_E) and math.isfinite(s.point_N):
            state.last_fresh_pos_E  = s.point_E
            state.last_fresh_pos_N  = s.point_N
            state.last_fresh_pos_ts = s.timestamp
            break

    convert_target_to_local_en_if_possible(state)
    return True


# ═══════════════════════════════════════════════════════════════════════════════
# HOMING GNC ARCHITECTURE — STEPS 4-7
# produceL1input → produceL1output → (motorapp applies cmd to control.py)
# ═══════════════════════════════════════════════════════════════════════════════

def _fail_l1input(reason: str) -> L1Input:
    out = L1Input()
    out.valid = False
    out.control_mode = ControlMode.FAIL
    out.reason = reason
    return out


# ── STEP 7: DETUMBLE check ────────────────────────────────────────────────────

def should_detumble(fresh: FreshResult, imu, state: GuidanceState, now: float) -> bool:
    """Return True if DETUMBLING mode should be active this cycle."""
    if not config.DETUMBLE_ENABLE:
        return False
    if not fresh.imu_gyrz_fresh or imu is None:
        return False
    gz = _try_float(_first_attr(imu, "gyrz_rad_s", "gyrz"))
    if gz is None:
        return False
    gyrz_dps = abs(math.degrees(float(config.GYRZ_SIGN) * gz))

    if gyrz_dps > config.DETUMBLE_GYRZ_THRESHOLD_DPS:
        state.detumble_exit_start = None
        return True

    was_detumbling = state.nav_control_mode == config.CONTROL_MODE_DETUMBLING
    if not was_detumbling:
        state.detumble_exit_start = None
        return False

    # Was detumbling — apply exit hysteresis
    if gyrz_dps <= config.DETUMBLE_EXIT_THRESHOLD_DPS:
        if state.detumble_exit_start is None:
            state.detumble_exit_start = now
        if now - state.detumble_exit_start >= config.DETUMBLE_EXIT_HOLD_S:
            state.detumble_exit_start = None
            return False
        return True

    # Between entry and exit thresholds — keep detumbling
    state.detumble_exit_start = None
    return True


# ── STEP 5: DR feasibility ────────────────────────────────────────────────────

def can_dead_reckon(fresh: FreshResult, state: GuidanceState) -> bool:
    """Return True if DR is possible with available sensors and saved GPS anchor."""
    has_anchor = (
        state.dr_start_E is not None
        and state.dr_start_N is not None
        and state.dr_start_V is not None
        and state.dr_start_course is not None
        and state.dr_start_time is not None
    )
    has_course_source = fresh.imu_yaw_fresh or fresh.imu_gyrz_fresh
    return has_anchor and has_course_source


# ── STEP 4: GPS tracking state update ─────────────────────────────────────────

def update_state_from_gps_tracking(
    state: GuidanceState,
    gps,
    imu,
    fresh: FreshResult,
    now: float,
) -> None:
    """Update nav state from fresh GPS. Also seeds the DR anchor for future use."""
    latest_pt = state.point_history[-1]
    latest_vel = state.velocity_history[-1]

    state.nav_E = latest_pt.point_E
    state.nav_N = latest_pt.point_N
    state.nav_course = latest_vel.course
    state.nav_V = latest_vel.speed

    V = float(latest_vel.speed)
    c = float(latest_vel.course)
    state.nav_vE = V * math.sin(c)
    state.nav_vN = V * math.cos(c)

    gps_time = max(
        latest_pt.timestamp,
        latest_vel.timestamp,
    )
    state.last_valid_gps_time = gps_time

    # Save DR anchor for potential future dropout
    state.dr_start_E = state.nav_E
    state.dr_start_N = state.nav_N
    state.dr_start_vE = state.nav_vE
    state.dr_start_vN = state.nav_vN
    state.dr_start_V = state.nav_V
    state.dr_start_course = state.nav_course
    state.dr_start_time = gps_time

    # Reset dropout tracking
    state.course_at_dropout = state.nav_course
    imu_yaw = _try_float(_first_attr(imu, "yaw_rad", "yaw")) if imu is not None else None
    state.yaw_at_dropout = imu_yaw
    state.gyro_integral_since_dropout = 0.0

    state.last_dr_update_time = now
    state.nav_dr_age = 0.0
    state.nav_confidence = 1.0
    state.nav_dr_method = config.DR_METHOD_NONE


# ── STEP 5: DR course estimation ──────────────────────────────────────────────

def _estimate_course_dr(
    state: GuidanceState,
    imu,
    fresh: FreshResult,
    dt: float,
) -> Optional[tuple]:
    """Estimate current course from IMU during GPS dropout.

    Returns (course_rad, course_source_str) or None on failure.
    """
    if state.course_at_dropout is None:
        return None

    course_dropout = float(state.course_at_dropout)
    course_from_yaw: Optional[float] = None
    course_from_gyro: Optional[float] = None

    if fresh.imu_yaw_fresh and imu is not None:
        imu_yaw = _try_float(_first_attr(imu, "yaw_rad", "yaw"))
        if imu_yaw is not None and state.yaw_at_dropout is not None:
            course_from_yaw = _wrap_pi(
                course_dropout + _wrap_pi(float(imu_yaw) - float(state.yaw_at_dropout))
            )

    if fresh.imu_gyrz_fresh and imu is not None:
        gz = _try_float(_first_attr(imu, "gyrz_rad_s", "gyrz"))
        if gz is not None:
            state.gyro_integral_since_dropout += float(config.GYRZ_SIGN) * float(gz) * dt
            course_from_gyro = _wrap_pi(course_dropout + state.gyro_integral_since_dropout)

    if course_from_yaw is not None and course_from_gyro is not None:
        residual = abs(_wrap_pi(course_from_yaw - course_from_gyro))
        if residual < math.radians(5.0):
            return course_from_yaw, "YAW_DELTA"
        if residual < math.radians(15.0):
            return angle_blend(course_from_gyro, course_from_yaw, 0.7), "YAW_GYRO_BLEND"
        return course_from_gyro, "GYRO_FALLBACK"

    if course_from_yaw is not None:
        return course_from_yaw, "YAW_DELTA"
    if course_from_gyro is not None:
        return course_from_gyro, "GYRO_FALLBACK"
    return None


def _quat_rotate(qw: float, qx: float, qy: float, qz: float,
                 vx: float, vy: float, vz: float) -> tuple:
    """Rotate vector (vx, vy, vz) by unit quaternion (qw, qx, qy, qz)."""
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    wx = vx + qw * tx + qy * tz - qz * ty
    wy = vy + qw * ty + qz * tx - qx * tz
    wz = vz + qw * tz + qx * ty - qy * tx
    return wx, wy, wz


def _acc_body_to_en_with_quaternion(
    ax: float, ay: float, az: float,
    qw: float, qx: float, qy: float, qz: float,
) -> tuple:
    """Transform linear body acceleration to EN frame using quaternion.

    Assumes BNO085 convention: body x=forward, y=right, z=down → NED world.
    Returns (aE, aN).
    """
    ax_s = float(config.ACC_X_SIGN) * float(ax)
    ay_s = float(config.ACC_Y_SIGN) * float(ay)
    world_n, world_e, _ = _quat_rotate(float(qw), float(qx), float(qy), float(qz),
                                       ax_s, ay_s, float(az))
    return float(world_e), float(world_n)


def _acc_body_to_en_with_yaw(ax: float, ay: float, yaw_rad: float) -> tuple:
    """Transform body acceleration to EN frame using heading angle.

    Returns (aE, aN).
    """
    ax_s = float(config.ACC_X_SIGN) * float(ax)
    ay_s = float(config.ACC_Y_SIGN) * float(ay)
    c = math.cos(float(yaw_rad))
    s = math.sin(float(yaw_rad))
    aN = ax_s * c - ay_s * s
    aE = ax_s * s + ay_s * c
    return aE, aN


# ── STEP 5: DR state update ────────────────────────────────────────────────────

def update_state_from_dead_reckoning(
    state: GuidanceState,
    imu,
    fresh: FreshResult,
    now: float,
) -> None:
    """Integrate nav state forward using IMU when GPS is stale."""
    last_t = state.last_dr_update_time if state.last_dr_update_time is not None else now
    dt = clamp(now - last_t, 0.005, 0.5)
    state.last_dr_update_time = now

    course_result = _estimate_course_dr(state, imu, fresh, dt)
    if course_result is None:
        state.nav_confidence = 0.0
        return

    est_course, _ = course_result
    state.nav_course = est_course

    # Speed decay (exponential)
    V_prev = state.nav_V if state.nav_V is not None else (state.dr_start_V or 0.0)
    tau = float(config.SPEED_DECAY_TAU_S)
    V_decayed = float(V_prev) * math.exp(-dt / tau) if tau > 0.0 else float(V_prev)
    V_decayed = clamp(V_decayed, float(config.V_MIN_MPS), float(config.V_MAX_MPS))

    vE_gyro = V_decayed * math.sin(est_course)
    vN_gyro = V_decayed * math.cos(est_course)

    E_prev = state.nav_E if state.nav_E is not None else (state.dr_start_E or 0.0)
    N_prev = state.nav_N if state.nav_N is not None else (state.dr_start_N or 0.0)
    E_gyro = float(E_prev) + vE_gyro * dt
    N_gyro = float(N_prev) + vN_gyro * dt

    acc_available = False
    E_out, N_out = E_gyro, N_gyro
    vE_out, vN_out = vE_gyro, vN_gyro

    # Acc-aided DR (optional)
    if (
        config.USE_ACC_DOUBLE_INTEGRATION
        and fresh.imu_linear_acc_fresh
        and imu is not None
        and state.nav_dr_age <= float(config.ACC_AID_END_AGE_S)
        and state.nav_dr_age >= float(config.ACC_AID_START_AGE_S)
    ):
        lin_ax = _try_float(_first_attr(imu, "lin_acc_x", "linear_ax"))
        lin_ay = _try_float(_first_attr(imu, "lin_acc_y", "linear_ay"))
        if lin_ax is not None and lin_ay is not None:
            # Priority 1: quaternion rotation
            qw = _try_float(getattr(imu, "quat_w", None))
            qx = _try_float(getattr(imu, "quat_x", None))
            qy = _try_float(getattr(imu, "quat_y", None))
            qz = _try_float(getattr(imu, "quat_z", None))
            lin_az = _try_float(_first_attr(imu, "lin_acc_z", "linear_az")) or 0.0

            aE_raw: Optional[float] = None
            aN_raw: Optional[float] = None
            if qw is not None and qx is not None and qy is not None and qz is not None:
                aE_raw, aN_raw = _acc_body_to_en_with_quaternion(
                    lin_ax, lin_ay, lin_az, qw, qx, qy, qz
                )
            else:
                imu_yaw = _try_float(_first_attr(imu, "yaw_rad", "yaw"))
                if imu_yaw is not None:
                    aE_raw, aN_raw = _acc_body_to_en_with_yaw(lin_ax, lin_ay, imu_yaw)

            if aE_raw is not None and aN_raw is not None:
                aE_lim = float(config.ACC_LIMIT_MPS2)
                aE = clamp(aE_raw, -aE_lim, aE_lim)
                aN = clamp(aN_raw, -aE_lim, aE_lim)
                vE_acc = float(state.nav_vE or vE_gyro) + aE * dt
                vN_acc = float(state.nav_vN or vN_gyro) + aN * dt
                E_acc = float(E_prev) + float(state.nav_vE or vE_gyro) * dt + 0.5 * aE * dt * dt
                N_acc = float(N_prev) + float(state.nav_vN or vN_gyro) * dt + 0.5 * aN * dt * dt
                w = float(config.ACC_BLEND_WEIGHT)
                E_out = (1.0 - w) * E_gyro + w * E_acc
                N_out = (1.0 - w) * N_gyro + w * N_acc
                vE_out = (1.0 - w) * vE_gyro + w * vE_acc
                vN_out = (1.0 - w) * vN_gyro + w * vN_acc
                acc_available = True

    state.nav_E = E_out
    state.nav_N = N_out
    state.nav_vE = vE_out
    state.nav_vN = vN_out
    state.nav_V = math.hypot(vE_out, vN_out)

    # DR confidence by age
    if state.last_valid_gps_time is not None:
        state.nav_dr_age = max(0.0, now - float(state.last_valid_gps_time))
    state.nav_confidence = compute_dr_confidence(state.nav_dr_age)

    state.nav_dr_method = (
        config.DR_METHOD_GYRO_ACC_BLEND if acc_available else config.DR_METHOD_GYRO_INTEGRATION
    )


# ── STEP 4: produceL1input ────────────────────────────────────────────────────

def produceL1input(
    fresh: FreshResult,
    gps,
    imu,
    state: GuidanceState,
    release_state: int,
    now: float,
) -> L1Input:
    """Decide control mode, update nav state, and return L1Input for homing.

    Call decidefresh() first each cycle to populate fresh and state histories.
    Does not compute yaw_rate_cmd — that is done by produceL1output().
    """
    # ── Gate: wait for STATE >= 3 ────────────────────────────────────────────
    if release_state < 3:
        return _fail_l1input("WAIT_RELEASE_STATE_3")

    # ── Origin ───────────────────────────────────────────────────────────────
    if not state.origin_ready:
        if fresh.point_fresh and state.point_history:
            set_origin_from_current_gps(state, state.point_history[-1])
        if not state.origin_ready:
            return _fail_l1input("FAIL_NO_ORIGIN")

    # Recalculate E/N in history samples after origin is set
    # (backfill already done by set_origin_from_current_gps)

    # ── Target ───────────────────────────────────────────────────────────────
    convert_target_to_local_en_if_possible(state)
    if not state.target_ready:
        return _fail_l1input("FAIL_NO_TARGET")

    # ── DETUMBLING ───────────────────────────────────────────────────────────
    if should_detumble(fresh, imu, state, now):
        state.nav_control_mode = config.CONTROL_MODE_DETUMBLING
        state.nav_dr_method = config.DR_METHOD_NONE
        out = L1Input()
        out.valid = True
        out.control_mode = ControlMode.DETUMBLING
        out.dr_method = config.DR_METHOD_NONE
        out.confidence = 1.0
        # Spec-primary fields
        out.E = state.nav_E
        out.N = state.nav_N
        out.V = state.nav_V
        out.vE = state.nav_vE
        out.vN = state.nav_vN
        out.course = state.nav_course
        out.origin_E = 0.0
        out.origin_N = 0.0
        out.target_E = state.target_E
        out.target_N = state.target_N
        out.target_lat = state.target_lat
        out.target_lon = state.target_lon
        out.point_age = fresh.point_age_s
        out.velocity_age = fresh.velocity_age_s
        out.imu_age = fresh.imu_age_s
        out.barometer_age = fresh.baro_age_s
        out.dr_age = state.nav_dr_age
        return out

    # ── GPS TRACKING ─────────────────────────────────────────────────────────
    if fresh.point_fresh and fresh.velocity_fresh and state.point_history and state.velocity_history:
        latest_pt = state.point_history[-1]
        if not (math.isfinite(latest_pt.point_E) and math.isfinite(latest_pt.point_N)):
            # Point in history without valid EN coords — try to re-project
            res = gps_point_to_en_if_origin_ready(state, latest_pt.lat, latest_pt.lon)
            if res is not None:
                latest_pt.point_E, latest_pt.point_N = res[0], res[1]
            else:
                return _fail_l1input("FAIL_NO_POSITION_EN")

        update_state_from_gps_tracking(state, gps, imu, fresh, now)

        state.nav_confidence = 1.0
        state.nav_dr_age = 0.0
        state.nav_dr_method = config.DR_METHOD_NONE

        if fresh.imu_gyrz_fresh:
            state.nav_control_mode = config.CONTROL_MODE_GPS_TRACKING_CLOSED
            mode = ControlMode.GPS_TRACKING_CLOSED
        else:
            state.nav_control_mode = config.CONTROL_MODE_GPS_TRACKING_OPEN
            mode = ControlMode.GPS_TRACKING_OPEN

    # ── DEAD RECKONING ────────────────────────────────────────────────────────
    else:
        if not can_dead_reckon(fresh, state):
            state.nav_control_mode = config.CONTROL_MODE_FAIL
            return _fail_l1input("FAIL_NO_VALID_DR")

        update_state_from_dead_reckoning(state, imu, fresh, now)

        if state.nav_confidence <= 0.0:
            state.nav_control_mode = config.CONTROL_MODE_FAIL
            return _fail_l1input("FAIL_DR_CONFIDENCE_ZERO")

        if fresh.imu_gyrz_fresh:
            state.nav_control_mode = config.CONTROL_MODE_DR_TRACKING_CLOSED
            mode = ControlMode.DR_TRACKING_CLOSED
        else:
            state.nav_control_mode = config.CONTROL_MODE_DR_TRACKING_OPEN
            mode = ControlMode.DR_TRACKING_OPEN

    # ── Build L1Input ─────────────────────────────────────────────────────────
    out = L1Input()
    out.valid = True
    out.control_mode = mode
    out.dr_method = state.nav_dr_method
    out.confidence = state.nav_confidence

    # Spec-primary nav state (E/N/V/vE/vN)
    out.E = state.nav_E
    out.N = state.nav_N
    out.V = state.nav_V
    out.vE = state.nav_vE
    out.vN = state.nav_vN
    out.course = state.nav_course

    # Origin is always the local-EN reference point → (0, 0)
    out.origin_E = 0.0
    out.origin_N = 0.0

    # Target
    out.target_E = state.target_E
    out.target_N = state.target_N
    out.target_lat = state.target_lat
    out.target_lon = state.target_lon

    # Sensor ages from the most recent FreshResult
    out.point_age = fresh.point_age_s
    out.velocity_age = fresh.velocity_age_s
    out.imu_age = fresh.imu_age_s
    out.barometer_age = fresh.baro_age_s
    out.dr_age = state.nav_dr_age

    return out


# ── STEP 6: produceL1output ────────────────────────────────────────────────────

def produceL1output(l1input: L1Input) -> L1Output:
    """Compute yaw_rate_cmd from L1Input.  Stores result in angular_velocity_cmd_rad_s.

    Uses fixed-target homing: the target itself is the carrot.
    Does not read sensor cache; consumes only l1input fields.
    """
    out = L1Output(timestamp=0.0)

    if not l1input.valid:
        reason = getattr(l1input, "reason", "FAIL")
        out.reason = reason if reason else "FAIL"
        out.control_valid = False
        return out

    mode = l1input.control_mode
    mode_val = getattr(mode, "value", str(mode))

    # ── FAIL ─────────────────────────────────────────────────────────────────
    if mode_val == config.CONTROL_MODE_FAIL or mode == ControlMode.FAIL:
        out.reason = getattr(l1input, "reason", config.CONTROL_MODE_FAIL) or config.CONTROL_MODE_FAIL
        out.control_valid = False
        return out

    # ── DETUMBLING: yaw_rate_cmd=0, PID counters the spin ────────────────────
    if mode_val == config.CONTROL_MODE_DETUMBLING or mode == ControlMode.DETUMBLING:
        out.reason = config.CONTROL_MODE_DETUMBLING
        out.control_valid = True
        out.nominal = True
        # Spec §10: yaw_rate_cmd = 0; yaw_rate_error = 0 - gyrz is handled by PID
        out.yaw_rate_cmd = 0.0
        out.angular_velocity_cmd_rad_s = 0.0   # backward compat
        out.nu = 0.0
        out.target_bearing = 0.0
        out.distance_to_target = 0.0
        out.pid_enabled = True
        out.angular_velocity_cmd_max_deg_s = 0.0
        out.delta_ff_max_deg = 0.0
        out.delta_pid_max_deg = config.MOTOR_NOMINAL_CLOSED_LOOP_DELTA_PID_MAX_DEG
        out.delta_total_max_deg = config.MOTOR_NOMINAL_CLOSED_LOOP_DELTA_TOTAL_MAX_DEG
        out.max_arm_rate_deg_s = config.MOTOR_NOMINAL_CLOSED_LOOP_MAX_ARM_RATE_DEG_S
        out.yaw_rate_limit_dps = config.DETUMBLING_YAW_RATE_LIMIT_DPS
        out.dr_confidence = l1input.confidence
        out.dr_method = l1input.dr_method
        out.kp_override = config.KP_DETUMBLE
        return out

    # ── Validate nav state ────────────────────────────────────────────────────
    pos_E = l1input.pos_E
    pos_N = l1input.pos_N
    tgt_E = l1input.target_E
    tgt_N = l1input.target_N
    course = l1input.course
    V_raw = l1input.ground_speed_mps

    if None in (pos_E, pos_N, tgt_E, tgt_N, course, V_raw):
        out.reason = "FAIL_NO_NAV_STATE"
        out.control_valid = False
        return out

    # ── Distance to target ────────────────────────────────────────────────────
    dE = float(tgt_E) - float(pos_E)
    dN = float(tgt_N) - float(pos_N)
    distance = math.hypot(dE, dN)
    out.alongTrack = distance
    out.crossTrack = 0.0

    if distance <= float(config.TARGET_RADIUS_M):
        out.reason = "TARGET_REACHED"
        out.control_valid = True
        out.nominal = True
        out.yaw_rate_cmd = 0.0
        out.angular_velocity_cmd_rad_s = 0.0   # backward compat
        out.nu = 0.0
        out.target_bearing = _wrap_pi(math.atan2(dE, dN))
        out.distance_to_target = distance
        out.alongTrack = distance
        out.pos_N = float(pos_N)
        out.pos_E = float(pos_E)
        out.target_N = float(tgt_N)
        out.target_E = float(tgt_E)
        out.current_heading_rad = float(course)
        _homing_fill_ctrl_params(out, mode_val, l1input.dr_method)
        return out

    # ── Target bearing and L1 guidance ────────────────────────────────────────
    target_bearing = _wrap_pi(math.atan2(dE, dN))
    nu = _wrap_pi(target_bearing - float(course))
    sin_nu_eff = saturated_sin(nu)

    V = clamp(float(V_raw), float(config.V_MIN_MPS), float(config.V_MAX_MPS))
    L = float(config.L_GAIN_M)
    yaw_rate_cmd = 2.0 * V / L * sin_nu_eff

    yaw_rate_cmd *= float(l1input.confidence)

    yaw_rate_limit_dps = choose_yaw_rate_limit(mode, l1input.dr_method)
    yaw_rate_limit_rad_s = math.radians(yaw_rate_limit_dps)
    yaw_rate_cmd = clamp(yaw_rate_cmd, -yaw_rate_limit_rad_s, yaw_rate_limit_rad_s)

    out.reason = mode_val
    out.control_valid = True
    out.nominal = True

    # ── Spec-primary output fields ──────────────────────────────────────────
    out.target_bearing = target_bearing           # atan2(dE,dN) [rad]
    out.nu = nu                                   # target_bearing - course [rad]
    out.distance_to_target = distance             # [m]
    out.yaw_rate_cmd = yaw_rate_cmd               # [rad/s]

    # ── Backward-compat / telemetry ─────────────────────────────────────────
    out.angular_velocity_cmd_rad_s = yaw_rate_cmd  # control.py reads this
    out.nu2 = nu
    out.angle_to_turn = nu
    out.alongTrack = distance
    out.crossTrack = 0.0
    out.ground_speed_mps = float(V_raw)
    out.pos_N = float(pos_N)
    out.pos_E = float(pos_E)
    out.target_N = float(tgt_N)
    out.target_E = float(tgt_E)
    out.carrot_N = float(tgt_N)   # target itself is carrot (no path projection)
    out.carrot_E = float(tgt_E)
    out.current_heading_rad = float(course)
    out.yaw_rate_limit_dps = yaw_rate_limit_dps
    out.dr_confidence = float(l1input.confidence)
    out.dr_method = str(l1input.dr_method)

    _homing_fill_ctrl_params(out, mode_val, l1input.dr_method)
    return out


def _homing_fill_ctrl_params(out: L1Output, mode_val: str, dr_method: str) -> None:
    """Set motor control authority limits on L1Output by mode."""
    c = config
    if mode_val == c.CONTROL_MODE_GPS_TRACKING_CLOSED:
        out.pid_enabled = True
        out.angular_velocity_cmd_max_deg_s = c.GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS
        out.delta_ff_max_deg = c.MOTOR_NOMINAL_CLOSED_LOOP_DELTA_FF_MAX_DEG
        out.delta_pid_max_deg = c.MOTOR_NOMINAL_CLOSED_LOOP_DELTA_PID_MAX_DEG
        out.delta_total_max_deg = c.MOTOR_NOMINAL_CLOSED_LOOP_DELTA_TOTAL_MAX_DEG
        out.max_arm_rate_deg_s = c.MOTOR_NOMINAL_CLOSED_LOOP_MAX_ARM_RATE_DEG_S
    elif mode_val == c.CONTROL_MODE_GPS_TRACKING_OPEN:
        out.pid_enabled = False
        out.angular_velocity_cmd_max_deg_s = c.GPS_TRACKING_OPEN_YAW_RATE_LIMIT_DPS
        out.delta_ff_max_deg = c.MOTOR_NOMINAL_FEEDFORWARD_DELTA_FF_MAX_DEG
        out.delta_pid_max_deg = 0.0
        out.delta_total_max_deg = c.MOTOR_NOMINAL_FEEDFORWARD_DELTA_TOTAL_MAX_DEG
        out.max_arm_rate_deg_s = c.MOTOR_NOMINAL_FEEDFORWARD_MAX_ARM_RATE_DEG_S
    elif mode_val == c.CONTROL_MODE_DR_TRACKING_CLOSED:
        out.pid_enabled = True
        out.angular_velocity_cmd_max_deg_s = c.DR_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS
        out.delta_ff_max_deg = c.MOTOR_DEGRADED_CLOSED_LOOP_DELTA_FF_MAX_DEG
        out.delta_pid_max_deg = c.MOTOR_DEGRADED_CLOSED_LOOP_DELTA_PID_MAX_DEG
        out.delta_total_max_deg = c.MOTOR_DEGRADED_CLOSED_LOOP_DELTA_TOTAL_MAX_DEG
        out.max_arm_rate_deg_s = c.MOTOR_DEGRADED_CLOSED_LOOP_MAX_ARM_RATE_DEG_S
    elif mode_val == c.CONTROL_MODE_DR_TRACKING_OPEN:
        out.pid_enabled = False
        out.angular_velocity_cmd_max_deg_s = c.DR_TRACKING_OPEN_YAW_RATE_LIMIT_DPS
        out.delta_ff_max_deg = c.MOTOR_DEGRADED_FEEDFORWARD_DELTA_FF_MAX_DEG
        out.delta_pid_max_deg = 0.0
        out.delta_total_max_deg = c.MOTOR_DEGRADED_FEEDFORWARD_DELTA_TOTAL_MAX_DEG
        out.max_arm_rate_deg_s = c.MOTOR_DEGRADED_FEEDFORWARD_MAX_ARM_RATE_DEG_S
    else:
        out.pid_enabled = False
        out.angular_velocity_cmd_max_deg_s = 0.0
        out.delta_ff_max_deg = 0.0
        out.delta_pid_max_deg = 0.0
        out.delta_total_max_deg = 0.0
        out.max_arm_rate_deg_s = 0.0


def reset_guidance_state_for_flight(state: GuidanceState) -> None:
    """Reset origin and nav state when transitioning back to pre-release state."""
    state.origin_ready = False
    state.origin_lat = None
    state.origin_lon = None
    state.target_ready = False
    state.target_E = None
    state.target_N = None
    state.nav_E = None
    state.nav_N = None
    state.nav_course = None
    state.nav_V = None
    state.nav_vE = None
    state.nav_vN = None
    state.nav_confidence = 0.0
    state.nav_dr_age = 0.0
    state.nav_control_mode = config.CONTROL_MODE_FAIL
    state.nav_dr_method = config.DR_METHOD_NONE
    state.dr_start_E = None
    state.dr_start_N = None
    state.dr_start_vE = None
    state.dr_start_vN = None
    state.dr_start_V = None
    state.dr_start_course = None
    state.dr_start_time = None
    state.course_at_dropout = None
    state.yaw_at_dropout = None
    state.gyro_integral_since_dropout = 0.0
    state.last_valid_gps_time = None
    state.last_dr_update_time = None
    state.detumble_exit_start = None
    state.point_history.clear()
    state.velocity_history.clear()
    state.imu_history.clear()
    state.baro_history.clear()
