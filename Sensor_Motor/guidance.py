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

#Sensor age
POS_FRESH_AGE    = 1.5    # s
MOTION_FRESH_AGE = 1.5    # s
GYRZ_FRESH_AGE   = 0.30   # s
ALT_FRESH_AGE    = 0.75   # s

# ── L1 parameters ─────────────────────────────────────────────────────────────
L1_DAMPING         = 0.75
L1_PERIOD_S        = 12.0
L1_MIN_M           = 5.0
V_MIN_MPS          = 2.0
LAT_ACC_MAX        = 4.0    # m/s^2
COURSE_RATE_MAX    = math.radians(config.MOTOR_NOMINAL_CLOSED_LOOP_ANGULAR_VELOCITY_CMD_MAX_DEG_S)
EARTH_RADIUS_M     = 6_371_000.0


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
    # Legacy values (kept for backward compatibility)
    NOMINAL_CLOSED_LOOP  = config.CONTROL_MODE_NOMINAL_CLOSED_LOOP
    NOMINAL_FEEDFORWARD  = config.CONTROL_MODE_NOMINAL_FEEDFORWARD
    DEGRADED_CLOSED_LOOP = config.CONTROL_MODE_DEGRADED_CLOSED_LOOP
    DEGRADED_FEEDFORWARD = config.CONTROL_MODE_DEGRADED_FEEDFORWARD
    FAIL                 = config.CONTROL_MODE_FAIL
    # New homing-architecture modes
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


class FailReason(enum.Enum):
    NONE = config.FAIL_REASON_NONE
    FREEFALL = config.FAIL_REASON_FREEFALL
    TUMBLE_YAW_DOMINANT = config.FAIL_REASON_TUMBLE_YAW_DOMINANT
    TUMBLE_ROLLPITCH = config.FAIL_REASON_TUMBLE_ROLLPITCH
    UNSTABLE_BODY = config.FAIL_REASON_UNSTABLE_BODY
    NO_POSITION = config.FAIL_REASON_NO_POSITION
    NO_MOTION = config.FAIL_REASON_NO_MOTION
    SENSOR_BLACKOUT = config.FAIL_REASON_SENSOR_BLACKOUT


@dataclass
class FreshResult:
    """Per-iteration sensor freshness snapshot tied to a single 'now' timestamp."""
    # POINT
    point_fresh: bool = False
    point_correction_usable: bool = False
    point_age_s: float = math.inf
    # VELOCITY
    velocity_fresh: bool = False
    velocity_correction_usable: bool = False
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


@dataclass(frozen=True)
class ControlPolicy:
    confidence_scale: float
    angular_velocity_cmd_max_deg_s: float
    lat_acc_max_mps2: float
    delta_ff_max_deg: float
    delta_pid_max_deg: float
    delta_total_max_deg: float
    max_arm_rate_deg_s: float
    pid_enabled: bool
    l1_enabled: bool = True
    l1_period_s: float = L1_PERIOD_S


@dataclass
class L1Input:
    pos_N: Optional[float] = None
    pos_E: Optional[float] = None
    course: Optional[float] = None
    ground_speed_mps: Optional[float] = None
    yaw: Optional[float] = None
    gyrz: Optional[float] = None
    gyrx: Optional[float] = None
    gyry: Optional[float] = None
    alt: Optional[float] = None
    pos_quality: SensorQuality = SensorQuality.STALE
    motion_quality: SensorQuality = SensorQuality.STALE
    gyrz_quality: SensorQuality = SensorQuality.STALE
    alt_quality: SensorQuality = SensorQuality.STALE
    origin_lat: Optional[float] = None
    origin_lon: Optional[float] = None
    target_lat: Optional[float] = None
    target_lon: Optional[float] = None
    control_mode: Optional[ControlMode] = None
    fail_reason: FailReason = FailReason.NONE
    sink_rate: Optional[float] = None
    freefall: int = 0
    tumble: int = 0
    # Guidance confidence [0..1]
    confidence: float = 0.0
    # DR state carried into the guidance computation
    dr_method: str = config.DR_METHOD_NONE
    dr_pos_N: Optional[float] = None
    dr_pos_E: Optional[float] = None
    dr_course: Optional[float] = None
    dr_speed_mps: Optional[float] = None

def _wrap_pi(angle_rad: float) -> float:
    return (float(angle_rad) + math.pi) % (2.0 * math.pi) - math.pi


def _is_fresh(timestamp: Optional[float], now: float, max_age: float) -> bool:
    age_s = timebase.age(now, timestamp)
    return math.isfinite(age_s) and 0.0 <= age_s <= float(max_age)


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


#have to add member
@dataclass
class L1Output:
    timestamp: float = 0.0
    nominal: bool = False
    degraded: bool = False
    control_valid: bool = False
    reason: str = config.MOTOR_REASON_INIT
    fail_reason: str = config.FAIL_REASON_NONE
    confidence_scale: float = 0.0
    angular_velocity_cmd_max_deg_s: float = 0.0
    lat_acc_max_mps2: float = 0.0
    delta_ff_max_deg: float = 0.0
    delta_pid_max_deg: float = 0.0
    delta_total_max_deg: float = 0.0
    max_arm_rate_deg_s: float = 0.0
    pid_enabled: bool = False
    angular_velocity_cmd_rad_s: float = 0.0
    lat_acc_cmd_mps2: float = 0.0
    ground_speed_mps: float = 0.0
    L1_distance: float = 0.0
    nu1: float = 0.0
    nu2: float = 0.0
    angle_to_turn: float = 0.0
    crossTrack: float = 0.0
    alongTrack: float = 0.0
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
    # New homing-architecture fields
    yaw_rate_limit_dps: float = 0.0
    dr_confidence: float = 0.0
    dr_method: str = config.DR_METHOD_NONE


def FillFresh(
    l1_input: L1Input,
    gps,
    imu,
    baro,
    start_lat: float,
    start_lon: float,
    now: float,
) -> L1Input:
    """Fill L1Input with fresh sensor values only."""
    gps_course = getattr(gps, "course", getattr(gps, "course_rad", None)) if gps is not None else None
    gps_speed = getattr(gps, "speed", getattr(gps, "speed_mps", None)) if gps is not None else None
    imu_yaw = getattr(imu, "yaw", getattr(imu, "yaw_rad", None)) if imu is not None else None
    imu_gyrx = getattr(imu, "gyrx", getattr(imu, "gyrx_rad_s", None)) if imu is not None else None
    imu_gyry = getattr(imu, "gyry", getattr(imu, "gyry_rad_s", None)) if imu is not None else None
    imu_gyrz = getattr(imu, "gyrz", getattr(imu, "gyrz_rad_s", None)) if imu is not None else None
    baro_alt = getattr(baro, "alt", getattr(baro, "alt_m", None)) if baro is not None else None

    if (
        gps is not None
        and gps.lat is not None
        and gps.lon is not None
        and gps.pos_ts is not None
        and _is_fresh(gps.pos_ts, now, POS_FRESH_AGE)
        and start_lat is not None
        and start_lon is not None
    ):
        projected = project_from_origin(gps.lat, gps.lon, start_lat, start_lon)
        if projected is None:
            l1_input.pos_N = None
            l1_input.pos_E = None
            l1_input.pos_quality = SensorQuality.STALE
        else:
            l1_input.pos_N, l1_input.pos_E = projected
            l1_input.pos_quality = SensorQuality.FRESH

    if (
        gps is not None
        and gps_course is not None
        and gps_speed is not None
        and gps.motion_ts is not None
        and _is_fresh(gps.motion_ts, now, MOTION_FRESH_AGE)
    ):
        l1_input.course = gps_course
        l1_input.ground_speed_mps = gps_speed
        l1_input.motion_quality = SensorQuality.FRESH

    if (
        imu is not None
        and imu_gyrz is not None
        and imu.ts is not None
        and _is_fresh(imu.ts, now, GYRZ_FRESH_AGE)
    ):
        l1_input.yaw = imu_yaw
        l1_input.gyrx = imu_gyrx
        l1_input.gyry = imu_gyry
        l1_input.gyrz = imu_gyrz
        l1_input.gyrz_quality = SensorQuality.FRESH
        l1_input.freefall = getattr(imu, "freefall", 0)
        l1_input.tumble   = getattr(imu, "tumble",   0)

    if (
        baro is not None
        and baro_alt is not None
        and baro.ts is not None
        and _is_fresh(baro.ts, now, ALT_FRESH_AGE)
    ):
        l1_input.alt         = baro_alt
        l1_input.alt_quality = SensorQuality.FRESH
        l1_input.sink_rate   = getattr(baro, "sink_rate", None)

    return l1_input


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
        result.point_fresh = age <= config.GPS_CONTROL_FRESH_MAX_AGE_S and latest.valid
        result.point_correction_usable = age <= config.GPS_CORRECTION_MAX_AGE_S and latest.valid
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
        result.velocity_fresh = age <= config.GPS_CONTROL_FRESH_MAX_AGE_S and latest.valid
        result.velocity_correction_usable = age <= config.GPS_CORRECTION_MAX_AGE_S and latest.valid
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
                lin_acc_valid=False,
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


def _body_fail_reason(l1_input: L1Input) -> FailReason:
    def _deg(value) -> Optional[float]:
        try:
            value_f = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(value_f):
            return None
        return math.degrees(value_f)

    gx = _deg(l1_input.gyrx)
    gy = _deg(l1_input.gyry)
    gz = _deg(l1_input.gyrz)
    if gx is None and gy is None and gz is None:
        return FailReason.TUMBLE_ROLLPITCH if l1_input.tumble else FailReason.NONE
    roll_pitch_abs = max(abs(gx or 0.0), abs(gy or 0.0))
    yaw_abs = abs(gz or 0.0)
    yaw_dominant = (
        yaw_abs > config.MOTOR_TUMBLE_YAW_DOMINANT_MIN_DEG_S
        and yaw_abs > config.MOTOR_TUMBLE_YAW_DOMINANT_RATIO * roll_pitch_abs
    )
    roll_pitch_dominant = roll_pitch_abs > config.MOTOR_TUMBLE_ROLLPITCH_MIN_DEG_S
    if yaw_dominant:
        return FailReason.TUMBLE_YAW_DOMINANT
    if roll_pitch_dominant:
        return FailReason.TUMBLE_ROLLPITCH
    return FailReason.UNSTABLE_BODY if l1_input.tumble else FailReason.NONE


def DecideControlMode(l1_input: L1Input) -> tuple[ControlMode, FailReason]:
    if l1_input.freefall:
        return ControlMode.FAIL, FailReason.FREEFALL
    if l1_input.tumble:
        return ControlMode.FAIL, _body_fail_reason(l1_input)

    pos_quality    = l1_input.pos_quality
    motion_quality = l1_input.motion_quality
    gyrz_quality   = l1_input.gyrz_quality

    if pos_quality == SensorQuality.STALE or motion_quality == SensorQuality.STALE:
        if (
            pos_quality    == SensorQuality.STALE
            and motion_quality == SensorQuality.STALE
            and gyrz_quality   == SensorQuality.STALE
            and l1_input.alt_quality == SensorQuality.STALE
        ):
            fail = FailReason.SENSOR_BLACKOUT
        elif pos_quality == SensorQuality.STALE:
            fail = FailReason.NO_POSITION
        else:
            fail = FailReason.NO_MOTION
        return ControlMode.FAIL, fail

    # pos and motion are both FRESH
    closed_loop = l1_input.gyrz is not None and gyrz_quality == SensorQuality.FRESH
    if closed_loop:
        return ControlMode.NOMINAL_CLOSED_LOOP, FailReason.NONE
    return ControlMode.NOMINAL_FEEDFORWARD, FailReason.NONE

def ProduceL1Input(
    gps,
    imu,
    baro,
    origin_lat: float,
    origin_lon: float,
    target_lat: float,
    target_lon: float,
    now: float,
) -> tuple[L1Input, ControlMode]:
    l1_input = L1Input()
    l1_input.origin_lat = origin_lat
    l1_input.origin_lon = origin_lon
    l1_input.target_lat = target_lat
    l1_input.target_lon = target_lon
    FillFresh(l1_input, gps, imu, baro, origin_lat, origin_lon, now)
    control_mode, fail_reason = DecideControlMode(l1_input)
    l1_input.control_mode = control_mode
    l1_input.fail_reason = fail_reason
    return (l1_input, control_mode)

def ProduceL1Output(
    l1_input: L1Input,
    mode: ControlMode,
    origin_lat: float,
    origin_lon: float,
    target_lat: float,
    target_lon: float,
    now: float,
) -> L1Output:
    l1_output = L1Output(timestamp=now)
    l1_output.reason = getattr(mode, "value", str(mode))
    l1_output.target_lat = target_lat
    l1_output.target_lon = target_lon
    fail_reason = getattr(l1_input, "fail_reason", FailReason.NONE)
    if not isinstance(fail_reason, FailReason):
        try:
            fail_reason = FailReason(str(fail_reason))
        except ValueError:
            fail_reason = FailReason.NONE
    l1_output.fail_reason = fail_reason.value
    policy = _policy_for_mode(mode, fail_reason)
    _apply_policy(l1_output, policy)

    if mode == ControlMode.FAIL:
        l1_output.reason = fail_reason.value if fail_reason != FailReason.NONE else config.CONTROL_MODE_FAIL
        if fail_reason == FailReason.TUMBLE_YAW_DOMINANT and l1_input.gyrz is not None:
            gyrz_deg_s = math.degrees(float(l1_input.gyrz))
            yaw_cmd_deg_s = max(
                -policy.angular_velocity_cmd_max_deg_s,
                min(policy.angular_velocity_cmd_max_deg_s, -config.MOTOR_TUMBLE_COUNTER_YAW_GAIN * gyrz_deg_s),
            )
            l1_output.control_valid = True
            l1_output.angular_velocity_cmd_rad_s = math.radians(yaw_cmd_deg_s)
            l1_output.ground_speed_mps = float(getattr(l1_input, "ground_speed_mps", 0.0) or 0.0)
            return l1_output
        if (
            fail_reason == FailReason.NO_MOTION
            and l1_input.pos_N is not None
            and l1_input.pos_E is not None
            and l1_input.yaw is not None
            and origin_lat is not None
            and origin_lon is not None
            and target_lat is not None
            and target_lon is not None
        ):
            target_N, target_E = latlon_to_ne(target_lat, target_lon, origin_lat, origin_lon)
            bearing_to_target = math.atan2(target_E - l1_input.pos_E, target_N - l1_input.pos_N)
            heading_error = _wrap_pi(bearing_to_target - float(l1_input.yaw))
            yaw_cmd_rad_s = config.MOTOR_TARGET_BEARING_GAIN * heading_error
            yaw_max_rad_s = math.radians(policy.angular_velocity_cmd_max_deg_s)
            l1_output.control_valid = True
            l1_output.angular_velocity_cmd_rad_s = max(-yaw_max_rad_s, min(yaw_max_rad_s, yaw_cmd_rad_s))
            l1_output.target_N = target_N
            l1_output.target_E = target_E
            l1_output.pos_N = l1_input.pos_N
            l1_output.pos_E = l1_input.pos_E
            l1_output.current_heading_rad = float(l1_input.yaw)
            return l1_output
        return l1_output

    pos_N = getattr(l1_input, "pos_N", None)
    pos_E = getattr(l1_input, "pos_E", None)
    course = getattr(l1_input, "course", None)
    speed = getattr(l1_input, "ground_speed_mps", None)
    if (
        pos_N is None
        or pos_E is None
        or course is None
        or speed is None
        or origin_lat is None
        or origin_lon is None
        or target_lat is None
        or target_lon is None
    ):
        l1_output.reason = config.FAIL_REASON_NO_POSITION
        l1_output.fail_reason = FailReason.NO_POSITION.value
        return l1_output

    target_N, target_E = latlon_to_ne(target_lat, target_lon, origin_lat, origin_lon)
    rel_N = target_N - float(pos_N)
    rel_E = target_E - float(pos_E)
    target_range = math.hypot(rel_N, rel_E)
    if target_range <= 1e-6:
        l1_output.reason = config.FAIL_REASON_NO_POSITION
        l1_output.fail_reason = FailReason.NO_POSITION.value
        return l1_output

    speed_for_l1 = max(float(speed), V_MIN_MPS)
    L1_distance = max((L1_DAMPING * policy.l1_period_s / math.pi) * speed_for_l1, L1_MIN_M)
    # Fixed-target homing: the target itself is the carrot for the entire flight.
    # The period-based L1 distance remains only as the controller gain scale.
    carrot_N = target_N
    carrot_E = target_E

    target_bearing = math.atan2(rel_E, rel_N)
    nu1 = 0.0
    nu2 = _wrap_pi(target_bearing - float(course))
    nu = nu2
    nu_clamped = max(-math.pi / 2.0, min(math.pi / 2.0, nu))
    K_L1 = 4.0 * L1_DAMPING * L1_DAMPING
    lat_acc = K_L1 * speed_for_l1 * speed_for_l1 / L1_distance * math.sin(nu_clamped)
    lat_acc *= policy.confidence_scale
    lat_acc = max(-policy.lat_acc_max_mps2, min(policy.lat_acc_max_mps2, lat_acc))
    angular_velocity_rad_s = lat_acc / speed_for_l1
    angular_velocity_max_rad_s = math.radians(policy.angular_velocity_cmd_max_deg_s)
    angular_velocity_rad_s = max(-angular_velocity_max_rad_s, min(angular_velocity_max_rad_s, angular_velocity_rad_s))

    l1_output.nominal = True
    l1_output.degraded = mode in (ControlMode.DEGRADED_CLOSED_LOOP, ControlMode.DEGRADED_FEEDFORWARD)
    l1_output.control_valid = True
    l1_output.angular_velocity_cmd_rad_s = angular_velocity_rad_s
    l1_output.lat_acc_cmd_mps2 = lat_acc
    l1_output.ground_speed_mps = float(speed)
    l1_output.L1_distance = L1_distance
    l1_output.nu1 = nu1
    l1_output.nu2 = nu2
    l1_output.angle_to_turn = nu
    l1_output.crossTrack = 0.0
    l1_output.alongTrack = target_range
    l1_output.pos_N = pos_N
    l1_output.pos_E = pos_E
    l1_output.target_N = target_N
    l1_output.target_E = target_E
    l1_output.carrot_N = carrot_N
    l1_output.carrot_E = carrot_E
    l1_output.carrot_lat, l1_output.carrot_lon = ne_to_latlon(carrot_N, carrot_E, origin_lat, origin_lon)
    l1_output.current_heading_rad = course
    return l1_output
