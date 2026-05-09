"""Guidance module: InputResolver, L1 guidance, and motor output computation.

Implements ArduPilot L1 navigation controller logic ported to Python.
Reference: libraries/AP_L1_Control/AP_L1_Control.cpp :: update_waypoint()

Sign convention: commanded_yaw_rate > 0 = LEFT turn.
Units: _deg / _rad / _m / _ms / _mps suffixes throughout.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Optional, Tuple

# ── Sensor age constants ──────────────────────────────────────────────────────
POS_FRESH_AGE    = 1.0    # s
POS_STALE_MAX    = 3.0    # s
MOTION_FRESH_AGE = 1.0    # s
MOTION_STALE_MAX = 3.0    # s
GYRZ_FRESH_AGE   = 0.20   # s
GYRZ_STALE_MAX   = 0.50   # s
ALT_FRESH_AGE    = 0.50   # s
ALT_STALE_MAX    = 3.0    # s

# ── L1 parameters ─────────────────────────────────────────────────────────────
L1_DAMPING         = 0.75
L1_PERIOD_S        = 8.0
L1_MIN_M           = 5.0
V_MIN_MPS          = 2.0
LAT_ACC_MAX        = 4.0    # m/s^2
COURSE_RATE_MAX    = 0.6    # rad/s
XTRACK_SOFT_FACTOR = 2.0    # × L1_dist
XTRACK_HARD_FACTOR = 4.0    # × L1_dist
XTRACK_SOFT_MIN_M  = 20.0
XTRACK_HARD_MIN_M  = 50.0

# ── Earth geometry ────────────────────────────────────────────────────────────
EARTH_R = 6_371_000.0  # m

# ── Legacy guidance constants ─────────────────────────────────────────────────
GPS_STABLE_COUNT_REQUIRED = 1
LANDING_YR_MAX  = 30.0   # deg/s  (yaw-rate cap for landing phase)
YR_MAX          = 45.0   # deg/s  (general)
L_DISTANCE      = 30.0   # m      (L1 look-ahead for legacy guidance())

# ── Legacy module-level state (for test compatibility) ────────────────────────
_PREV_GPS = SimpleNamespace(
    initialized=False,
    lat=0.0,
    lon=0.0,
    time=0.0,
)
_GPS_STABLE_COUNT: int = 0
_START_LAT: Optional[float] = None
_START_LON: Optional[float] = None
START_POINT = SimpleNamespace(lat=0.0, lon=0.0)
CASCADE_PI = SimpleNamespace(MAX_CMD=YR_MAX)


# ═══════════════════════════════════════════════════════════════════════════════
# Enums / field-status strings
# ═══════════════════════════════════════════════════════════════════════════════

class SensorQuality:
    FRESH   = "fresh"
    STALE   = "stale"
    MISSING = "missing"


class ControlMode:
    ACTIVE_CLOSED_LOOP   = "ACTIVE_CLOSED_LOOP"
    ACTIVE_FEEDFORWARD   = "ACTIVE_FEEDFORWARD"
    DEGRADED_CLOSED_LOOP = "DEGRADED_CLOSED_LOOP"
    DEGRADED_FEEDFORWARD = "DEGRADED_FEEDFORWARD"
    SAFE_GLIDE           = "SAFE_GLIDE"


# ═══════════════════════════════════════════════════════════════════════════════
# Dataclasses
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class InputResolverState:
    # GPS raw cache
    raw_lat: Optional[float]           = None
    raw_lon: Optional[float]           = None
    raw_course_rad: Optional[float]    = None
    raw_ground_speed_mps: Optional[float] = None
    raw_gps_ts: Optional[float]        = None
    raw_pos_health: bool               = False
    raw_motion_health: bool            = False
    # last_good GPS (health=True only)
    lg_lat: Optional[float]            = None
    lg_lon: Optional[float]            = None
    lg_course_rad: Optional[float]     = None
    lg_ground_speed_mps: Optional[float] = None
    lg_gps_ts: Optional[float]         = None
    # IMU raw cache
    raw_gyrz_rad_s: Optional[float]   = None
    raw_imu_ts: Optional[float]       = None
    raw_imu_health: bool              = False
    # last_good IMU (health=True only)
    lg_gyrz_rad_s: Optional[float]   = None
    lg_imu_ts: Optional[float]       = None
    # Baro raw cache
    raw_alt_m: Optional[float]        = None
    raw_baro_ts: Optional[float]      = None
    raw_baro_health: bool             = False
    # last_good Baro (health=True only)
    lg_alt_m: Optional[float]         = None
    lg_baro_ts: Optional[float]       = None
    # Origin for NE conversion
    origin_lat: Optional[float]       = None
    origin_lon: Optional[float]       = None
    # Legacy GPS jump gate (test compatibility)
    _prev_gps_initialized: bool       = False
    _prev_gps_lat: float              = 0.0
    _prev_gps_lon: float              = 0.0
    _prev_gps_time: float             = 0.0
    _gps_stable_count: int            = 0


@dataclass
class GuidanceInput:
    timestamp: float
    # Position (L)
    pos_N: Optional[float]           = None   # m north of origin
    pos_E: Optional[float]           = None   # m east of origin
    pos_status: str                  = SensorQuality.MISSING
    # Course (C)
    course: Optional[float]          = None   # rad, 0=north, +east
    motion_health: str               = SensorQuality.MISSING
    # Speed (S)
    ground_speed_mps: Optional[float] = None
    # GyroZ (G)
    gyrz: Optional[float]            = None   # rad/s
    gyrz_health: str                 = SensorQuality.MISSING
    # Altitude
    altitude: Optional[float]        = None   # m
    alt_health: str                  = SensorQuality.MISSING
    # LCSG decision
    lcsg_case: str                   = "----"
    input_policy: str                = ""
    control_mode: str                = ControlMode.SAFE_GLIDE
    reason: str                      = ""


@dataclass
class L1Config:
    damping: float        = L1_DAMPING
    period: float         = L1_PERIOD_S
    L1_min: float         = L1_MIN_M
    v_min: float          = V_MIN_MPS
    lat_acc_max: float    = LAT_ACC_MAX
    course_rate_max: float = COURSE_RATE_MAX


@dataclass
class L1State:
    config: L1Config          = field(default_factory=L1Config)
    start_N: float            = 0.0
    start_E: float            = 0.0
    target_N: Optional[float] = None
    target_E: Optional[float] = None
    origin_lat: Optional[float] = None
    origin_lon: Optional[float] = None
    submode: str              = "LINE_FOLLOW"


@dataclass
class GuidanceOutput:
    timestamp: float
    active: bool               = False
    degraded: bool             = False
    reason: str                = ""
    crossTrack: float          = float("nan")
    alongTrack: float          = float("nan")
    L1_dist: float             = float("nan")
    Nu1: float                 = float("nan")
    Nu2: float                 = float("nan")
    Nu: float                  = float("nan")
    lat_acc_cmd_mps2: float    = float("nan")
    course_rate_cmd_rad_s: float = float("nan")
    submode: str               = "SAFE_GLIDE"
    # Telemetry
    start_lat: float           = float("nan")
    start_lon: float           = float("nan")
    target_lat: float          = float("nan")
    target_lon: float          = float("nan")
    carrot_lat: float          = float("nan")
    carrot_lon: float          = float("nan")
    current_heading_deg: float = float("nan")
    desired_heading_deg: float = float("nan")


# ═══════════════════════════════════════════════════════════════════════════════
# Utility
# ═══════════════════════════════════════════════════════════════════════════════

def _ll_to_ne(
    lat: float,
    lon: float,
    origin_lat: float = 0.0,
    origin_lon: float = 0.0,
) -> Tuple[float, float]:
    """Convert lat/lon to North/East offset (m) relative to origin."""
    dlat = math.radians(lat - origin_lat)
    dlon = math.radians(lon - origin_lon)
    N = dlat * EARTH_R
    E = dlon * EARTH_R * math.cos(math.radians(origin_lat))
    return N, E


def _haversine_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in metres."""
    R = EARTH_R
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return 2 * R * math.asin(min(1.0, math.sqrt(a)))


def _haversine_bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Initial bearing from (lat1,lon1) to (lat2,lon2), degrees, 0=North, +East."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    x = math.sin(dlam) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def _wrap_180(angle_deg: float) -> float:
    """Wrap angle to (−180, +180]."""
    angle_deg = angle_deg % 360.0
    if angle_deg > 180.0:
        angle_deg -= 360.0
    return angle_deg


# ═══════════════════════════════════════════════════════════════════════════════
# InputResolverState factory and update functions
# ═══════════════════════════════════════════════════════════════════════════════

def make_resolver_state() -> InputResolverState:
    return InputResolverState()


def resolver_update_gnss(
    resolver: InputResolverState,
    lat: float,
    lon: float,
    course_rad: float,
    groundSpeed: float,
    posHealth: bool,
    motionHealth: bool,
    ts: float,
) -> None:
    """Cache raw GPS reading; update last_good only when health=True."""
    resolver.raw_lat           = lat
    resolver.raw_lon           = lon
    resolver.raw_course_rad    = course_rad
    resolver.raw_ground_speed_mps = groundSpeed
    resolver.raw_gps_ts        = ts
    resolver.raw_pos_health    = bool(posHealth)
    resolver.raw_motion_health = bool(motionHealth)

    if posHealth:
        resolver.lg_lat    = lat
        resolver.lg_lon    = lon
        resolver.lg_gps_ts = ts
    if motionHealth:
        resolver.lg_course_rad        = course_rad
        resolver.lg_ground_speed_mps  = groundSpeed
        if not posHealth:
            # motion timestamp shares gps timestamp
            pass
        resolver.lg_gps_ts = ts  # always refresh ts when any health flag is True


def resolver_update_imu(
    resolver: InputResolverState,
    roll: float  = 0.0,
    pitch: float = 0.0,
    yaw: float   = 0.0,
    ax: float    = 0.0,
    ay: float    = 0.0,
    az: float    = 0.0,
    gx: float    = 0.0,
    gy: float    = 0.0,
    gz: float    = 0.0,
    imu_health: bool = False,
    ts: Optional[float] = None,
) -> None:
    """Cache raw IMU reading (gz = yaw rate rad/s); update last_good when health=True."""
    if ts is None:
        ts = time.time()
    resolver.raw_gyrz_rad_s = gz
    resolver.raw_imu_ts     = ts
    resolver.raw_imu_health = bool(imu_health)

    if imu_health:
        resolver.lg_gyrz_rad_s = gz
        resolver.lg_imu_ts     = ts


def resolver_update_baro(
    resolver: InputResolverState,
    alt: float,
    ts: float,
    baro_health: bool = False,
) -> None:
    """Cache raw barometer reading; update last_good when health=True."""
    resolver.raw_alt_m    = alt
    resolver.raw_baro_ts  = ts
    resolver.raw_baro_health = bool(baro_health)

    if baro_health:
        resolver.lg_alt_m   = alt
        resolver.lg_baro_ts = ts


def resolver_set_origin(resolver: InputResolverState, lat: float, lon: float) -> None:
    resolver.origin_lat = lat
    resolver.origin_lon = lon


def resolver_reset_origin(resolver: InputResolverState) -> None:
    resolver.origin_lat = None
    resolver.origin_lon = None


# ═══════════════════════════════════════════════════════════════════════════════
# fill_current_data / fill_stale_data / decide_control_mode
# ═══════════════════════════════════════════════════════════════════════════════

def fill_current_data(
    resolver: InputResolverState,
    state: GuidanceInput,
    now: float,
) -> None:
    """Classify each field as FRESH/STALE based on health flag and age.

    FRESH  : health=True AND age <= fresh threshold → write value
    STALE  : health=False OR age > threshold        → mark STALE, do NOT write value
             (fill_stale_data will supply a value from last_good or raw)
    """
    origin_lat = resolver.origin_lat or 0.0
    origin_lon = resolver.origin_lon or 0.0

    # ── Position (L) ──────────────────────────────────────────────────────────
    if (
        resolver.raw_gps_ts is not None
        and resolver.raw_pos_health
        and (now - resolver.raw_gps_ts) <= POS_FRESH_AGE
    ):
        state.pos_status = SensorQuality.FRESH
        lat = resolver.raw_lat
        lon = resolver.raw_lon
        state.pos_N, state.pos_E = _ll_to_ne(lat, lon, origin_lat, origin_lon)
    elif resolver.raw_gps_ts is not None:
        state.pos_status = SensorQuality.STALE
        # value left as None; fill_stale_data will populate
    # else: remains MISSING

    # ── Course / motion (C + S) ───────────────────────────────────────────────
    if (
        resolver.raw_gps_ts is not None
        and resolver.raw_motion_health
        and (now - resolver.raw_gps_ts) <= MOTION_FRESH_AGE
    ):
        state.motion_health    = SensorQuality.FRESH
        state.course           = resolver.raw_course_rad
        state.ground_speed_mps = resolver.raw_ground_speed_mps
    elif resolver.raw_gps_ts is not None:
        state.motion_health = SensorQuality.STALE

    # ── GyroZ (G) ─────────────────────────────────────────────────────────────
    if (
        resolver.raw_imu_ts is not None
        and resolver.raw_imu_health
        and (now - resolver.raw_imu_ts) <= GYRZ_FRESH_AGE
    ):
        state.gyrz_health = SensorQuality.FRESH
        state.gyrz        = resolver.raw_gyrz_rad_s
    elif resolver.raw_imu_ts is not None:
        state.gyrz_health = SensorQuality.STALE

    # ── Altitude ──────────────────────────────────────────────────────────────
    if (
        resolver.raw_baro_ts is not None
        and resolver.raw_baro_health
        and (now - resolver.raw_baro_ts) <= ALT_FRESH_AGE
    ):
        state.alt_health = SensorQuality.FRESH
        state.altitude   = resolver.raw_alt_m
    elif resolver.raw_baro_ts is not None:
        state.alt_health = SensorQuality.STALE


def fill_stale_data(
    resolver: InputResolverState,
    state: GuidanceInput,
    now: float,
) -> None:
    """For STALE fields, supply values from last_good (within stale window)
    or fall back to raw values if last_good is absent.
    FRESH fields are not touched.
    """
    origin_lat = resolver.origin_lat or 0.0
    origin_lon = resolver.origin_lon or 0.0

    # ── Position ──────────────────────────────────────────────────────────────
    if state.pos_status == SensorQuality.STALE and state.pos_N is None:
        if (
            resolver.lg_gps_ts is not None
            and resolver.lg_lat is not None
            and (now - resolver.lg_gps_ts) <= POS_STALE_MAX
        ):
            state.pos_N, state.pos_E = _ll_to_ne(
                resolver.lg_lat, resolver.lg_lon, origin_lat, origin_lon
            )
        elif (
            resolver.raw_lat is not None
            and resolver.raw_gps_ts is not None
            and (now - resolver.raw_gps_ts) <= POS_STALE_MAX
        ):
            state.pos_N, state.pos_E = _ll_to_ne(
                resolver.raw_lat, resolver.raw_lon, origin_lat, origin_lon
            )
        else:
            state.pos_status = SensorQuality.MISSING

    # ── Course / speed ────────────────────────────────────────────────────────
    if state.motion_health == SensorQuality.STALE and state.course is None:
        if (
            resolver.lg_gps_ts is not None
            and resolver.lg_course_rad is not None
            and (now - resolver.lg_gps_ts) <= MOTION_STALE_MAX
        ):
            state.course           = resolver.lg_course_rad
            state.ground_speed_mps = resolver.lg_ground_speed_mps
        elif (
            resolver.raw_course_rad is not None
            and resolver.raw_gps_ts is not None
            and (now - resolver.raw_gps_ts) <= MOTION_STALE_MAX
        ):
            state.course           = resolver.raw_course_rad
            state.ground_speed_mps = resolver.raw_ground_speed_mps
        else:
            state.motion_health = SensorQuality.MISSING

    # ── GyroZ ─────────────────────────────────────────────────────────────────
    if state.gyrz_health == SensorQuality.STALE and state.gyrz is None:
        if (
            resolver.lg_imu_ts is not None
            and resolver.lg_gyrz_rad_s is not None
            and (now - resolver.lg_imu_ts) <= GYRZ_STALE_MAX
        ):
            state.gyrz = resolver.lg_gyrz_rad_s
        elif (
            resolver.raw_gyrz_rad_s is not None
            and resolver.raw_imu_ts is not None
            and (now - resolver.raw_imu_ts) <= GYRZ_STALE_MAX
        ):
            state.gyrz = resolver.raw_gyrz_rad_s
        else:
            state.gyrz_health = SensorQuality.MISSING

    # ── Altitude ──────────────────────────────────────────────────────────────
    if state.alt_health == SensorQuality.STALE and state.altitude is None:
        if (
            resolver.lg_baro_ts is not None
            and resolver.lg_alt_m is not None
            and (now - resolver.lg_baro_ts) <= ALT_STALE_MAX
        ):
            state.altitude = resolver.lg_alt_m
        elif (
            resolver.raw_alt_m is not None
            and resolver.raw_baro_ts is not None
            and (now - resolver.raw_baro_ts) <= ALT_STALE_MAX
        ):
            state.altitude = resolver.raw_alt_m
        else:
            state.alt_health = SensorQuality.MISSING


def decide_control_mode(state: GuidanceInput) -> None:
    """Classify LCSG and set control_mode + reason on the GuidanceInput.

    LCSG flags:
      L = pos_status FRESH
      C = motion_health FRESH
      S = ground_speed_mps not None AND motion_health FRESH
      G = gyrz_health FRESH
    """
    L_fresh = state.pos_status    == SensorQuality.FRESH
    C_fresh = state.motion_health == SensorQuality.FRESH
    S_fresh = C_fresh and state.ground_speed_mps is not None
    G_fresh = state.gyrz_health   == SensorQuality.FRESH

    L_stale = state.pos_status    == SensorQuality.STALE
    C_stale = state.motion_health == SensorQuality.STALE
    S_stale = C_stale  # speed shares motion timestamp

    L_miss  = state.pos_status    == SensorQuality.MISSING
    C_miss  = state.motion_health == SensorQuality.MISSING

    lcsg = (
        ("L" if L_fresh else ("-" if L_miss else "l"))
        + ("C" if C_fresh else ("-" if C_miss else "c"))
        + ("S" if S_fresh else "-")
        + ("G" if G_fresh else "-")
    )
    state.lcsg_case = lcsg

    # Determine mode
    if L_miss or C_miss or (not L_fresh and not L_stale) or (not C_fresh and not C_stale):
        state.control_mode = ControlMode.SAFE_GLIDE
        state.reason       = "MISSING position or course → safe_glide"
        return

    # Check if any of L/C/S are stale (not fresh)
    any_stale = L_stale or C_stale
    all_lcs_fresh = L_fresh and C_fresh and S_fresh

    if all_lcs_fresh:
        if G_fresh:
            state.control_mode = ControlMode.ACTIVE_CLOSED_LOOP
            state.reason       = "L+C+S+G all fresh → active closed-loop"
        else:
            state.control_mode = ControlMode.ACTIVE_FEEDFORWARD
            state.reason       = "L+C+S fresh, G missing/stale → active feedforward"
    elif any_stale and not L_miss and not C_miss:
        if G_fresh:
            state.control_mode = ControlMode.DEGRADED_CLOSED_LOOP
            state.reason       = "some stale sensor data → degraded closed-loop"
        else:
            state.control_mode = ControlMode.DEGRADED_FEEDFORWARD
            state.reason       = "some stale sensor data, no gyro → degraded feedforward"
    else:
        state.control_mode = ControlMode.SAFE_GLIDE
        state.reason       = "insufficient sensor data → safe_glide"


def resolver_resolve(resolver: InputResolverState, now: float) -> GuidanceInput:
    """Full resolve pipeline: fill_current → fill_stale → decide_mode."""
    state = GuidanceInput(timestamp=now)
    fill_current_data(resolver, state, now)
    fill_stale_data(resolver, state, now)
    decide_control_mode(state)

    _INPUT_POLICY_MAP = {
        ControlMode.ACTIVE_CLOSED_LOOP:   "nominal_l1_with_yaw_rate_feedback",
        ControlMode.ACTIVE_FEEDFORWARD:   "l1_valid_feedforward_only_no_gyro",
        ControlMode.DEGRADED_CLOSED_LOOP: "degraded_l1_with_yaw_rate_feedback",
        ControlMode.DEGRADED_FEEDFORWARD: "degraded_l1_feedforward_only",
        ControlMode.SAFE_GLIDE:           "safe_glide_neutral",
    }
    state.input_policy = _INPUT_POLICY_MAP.get(state.control_mode, "safe_glide_neutral")
    return state


# ═══════════════════════════════════════════════════════════════════════════════
# L1State factory and control functions
# ═══════════════════════════════════════════════════════════════════════════════

def make_l1_state(cfg: Optional[L1Config] = None) -> L1State:
    return L1State(config=cfg or L1Config())


def l1_reset(l1: L1State) -> None:
    l1.start_N   = 0.0
    l1.start_E   = 0.0
    l1.target_N  = None
    l1.target_E  = None
    l1.submode   = "LINE_FOLLOW"


def l1_set_start(l1: L1State, N: float, E: float) -> None:
    l1.start_N = N
    l1.start_E = E


def l1_set_target(l1: L1State, N: float, E: float) -> None:
    l1.target_N = N
    l1.target_E = E
    l1.submode  = "LINE_FOLLOW"  # reset submode on new target


def l1_update(l1: L1State, inp: GuidanceInput, now: float) -> GuidanceOutput:
    """ArduPilot L1 update_waypoint port.

    Reference: AP_L1_Control::update_waypoint()
    sine_Nu1 clamped to ±0.7071 (AP L1 line ~260)
    Nu clamped to ±π/2 (prevent_indecision equivalent)
    """
    out = GuidanceOutput(timestamp=now)

    # SAFE_GLIDE: no target or insufficient input
    if l1.target_N is None or inp.control_mode == ControlMode.SAFE_GLIDE:
        out.submode = "SAFE_GLIDE"
        out.reason  = "safe_glide"
        return out

    speed  = max(inp.ground_speed_mps or 0.0, l1.config.v_min)
    course = inp.course or 0.0
    pos_N  = inp.pos_N  or 0.0
    pos_E  = inp.pos_E  or 0.0

    cfg    = l1.config
    K_L1   = 4.0 * cfg.damping ** 2   # = 2.25 for damping=0.75
    L1_dist = max(cfg.damping * cfg.period / math.pi * speed, cfg.L1_min)

    # XTRACK thresholds
    xtrack_soft = max(XTRACK_SOFT_FACTOR * L1_dist, XTRACK_SOFT_MIN_M)
    xtrack_hard = max(XTRACK_HARD_FACTOR * L1_dist, XTRACK_HARD_MIN_M)

    # Path vector AB and vehicle offset AP
    if l1.submode == "DIRECT_TO_TARGET":
        AB_N = l1.target_N - pos_N
        AB_E = l1.target_E - pos_E
        AP_N = 0.0
        AP_E = 0.0
    else:
        AB_N = l1.target_N - l1.start_N
        AB_E = l1.target_E - l1.start_E
        AP_N = pos_N - l1.start_N
        AP_E = pos_E - l1.start_E

    AB_len = math.hypot(AB_N, AB_E)
    if AB_len < 0.1:
        out.reason = "start==target"
        return out

    unit_AB_N = AB_N / AB_len
    unit_AB_E = AB_E / AB_len

    alongTrack  = AP_N * unit_AB_N + AP_E * unit_AB_E
    crossTrack  = AP_N * unit_AB_E - AP_E * unit_AB_N   # + = left of path

    out.crossTrack  = crossTrack
    out.alongTrack  = alongTrack
    out.L1_dist     = L1_dist

    # DIRECT_TO_TARGET latch on hard xtrack violation
    if l1.submode != "DIRECT_TO_TARGET" and abs(crossTrack) > xtrack_hard:
        l1.submode = "DIRECT_TO_TARGET"

    # Velocity vector components along/across path
    vel_N = speed * math.cos(course)
    vel_E = speed * math.sin(course)

    # ArduPilot update_waypoint branches
    if (
        alongTrack < 0.0
        and abs(alongTrack) > L1_dist
        and l1.submode != "DIRECT_TO_TARGET"
    ):
        # Vehicle is behind start waypoint A — steer toward AB direction
        xtrackVel = vel_N * unit_AB_E - vel_E * unit_AB_N
        ltrackVel = vel_N * unit_AB_N + vel_E * unit_AB_E
        # sine_Nu1 clamped ±0.7071 (AP_L1_Control.cpp ~L260)
        Nu1 = math.asin(max(-0.7071, min(0.7071, crossTrack / L1_dist)))
        Nu2 = math.atan2(xtrackVel, ltrackVel)

    elif alongTrack > AB_len + speed * 3.0:
        # Vehicle has passed end waypoint B → steer directly to B
        to_B_N = l1.target_N - pos_N
        to_B_E = l1.target_E - pos_E
        dist_B = math.hypot(to_B_N, to_B_E)
        if dist_B > 0.1:
            unit_N = to_B_N / dist_B
            unit_E = to_B_E / dist_B
            xtrackVel = vel_N * unit_E - vel_E * unit_N
            ltrackVel = vel_N * unit_N + vel_E * unit_E
        else:
            xtrackVel, ltrackVel = 0.0, speed
        Nu1 = 0.0
        Nu2 = math.atan2(xtrackVel, ltrackVel)

    else:
        # Normal tracking — vehicle between A and B (or DIRECT_TO_TARGET)
        xtrackVel = vel_N * unit_AB_E - vel_E * unit_AB_N
        ltrackVel = vel_N * unit_AB_N + vel_E * unit_AB_E
        # sine_Nu1 clamped ±0.7071
        Nu1 = math.asin(max(-0.7071, min(0.7071, crossTrack / L1_dist)))
        Nu2 = math.atan2(xtrackVel, ltrackVel)

    Nu = Nu1 + Nu2
    # _prevent_indecision equivalent: clamp to ±π/2
    Nu = max(-math.pi / 2, min(math.pi / 2, Nu))

    # Lateral acceleration command
    lat_acc = K_L1 * speed ** 2 / L1_dist * math.sin(Nu)
    lat_acc = max(-LAT_ACC_MAX, min(LAT_ACC_MAX, lat_acc))

    # Course rate command (rad/s)
    course_rate = lat_acc / max(speed, V_MIN_MPS)
    course_rate = max(-COURSE_RATE_MAX, min(COURSE_RATE_MAX, course_rate))

    out.Nu1 = Nu1
    out.Nu2 = Nu2
    out.Nu  = Nu
    out.lat_acc_cmd_mps2     = lat_acc
    out.course_rate_cmd_rad_s = course_rate
    out.active   = True
    out.degraded = inp.control_mode in (
        ControlMode.DEGRADED_CLOSED_LOOP, ControlMode.DEGRADED_FEEDFORWARD
    )

    # Submode label for telemetry
    if l1.submode == "DIRECT_TO_TARGET":
        out.submode = "DIRECT_TO_TARGET"
    elif abs(crossTrack) > xtrack_soft:
        out.submode = "REJOIN"
    else:
        out.submode = "LINE_FOLLOW"

    return out


# ═══════════════════════════════════════════════════════════════════════════════
# compute_motor_output
# ═══════════════════════════════════════════════════════════════════════════════

def compute_motor_output(l1, controller, inp: GuidanceInput, now: float):
    """Run L1 guidance then brake controller.

    Returns (BrakeCommand, GuidanceOutput).
    Imports from control to avoid circular imports.
    """
    from Sensor_Motor.control import (
        BrakeCommand, GuidanceCommand, controller_update,
        LEFT_NEUTRAL, RIGHT_NEUTRAL, NEUTRAL_ARM_DEG,
    )

    g_out = l1_update(l1, inp, now)

    if inp.control_mode == ControlMode.SAFE_GLIDE or not g_out.active:
        cmd = BrakeCommand(timestamp=now)
        cmd.fallback_mode = "SAFE_GLIDE"
        cmd.mode          = "SAFE_GLIDE"
        return cmd, g_out

    use_closed_loop = inp.control_mode in (
        ControlMode.ACTIVE_CLOSED_LOOP, ControlMode.DEGRADED_CLOSED_LOOP
    )
    gyrz_meas_deg_s: float = float("nan")
    if use_closed_loop and inp.gyrz_health == SensorQuality.FRESH and inp.gyrz is not None:
        gyrz_meas_deg_s = math.degrees(inp.gyrz)

    guidance_cmd = GuidanceCommand(
        yaw_rate_cmd_deg_s = math.degrees(g_out.course_rate_cmd_rad_s),
        lat_acc_cmd_mps2   = g_out.lat_acc_cmd_mps2,
        ground_speed_mps   = inp.ground_speed_mps or 0.0,
        valid              = True,
        timestamp          = now,
    )
    brake_cmd = controller_update(controller, guidance_cmd, yaw_rate_meas_deg_s=gyrz_meas_deg_s, now=now)
    return brake_cmd, g_out


# ═══════════════════════════════════════════════════════════════════════════════
# Legacy guidance() function (for test_motor_guidance.py backwards compat)
# ═══════════════════════════════════════════════════════════════════════════════

def init_guidance() -> None:
    """Reset legacy module-level state."""
    global _PREV_GPS, _GPS_STABLE_COUNT, _START_LAT, _START_LON, START_POINT
    _PREV_GPS = SimpleNamespace(
        initialized=False,
        lat=0.0,
        lon=0.0,
        time=0.0,
    )
    _GPS_STABLE_COUNT = 0
    _START_LAT = None
    _START_LON = None
    START_POINT = SimpleNamespace(lat=0.0, lon=0.0)


def set_start_coordinates(lat: float, lon: float) -> None:
    global _START_LAT, _START_LON, START_POINT
    _START_LAT = float(lat)
    _START_LON = float(lon)
    START_POINT = SimpleNamespace(lat=float(lat), lon=float(lon))


def is_gps_jump(lat: float, lon: float) -> bool:
    """Legacy API: returns True only when GPS is considered stable."""
    return _check_gps_jump(float(lat), float(lon), time.time())


def _check_gps_jump(lat: float, lon: float, ts: float) -> bool:
    """GPS jump gate — returns True when GPS is stable enough to use.

    Requires GPS_STABLE_COUNT_REQUIRED consecutive readings within 0.01 deg
    and at least 0.5 s apart.
    """
    global _PREV_GPS, _GPS_STABLE_COUNT

    MAX_JUMP_DEG = 0.01   # ~1 km

    if not _PREV_GPS.initialized:
        _PREV_GPS.initialized = True
        _PREV_GPS.lat  = lat
        _PREV_GPS.lon  = lon
        _PREV_GPS.time = ts
        _GPS_STABLE_COUNT = 0
        return False

    dt = ts - _PREV_GPS.time
    if dt < 0.5:
        # Too soon — don't count
        return _GPS_STABLE_COUNT >= GPS_STABLE_COUNT_REQUIRED

    dist_deg = math.hypot(lat - _PREV_GPS.lat, lon - _PREV_GPS.lon)
    if dist_deg <= MAX_JUMP_DEG:
        _GPS_STABLE_COUNT += 1
    else:
        _GPS_STABLE_COUNT = 0

    _PREV_GPS.lat  = lat
    _PREV_GPS.lon  = lon
    _PREV_GPS.time = ts

    return _GPS_STABLE_COUNT >= GPS_STABLE_COUNT_REQUIRED


def guidance(imu, gps, fid, tgt, alt: float):
    """Legacy guidance function for backwards compatibility.

    Parameters
    ----------
    imu : namespace with .yaw, .gyrz
    gps : namespace with .lat, .lon, .direction, .velocity
    fid : namespace with .pos_health, .motion_health
    tgt : namespace with .lat, .lon, or None
    alt : float, altitude in metres
    """
    ts = time.time()

    out = SimpleNamespace(
        state="GPS_INVALID",
        distance=float("nan"),
        commanded_yaw_rate=0.0,
    )

    # GPS health check
    if not fid.pos_health:
        return out

    lat = float(gps.lat)
    lon = float(gps.lon)

    # GPS jump gate
    if not _check_gps_jump(lat, lon, ts):
        return out

    # Target check
    if tgt is None:
        out.state = "TARGET_UNSET"
        out.distance = float("nan")
        return out

    # Start point check (legacy safety contract)
    if _START_LAT is None or _START_LON is None:
        out.state = "START_UNSET"
        out.distance = float("nan")
        return out

    # Distance to target
    tgt_lat = float(tgt.lat)
    tgt_lon = float(tgt.lon)
    distance_m = _haversine_distance_m(lat, lon, tgt_lat, tgt_lon)
    out.distance = distance_m

    if distance_m < 5.0:
        out.state = "TARGET_REACHED"
        out.commanded_yaw_rate = 0.0
        return out

    # Phase 1 carrot-chasing formula
    bearing_to_carrot_deg = _haversine_bearing_deg(lat, lon, tgt_lat, tgt_lon)
    gps_track_deg         = float(gps.direction)

    eta_deg = _wrap_180(bearing_to_carrot_deg - gps_track_deg)
    eta_deg = max(-90.0, min(90.0, eta_deg))

    V = max(float(gps.velocity), 0.5)
    accel_lat = 2.0 * V ** 2 / L_DISTANCE * math.sin(math.radians(eta_deg))
    desired_yaw_rate = math.degrees(accel_lat / V)

    # Landing altitude cap
    yr_max = LANDING_YR_MAX if alt < 20.0 else YR_MAX
    desired_yaw_rate = max(-yr_max, min(yr_max, desired_yaw_rate))

    # Heading state label
    if abs(eta_deg) < 5.0:
        state_label = "STRAIGHT"
    elif abs(eta_deg) < 45.0:
        state_label = "TURNING"
    else:
        state_label = "PATTERN"

    out.state              = state_label
    out.commanded_yaw_rate = desired_yaw_rate
    return out
