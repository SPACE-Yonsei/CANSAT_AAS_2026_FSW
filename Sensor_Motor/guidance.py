"""Sensor_Motor/guidance.py – L1 target-fixed homing guidance pipeline.

Pipeline: decidefresh -> produceL1input -> produceL1output
Internal units: distance=m, time=s, angle=rad, speed=m/s
"""

from __future__ import annotations

import math
import logging
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional

from lib import config

logger = logging.getLogger(__name__)

EARTH_RADIUS_M = 6_371_000.0


# ── Enums ──────────────────────────────────────────────────────────────────────

class ControlMode(str, Enum):
    GPS_TRACKING_CLOSED = "GPS_TRACKING_CLOSED"
    GPS_TRACKING_OPEN   = "GPS_TRACKING_OPEN"
    DR_TRACKING_CLOSED  = "DR_TRACKING_CLOSED"
    DR_TRACKING_OPEN    = "DR_TRACKING_OPEN"
    DETUMBLING          = "DETUMBLING"
    FAIL                = "FAIL"


class DRMethod(str, Enum):
    NONE                   = "NONE"
    GYRO_INTEGRATION       = "GYRO_INTEGRATION"
    ACC_DOUBLE_INTEGRATION = "ACC_DOUBLE_INTEGRATION"
    GYRO_ACC_BLEND         = "GYRO_ACC_BLEND"


# ── Utility functions ─────────────────────────────────────────────────────────

def latlon_to_ne(lat: float, lon: float, origin_lat: float, origin_lon: float):
    """Return (N, E) metres relative to origin."""
    dLat = math.radians(lat - origin_lat)
    dLon = math.radians(lon - origin_lon)
    N = dLat * EARTH_RADIUS_M
    E = dLon * EARTH_RADIUS_M * math.cos(math.radians(origin_lat))
    return N, E


def ne_to_latlon(N: float, E: float, origin_lat: float, origin_lon: float):
    """Return (lat, lon) from local NE metres."""
    lat = origin_lat + math.degrees(N / EARTH_RADIUS_M)
    lon = origin_lon + math.degrees(
        E / (EARTH_RADIUS_M * math.cos(math.radians(origin_lat)))
    )
    return lat, lon


def convert_latlon_to_local_en(lat: float, lon: float, origin_lat: float, origin_lon: float):
    """Return (E, N) metres relative to origin."""
    N, E = latlon_to_ne(lat, lon, origin_lat, origin_lon)
    return E, N


def wrap_pi(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def saturated_sin(nu: float) -> float:
    """Sine saturated at ±sin(π/4) ≈ ±0.7071."""
    return clamp(math.sin(nu), -0.7071067811865476, 0.7071067811865476)


def compute_dr_confidence(dr_age: float) -> float:
    a1 = config.DR_CONF_AGE_1_S
    a2 = config.DR_CONF_AGE_2_S
    a3 = config.DR_CONF_AGE_3_S
    if dr_age <= a1:
        return 1.0
    if dr_age <= a2:
        return 1.0 - 0.5 * (dr_age - a1) / max(a2 - a1, 1e-6)
    if dr_age <= a3:
        return 0.5 * (1.0 - (dr_age - a2) / max(a3 - a2, 1e-6))
    return 0.0


def choose_yaw_rate_limit(control_mode, dr_method=None) -> float:
    """Return yaw rate limit in rad/s for the given ControlMode."""
    if control_mode == ControlMode.GPS_TRACKING_CLOSED:
        return math.radians(config.GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS)
    if control_mode == ControlMode.GPS_TRACKING_OPEN:
        return math.radians(config.GPS_TRACKING_OPEN_YAW_RATE_LIMIT_DPS)
    if control_mode == ControlMode.DR_TRACKING_CLOSED:
        return math.radians(config.DR_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS)
    if control_mode == ControlMode.DR_TRACKING_OPEN:
        return math.radians(config.DR_TRACKING_OPEN_YAW_RATE_LIMIT_DPS)
    if control_mode == ControlMode.DETUMBLING:
        return math.radians(config.DETUMBLING_YAW_RATE_LIMIT_DPS)
    return math.radians(config.FAIL_YAW_RATE_LIMIT_DPS)


# ── Sample dataclasses ────────────────────────────────────────────────────────

@dataclass
class GpsSample:
    """One GPS message: position + velocity snapshot.

    pos_ts / pos_valid  : position fix data
    motion_ts / motion_valid : course+speed data (may be absent)
    timestamp property  : equals pos_ts (used by _prune_history)
    """
    lat: float = 0.0
    lon: float = 0.0
    point_E: float = float("nan")
    point_N: float = float("nan")
    pos_ts: float = 0.0
    pos_valid: bool = False
    course_rad: float = float("nan")
    speed_mps: float = float("nan")
    motion_ts: float = float("nan")
    motion_valid: bool = False

    @property
    def timestamp(self) -> float:
        return self.pos_ts

    def has_valid_position(self) -> bool:
        return self.pos_valid and _ok(self.lat) and _ok(self.lon) and _ok(self.pos_ts)

    def has_valid_point(self) -> bool:
        return self.has_valid_position() and _ok(self.point_E) and _ok(self.point_N)

    def has_valid_velocity(self) -> bool:
        return (
            self.motion_valid
            and _ok(self.course_rad)
            and _ok(self.speed_mps)
            and _ok(self.motion_ts)
        )

    def has_valid_nav(self) -> bool:
        return self.has_valid_point() and self.has_valid_velocity()


@dataclass
class ImuSample:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    acc_x: float = 0.0
    acc_y: float = 0.0
    acc_z: float = 0.0
    gyr_x: float = 0.0
    gyr_y: float = 0.0
    gyr_z: float = 0.0
    mag_x: float = 0.0
    mag_y: float = 0.0
    mag_z: float = 0.0
    timestamp: float = 0.0
    lin_acc_x: float = 0.0
    lin_acc_y: float = 0.0
    lin_acc_z: float = 0.0
    quat_w: float = 1.0
    quat_x: float = 0.0
    quat_y: float = 0.0
    quat_z: float = 0.0
    gyrz_valid: bool = False
    yaw_valid: bool = False
    acc_valid: bool = False
    lin_acc_valid: bool = False
    freefall: bool = False
    tumble: bool = False


@dataclass
class BarometerSample:
    altitude: float = 0.0
    timestamp: float = 0.0
    pressure: Optional[float] = None
    valid: bool = False


# ── FreshResult ───────────────────────────────────────────────────────────────

@dataclass
class FreshResult:
    point_fresh: bool = False
    point_age_s: float = float("inf")
    velocity_fresh: bool = False
    velocity_age_s: float = float("inf")
    imu_fresh: bool = False
    imu_age_s: float = float("inf")
    imu_gyrz_fresh: bool = False
    imu_yaw_fresh: bool = False
    imu_acc_fresh: bool = False
    imu_linear_acc_fresh: bool = False
    barometer_fresh: bool = False
    baro_age_s: float = float("inf")

    @property
    def gyrz_fresh(self) -> bool:
        return self.imu_gyrz_fresh

    @property
    def gyrz_age_s(self) -> float:
        return self.imu_age_s

    @property
    def baro_fresh(self) -> bool:
        return self.barometer_fresh


# ── GuidanceState ─────────────────────────────────────────────────────────────

@dataclass
class GuidanceState:
    # Origin
    origin_lat: float = 0.0
    origin_lon: float = 0.0
    origin_ready: bool = False

    # Target
    target_lat: float = 0.0
    target_lon: float = 0.0
    target_E: float = float("nan")
    target_N: float = float("nan")
    target_ready: bool = False

    # Sensor histories (ring buffer, pruned to HISTORY_WINDOW_S)
    # last_fresh_* 값은 별도 저장하지 않고 history에서 조회한다.
    gps_history:  List[GpsSample]      = field(default_factory=list)
    imu_history:  List[ImuSample]      = field(default_factory=list)
    baro_history: List[BarometerSample] = field(default_factory=list)

    # Current navigation estimate (updated every cycle)
    nav_E: float = float("nan")
    nav_N: float = float("nan")
    nav_course: float = float("nan")
    nav_V: float = float("nan")
    nav_vE: float = float("nan")
    nav_vN: float = float("nan")
    nav_confidence: float = 0.0
    nav_dr_age: float = 0.0
    nav_control_mode: ControlMode = ControlMode.FAIL
    nav_dr_method: DRMethod = DRMethod.NONE

    # DR anchor – snapshot at GPS dropout; persists beyond history window
    dr_start_E: float = float("nan")
    dr_start_N: float = float("nan")
    dr_start_vE: float = float("nan")
    dr_start_vN: float = float("nan")
    dr_start_V: float = float("nan")
    dr_start_course: float = float("nan")   # replaces course_at_dropout
    dr_start_time: float = float("nan")     # wall-clock when GPS was last fresh

    # DR course estimation (accumulated since dropout)
    yaw_at_dropout: float = float("nan")
    gyro_integral_since_dropout: float = 0.0

    # DR step timing
    last_dr_update_time: float = float("nan")

    # Detumbling exit hold timer
    detumble_exit_start: float = float("nan")


# ── L1Input / L1Output ────────────────────────────────────────────────────────

@dataclass
class L1Input:
    valid: bool = False
    reason: str = "INIT"
    control_mode: ControlMode = ControlMode.FAIL
    dr_method: DRMethod = DRMethod.NONE
    confidence: float = 0.0
    E: float = float("nan")
    N: float = float("nan")
    vE: float = float("nan")
    vN: float = float("nan")
    V: float = float("nan")
    course: float = float("nan")
    origin_E: float = 0.0
    origin_N: float = 0.0
    target_E: float = float("nan")
    target_N: float = float("nan")
    target_lat: float = 0.0
    target_lon: float = 0.0
    point_age: float = float("inf")
    velocity_age: float = float("inf")
    imu_age: float = float("inf")
    barometer_age: float = float("inf")
    dr_age: float = 0.0


@dataclass
class L1Output:
    timestamp: float = 0.0
    control_valid: bool = False
    nominal: bool = False
    reason: str = "INIT"
    control_mode: ControlMode = ControlMode.FAIL
    dr_method: DRMethod = DRMethod.NONE
    confidence: float = 0.0
    origin_E: float = 0.0
    origin_N: float = 0.0
    target_E: float = float("nan")
    target_N: float = float("nan")
    target_bearing: float = float("nan")
    nu: float = float("nan")
    distance_to_target: float = float("nan")
    yaw_rate_cmd: float = 0.0
    yaw_rate_limit_dps: float = 0.0
    angular_velocity_cmd_rad_s: float = 0.0   # alias; always == yaw_rate_cmd
    ground_speed_mps: float = 0.0
    crossTrack: float = 0.0
    alongTrack: float = float("nan")
    pos_E: float = float("nan")
    pos_N: float = float("nan")
    carrot_E: float = float("nan")
    carrot_N: float = float("nan")
    current_heading_rad: float = float("nan")
    pid_enabled: bool = False
    kp_override: Optional[float] = None


# ── Internal helpers ──────────────────────────────────────────────────────────

def _prune_history(history: list, oldest_ts: float) -> None:
    i = 0
    while i < len(history) and history[i].timestamp < oldest_ts:
        i += 1
    del history[:i]


def _latest(history: list):
    return history[-1] if history else None


def _ok(v) -> bool:
    try:
        return math.isfinite(float(v))
    except (TypeError, ValueError):
        return False


# ── History query helpers (replaces last_fresh_* stored fields) ───────────────

def _last_valid_position(state: GuidanceState) -> Optional[GpsSample]:
    """Last GpsSample with a valid lat/lon fix."""
    for s in reversed(state.gps_history):
        if s.has_valid_position():
            return s
    return None


def _last_valid_point(state: GuidanceState) -> Optional[GpsSample]:
    """Last GpsSample with a valid projected position (point_E/N finite)."""
    for s in reversed(state.gps_history):
        if s.has_valid_point():
            return s
    return None


def _last_valid_velocity(state: GuidanceState) -> Optional[GpsSample]:
    """Last GpsSample with valid course + speed."""
    for s in reversed(state.gps_history):
        if s.has_valid_velocity():
            return s
    return None


def _last_valid_gps_sample(state: GuidanceState) -> Optional[GpsSample]:
    """Last GpsSample usable as a complete GPS navigation snapshot."""
    for s in reversed(state.gps_history):
        if s.has_valid_nav():
            return s
    return None


# ── decidefresh ───────────────────────────────────────────────────────────────

def decidefresh(gps, imu, baro, state: GuidanceState, now: float) -> FreshResult:
    """Ingest sensor cache objects into history and return freshness flags.

    Validity of data values is the sensor app's responsibility.
    decidefresh only judges *staleness*: is the timestamp recent enough?
    """
    res = FreshResult()
    window_start = now - config.HISTORY_WINDOW_S

    # ── GPS (position + velocity in one sample) ───────────────────────────────
    pos_ts = getattr(gps, "pos_ts", None)
    if pos_ts is not None and _ok(pos_ts):
        lat      = getattr(gps, "lat",      None)
        lon      = getattr(gps, "lon",      None)
        # pos_valid: sensor app already validated; we just check values are present
        pos_valid = lat is not None and lon is not None and _ok(lat) and _ok(lon)

        pE = pN = float("nan")
        if pos_valid and state.origin_ready:
            pE, pN = convert_latlon_to_local_en(float(lat), float(lon),
                                                state.origin_lat, state.origin_lon)

        # velocity (None when sensor app reports no valid motion fix)
        motion_ts = getattr(gps, "motion_ts", None)
        course_r  = getattr(gps, "course_rad", None)
        spd       = getattr(gps, "speed_mps",  None)
        motion_valid = (
            motion_ts is not None and _ok(motion_ts)
            and course_r  is not None and _ok(course_r)
            and spd       is not None and _ok(spd)
        )

        state.gps_history.append(GpsSample(
            lat=float(lat) if pos_valid else 0.0,
            lon=float(lon) if pos_valid else 0.0,
            point_E=pE, point_N=pN,
            pos_ts=float(pos_ts), pos_valid=pos_valid,
            course_rad=float(course_r) if motion_valid else float("nan"),
            speed_mps =float(spd)      if motion_valid else float("nan"),
            motion_ts =float(motion_ts) if motion_valid else float("nan"),
            motion_valid=motion_valid,
        ))

    _prune_history(state.gps_history, window_start)

    lp_gps = _last_valid_position(state)
    if lp_gps is not None:
        res.point_age_s = now - lp_gps.pos_ts
        res.point_fresh = res.point_age_s <= config.GPS_FRESH_MAX_AGE_S

    lv_gps = _last_valid_velocity(state)
    if lv_gps is not None:
        res.velocity_age_s = now - lv_gps.motion_ts
        res.velocity_fresh = res.velocity_age_s <= config.GPS_FRESH_MAX_AGE_S

    # ── IMU ───────────────────────────────────────────────────────────────────
    imu_ts = getattr(imu, "ts", None)
    if imu_ts is None:
        imu_ts = getattr(imu, "rx_ts", None)
    if imu_ts is not None and _ok(imu_ts):
        # All field values are trusted as valid if present (sensor app responsibility)
        roll_r  = getattr(imu, "roll_rad",  None)
        pitch_r = getattr(imu, "pitch_rad", None)
        yaw_r   = getattr(imu, "yaw_rad",   None)
        ax  = getattr(imu, "accx_mps2",  None)
        ay  = getattr(imu, "accy_mps2",  None)
        az  = getattr(imu, "accz_mps2",  None)
        gx  = getattr(imu, "gyrx_rad_s", None)
        gy  = getattr(imu, "gyry_rad_s", None)
        gz  = getattr(imu, "gyrz_rad_s", None)
        mx  = getattr(imu, "magx_uT",    None)
        my  = getattr(imu, "magy_uT",    None)
        mz  = getattr(imu, "magz_uT",    None)
        lax = getattr(imu, "lin_acc_x",  None)
        lay = getattr(imu, "lin_acc_y",  None)
        laz = getattr(imu, "lin_acc_z",  None)
        # Validity flags come from the sensor/motorapp layer
        gyrz_valid    = gz  is not None
        yaw_valid     = yaw_r is not None
        acc_valid     = ax  is not None and ay is not None and az is not None
        lin_acc_valid = bool(getattr(imu, "lin_acc_valid", False))
        freefall      = bool(getattr(imu, "freefall", 0))
        tumble        = bool(getattr(imu, "tumble",   0))

        state.imu_history.append(ImuSample(
            roll      = float(roll_r)  if roll_r  is not None else 0.0,
            pitch     = float(pitch_r) if pitch_r is not None else 0.0,
            yaw       = float(yaw_r)   if yaw_r   is not None else 0.0,
            acc_x     = float(ax) if ax is not None else 0.0,
            acc_y     = float(ay) if ay is not None else 0.0,
            acc_z     = float(az) if az is not None else 0.0,
            gyr_x     = float(gx) if gx is not None else 0.0,
            gyr_y     = float(gy) if gy is not None else 0.0,
            gyr_z     = float(gz) if gz is not None else 0.0,
            mag_x     = float(mx) if mx is not None else 0.0,
            mag_y     = float(my) if my is not None else 0.0,
            mag_z     = float(mz) if mz is not None else 0.0,
            timestamp = float(imu_ts),
            lin_acc_x = float(lax) if lax is not None else 0.0,
            lin_acc_y = float(lay) if lay is not None else 0.0,
            lin_acc_z = float(laz) if laz is not None else 0.0,
            gyrz_valid    = gyrz_valid,
            yaw_valid     = yaw_valid,
            acc_valid     = acc_valid,
            lin_acc_valid = lin_acc_valid,
            freefall      = freefall,
            tumble        = tumble,
        ))

    _prune_history(state.imu_history, window_start)
    li = _latest(state.imu_history)
    if li is not None:
        res.imu_age_s = now - li.timestamp
        res.imu_fresh = res.imu_age_s <= config.IMU_FRESH_MAX_AGE_S
        if res.imu_fresh:
            res.imu_gyrz_fresh       = li.gyrz_valid
            res.imu_yaw_fresh        = li.yaw_valid
            res.imu_acc_fresh        = li.acc_valid
            res.imu_linear_acc_fresh = li.lin_acc_valid

    # ── BAROMETER ─────────────────────────────────────────────────────────────
    baro_ts = getattr(baro, "ts", None)
    if baro_ts is not None and _ok(baro_ts):
        alt  = getattr(baro, "alt_m",  None)
        pres = getattr(baro, "pressure_hpa", None)
        # alt present → valid (sensor app checked)
        baro_valid = alt is not None
        state.baro_history.append(BarometerSample(
            altitude  = float(alt) if baro_valid else 0.0,
            timestamp = float(baro_ts),
            pressure  = float(pres) if pres is not None else None,
            valid     = baro_valid,
        ))

    _prune_history(state.baro_history, window_start)
    lb = _latest(state.baro_history)
    if lb is not None:
        res.baro_age_s = now - lb.timestamp
        res.barometer_fresh = res.baro_age_s <= config.BARO_FRESH_MAX_AGE_S and lb.valid

    return res


# ── DR helpers ────────────────────────────────────────────────────────────────

def _can_dead_reckon(state: GuidanceState, fresh: FreshResult) -> bool:
    """True when a GPS anchor is locked and IMU is available for course estimation."""
    return (
        _ok(state.dr_start_E) and _ok(state.dr_start_N)
        and _ok(state.dr_start_V) and _ok(state.dr_start_course)
        and _ok(state.dr_start_time)
        and (fresh.imu_yaw_fresh or fresh.imu_gyrz_fresh)
    )


def _circular_mean(a: float, b: float) -> float:
    return math.atan2(math.sin(a) + math.sin(b), math.cos(a) + math.cos(b))


def _estimate_dr_course(state: GuidanceState, li: Optional[ImuSample],
                         fresh: FreshResult) -> float:
    """Estimate current heading from DR anchor + IMU."""
    base = state.dr_start_course   # was course_at_dropout
    if not _ok(base):
        return 0.0
    have_yaw  = fresh.imu_yaw_fresh and li is not None and _ok(state.yaw_at_dropout)
    have_gyro = fresh.imu_gyrz_fresh
    course_yaw = course_gyro = None
    if have_yaw:
        course_yaw  = wrap_pi(base + wrap_pi(li.yaw - state.yaw_at_dropout))
    if have_gyro:
        course_gyro = wrap_pi(base + state.gyro_integral_since_dropout)
    if course_yaw is not None and course_gyro is not None:
        residual = abs(wrap_pi(course_yaw - course_gyro))
        return _circular_mean(course_yaw, course_gyro) if residual < math.pi / 4 else course_gyro
    if course_yaw  is not None: return course_yaw
    if course_gyro is not None: return course_gyro
    return base


def _update_state_from_dead_reckoning(state: GuidanceState,
                                       fresh: FreshResult, now: float) -> None:
    li = _latest(state.imu_history)

    # dt for this integration step
    if _ok(state.last_dr_update_time):
        dt = now - state.last_dr_update_time
    elif _ok(state.dr_start_time):
        dt = now - state.dr_start_time   # first DR step
    else:
        dt = 0.0
    dt = clamp(dt, 0.0, 0.5)

    total_age = (now - state.dr_start_time) if _ok(state.dr_start_time) else 0.0

    # Initialise nav position from DR anchor on first step
    if not _ok(state.nav_E) or not _ok(state.nav_N):
        state.nav_E = state.dr_start_E
        state.nav_N = state.dr_start_N

    # Gyro integration
    if fresh.imu_gyrz_fresh and li is not None:
        state.gyro_integral_since_dropout += li.gyr_z * config.GYRZ_SIGN * dt

    course_est = _estimate_dr_course(state, li, fresh)

    # Speed decay
    V_dr = state.dr_start_V * math.exp(-total_age / max(config.SPEED_DECAY_TAU_S, 1e-6))
    V_dr = max(0.0, V_dr)

    # Base EN velocity from gyro-speed DR
    vE = V_dr * math.sin(course_est)
    vN = V_dr * math.cos(course_est)

    # Accelerometer blend (optional)
    acc_ok = False
    if (config.USE_ACC_DOUBLE_INTEGRATION
            and fresh.imu_linear_acc_fresh
            and li is not None
            and config.ACC_AID_START_AGE_S <= total_age <= config.ACC_AID_END_AGE_S):
        lax = li.lin_acc_x * config.ACC_X_SIGN
        lay = li.lin_acc_y * config.ACC_Y_SIGN
        if math.hypot(lax, lay) <= config.ACC_LIMIT_MPS2:
            # body-x=fwd, body-y=right  →  EN frame
            aE = lax * math.sin(course_est) + lay * math.cos(course_est)
            aN = lax * math.cos(course_est) - lay * math.sin(course_est)
            vE += config.ACC_BLEND_WEIGHT * aE * dt
            vN += config.ACC_BLEND_WEIGHT * aN * dt
            acc_ok = True

    state.nav_E += vE * dt
    state.nav_N += vN * dt
    state.nav_vE = vE
    state.nav_vN = vN
    state.nav_V  = V_dr
    state.nav_course   = course_est
    state.nav_dr_method = DRMethod.GYRO_ACC_BLEND if acc_ok else DRMethod.GYRO_INTEGRATION
    state.last_dr_update_time = now


# ── Public: reset / convert ───────────────────────────────────────────────────

def convert_target_to_local_en_if_possible(state: GuidanceState) -> None:
    """Project target lat/lon to local EN once origin is ready."""
    if not state.origin_ready or state.target_ready:
        return
    if not (_ok(state.target_lat) and _ok(state.target_lon)):
        return
    if abs(state.target_lat) < 1e-9 and abs(state.target_lon) < 1e-9:
        return
    E, N = convert_latlon_to_local_en(
        state.target_lat, state.target_lon,
        state.origin_lat, state.origin_lon,
    )
    state.target_E = E
    state.target_N = N
    state.target_ready = True
    logger.info("Target projected: E=%.1f N=%.1f (lat=%.6f lon=%.6f)",
                E, N, state.target_lat, state.target_lon)


def reset_guidance_state_for_flight(state: GuidanceState) -> None:
    """Reset flight nav state. Target lat/lon preserved; origin cleared."""
    state.origin_lat   = 0.0
    state.origin_lon   = 0.0
    state.origin_ready = False
    state.gps_history.clear()
    state.imu_history.clear()
    state.baro_history.clear()
    nan = float("nan")
    state.nav_E = nan; state.nav_N = nan; state.nav_course = nan
    state.nav_V = nan; state.nav_vE = nan; state.nav_vN = nan
    state.nav_confidence = 0.0; state.nav_dr_age = 0.0
    state.nav_control_mode = ControlMode.FAIL; state.nav_dr_method = DRMethod.NONE
    state.dr_start_E = nan; state.dr_start_N = nan
    state.dr_start_vE = nan; state.dr_start_vN = nan
    state.dr_start_V = nan; state.dr_start_course = nan; state.dr_start_time = nan
    state.yaw_at_dropout = nan
    state.gyro_integral_since_dropout = 0.0
    state.last_dr_update_time = nan
    state.detumble_exit_start = nan
    if state.target_lat or state.target_lon:
        state.target_ready = False


# ── produceL1input ────────────────────────────────────────────────────────────

def _fail_l1input(reason: str, state: GuidanceState, fresh: FreshResult) -> L1Input:
    return L1Input(
        valid=False, reason=reason,
        control_mode=ControlMode.FAIL,
        point_age=fresh.point_age_s,
        velocity_age=fresh.velocity_age_s,
        imu_age=fresh.imu_age_s,
        barometer_age=fresh.baro_age_s,
        target_lat=state.target_lat, target_lon=state.target_lon,
        target_E=state.target_E,    target_N=state.target_N,
    )


def _should_detumble(state: GuidanceState, fresh: FreshResult, now: float) -> bool:
    if not config.DETUMBLE_ENABLE or not fresh.imu_gyrz_fresh:
        return False
    li = _latest(state.imu_history)
    if li is None:
        return False
    gyrz_dps = abs(math.degrees(li.gyr_z))
    currently = (state.nav_control_mode == ControlMode.DETUMBLING)
    if currently:
        if gyrz_dps <= config.DETUMBLE_EXIT_THRESHOLD_DPS:
            if not _ok(state.detumble_exit_start):
                state.detumble_exit_start = now
            elif now - state.detumble_exit_start >= config.DETUMBLE_EXIT_HOLD_S:
                state.detumble_exit_start = float("nan")
                return False
        else:
            state.detumble_exit_start = float("nan")
        return True
    else:
        if gyrz_dps >= config.DETUMBLE_GYRZ_THRESHOLD_DPS:
            state.detumble_exit_start = float("nan")
            return True
        return False


def produceL1input(fresh: FreshResult, gps, imu,
                   state: GuidanceState, release_state: int, now: float) -> L1Input:
    """Update nav state from sensor data and return L1Input for produceL1output."""

    # ── Gate ─────────────────────────────────────────────────────────────────
    if release_state < 3 and not state.origin_ready:
        return _fail_l1input("WAIT_RELEASE_STATE_3", state, fresh)

    # ── Origin acquisition (one-time from first fresh GPS point) ─────────────
    if not state.origin_ready and fresh.point_fresh:
        lp = _last_valid_position(state)
        if lp is not None:
            state.origin_lat   = lp.lat
            state.origin_lon   = lp.lon
            state.origin_ready = True
            logger.info("Origin set: lat=%.6f lon=%.6f", lp.lat, lp.lon)
            # back-project existing history
            for s in state.gps_history:
                if s.pos_valid:
                    s.point_E, s.point_N = convert_latlon_to_local_en(
                        s.lat, s.lon, state.origin_lat, state.origin_lon)
            convert_target_to_local_en_if_possible(state)

    if not state.origin_ready:
        return _fail_l1input("FAIL_NO_ORIGIN", state, fresh)

    # ── Target ────────────────────────────────────────────────────────────────
    if not state.target_ready:
        convert_target_to_local_en_if_possible(state)
    if not state.target_ready:
        return _fail_l1input("FAIL_NO_TARGET", state, fresh)

    # ── Detumbling ────────────────────────────────────────────────────────────
    if _should_detumble(state, fresh, now):
        state.nav_control_mode = ControlMode.DETUMBLING
        return L1Input(
            valid=True, reason="DETUMBLING",
            control_mode=ControlMode.DETUMBLING, dr_method=DRMethod.NONE,
            confidence=1.0,
            E=state.nav_E, N=state.nav_N,
            vE=state.nav_vE, vN=state.nav_vN, V=state.nav_V,
            course=state.nav_course,
            origin_E=0.0, origin_N=0.0,
            target_E=state.target_E, target_N=state.target_N,
            target_lat=state.target_lat, target_lon=state.target_lon,
            point_age=fresh.point_age_s, velocity_age=fresh.velocity_age_s,
            imu_age=fresh.imu_age_s, barometer_age=fresh.baro_age_s,
            dr_age=state.nav_dr_age,
        )

    # ── GPS TRACKING ──────────────────────────────────────────────────────────
    if fresh.point_fresh and fresh.velocity_fresh:
        lg = _last_valid_gps_sample(state)
        li = _latest(state.imu_history)
        if lg is None:
            return _fail_l1input("FAIL_NO_VALID_GPS_SAMPLE", state, fresh)
        if (now - lg.pos_ts > config.GPS_FRESH_MAX_AGE_S
                or now - lg.motion_ts > config.GPS_FRESH_MAX_AGE_S):
            return _fail_l1input("FAIL_NO_FRESH_GPS_SAMPLE", state, fresh)

        state.nav_E      = lg.point_E
        state.nav_N      = lg.point_N
        state.nav_course = lg.course_rad
        state.nav_V      = lg.speed_mps
        state.nav_vE     = lg.speed_mps * math.sin(lg.course_rad)
        state.nav_vN     = lg.speed_mps * math.cos(lg.course_rad)
        state.nav_confidence = 1.0
        state.nav_dr_age     = 0.0
        state.nav_dr_method  = DRMethod.NONE

        # Lock DR anchor from current GPS state
        state.dr_start_E      = state.nav_E
        state.dr_start_N      = state.nav_N
        state.dr_start_V      = state.nav_V
        state.dr_start_course = state.nav_course   # used by _estimate_dr_course
        state.dr_start_vE     = state.nav_vE
        state.dr_start_vN     = state.nav_vN
        state.dr_start_time   = now
        state.gyro_integral_since_dropout = 0.0
        state.last_dr_update_time = float("nan")
        state.yaw_at_dropout = li.yaw if (li is not None and li.yaw_valid) else float("nan")

        mode = (ControlMode.GPS_TRACKING_CLOSED
                if fresh.imu_gyrz_fresh else ControlMode.GPS_TRACKING_OPEN)
        state.nav_control_mode = mode

        return L1Input(
            valid=True, reason="GPS_TRACKING",
            control_mode=mode, dr_method=DRMethod.NONE, confidence=1.0,
            E=state.nav_E, N=state.nav_N,
            vE=state.nav_vE, vN=state.nav_vN, V=state.nav_V,
            course=state.nav_course,
            origin_E=0.0, origin_N=0.0,
            target_E=state.target_E, target_N=state.target_N,
            target_lat=state.target_lat, target_lon=state.target_lon,
            point_age=fresh.point_age_s, velocity_age=fresh.velocity_age_s,
            imu_age=fresh.imu_age_s, barometer_age=fresh.baro_age_s,
            dr_age=0.0,
        )

    # ── DEAD RECKONING ────────────────────────────────────────────────────────
    if not _can_dead_reckon(state, fresh):
        return _fail_l1input("FAIL_NO_VALID_DR", state, fresh)

    _update_state_from_dead_reckoning(state, fresh, now)

    dr_age     = now - state.dr_start_time   # wall-clock age since last GPS
    confidence = compute_dr_confidence(dr_age)
    state.nav_dr_age = dr_age

    if confidence <= 0.0:
        state.nav_confidence = 0.0
        return _fail_l1input("FAIL_DR_CONFIDENCE_ZERO", state, fresh)

    state.nav_confidence = confidence
    mode = (ControlMode.DR_TRACKING_CLOSED
            if fresh.imu_gyrz_fresh else ControlMode.DR_TRACKING_OPEN)
    state.nav_control_mode = mode

    return L1Input(
        valid=True, reason="DR_TRACKING",
        control_mode=mode, dr_method=state.nav_dr_method,
        confidence=confidence,
        E=state.nav_E, N=state.nav_N,
        vE=state.nav_vE, vN=state.nav_vN, V=state.nav_V,
        course=state.nav_course,
        origin_E=0.0, origin_N=0.0,
        target_E=state.target_E, target_N=state.target_N,
        target_lat=state.target_lat, target_lon=state.target_lon,
        point_age=fresh.point_age_s, velocity_age=fresh.velocity_age_s,
        imu_age=fresh.imu_age_s, barometer_age=fresh.baro_age_s,
        dr_age=dr_age,
    )


# ── produceL1output ───────────────────────────────────────────────────────────

def produceL1output(l1input: L1Input) -> L1Output:
    """Compute target_bearing, nu, yaw_rate_cmd from L1Input. Pure computation."""
    base = L1Output(
        control_mode=l1input.control_mode,
        dr_method=l1input.dr_method,
        confidence=l1input.confidence,
        origin_E=l1input.origin_E, origin_N=l1input.origin_N,
        target_E=l1input.target_E, target_N=l1input.target_N,
        carrot_E=l1input.target_E, carrot_N=l1input.target_N,
        pos_E=l1input.E, pos_N=l1input.N,
        current_heading_rad=l1input.course,
        ground_speed_mps=l1input.V if _ok(l1input.V) else 0.0,
    )

    # ── Invalid / FAIL ────────────────────────────────────────────────────────
    if not l1input.valid:
        base.control_valid = False
        base.nominal       = False
        base.reason        = l1input.reason
        base.yaw_rate_cmd  = 0.0
        base.angular_velocity_cmd_rad_s = 0.0
        return base

    # ── DETUMBLING ────────────────────────────────────────────────────────────
    if l1input.control_mode == ControlMode.DETUMBLING:
        base.control_valid = True
        base.nominal       = False
        base.reason        = "DETUMBLING"
        base.pid_enabled   = True
        base.kp_override   = config.KP_DETUMBLE
        base.yaw_rate_cmd  = 0.0
        base.angular_velocity_cmd_rad_s = 0.0
        lim = choose_yaw_rate_limit(ControlMode.DETUMBLING)
        base.yaw_rate_limit_dps = math.degrees(lim)
        return base

    # ── Validate nav state ────────────────────────────────────────────────────
    for v in (l1input.E, l1input.N, l1input.target_E, l1input.target_N,
              l1input.course, l1input.V):
        if not _ok(v):
            base.control_valid = False
            base.reason = "FAIL_NAN_NAV_STATE"
            base.yaw_rate_cmd = 0.0
            base.angular_velocity_cmd_rad_s = 0.0
            return base

    dE = l1input.target_E - l1input.E
    dN = l1input.target_N - l1input.N
    dist = math.hypot(dE, dN)
    base.distance_to_target = dist
    base.alongTrack = dist
    base.crossTrack  = 0.0

    # ── Target reached ────────────────────────────────────────────────────────
    if dist <= config.TARGET_RADIUS_M:
        base.control_valid  = True
        base.nominal        = True
        base.reason         = "TARGET_REACHED"
        base.yaw_rate_cmd   = 0.0
        base.angular_velocity_cmd_rad_s = 0.0
        base.target_bearing = wrap_pi(math.atan2(dE, dN))
        base.nu             = 0.0
        return base

    # ── L1 computation ────────────────────────────────────────────────────────
    target_bearing = wrap_pi(math.atan2(dE, dN))
    nu             = wrap_pi(target_bearing - l1input.course)
    sin_nu_eff     = saturated_sin(nu)
    V              = clamp(l1input.V, config.V_MIN_MPS, config.V_MAX_MPS)

    yaw_rate_cmd = 2.0 * V / config.L_GAIN_M * sin_nu_eff
    yaw_rate_cmd *= l1input.confidence

    lim = choose_yaw_rate_limit(l1input.control_mode, l1input.dr_method)
    yaw_rate_cmd = clamp(yaw_rate_cmd, -lim, lim)

    base.target_bearing = target_bearing
    base.nu             = nu
    base.yaw_rate_cmd   = yaw_rate_cmd
    base.angular_velocity_cmd_rad_s = yaw_rate_cmd
    base.yaw_rate_limit_dps = math.degrees(lim)
    base.control_valid  = True
    base.nominal        = True
    base.reason         = l1input.reason
    base.pid_enabled    = True

    logger.debug(
        "L1 mode=%s dist=%.1fm bear=%.1f° nu=%.1f° cmd=%.2f°/s",
        l1input.control_mode.value, dist,
        math.degrees(target_bearing), math.degrees(nu),
        math.degrees(yaw_rate_cmd),
    )
    return base
