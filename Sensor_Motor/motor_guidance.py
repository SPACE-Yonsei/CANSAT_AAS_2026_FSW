#!/usr/bin/env python3
"""
Navigation state estimation and L1 guidance for CanSat parafoil FSW.

Architecture:
  sensor apps → NavigationStateEstimator → L1Guidance → ParafoilBrakeController

Coordinate convention:
  Local N/E frame.  +N = north, +E = east.
  course chi:  0 = north, +pi/2 = east, increasing = right turn.
  crossTrack > 0  →  left of path (A→B direction).
  Nu > 0  →  latAccDem > 0  →  courseRateCmd > 0  →  right turn.
"""
import math
import os
import time
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from types import SimpleNamespace
from typing import Optional, Tuple

# ── Debug log ──────────────────────────────────────────────────────────────────
_SIM_LOG_PATH = os.getenv("CANSAT_SIM_LOG", datetime.now().strftime("%m%d_sim.txt"))
_sim_log = open(_SIM_LOG_PATH, "a", encoding="utf-8")
DEBUG_GUIDANCE: bool = os.environ.get("CANSAT_DEBUG_GUIDANCE", "").strip().lower() in (
    "1", "true", "yes", "on",
)

def _dbg(line: str) -> None:
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    full = f"[{ts}] {line}"
    try:
        print(full)
    except UnicodeEncodeError:
        import sys as _sys
        enc = getattr(_sys.stdout, "encoding", None) or "ascii"
        _sys.stdout.write(full.encode(enc, errors="replace").decode(enc, errors="replace") + "\n")
        _sys.stdout.flush()
    _sim_log.write(full + "\n")
    _sim_log.flush()


# ── Earth radius ───────────────────────────────────────────────────────────────
R_E = 6_378_137.0  # m, WGS-84 equatorial radius

# ── Freshness thresholds [s] ───────────────────────────────────────────────────
POS_FRESH_AGE             = 1.0
POS_PROPAGATE_MAX_AGE     = 2.0
POS_HARD_STALE_AGE        = 3.0

MOTION_FRESH_AGE          = 1.0
MOTION_PROPAGATE_MAX_AGE  = 2.0
MOTION_HARD_STALE_AGE     = 3.0

YAW_RATE_FRESH_AGE        = 0.20
YAW_RATE_PROPAGATE_MAX_AGE= 0.50
YAW_RATE_HARD_STALE_AGE   = 1.0

ALT_FRESH_AGE             = 0.50
ALT_PROPAGATE_MAX_AGE     = 2.0
ALT_HARD_STALE_AGE        = 3.0

CONTROL_COMMAND_HOLD_MAX_AGE = 0.3
ACCEL_PROPAGATE_MAX_AGE   = 0.5  # max bridging time via accelerometer


# ── Enums ──────────────────────────────────────────────────────────────────────
class FieldStatus(Enum):
    FRESH        = "FRESH"
    PROPAGATABLE = "PROPAGATABLE"
    HARD_STALE   = "HARD_STALE"
    MISSING      = "MISSING"


class GuidanceMode(Enum):
    ACTIVE   = "ACTIVE"
    DEGRADED = "DEGRADED"
    DISABLED = "DISABLED"


# ── Data classes ───────────────────────────────────────────────────────────────
@dataclass
class _FieldRecord:
    value: float
    ts: float


@dataclass
class EstimatedState:
    """Best-estimate navigation state produced by NavigationStateEstimator."""
    timestamp: float

    # Position in local N/E [m]
    pos_N: Optional[float] = None
    pos_E: Optional[float] = None
    pos_status: FieldStatus = FieldStatus.MISSING

    # Ground-track motion
    vel_N:       Optional[float] = None   # m/s
    vel_E:       Optional[float] = None   # m/s
    course:      Optional[float] = None   # rad, chi
    groundSpeed: Optional[float] = None   # m/s
    motion_status: FieldStatus = FieldStatus.MISSING

    # Yaw rate from gz [rad/s]
    gyrz: Optional[float] = None
    yaw_rate_status: FieldStatus = FieldStatus.MISSING

    # Barometer altitude [m]
    altitude: Optional[float] = None
    alt_status: FieldStatus = FieldStatus.MISSING

    # IMU attitude — for propagation and logging, not L1 core input
    roll:  Optional[float] = None  # deg
    pitch: Optional[float] = None  # deg
    yaw:   Optional[float] = None  # deg

    # Tilt rate derived from gx, gy [deg/s]
    tilt_rate: Optional[float] = None

    guidance_mode: GuidanceMode = GuidanceMode.DISABLED
    reason: str = ""


@dataclass
class GuidanceOutput:
    """Output of L1Guidance.update()."""
    timestamp:    float
    active:       bool  = False
    degraded:     bool  = False
    latAccDem:    float = 0.0   # m/s²
    courseRateCmd:float = 0.0   # rad/s, + = right turn
    Nu1:          float = 0.0   # rad
    Nu2:          float = 0.0   # rad
    Nu:           float = 0.0   # rad
    crossTrack:   float = 0.0   # m, + = left of path
    alongTrack:   float = 0.0   # m
    L1_dist:      float = 0.0   # m
    groundSpeed:  float = 0.0   # m/s
    reason:       str   = ""


@dataclass
class L1Config:
    """Tunable L1 guidance parameters — all adjustable at runtime."""
    damping:         float = 0.75
    period:          float = 8.0   # s
    L1_MIN:          float = 5.0   # m
    V_MIN:           float = 2.0   # m/s
    LAT_ACC_MAX:     float = 4.0   # m/s²
    COURSE_RATE_MAX: float = 0.6   # rad/s


# ── NavigationStateEstimator ───────────────────────────────────────────────────
class NavigationStateEstimator:
    """
    Receives sensor fields from message handlers, tracks their age, and
    produces the best EstimatedState for guidance.

    This is field-availability and data-age management.
    It is NOT a sensor reliability checker — raw sensor validity is the
    responsibility of sensorapps.  posHealth / motionHealth are the only
    availability flags honoured here.
    """

    def __init__(self) -> None:
        # GNSS
        self._lat:         Optional[_FieldRecord] = None
        self._lon:         Optional[_FieldRecord] = None
        self._course:      Optional[_FieldRecord] = None   # rad
        self._groundSpeed: Optional[_FieldRecord] = None   # m/s

        # IMU
        self._roll:  Optional[_FieldRecord] = None  # deg
        self._pitch: Optional[_FieldRecord] = None  # deg
        self._yaw:   Optional[_FieldRecord] = None  # deg
        self._ax:    Optional[_FieldRecord] = None  # m/s²
        self._ay:    Optional[_FieldRecord] = None  # m/s²
        self._az:    Optional[_FieldRecord] = None  # m/s²
        self._gx:    Optional[_FieldRecord] = None  # deg/s
        self._gy:    Optional[_FieldRecord] = None  # deg/s
        self._gz:    Optional[_FieldRecord] = None  # deg/s

        # Baro
        self._altitude: Optional[_FieldRecord] = None  # m

        # Local N/E coordinate origin (locked on first fresh GNSS position)
        self._origin_lat: Optional[float] = None
        self._origin_lon: Optional[float] = None

        # Previous position for course-from-history estimation
        self._prev_pos_N:  Optional[float] = None
        self._prev_pos_E:  Optional[float] = None
        self._prev_pos_ts: Optional[float] = None

    # ── Public update methods ──────────────────────────────────────────────────

    def update_gnss(
        self,
        lat:         Optional[float],
        lon:         Optional[float],
        course_rad:  Optional[float],
        groundSpeed: Optional[float],
        posHealth:   bool,
        motionHealth: bool,
        ts: float,
    ) -> None:
        if posHealth and lat is not None and lon is not None:
            self._lat = _FieldRecord(lat, ts)
            self._lon = _FieldRecord(lon, ts)
            if self._origin_lat is None:
                self._origin_lat = lat
                self._origin_lon = lon
        if motionHealth:
            if course_rad is not None:
                self._course = _FieldRecord(course_rad, ts)
            if groundSpeed is not None:
                self._groundSpeed = _FieldRecord(groundSpeed, ts)

    def update_imu(
        self,
        roll:  Optional[float] = None,
        pitch: Optional[float] = None,
        yaw:   Optional[float] = None,
        ax:    Optional[float] = None,
        ay:    Optional[float] = None,
        az:    Optional[float] = None,
        gx:    Optional[float] = None,
        gy:    Optional[float] = None,
        gz:    Optional[float] = None,
        ts:    float = 0.0,
    ) -> None:
        def _set(rec: Optional[_FieldRecord], v: Optional[float]) -> Optional[_FieldRecord]:
            return _FieldRecord(v, ts) if v is not None else rec

        self._roll  = _set(self._roll,  roll)
        self._pitch = _set(self._pitch, pitch)
        self._yaw   = _set(self._yaw,   yaw)
        self._ax    = _set(self._ax,    ax)
        self._ay    = _set(self._ay,    ay)
        self._az    = _set(self._az,    az)
        self._gx    = _set(self._gx,    gx)
        self._gy    = _set(self._gy,    gy)
        self._gz    = _set(self._gz,    gz)

    def update_baro(self, altitude: float, ts: float) -> None:
        self._altitude = _FieldRecord(altitude, ts)

    def set_origin(self, lat: float, lon: float) -> None:
        """Manually override the local N/E coordinate origin."""
        self._origin_lat = lat
        self._origin_lon = lon
        self._prev_pos_N = None
        self._prev_pos_E = None
        self._prev_pos_ts = None

    def reset_origin(self) -> None:
        self._origin_lat = None
        self._origin_lon = None
        self._prev_pos_N = None
        self._prev_pos_E = None
        self._prev_pos_ts = None

    @property
    def origin_lat(self) -> Optional[float]:
        return self._origin_lat

    @property
    def origin_lon(self) -> Optional[float]:
        return self._origin_lon

    # ── Primary query ─────────────────────────────────────────────────────────

    def estimate(self, now: float) -> EstimatedState:
        """
        Build the best EstimatedState at time `now`.
        Input combination cases A–L are dispatched by the fill methods.
        """
        state = EstimatedState(timestamp=now)
        self._fill_position(state, now)
        self._fill_motion(state, now)
        self._fill_yaw_rate(state, now)
        self._fill_altitude(state, now)
        self._fill_attitude(state)
        self._fill_tilt_rate(state)
        self._classify_guidance_mode(state)
        return state

    # ── Private fill methods (Section 7 skeleton) ─────────────────────────────

    def _fill_position(self, state: EstimatedState, now: float) -> None:
        """Cases A/B/C/D/E/H/I/K: resolve pos_N, pos_E."""
        lat_rec = self._lat
        lon_rec = self._lon

        if lat_rec is None or lon_rec is None or self._origin_lat is None:
            state.pos_status = FieldStatus.MISSING
            return

        pos_age = now - max(lat_rec.ts, lon_rec.ts)

        if pos_age <= POS_FRESH_AGE:
            # Cases A/B/C/H/I — fresh GNSS position
            pos_N, pos_E = _ll_to_ne(
                lat_rec.value, lon_rec.value,
                self._origin_lat, self._origin_lon,
            )
            state.pos_N, state.pos_E = pos_N, pos_E
            state.pos_status = FieldStatus.FRESH
            self._prev_pos_N  = pos_N
            self._prev_pos_E  = pos_E
            self._prev_pos_ts = now

        elif pos_age <= POS_PROPAGATE_MAX_AGE:
            # Case E skeleton: velocity-based propagation placeholder
            # Case K skeleton: accel-based bridging (≤ ACCEL_PROPAGATE_MAX_AGE) placeholder
            state.pos_N, state.pos_E = _ll_to_ne(
                lat_rec.value, lon_rec.value,
                self._origin_lat, self._origin_lon,
            )
            state.pos_status = FieldStatus.PROPAGATABLE

        else:
            state.pos_status = FieldStatus.HARD_STALE

    def _fill_motion(self, state: EstimatedState, now: float) -> None:
        """Cases A/B/C/D/E/H/I/J: resolve course, groundSpeed, vel_N, vel_E."""
        c_rec  = self._course
        gs_rec = self._groundSpeed

        def _age(r: Optional[_FieldRecord]) -> float:
            return (now - r.ts) if r is not None else float("inf")

        c_age  = _age(c_rec)
        gs_age = _age(gs_rec)

        if c_rec is None and gs_rec is None:
            # Case D/F/G/J skeleton: no motion data at all
            state.motion_status = FieldStatus.MISSING
            return

        if c_age <= MOTION_FRESH_AGE and gs_age <= MOTION_FRESH_AGE:
            # Cases A/B/C — full fresh motion
            state.course      = c_rec.value
            state.groundSpeed = gs_rec.value
            state.vel_N = gs_rec.value * math.cos(c_rec.value)
            state.vel_E = gs_rec.value * math.sin(c_rec.value)
            state.motion_status = FieldStatus.FRESH

        elif c_age <= MOTION_PROPAGATE_MAX_AGE and gs_age <= MOTION_PROPAGATE_MAX_AGE:
            # Cases D/E skeleton: propagatable stale motion
            state.course      = c_rec.value
            state.groundSpeed = gs_rec.value
            state.vel_N = gs_rec.value * math.cos(c_rec.value)
            state.vel_E = gs_rec.value * math.sin(c_rec.value)
            state.motion_status = FieldStatus.PROPAGATABLE

        elif c_rec is None and gs_rec is not None and gs_age <= MOTION_PROPAGATE_MAX_AGE:
            # Case I skeleton: groundSpeed present, course missing
            # Try course from position history if available
            course_est = self._course_from_history()
            if course_est is not None:
                state.course      = course_est
                state.groundSpeed = gs_rec.value
                state.vel_N = gs_rec.value * math.cos(course_est)
                state.vel_E = gs_rec.value * math.sin(course_est)
                state.motion_status = FieldStatus.PROPAGATABLE
            else:
                state.motion_status = FieldStatus.MISSING

        elif c_rec is not None and c_age <= MOTION_PROPAGATE_MAX_AGE and gs_rec is None:
            # Case H skeleton: course present, groundSpeed missing
            state.motion_status = FieldStatus.MISSING

        else:
            state.motion_status = FieldStatus.HARD_STALE

    def _fill_yaw_rate(self, state: EstimatedState, now: float) -> None:
        """Cases B/C/F/J: gz → gyrz [rad/s]."""
        gz_rec = self._gz
        if gz_rec is None:
            state.yaw_rate_status = FieldStatus.MISSING
            return
        age = now - gz_rec.ts
        if age <= YAW_RATE_FRESH_AGE:
            state.gyrz            = math.radians(gz_rec.value)
            state.yaw_rate_status = FieldStatus.FRESH
        elif age <= YAW_RATE_PROPAGATE_MAX_AGE:
            state.gyrz            = math.radians(gz_rec.value)
            state.yaw_rate_status = FieldStatus.PROPAGATABLE
        else:
            state.yaw_rate_status = FieldStatus.HARD_STALE

    def _fill_altitude(self, state: EstimatedState, now: float) -> None:
        """Cases C/G: barometer altitude."""
        alt_rec = self._altitude
        if alt_rec is None:
            state.alt_status = FieldStatus.MISSING
            return
        age = now - alt_rec.ts
        if age <= ALT_FRESH_AGE:
            state.altitude   = alt_rec.value
            state.alt_status = FieldStatus.FRESH
        elif age <= ALT_PROPAGATE_MAX_AGE:
            state.altitude   = alt_rec.value
            state.alt_status = FieldStatus.PROPAGATABLE
        else:
            state.alt_status = FieldStatus.HARD_STALE

    def _fill_attitude(self, state: EstimatedState) -> None:
        if self._roll  is not None: state.roll  = self._roll.value
        if self._pitch is not None: state.pitch = self._pitch.value
        if self._yaw   is not None: state.yaw   = self._yaw.value

    def _fill_tilt_rate(self, state: EstimatedState) -> None:
        """Case L: tilt rate from gx, gy for payload oscillation detection."""
        if self._gx is not None and self._gy is not None:
            state.tilt_rate = math.hypot(self._gx.value, self._gy.value)

    def _classify_guidance_mode(self, state: EstimatedState) -> None:
        pos_ok    = state.pos_status    in (FieldStatus.FRESH, FieldStatus.PROPAGATABLE)
        motion_ok = state.motion_status in (FieldStatus.FRESH, FieldStatus.PROPAGATABLE)

        if not pos_ok and not motion_ok:
            state.guidance_mode = GuidanceMode.DISABLED
            state.reason        = "pos+motion unavailable"
        elif not pos_ok:
            state.guidance_mode = GuidanceMode.DISABLED
            state.reason        = "position unavailable"
        elif not motion_ok:
            state.guidance_mode = GuidanceMode.DISABLED
            state.reason        = "motion unavailable"
        elif (state.pos_status    == FieldStatus.PROPAGATABLE
              or state.motion_status == FieldStatus.PROPAGATABLE):
            state.guidance_mode = GuidanceMode.DEGRADED
            state.reason        = "propagated data in use"
        else:
            state.guidance_mode = GuidanceMode.ACTIVE
            state.reason        = ""

    def _course_from_history(self) -> Optional[float]:
        """Estimate course [rad] from last two cached positions."""
        if (self._prev_pos_N is None or self._lat is None
                or self._origin_lat is None):
            return None
        cur_N, cur_E = _ll_to_ne(
            self._lat.value, self._lon.value,
            self._origin_lat, self._origin_lon,
        )
        dN = cur_N - self._prev_pos_N
        dE = cur_E - self._prev_pos_E
        if math.hypot(dN, dE) < 0.5:
            return None
        return math.atan2(dE, dN)


# ── L1Guidance ─────────────────────────────────────────────────────────────────
class L1Guidance:
    """
    Port of ArduPilot AP_L1_Control::update_waypoint() for parafoil.

    Outputs courseRateCmd [rad/s] instead of bank angle.
    Positive courseRateCmd = right turn (chi increases).

    Nu = Nu1 + Nu2
      Nu1 = cross-track capture angle  (asin(crossTrack / L1_dist))
      Nu2 = velocity-track angle       (atan2(xtrackVel, ltrackVel))
    """

    def __init__(self, config: Optional[L1Config] = None) -> None:
        self.cfg = config or L1Config()
        self._target_N: Optional[float] = None
        self._target_E: Optional[float] = None
        self._start_N:  Optional[float] = None
        self._start_E:  Optional[float] = None
        self._start_locked: bool  = False
        self._last_Nu:  float     = 0.0   # for _prevent_indecision

    def set_target(self, target_N: float, target_E: float) -> None:
        self._target_N = target_N
        self._target_E = target_E

    def set_start(self, start_N: float, start_E: float) -> None:
        self._start_N      = start_N
        self._start_E      = start_E
        self._start_locked = True

    def reset(self) -> None:
        self._start_locked = False
        self._start_N      = None
        self._start_E      = None
        self._last_Nu      = 0.0

    def update(self, state: EstimatedState, now: float) -> GuidanceOutput:
        out = GuidanceOutput(timestamp=now)

        # ── Preconditions ────────────────────────────────────────────────────
        if self._target_N is None or self._target_E is None:
            out.reason = "no target"
            return out

        if state.guidance_mode == GuidanceMode.DISABLED:
            out.reason = state.reason
            return out

        pos_N = state.pos_N
        pos_E = state.pos_E
        if pos_N is None or pos_E is None:
            out.reason = "position missing"
            return out

        # ── Lock start point on first active position ────────────────────────
        if not self._start_locked:
            self._start_N      = pos_N
            self._start_E      = pos_E
            self._start_locked = True

        # ── Path geometry (A → B) ─────────────────────────────────────────────
        A_N, A_E = self._start_N,  self._start_E
        B_N, B_E = self._target_N, self._target_E

        AB_N   = B_N - A_N
        AB_E   = B_E - A_E
        AB_len = math.hypot(AB_N, AB_E)

        if AB_len < 1.0:
            out.reason = "start ≈ target"
            return out

        e_N = AB_N / AB_len
        e_E = AB_E / AB_len

        # ── AP = current position relative to A ──────────────────────────────
        AP_N = pos_N - A_N
        AP_E = pos_E - A_E

        alongTrack = AP_N * e_N + AP_E * e_E   # projection onto AB
        crossTrack = AP_N * e_E - AP_E * e_N   # + = left of path

        # ── Velocity ─────────────────────────────────────────────────────────
        gs     = state.groundSpeed or 0.0
        course = state.course      or 0.0
        vel_N  = gs * math.cos(course)
        vel_E  = gs * math.sin(course)

        ltrackVel = vel_N * e_N + vel_E * e_E   # along-path velocity
        xtrackVel = vel_N * e_E - vel_E * e_N   # cross-path velocity, + = left

        # ── L1 distance ───────────────────────────────────────────────────────
        gs_for_l1 = max(gs, self.cfg.V_MIN)
        L1_dist   = max(
            self.cfg.damping * self.cfg.period / math.pi * gs_for_l1,
            self.cfg.L1_MIN,
        )

        # ── Nu1: cross-track capture ──────────────────────────────────────────
        sine_Nu1 = _clamp(crossTrack / L1_dist, -0.7071, 0.7071)
        Nu1      = math.asin(sine_Nu1)

        # ── Nu2: velocity-track alignment ─────────────────────────────────────
        vel_mag = math.hypot(ltrackVel, xtrackVel)
        Nu2     = math.atan2(xtrackVel, ltrackVel) if vel_mag >= 0.01 else 0.0

        # ── Total L1 angle ────────────────────────────────────────────────────
        target_bearing = math.atan2(B_E - pos_E, B_N - pos_N)
        Nu = self._prevent_indecision(Nu1 + Nu2, target_bearing, course)
        Nu = _clamp(Nu, -math.pi / 2.0, math.pi / 2.0)

        # ── K_L1 = 4 * damping² ──────────────────────────────────────────────
        K_L1 = 4.0 * self.cfg.damping ** 2

        # ── Lateral acceleration demand ───────────────────────────────────────
        latAccDem = _clamp(
            K_L1 * (gs ** 2) / L1_dist * math.sin(Nu),
            -self.cfg.LAT_ACC_MAX, self.cfg.LAT_ACC_MAX,
        )

        # ── Course rate command ───────────────────────────────────────────────
        courseRateCmd = _clamp(
            latAccDem / max(gs, self.cfg.V_MIN),
            -self.cfg.COURSE_RATE_MAX, self.cfg.COURSE_RATE_MAX,
        )

        out.active        = True
        out.degraded      = (state.guidance_mode == GuidanceMode.DEGRADED)
        out.latAccDem     = latAccDem
        out.courseRateCmd = courseRateCmd
        out.Nu1           = Nu1
        out.Nu2           = Nu2
        out.Nu            = Nu
        out.crossTrack    = crossTrack
        out.alongTrack    = alongTrack
        out.L1_dist       = L1_dist
        out.groundSpeed   = gs

        if DEBUG_GUIDANCE:
            _dbg(
                f"[L1] active={out.active} degraded={out.degraded} "
                f"Nu1={math.degrees(Nu1):.1f}° Nu2={math.degrees(Nu2):.1f}° "
                f"Nu={math.degrees(Nu):.1f}° xtrack={crossTrack:.1f}m "
                f"atrack={alongTrack:.1f}m L1={L1_dist:.1f}m "
                f"gs={gs:.1f}m/s latAcc={latAccDem:.3f}m/s² "
                f"crCmd={math.degrees(courseRateCmd):.2f}°/s"
            )

        return out

    def _prevent_indecision(
        self,
        Nu: float,
        target_bearing: Optional[float] = None,
        course: Optional[float] = None,
    ) -> float:
        """
        Hold turn direction when target is nearly behind vehicle (|Nu| > 150°).
        Prevents L/R oscillation when pointing away from target.
        Mirrors ArduPilot AP_L1_Control::_prevent_indecision().
        """
        THRESHOLD = 0.9 * math.pi
        pointing_away = True
        if target_bearing is not None and course is not None:
            pointing_away = abs(_wrap_pi(target_bearing - course)) > math.radians(120.0)
        if (abs(Nu) > THRESHOLD
                and abs(self._last_Nu) > THRESHOLD
                and pointing_away
                and Nu * self._last_Nu < 0.0):
            Nu = self._last_Nu
        self._last_Nu = Nu
        return Nu


# ── Utilities ──────────────────────────────────────────────────────────────────

def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else (hi if x > hi else x)


def _wrap_pi(a: float) -> float:
    """Wrap angle to (-pi, +pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _ll_to_ne(lat: float, lon: float,
              lat0: float, lon0: float) -> Tuple[float, float]:
    """
    Lat/lon [deg] → local North/East [m] relative to origin (lat0, lon0).
    Flat-earth approximation, valid for small areas (< ~50 km).
    """
    N = (lat  - lat0)  * math.pi / 180.0 * R_E
    E = (lon  - lon0)  * math.pi / 180.0 * R_E * math.cos(math.radians(lat0))
    return N, E


def ne_to_ll(N: float, E: float,
             lat0: float, lon0: float) -> Tuple[float, float]:
    """Inverse of _ll_to_ne — local N/E [m] → lat/lon [deg]."""
    lat = lat0 + N / (math.pi / 180.0 * R_E)
    lon_scale = math.pi / 180.0 * R_E * math.cos(math.radians(lat0))
    lon = lon0 + (E / lon_scale if abs(lon_scale) > 1e-6 else 0.0)
    return lat, lon


# Legacy facade ---------------------------------------------------------------
#
# The new runtime uses NavigationStateEstimator and L1Guidance directly.  These
# wrappers keep replay tools and older unit tests callable while they migrate to
# the class-based API.

GPS_JUMP_MAX_SPEED = 200.0          # m/s
GPS_STABLE_COUNT_REQUIRED = 2
LANDING_YR_MAX = 0.25               # rad/s


@dataclass(init=False)
class GpsVector:
    lat: float
    lon: float
    velocity: float                 # m/s
    direction: float                # deg, legacy course

    def __init__(self, lat: float, lon: float, a: float = 0.0, b: float = 0.0, **kwargs) -> None:
        self.lat = lat
        self.lon = lon
        if "speed" in kwargs or "course" in kwargs:
            self.velocity = float(kwargs.get("speed", a))
            self.direction = float(kwargs.get("course", b))
        elif abs(float(a)) <= 80.0 and abs(float(b)) > 80.0:
            self.velocity = float(a)
            self.direction = float(b)
        else:
            self.direction = float(a)
            self.velocity = float(b)


@dataclass
class GpsFidelity:
    fix_quality: int = 1
    sats: int = 4
    rmc_status: str = "A"
    gps_health: int = 1
    pos_health: int = 1
    motion_health: int = 1


@dataclass
class GuidanceResult:
    state: str
    distance: float = 0.0
    commanded_yaw_rate: float = 0.0
    desired_yaw_rate: float = 0.0
    crosstrack_error: float = 0.0
    along_track: float = 0.0
    heading_error: float = 0.0


@dataclass
class _GpsJumpState:
    initialized: bool = False
    lat: float = 0.0
    lon: float = 0.0
    time: float = 0.0


start_point = SimpleNamespace(lat=None, lon=None)
target_coord = SimpleNamespace(lat=None, lon=None)
_prev_gps = _GpsJumpState()
_gps_stable_count = 0
_legacy_estimator = NavigationStateEstimator()
_legacy_guidance = L1Guidance(L1Config())
cascade_pi = SimpleNamespace(MAX_CMD=L1Config().COURSE_RATE_MAX)


def init_guidance() -> None:
    global _prev_gps, _gps_stable_count, _legacy_estimator, _legacy_guidance
    start_point.lat = None
    start_point.lon = None
    target_coord.lat = None
    target_coord.lon = None
    _prev_gps = _GpsJumpState()
    _gps_stable_count = 0
    _legacy_estimator = NavigationStateEstimator()
    _legacy_guidance = L1Guidance(L1Config())


def set_start_coordinates(lat: float, lon: float) -> None:
    start_point.lat = lat
    start_point.lon = lon
    _legacy_estimator.set_origin(lat, lon)
    _legacy_guidance.set_start(0.0, 0.0)


def set_target_coord(lat: float, lon: float) -> None:
    target_coord.lat = lat
    target_coord.lon = lon
    if start_point.lat is not None and start_point.lon is not None:
        tgt_N, tgt_E = _ll_to_ne(lat, lon, start_point.lat, start_point.lon)
        _legacy_guidance.set_target(tgt_N, tgt_E)


def is_gps_valid(gps, gps_fidelity) -> bool:
    try:
        lat = float(gps.lat)
        lon = float(gps.lon)
    except (TypeError, ValueError, AttributeError):
        return False
    if not (math.isfinite(lat) and math.isfinite(lon)):
        return False
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return False

    pos_health = getattr(gps_fidelity, "pos_health", None)
    motion_health = getattr(gps_fidelity, "motion_health", None)
    if pos_health is not None or motion_health is not None:
        return bool(pos_health) and bool(motion_health)

    fix_quality = int(getattr(gps_fidelity, "fix_quality", 0))
    sats = int(getattr(gps_fidelity, "sats", 0))
    rmc_status = str(getattr(gps_fidelity, "rmc_status", "V")).upper()
    gps_health = int(getattr(gps_fidelity, "gps_health", 1))
    return fix_quality >= 1 and sats >= 4 and rmc_status == "A" and gps_health >= 1


def is_gps_jump(lat: float, lon: float) -> bool:
    global _gps_stable_count
    now = time.time()
    if not _prev_gps.initialized:
        _prev_gps.initialized = True
        _prev_gps.lat = lat
        _prev_gps.lon = lon
        _prev_gps.time = now
        return False

    dt = max(now - _prev_gps.time, 1e-3)
    dN, dE = _ll_to_ne(lat, lon, _prev_gps.lat, _prev_gps.lon)
    jump = math.hypot(dN, dE) / dt > GPS_JUMP_MAX_SPEED
    if jump:
        _gps_stable_count = 0
        return True

    _prev_gps.lat = lat
    _prev_gps.lon = lon
    _prev_gps.time = now
    _gps_stable_count += 1
    return False


def guidance(imu_data, gps_vec, gps_fidelity, target, baro_m=None) -> GuidanceResult:
    if target is None:
        return GuidanceResult("TARGET_UNSET")
    if not is_gps_valid(gps_vec, gps_fidelity):
        return GuidanceResult("GPS_INVALID")
    if start_point.lat is None or start_point.lon is None:
        return GuidanceResult("START_UNSET")
    if is_gps_jump(float(gps_vec.lat), float(gps_vec.lon)):
        return GuidanceResult("GPS_INVALID")
    if _gps_stable_count < GPS_STABLE_COUNT_REQUIRED:
        return GuidanceResult("GPS_INVALID")
    if baro_m is not None and float(baro_m) <= 0.0:
        return GuidanceResult("BARO_INVALID")

    now = time.time()
    set_target_coord(float(target.lat), float(target.lon))
    direction = getattr(gps_vec, "direction", getattr(gps_vec, "course", 0.0))
    velocity = getattr(gps_vec, "velocity", getattr(gps_vec, "speed", 0.0))
    _legacy_estimator.update_gnss(
        float(gps_vec.lat),
        float(gps_vec.lon),
        math.radians(float(direction)),
        float(velocity),
        True,
        True,
        now,
    )
    _legacy_estimator.update_imu(
        yaw=float(getattr(imu_data, "yaw", 0.0)),
        gz=float(getattr(imu_data, "gyrz", 0.0)),
        ts=now,
    )
    if baro_m is not None:
        _legacy_estimator.update_baro(float(baro_m), now)

    est = _legacy_estimator.estimate(now)
    out = _legacy_guidance.update(est, now)
    if not out.active:
        return GuidanceResult("FDIR", commanded_yaw_rate=0.0)

    tgt_N, tgt_E = _ll_to_ne(float(target.lat), float(target.lon), start_point.lat, start_point.lon)
    pos_N = est.pos_N or 0.0
    pos_E = est.pos_E or 0.0
    distance = math.hypot(tgt_N - pos_N, tgt_E - pos_E)
    cmd = out.courseRateCmd
    if baro_m is not None and float(baro_m) <= 10.0:
        cmd = _clamp(cmd, -LANDING_YR_MAX, LANDING_YR_MAX)

    if distance <= 3.0:
        state = "TARGET_REACHED"
    elif abs(cmd) < 1e-4:
        state = "STRAIGHT"
    else:
        state = "TURNING"

    return GuidanceResult(
        state=state,
        distance=distance,
        commanded_yaw_rate=cmd,
        desired_yaw_rate=out.courseRateCmd,
        crosstrack_error=out.crossTrack,
        along_track=out.alongTrack,
        heading_error=math.degrees(out.Nu),
    )
