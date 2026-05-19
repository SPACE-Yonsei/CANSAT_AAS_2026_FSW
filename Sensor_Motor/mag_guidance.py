"""Magnetic-bearing guidance module (GPS-free).

Uses IMU yaw + gyro + barometer only.  A single magnetic bearing
(start → target, deg, 0=N, CW+) is injected at runtime via IPC.

Control law
-----------
  heading_error = wrap_pi(target_bearing - current_yaw)
  raw_cmd       = Kp * heading_error - Kd * gyrz        (rad/s)
  cmd_deg_s     = clamp(deg(raw_cmd), -MAX, +MAX)

Sign convention (matches control.py / motorapp.py)
  angular_velocity_cmd_deg_s > 0  → RIGHT turn (CW)
  angular_velocity_cmd_deg_s < 0  → LEFT  turn (CCW)
  gyrz_rad_s > 0                  → CW (already negated at source in motorapp)
  yaw_rad : navigation frame, 0=North, CW increasing
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional


# ── constants ──────────────────────────────────────────────────────────────────
_TWO_PI = 2.0 * math.pi


def _wrap_pi(angle_rad: float) -> float:
    """Wrap angle to (-π, π]."""
    return (angle_rad + math.pi) % _TWO_PI - math.pi


# ── config ─────────────────────────────────────────────────────────────────────
@dataclass
class MagGuidanceConfig:
    """Tuning parameters for bearing-hold guidance.

    Kp_rad_s_per_rad : proportional gain  (rad/s per rad of heading error)
    Kd_damping       : gyro-rate damping ratio (0 = off, 1 = critically damped)
    max_cmd_deg_s    : saturation limit for output command (deg/s)
    imu_timeout_s    : max age of IMU sample before declaring FAIL (s)
    baro_timeout_s   : max age of baro sample before marking altitude stale (s)
    """
    Kp_rad_s_per_rad: float = 1.2
    Kd_damping:       float = 0.6
    max_cmd_deg_s:    float = 45.0
    imu_timeout_s:    float = 1.0
    baro_timeout_s:   float = 2.0


# ── input / output dataclasses ─────────────────────────────────────────────────
@dataclass
class MagGuidanceInput:
    """Sensor snapshot fed into ProduceMagGuidance each control tick.

    yaw_rad          : IMU fusion yaw, navigation frame (0=N, CW+)
    gyrz_rad_s       : IMU yaw rate, navigation frame (CW+ after source negation)
    alt_m            : barometer altitude (m, AGL or MSL depending on calibration)
    target_bearing_rad: pre-computed start→target bearing (rad, NaN = not set)
    timestamp        : current monotonic time (s)
    imu_ts           : timestamp of latest IMU sample (None = never received)
    baro_ts          : timestamp of latest baro sample (None = never received)
    imu_health       : 1 = hardware OK
    baro_health      : 1 = hardware OK
    """
    yaw_rad:            Optional[float] = None
    gyrz_rad_s:         float           = 0.0
    alt_m:              Optional[float] = None
    target_bearing_rad: float           = float("nan")
    timestamp:          float           = 0.0
    imu_ts:             Optional[float] = None
    baro_ts:            Optional[float] = None
    imu_health:         int             = 0
    baro_health:        int             = 0


@dataclass
class MagGuidanceOutput:
    """Guidance result for one control tick.

    angular_velocity_cmd_deg_s : turn rate command passed to control.py
    heading_error_deg           : signed error (target − current), for logging
    target_bearing_deg          : echoes back the input bearing, for logging
    current_yaw_deg             : echoes back IMU yaw, for logging
    alt_m                       : altitude at this tick
    mode                        : one of NOMINAL / DEGRADED / NO_BEARING / IMU_FAIL
    valid                       : True ↔ cmd is safe to actuate
    timestamp                   : echoes back input timestamp
    """
    angular_velocity_cmd_deg_s: float  = 0.0
    heading_error_deg:          float  = 0.0
    target_bearing_deg:         float  = float("nan")
    current_yaw_deg:            float  = float("nan")
    alt_m:                      Optional[float] = None
    mode:                       str    = "NO_BEARING"
    valid:                      bool   = False
    timestamp:                  float  = 0.0


# ── mode label constants ────────────────────────────────────────────────────────
MODE_NOMINAL    = "NOMINAL"       # all sensors OK, bearing set
MODE_DEGRADED   = "DEGRADED"      # baro stale but can still steer
MODE_NO_BEARING = "NO_BEARING"    # bearing not yet received
MODE_IMU_FAIL   = "IMU_FAIL"      # IMU data missing or unhealthy


# ── main function ──────────────────────────────────────────────────────────────
def ProduceMagGuidance(
    inp: MagGuidanceInput,
    cfg: MagGuidanceConfig = MagGuidanceConfig(),
) -> MagGuidanceOutput:
    """Compute turn-rate command from bearing error.

    Returns a MagGuidanceOutput with valid=False and cmd=0 whenever:
      - target_bearing_rad is NaN (bearing not yet set)
      - IMU yaw is None or IMU hardware unhealthy
      - IMU sample is stale (age > cfg.imu_timeout_s)
    """
    now = inp.timestamp

    # ── 1. target bearing validity ─────────────────────────────────────────────
    if not math.isfinite(inp.target_bearing_rad):
        return MagGuidanceOutput(
            mode=MODE_NO_BEARING,
            valid=False,
            timestamp=now,
            alt_m=inp.alt_m,
        )

    # ── 2. IMU validity ────────────────────────────────────────────────────────
    imu_age = (now - inp.imu_ts) if (inp.imu_ts is not None) else float("inf")
    imu_ok = (
        inp.yaw_rad is not None
        and math.isfinite(inp.yaw_rad)
        and inp.imu_health == 1
        and imu_age <= cfg.imu_timeout_s
    )

    if not imu_ok:
        return MagGuidanceOutput(
            mode=MODE_IMU_FAIL,
            valid=False,
            timestamp=now,
            alt_m=inp.alt_m,
            target_bearing_deg=math.degrees(inp.target_bearing_rad),
        )

    # ── 3. baro status (informational — does not block steering) ──────────────
    baro_age = (now - inp.baro_ts) if (inp.baro_ts is not None) else float("inf")
    baro_fresh = inp.baro_health == 1 and baro_age <= cfg.baro_timeout_s
    mode = MODE_NOMINAL if baro_fresh else MODE_DEGRADED

    # ── 4. PD heading controller ──────────────────────────────────────────────
    #  heading_error > 0  → target is clockwise from current heading → right turn
    heading_error_rad = _wrap_pi(inp.target_bearing_rad - inp.yaw_rad)

    # Proportional term: Kp * error
    # Derivative  term: -Kd * gyrz  (opposes rotation; gyrz CW+ so damp CW when turning CW)
    raw_cmd_rad_s = (
        cfg.Kp_rad_s_per_rad * heading_error_rad
        - cfg.Kd_damping * inp.gyrz_rad_s
    )

    # Convert to deg/s and clamp
    raw_cmd_deg_s = math.degrees(raw_cmd_rad_s)
    cmd_deg_s = max(-cfg.max_cmd_deg_s, min(cfg.max_cmd_deg_s, raw_cmd_deg_s))

    return MagGuidanceOutput(
        angular_velocity_cmd_deg_s=cmd_deg_s,
        heading_error_deg=math.degrees(heading_error_rad),
        target_bearing_deg=math.degrees(inp.target_bearing_rad),
        current_yaw_deg=math.degrees(inp.yaw_rad),
        alt_m=inp.alt_m,
        mode=mode,
        valid=True,
        timestamp=now,
    )
