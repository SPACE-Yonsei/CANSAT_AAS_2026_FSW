#!/usr/bin/env python3
"""
Parafoil Guidance Module
========================

Implements L1 carrot-chase guidance with cascaded heading / yaw-rate control
for autonomous parafoil navigation.

Architecture (responsibility separation):
  +-------------------------------------------------------------+
  | 1. Guidance layer   - "where should we go?"                  |
  |    -> computes desired_course and desired_yaw_rate            |
  |    using L1 / pure-pursuit carrot-chase on the path           |
  |    from start to target.                                      |
  |                                                               |
  | 2. Yaw-rate controller - "how fast should we turn?"           |
  |    -> PI law tracking desired_yaw_rate with anti-windup       |
  |    -> outputs commanded_yaw_rate (deg/s)                      |
  |                                                               |
  | 3. Actuator mixer (motor_control.py)                          |
  |    -> converts commanded_yaw_rate to left/right arm angles    |
  |    using the empirical K_delta differential model             |
  |                                                               |
  | 4. Servo mapping (motor_control.py)                           |
  |    -> converts arm angles (deg) to PWM pulse widths (us)      |
  +-------------------------------------------------------------+

Coordinate convention:
  - GPS raw data: (lat, lon) in degrees.
  - Local frame: ALL coordinates are (E, N) - East, North:
      E = east displacement  [m]  (positive = east of origin)
      N = north displacement [m]  (positive = north of origin)
  - Heading and course: measured clockwise from North:
      North = 0 deg, East = 90 deg, South = +/-180 deg, West = -90 deg
  - heading = atan2(delta_E, delta_N)
    Why? atan2(y, x) gives the angle from the x-axis.  We want the angle
    FROM the North (N) axis with East as positive rotation.  Placing E in
    the y-slot and N in the x-slot yields the North-referenced bearing.
"""
import math
import time
import types


# =============================================================================
# Tunable Parameters - PI Controller
# =============================================================================
cascade_pi = types.SimpleNamespace(
    # The outer loop uses the L1 guidance law (not a simple P gain).
    # Kp_outer is retained for reference but the actual outer-loop gain is
    # embedded in: desired_yaw_rate = 2 * V / L * sin(heading_error).
    Kp_outer=0.6,

    # Inner-loop PI gains for yaw-rate tracking:
    #   output = Kp_inner * rate_error + Ki_inner * integral
    # Units: rate_error [deg/s], integral [deg], output [deg/s]
    Kp_inner=1.0,
    Ki_inner=0.1,

    # Integral state and hard clamp (safety net beyond anti-windup)
    pi_integral=0.0,
    MAX_INTEGRAL=15.0,  # [deg] maximum magnitude of accumulated integral

    # Heading deadband: if |heading_error| < DEADBAND, fly straight
    # and clear the integral to prevent slow drift corrections.
    DEADBAND=5.0,  # [deg]

    # Saturation limit on the PI controller output (commanded yaw rate)
    MAX_CMD=60.0   # [deg/s]
)


# =============================================================================
# Tunable Parameters - Guidance / Navigation
# =============================================================================
target = types.SimpleNamespace(lat=0.0, lon=0.0)
start_point = types.SimpleNamespace(lat=0.0, lon=0.0)

# Flat-Earth conversion factor (accurate < 0.1% within ~10 km)
LAT_TO_METER = 111320.0  # [m/deg] meters per degree of latitude

# -----------------------------------------------------------------------------
# Altitude-adaptive look-ahead distance
#
# Rationale:
#   At HIGH altitude the parafoil is far from the target; aggressive turning
#   wastes energy and causes oscillation.  A larger L_DISTANCE smooths the
#   path.  At MID altitude the baseline provides balanced tracking.  At LOW
#   altitude (final approach) a shorter L_DISTANCE gives tighter alignment
#   for landing precision.
#
# This is a simple altitude-based tuning of the single L_DISTANCE value,
# NOT a complex L1 parameter schedule.  It is designed for easy integration
# with the existing system.
# -----------------------------------------------------------------------------
L_DISTANCE      = 15.0   # [m] current effective look-ahead (updated by altitude)
L_DISTANCE_BASE = 15.0   # [m] baseline (mid altitude)
L_DISTANCE_HIGH = 25.0   # [m] high altitude - less aggressive
L_DISTANCE_LOW  = 10.0   # [m] low altitude / final approach - more precise

ALT_HIGH = 100.0          # [m] threshold: above this -> high-altitude mode
ALT_LOW  = 30.0           # [m] threshold: below this -> final-approach mode

# Wind / crab estimation (first-order low-pass filtered)
wind_effect = 0.0  # [deg] estimated crab angle

# Barometer altitude (updated from motorapp via update_altitude())
altitude_m = 0.0  # [m] AGL from barometer

# Timing
last_time = None


# =============================================================================
# Tunable Parameters - Figure-Eight Pattern
# =============================================================================
# The figure-eight pattern is activated when patterned == True to dissipate
# excess altitude/energy.
#
# Why figure-eight rather than simple circles:
#   - Alternates left/right turns -> reduces control surface bias
#   - More balanced energy dissipation under crosswind
#   - Avoids persistent gyroscopic/aerodynamic asymmetry accumulation
#   - The parafoil cannot reduce energy with throttle like a powered aircraft;
#     it must increase horizontal path length.  The figure-eight achieves this
#     while staying near the target.
#   - Integrates easily into the existing 2D guidance pipeline: the pattern
#     simply replaces the guidance waypoint; the heading/yaw-rate cascade
#     runs unchanged.
_pattern = types.SimpleNamespace(
    lobe_sign=1,           # +1 = first lobe, -1 = second lobe
    last_switch_time=0.0,  # [s] timestamp of last lobe switch
    LOBE_PERIOD=25.0,      # [s] time per lobe before switching sides
    RADIUS=25.0,           # [m] lateral offset from target per lobe
)

# Final approach parameters
FINAL_APPROACH_ALT = 20.0   # [m] below this, cancel pattern -> straight-in
MAX_YAW_RATE_FINAL = 15.0   # [deg/s] reduced aggressiveness during final approach


# =============================================================================
# Initialization / Reset
# =============================================================================
def init_guidance():
    """Initialize / reset all guidance state.

    Called once at motorapp startup.  Resets PI integral, wind estimate,
    altitude, L_DISTANCE, pattern state, and timing.
    """
    global wind_effect, last_time, altitude_m, L_DISTANCE
    wind_effect = 0.0
    altitude_m = 0.0
    L_DISTANCE = L_DISTANCE_BASE
    cascade_pi.pi_integral = 0.0
    last_time = time.time()
    _pattern.lobe_sign = 1
    _pattern.last_switch_time = time.time()


def reset_control():
    """Reset controller state (integral, wind) without full re-init.

    Useful when transitioning between flight phases.
    """
    global wind_effect, last_time
    wind_effect = 0.0
    cascade_pi.pi_integral = 0.0
    last_time = time.time()


# =============================================================================
# Angle Utility
# =============================================================================
def _wrap_180(a: float) -> float:
    """Wrap angle to [-180, +180) degrees.

    Used for heading-error computation so the controller always commands
    the shortest rotational path (e.g. 350 deg error -> -10 deg).
    """
    return (a + 180.0) % 360.0 - 180.0


# =============================================================================
# GPS Utilities
# =============================================================================
def is_gps_valid(lat: float, lon: float,
                 fix_quality: int = 0, sats: int = 0,
                 rmc_status: str = "V") -> bool:
    """Check whether a GPS fix is trustworthy enough for closed-loop guidance.

    Four independent checks (ALL must pass):
      1. Coordinate sanity - not (0,0) and within valid geographic bounds.
         (0,0) is a common default for uninitialized GPS receivers.
      2. Fix quality >= 1 - at least a standard GPS fix (0 = no fix).
      3. Satellite count >= 4 - minimum for a 3-D position solution.
      4. RMC status == 'A' - NMEA RMC sentence reports Active/valid.

    If ANY check fails, guidance should hold the last safe command or
    go to neutral to prevent erratic behavior from bad position data.

    Args:
        lat, lon: geographic coordinates [deg]
        fix_quality: GGA fix quality (0=invalid, 1=GPS, 2=DGPS, ...)
        sats: number of satellites in use
        rmc_status: 'A' = active/valid, 'V' = void/warning

    Returns:
        True if the fix is good enough for guidance.
    """
    coord_ok = (not (lat == 0.0 and lon == 0.0)
                and abs(lat) <= 90.0
                and abs(lon) <= 180.0)
    fidelity_ok = fix_quality >= 1 and sats >= 4 and rmc_status == "A"
    return coord_ok and fidelity_ok


def calculate_distance_haversine(lat1: float, lon1: float,
                                  lat2: float, lon2: float) -> float:
    """Great-circle distance between two geographic points [m].

    Uses the Haversine formula, which is numerically stable for the short
    distances typical in CanSat operations (< 10 km).

    Args:
        lat1, lon1: first point [deg]
        lat2, lon2: second point [deg]

    Returns:
        Distance in meters.
    """
    R = 6371000.0  # Earth mean radius [m]
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)
    a = (math.sin(d_phi / 2.0) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2.0) ** 2)
    return R * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def _llh_to_en(lat: float, lon: float) -> tuple:
    """Convert geographic (lat, lon) to local (East, North) meters.

    Uses a flat-Earth tangent-plane approximation centered at start_point.
    Accurate to < 0.1% for distances under ~10 km.

    The local frame uses (E, N) ordering:
      E = east displacement  [m] - positive eastward
      N = north displacement [m] - positive northward

    This (E, N) order is used EVERYWHERE in the local frame to prevent
    coordinate-swap bugs.  GPS (lat, lon) is converted here and never
    used directly in guidance math afterward.

    Args:
        lat, lon: geographic coordinates [deg]

    Returns:
        (E, N) tuple in meters relative to start_point.
    """
    N = (lat - start_point.lat) * LAT_TO_METER
    E = (lon - start_point.lon) * LAT_TO_METER * math.cos(math.radians(start_point.lat))
    return E, N


# =============================================================================
# L1 Carrot Point Generation
# =============================================================================
def _carrot(my_E: float, my_N: float,
            tgt_E: float, tgt_N: float) -> tuple:
    """Compute a look-ahead carrot point on the straight-line path to target.

    This implements a simplified L1 / pure-pursuit guidance concept:
      1. The desired path is the straight line from the local origin (0,0)
         -- typically the launch/start point -- to the target (tgt_E, tgt_N).
      2. The current position is projected onto this line.  The projection is
         clamped to [0, path_length] so the carrot never falls behind the
         start or beyond the target.
      3. The carrot is placed L_DISTANCE ahead of the projection along the
         line.  This creates a "pull" toward the path: if the vehicle drifts
         off-track, the carrot pulls it back; if on-track, it moves forward.

    The underlying idea is similar to pure pursuit / L1 guidance, where
    the commanded turn rate scales with heading misalignment relative to
    a look-ahead reference point on the desired path.

    Geometry (in local E-N frame):
        path_vector  = (tgt_E, tgt_N) - origin is (0, 0)
        unit_vector  = path_vector / |path_vector|
        projection_s = clamp( dot(my_pos, unit_vector), 0, |path_vector| )
        carrot       = (projection_s + L_DISTANCE) * unit_vector

    Args:
        my_E, my_N:   current position in local (E, N) [m]
        tgt_E, tgt_N: target position in local (E, N) [m]

    Returns:
        (carrot_E, carrot_N) in local frame [m]
    """
    rope_len = math.hypot(tgt_E, tgt_N)
    if rope_len < 0.01:
        return tgt_E, tgt_N

    uE = tgt_E / rope_len
    uN = tgt_N / rope_len

    # Scalar projection of current position onto the start->target line
    s = max(0.0, min(my_E * uE + my_N * uN, rope_len))

    carrot_s = s + L_DISTANCE
    return carrot_s * uE, carrot_s * uN


# =============================================================================
# Figure-Eight Pattern Waypoint Generation
# =============================================================================
def _figure_eight_target(my_E: float, my_N: float,
                         tgt_E: float, tgt_N: float) -> tuple:
    """Generate a figure-eight pattern waypoint near the target.

    Purpose:
      The parafoil cannot reduce energy with throttle like a powered
      aircraft.  When above the target with excess altitude, the figure-eight
      pattern increases the horizontal flight path to dissipate altitude and
      energy before final approach.  The pattern phase increases horizontal
      path length to dissipate altitude/energy because the parafoil cannot
      reduce energy with throttle like a powered aircraft.

    Why figure-eight instead of simple circles:
      - Alternates left and right turns, reducing control surface bias
        and preventing persistent aerodynamic asymmetry buildup.
      - In crosswind, alternating turns give more balanced ground coverage
        than a single circle that would drift downwind.
      - Simpler to integrate with the existing 2D guidance pipeline than
        a full holding pattern or racetrack.

    Implementation:
      - The pattern center is at the target position.
      - Two lobe waypoints are placed perpendicular to the bearing from
        current position to target, at +/- RADIUS offset.
      - Lobes switch on a fixed time period (LOBE_PERIOD seconds).
        Time-based switching was chosen for simplicity of integration
        with the existing system.
      - The resulting waypoint is fed into the normal heading/yaw-rate
        cascade, so no special controller logic is needed.

    Lobe geometry (top view, wind from left):

                     .  lobe +1
                   .   .
        wind ->  .  [TGT]  .
                   .   .
                     .  lobe -1

    Args:
        my_E, my_N:   current position [m] in local (E, N)
        tgt_E, tgt_N: target position [m] in local (E, N)

    Returns:
        (wp_E, wp_N): pattern waypoint in local (E, N) [m]
    """
    now = time.time()

    # Time-based lobe switching
    if now - _pattern.last_switch_time > _pattern.LOBE_PERIOD:
        _pattern.lobe_sign *= -1
        _pattern.last_switch_time = now

    # Bearing from current position to target (from North, clockwise) [rad]
    bearing_to_tgt = math.atan2(tgt_E - my_E, tgt_N - my_N)

    # Place lobes perpendicular to the bearing direction.
    # perp_angle is 90 deg to the right of the bearing.
    perp_angle = bearing_to_tgt + math.pi / 2.0

    # Offset the lobe waypoint from the target center.
    # lobe_sign alternates +/-1, flipping the waypoint to the other side.
    offset_E = _pattern.RADIUS * _pattern.lobe_sign * math.sin(perp_angle)
    offset_N = _pattern.RADIUS * _pattern.lobe_sign * math.cos(perp_angle)

    return tgt_E + offset_E, tgt_N + offset_N


# =============================================================================
# Altitude Update (Barometer Integration)
# =============================================================================
def update_altitude(alt_m: float):
    """Receive barometer altitude [m AGL] and update L_DISTANCE.

    Called from motorapp when a barometer message arrives.

    The look-ahead distance is adjusted in three altitude bands:
      - Above ALT_HIGH (100 m): L_DISTANCE_HIGH (25 m)
        -> avoids oscillation from aggressive correction while far away
      - Between ALT_LOW and ALT_HIGH: L_DISTANCE_BASE (15 m)
        -> balanced tracking for normal descent
      - Below ALT_LOW (30 m): L_DISTANCE_LOW (10 m)
        -> tighter alignment for final-approach precision

    This is a simple altitude-based tuning, not a complex L1 parameter
    schedule.  L_DISTANCE can also be updated externally via apparg/MID
    if needed for ground-station override.

    Args:
        alt_m: altitude above ground level [m] from barometer
    """
    global altitude_m, L_DISTANCE
    altitude_m = alt_m

    if alt_m > ALT_HIGH:
        L_DISTANCE = L_DISTANCE_HIGH
    elif alt_m < ALT_LOW:
        L_DISTANCE = L_DISTANCE_LOW
    else:
        L_DISTANCE = L_DISTANCE_BASE


# =============================================================================
# Target Coordinate Management
# =============================================================================
def set_start_coordinates(lat: float, lon: float):
    """Set the local-frame origin (typically launch/landing site).

    All subsequent _llh_to_en() calls produce coordinates relative to
    this point.
    """
    start_point.lat = lat
    start_point.lon = lon


def set_target_coord(lat: float, lon: float):
    """Set the guidance target in geographic coordinates.

    Called when flightlogic sends updated target coordinates.
    """
    target.lat = lat
    target.lon = lon


def draw_pattern():
    """Legacy stub - pattern logic is now handled inside guidance().

    Kept for backward compatibility with callers that reference this
    function.  The actual figure-eight pattern is executed when
    guidance() is called with patterned=True.
    """
    return


# =============================================================================
# Yaw-Rate PI Controller (Inner Loop) with Conditional Anti-Windup
# =============================================================================
def _yaw_rate_pi_control(desired_yaw_rate: float,
                         measured_yaw_rate: float,
                         dt: float,
                         is_final: bool) -> tuple:
    """PI controller tracking the desired yaw rate (inner loop).

    The inner loop uses a PI law with simple conditional anti-windup.

    The outer loop (L1 guidance law) computes how fast the parafoil
    should turn (desired_yaw_rate); this function ensures the actual
    yaw rate (from the gyroscope) tracks that command.

    Control law:
        rate_error = desired_yaw_rate - measured_yaw_rate
        u = Kp * rate_error + Ki * integral(rate_error * dt)

    Anti-windup (conditional integration):
        Integrator windup is a phenomenon where the integral term keeps
        accumulating error even though the actuator is already at its
        physical limit (saturated).  When the disturbance eventually
        clears, the bloated integral causes a large overshoot before it
        can "unwind" back to normal levels.

        The current implementation uses the simplest form of anti-windup
        -- "conditional integration" -- which requires NO extra tuning
        parameters beyond the existing PI gains:

          - If the controller output is NOT saturated:
            -> integrate normally.
          - If saturated AND the error pushes DEEPER into saturation
            (rate_error has the same sign as the saturated output):
            -> FREEZE the integral (do not accumulate).
          - If saturated BUT the error is REDUCING saturation
            (rate_error has the opposite sign):
            -> integrate normally (helps the system recover faster).

        Why this matters for a parafoil:
          The servo actuators have hard physical limits (arm angle 0-120 deg).
          If the integral keeps growing while the servo is already at max
          deflection, the accumulated integral must unwind before the
          controller can respond to new heading errors -- causing sluggish
          overshoot after a turn.

    Args:
        desired_yaw_rate:  target yaw rate [deg/s] from outer loop
        measured_yaw_rate: gyroscope measurement [deg/s]
        dt: time step [s]
        is_final: True during final approach (uses reduced MAX_CMD)

    Returns:
        (u_before_sat, u_after_sat, integral)
        u_before_sat: raw PI output before saturation [deg/s]
        u_after_sat:  clamped PI output [deg/s] = commanded_yaw_rate
        integral:     current integral state [deg]
    """
    rate_error = desired_yaw_rate - measured_yaw_rate

    # Compute unsaturated output with current integral
    u_before_sat = (cascade_pi.Kp_inner * rate_error
                    + cascade_pi.Ki_inner * cascade_pi.pi_integral)

    # Saturation limit (reduced during final approach for safety)
    max_cmd = MAX_YAW_RATE_FINAL if is_final else cascade_pi.MAX_CMD

    # Apply saturation
    u_after_sat = max(-max_cmd, min(max_cmd, u_before_sat))

    # --- Conditional anti-windup ---
    # Check: is the output saturated (beyond the actuator limit)?
    is_saturated = abs(u_before_sat) > max_cmd
    # Check: does the error push the output DEEPER into saturation?
    # (both rate_error and u_before_sat have the same sign)
    error_deepens_sat = (rate_error * u_before_sat > 0.0)

    if is_saturated and error_deepens_sat:
        # Actuator at limit, error pushing further -> FREEZE integral
        # This prevents the integral from growing unbounded during
        # sustained saturation.
        pass
    else:
        # Either not saturated, or error helps recovery -> integrate
        cascade_pi.pi_integral += rate_error * dt
        # Hard clamp as safety net (should rarely activate with anti-windup)
        cascade_pi.pi_integral = max(
            -cascade_pi.MAX_INTEGRAL,
            min(cascade_pi.MAX_INTEGRAL, cascade_pi.pi_integral)
        )

    # Recompute output after potential integral update
    u_before_sat = (cascade_pi.Kp_inner * rate_error
                    + cascade_pi.Ki_inner * cascade_pi.pi_integral)
    u_after_sat = max(-max_cmd, min(max_cmd, u_before_sat))

    return u_before_sat, u_after_sat, cascade_pi.pi_integral


# =============================================================================
# Main Guidance Entry Point
# =============================================================================
def guidance(imu_data, gps_data, target_data,
             patterned: bool = False) -> types.SimpleNamespace:
    """Main guidance function -- called every control cycle (~10 Hz).

    Orchestrates the full guidance-to-command pipeline:

      Step 1 - Guidance layer ("where should we go?")
        Depending on flight phase (homing / pattern / final approach),
        selects a guidance waypoint and computes the desired course.
        - HOMING:  L1 carrot-chase along the start-to-target line.
        - PATTERN: Figure-eight near the target to dissipate energy.
        - FINAL:   Straight-in to target with reduced aggressiveness.

      Step 2 - Wind compensation
        Estimates the crab angle from GPS course vs. IMU heading and
        adjusts the desired heading to compensate for steady wind.

      Step 3 - Outer loop: heading error -> desired yaw rate
        Uses the L1 lateral-acceleration law:
          desired_yaw_rate = 2 * V / L * sin(heading_error)
        This naturally scales: faster speed or shorter L -> stronger turn.

      Step 4 - Inner loop: PI on yaw-rate error -> commanded_yaw_rate
        Tracks desired_yaw_rate with conditional anti-windup.
        Output is passed to the actuator mixer (motor_control.py).

    The function does NOT command actuators directly.  It returns a
    namespace with all values needed by the actuator mixer and for logging.

    Args:
        imu_data:    namespace with .yaw [deg], .gyrz [deg/s]
                     NOTE: verify that gyrz units match the expected deg/s;
                     if the IMU outputs rad/s, a conversion is needed.
        gps_data:    namespace with .lat, .lon [deg], .speed [m/s],
                     .course [deg], .fix_quality, .sats, .rmc_status
        target_data: namespace with .lat, .lon [deg]
        patterned:   if True, execute figure-eight pattern mode

    Returns:
        SimpleNamespace containing all guidance outputs:
          .commanded_yaw_rate [deg/s] - for actuator mixer
          .heading_error      [deg]
          .desired_course     [deg]
          .desired_heading    [deg]
          .desired_yaw_rate   [deg/s] - from outer loop before PI
          .measured_yaw_rate  [deg/s] - gyro reading
          .u_before_sat       [deg/s] - PI output before saturation
          .u_after_sat        [deg/s] - PI output after saturation
          .integral           [deg]   - PI integral state
          .wind_effect        [deg]   - crab angle estimate
          .distance           [m]     - distance to target
          .altitude           [m]     - barometer altitude AGL
          .l_distance         [m]     - current look-ahead distance
          .patterned          [bool]
          .pattern_wp_E/N     [m]     - pattern waypoint if applicable
          .state              [str]   - phase label
          .gps_speed          [m/s]
          .gps_course         [deg]
          .yaw                [deg]
    """
    global wind_effect, last_time

    # -- Timing --
    now = time.time()
    dt = now - last_time if last_time else 0.1
    if dt <= 0.02 or dt > 0.5:
        dt = 0.1  # guard against timer glitches
    last_time = now

    # -- Helper: build result namespace with defaults for early returns --
    def _result(**kw):
        defaults = dict(
            commanded_yaw_rate=0.0, heading_error=0.0,
            desired_course=0.0, desired_heading=0.0,
            desired_yaw_rate=0.0, measured_yaw_rate=imu_data.gyrz,
            u_before_sat=0.0, u_after_sat=0.0,
            integral=cascade_pi.pi_integral,
            wind_effect=wind_effect, distance=0.0,
            altitude=altitude_m, l_distance=L_DISTANCE,
            patterned=patterned,
            pattern_wp_E=0.0, pattern_wp_N=0.0,
            state="UNKNOWN",
            gps_speed=gps_data.speed, gps_course=gps_data.course,
            yaw=imu_data.yaw
        )
        defaults.update(kw)
        return types.SimpleNamespace(**defaults)

    # ===== GPS validity gate =====
    # If GPS is unreliable, return zero command (neutral) immediately.
    # The actuator mixer will set servos to neutral position.
    if not is_gps_valid(gps_data.lat, gps_data.lon,
                        gps_data.fix_quality, gps_data.sats,
                        gps_data.rmc_status):
        return _result(state="GPS_INVALID")

    # ===== Convert to local (E, N) frame =====
    # From this point on, all positions are in the (E, N) local frame.
    my_E, my_N = _llh_to_en(gps_data.lat, gps_data.lon)
    tgt_E, tgt_N = _llh_to_en(target_data.lat, target_data.lon)
    distance = math.hypot(tgt_E - my_E, tgt_N - my_N)

    # ===== Target reached check =====
    if distance < 5.0:
        return _result(distance=distance, state="TARGET_REACHED")

    # ===== Step 1: Select guidance waypoint based on flight phase =====
    pattern_wp_E, pattern_wp_N = 0.0, 0.0

    # Final approach: barometer reports valid altitude AND altitude is low
    is_final = (altitude_m > 0.0 and altitude_m < FINAL_APPROACH_ALT)

    if patterned and not is_final:
        # ---- PATTERN mode ----
        # Figure-eight near target to dissipate excess altitude/energy.
        # The pattern waypoint replaces the normal carrot; the heading
        # and yaw-rate cascade runs unchanged.
        pattern_wp_E, pattern_wp_N = _figure_eight_target(
            my_E, my_N, tgt_E, tgt_N
        )
        guide_E, guide_N = pattern_wp_E, pattern_wp_N
        phase = "PATTERN"

    elif is_final:
        # ---- FINAL APPROACH ----
        # Go straight to target with reduced turn aggressiveness.
        #
        # Final approach is critical because:
        #   - Little altitude margin for recovery from overshoots.
        #   - Sharp turns at low altitude risk stall or ground strike.
        #   - A predictable straight-in path minimizes landing error.
        #   - The parafoil glide ratio is fixed; there is no go-around
        #     option, so the final alignment must be right the first time.
        guide_E, guide_N = tgt_E, tgt_N
        phase = "FINAL"

    else:
        # ---- HOMING mode ----
        # L1 carrot-chase: follow the start-to-target line with look-ahead.
        cE, cN = _carrot(my_E, my_N, tgt_E, tgt_N)
        guide_E, guide_N = cE, cN
        phase = "HOMING"

    # ===== Desired course toward guidance waypoint =====
    # heading = atan2(delta_E, delta_N)
    # Why atan2(E, N) and not atan2(N, E)?
    #   Heading is the angle FROM the North axis, measured clockwise.
    #   atan2(y, x) gives the angle from the x-axis.  By placing East (E)
    #   in the y-slot and North (N) in the x-slot, we get the angle from
    #   North directly, with East = +90 deg -- exactly the aviation heading
    #   convention.
    desired_course = math.degrees(
        math.atan2(guide_E - my_E, guide_N - my_N)
    )

    # ===== Step 2: Wind / crab-angle compensation =====
    # Crab angle = GPS ground track (course) - body heading (yaw).
    # If the wind blows from the right, the parafoil crabs left to maintain
    # ground track, resulting in a negative crab angle.  We compensate by
    # aiming the heading to the right of the desired course.
    #
    # A first-order low-pass filter (alpha = 0.05) smooths noisy estimates.
    # Only update when moving (speed > 1 m/s) and not spinning (|gyrz| < 20).
    if gps_data.speed > 1.0 and abs(imu_data.gyrz) < 20.0:
        current_crab = _wrap_180(gps_data.course - imu_data.yaw)
        wind_effect = 0.95 * wind_effect + 0.05 * current_crab

    desired_heading = _wrap_180(desired_course - wind_effect)

    # ===== Step 3: Outer loop - heading error -> desired yaw rate =====
    # L1 lateral-acceleration law (ArduPilot-style):
    #   a_lat    = 2 * V^2 / L * sin(eta)       [m/s^2]
    #   yaw_rate = a_lat / V = 2 * V / L * sin(eta)   [rad/s]
    #
    # where:
    #   eta = heading error [rad]
    #   V   = ground speed [m/s]
    #   L   = look-ahead distance [m]
    #
    # This naturally scales: faster speed or shorter L -> stronger correction.
    # At small heading errors, sin(eta) ~ eta, so the law is nearly linear.
    # At large errors, sin(eta) saturates at 1, providing a natural limit.
    heading_error = _wrap_180(desired_heading - imu_data.yaw)
    V = max(gps_data.speed, 1.0)  # floor at 1 m/s to avoid instability
    desired_yaw_rate = math.degrees(
        2.0 * (V / L_DISTANCE) * math.sin(math.radians(heading_error))
    )

    # ===== Step 4: Inner loop - PI yaw-rate controller =====
    if abs(heading_error) <= cascade_pi.DEADBAND:
        # Within deadband: fly straight, clear integral to prevent drift
        cascade_pi.pi_integral = 0.0
        u_before_sat = 0.0
        u_after_sat = 0.0
        if phase == "HOMING":
            phase = "STRAIGHT"
    else:
        u_before_sat, u_after_sat, _ = _yaw_rate_pi_control(
            desired_yaw_rate, imu_data.gyrz, dt, is_final
        )
        if phase == "HOMING":
            phase = "TURNING"

    return _result(
        commanded_yaw_rate=u_after_sat,
        heading_error=heading_error,
        desired_course=desired_course,
        desired_heading=desired_heading,
        desired_yaw_rate=desired_yaw_rate,
        u_before_sat=u_before_sat,
        u_after_sat=u_after_sat,
        integral=cascade_pi.pi_integral,
        distance=distance,
        pattern_wp_E=pattern_wp_E,
        pattern_wp_N=pattern_wp_N,
        state=phase
    )
