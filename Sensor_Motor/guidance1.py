







# ── Legacy guidance constants ─────────────────────────────────────────────────
import enum


GPS_STABLE_COUNT_REQUIRED = 1
LANDING_YR_MAX  = 30.0   # deg/s
YR_MAX          = 45.0   # deg/s
L_DISTANCE      = 30.0   # m

# ── Legacy module-level state ─────────────────────────────────────────────────
_PREV_GPS = SimpleNamespace(initialized=False, lat=0.0, lon=0.0, time=0.0)
_GPS_STABLE_COUNT: int = 0
_START_LAT: Optional[float] = None
_START_LON: Optional[float] = None
START_POINT = SimpleNamespace(lat=0.0, lon=0.0)
CASCADE_PI  = SimpleNamespace(MAX_CMD=YR_MAX)









# ═══════════════════════════════════════════════════════════════════════════════
# SimpleNamespace factories
# ═══════════════════════════════════════════════════════════════════════════════

def _gps_sns():
    """GPS sensor reading — shared shape for raw and last_good."""
    return SimpleNamespace(
        lat=None, lon=None, course_rad=None, speed_mps=None,
        ts=None, pos_health=False, motion_health=False,
    )


def _imu_sns():
    """IMU sensor reading — shared shape for raw and last_good."""
    return SimpleNamespace(gyrz_rad_s=None, ts=None, health=False)


def _baro_sns():
    """Baro sensor reading — shared shape for raw and last_good."""
    return SimpleNamespace(alt_m=None, ts=None, health=False)


def make_resolver_state():
    """Create resolver state holding raw and last_good sensor caches."""
    return SimpleNamespace(
        raw_gps=_gps_sns(),  lg_gps=_gps_sns(),
        raw_imu=_imu_sns(),  lg_imu=_imu_sns(),
        raw_baro=_baro_sns(), lg_baro=_baro_sns(),
        origin_lat=None, origin_lon=None,
    )


def L1Input(timestamp):
    """Create guidance pipeline state for one control cycle."""
    return SimpleNamespace(
        timestamp=timestamp,
        pos_N=None, pos_E=None,          pos_status=SensorQuality.MISSING,
        course=None,                      motion_health=SensorQuality.MISSING,
        ground_speed_mps=None,
        gyrz=None,                        gyrz_health=SensorQuality.MISSING,
        altitude=None,                    alt_health=SensorQuality.MISSING,
        lcsg_case="----", input_policy="",
        control_mode=ControlMode.SAFE_GLIDE, reason="",
    )

GuidanceInput = L1Input  # backward-compat alias


def L1Config(**kwargs):
    """Create L1 algorithm configuration (all fields overridable via kwargs)."""
    ns = SimpleNamespace(
        damping=L1_DAMPING, period=L1_PERIOD_S, L1_min=L1_MIN_M,
        v_min=V_MIN_MPS, lat_acc_max=LAT_ACC_MAX, course_rate_max=COURSE_RATE_MAX,
    )
    for k, v in kwargs.items():
        setattr(ns, k, v)
    return ns


def L1Output(timestamp):
    """Create L1 guidance output."""
    return SimpleNamespace(
        timestamp=timestamp,
        active=False, degraded=False, reason="",
        crossTrack=float("nan"), alongTrack=float("nan"),
        L1_dist=float("nan"),
        Nu1=float("nan"), Nu2=float("nan"), Nu=float("nan"),
        lat_acc_cmd_mps2=float("nan"),
        course_rate_cmd_rad_s=float("nan"),
        submode="SAFE_GLIDE",
        start_lat=float("nan"), start_lon=float("nan"),
        target_lat=float("nan"), target_lon=float("nan"),
        carrot_lat=float("nan"), carrot_lon=float("nan"),
        current_heading_deg=float("nan"), desired_heading_deg=float("nan"),
    )


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
# Resolver update functions
# ═══════════════════════════════════════════════════════════════════════════════

def resolver_update_gnss(
    resolver,
    lat: float,
    lon: float,
    course_rad: float,
    groundSpeed: float,
    posHealth: bool,
    motionHealth: bool,
    ts: float,
) -> None:
    """Cache raw GPS reading; update last_good only when health=True."""
    r = resolver.raw_gps
    r.lat          = lat
    r.lon          = lon
    r.course_rad   = course_rad
    r.speed_mps    = groundSpeed
    r.ts           = ts
    r.pos_health   = bool(posHealth)
    r.motion_health = bool(motionHealth)

    if posHealth:
        lg = resolver.lg_gps
        lg.lat       = lat
        lg.lon       = lon
        lg.ts        = ts
        lg.pos_health = True
    if motionHealth:
        lg = resolver.lg_gps
        lg.course_rad  = course_rad
        lg.speed_mps   = groundSpeed
        lg.ts          = ts
        lg.motion_health = True


def resolver_update_imu(
    resolver,
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
    r = resolver.raw_imu
    r.gyrz_rad_s = gz
    r.ts         = ts
    r.health     = bool(imu_health)

    if imu_health:
        lg = resolver.lg_imu
        lg.gyrz_rad_s = gz
        lg.ts         = ts
        lg.health     = True


def resolver_update_baro(
    resolver,
    alt: float,
    ts: float,
    baro_health: bool = False,
) -> None:
    """Cache raw barometer reading; update last_good when health=True."""
    r = resolver.raw_baro
    r.alt_m  = alt
    r.ts     = ts
    r.health = bool(baro_health)

    if baro_health:
        lg = resolver.lg_baro
        lg.alt_m  = alt
        lg.ts     = ts
        lg.health = True


def resolver_set_origin(resolver, lat: float, lon: float) -> None:
    resolver.origin_lat = lat
    resolver.origin_lon = lon


def resolver_reset_origin(resolver) -> None:
    resolver.origin_lat = None
    resolver.origin_lon = None


# ═══════════════════════════════════════════════════════════════════════════════
# fill_current_data / fill_stale_data / decide_control_mode
# ═══════════════════════════════════════════════════════════════════════════════

def fill_current_data(resolver, state, now: float) -> None:
    """Classify each field as FRESH/STALE based on health flag and age.

    FRESH  : health=True AND age <= fresh threshold → write value
    STALE  : health=False OR age > threshold        → mark STALE, do NOT write value
             (fill_stale_data will supply a value from last_good or raw)
    """
    origin_lat = resolver.origin_lat or 0.0
    origin_lon = resolver.origin_lon or 0.0
    rg = resolver.raw_gps
    ri = resolver.raw_imu
    rb = resolver.raw_baro

    # ── Position (L) ──────────────────────────────────────────────────────────
    if rg.ts is not None and rg.pos_health and (now - rg.ts) <= POS_FRESH_AGE:
        state.pos_status = SensorQuality.FRESH
        state.pos_N, state.pos_E = _ll_to_ne(rg.lat, rg.lon, origin_lat, origin_lon)
    elif rg.ts is not None:
        state.pos_status = SensorQuality.STALE

    # ── Course / motion (C + S) ───────────────────────────────────────────────
    if rg.ts is not None and rg.motion_health and (now - rg.ts) <= MOTION_FRESH_AGE:
        state.motion_health    = SensorQuality.FRESH
        state.course           = rg.course_rad
        state.ground_speed_mps = rg.speed_mps
    elif rg.ts is not None:
        state.motion_health = SensorQuality.STALE

    # ── GyroZ (G) ─────────────────────────────────────────────────────────────
    if ri.ts is not None and ri.health and (now - ri.ts) <= GYRZ_FRESH_AGE:
        state.gyrz_health = SensorQuality.FRESH
        state.gyrz        = ri.gyrz_rad_s
    elif ri.ts is not None:
        state.gyrz_health = SensorQuality.STALE

    # ── Altitude ──────────────────────────────────────────────────────────────
    if rb.ts is not None and rb.health and (now - rb.ts) <= ALT_FRESH_AGE:
        state.alt_health = SensorQuality.FRESH
        state.altitude   = rb.alt_m
    elif rb.ts is not None:
        state.alt_health = SensorQuality.STALE


def fill_stale_data(resolver, state, now: float) -> None:
    """For STALE fields, supply values from last_good (within stale window)
    or fall back to raw values if last_good is absent.
    FRESH fields are not touched.
    """
    origin_lat = resolver.origin_lat or 0.0
    origin_lon = resolver.origin_lon or 0.0
    rg = resolver.raw_gps
    lg = resolver.lg_gps
    ri = resolver.raw_imu
    li = resolver.lg_imu
    rb = resolver.raw_baro
    lb = resolver.lg_baro

    # ── Position ──────────────────────────────────────────────────────────────
    if state.pos_status == SensorQuality.STALE and state.pos_N is None:
        if lg.ts is not None and lg.lat is not None and (now - lg.ts) <= POS_STALE_MAX:
            state.pos_N, state.pos_E = _ll_to_ne(lg.lat, lg.lon, origin_lat, origin_lon)
        elif rg.lat is not None and rg.ts is not None and (now - rg.ts) <= POS_STALE_MAX:
            state.pos_N, state.pos_E = _ll_to_ne(rg.lat, rg.lon, origin_lat, origin_lon)
        else:
            state.pos_status = SensorQuality.MISSING

    # ── Course / speed ────────────────────────────────────────────────────────
    if state.motion_health == SensorQuality.STALE and state.course is None:
        if lg.ts is not None and lg.course_rad is not None and (now - lg.ts) <= MOTION_STALE_MAX:
            state.course           = lg.course_rad
            state.ground_speed_mps = lg.speed_mps
        elif rg.course_rad is not None and rg.ts is not None and (now - rg.ts) <= MOTION_STALE_MAX:
            state.course           = rg.course_rad
            state.ground_speed_mps = rg.speed_mps
        else:
            state.motion_health = SensorQuality.MISSING

    # ── GyroZ ─────────────────────────────────────────────────────────────────
    if state.gyrz_health == SensorQuality.STALE and state.gyrz is None:
        if li.ts is not None and li.gyrz_rad_s is not None and (now - li.ts) <= GYRZ_STALE_MAX:
            state.gyrz = li.gyrz_rad_s
        elif ri.gyrz_rad_s is not None and ri.ts is not None and (now - ri.ts) <= GYRZ_STALE_MAX:
            state.gyrz = ri.gyrz_rad_s
        else:
            state.gyrz_health = SensorQuality.MISSING

    # ── Altitude ──────────────────────────────────────────────────────────────
    if state.alt_health == SensorQuality.STALE and state.altitude is None:
        if lb.ts is not None and lb.alt_m is not None and (now - lb.ts) <= ALT_STALE_MAX:
            state.altitude = lb.alt_m
        elif rb.alt_m is not None and rb.ts is not None and (now - rb.ts) <= ALT_STALE_MAX:
            state.altitude = rb.alt_m
        else:
            state.alt_health = SensorQuality.MISSING


def decide_control_mode(state) -> None:
    """Classify LCSG and set control_mode + reason on the L1Input.

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

    L_miss  = state.pos_status    == SensorQuality.MISSING
    C_miss  = state.motion_health == SensorQuality.MISSING

    lcsg = (
        ("L" if L_fresh else ("-" if L_miss else "l"))
        + ("C" if C_fresh else ("-" if C_miss else "c"))
        + ("S" if S_fresh else "-")
        + ("G" if G_fresh else "-")
    )
    state.lcsg_case = lcsg

    if L_miss or C_miss or (not L_fresh and not L_stale) or (not C_fresh and not C_stale):
        state.control_mode = ControlMode.SAFE_GLIDE
        state.reason       = "MISSING position or course → safe_glide"
        return

    any_stale     = L_stale or C_stale
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


def resolver_resolve(resolver, now: float):
    """Full resolve pipeline: fill_current → fill_stale → decide_mode."""
    state = L1Input(timestamp=now)
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
# L1 state factory and control functions
# ═══════════════════════════════════════════════════════════════════════════════

def make_l1_state(cfg=None):
    return SimpleNamespace(
        config=cfg or L1Config(),
        start_N=0.0, start_E=0.0,
        target_N=None, target_E=None,
        submode="LINE_FOLLOW",
    )


def l1_reset(l1) -> None:
    l1.start_N  = 0.0
    l1.start_E  = 0.0
    l1.target_N = None
    l1.target_E = None
    l1.submode  = "LINE_FOLLOW"


def l1_set_start(l1, N: float, E: float) -> None:
    l1.start_N = N
    l1.start_E = E


def l1_set_target(l1, N: float, E: float) -> None:
    l1.target_N = N
    l1.target_E = E
    l1.submode  = "LINE_FOLLOW"


def l1_update(l1, inp, now: float):
    """ArduPilot L1 update_waypoint port.

    Reference: AP_L1_Control::update_waypoint()
    sine_Nu1 clamped to ±0.7071 (AP L1 line ~260)
    Nu clamped to ±π/2 (prevent_indecision equivalent)
    """
    out = L1Output(timestamp=now)

    if l1.target_N is None or inp.control_mode == ControlMode.SAFE_GLIDE:
        out.submode = "SAFE_GLIDE"
        out.reason  = "safe_glide"
        return out

    speed  = max(inp.ground_speed_mps or 0.0, l1.config.v_min)
    course = inp.course or 0.0
    pos_N  = inp.pos_N  or 0.0
    pos_E  = inp.pos_E  or 0.0

    cfg     = l1.config
    K_L1    = 4.0 * cfg.damping ** 2
    L1_dist = max(cfg.damping * cfg.period / math.pi * speed, cfg.L1_min)

    xtrack_soft = max(XTRACK_SOFT_FACTOR * L1_dist, XTRACK_SOFT_MIN_M)
    xtrack_hard = max(XTRACK_HARD_FACTOR * L1_dist, XTRACK_HARD_MIN_M)

    if l1.submode == "DRAW_LINE":
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

    alongTrack = AP_N * unit_AB_N + AP_E * unit_AB_E
    crossTrack = AP_N * unit_AB_E - AP_E * unit_AB_N   # + = left of path

    out.crossTrack = crossTrack
    out.alongTrack = alongTrack
    out.L1_dist    = L1_dist

    if l1.submode != "DRAW_LINE" and abs(crossTrack) > xtrack_hard:
        l1.submode = "DRAW_LINE"

    vel_N = speed * math.cos(course)
    vel_E = speed * math.sin(course)

    if (
        alongTrack < 0.0
        and abs(alongTrack) > L1_dist
        and l1.submode != "DRAW_LINE"
    ):
        # Behind start waypoint A — steer toward AB direction
        xtrackVel = vel_N * unit_AB_E - vel_E * unit_AB_N
        ltrackVel = vel_N * unit_AB_N + vel_E * unit_AB_E
        Nu1 = math.asin(max(-0.7071, min(0.7071, crossTrack / L1_dist)))
        Nu2 = math.atan2(xtrackVel, ltrackVel)

    elif alongTrack > AB_len + speed * 3.0:
        # Past end waypoint B — steer directly to B
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
        # Normal tracking — between A and B (or DRAW_LINE)
        xtrackVel = vel_N * unit_AB_E - vel_E * unit_AB_N
        ltrackVel = vel_N * unit_AB_N + vel_E * unit_AB_E
        Nu1 = math.asin(max(-0.7071, min(0.7071, crossTrack / L1_dist)))
        Nu2 = math.atan2(xtrackVel, ltrackVel)

    Nu = Nu1 + Nu2
    Nu = max(-math.pi / 2, min(math.pi / 2, Nu))

    lat_acc = K_L1 * speed ** 2 / L1_dist * math.sin(Nu)
    lat_acc = max(-LAT_ACC_MAX, min(LAT_ACC_MAX, lat_acc))

    course_rate = lat_acc / max(speed, V_MIN_MPS)
    course_rate = max(-COURSE_RATE_MAX, min(COURSE_RATE_MAX, course_rate))

    out.Nu1 = Nu1
    out.Nu2 = Nu2
    out.Nu  = Nu
    out.lat_acc_cmd_mps2      = lat_acc
    out.course_rate_cmd_rad_s = course_rate
    out.active   = True
    out.degraded = inp.control_mode in (
        ControlMode.DEGRADED_CLOSED_LOOP, ControlMode.DEGRADED_FEEDFORWARD
    )

    if l1.submode == "DRAW_LINE":
        out.submode = "DRAW_LINE"
    elif abs(crossTrack) > xtrack_soft:
        out.submode = "REJOIN"
    else:
        out.submode = "LINE_FOLLOW"

    return out


# ═══════════════════════════════════════════════════════════════════════════════
# compute_motor_output
# ═══════════════════════════════════════════════════════════════════════════════

def compute_motor_output(l1, controller, inp, now: float):
    """Run L1 guidance then brake controller.

    Returns (BrakeCommand, L1Output).
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
# Legacy guidance() function (backward compat)
# ═══════════════════════════════════════════════════════════════════════════════

def init_guidance() -> None:
    global _PREV_GPS, _GPS_STABLE_COUNT, _START_LAT, _START_LON, START_POINT
    _PREV_GPS = SimpleNamespace(initialized=False, lat=0.0, lon=0.0, time=0.0)
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
    return _check_gps_jump(float(lat), float(lon), time.time())


def _check_gps_jump(lat: float, lon: float, ts: float) -> bool:
    global _PREV_GPS, _GPS_STABLE_COUNT

    MAX_JUMP_DEG = 0.01

    if not _PREV_GPS.initialized:
        _PREV_GPS.initialized = True
        _PREV_GPS.lat  = lat
        _PREV_GPS.lon  = lon
        _PREV_GPS.time = ts
        _GPS_STABLE_COUNT = 0
        return False

    dt = ts - _PREV_GPS.time
    if dt < 0.5:
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
    """Legacy guidance function for backwards compatibility."""
    ts = time.time()

    out = SimpleNamespace(
        state="GPS_INVALID",
        distance=float("nan"),
        commanded_yaw_rate=0.0,
    )

    if not fid.pos_health:
        return out

    lat = float(gps.lat)
    lon = float(gps.lon)

    if not _check_gps_jump(lat, lon, ts):
        return out

    if tgt is None:
        out.state    = "TARGET_UNSET"
        out.distance = float("nan")
        return out

    if _START_LAT is None or _START_LON is None:
        out.state    = "START_UNSET"
        out.distance = float("nan")
        return out

    tgt_lat    = float(tgt.lat)
    tgt_lon    = float(tgt.lon)
    distance_m = _haversine_distance_m(lat, lon, tgt_lat, tgt_lon)
    out.distance = distance_m

    if distance_m < 5.0:
        out.state              = "TARGET_REACHED"
        out.commanded_yaw_rate = 0.0
        return out

    bearing_to_carrot_deg = _haversine_bearing_deg(lat, lon, tgt_lat, tgt_lon)
    gps_track_deg         = float(gps.direction)

    eta_deg = _wrap_180(bearing_to_carrot_deg - gps_track_deg)
    eta_deg = max(-90.0, min(90.0, eta_deg))

    V = max(float(gps.velocity), 0.5)
    accel_lat        = 2.0 * V ** 2 / L_DISTANCE * math.sin(math.radians(eta_deg))
    desired_yaw_rate = math.degrees(accel_lat / V)

    yr_max = LANDING_YR_MAX if alt < 20.0 else YR_MAX
    desired_yaw_rate = max(-yr_max, min(yr_max, desired_yaw_rate))

    if abs(eta_deg) < 5.0:
        state_label = "STRAIGHT"
    elif abs(eta_deg) < 45.0:
        state_label = "TURNING"
    else:
        state_label = "PATTERN"

    out.state              = state_label
    out.commanded_yaw_rate = desired_yaw_rate
    return out
