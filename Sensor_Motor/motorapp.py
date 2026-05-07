"""Motor app: sensor ingestion, L1 guidance, and brake control.

Architecture:
  sensor apps → GuidanceInputResolver → L1Guidance → ParafoilBrakeController → servo

Responsibilities:
  - Receive sensor messages, update GuidanceInputResolver
  - ~10 Hz control loop: resolve input → guidance → controller → servo PWM
  - STATE gate: active guidance only in STATE 3 / 4
  - Burnwire and egg-drop activation (pass-through to hardware threads)
  - Send diagnostics to Comm

motorapp does NOT perform sensor reliability checks.
posHealth / motionHealth from GNSS are the only availability signals honoured.
"""
from __future__ import annotations

import logging
import math
import threading
import time
from types import SimpleNamespace
from typing import Optional

from lib import appargs, config, msgstructure
from Sensor_Motor import motor_guidance
from Sensor_Motor import Motor_Egg, Motor_Release
from Sensor_Motor.motor_guidance import (
    GuidanceInputResolver,
    L1Guidance,
    L1Config,
    GuidanceMode,
    GuidanceOutput,
    _ll_to_ne,
)
from Sensor_Motor.motor_control import (
    ParafoilBrakeController,
    ControlConfig,
    GuidanceCommand,
    BrakeCommand,
    init_control,
    set_neutral,
    set_motors_off,
    set_brake_command,
    terminate_control,
    LEFT_NEUTRAL,
    RIGHT_NEUTRAL,
)

LOGGER = logging.getLogger(__name__)

# ── Runtime state ──────────────────────────────────────────────────────────────
MOTORAPP_RUNSTATUS: bool = True
MOTOR_ENABLED: bool      = True
STATE: int               = 0
PI                       = None

_UPDATE_LOCK = threading.Lock()

_INPUT_RESOLVER = GuidanceInputResolver()
_GUIDANCE       = L1Guidance(L1Config())
_CONTROLLER     = ParafoilBrakeController(ControlConfig())

_TARGET_LAT: Optional[float] = None
_TARGET_LON: Optional[float] = None

# Legacy observable state kept for tests/replay scripts during migration.
IMU = SimpleNamespace(yaw=0.0, gyrz=0.0, imu_health=0)
GPS_VECTOR = SimpleNamespace(lat=0.0, lon=0.0, direction=0.0, velocity=0.0)
GPS_HEALTH = SimpleNamespace(pos_health=0, motion_health=0)
TARGET = None
ALT = 0.0
_PREV_STATE = -1
_START_POINT_LOCKED = False


def _motor_rate_hz() -> float:
    try:
        return max(0.1, float(config.MOTOR_RATE_HZ))
    except (TypeError, ValueError):
        return 10.0


def _motor_period_sec() -> float:
    return 1.0 / _motor_rate_hz()


# ── Message handlers ───────────────────────────────────────────────────────────

def handle_gnss(data: str) -> None:
    """lat,lon,course_deg,groundSpeed_ms,posHealth,motionHealth"""
    fields = data.split(",")
    if len(fields) != 6:
        LOGGER.warning("GNSS parse: expected 6 fields | raw=%r", data)
        return
    try:
        lat          = float(fields[0])
        lon          = float(fields[1])
        course_deg   = float(fields[2])
        groundSpeed  = float(fields[3])
        posHealth    = bool(int(float(fields[4])))
        motionHealth = bool(int(float(fields[5])))
    except (ValueError, IndexError) as exc:
        LOGGER.warning("GNSS parse error: %s | raw=%r", exc, data)
        return

    with _UPDATE_LOCK:
        GPS_VECTOR.lat = lat
        GPS_VECTOR.lon = lon
        GPS_VECTOR.direction = course_deg
        GPS_VECTOR.velocity = groundSpeed
        GPS_HEALTH.pos_health = int(posHealth)
        GPS_HEALTH.motion_health = int(motionHealth)
        _INPUT_RESOLVER.update_gnss(
            lat, lon,
            math.radians(course_deg), groundSpeed,
            posHealth, motionHealth,
            time.time(),
        )
        _lock_start_for_legacy_if_ready()
        _maybe_push_target()


def handle_gps(data: str) -> None:
    """Compatibility handler for legacy GPS payloads."""
    fields = data.split(",")
    if len(fields) == 6:
        handle_gnss(data)
        return
    if len(fields) >= 8:
        try:
            lat = float(fields[0])
            lon = float(fields[1])
            speed = float(fields[2])
            course = float(fields[3])
            fix_quality = int(float(fields[4]))
            sats = int(float(fields[5]))
            rmc_status = fields[6].strip().upper()
            gps_health = int(float(fields[7]))
        except (ValueError, IndexError) as exc:
            LOGGER.warning("GPS parse error: %s | raw=%r", exc, data)
            return
        ok = fix_quality >= 1 and sats >= 4 and rmc_status == "A" and gps_health >= 1
        handle_gnss(f"{lat},{lon},{course},{speed},{int(ok)},{int(ok)}")
        return
    LOGGER.warning("GPS parse: expected 6 or 8 fields | raw=%r", data)


def handle_imu(data: str) -> None:
    """
    Full format (9 fields): roll,pitch,yaw,ax,ay,az,gx,gy,gz
    Legacy format (3 fields): yaw_deg,gyrz_deg_s,imu_health
    """
    fields = data.split(",")
    ts = time.time()
    try:
        if len(fields) >= 9:
            v = [float(f) for f in fields[:9]]
            with _UPDATE_LOCK:
                IMU.yaw = v[2]
                IMU.gyrz = v[8]
                IMU.imu_health = 1
                _INPUT_RESOLVER.update_imu(
                    roll=v[0], pitch=v[1], yaw=v[2],
                    ax=v[3],   ay=v[4],   az=v[5],
                    gx=v[6],   gy=v[7],   gz=v[8],
                    ts=ts,
                )
        elif len(fields) >= 3:
            # Legacy: yaw_deg, gyrz_deg_s, imu_health
            yaw_deg   = float(fields[0])
            gyrz_degs = float(fields[1])
            imu_health = int(float(fields[2]))
            with _UPDATE_LOCK:
                IMU.yaw = yaw_deg
                IMU.gyrz = gyrz_degs
                IMU.imu_health = imu_health
                _INPUT_RESOLVER.update_imu(yaw=yaw_deg, gz=gyrz_degs, ts=ts)
        else:
            LOGGER.warning("IMU parse: too few fields | raw=%r", data)
    except (ValueError, IndexError) as exc:
        LOGGER.warning("IMU parse error: %s | raw=%r", exc, data)


def handle_barometer(data: str) -> None:
    """altitude_m[,...]"""
    global ALT
    try:
        alt = float(data.split(",")[0].strip())
    except (ValueError, IndexError) as exc:
        LOGGER.warning("Baro parse error: %s | raw=%r", exc, data)
        return
    with _UPDATE_LOCK:
        ALT = alt
        _INPUT_RESOLVER.update_baro(alt, time.time())


def handle_target_coord(data: str) -> None:
    """lat,lon"""
    global _TARGET_LAT, _TARGET_LON, TARGET
    fields = data.split(",")
    if len(fields) != 2:
        LOGGER.warning("Target parse: expected 2 fields | raw=%r", data)
        return
    try:
        lat = float(fields[0])
        lon = float(fields[1])
    except (ValueError, IndexError) as exc:
        LOGGER.warning("Target parse error: %s | raw=%r", exc, data)
        return
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        LOGGER.warning("Target coord out of range: %.6f, %.6f", lat, lon)
        return
    with _UPDATE_LOCK:
        _TARGET_LAT = lat
        _TARGET_LON = lon
        TARGET = SimpleNamespace(lat=lat, lon=lon)
        _maybe_push_target()
    LOGGER.info("Target updated: %.6f, %.6f", lat, lon)


def handle_flight_state(data: str) -> None:
    global STATE, _PREV_STATE, _START_POINT_LOCKED
    try:
        new_state = int(data.split(",")[0])
    except (ValueError, IndexError) as exc:
        LOGGER.warning("State parse error: %s | raw=%r", exc, data)
        return
    if new_state == STATE:
        return
    LOGGER.info("State %d → %d", STATE, new_state)
    with _UPDATE_LOCK:
        _PREV_STATE = STATE
        STATE = new_state
        if new_state < 3:
            _GUIDANCE.reset()
            _CONTROLLER.reset()
            _INPUT_RESOLVER.reset_origin()
            _START_POINT_LOCKED = False
        elif new_state in (3, 4):
            _lock_start_for_legacy_if_ready()


def handle_release() -> None:
    threading.Thread(
        target=Motor_Release.activate_burnwire, daemon=True, name="Burnwire"
    ).start()


def handle_egg_drop() -> None:
    threading.Thread(
        target=Motor_Egg.activate_solenoid, daemon=True, name="Solenoid"
    ).start()


def handle_mec(data: str) -> None:
    global MOTOR_ENABLED
    cmd = data.strip().upper()
    if cmd == "ON":
        MOTOR_ENABLED = True
        LOGGER.info("MOTOR_ENABLED = True")
    elif cmd == "OFF":
        MOTOR_ENABLED = False
        with _UPDATE_LOCK:
            if PI is not None:
                set_neutral(PI)
        LOGGER.info("MOTOR_ENABLED = False → neutral")
    else:
        LOGGER.warning("Unknown MEC command: %r", data)


# ── Helpers ────────────────────────────────────────────────────────────────────

def _maybe_push_target() -> None:
    """Convert target lat/lon to N/E and push to guidance. Call under _UPDATE_LOCK."""
    if _TARGET_LAT is None or _TARGET_LON is None:
        return
    if _INPUT_RESOLVER.origin_lat is None:
        return
    tgt_N, tgt_E = _ll_to_ne(
        _TARGET_LAT, _TARGET_LON,
        _INPUT_RESOLVER.origin_lat, _INPUT_RESOLVER.origin_lon,
    )
    _GUIDANCE.set_target(tgt_N, tgt_E)


def _lock_start_for_legacy_if_ready() -> None:
    """Mirror the active start point into legacy guidance state."""
    global _START_POINT_LOCKED
    if _START_POINT_LOCKED or STATE < 3:
        return
    if not GPS_HEALTH.pos_health:
        return
    lat = float(GPS_VECTOR.lat)
    lon = float(GPS_VECTOR.lon)
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return
    _INPUT_RESOLVER.set_origin(lat, lon)
    motor_guidance.set_start_coordinates(lat, lon)
    _GUIDANCE.set_start(0.0, 0.0)
    _START_POINT_LOCKED = True
    _maybe_push_target()


def _snapshot_sensors():
    return SimpleNamespace(
        yaw=IMU.yaw,
        gyrz=IMU.gyrz,
        imu_health=IMU.imu_health,
        lat=GPS_VECTOR.lat,
        lon=GPS_VECTOR.lon,
        course=GPS_VECTOR.direction,
        speed=GPS_VECTOR.velocity,
        gps_health=1 if (GPS_HEALTH.pos_health and GPS_HEALTH.motion_health) else 0,
        fix_quality=1 if GPS_HEALTH.pos_health else 0,
        sats=4 if GPS_HEALTH.pos_health else 0,
        rmc_status="A" if GPS_HEALTH.pos_health else "V",
        alt=ALT,
        target=TARGET,
    )


def _check_fdir(snap) -> Optional[str]:
    if snap.target is None:
        return "target missing"
    if not snap.gps_health:
        return "GPS invalid"
    if not (-90.0 <= float(snap.lat) <= 90.0 and -180.0 <= float(snap.lon) <= 180.0):
        return "GPS out of range"
    if not snap.imu_health:
        return "IMU unhealthy"
    return None


def _apply_comm_tlm_fallback(g_out: GuidanceOutput) -> None:
    """Fill comm CSV geo fields when L1 left them unset (no target in NE yet, etc.)."""
    if _TARGET_LAT is not None and _TARGET_LON is not None:
        if not math.isfinite(g_out.target_lat):
            g_out.target_lat = float(_TARGET_LAT)
            g_out.target_lon = float(_TARGET_LON)
    if not GPS_HEALTH.pos_health:
        return
    lat, lon = float(GPS_VECTOR.lat), float(GPS_VECTOR.lon)
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return
    if not math.isfinite(g_out.current_heading_deg):
        if GPS_HEALTH.motion_health:
            g_out.current_heading_deg = float(GPS_VECTOR.direction)
        elif IMU.imu_health:
            g_out.current_heading_deg = float(IMU.yaw)
    if (
        _TARGET_LAT is not None
        and _TARGET_LON is not None
        and not math.isfinite(g_out.desired_heading_deg)
    ):
        tN, tE = _ll_to_ne(
            float(_TARGET_LAT), float(_TARGET_LON), lat, lon
        )
        if tN * tN + tE * tE > 1e-6:
            g_out.desired_heading_deg = math.degrees(math.atan2(tE, tN))


# ── Diagnostics ────────────────────────────────────────────────────────────────

def _send_diag(main_queue, cmd: BrakeCommand, g_out: GuidanceOutput,
               diag_state: str) -> None:
    if main_queue is None:
        return

    now = time.time()
    with _UPDATE_LOCK:
        if diag_state in ("IDLE", "LANDED"):
            inp = _INPUT_RESOLVER.resolve(now)
            g_out = _GUIDANCE.update(inp, now)
        _apply_comm_tlm_fallback(g_out)

    def _fmt(v) -> str:
        try:
            f = float(v)
            return "nan" if (f != f) else f"{f:.4f}"
        except (TypeError, ValueError):
            return "nan"

    def _fmt_ll(v: float) -> str:
        try:
            f = float(v)
            return "nan" if (f != f) else f"{f:.6f}"
        except (TypeError, ValueError):
            return "nan"

    def _fmt_hdg(v: float) -> str:
        try:
            f = float(v)
            return "nan" if (f != f) else f"{f:.2f}"
        except (TypeError, ValueError):
            return "nan"

    head = [
        str(cmd.left_pw),
        str(cmd.right_pw),
        _fmt_ll(g_out.start_lat),
        _fmt_ll(g_out.start_lon),
        _fmt_ll(g_out.target_lat),
        _fmt_ll(g_out.target_lon),
        _fmt_ll(g_out.carrot_lat),
        _fmt_ll(g_out.carrot_lon),
        _fmt_hdg(g_out.current_heading_deg),
        _fmt_hdg(g_out.desired_heading_deg),
        diag_state,
    ]
    tail = [
        _fmt(g_out.crossTrack),
        _fmt(g_out.alongTrack),
        _fmt(cmd.yaw_rate_cmd_deg_s),
        _fmt(cmd.yaw_rate_meas_deg_s),
        _fmt(cmd.yaw_rate_error_deg_s),
        _fmt(cmd.delta_ff_deg),
        _fmt(cmd.delta_pid_deg),
        _fmt(cmd.delta_arm_deg),
        _fmt(cmd.left_angle_deg),
        _fmt(cmd.right_angle_deg),
        str(int(cmd.saturated)),
        str(int(cmd.sensor_valid)),
        _fmt(cmd.guidance_command_age_s),
        cmd.fallback_mode,
        cmd.mode,
    ]
    payload = ",".join(head + tail)
    msgstructure.send_msg(
        main_queue,
        appargs.MotorAppArg.AppID,
        appargs.CommAppArg.AppID,
        appargs.MotorAppArg.MID_comm_motor_diag,
        payload,
    )


# ── Control loop ───────────────────────────────────────────────────────────────

def ctrl_paragldr(main_queue=None) -> None:
    """Parafoil control loop (~10 Hz, daemon thread).

    STATE < 3 or !MOTOR_ENABLED  → servo neutral
    STATE == 5                   → servo off
    STATE 3 / 4, MOTOR_ENABLED   → resolve input → guidance → controller → servo
    """
    _null_g = GuidanceOutput(timestamp=0.0)

    period = _motor_period_sec()
    while MOTORAPP_RUNSTATUS:
        try:
            now = time.time()

            if not MOTOR_ENABLED or STATE < 3:
                if PI is not None:
                    set_neutral(PI)
                _send_diag(main_queue, BrakeCommand(now), _null_g, "IDLE")
                time.sleep(period)
                continue

            if STATE == 5:
                if PI is not None:
                    set_motors_off(PI)
                _send_diag(
                    main_queue,
                    BrakeCommand(now, left_pw=0, right_pw=0),
                    _null_g, "LANDED",
                )
                time.sleep(period)
                continue

            with _UPDATE_LOCK:
                guidance_input = _INPUT_RESOLVER.resolve(now)

            g_out = _GUIDANCE.update(guidance_input, now)

            if g_out.active:
                guidance_cmd = GuidanceCommand(
                    yaw_rate_cmd_deg_s=math.degrees(g_out.courseRateCmd),
                    lat_acc_cmd_mps2=g_out.latAccDem,
                    ground_speed_mps=g_out.groundSpeed,
                    valid=True,
                    timestamp=g_out.timestamp,
                )
                yaw_rate_meas_deg_s = (
                    math.degrees(guidance_input.gyrz)
                    if guidance_input.gyrz is not None else None
                )
                cmd = _CONTROLLER.update(guidance_cmd, yaw_rate_meas_deg_s, now)
            else:
                _CONTROLLER.reset()
                guidance_cmd = GuidanceCommand(valid=False, timestamp=now)
                cmd = _CONTROLLER.update(guidance_cmd, None, now)

            if PI is not None:
                set_brake_command(PI, cmd)

            diag_state = (
                "DEGRADED" if (g_out.active and g_out.degraded)
                else "ACTIVE"  if g_out.active
                else g_out.reason or "DISABLED"
            )
            _send_diag(main_queue, cmd, g_out, diag_state)

        except Exception as exc:
            LOGGER.error("ctrl_paragldr exception: %s", exc, exc_info=True)
            try:
                if PI is not None:
                    set_neutral(PI)
            except Exception:
                pass

        time.sleep(period)


# ── Message dispatcher ─────────────────────────────────────────────────────────

def dispatch(msg: str) -> None:
    global MOTORAPP_RUNSTATUS
    unpacked = msgstructure.unpack_msg(msg)
    if unpacked is False:
        return
    mid = unpacked.msg_id
    if   mid == appargs.MainAppArg.MID_TerminateProcess:
        MOTORAPP_RUNSTATUS = False
    elif mid == appargs.GpsAppArg.MID_motor_gps:
        handle_gnss(unpacked.data)
    elif mid == appargs.ImuAppArg.MID_motor_imu:
        handle_imu(unpacked.data)
    elif mid == appargs.BarometerAppArg.MID_motor_alt:
        handle_barometer(unpacked.data)
    elif mid == appargs.FlightlogicAppArg.MID_motor_TargetCor:
        handle_target_coord(unpacked.data)
    elif mid == appargs.FlightlogicAppArg.MID_motor_state:
        handle_flight_state(unpacked.data)
    elif mid == appargs.FlightlogicAppArg.MID_motor_burnwire:
        handle_release()
    elif mid == appargs.FlightlogicAppArg.MID_motor_EggDrop:
        handle_egg_drop()
    elif mid == appargs.CommAppArg.MID_RouteCmd_MEC:
        handle_mec(unpacked.data)


# ── Lifecycle ──────────────────────────────────────────────────────────────────

def init() -> None:
    global PI
    Motor_Release.init_burnwire()
    Motor_Egg.init_solenoid()
    PI = init_control()
    LOGGER.info("MotorApp init | pigpio: %s", getattr(PI, "connected", "N/A"))


def motorapp_main(main_queue, main_pipe=None) -> None:
    if main_pipe is None:
        main_pipe  = main_queue
        main_queue = None
    init()

    ctrl_thread = threading.Thread(
        target=ctrl_paragldr, args=(main_queue,),
        daemon=True, name="MotorControlLoop",
    )
    ctrl_thread.start()
    LOGGER.info("MotorControlLoop started")

    poll_period = _motor_period_sec()
    try:
        while MOTORAPP_RUNSTATUS:
            try:
                if main_pipe.poll(poll_period):
                    dispatch(main_pipe.recv())
            except (KeyboardInterrupt, EOFError, OSError):
                break
    except KeyboardInterrupt:
        pass

    LOGGER.info("MotorApp exiting")
