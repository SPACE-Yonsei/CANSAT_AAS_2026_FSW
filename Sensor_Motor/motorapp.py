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

from lib import appargs, msgstructure
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

logger = logging.getLogger(__name__)

# ── Runtime state ──────────────────────────────────────────────────────────────
MOTORAPP_RUNSTATUS: bool = True
MOTOR_ENABLED: bool      = True
STATE: int               = 0
PI                       = None

_UPDATE_LOCK = threading.Lock()

_input_resolver  = GuidanceInputResolver()
_guidance   = L1Guidance(L1Config())
_controller = ParafoilBrakeController(ControlConfig())

_target_lat: Optional[float] = None
_target_lon: Optional[float] = None

# Legacy observable state kept for tests/replay scripts during migration.
IMU = SimpleNamespace(yaw=0.0, gyrz=0.0, imu_health=0)
GPS_VECTOR = SimpleNamespace(lat=0.0, lon=0.0, direction=0.0, velocity=0.0)
GPS_HEALTH = SimpleNamespace(pos_health=0, motion_health=0)
TARGET = None
ALT = 0.0
_PREV_STATE = -1
_START_POINT_LOCKED = False


# ── Message handlers ───────────────────────────────────────────────────────────

def handle_gnss(data: str) -> None:
    """lat,lon,course_deg,groundSpeed_ms,posHealth,motionHealth"""
    fields = data.split(",")
    if len(fields) != 6:
        logger.warning("GNSS parse: expected 6 fields | raw=%r", data)
        return
    try:
        lat          = float(fields[0])
        lon          = float(fields[1])
        course_deg   = float(fields[2])
        groundSpeed  = float(fields[3])
        posHealth    = bool(int(float(fields[4])))
        motionHealth = bool(int(float(fields[5])))
    except (ValueError, IndexError) as exc:
        logger.warning("GNSS parse error: %s | raw=%r", exc, data)
        return

    with _UPDATE_LOCK:
        GPS_VECTOR.lat = lat
        GPS_VECTOR.lon = lon
        GPS_VECTOR.direction = course_deg
        GPS_VECTOR.velocity = groundSpeed
        GPS_HEALTH.pos_health = int(posHealth)
        GPS_HEALTH.motion_health = int(motionHealth)
        _input_resolver.update_gnss(
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
            logger.warning("GPS parse error: %s | raw=%r", exc, data)
            return
        ok = fix_quality >= 1 and sats >= 4 and rmc_status == "A" and gps_health >= 1
        handle_gnss(f"{lat},{lon},{course},{speed},{int(ok)},{int(ok)}")
        return
    logger.warning("GPS parse: expected 6 or 8 fields | raw=%r", data)


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
                _input_resolver.update_imu(
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
                _input_resolver.update_imu(yaw=yaw_deg, gz=gyrz_degs, ts=ts)
        else:
            logger.warning("IMU parse: too few fields | raw=%r", data)
    except (ValueError, IndexError) as exc:
        logger.warning("IMU parse error: %s | raw=%r", exc, data)


def handle_barometer(data: str) -> None:
    """altitude_m[,...]"""
    global ALT
    try:
        alt = float(data.split(",")[0].strip())
    except (ValueError, IndexError) as exc:
        logger.warning("Baro parse error: %s | raw=%r", exc, data)
        return
    with _UPDATE_LOCK:
        ALT = alt
        _input_resolver.update_baro(alt, time.time())


def handle_target_coord(data: str) -> None:
    """lat,lon"""
    global _target_lat, _target_lon, TARGET
    fields = data.split(",")
    if len(fields) != 2:
        logger.warning("Target parse: expected 2 fields | raw=%r", data)
        return
    try:
        lat = float(fields[0])
        lon = float(fields[1])
    except (ValueError, IndexError) as exc:
        logger.warning("Target parse error: %s | raw=%r", exc, data)
        return
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        logger.warning("Target coord out of range: %.6f, %.6f", lat, lon)
        return
    with _UPDATE_LOCK:
        _target_lat = lat
        _target_lon = lon
        TARGET = SimpleNamespace(lat=lat, lon=lon)
        _maybe_push_target()
    logger.info("Target updated: %.6f, %.6f", lat, lon)


def handle_flight_state(data: str) -> None:
    global STATE, _PREV_STATE, _START_POINT_LOCKED
    try:
        new_state = int(data.split(",")[0])
    except (ValueError, IndexError) as exc:
        logger.warning("State parse error: %s | raw=%r", exc, data)
        return
    if new_state == STATE:
        return
    logger.info("State %d → %d", STATE, new_state)
    with _UPDATE_LOCK:
        _PREV_STATE = STATE
        STATE = new_state
        if new_state < 3:
            _guidance.reset()
            _controller.reset()
            _input_resolver.reset_origin()
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
        logger.info("MOTOR_ENABLED = True")
    elif cmd == "OFF":
        MOTOR_ENABLED = False
        with _UPDATE_LOCK:
            if PI is not None:
                set_neutral(PI)
        logger.info("MOTOR_ENABLED = False → neutral")
    else:
        logger.warning("Unknown MEC command: %r", data)


# ── Helpers ────────────────────────────────────────────────────────────────────

def _maybe_push_target() -> None:
    """Convert target lat/lon to N/E and push to guidance. Call under _UPDATE_LOCK."""
    if _target_lat is None or _target_lon is None:
        return
    if _input_resolver.origin_lat is None:
        return
    tgt_N, tgt_E = _ll_to_ne(
        _target_lat, _target_lon,
        _input_resolver.origin_lat, _input_resolver.origin_lon,
    )
    _guidance.set_target(tgt_N, tgt_E)


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
    _input_resolver.set_origin(lat, lon)
    motor_guidance.set_start_coordinates(lat, lon)
    _guidance.set_start(0.0, 0.0)
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


# ── Diagnostics ────────────────────────────────────────────────────────────────

def _send_diag(main_queue, cmd: BrakeCommand, g_out: GuidanceOutput,
               diag_state: str) -> None:
    if main_queue is None:
        return

    def _fmt(v) -> str:
        try:
            f = float(v)
            return "nan" if (f != f) else f"{f:.4f}"
        except (TypeError, ValueError):
            return "nan"

    payload = ",".join([
        str(cmd.left_pw),
        str(cmd.right_pw),
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
        diag_state,
    ])
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

    while MOTORAPP_RUNSTATUS:
        try:
            now = time.time()

            if not MOTOR_ENABLED or STATE < 3:
                if PI is not None:
                    set_neutral(PI)
                _send_diag(main_queue, BrakeCommand(now), _null_g, "IDLE")
                time.sleep(0.1)
                continue

            if STATE == 5:
                if PI is not None:
                    set_motors_off(PI)
                _send_diag(
                    main_queue,
                    BrakeCommand(now, left_pw=0, right_pw=0),
                    _null_g, "LANDED",
                )
                time.sleep(0.1)
                continue

            with _UPDATE_LOCK:
                guidance_input = _input_resolver.resolve(now)

            g_out = _guidance.update(guidance_input, now)

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
                cmd = _controller.update(guidance_cmd, yaw_rate_meas_deg_s, now)
            else:
                _controller.reset()
                guidance_cmd = GuidanceCommand(valid=False, timestamp=now)
                cmd = _controller.update(guidance_cmd, None, now)

            if PI is not None:
                set_brake_command(PI, cmd)

            diag_state = (
                "DEGRADED" if (g_out.active and g_out.degraded)
                else "ACTIVE"  if g_out.active
                else g_out.reason or "DISABLED"
            )
            _send_diag(main_queue, cmd, g_out, diag_state)

        except Exception as exc:
            logger.error("ctrl_paragldr exception: %s", exc, exc_info=True)
            try:
                if PI is not None:
                    set_neutral(PI)
            except Exception:
                pass

        time.sleep(0.1)


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
    logger.info("MotorApp init | pigpio: %s", getattr(PI, "connected", "N/A"))


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
    logger.info("MotorControlLoop started")

    try:
        while MOTORAPP_RUNSTATUS:
            try:
                if main_pipe.poll(0.1):
                    dispatch(main_pipe.recv())
            except (KeyboardInterrupt, EOFError, OSError):
                break
    except KeyboardInterrupt:
        pass

    logger.info("MotorApp exiting")
