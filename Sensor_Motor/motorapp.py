"""Motor app: data ingestion and parafoil control loop.

Message flow (inbound via main_pipe):
  GpsApp       -> MID_motor_gps       -> handle_gps()
  ImuApp       -> MID_motor_imu       -> handle_imu()
  BarometerApp -> MID_motor_alt       -> handle_barometer()
  FlightLogic  -> MID_motor_state     -> handle_flight_state()
  FlightLogic  -> MID_motor_TargetCor -> handle_target_coord()
  FlightLogic  -> MID_motor_burnwire  -> handle_release()
  FlightLogic  -> MID_motor_EggDrop   -> handle_egg_drop()
  CommApp      -> MID_RouteCmd_MEC    -> handle_mec()

Control thread (ctrl_paragldr, ~10 Hz):
  STATE < 3 or MOTOR_ENABLED=False  ->  servo neutral  (safe idle)
  STATE == 5  (LANDED)              ->  servo off       (no PWM)
  nominal                           ->  guidance -> motor_control
"""

from __future__ import annotations

import logging
import math
import threading
import time
from types import SimpleNamespace

from lib import appargs, msgstructure
from Sensor_Motor import Motor_Egg, Motor_Release, motor_control, motor_guidance

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Runtime state (module-level so tests can inspect / inject directly)
# ---------------------------------------------------------------------------
MOTORAPP_RUNSTATUS: bool = True
MOTOR_ENABLED: bool = True
STATE: int = 0
PI = None  # ControlHandle; assigned by init()

_PREV_STATE: int = -1
_UPDATE_LOCK = threading.Lock()

IMU = SimpleNamespace(
    yaw=0.0,
    gyrz=0.0,       # deg/s, per MID_motor_imu contract
    imu_health=1,
)
ALT: float = 0.0

GPS_VECTOR = SimpleNamespace(
    lat=0.0,
    lon=0.0,
    direction=0.0,  # deg, GPS ground-track direction; not IMU yaw.
    velocity=0.0,
)
GPS_HEALTH = SimpleNamespace(
    pos_health=0,
    motion_health=0,
)

# target is None until a valid TC message is received.
# NEVER initialise to (0, 0) — that is a real coordinate (Gulf of Guinea).
TARGET: SimpleNamespace | None = None

_START_POINT_LOCKED: bool = False


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _is_finite(x) -> bool:
    try:
        return x is not None and not math.isnan(float(x)) and not math.isinf(float(x))
    except (TypeError, ValueError):
        return False


def _gps_valid_for_start_lock() -> bool:
    return (
        int(GPS_HEALTH.pos_health) > 0
        and _is_finite(GPS_VECTOR.lat)
        and _is_finite(GPS_VECTOR.lon)
        and GPS_VECTOR.lat != 0.0
        and GPS_VECTOR.lon != 0.0
    )


def _maybe_lock_start_point(reason: str) -> bool:
    """Lock start coordinates only after GPS quality is valid."""
    global _START_POINT_LOCKED
    with _UPDATE_LOCK:
        if _START_POINT_LOCKED:
            return True
        if not _gps_valid_for_start_lock():
            return False
        cur_lat = GPS_VECTOR.lat
        cur_lon = GPS_VECTOR.lon
        _START_POINT_LOCKED = True
    motor_guidance.set_start_coordinates(cur_lat, cur_lon)
    logger.info("Start point locked (%s): %.6f, %.6f", reason, cur_lat, cur_lon)
    return True


# ---------------------------------------------------------------------------
# Message handlers
# ---------------------------------------------------------------------------

def handle_gps(data: str) -> None:
    """Parse GPS payload.

    Contract: lat,lon,direction,velocity,pos_health,motion_health.
    """
    fields = [x.strip() for x in data.split(",")]
    if len(fields) != 6:
        logger.warning("GPS parse: expected 6 fields, got %d | raw=%r", len(fields), data)
        return
    try:
        lat = float(fields[0])
        lon = float(fields[1])
        direction = float(fields[2])
        velocity = float(fields[3])
        pos_health = int(float(fields[4]))
        motion_health = int(float(fields[5]))
    except (ValueError, IndexError) as exc:
        logger.warning("GPS parse error: %s | raw=%r", exc, data)
        return

    with _UPDATE_LOCK:
        GPS_VECTOR.lat           = lat
        GPS_VECTOR.lon           = lon
        GPS_VECTOR.direction     = direction % 360.0 if _is_finite(direction) else direction
        GPS_VECTOR.velocity      = velocity
        GPS_HEALTH.pos_health    = pos_health
        GPS_HEALTH.motion_health = motion_health

    if STATE >= 3 and not _START_POINT_LOCKED:
        _maybe_lock_start_point("gps-valid-after-state-3")


def handle_imu(data: str) -> None:
    """Parse: yaw_deg,gyrz_deg_s,imu_health.

    The IPC contract fixes IMU yaw rate in deg/s. The IMU app publishes
    converted deg/s values, so Motor stores the value as-is.
    """
    fields = [x.strip() for x in data.split(",")]
    if len(fields) < 3:
        logger.warning("IMU parse: expected >=3 fields, got %d | raw=%r", len(fields), data)
        return
    try:
        yaw        = float(fields[0])
        gyrz_deg_s = float(fields[1])
        imu_health = int(float(fields[2]))
    except (ValueError, IndexError) as exc:
        logger.warning("IMU parse error: %s | raw=%r", exc, data)
        return
    with _UPDATE_LOCK:
        IMU.yaw        = yaw
        IMU.gyrz       = gyrz_deg_s
        IMU.imu_health = imu_health


def handle_barometer(data: str) -> None:
    """Parse: altitude_m[,pressure,...]"""
    fields = data.split(",")
    if not fields or not fields[0].strip():
        logger.warning("Barometer parse: empty data")
        return
    try:
        alt = float(fields[0].strip())
    except (ValueError, IndexError) as exc:
        logger.warning("Barometer parse error: %s | raw=%r", exc, data)
        return
    global ALT
    with _UPDATE_LOCK:
        ALT = alt


def handle_target_coord(data: str) -> None:
    """Parse: lat,lon  — stores to module-level target and passes to guidance."""
    global TARGET
    fields = [x.strip() for x in data.split(",")]
    if len(fields) != 2:
        logger.warning("Target parse: expected 2 fields, got %d | raw=%r", len(fields), data)
        return
    try:
        lat = float(fields[0])
        lon = float(fields[1])
    except (ValueError, IndexError) as exc:
        logger.warning("Target parse error: %s | raw=%r", exc, data)
        return
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        logger.warning("Target coord out of range: lat=%.6f lon=%.6f", lat, lon)
        return
    with _UPDATE_LOCK:
        TARGET = SimpleNamespace(lat=lat, lon=lon)
    motor_guidance.set_target_coord(lat, lon)
    logger.info("Target updated: %.6f, %.6f", lat, lon)


def handle_flight_state(data: str) -> None:
    """Update flight state and handle state-entry side effects."""
    global STATE, _PREV_STATE, _START_POINT_LOCKED
    try:
        new_state = int(data.split(",")[0])
    except (ValueError, IndexError) as exc:
        logger.warning("Flight state parse error: %s | raw=%r", exc, data)
        return

    if new_state == STATE:
        return

    logger.info("State transition: %d -> %d", STATE, new_state)
    _PREV_STATE = STATE
    STATE = new_state

    if new_state < 3:
        _START_POINT_LOCKED = False

    # On entering RELEASE (state 3), lock only if GPS is already valid.
    if new_state == 3:
        if not _maybe_lock_start_point("state-3"):
            logger.warning("State-3 entered before valid GPS; start lock deferred")


def handle_release() -> None:
    """Fire burnwire in a daemon thread — does not block the message loop."""
    logger.info("Burnwire activate commanded")
    threading.Thread(
        target=Motor_Release.activate_burnwire,
        daemon=True,
        name="BurnwireActivate",
    ).start()


def handle_egg_drop() -> None:
    """Fire egg-drop solenoid in a daemon thread — does not block the message loop."""
    logger.info("Solenoid (egg-drop) activate commanded")
    threading.Thread(
        target=Motor_Egg.activate_solenoid,
        daemon=True,
        name="SolenoidActivate",
    ).start()


def handle_mec(data: str) -> None:
    """Handle MEC ON / OFF command."""
    global MOTOR_ENABLED
    cmd = data.strip().upper()
    if cmd == "ON":
        MOTOR_ENABLED = True
        logger.info("Motor enabled (MEC ON)")
    elif cmd == "OFF":
        MOTOR_ENABLED = False
        if PI is not None:
            motor_control.set_neutral(PI)   # immediately safe when disabled
        logger.info("Motor disabled (MEC OFF) -> neutral")
    else:
        logger.warning("Unknown MEC command: %r", data)


# ---------------------------------------------------------------------------
# Sensor snapshot
# ---------------------------------------------------------------------------

def _snapshot_sensors() -> SimpleNamespace:
    """Return a thread-safe copy of all current sensor state."""
    with _UPDATE_LOCK:
        tgt_copy = (
            SimpleNamespace(lat=TARGET.lat, lon=TARGET.lon)
            if TARGET is not None
            else None
        )
        return SimpleNamespace(
            yaw           = IMU.yaw,
            gyrz          = IMU.gyrz,
            imu_health    = IMU.imu_health,
            lat           = GPS_VECTOR.lat,
            lon           = GPS_VECTOR.lon,
            direction     = GPS_VECTOR.direction,
            velocity      = GPS_VECTOR.velocity,
            pos_health    = GPS_HEALTH.pos_health,
            motion_health = GPS_HEALTH.motion_health,
            alt           = ALT,
            target        = tgt_copy,
        )


# ---------------------------------------------------------------------------
# Message dispatcher
# ---------------------------------------------------------------------------

def dispatch(msg: str) -> None:
    global MOTORAPP_RUNSTATUS
    unpacked = msgstructure.unpack_msg(msg)
    if unpacked is False:
        return

    mid = unpacked.msg_id
    if   mid == appargs.MainAppArg.MID_TerminateProcess:
        MOTORAPP_RUNSTATUS = False
    elif mid == appargs.GpsAppArg.MID_motor_gps:
        handle_gps(unpacked.data)
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


# ---------------------------------------------------------------------------
# Control loop
# ---------------------------------------------------------------------------

def _fmt_num(value: float) -> str:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return "nan"
    if math.isnan(v) or math.isinf(v):
        return "nan"
    return f"{v:.8f}"


def _send_motor_diag(main_queue, left_pulse: int, right_pulse: int, result) -> None:
    if main_queue is None:
        return
    payload = ",".join(
        [
            str(int(left_pulse)),
            str(int(right_pulse)),
            _fmt_num(getattr(result, "start_lat", float("nan"))),
            _fmt_num(getattr(result, "start_lon", float("nan"))),
            _fmt_num(getattr(result, "target_lat", float("nan"))),
            _fmt_num(getattr(result, "target_lon", float("nan"))),
            _fmt_num(getattr(result, "carrot_lat", float("nan"))),
            _fmt_num(getattr(result, "carrot_lon", float("nan"))),
            _fmt_num(getattr(result, "current_heading", float("nan"))),
            _fmt_num(getattr(result, "desired_heading", float("nan"))),
            str(getattr(result, "state", "")),
        ]
    )
    msgstructure.send_msg(
        main_queue,
        appargs.MotorAppArg.AppID,
        appargs.CommAppArg.AppID,
        appargs.MotorAppArg.MID_comm_motor_diag,
        payload,
    )


def ctrl_paragldr(main_queue=None) -> None:
    """Parafoil control loop (~10 Hz, daemon thread).

    Safe-output contract:
      Any exception      -> set_neutral, continue (loop never dies)
      STATE < 3 or !MEC  -> set_neutral (idle)
      STATE == 5          -> set_motors_off
      nominal             -> motor_control.control(commanded_yaw_rate)
    """
    while MOTORAPP_RUNSTATUS:
        try:
            if STATE < 3 or not MOTOR_ENABLED:
                if PI is not None:
                    motor_control.set_neutral(PI)
                _send_motor_diag(
                    main_queue,
                    motor_control.LEFT_NEUTRAL,
                    motor_control.RIGHT_NEUTRAL,
                    SimpleNamespace(state="IDLE", current_heading=IMU.yaw),
                )
                time.sleep(0.1)
                continue

            if STATE == 5:
                motor_control.set_motors_off(PI)
                _send_motor_diag(
                    main_queue,
                    0,
                    0,
                    SimpleNamespace(state="LANDED", current_heading=IMU.yaw),
                )
                time.sleep(0.1)
                continue

            snap = _snapshot_sensors()

            imu_data = SimpleNamespace(yaw=snap.yaw, gyrz=snap.gyrz)
            guidance_position = SimpleNamespace(
                lat=snap.lat,
                lon=snap.lon,
                direction=snap.direction,
                velocity=snap.velocity,
            )
            guidance_health = SimpleNamespace(
                pos_health=snap.pos_health,
                motion_health=snap.motion_health,
            )

            result = motor_guidance.guidance(imu_data, guidance_position, guidance_health, snap.target, snap.alt)
            motor_out = motor_control.control(PI, float(result.commanded_yaw_rate))
            _send_motor_diag(main_queue, motor_out.left_pulse, motor_out.right_pulse, result)

        except Exception as exc:
            logger.error("ctrl_paragldr unhandled exception: %s", exc, exc_info=True)
            try:
                motor_control.set_neutral(PI)
            except Exception:
                pass

        time.sleep(0.1)


# ---------------------------------------------------------------------------
# App lifecycle
# ---------------------------------------------------------------------------

def init() -> None:
    global PI, _START_POINT_LOCKED
    _START_POINT_LOCKED = False
    motor_guidance.init_guidance()
    Motor_Release.init_burnwire()
    Motor_Egg.init_solenoid()
    PI = motor_control.init_control()
    logger.info(
        "MotorApp init complete | pigpio connected: %s",
        getattr(PI, "connected", "unknown"),
    )


def motorapp_main(main_queue, main_pipe=None) -> None:
    # Backward compatibility: allow motorapp_main(main_pipe) in tests.
    if main_pipe is None:
        main_pipe = main_queue
        main_queue = None
    init()

    ctrl_thread = threading.Thread(
        target=ctrl_paragldr, args=(main_queue,), daemon=True, name="MotorControlLoop"
    )
    ctrl_thread.start()
    logger.info("MotorControlLoop started")

    try:
        while MOTORAPP_RUNSTATUS:
            try:
                has_msg = main_pipe.poll(0.1)
            except (KeyboardInterrupt, EOFError, OSError):
                break
            if has_msg:
                msg = main_pipe.recv()
                dispatch(msg)
    except KeyboardInterrupt:
        pass

    logger.info("MotorApp exiting")
