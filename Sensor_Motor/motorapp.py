"""Motor app: data ingestion, FDIR, and parafoil control loop.

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
  any FDIR fault                    ->  servo neutral
  guidance.state == 'FDIR'          ->  servo neutral
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
from Sensor_Motor.motor_guidance import GpsFidelity, GpsVector

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

SENSOR = SimpleNamespace(
    yaw=0.0,
    gyrz=0.0,        # deg/s, per MID_motor_imu contract
    imu_health=1,
    lat=0.0,
    lon=0.0,
    speed=0.0,
    course=0.0,
    fix_quality=0,
    sats=0,
    rmc_status="V",
    gps_health=0,
    alt=0.0,
)

# target is None until a valid TC message is received.
# NEVER initialise to (0, 0) — that is a real coordinate (Gulf of Guinea).
TARGET: SimpleNamespace | None = None

# Stale-sensor timestamps (0.0 = never received)
_LAST_GPS_UPDATE: float = 0.0
_LAST_IMU_UPDATE: float = 0.0
_LAST_BARO_UPDATE: float = 0.0

GPS_STALE_TIMEOUT: float  = 10.0
IMU_STALE_TIMEOUT: float  =  5.0
BARO_STALE_TIMEOUT: float = 15.0

# FDIR rate-limited logging
_LAST_FDIR_REASON: str | None = None
_LAST_FDIR_LOG_TS: float = 0.0
FDIR_LOG_INTERVAL: float = 5.0   # min seconds between identical FDIR log lines

_START_POINT_LOCKED: bool = False
_LAST_GYRZ_FOR_FDIR: float | None = None

# deg/s. Drop-test was motor-off, so high yaw rate can be natural rotation.
# Use a spike/extreme gate instead of treating every >200 deg/s sample as a
# hard actuator-safety fault.
GYRZ_WARN_DPS: float = 200.0
GYRZ_SPIKE_DELTA_DPS: float = 350.0
GYRZ_EXTREME_SPIKE_DPS: float = 500.0


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _is_finite(x) -> bool:
    try:
        return x is not None and not math.isnan(float(x)) and not math.isinf(float(x))
    except (TypeError, ValueError):
        return False


def _gps_vector_from_sensor() -> GpsVector:
    return GpsVector(lat=SENSOR.lat, lon=SENSOR.lon, speed=SENSOR.speed, course=SENSOR.course)


def _gps_fidelity_from_sensor() -> GpsFidelity:
    return GpsFidelity(
        fix_quality=SENSOR.fix_quality,
        sats=SENSOR.sats,
        rmc_status=SENSOR.rmc_status,
        gps_health=SENSOR.gps_health,
    )


def _gps_valid_for_start_lock() -> bool:
    gps_vector = _gps_vector_from_sensor()
    gps_fidelity = _gps_fidelity_from_sensor()
    return motor_guidance.is_gps_valid(gps_vector, gps_fidelity)


def _maybe_lock_start_point(reason: str) -> bool:
    """Lock start coordinates only after GPS quality is valid."""
    global _START_POINT_LOCKED
    with _UPDATE_LOCK:
        if _START_POINT_LOCKED:
            return True
        if not _gps_valid_for_start_lock():
            return False
        cur_lat = SENSOR.lat
        cur_lon = SENSOR.lon
        _START_POINT_LOCKED = True
    motor_guidance.set_start_coordinates(cur_lat, cur_lon)
    logger.info("Start point locked (%s): %.6f, %.6f", reason, cur_lat, cur_lon)
    return True


def _check_yaw_rate_fault(gyrz_deg_s: float) -> str | None:
    """Classify IMU yaw-rate faults in deg/s.

    Motor-off replay shows large natural spins and occasional spikes. A single
    moderate high-yaw sample is logged by replay but should not necessarily
    force neutral; extreme values or abrupt high-rate jumps still fail FDIR.
    """
    global _LAST_GYRZ_FOR_FDIR
    g = float(gyrz_deg_s)
    prev = _LAST_GYRZ_FOR_FDIR
    _LAST_GYRZ_FOR_FDIR = g

    if abs(g) >= GYRZ_EXTREME_SPIKE_DPS:
        return f"FDIR-5 yaw-rate extreme spike ({g:.1f} deg/s)"
    if prev is not None and abs(g) >= GYRZ_WARN_DPS and abs(g - prev) >= GYRZ_SPIKE_DELTA_DPS:
        return f"FDIR-5 yaw-rate spike ({g:.1f} deg/s, delta={g - prev:.1f})"
    return None


# ---------------------------------------------------------------------------
# Message handlers
# ---------------------------------------------------------------------------

def handle_gps(data: str) -> None:
    """Parse: lat,lon,speed,course,fix_quality,sats,rmc_status,gps_health"""
    global _LAST_GPS_UPDATE
    fields = [x.strip() for x in data.split(",")]
    if len(fields) < 8:
        logger.warning("GPS parse: expected >=8 fields, got %d | raw=%r", len(fields), data)
        return
    try:
        lat      = float(fields[0])
        lon      = float(fields[1])
        speed    = float(fields[2])
        course   = float(fields[3])
        fix_qual = int(float(fields[4]))
        sats     = int(float(fields[5]))
        rmc_stat = fields[6]
        gps_hlth = int(float(fields[7]))
    except (ValueError, IndexError) as exc:
        logger.warning("GPS parse error: %s | raw=%r", exc, data)
        return
    with _UPDATE_LOCK:
        SENSOR.lat         = lat
        SENSOR.lon         = lon
        SENSOR.speed       = speed
        SENSOR.course      = course
        SENSOR.fix_quality = fix_qual
        SENSOR.sats        = sats
        SENSOR.rmc_status  = rmc_stat
        SENSOR.gps_health  = gps_hlth
    _LAST_GPS_UPDATE = time.time()
    if STATE >= 3 and not _START_POINT_LOCKED:
        _maybe_lock_start_point("gps-valid-after-state-3")


def handle_imu(data: str) -> None:
    """Parse: yaw_deg,gyrz_deg_s,imu_health.

    The IPC contract fixes IMU yaw rate in deg/s. The IMU app publishes
    converted deg/s values, so Motor stores the value as-is.
    """
    global _LAST_IMU_UPDATE
    fields = [x.strip() for x in data.split(",")]
    if len(fields) < 3:
        logger.warning("IMU parse: expected >=3 fields, got %d | raw=%r", len(fields), data)
        return
    try:
        yaw            = float(fields[0])
        gyrz_deg_s     = float(fields[1])
        imu_health     = int(float(fields[2]))
    except (ValueError, IndexError) as exc:
        logger.warning("IMU parse error: %s | raw=%r", exc, data)
        return
    with _UPDATE_LOCK:
        SENSOR.yaw        = yaw
        SENSOR.gyrz       = gyrz_deg_s
        SENSOR.imu_health = imu_health
    _LAST_IMU_UPDATE = time.time()


def handle_barometer(data: str) -> None:
    """Parse: altitude_m[,pressure,...]"""
    global _LAST_BARO_UPDATE
    fields = data.split(",")
    if not fields or not fields[0].strip():
        logger.warning("Barometer parse: empty data")
        return
    try:
        alt = float(fields[0].strip())
    except (ValueError, IndexError) as exc:
        logger.warning("Barometer parse error: %s | raw=%r", exc, data)
        return
    with _UPDATE_LOCK:
        SENSOR.alt = alt
    _LAST_BARO_UPDATE = time.time()


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
# Sensor snapshot & FDIR
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
            yaw        = SENSOR.yaw,
            gyrz       = SENSOR.gyrz,
            imu_health = SENSOR.imu_health,
            lat        = SENSOR.lat,
            lon        = SENSOR.lon,
            speed      = SENSOR.speed,
            course     = SENSOR.course,
            fix_quality= SENSOR.fix_quality,
            sats       = SENSOR.sats,
            rmc_status = SENSOR.rmc_status,
            gps_health = SENSOR.gps_health,
            alt        = SENSOR.alt,
            target     = tgt_copy,
        )


def _check_fdir(snap) -> str | None:
    """Return a descriptive FDIR reason string, or None when all checks pass.

    Check order (fail-fast, most critical first):
      FDIR-0   NaN / Inf in numeric sensors
      FDIR-1   IMU health flag
      FDIR-2a  GPS stale (no message within GPS_STALE_TIMEOUT)
      FDIR-2b  IMU stale
      FDIR-2c  Baro stale
      FDIR-3   GPS fix quality / sats / rmc_status invalid
      FDIR-4   GPS position jump (unrealistic speed)
      FDIR-5   Yaw-rate physically implausible (deg/s)
      FDIR-6   Negative altitude
      FDIR-7   Target coordinate not yet received
    """
    if not _is_finite(snap.yaw) or not _is_finite(snap.gyrz) or not _is_finite(snap.alt):
        return "FDIR-0 non-finite numeric sensor value"

    if snap.imu_health <= 0:
        return "FDIR-1 imu_health flag unhealthy"

    now = time.time()
    if _LAST_GPS_UPDATE == 0.0 or now - _LAST_GPS_UPDATE > GPS_STALE_TIMEOUT:
        return f"FDIR-2a gps stale ({now - _LAST_GPS_UPDATE:.1f}s since last msg)"
    if _LAST_IMU_UPDATE == 0.0 or now - _LAST_IMU_UPDATE > IMU_STALE_TIMEOUT:
        return f"FDIR-2b imu stale ({now - _LAST_IMU_UPDATE:.1f}s since last msg)"
    if _LAST_BARO_UPDATE == 0.0 or now - _LAST_BARO_UPDATE > BARO_STALE_TIMEOUT:
        return f"FDIR-2c baro stale ({now - _LAST_BARO_UPDATE:.1f}s since last msg)"

    gps_vector   = GpsVector(lat=snap.lat, lon=snap.lon, speed=snap.speed, course=snap.course)
    gps_fidelity = GpsFidelity(fix_quality=snap.fix_quality, sats=snap.sats, rmc_status=snap.rmc_status, gps_health=snap.gps_health)
    if not motor_guidance.is_gps_valid(gps_vector, gps_fidelity):
        return "FDIR-3 gps invalid (fix/sats/rmc_status)"
    if motor_guidance.is_gps_jump(snap.lat, snap.lon):
        return "FDIR-4 gps position jump"

    yaw_fault = _check_yaw_rate_fault(float(snap.gyrz))
    if yaw_fault is not None:
        return yaw_fault

    if snap.alt < 0.0:
        return f"FDIR-6 negative altitude ({snap.alt:.1f} m)"

    if snap.target is None:
        return "FDIR-7 target coordinate not received"

    return None


def _log_fdir(reason: str) -> None:
    """Rate-limited FDIR logger: same reason logged at most every FDIR_LOG_INTERVAL s."""
    global _LAST_FDIR_REASON, _LAST_FDIR_LOG_TS
    now = time.time()
    if reason != _LAST_FDIR_REASON or now - _LAST_FDIR_LOG_TS >= FDIR_LOG_INTERVAL:
        logger.warning("FDIR -> neutral | %s", reason)
        _LAST_FDIR_REASON = reason
        _LAST_FDIR_LOG_TS = now


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

def ctrl_paragldr() -> None:
    """Parafoil control loop (~10 Hz, daemon thread).

    Safe-output contract:
      Any exception       -> set_neutral, continue (loop never dies)
      FDIR fault          -> set_neutral
      guidance FDIR       -> set_neutral
      STATE < 3 or !MEC  -> set_neutral (idle)
      STATE == 5          -> set_motors_off
      nominal             -> motor_control.control(commanded_yaw_rate)
    """
    global _LAST_FDIR_REASON

    while MOTORAPP_RUNSTATUS:
        try:
            if STATE < 3 or not MOTOR_ENABLED:
                if PI is not None:
                    motor_control.set_neutral(PI)
                time.sleep(0.1)
                continue

            if STATE == 5:
                motor_control.set_motors_off(PI)
                time.sleep(0.1)
                continue

            snap = _snapshot_sensors()
            fdir = _check_fdir(snap)
            if fdir is not None:
                _log_fdir(fdir)
                motor_control.set_neutral(PI)
                time.sleep(0.1)
                continue

            if _LAST_FDIR_REASON is not None:
                logger.info("FDIR cleared, resuming guidance")
                _LAST_FDIR_REASON = None

            imu_data     = SimpleNamespace(yaw=snap.yaw, gyrz=snap.gyrz)
            gps_vector   = GpsVector(lat=snap.lat, lon=snap.lon, speed=snap.speed, course=snap.course)
            gps_fidelity = GpsFidelity(fix_quality=snap.fix_quality, sats=snap.sats, rmc_status=snap.rmc_status, gps_health=snap.gps_health)

            result = motor_guidance.guidance(imu_data, gps_vector, gps_fidelity, snap.target, snap.alt)

            if result.state == "FDIR":
                _log_fdir("guidance internal FDIR")
                motor_control.set_neutral(PI)
            else:
                motor_control.control(PI, float(result.commanded_yaw_rate))

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
    global PI, _START_POINT_LOCKED, _LAST_GYRZ_FOR_FDIR
    _START_POINT_LOCKED = False
    _LAST_GYRZ_FOR_FDIR = None
    motor_guidance.init_guidance()
    Motor_Release.init_burnwire()
    Motor_Egg.init_solenoid()
    PI = motor_control.init_control()
    logger.info(
        "MotorApp init complete | pigpio connected: %s",
        getattr(PI, "connected", "unknown"),
    )


def motorapp_main(main_pipe) -> None:
    init()

    ctrl_thread = threading.Thread(
        target=ctrl_paragldr, daemon=True, name="MotorControlLoop"
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
