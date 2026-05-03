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
  state < 3 or motor_enabled=False  ->  servo neutral  (safe idle)
  state == 5  (LANDED)              ->  servo off       (no PWM)
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
motor_enabled: bool = True
state: int = 0
_prev_state: int = -1
pi = None  # ControlHandle; assigned by init()

update_lock = threading.Lock()

sensor = SimpleNamespace(
    yaw=0.0,
    gyrz=0.0,        # deg/s  — see IMU_GYRZ_IS_DEG_S note in handle_imu()
    imu_health=1,
    lat=0.0,
    lon=0.0,
    speed=0.0,
    course=0.0,
    fix_quality=0,
    sats=0,
    rmc_status="V",
    gps_health=0,
    baro_m=0.0,
)

# target is None until a valid TC message is received.
# NEVER initialise to (0, 0) — that is a real coordinate (Gulf of Guinea).
target: SimpleNamespace | None = None

# Stale-sensor timestamps (0.0 = never received)
last_gps_update: float = 0.0
last_imu_update: float = 0.0
last_baro_update: float = 0.0

GPS_STALE_TIMEOUT: float  = 10.0
IMU_STALE_TIMEOUT: float  =  5.0
BARO_STALE_TIMEOUT: float = 15.0

# FDIR rate-limited logging
_last_fdir_reason: str | None = None
_last_fdir_log_ts: float = 0.0
FDIR_LOG_INTERVAL: float = 5.0   # min seconds between identical FDIR log lines


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _send(main_queue, msg_id: int, data: str) -> None:
    msgstructure.send_msg(
        main_queue,
        appargs.MotorAppArg.AppID,
        appargs.CommAppArg.AppID,
        msg_id,
        data,
    )


def _is_finite(x) -> bool:
    try:
        return x is not None and not math.isnan(float(x)) and not math.isinf(float(x))
    except (TypeError, ValueError):
        return False


# ---------------------------------------------------------------------------
# Message handlers
# ---------------------------------------------------------------------------

def handle_gps(data: str) -> None:
    """Parse: lat,lon,speed,course,fix_quality,sats,rmc_status,gps_health"""
    global last_gps_update
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
    with update_lock:
        sensor.lat         = lat
        sensor.lon         = lon
        sensor.speed       = speed
        sensor.course      = course
        sensor.fix_quality = fix_qual
        sensor.sats        = sats
        sensor.rmc_status  = rmc_stat
        sensor.gps_health  = gps_hlth
    last_gps_update = time.time()


_RAD_TO_DEG = 180.0 / 3.141592653589793


def handle_imu(data: str) -> None:
    """Parse: yaw_deg, gyrz_rad_s, imu_health

    BNO085 GYROSCOPE reports angular velocity in rad/s (confirmed from data schema).
    gyrz is converted to deg/s here so that guidance and FDIR thresholds
    can work in the same unit as yaw (deg) and commanded_yaw_rate (deg/s).
    """
    global last_imu_update
    fields = [x.strip() for x in data.split(",")]
    if len(fields) < 3:
        logger.warning("IMU parse: expected >=3 fields, got %d | raw=%r", len(fields), data)
        return
    try:
        yaw            = float(fields[0])
        gyrz_rad_s     = float(fields[1])          # rad/s from BNO085
        imu_health     = int(float(fields[2]))
    except (ValueError, IndexError) as exc:
        logger.warning("IMU parse error: %s | raw=%r", exc, data)
        return
    with update_lock:
        sensor.yaw        = yaw
        sensor.gyrz       = gyrz_rad_s * _RAD_TO_DEG   # stored as deg/s
        sensor.imu_health = imu_health
    last_imu_update = time.time()


def handle_barometer(data: str) -> None:
    """Parse: altitude_m[,pressure,...]"""
    global last_baro_update
    fields = data.split(",")
    if not fields or not fields[0].strip():
        logger.warning("Barometer parse: empty data")
        return
    try:
        alt = float(fields[0].strip())
    except (ValueError, IndexError) as exc:
        logger.warning("Barometer parse error: %s | raw=%r", exc, data)
        return
    with update_lock:
        sensor.baro_m = alt
    last_baro_update = time.time()


def handle_target_coord(data: str) -> None:
    """Parse: lat,lon  — stores to module-level target and passes to guidance."""
    global target
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
    with update_lock:
        target = SimpleNamespace(lat=lat, lon=lon)
    motor_guidance.set_target_coord(lat, lon)
    logger.info("Target updated: %.6f, %.6f", lat, lon)


def handle_flight_state(data: str) -> None:
    """Update flight state and handle state-entry side effects."""
    global state, _prev_state
    try:
        new_state = int(data.split(",")[0])
    except (ValueError, IndexError) as exc:
        logger.warning("Flight state parse error: %s | raw=%r", exc, data)
        return

    if new_state == state:
        return

    logger.info("State transition: %d -> %d", state, new_state)
    _prev_state = state
    state = new_state

    # On entering RELEASE (state 3), lock the start-point for guidance
    if new_state == 3:
        with update_lock:
            cur_lat = sensor.lat
            cur_lon = sensor.lon
        motor_guidance.set_start_coordinates(cur_lat, cur_lon)
        logger.info("Start point locked at state-3: %.6f, %.6f", cur_lat, cur_lon)


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
    global motor_enabled
    cmd = data.strip().upper()
    if cmd == "ON":
        motor_enabled = True
        logger.info("Motor enabled (MEC ON)")
    elif cmd == "OFF":
        motor_enabled = False
        if pi is not None:
            motor_control.set_neutral(pi)   # immediately safe when disabled
        logger.info("Motor disabled (MEC OFF) -> neutral")
    else:
        logger.warning("Unknown MEC command: %r", data)


# ---------------------------------------------------------------------------
# Sensor snapshot & FDIR
# ---------------------------------------------------------------------------

def _snapshot_sensors() -> SimpleNamespace:
    """Return a thread-safe copy of all current sensor state."""
    with update_lock:
        tgt_copy = (
            SimpleNamespace(lat=target.lat, lon=target.lon)
            if target is not None
            else None
        )
        return SimpleNamespace(
            yaw        = sensor.yaw,
            gyrz       = sensor.gyrz,
            imu_health = sensor.imu_health,
            lat        = sensor.lat,
            lon        = sensor.lon,
            speed      = sensor.speed,
            course     = sensor.course,
            fix_quality= sensor.fix_quality,
            sats       = sensor.sats,
            rmc_status = sensor.rmc_status,
            gps_health = sensor.gps_health,
            baro_m     = sensor.baro_m,
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
    if not _is_finite(snap.yaw) or not _is_finite(snap.gyrz) or not _is_finite(snap.baro_m):
        return "FDIR-0 non-finite numeric sensor value"

    if snap.imu_health <= 0:
        return "FDIR-1 imu_health flag unhealthy"

    now = time.time()
    if last_gps_update == 0.0 or now - last_gps_update > GPS_STALE_TIMEOUT:
        return f"FDIR-2a gps stale ({now - last_gps_update:.1f}s since last msg)"
    if last_imu_update == 0.0 or now - last_imu_update > IMU_STALE_TIMEOUT:
        return f"FDIR-2b imu stale ({now - last_imu_update:.1f}s since last msg)"
    if last_baro_update == 0.0 or now - last_baro_update > BARO_STALE_TIMEOUT:
        return f"FDIR-2c baro stale ({now - last_baro_update:.1f}s since last msg)"

    gps_vector   = GpsVector(lat=snap.lat, lon=snap.lon, speed=snap.speed, course=snap.course)
    gps_fidelity = GpsFidelity(fix_quality=snap.fix_quality, sats=snap.sats, rmc_status=snap.rmc_status, gps_health=snap.gps_health)
    if not motor_guidance.is_gps_valid(gps_vector, gps_fidelity):
        return "FDIR-3 gps invalid (fix/sats/rmc_status)"
    if motor_guidance.is_gps_jump(snap.lat, snap.lon):
        return "FDIR-4 gps position jump"

    # gyrz is deg/s; >200 deg/s is physically implausible for a parafoil cansat
    if abs(snap.gyrz) > 200.0:
        return f"FDIR-5 yaw-rate implausible ({snap.gyrz:.1f} deg/s)"

    if snap.baro_m < 0.0:
        return f"FDIR-6 negative altitude ({snap.baro_m:.1f} m)"

    if snap.target is None:
        return "FDIR-7 target coordinate not received"

    return None


def _log_fdir(reason: str) -> None:
    """Rate-limited FDIR logger: same reason logged at most every FDIR_LOG_INTERVAL s."""
    global _last_fdir_reason, _last_fdir_log_ts
    now = time.time()
    if reason != _last_fdir_reason or now - _last_fdir_log_ts >= FDIR_LOG_INTERVAL:
        logger.warning("FDIR -> neutral | %s", reason)
        _last_fdir_reason = reason
        _last_fdir_log_ts = now


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
      state < 3 or !MEC  -> set_neutral (idle)
      state == 5          -> set_motors_off
      nominal             -> motor_control.control(commanded_yaw_rate)
    """
    global _last_fdir_reason

    while MOTORAPP_RUNSTATUS:
        try:
            if state < 3 or not motor_enabled:
                time.sleep(0.1)
                continue

            if state == 5:
                motor_control.set_motors_off(pi)
                time.sleep(0.1)
                continue

            snap = _snapshot_sensors()
            fdir = _check_fdir(snap)
            if fdir is not None:
                _log_fdir(fdir)
                motor_control.set_neutral(pi)
                time.sleep(0.1)
                continue

            if _last_fdir_reason is not None:
                logger.info("FDIR cleared, resuming guidance")
                _last_fdir_reason = None

            imu_data     = SimpleNamespace(yaw=snap.yaw, gyrz=snap.gyrz)
            gps_vector   = GpsVector(lat=snap.lat, lon=snap.lon, speed=snap.speed, course=snap.course)
            gps_fidelity = GpsFidelity(fix_quality=snap.fix_quality, sats=snap.sats, rmc_status=snap.rmc_status, gps_health=snap.gps_health)

            result = motor_guidance.guidance(imu_data, gps_vector, gps_fidelity, snap.target, snap.baro_m)

            if result.state == "FDIR":
                _log_fdir("guidance internal FDIR")
                motor_control.set_neutral(pi)
            else:
                motor_control.control(pi, float(result.commanded_yaw_rate))

        except Exception as exc:
            logger.error("ctrl_paragldr unhandled exception: %s", exc, exc_info=True)
            try:
                motor_control.set_neutral(pi)
            except Exception:
                pass

        time.sleep(0.1)


# ---------------------------------------------------------------------------
# App lifecycle
# ---------------------------------------------------------------------------

def init() -> None:
    global pi
    motor_guidance.init_guidance()
    Motor_Release.init_burnwire()
    Motor_Egg.init_solenoid()
    pi = motor_control.init_control()
    logger.info(
        "MotorApp init complete | pigpio connected: %s",
        getattr(pi, "connected", "unknown"),
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
