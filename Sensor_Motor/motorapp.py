"""Motor app: sensor ingestion, guidance orchestration, actuator output.

Per-cycle flow:
  sensor handlers -> _RAW -> decidefresh -> produceL1input
  -> produceL1output -> ProduceCtrlInput -> ProduceCtrlOutput
  -> ProducePulse / sensorlog / diag
"""

from __future__ import annotations

from dataclasses import dataclass, field
import logging
import math
import threading
import time
from typing import Optional

from lib import appargs, config, msgstructure, prevstate, sensorlog

logger = logging.getLogger(__name__)

from . import control, guidance


@dataclass
class _GpsFromApp:
    lat: Optional[float] = None
    lon: Optional[float] = None
    course_rad: Optional[float] = None
    speed_mps: Optional[float] = None
    pos_ts: Optional[float] = None
    motion_ts: Optional[float] = None
    rx_ts: Optional[float] = None
    pos_health: int = 0
    motion_health: int = 0


@dataclass
class _ImuFromApp:
    roll_rad: Optional[float] = None
    pitch_rad: Optional[float] = None
    yaw_rad: Optional[float] = None
    accx_mps2: Optional[float] = None
    accy_mps2: Optional[float] = None
    accz_mps2: Optional[float] = None
    gyrx_rad_s: Optional[float] = None
    gyry_rad_s: Optional[float] = None
    gyrz_rad_s: Optional[float] = None
    ts: Optional[float] = None
    rx_ts: Optional[float] = None
    lin_acc_x: Optional[float] = None
    lin_acc_y: Optional[float] = None
    lin_acc_z: Optional[float] = None
    lin_acc_valid: bool = False
    health: int = 0


@dataclass
class _BaroFromApp:
    alt_m:     Optional[float] = None
    sink_rate: Optional[float] = None
    rx_ts:     Optional[float] = None
    health:    int = 0


@dataclass
class _Raw:
    latest_gps:  _GpsFromApp  = field(default_factory=_GpsFromApp)
    latest_imu:  _ImuFromApp  = field(default_factory=_ImuFromApp)
    latest_baro: _BaroFromApp = field(default_factory=_BaroFromApp)

    target_lat: Optional[float] = None
    target_lon: Optional[float] = None
    start_lat:  Optional[float] = None
    start_lon:  Optional[float] = None


_ORIGIN_SAVED: bool = False

MOTORAPP_RUNSTATUS: bool = True
MOTOR_ENABLED: bool = True
RELEASE_ACTION_ENABLED: bool = True
EGG_ACTION_ENABLED: bool = True
MOTOR_CTRL_MODE:   str = config.MOTOR_CTRL_MODE
STATE: int = 0
PI = None

_UPDATE_LOCK = threading.Lock()
_CTRL_LOCK = threading.Lock()
_RAW = _Raw()
_PREV_STATE = -1
_GUIDANCE_STATE = guidance.GuidanceState()

_imu_heading_fallback_logged: bool = False


_CONTROLLER = None

def _raw_snapshot() -> _Raw:
    return _Raw(
        latest_gps=_GpsFromApp(**vars(_RAW.latest_gps)),
        latest_imu=_ImuFromApp(**vars(_RAW.latest_imu)),
        latest_baro=_BaroFromApp(**vars(_RAW.latest_baro)),
        target_lat=_RAW.target_lat,
        target_lon=_RAW.target_lon,
        start_lat=_RAW.start_lat,
        start_lon=_RAW.start_lon,
    )

# handler
def handle_gps(data: str) -> None:
    """Parse GPS payload and update raw store.

    Payload (8 fields): lat,lon,pos_health,pos_ts,course_deg,speed_mps,motion_health,motion_ts
    pos_health=0  → lat/lon/pos_ts are nan.
    motion_health=0 → course/speed/motion_ts are nan.
    Origin acquisition is handled exclusively by guidance.produceL1input.
    """
    fields = data.split(",")
    if len(fields) != 8:
        return
    try:
        lat          = float(fields[0])
        lon          = float(fields[1])
        pos_health   = int(float(fields[2]))
        pos_ts       = float(fields[3])
        course_deg   = float(fields[4])
        speed_mps    = float(fields[5])
        motion_health = int(float(fields[6]))
        motion_ts    = float(fields[7])
    except (ValueError, IndexError):
        return

    sample = _GpsFromApp(
        lat=lat          if pos_health else None,
        lon=lon          if pos_health else None,
        pos_ts=pos_ts    if pos_health else None,
        course_rad=math.radians(course_deg) if motion_health else None,
        speed_mps=speed_mps                 if motion_health else None,
        motion_ts=motion_ts                 if motion_health else None,
        rx_ts=time.monotonic(),
        pos_health=pos_health,
        motion_health=motion_health,
    )
    with _UPDATE_LOCK:
        _RAW.latest_gps = sample

def _compute_linear_acc(
    roll_deg: float,
    pitch_deg: float,
    ax: float, ay: float, az: float,
    g: float = 9.81,
) -> tuple:
    """Remove gravity from body-frame accelerometer.

    Yaw is irrelevant for gravity removal (NED z-axis is yaw-invariant).
    Gravity body frame (NED z-down, gravity NED = [0,0,+g]):
        g_x = -sin(pitch)*g
        g_y =  cos(pitch)*sin(roll)*g
        g_z =  cos(pitch)*cos(roll)*g
    """
    roll  = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    cp = math.cos(pitch)
    g_x = -math.sin(pitch) * g
    g_y = cp * math.sin(roll) * g
    g_z = cp * math.cos(roll) * g
    return ax - g_x, ay - g_y, az - g_z


def handle_imu(data: str) -> None:
    """Parse IMU payload and update raw store.

    Payload (11 fields): roll,pitch,yaw,ax,ay,az,gyrx,gyry,gyrz,health,sample_ts
    All angles in degrees, acc in m/s², gyro in deg/s.
    health=0 → data fields 0-8 are nan; timestamps still valid.
    """
    fields = data.split(",")
    if len(fields) != 11:
        return
    try:
        health    = int(float(fields[9]))
        sample_ts = float(fields[10])
        rx_ts     = time.monotonic()
    except (ValueError, IndexError):
        return

    if health:
        try:
            roll_deg   = float(fields[0])
            pitch_deg  = float(fields[1])
            yaw_deg    = float(fields[2])
            accx_mps2  = float(fields[3])
            accy_mps2  = float(fields[4])
            accz_mps2  = float(fields[5])
            gyrx_deg_s = float(fields[6])
            gyry_deg_s = float(fields[7])
            gyrz_deg_s = float(fields[8])
        except (ValueError, IndexError):
            return
        lin_ax, lin_ay, lin_az = _compute_linear_acc(
            roll_deg, pitch_deg, accx_mps2, accy_mps2, accz_mps2
        )
        lin_valid = math.isfinite(lin_ax) and math.isfinite(lin_ay) and math.isfinite(lin_az)
        imu = _ImuFromApp(
            roll_rad=math.radians(roll_deg),
            pitch_rad=math.radians(pitch_deg),
            yaw_rad=math.radians(yaw_deg),
            accx_mps2=accx_mps2,
            accy_mps2=accy_mps2,
            accz_mps2=accz_mps2,
            gyrx_rad_s=math.radians(gyrx_deg_s),
            gyry_rad_s=math.radians(gyry_deg_s),
            gyrz_rad_s=math.radians(-gyrz_deg_s),  # IMU Z-up gz+= CCW; negate → nav gz+ = CW = right turn
            ts=sample_ts,
            rx_ts=rx_ts,
            lin_acc_x=lin_ax if lin_valid else None,
            lin_acc_y=lin_ay if lin_valid else None,
            lin_acc_z=lin_az if lin_valid else None,
            lin_acc_valid=lin_valid,
            health=1,
        )
    else:
        # 하드웨어 이상: 타임스탬프만 갱신, 모든 데이터 필드는 None 유지
        imu = _ImuFromApp(ts=sample_ts, rx_ts=rx_ts, health=0)

    with _UPDATE_LOCK:
        _RAW.latest_imu = imu


def handle_barometer(data: str) -> None:
    """Parse barometer payload and update raw store.

    Payload (3 fields): alt_m,sink_rate,health
    health=0 → alt_m and sink_rate are nan.
    """
    fields = data.split(",")
    if len(fields) != 3:
        return
    try:
        alt_raw   = float(fields[0].strip())
        sink_raw  = float(fields[1].strip())
        health    = int(float(fields[2].strip()))
        rx_ts     = time.monotonic()
    except (ValueError, IndexError):
        return

    baro = _BaroFromApp(
        alt_m=alt_raw     if health and math.isfinite(alt_raw)  else None,
        sink_rate=sink_raw if health and math.isfinite(sink_raw) else None,
        rx_ts=rx_ts,
        health=health,
    )
    with _UPDATE_LOCK:
        _RAW.latest_baro = baro


def handle_target_coord(data: str) -> None:
    """Target lat,lon — single source of truth is _GUIDANCE_STATE.

    Rejects out-of-range coords and the (0,0) sentinel (matches init()).
    """
    fields = data.split(",")
    if len(fields) != 2:
        return
    try:
        lat = float(fields[0])
        lon = float(fields[1])
    except (ValueError, IndexError):
        return
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return
    if abs(lat) < 1e-9 and abs(lon) < 1e-9:
        return   # (0,0) sentinel — not a real target
    with _UPDATE_LOCK:
        _RAW.target_lat = lat   # mirror for diag/back-compat snapshot
        _RAW.target_lon = lon
        _GUIDANCE_STATE.target_lat = lat
        _GUIDANCE_STATE.target_lon = lon
        _GUIDANCE_STATE.target_ready = False  # trigger re-projection next cycle


def handle_flight_state(data: str) -> None:
    """Update flight state. Origin acquisition stays in guidance pipeline."""
    global STATE, _PREV_STATE, _CONTROLLER, _ORIGIN_SAVED
    try:
        new_state = int(data.split(",")[0])
    except (ValueError, IndexError):
        return
    if new_state == STATE:
        return

    do_ctrl_reset = False
    with _UPDATE_LOCK:
        _PREV_STATE = STATE
        STATE = new_state
        if new_state < 3:
            _RAW.start_lat = None
            _RAW.start_lon = None
            _ORIGIN_SAVED = False
            prevstate.clear_start_point()
            guidance.reset_guidance_state_for_flight(_GUIDANCE_STATE)
            do_ctrl_reset = True

    if do_ctrl_reset and _CONTROLLER is not None:
        with _CTRL_LOCK:
            control.controller_reset(_CONTROLLER)


def handle_release(data: str = "TRIGGER") -> None:
    if not RELEASE_ACTION_ENABLED:
        return
    try:
        from . import Motor_Release
    except Exception:
        return
    if not hasattr(Motor_Release, "activate_burnwire"):
        return
    threading.Thread(target=Motor_Release.activate_burnwire, daemon=True, name="Burnwire").start()


def handle_egg_drop() -> None:
    if not EGG_ACTION_ENABLED:
        return
    try:
        from . import Motor_Egg
    except Exception:
        return
    if not hasattr(Motor_Egg, "activate_solenoid"):
        return
    threading.Thread(target=Motor_Egg.activate_solenoid, daemon=True, name="Solenoid").start()


def handle_mec(data: str) -> None:
    global MOTOR_ENABLED
    cmd = data.strip().upper()
    if cmd == "ON":
        MOTOR_ENABLED = True
        prevstate.update_motor_enabled(True)
    elif cmd == "OFF":
        MOTOR_ENABLED = False
        prevstate.update_motor_enabled(False)
        with _UPDATE_LOCK:
            if PI is not None:
                control.WriteZero(PI)


def _imu_heading_ctrl_input(now: float, yaw_rad: float, snap: _Raw) -> control.CtrlInput:
    """Build CtrlInput for IMU_HEADING mode using magnetometer yaw.

    When GPS position and target are available, computes the geographic bearing
    to the target and converts it to the startup-zeroed IMU yaw frame using
    prevstate.PREV_YAW_OFFSET.  Works at any GPS speed (position only, no
    velocity needed).  Falls back to holding the current heading when GPS
    position or target are missing.
    """
    global _imu_heading_fallback_logged
    # Default: hold current heading (error=0) until GPS+target are available.
    target_heading_deg = math.degrees(yaw_rad)

    gps = snap.latest_gps
    t_lat = snap.target_lat
    t_lon = snap.target_lon
    gps_lat = gps.lat if gps is not None else None
    gps_lon = gps.lon if gps is not None else None

    if (gps_lat is not None and math.isfinite(gps_lat)
            and gps_lon is not None and math.isfinite(gps_lon)
            and t_lat is not None and math.isfinite(t_lat)
            and t_lon is not None and math.isfinite(t_lon)
            and abs(t_lat) > 1e-9 and abs(t_lon) > 1e-9):
        # Geographic bearing from GPS position to target (True North, radians)
        dlon = math.radians(t_lon - gps_lon)
        lat1 = math.radians(gps_lat)
        lat2 = math.radians(t_lat)
        y_b = math.sin(dlon) * math.cos(lat2)
        x_b = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        abs_bearing_rad = math.atan2(y_b, x_b)

        # Convert absolute bearing to IMU-relative yaw frame.
        # YAW = raw_yaw + yaw_offset, so target_imu = target_compass + yaw_offset.
        # yaw_offset is delivered per-cycle via the IMU message (field 16).
        target_heading_deg = math.degrees(abs_bearing_rad) + prevstate.PREV_YAW_OFFSET
        if _imu_heading_fallback_logged:
            logger.info(
                "IMU_HEADING: bearing restored — gps=(%.5f,%.5f) target=(%.5f,%.5f)"
                " bearing=%.1f° yaw_off=%.1f°",
                gps_lat, gps_lon, t_lat, t_lon,
                math.degrees(abs_bearing_rad), prevstate.PREV_YAW_OFFSET,
            )
            _imu_heading_fallback_logged = False
    else:
        if not _imu_heading_fallback_logged:
            logger.info(
                "IMU_HEADING fallback (holding heading): gps_lat=%s gps_lon=%s t_lat=%s t_lon=%s",
                gps_lat, gps_lon, t_lat, t_lon,
            )
            _imu_heading_fallback_logged = True

    error_deg = (target_heading_deg - math.degrees(yaw_rad) + 180.0) % 360.0 - 180.0
    cmd_dps = max(-config.IMU_HEADING_MAX_CMD_DEG_S,
                  min(config.IMU_HEADING_MAX_CMD_DEG_S,
                      config.IMU_HEADING_KP * error_deg))
    return control.CtrlInput(
        angular_velocity_cmd_deg_s=cmd_dps,
        ground_speed_mps=0.0,
        valid=True,
        timestamp=now,
        pid_enabled=True,
    )


def handle_mtr(data: str) -> None:
    global MANUAL_STEER_MODE
    mode = str(data or "").strip().upper()
    if mode in {
        config.MOTOR_MANUAL_LEFT,
        config.MOTOR_MANUAL_RIGHT,
        config.MOTOR_MANUAL_NEUTRAL,
    }:
        MANUAL_STEER_MODE = mode


def handle_cmc(data: str) -> None:
    global MOTOR_CTRL_MODE
    mode = str(data or "").strip().upper()
    valid = {
        config.MOTOR_CTRL_MODE_GPS_GUIDED,
        config.MOTOR_CTRL_MODE_GPS_ONLY,
        config.MOTOR_CTRL_MODE_IMU_HEADING,
    }
    if mode not in valid:
        logger.debug("CMC: rejected unknown mode %r", mode)
        return
    MOTOR_CTRL_MODE = mode
    logger.info("CMC: switched to %s", mode)


def handle_fac(data: str) -> None:
    global RELEASE_ACTION_ENABLED, EGG_ACTION_ENABLED
    raw = data.strip().upper().replace(" ", "")
    parts = [p for p in raw.split(",") if p]
    if len(parts) == 1 and parts[0] in {"ON", "OFF"}:
        actor = "ALL"
        state = parts[0]
    elif len(parts) == 2 and parts[0] in {"ALL", "REL", "EGG"} and parts[1] in {"ON", "OFF"}:
        actor = parts[0]
        state = parts[1]
    else:
        return

    enabled = state == "ON"
    if actor in {"ALL", "REL"}:
        RELEASE_ACTION_ENABLED = enabled
    if actor in {"ALL", "EGG"}:
        EGG_ACTION_ENABLED = enabled

def _fmt_num(value, digits: int = 4) -> str:
    """Format a value as fixed-point, or 'nan' on any error/non-finite."""
    try:
        f = float(value)
        return "nan" if not math.isfinite(f) else f"{f:.{digits}f}"
    except (TypeError, ValueError):
        return "nan"


# Diagnostic telemetry
def _send_diag(main_queue, cmd, g_out, diag_state: str, snap: _Raw) -> None:
    """Build and emit the motor diag string. snap supplies thread-safe state."""
    if main_queue is None:
        return

    # Carrot == target in target-fixed L1 homing; logged as latlon for ground display.
    carrot_lat = snap.target_lat
    carrot_lon = snap.target_lon
    payload = ",".join([
        str(cmd.left_pw),
        str(cmd.right_pw),
        _fmt_num(snap.start_lat,  6),
        _fmt_num(snap.start_lon,  6),
        _fmt_num(snap.target_lat, 6),
        _fmt_num(snap.target_lon, 6),
        _fmt_num(carrot_lat,      6),
        _fmt_num(carrot_lon,      6),
        _fmt_num(math.degrees(
            snap.latest_imu.yaw_rad
            if (MOTOR_CTRL_MODE == config.MOTOR_CTRL_MODE_IMU_HEADING
                and snap.latest_imu.yaw_rad is not None
                and math.isfinite(snap.latest_imu.yaw_rad))
            else g_out.current_heading_rad
            if math.isfinite(g_out.current_heading_rad)
            else float("nan")
        ), 2),
        diag_state,
        str(int(bool(MOTOR_ENABLED))),
        str(int(bool(RELEASE_ACTION_ENABLED and EGG_ACTION_ENABLED))),
        str(int(RELEASE_ACTION_ENABLED)),
        str(int(EGG_ACTION_ENABLED)),
        _fmt_num(g_out.crossTrack),
        _fmt_num(g_out.alongTrack),
        _fmt_num(cmd.angular_velocity_cmd_deg_s),
        _fmt_num(cmd.angular_velocity_meas_deg_s),
        _fmt_num(cmd.angular_velocity_error_deg_s),
        _fmt_num(cmd.delta_ff_deg),
        _fmt_num(cmd.delta_pid_deg),
        _fmt_num(cmd.delta_arm_deg),
        _fmt_num(cmd.left_angle_deg),
        _fmt_num(cmd.right_angle_deg),
        str(int(bool(cmd.saturated))),
        str(int(bool(cmd.sensor_valid))),
        _fmt_num(cmd.guidance_command_age_s),
        str(cmd.fallback_mode),
        str(cmd.mode),
    ])
    msgstructure.send_msg(
        main_queue,
        appargs.MotorAppArg.AppID,
        appargs.CommAppArg.AppID,
        appargs.MotorAppArg.MID_comm_motor_diag,
        payload,
    )


def _measured_yaw_rate_dps(g_out, fresh, imu) -> float:
    """Return measured yaw rate (deg/s) for PID feedback, or NaN if unusable."""
    if not g_out.pid_enabled or not fresh.imu_gyrz_fresh:
        return float("nan")
    gz = getattr(imu, "gyrz_rad_s", None) if imu is not None else None
    if gz is None or not math.isfinite(float(gz)):
        return float("nan")
    return math.degrees(config.GYRZ_SIGN * float(gz))


def _sync_origin_to_prevstate() -> bool:
    """One-shot: copy guidance origin to _RAW.start_* and prevstate.
    Returns True if a sync happened, False otherwise. Caller updates _ORIGIN_SAVED.
    """
    if not _GUIDANCE_STATE.origin_ready:
        return False
    with _UPDATE_LOCK:
        _RAW.start_lat = float(_GUIDANCE_STATE.origin_lat)
        _RAW.start_lon = float(_GUIDANCE_STATE.origin_lon)
    prevstate.update_start_point(
        _GUIDANCE_STATE.origin_lat, _GUIDANCE_STATE.origin_lon, True
    )
    return True


def _ctrl_cycle(main_queue, now: float) -> Optional[control.CtrlOutput]:
    """One iteration of the ctrl_parafoil loop. Pulled out for testability.

    Returns the CtrlOutput emitted this cycle (or None if a gate short-circuited
    without producing a command). Side effects: PWM, logs, diag telemetry.
    """
    global _ORIGIN_SAVED
    try:
        with _UPDATE_LOCK:
            motor_enabled = MOTOR_ENABLED
            state         = STATE
            snap          = _raw_snapshot()

        # ── Gate 1: motor disabled or pre-deploy → zero PWM ───────────────────
        if not motor_enabled or state < 3:
            if PI is not None:
                control.WriteZero(PI)
            return None

        # ── Gate 2: landed → cut PWM ──────────────────────────────────────────
        if state == 5:
            if PI is not None:
                control.WriteOff(PI)
            return None

        # ── Gate 3.5: IMU heading mode (bypass GPS/DR guidance) ───────────────
        if MOTOR_CTRL_MODE == config.MOTOR_CTRL_MODE_IMU_HEADING:
            yaw = snap.latest_imu.yaw_rad
            if yaw is not None and math.isfinite(yaw):
                gyrz = snap.latest_imu.gyrz_rad_s
                measured_dps = math.degrees(gyrz) if (gyrz is not None and math.isfinite(gyrz)) else float("nan")
                ctrl_input = _imu_heading_ctrl_input(now, yaw, snap)
                with _CTRL_LOCK:
                    cmd = control.ProduceCtrlOutput(_CONTROLLER, ctrl_input, measured_dps, now)
                if PI is not None:
                    control.ProducePulse(PI, cmd)
                sensorlog.log_motor_ctrl(cmd)
                g_diag = guidance.L1Output(timestamp=now, current_heading_rad=yaw, reason="IMU_HEADING")
                _send_diag(main_queue, cmd, g_diag, "IMU_HEADING", snap)
                return cmd
            # IMU yaw invalid → fall through to GPS/DR guidance

        # ── Guidance pipeline ─────────────────────────────────────────────────
        fresh    = guidance.decidefresh(snap.latest_gps, snap.latest_imu,
                                         snap.latest_baro, _GUIDANCE_STATE, now)
        l1_input = guidance.produceL1input(fresh, snap.latest_gps, snap.latest_imu,
                                            _GUIDANCE_STATE, state, now)

        # Sync origin to cache + prevstate exactly once
        if not _ORIGIN_SAVED and _sync_origin_to_prevstate():
            _ORIGIN_SAVED = True

        l1_output = guidance.produceL1output(l1_input)
        l1_output.timestamp = now

        # ── Control ───────────────────────────────────────────────────────────
        if l1_output.control_valid:
            measured_dps = _measured_yaw_rate_dps(l1_output, fresh, snap.latest_imu)
            ctrl_input = control.ProduceCtrlInput(l1_output, now)
            with _CTRL_LOCK:
                cmd = control.ProduceCtrlOutput(
                    _CONTROLLER, ctrl_input, measured_dps, now,
                )
        else:
            cmd = control.WriteNeutral(now, l1_output.reason
                or config.MOTOR_REASON_GUIDANCE_INACTIVE)

        if PI is not None:
            control.ProducePulse(PI, cmd)
        sensorlog.log_motor_ctrl(cmd)
        diag_state = l1_output.reason or (
            config.MOTOR_REASON_DISABLED if not l1_output.control_valid
            else config.MOTOR_REASON_GUIDANCE_INACTIVE
        )
        _send_diag(main_queue, cmd, l1_output, diag_state, snap)
        return cmd

    except Exception:
        logger.exception("ctrl_parafoil: unhandled exception; writing zero PWM")
        if PI is not None:
            control.WriteZero(PI)
        return None


def ctrl_parafoil(main_queue=None) -> None:
    """Parafoil control loop. Delegates per-cycle work to _ctrl_cycle.

    Loop rate compensated: sleep duration = period − cycle compute time.
    """
    period = 1.0 / max(0.1, float(config.MOTOR_RATE_HZ))
    while MOTORAPP_RUNSTATUS:
        cycle_start = time.monotonic()
        _ctrl_cycle(main_queue, time.monotonic())
        _sleep_for_period(cycle_start, period)


def _sleep_for_period(cycle_start: float, period: float) -> None:
    """Sleep so that the loop period stays close to 1/MOTOR_RATE_HZ regardless
    of per-cycle compute time."""
    remaining = period - (time.monotonic() - cycle_start)
    if remaining > 0.0:
        time.sleep(remaining)


def dispatch(msg: str) -> None:
    global MOTORAPP_RUNSTATUS
    unpacked = msgstructure.unpack_msg(msg)
    if unpacked is False:
        return
    mid = unpacked.msg_id
    if mid == appargs.MainAppArg.MID_TerminateProcess:
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
        handle_release(unpacked.data)
    elif mid == appargs.FlightlogicAppArg.MID_motor_EggDrop:
        handle_egg_drop()
    elif mid == appargs.CommAppArg.MID_RouteCmd_MEC:
        handle_mec(unpacked.data)
    elif mid == appargs.CommAppArg.MID_RouteCmd_MTR:
        handle_mtr(unpacked.data)
    elif mid == appargs.CommAppArg.MID_RouteCmd_FAC:
        handle_fac(unpacked.data)
    elif mid == appargs.CommAppArg.MID_RouteCmd_CMC:
        handle_cmc(unpacked.data)


def init() -> None:
    global PI, MOTOR_ENABLED, RELEASE_ACTION_ENABLED, EGG_ACTION_ENABLED
    global MANUAL_STEER_MODE, _CONTROLLER, _ORIGIN_SAVED
    prevstate.init_prevstate()
    MOTOR_ENABLED = prevstate.is_motor_enabled()
    RELEASE_ACTION_ENABLED = True
    EGG_ACTION_ENABLED = True
    MANUAL_STEER_MODE = config.MOTOR_MANUAL_NEUTRAL

    # Restore target from prevstate
    target_lat, target_lon = prevstate.get_target_gps()
    if (
        -90.0 <= float(target_lat) <= 90.0
        and -180.0 <= float(target_lon) <= 180.0
        and not (target_lat == 0.0 and target_lon == 0.0)
    ):
        _RAW.target_lat = float(target_lat)
        _RAW.target_lon = float(target_lon)
        _GUIDANCE_STATE.target_lat = float(target_lat)
        _GUIDANCE_STATE.target_lon = float(target_lon)

    # Restore origin from prevstate — already-saved, so mark _ORIGIN_SAVED
    start_point = prevstate.get_start_point()
    if start_point is not None:
        lat, lon = start_point
        if -90.0 <= float(lat) <= 90.0 and -180.0 <= float(lon) <= 180.0:
            _RAW.start_lat = float(lat)
            _RAW.start_lon = float(lon)
            _GUIDANCE_STATE.origin_lat = float(lat)
            _GUIDANCE_STATE.origin_lon = float(lon)
            _GUIDANCE_STATE.origin_ready = True
            _ORIGIN_SAVED = True
            guidance.convert_target_to_local_en_if_possible(_GUIDANCE_STATE)

    _CONTROLLER = control.MakeCtrler()
    PI = control.init_control()

    try:
        from . import Motor_Release
        if hasattr(Motor_Release, "init_burnwire"):
            Motor_Release.init_burnwire()
    except Exception:
        pass

    try:
        from . import Motor_Egg
        if hasattr(Motor_Egg, "init_solenoid"):
            Motor_Egg.init_solenoid()
    except Exception:
        pass


def motorapp_main(main_queue, main_pipe=None) -> None:
    global MOTORAPP_RUNSTATUS
    if main_pipe is None:
        main_pipe = main_queue
        main_queue = None
    init()

    ctrl_thread = threading.Thread(
        target=ctrl_parafoil,
        args=(main_queue,),
        daemon=True,
        name="MotorControlLoop",
    )
    ctrl_thread.start()

    poll_period = 1.0 / max(0.1, float(config.MOTOR_RATE_HZ))
    try:
        while MOTORAPP_RUNSTATUS:
            try:
                if main_pipe.poll(poll_period):
                    dispatch(main_pipe.recv())
            except (KeyboardInterrupt, EOFError, OSError):
                break
    except KeyboardInterrupt:
        pass

    MOTORAPP_RUNSTATUS = False
    ctrl_thread.join(timeout=1.0)
