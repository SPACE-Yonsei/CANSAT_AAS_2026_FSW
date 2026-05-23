"""Motor app: sensor ingestion, guidance orchestration, actuator output.

Per-cycle flow:
  sensor handlers -> _CACHE -> decidefresh -> produceL1input
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

from lib import appargs, config, msgstructure, prevstate, sensorlog, timebase

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
    magx_uT: Optional[float] = None
    magy_uT: Optional[float] = None
    magz_uT: Optional[float] = None
    gyrx_rad_s: Optional[float] = None
    gyry_rad_s: Optional[float] = None
    gyrz_rad_s: Optional[float] = None
    ts: Optional[float] = None
    rx_ts: Optional[float] = None
    freefall: int = 0   # 1=자유낙하 중, 0=정상
    tumble:   int = 0   # 1=텀블링 중,  0=안정
    lin_acc_x: Optional[float] = None
    lin_acc_y: Optional[float] = None
    lin_acc_z: Optional[float] = None
    lin_acc_valid: bool = False


@dataclass
class _BaroFromApp:
    alt_m:     Optional[float] = None
    sink_rate: Optional[float] = None
    ts:        Optional[float] = None
    rx_ts:     Optional[float] = None


@dataclass
class _Cache:
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
MANUAL_STEER_MODE: str = config.MOTOR_MANUAL_NEUTRAL
MOTOR_CTRL_MODE:   str = config.MOTOR_CTRL_MODE
STATE: int = 0
PI = None

_UPDATE_LOCK = threading.Lock()
_CTRL_LOCK = threading.Lock()
_CACHE = _Cache()
_PREV_STATE = -1
_GUIDANCE_STATE = guidance.GuidanceState()

_MANUAL_STEER_DELTA_DEG = config.MANUAL_STEER_DELTA_DEG


_CONTROLLER = None

def _cache_snapshot() -> _Cache:
    return _Cache(
        latest_gps=_GpsFromApp(**vars(_CACHE.latest_gps)),
        latest_imu=_ImuFromApp(**vars(_CACHE.latest_imu)),
        latest_baro=_BaroFromApp(**vars(_CACHE.latest_baro)),
        target_lat=_CACHE.target_lat,
        target_lon=_CACHE.target_lon,
        start_lat=_CACHE.start_lat,
        start_lon=_CACHE.start_lon,
    )

#handler
def handle_gps(data: str) -> None:
    """Parse GPS payload and update cache.

    Payload: lat,lon,pos_ts,course_deg,spd_mps,motion_ts
    Origin acquisition is handled exclusively by guidance.produceL1input;
    this handler only refreshes the cache.
    """
    fields = data.split(",")
    if len(fields) != 6:
        return
    try:
        lat       = float(fields[0])
        lon       = float(fields[1])
        pos_ts    = float(fields[2])
        course_deg = float(fields[3])   # nan when motion invalid
        speed_mps  = float(fields[4])   # nan when motion invalid
        motion_ts  = float(fields[5])   # nan when motion invalid
    except (ValueError, IndexError):
        return

    motion_ok = (math.isfinite(course_deg)
                 and math.isfinite(speed_mps)
                 and math.isfinite(motion_ts))

    sample = _GpsFromApp(
        lat=lat,
        lon=lon,
        course_rad=math.radians(course_deg) if motion_ok else None,
        speed_mps=speed_mps if motion_ok else None,
        pos_ts=pos_ts,
        motion_ts=motion_ts if motion_ok else None,
        rx_ts=timebase.now(),
        pos_health=1,
        motion_health=int(motion_ok),
    )
    with _UPDATE_LOCK:
        _CACHE.latest_gps = sample

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
    """Parse IMU payload and update cache.

    Current payload (16 fields):
      roll,pitch,yaw,ax,ay,az,magx,magy,magz,gyrx,gyry,gyrz_deg_s,sample_ts,freefall,tumble,health
    """
    fields = data.split(",")
    try:
        if len(fields) < 15:
            return
        roll_deg   = float(fields[0])
        pitch_deg  = float(fields[1])
        yaw_deg    = float(fields[2])
        accx_mps2  = float(fields[3])
        accy_mps2  = float(fields[4])
        accz_mps2  = float(fields[5])
        magx_uT    = float(fields[6])
        magy_uT    = float(fields[7])
        magz_uT    = float(fields[8])
        gyrx_deg_s = float(fields[9])
        gyry_deg_s = float(fields[10])
        gyrz_deg_s = float(fields[11])
        sample_ts  = float(fields[12])
        freefall   = int(float(fields[13]))
        tumble     = int(float(fields[14]))
        # field[15]: imuapp이 전송하는 HEALTH 플래그 (0=하드웨어 이상, 1=정상)
        # health=0이면 캐시를 갱신하지 않아 타임스탬프 노후화로 자연스럽게 stale 처리
        imu_health = int(float(fields[15])) if len(fields) >= 16 else 1
        rx_ts = timebase.now()
    except (ValueError, IndexError):
        return

    if not imu_health:
        # 하드웨어 이상 신호: 구 타임스탬프가 유지되도록 캐시 미갱신
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
        magx_uT=magx_uT,
        magy_uT=magy_uT,
        magz_uT=magz_uT,
        gyrx_rad_s=math.radians(gyrx_deg_s),
        gyry_rad_s=math.radians(gyry_deg_s),
        gyrz_rad_s=math.radians(-gyrz_deg_s),  # IMU Z-up: gz+= CCW; negate to match nav convention (gz+ = CW = right turn)
        ts=sample_ts,
        rx_ts=rx_ts,
        freefall=freefall,
        tumble=tumble,
        lin_acc_x=lin_ax if lin_valid else None,
        lin_acc_y=lin_ay if lin_valid else None,
        lin_acc_z=lin_az if lin_valid else None,
        lin_acc_valid=lin_valid,
    )
    with _UPDATE_LOCK:
        _CACHE.latest_imu = imu


def handle_barometer(data: str) -> None:
    """Parse barometer payload and update cache.

    Current payload:
      alt_m,sample_ts,sink_rate
    """
    fields = data.split(",")
    try:
        if len(fields) != 3:
            return
        alt_m     = float(fields[0].strip())
        sample_ts = float(fields[1])
        sink_rate = None if fields[2].strip() == "nan" else float(fields[2].strip())
        rx_ts = timebase.now()
    except (ValueError, IndexError):
        return

    baro = _BaroFromApp(
        alt_m=alt_m,
        sink_rate=sink_rate,
        ts=sample_ts,
        rx_ts=rx_ts,
    )
    with _UPDATE_LOCK:
        _CACHE.latest_baro = baro


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
        _CACHE.target_lat = lat   # mirror for diag/back-compat snapshot
        _CACHE.target_lon = lon
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
            _CACHE.start_lat = None
            _CACHE.start_lon = None
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


def _manual_steer_command(now: float, mode: str) -> control.CtrlOutput:
    """Build a fixed-deflection CtrlOutput for manual MTR LEFT/RIGHT.

    NEUTRAL is filtered upstream (Gate 3 skips this function for NEUTRAL).
    """
    label = f"MANUAL_{mode}"
    cmd = control.WriteNeutral(now, label)
    if mode == config.MOTOR_MANUAL_LEFT:
        delta = -_MANUAL_STEER_DELTA_DEG
    elif mode == config.MOTOR_MANUAL_RIGHT:
        delta = _MANUAL_STEER_DELTA_DEG
    else:
        # defensive: never reached when Gate 3 routes this correctly
        return cmd

    left_pw, right_pw, left_angle, right_angle, delta_arm = control.ConnectRoMo(delta)
    cmd.left_pw = left_pw
    cmd.right_pw = right_pw
    cmd.left_angle_deg = left_angle
    cmd.right_angle_deg = right_angle
    cmd.delta_arm_deg = delta_arm
    # angular_velocity_cmd_deg_s is left at 0 — manual mode skips the yaw-rate loop.
    cmd.valid = True
    cmd.fallback_mode = label
    return cmd


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
def _send_diag(main_queue, cmd, g_out, diag_state: str, snap: _Cache) -> None:
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
    """One-shot: copy guidance origin to _CACHE.start_* and prevstate.
    Returns True if a sync happened, False otherwise. Caller updates _ORIGIN_SAVED.
    """
    if not _GUIDANCE_STATE.origin_ready:
        return False
    with _UPDATE_LOCK:
        _CACHE.start_lat = float(_GUIDANCE_STATE.origin_lat)
        _CACHE.start_lon = float(_GUIDANCE_STATE.origin_lon)
    prevstate.update_start_point(
        _GUIDANCE_STATE.origin_lat, _GUIDANCE_STATE.origin_lon, True
    )
    return True


def ctrl_parafoil(main_queue=None) -> None:
    """Parafoil control loop.

    Per-cycle pipeline:
        snapshot _CACHE  (under _UPDATE_LOCK)
        gates: motor enabled / state >= 3 / state != landed / manual override
        guidance.decidefresh -> produceL1input -> produceL1output
        sync origin to _CACHE + prevstate on first lock
        control.ProduceCtrlInput -> ProduceCtrlOutput
        control.ProducePulse / sensorlog / diag
        sleep(max(0, period - elapsed))  ← rate compensated
    """
    global _ORIGIN_SAVED
    period = 1.0 / max(0.1, float(config.MOTOR_RATE_HZ))
    while MOTORAPP_RUNSTATUS:
        cycle_start = time.monotonic()
        now = timebase.now()
        try:
            with _UPDATE_LOCK:
                motor_enabled = MOTOR_ENABLED
                state         = STATE
                manual_mode   = MANUAL_STEER_MODE
                snap          = _cache_snapshot()

            # ── Gate 1: motor disabled or pre-deploy → zero PWM ───────────────
            if not motor_enabled or state < 3:
                if PI is not None:
                    control.WriteZero(PI)
                _sleep_for_period(cycle_start, period)
                continue

            # ── Gate 2: landed → cut PWM ──────────────────────────────────────
            if state == 5:
                if PI is not None:
                    control.WriteOff(PI)
                _sleep_for_period(cycle_start, period)
                continue

            # ── Gate 3: manual steer override ─────────────────────────────────
            if manual_mode != config.MOTOR_MANUAL_NEUTRAL:
                cmd = _manual_steer_command(now, manual_mode)
                if PI is not None:
                    control.ProducePulse(PI, cmd)
                sensorlog.log_motor_ctrl(cmd)
                _send_diag(main_queue, cmd,
                           guidance.L1Output(timestamp=now, reason=cmd.mode),
                           cmd.mode, snap)
                _sleep_for_period(cycle_start, period)
                continue

            # ── Guidance pipeline ─────────────────────────────────────────────
            fresh    = guidance.decidefresh(snap.latest_gps, snap.latest_imu,
                                             snap.latest_baro, _GUIDANCE_STATE, now)
            l1_input = guidance.produceL1input(fresh, snap.latest_gps, snap.latest_imu,
                                                _GUIDANCE_STATE, state, now)

            # Sync origin to cache + prevstate exactly once
            if not _ORIGIN_SAVED and _sync_origin_to_prevstate():
                _ORIGIN_SAVED = True

            g_out = guidance.produceL1output(l1_input)
            g_out.timestamp = now

            # ── Control ───────────────────────────────────────────────────────
            if g_out.control_valid:
                measured_dps = _measured_yaw_rate_dps(g_out, fresh, snap.latest_imu)
                ctrl_in = control.ProduceCtrlInput(g_out, now)
                with _CTRL_LOCK:
                    cmd = control.ProduceCtrlOutput(
                        _CONTROLLER, ctrl_in, measured_dps, now,
                    )
            else:
                cmd = control.WriteNeutral(now, g_out.reason
                    or config.MOTOR_REASON_GUIDANCE_INACTIVE)

            if PI is not None:
                control.ProducePulse(PI, cmd)
            sensorlog.log_motor_ctrl(cmd)
            diag_state = g_out.reason or (
                config.MOTOR_REASON_DISABLED if not g_out.control_valid
                else config.MOTOR_REASON_GUIDANCE_INACTIVE
            )
            _send_diag(main_queue, cmd, g_out, diag_state, snap)

        except Exception:
            logger.exception("ctrl_parafoil: unhandled exception; writing zero PWM")
            if PI is not None:
                control.WriteZero(PI)

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
        _CACHE.target_lat = float(target_lat)
        _CACHE.target_lon = float(target_lon)
        _GUIDANCE_STATE.target_lat = float(target_lat)
        _GUIDANCE_STATE.target_lon = float(target_lon)

    # Restore origin from prevstate — already-saved, so mark _ORIGIN_SAVED
    start_point = prevstate.get_start_point()
    if start_point is not None:
        lat, lon = start_point
        if -90.0 <= float(lat) <= 90.0 and -180.0 <= float(lon) <= 180.0:
            _CACHE.start_lat = float(lat)
            _CACHE.start_lon = float(lon)
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
