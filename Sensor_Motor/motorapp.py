"""Motor app: sensor ingestion, guidance orchestration, and actuator output.

Current boundary:
  sensor apps -> MotorSensorCache -> ProduceL1Input/ProduceL1Output -> controller -> servo

This module keeps runtime data in an explicit sensor cache and passes snapshots
into guidance instead of reading guidance/control globals.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import threading
import time
from typing import Optional

from lib import appargs, config, msgstructure, prevstate, timebase

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
    health:   int = 0   # 1=하드웨어 정상


@dataclass
class _BaroFromApp:
    alt_m:     Optional[float] = None
    sink_rate: Optional[float] = None
    ts:        Optional[float] = None
    rx_ts:     Optional[float] = None
    health:    int = 0  # 1=하드웨어 정상


@dataclass
class _Cache:
    latest_gps:  _GpsFromApp  = field(default_factory=_GpsFromApp)
    latest_imu:  _ImuFromApp  = field(default_factory=_ImuFromApp)
    latest_baro: _BaroFromApp = field(default_factory=_BaroFromApp)

    target_lat: Optional[float] = None
    target_lon: Optional[float] = None
    start_lat:  Optional[float] = None
    start_lon:  Optional[float] = None


MOTORAPP_RUNSTATUS: bool = True
MOTOR_ENABLED: bool = True
RELEASE_ACTION_ENABLED: bool = True
EGG_ACTION_ENABLED: bool = True
MANUAL_STEER_MODE: str = config.MOTOR_MANUAL_NEUTRAL
STATE: int = 0
PI = None

_UPDATE_LOCK = threading.Lock()
_CTRL_LOCK = threading.Lock()
_CACHE = _Cache()
_PREV_STATE = -1
_START_POINT_LOCKED = False

_MANUAL_STEER_DELTA_DEG = 60.0


_CONTROLLER = None
_L1_STATE = None

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

    Current gpsapp payload is lat,lon,pos_ts,course_deg,spd_mps,motion_ts.
    """
    global _START_POINT_LOCKED
    fields = data.split(",")
    if len(fields) != 6:
        return
    try:
        lat       = float(fields[0])
        lon       = float(fields[1])
        rx_ts = timebase.now()
        pos_ts = float(fields[2])
        course_deg = float(fields[3])   # nan when motion invalid
        speed_mps = float(fields[4])    # nan when motion invalid
        motion_ts = float(fields[5])    # nan when motion invalid
    except (ValueError, IndexError):
        return

    has_motion_sample = (
        math.isfinite(course_deg)
        and math.isfinite(speed_mps)
        and math.isfinite(motion_ts)
    )

    course_rad = math.radians(course_deg) if has_motion_sample else None
    sample = _GpsFromApp(
        lat=lat,
        lon=lon,
        course_rad=course_rad,
        speed_mps=speed_mps if has_motion_sample else None,
        pos_ts=pos_ts,
        motion_ts=motion_ts if has_motion_sample else None,
        rx_ts=rx_ts,
        pos_health=1,
        motion_health=int(has_motion_sample),
    )
    with _UPDATE_LOCK:
        _CACHE.latest_gps = sample
        if not _START_POINT_LOCKED and STATE >= 3:
            _CACHE.start_lat = float(lat)
            _CACHE.start_lon = float(lon)
            _START_POINT_LOCKED = True
            prevstate.update_start_point(float(lat), float(lon), True)

def handle_imu(data: str) -> None:
    """Parse IMU payload and update cache.

    Current payload:
      roll,pitch,yaw,ax,ay,az,magx,magy,magz,gyrx,gyry,gyrz_deg_s,sample_ts,freefall,tumble,health

    Legacy sensor-log payload:
      roll,pitch,yaw,ax,ay,az,magx,magy,magz,gyrx,gyry,gyrz_deg_s,health,sample_ts
    """
    fields = data.split(",")
    try:
        if len(fields) not in (14, 16):
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
        if len(fields) == 16:
            sample_ts  = float(fields[12])
            freefall   = int(float(fields[13]))
            tumble     = int(float(fields[14]))
            health     = int(float(fields[15]))
        else:
            health     = int(float(fields[12]))
            sample_ts  = float(fields[13])
            freefall   = 0
            tumble     = 0
        rx_ts = timebase.now()
    except (ValueError, IndexError):
        return

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
        health=health,
    )
    with _UPDATE_LOCK:
        _CACHE.latest_imu = imu


def handle_barometer(data: str) -> None:
    """Parse barometer payload and update cache.

    Current payload:
      alt_m,sample_ts,sink_rate,health

    Legacy sensor-log payload:
      alt_m,health,sample_ts
    """
    fields = data.split(",")
    try:
        if len(fields) not in (3, 4):
            return
        alt_m     = float(fields[0].strip())
        if len(fields) == 4:
            sample_ts = float(fields[1])
            sink_s    = fields[2].strip()
            sink_rate = None if sink_s == "nan" else float(sink_s)
            health    = int(float(fields[3]))
        else:
            health    = int(float(fields[1]))
            sample_ts = float(fields[2])
            sink_rate = None
        rx_ts = timebase.now()
    except (ValueError, IndexError):
        return

    baro = _BaroFromApp(
        alt_m=alt_m,
        sink_rate=sink_rate,
        ts=sample_ts,
        rx_ts=rx_ts,
        health=health,
    )
    with _UPDATE_LOCK:
        _CACHE.latest_baro = baro


def handle_target_coord(data: str) -> None:
    """lat,lon"""
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
    with _UPDATE_LOCK:
        _CACHE.target_lat = lat
        _CACHE.target_lon = lon


def handle_flight_state(data: str) -> None:
    global STATE, _PREV_STATE, _START_POINT_LOCKED, _L1_STATE, _CONTROLLER
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
            _START_POINT_LOCKED = False
            prevstate.clear_start_point()
            if _L1_STATE is not None and hasattr(guidance, "l1_reset"):
                try:
                    guidance.l1_reset(_L1_STATE)
                except Exception:
                    pass
            do_ctrl_reset = True
        elif new_state in (3, 4):
            gps = _CACHE.latest_gps
            if (
                not _START_POINT_LOCKED
                and gps.pos_ts is not None
                and gps.lat is not None
                and gps.lon is not None
                and -90.0 <= float(gps.lat) <= 90.0
                and -180.0 <= float(gps.lon) <= 180.0
            ):
                _CACHE.start_lat = float(gps.lat)
                _CACHE.start_lon = float(gps.lon)
                _START_POINT_LOCKED = True
                prevstate.update_start_point(float(gps.lat), float(gps.lon), True)

    if do_ctrl_reset and _CONTROLLER is not None and hasattr(control, "controller_reset"):
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
    cmd = control.WriteNeutral(now, f"MANUAL_{mode}")
    steer = str(mode or "").strip().upper()
    if steer == config.MOTOR_MANUAL_LEFT:
        delta = -_MANUAL_STEER_DELTA_DEG
    elif steer == config.MOTOR_MANUAL_RIGHT:
        delta = _MANUAL_STEER_DELTA_DEG
    else:
        cmd.mode = f"MANUAL_{config.MOTOR_MANUAL_NEUTRAL}"
        cmd.fallback_mode = cmd.mode
        return cmd

    left_pw, right_pw, left_angle, right_angle, delta_arm = control.ConnectRoMo(delta)
    cmd.left_pw = left_pw
    cmd.right_pw = right_pw
    cmd.left_angle_deg = left_angle
    cmd.right_angle_deg = right_angle
    cmd.delta_arm_deg = delta_arm
    cmd.angular_velocity_cmd_deg_s = delta
    cmd.valid = True
    cmd.fallback_mode = cmd.mode
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

# Diagnostic telemetry
def _send_diag(main_queue, cmd, g_out, diag_state: str,
               start_lat=None, start_lon=None) -> None:
    if main_queue is None:
        return

    def _fmt(value, digits: int = 4) -> str:
        try:
            f = float(value)
            return "nan" if not math.isfinite(f) else f"{f:.{digits}f}"
        except (TypeError, ValueError):
            return "nan"

    payload = ",".join(
        [
            str(getattr(cmd, "left_pw", 0)),
            str(getattr(cmd, "right_pw", 0)),
            _fmt(start_lat, 6),
            _fmt(start_lon, 6),
            _fmt(getattr(g_out, "target_lat", _CACHE.target_lat), 6),
            _fmt(getattr(g_out, "target_lon", _CACHE.target_lon), 6),
            _fmt(getattr(g_out, "carrot_lat", float("nan")), 6),
            _fmt(getattr(g_out, "carrot_lon", float("nan")), 6),
            _fmt(math.degrees(float(getattr(g_out, "current_heading_rad", float("nan")))), 2),
            diag_state,
            str(int(bool(MOTOR_ENABLED))),
            str(int(bool(RELEASE_ACTION_ENABLED and EGG_ACTION_ENABLED))),
            str(int(bool(RELEASE_ACTION_ENABLED))),
            str(int(bool(EGG_ACTION_ENABLED))),
            _fmt(getattr(g_out, "crossTrack", float("nan"))),
            _fmt(getattr(g_out, "alongTrack", float("nan"))),
            _fmt(getattr(cmd, "angular_velocity_cmd_deg_s", 0.0)),
            _fmt(getattr(cmd, "angular_velocity_meas_deg_s", float("nan"))),
            _fmt(getattr(cmd, "angular_velocity_error_deg_s", 0.0)),
            _fmt(getattr(cmd, "delta_ff_deg", 0.0)),
            _fmt(getattr(cmd, "delta_pid_deg", 0.0)),
            _fmt(getattr(cmd, "delta_arm_deg", 0.0)),
            _fmt(getattr(cmd, "left_angle_deg", 0.0)),
            _fmt(getattr(cmd, "right_angle_deg", 0.0)),
            str(int(bool(getattr(cmd, "saturated", False)))),
            str(int(bool(getattr(cmd, "sensor_valid", False)))),
            _fmt(getattr(cmd, "guidance_command_age_s", 0.0)),
            str(getattr(cmd, "fallback_mode", "")),
            str(getattr(cmd, "mode", "")),
        ]
    )
    msgstructure.send_msg(
        main_queue,
        appargs.MotorAppArg.AppID,
        appargs.CommAppArg.AppID,
        appargs.MotorAppArg.MID_comm_motor_diag,
        payload,
    )


def ctrl_parafoil(main_queue=None) -> None:
    """Parafoil control loop."""
    global _CONTROLLER
    period = 1.0 / max(0.1, float(config.MOTOR_RATE_HZ))
    while MOTORAPP_RUNSTATUS:
        now = timebase.now()
        try:
            if not MOTOR_ENABLED or STATE < 3:
                if PI is not None:
                    control.WriteZero(PI)
                idle_cmd = control.WriteNeutral(now, config.MOTOR_REASON_IDLE)
                idle_out = guidance.L1Output(
                    timestamp=now, nominal=False, degraded=False, reason=config.MOTOR_REASON_IDLE
                )
                with _UPDATE_LOCK:
                    idle_snap = _cache_snapshot()
                _send_diag(main_queue, idle_cmd, idle_out, config.MOTOR_REASON_IDLE,
                           idle_snap.start_lat, idle_snap.start_lon)
                time.sleep(period)
                continue

            if STATE == 5:
                if PI is not None:
                    control.WriteOff(PI)
                landed_cmd = control.WriteNeutral(now, config.MOTOR_REASON_LANDED)
                landed_out = guidance.L1Output(
                    timestamp=now, nominal=False, degraded=False, reason=config.MOTOR_REASON_LANDED
                )
                with _UPDATE_LOCK:
                    landed_snap = _cache_snapshot()
                _send_diag(main_queue, landed_cmd, landed_out, config.MOTOR_REASON_LANDED,
                           landed_snap.start_lat, landed_snap.start_lon)
                time.sleep(period)
                continue

            with _UPDATE_LOCK:
                snap = _cache_snapshot()

            # DR을 현재 시각으로 갱신 (lock 밖 - snap 은 이미 복사됨)
            if MANUAL_STEER_MODE != config.MOTOR_MANUAL_NEUTRAL:
                manual_cmd = _manual_steer_command(now, MANUAL_STEER_MODE)
                manual_out = guidance.L1Output(
                    timestamp=now,
                    nominal=False,
                    degraded=False,
                    reason=manual_cmd.mode,
                    control_valid=bool(manual_cmd.valid),
                    angular_velocity_cmd_rad_s=math.radians(manual_cmd.angular_velocity_cmd_deg_s),
                    target_lat=snap.target_lat,
                    target_lon=snap.target_lon,
                )
                if PI is not None:
                    control.ProducePulse(PI, manual_cmd)
                _send_diag(main_queue, manual_cmd, manual_out, manual_cmd.mode,
                           snap.start_lat, snap.start_lon)
                time.sleep(period)
                continue

            l1_input, mode = guidance.ProduceL1Input(
                gps=snap.latest_gps,
                imu=snap.latest_imu,
                baro=snap.latest_baro,
                origin_lat=snap.start_lat,
                origin_lon=snap.start_lon,
                target_lat=snap.target_lat,
                target_lon=snap.target_lon,
                now=now,
                l1_state=_L1_STATE,
            )
            g_out = guidance.ProduceL1Output(
                l1_input=l1_input,
                mode=mode,
                origin_lat=snap.start_lat,
                origin_lon=snap.start_lon,
                target_lat=snap.target_lat,
                target_lon=snap.target_lon,
                now=now,
                l1_state=_L1_STATE,
            )

            if bool(getattr(g_out, "control_valid", getattr(g_out, "nominal", False))):
                if _CONTROLLER is None:
                    _CONTROLLER = control.MakeCtrler()
                angular_velocity_meas_deg_s = float("nan")
                if (
                    getattr(l1_input, "gyrz", None) is not None
                    and getattr(l1_input, "gyrz_quality", guidance.SensorQuality.STALE)
                    == guidance.SensorQuality.FRESH
                ):
                    angular_velocity_meas_deg_s = math.degrees(float(l1_input.gyrz))
                with _CTRL_LOCK:
                    cmd = control.ProduceCtrlOutput(
                        _CONTROLLER,
                        control.ProduceCtrlInput(g_out, now),
                        angular_velocity_meas_deg_s,
                        now,
                    )
            else:
                cmd = control.WriteNeutral(now, getattr(g_out, "reason", config.MOTOR_REASON_GUIDANCE_INACTIVE))

            if PI is not None:
                control.ProducePulse(PI, cmd)
            diag_state = (
                config.MOTOR_REASON_DEGRADED
                if bool(getattr(g_out, "control_valid", False)) and bool(getattr(g_out, "degraded", False))
                else config.MOTOR_REASON_ACTIVE if bool(getattr(g_out, "nominal", False))
                else str(getattr(g_out, "reason", config.MOTOR_REASON_DISABLED) or config.MOTOR_REASON_DISABLED)
            )
            _send_diag(main_queue, cmd, g_out, diag_state, snap.start_lat, snap.start_lon)

        except Exception:
            if PI is not None:
                control.WriteZero(PI)
        time.sleep(period)


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


def init() -> None:
    global PI, MOTOR_ENABLED, RELEASE_ACTION_ENABLED, EGG_ACTION_ENABLED
    global MANUAL_STEER_MODE, _START_POINT_LOCKED, _CONTROLLER, _L1_STATE
    prevstate.init_prevstate()
    MOTOR_ENABLED = prevstate.is_motor_enabled()
    RELEASE_ACTION_ENABLED = True
    EGG_ACTION_ENABLED = True
    MANUAL_STEER_MODE = config.MOTOR_MANUAL_NEUTRAL

    target_lat, target_lon = prevstate.get_target_gps()
    if (
        -90.0 <= float(target_lat) <= 90.0
        and -180.0 <= float(target_lon) <= 180.0
        and not (target_lat == 0.0 and target_lon == 0.0)
    ):
        _CACHE.target_lat = float(target_lat)
        _CACHE.target_lon = float(target_lon)

    start_point = prevstate.get_start_point()
    if start_point is not None:
        lat, lon = start_point
        if -90.0 <= float(lat) <= 90.0 and -180.0 <= float(lon) <= 180.0:
            _CACHE.start_lat = float(lat)
            _CACHE.start_lon = float(lon)
            _START_POINT_LOCKED = True

    if hasattr(guidance, "make_l1_state"):
        try:
            _L1_STATE = guidance.make_l1_state()
        except Exception:
            pass

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
