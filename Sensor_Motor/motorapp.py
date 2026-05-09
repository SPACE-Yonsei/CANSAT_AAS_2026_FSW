"""Motor app: sensor ingestion, guidance orchestration, and actuator output.

Current boundary:
  sensor apps -> MotorSensorCache -> ProduceL1Input/ProduceL1Output -> controller -> servo

This module keeps runtime data in an explicit sensor cache and passes snapshots
into guidance instead of reading guidance/control globals.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import importlib
import logging
import math
import threading
import time
from types import SimpleNamespace
from typing import Any, Optional

from lib import appargs, config, msgstructure, prevstate

LOGGER = logging.getLogger(__name__)

GPS_HISTORY_SEC = 10.0
IMU_HISTORY_SEC = 5.0
BARO_HISTORY_SEC = 10.0


def _history_len(rate_hz: float, seconds: float) -> int:
    return max(1, int(math.ceil(max(0.1, rate_hz) * seconds)))


@dataclass
class _GpsFromApp:
    lat: Optional[float] = None
    lon: Optional[float] = None
    course_rad: Optional[float] = None
    speed_mps: Optional[float] = None
    sample_ts: Optional[float] = None
    rx_ts: Optional[float] = None
    pos_ts: Optional[float] = None
    motion_ts: Optional[float] = None
    pos_health: bool = False
    motion_health: bool = False


@dataclass
class _ImuFromApp:
    gyrz_rad_s: Optional[float] = None
    sample_ts: Optional[float] = None
    rx_ts: Optional[float] = None
    ts: Optional[float] = None
    health: bool = False


@dataclass
class _BaroFromApp:
    alt_m: Optional[float] = None
    sample_ts: Optional[float] = None
    rx_ts: Optional[float] = None
    ts: Optional[float] = None
    health: bool = False


@dataclass
class _Cache:
    latest_gps: _GpsFromApp = field(default_factory=_GpsFromApp)
    last_gps: _GpsFromApp = field(default_factory=_GpsFromApp)
    gps_history: deque[_GpsFromApp] = field(
        default_factory=lambda: deque(
            maxlen=_history_len(float(config.GPS_RATE_HZ), GPS_HISTORY_SEC)
        )
    )

    latest_imu: _ImuFromApp = field(default_factory=_ImuFromApp)
    last_imu: _ImuFromApp = field(default_factory=_ImuFromApp)
    imu_history: deque[_ImuFromApp] = field(
        default_factory=lambda: deque(
            maxlen=_history_len(float(config.IMU_RATE_HZ), IMU_HISTORY_SEC)
        )
    )

    latest_baro: _BaroFromApp = field(default_factory=_BaroFromApp)
    last_baro: _BaroFromApp = field(default_factory=_BaroFromApp)
    baro_history: deque[_BaroFromApp] = field(
        default_factory=lambda: deque(
            maxlen=_history_len(float(config.BAROMETER_RATE_HZ), BARO_HISTORY_SEC)
        )
    )

    target_lat: Optional[float] = None
    target_lon: Optional[float] = None
    start_lat: Optional[float] = None
    start_lon: Optional[float] = None


MOTORAPP_RUNSTATUS: bool = True
MOTOR_ENABLED: bool = True
STATE: int = 0
PI = None

_UPDATE_LOCK = threading.Lock()
_CACHE = _Cache()
_PREV_STATE = -1
_START_POINT_LOCKED = False
_CONTROLLER = None
_L1_STATE = None
_CONTROL_MOD = None
_GUIDANCE_MOD = None


def _motor_rate_hz() -> float:
    try:
        return max(0.1, float(config.MOTOR_RATE_HZ))
    except (TypeError, ValueError):
        return 10.0


def _motor_period_sec() -> float:
    return 1.0 / _motor_rate_hz()


def _load_module(name: str):
    try:
        return importlib.import_module(name)
    except Exception as exc:
        LOGGER.error("Failed to import %s: %s", name, exc)
        return None


def _guidance():
    global _GUIDANCE_MOD
    if _GUIDANCE_MOD is None:
        _GUIDANCE_MOD = _load_module("Sensor_Motor.guidance")
    return _GUIDANCE_MOD


def _control():
    global _CONTROL_MOD
    if _CONTROL_MOD is None:
        _CONTROL_MOD = _load_module("Sensor_Motor.control")
    return _CONTROL_MOD


def _null_brake_command(now: float, mode: str = "NEUTRAL"):
    ctl = _control()
    if ctl is not None and hasattr(ctl, "BrakeCommand"):
        cmd = ctl.BrakeCommand(timestamp=now)
        cmd.mode = mode
        cmd.fallback_mode = mode
        return cmd
    return SimpleNamespace(
        timestamp=now,
        left_pw=0,
        right_pw=0,
        yaw_rate_cmd_deg_s=0.0,
        yaw_rate_meas_deg_s=float("nan"),
        yaw_rate_error_deg_s=0.0,
        delta_ff_deg=0.0,
        delta_pid_deg=0.0,
        delta_arm_deg=0.0,
        left_angle_deg=0.0,
        right_angle_deg=0.0,
        saturated=False,
        sensor_valid=False,
        guidance_command_age_s=0.0,
        fallback_mode=mode,
        mode=mode,
        valid=False,
    )


def _null_guidance_output(now: float, reason: str = "DISABLED"):
    g = _guidance()
    if g is not None and hasattr(g, "L1Output"):
        try:
            out = g.L1Output(timestamp=now)
        except Exception:
            out = SimpleNamespace(timestamp=now)
    else:
        out = SimpleNamespace(timestamp=now)
    for name, value in {
        "active": False,
        "degraded": False,
        "reason": reason,
        "crossTrack": float("nan"),
        "alongTrack": float("nan"),
        "start_lat": float("nan"),
        "start_lon": float("nan"),
        "target_lat": float("nan"),
        "target_lon": float("nan"),
        "carrot_lat": float("nan"),
        "carrot_lon": float("nan"),
        "current_heading_deg": float("nan"),
        "desired_heading_deg": float("nan"),
    }.items():
        if not hasattr(out, name):
            setattr(out, name, value)
    return out


def _set_neutral() -> None:
    ctl = _control()
    if PI is not None and ctl is not None and hasattr(ctl, "set_neutral"):
        ctl.set_neutral(PI)


def _set_motors_off() -> None:
    ctl = _control()
    if PI is not None and ctl is not None and hasattr(ctl, "set_motors_off"):
        ctl.set_motors_off(PI)


def _set_brake_command(cmd) -> None:
    ctl = _control()
    if PI is not None and ctl is not None and hasattr(ctl, "set_brake_command"):
        ctl.set_brake_command(PI, cmd)


def _latlon_to_ne(lat: float, lon: float, origin_lat: float, origin_lon: float) -> tuple[float, float]:
    earth_r = 6_371_000.0
    dlat = math.radians(lat - origin_lat)
    dlon = math.radians(lon - origin_lon)
    north = dlat * earth_r
    east = dlon * earth_r * math.cos(math.radians(origin_lat))
    return north, east


def _cache_snapshot() -> _Cache:
    snap = _Cache()
    snap.latest_gps = _GpsFromApp(**vars(_CACHE.latest_gps))
    snap.last_gps = _GpsFromApp(**vars(_CACHE.last_gps))
    snap.latest_imu = _ImuFromApp(**vars(_CACHE.latest_imu))
    snap.last_imu = _ImuFromApp(**vars(_CACHE.last_imu))
    snap.latest_baro = _BaroFromApp(**vars(_CACHE.latest_baro))
    snap.last_baro = _BaroFromApp(**vars(_CACHE.last_baro))
    snap.target_lat = _CACHE.target_lat
    snap.target_lon = _CACHE.target_lon
    snap.start_lat = _CACHE.start_lat
    snap.start_lon = _CACHE.start_lon
    return snap


def _lock_start_if_ready() -> None:
    global _START_POINT_LOCKED
    if _START_POINT_LOCKED or STATE < 3:
        return
    gps = _CACHE.latest_gps
    if not gps.pos_health or gps.lat is None or gps.lon is None:
        return
    if not (-90.0 <= float(gps.lat) <= 90.0 and -180.0 <= float(gps.lon) <= 180.0):
        return
    _CACHE.start_lat = float(gps.lat)
    _CACHE.start_lon = float(gps.lon)
    _START_POINT_LOCKED = True
    prevstate.update_start_point(float(gps.lat), float(gps.lon), True)

def handle_gps(data: str) -> None:
    """lat,lon,course_deg,groundSpeed_mps,posHealth,motionHealth[,sample_ts]"""
    fields = data.split(",")
    if len(fields) not in (6, 7):
        LOGGER.warning("GNSS parse: expected 6 or 7 fields | raw=%r", data)
        return
    try:
        lat = float(fields[0])
        lon = float(fields[1])
        course_deg = float(fields[2])
        ground_speed = float(fields[3])
        pos_health = bool(int(float(fields[4])))
        motion_health = bool(int(float(fields[5])))
        rx_ts = time.monotonic()
        sample_ts = float(fields[6]) if len(fields) == 7 else rx_ts
    except (ValueError, IndexError) as exc:
        LOGGER.warning("GNSS parse error: %s | raw=%r", exc, data)
        return

    course_rad = math.radians(course_deg)
    sample = _GpsFromApp(
        lat=lat,
        lon=lon,
        course_rad=course_rad,
        speed_mps=ground_speed,
        sample_ts=sample_ts,
        rx_ts=rx_ts,
        pos_ts=sample_ts,
        motion_ts=sample_ts,
        pos_health=pos_health,
        motion_health=motion_health,
    )
    with _UPDATE_LOCK:
        _CACHE.latest_gps = sample
        _CACHE.gps_history.append(sample)
        if pos_health:
            _CACHE.last_gps.lat = lat
            _CACHE.last_gps.lon = lon
            _CACHE.last_gps.sample_ts = sample_ts
            _CACHE.last_gps.rx_ts = rx_ts
            _CACHE.last_gps.pos_ts = sample_ts
            _CACHE.last_gps.pos_health = True
        if motion_health:
            _CACHE.last_gps.course_rad = course_rad
            _CACHE.last_gps.speed_mps = ground_speed
            _CACHE.last_gps.sample_ts = sample_ts
            _CACHE.last_gps.rx_ts = rx_ts
            _CACHE.last_gps.motion_ts = sample_ts
            _CACHE.last_gps.motion_health = True
        _lock_start_if_ready()


handle_gつい = handle_gps
handle_g勾中 = handle_gps
globals()["handle_g\u1166\u1102"] = handle_gps


def handle_imu(data: str) -> None:
    """roll,pitch,yaw,accx,accy,accz,magx,magy,magz,gyrx,gyry,gyrz_deg_s,health[,sample_ts]"""
    fields = data.split(",")
    try:
        if len(fields) not in (13, 14):
            LOGGER.warning("IMU parse: expected 13 or 14 fields, got %d | raw=%r", len(fields), data)
            return
        gyrz_deg_s = float(fields[11])
        health = bool(int(float(fields[12])))
        rx_ts = time.monotonic()
        sample_ts = float(fields[13]) if len(fields) == 14 else rx_ts
    except (ValueError, IndexError) as exc:
        LOGGER.warning("IMU parse error: %s | raw=%r", exc, data)
        return

    gyrz_rad_s = math.radians(gyrz_deg_s)
    sample = _ImuFromApp(
        gyrz_rad_s=gyrz_rad_s,
        sample_ts=sample_ts,
        rx_ts=rx_ts,
        ts=sample_ts,
        health=health,
    )
    with _UPDATE_LOCK:
        _CACHE.latest_imu = sample
        _CACHE.imu_history.append(sample)
        if health:
            _CACHE.last_imu = sample


def handle_barometer(data: str) -> None:
    """altitude_m[,health[,sample_ts]]"""
    fields = data.split(",")
    try:
        alt_m = float(fields[0].strip())
        health = bool(int(float(fields[1]))) if len(fields) >= 2 else True
        rx_ts = time.monotonic()
        sample_ts = float(fields[2]) if len(fields) >= 3 else rx_ts
    except (ValueError, IndexError) as exc:
        LOGGER.warning("Baro parse error: %s | raw=%r", exc, data)
        return

    sample = _BaroFromApp(
        alt_m=alt_m,
        sample_ts=sample_ts,
        rx_ts=rx_ts,
        ts=sample_ts,
        health=health,
    )
    with _UPDATE_LOCK:
        _CACHE.latest_baro = sample
        _CACHE.baro_history.append(sample)
        if health:
            _CACHE.last_baro = sample


def handle_target_coord(data: str) -> None:
    """lat,lon"""
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
        _CACHE.target_lat = lat
        _CACHE.target_lon = lon
    LOGGER.info("Target updated: %.6f, %.6f", lat, lon)


def handle_flight_state(data: str) -> None:
    global STATE, _PREV_STATE, _START_POINT_LOCKED, _L1_STATE, _CONTROLLER
    try:
        new_state = int(data.split(",")[0])
    except (ValueError, IndexError) as exc:
        LOGGER.warning("State parse error: %s | raw=%r", exc, data)
        return
    if new_state == STATE:
        return

    LOGGER.info("State %d -> %d", STATE, new_state)
    with _UPDATE_LOCK:
        _PREV_STATE = STATE
        STATE = new_state
        if new_state < 3:
            _CACHE.start_lat = None
            _CACHE.start_lon = None
            _START_POINT_LOCKED = False
            prevstate.clear_start_point()
            gmod = _guidance()
            if gmod is not None and _L1_STATE is not None and hasattr(gmod, "l1_reset"):
                try:
                    gmod.l1_reset(_L1_STATE)
                except Exception:
                    LOGGER.debug("Failed to reset L1 state", exc_info=True)
            ctl = _control()
            if ctl is not None and _CONTROLLER is not None and hasattr(ctl, "controller_reset"):
                ctl.controller_reset(_CONTROLLER)
        elif new_state in (3, 4):
            _lock_start_if_ready()


def handle_release(data: str = "TRIGGER") -> None:
    mod = _load_module("Sensor_Motor.Motor_Release")
    if mod is None or not hasattr(mod, "activate_burnwire"):
        LOGGER.error("Burnwire module unavailable")
        return
    reason = data.split(":", 1)[1] if isinstance(data, str) and ":" in data else str(data or "UNKNOWN")
    LOGGER.warning("Burnwire trigger received | reason=%s", reason)
    threading.Thread(target=mod.activate_burnwire, daemon=True, name="Burnwire").start()


def handle_egg_drop() -> None:
    mod = _load_module("Sensor_Motor.Motor_Egg")
    if mod is None or not hasattr(mod, "activate_solenoid"):
        LOGGER.error("Egg module unavailable")
        return
    threading.Thread(target=mod.activate_solenoid, daemon=True, name="Solenoid").start()


def handle_mec(data: str) -> None:
    global MOTOR_ENABLED
    cmd = data.strip().upper()
    if cmd == "ON":
        MOTOR_ENABLED = True
        prevstate.update_motor_enabled(True)
        LOGGER.info("MOTOR_ENABLED = True")
    elif cmd == "OFF":
        MOTOR_ENABLED = False
        prevstate.update_motor_enabled(False)
        with _UPDATE_LOCK:
            _set_neutral()
        LOGGER.info("MOTOR_ENABLED = False -> neutral")
    else:
        LOGGER.warning("Unknown MEC command: %r", data)


def _send_diag(main_queue, cmd, g_out, diag_state: str) -> None:
    if main_queue is None:
        return

    def _fmt(v) -> str:
        try:
            f = float(v)
            return "nan" if f != f else f"{f:.4f}"
        except (TypeError, ValueError):
            return "nan"

    def _fmt_ll(v) -> str:
        try:
            f = float(v)
            return "nan" if f != f else f"{f:.6f}"
        except (TypeError, ValueError):
            return "nan"

    def _fmt_hdg(v) -> str:
        try:
            f = float(v)
            return "nan" if f != f else f"{f:.2f}"
        except (TypeError, ValueError):
            return "nan"

    with _UPDATE_LOCK:
        target_lat = _CACHE.target_lat
        target_lon = _CACHE.target_lon
        origin_lat = _CACHE.start_lat
        origin_lon = _CACHE.start_lon

    head = [
        str(getattr(cmd, "left_pw", 0)),
        str(getattr(cmd, "right_pw", 0)),
        _fmt_ll(origin_lat),
        _fmt_ll(origin_lon),
        _fmt_ll(getattr(g_out, "target_lat", target_lat)),
        _fmt_ll(getattr(g_out, "target_lon", target_lon)),
        _fmt_ll(getattr(g_out, "carrot_lat", float("nan"))),
        _fmt_ll(getattr(g_out, "carrot_lon", float("nan"))),
        _fmt_hdg(getattr(g_out, "current_heading_deg", float("nan"))),
        _fmt_hdg(getattr(g_out, "desired_heading_deg", float("nan"))),
        diag_state,
    ]
    tail = [
        _fmt(getattr(g_out, "crossTrack", float("nan"))),
        _fmt(getattr(g_out, "alongTrack", float("nan"))),
        _fmt(getattr(cmd, "yaw_rate_cmd_deg_s", 0.0)),
        _fmt(getattr(cmd, "yaw_rate_meas_deg_s", float("nan"))),
        _fmt(getattr(cmd, "yaw_rate_error_deg_s", 0.0)),
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
    msgstructure.send_msg(
        main_queue,
        appargs.MotorAppArg.AppID,
        appargs.CommAppArg.AppID,
        appargs.MotorAppArg.MID_comm_motor_diag,
        ",".join(head + tail),
    )


def ctrl_paragldr(main_queue=None) -> None:
    """Parafoil control loop."""
    period = _motor_period_sec()
    while MOTORAPP_RUNSTATUS:
        now = time.time()
        try:
            if not MOTOR_ENABLED or STATE < 3:
                _set_neutral()
                _send_diag(main_queue, _null_brake_command(now, "IDLE"), _null_guidance_output(now, "IDLE"), "IDLE")
                time.sleep(period)
                continue

            if STATE == 5:
                _set_motors_off()
                _send_diag(main_queue, _null_brake_command(now, "LANDED"), _null_guidance_output(now, "LANDED"), "LANDED")
                time.sleep(period)
                continue

            with _UPDATE_LOCK:
                snap = _cache_snapshot()

            gmod = _guidance()
            if gmod is None:
                cmd = _null_brake_command(now, "GUIDANCE_UNAVAILABLE")
                g_out = _null_guidance_output(now, "GUIDANCE_UNAVAILABLE")
            else:
                l1_input, _mode, reason = _produce_l1_input(gmod, snap, now)
                if l1_input is None:
                    cmd = _null_brake_command(now, reason or "INPUT_UNAVAILABLE")
                    g_out = _null_guidance_output(now, reason or "INPUT_UNAVAILABLE")
                else:
                    g_out = _produce_l1_output(gmod, l1_input, snap, now)
                    if bool(getattr(g_out, "active", False)):
                        cmd = _controller_update(g_out, l1_input, now)
                    else:
                        cmd = _null_brake_command(now, getattr(g_out, "reason", "SAFE_GLIDE") or "SAFE_GLIDE")

            _set_brake_command(cmd)
            diag_state = (
                "DEGRADED" if bool(getattr(g_out, "active", False)) and bool(getattr(g_out, "degraded", False))
                else "ACTIVE" if bool(getattr(g_out, "active", False))
                else str(getattr(g_out, "reason", "DISABLED") or "DISABLED")
            )
            _send_diag(main_queue, cmd, g_out, diag_state)

        except Exception as exc:
            LOGGER.error("ctrl_paragldr exception: %s", exc, exc_info=True)
            _set_neutral()
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
        handle_gㅔㄴ(unpacked.data)
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


def init() -> None:
    global PI, MOTOR_ENABLED, _START_POINT_LOCKED, _CONTROLLER, _L1_STATE
    prevstate.init_prevstate()
    MOTOR_ENABLED = prevstate.is_motor_enabled()

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

    gmod = _guidance()
    if gmod is not None and hasattr(gmod, "make_l1_state"):
        try:
            _L1_STATE = gmod.make_l1_state()
        except Exception:
            LOGGER.debug("Failed to create L1 state", exc_info=True)

    ctl = _control()
    if ctl is not None:
        if hasattr(ctl, "make_controller_state"):
            _CONTROLLER = ctl.make_controller_state()
        if hasattr(ctl, "init_control"):
            PI = ctl.init_control()

    for module_name, init_name in (
        ("Sensor_Motor.Motor_Release", "init_burnwire"),
        ("Sensor_Motor.Motor_Egg", "init_solenoid"),
    ):
        mod = _load_module(module_name)
        if mod is not None and hasattr(mod, init_name):
            getattr(mod, init_name)()

    LOGGER.info(
        "MotorApp init | pigpio=%s | motor_enabled=%s | start_locked=%s",
        getattr(PI, "connected", "N/A"),
        MOTOR_ENABLED,
        _START_POINT_LOCKED,
    )


def motorapp_main(main_queue, main_pipe=None) -> None:
    if main_pipe is None:
        main_pipe = main_queue
        main_queue = None
    init()

    ctrl_thread = threading.Thread(
        target=ctrl_paragldr,
        args=(main_queue,),
        daemon=True,
        name="MotorControlLoop",
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
