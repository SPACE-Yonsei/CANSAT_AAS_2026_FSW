from collections import deque
from dataclasses import dataclass, field
import logging
import math
import threading
import time
from typing import Optional

from lib import appargs, config, msgstructure, prevstate, sensorlog

logger = logging.getLogger(__name__)

from . import control, guidance

_TARGET_LAT: Optional[float] = None
_TARGET_LON: Optional[float] = None
_START_LAT:  Optional[float] = None
_START_LON:  Optional[float] = None

MOTORAPP_RUNSTATUS: bool = True

_CACHE = _Cache()                 # 최신 raw 센서 데이터
_GUIDANCE_STATE = GuidanceState() # DR 상태, origin, target

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
class _Cache:
    """매 사이클 교체되는 센서 데이터만 보관. 임무 상수(target/start)는 별도 전역."""
    latest_gps:  _GpsFromApp  = field(default_factory=_GpsFromApp)
    latest_imu:  _ImuFromApp  = field(default_factory=_ImuFromApp)
    latest_baro: _BaroFromApp = field(default_factory=_BaroFromApp)

_TARGET_LAT: Optional[float] = None
_TARGET_LON: Optional[float] = None
_START_LAT:  Optional[float] = None
_START_LON:  Optional[float] = None

_ORIGIN_SAVED: bool = False

RELEASE_ACTION_ENABLED: bool = True
EGG_ACTION_ENABLED: bool = True

PI = None

_CACHE = _Cache()

def _cache_snapshot() -> _Cache:
    return _Cache(
        latest_gps=_GpsFromApp(**vars(_CACHE.latest_gps)),
        latest_imu=_ImuFromApp(**vars(_CACHE.latest_imu)),
        latest_baro=_BaroFromApp(**vars(_CACHE.latest_baro)),
    )

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
        _CACHE.latest_imu = imu


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
        _CACHE.latest_baro = baro


def handle_target_coord(data: str) -> None:
    """Target lat,lon — single source of truth is _GUIDANCE_STATE.

    Rejects out-of-range coords and the (0,0) sentinel (matches init()).
    """
    global _TARGET_LAT, _TARGET_LON
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
        _TARGET_LAT = lat
        _TARGET_LON = lon
        _GUIDANCE_STATE.target_lat = lat
        _GUIDANCE_STATE.target_lon = lon
        _GUIDANCE_STATE.target_ready = False  # trigger re-projection next cycle


def handle_flight_state(data: str) -> None:
    """Update flight state. Origin acquisition stays in guidance pipeline."""
    global STATE, _PREV_STATE, _CONTROLLER, _ORIGIN_SAVED, _START_LAT, _START_LON
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
            _START_LAT = None
            _START_LON = None
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


def _ctrl_cycle(main_queue, now: float) -> Optional[control.CtrlOutput]:
    global _ORIGIN_SAVED
    try:
        with _UPDATE_LOCK:
            motor_enabled = MOTOR_ENABLED
            state         = STATE
            snap          = _cache_snapshot()

        if not motor_enabled or state < 3:
            if PI is not None:
                control.WriteZero(PI)
            return None

        if state == 5:
            if PI is not None:
                control.WriteOff(PI)
            return None
        decided_snap = guidance.DecideFresh(snap)
        control_mode = guidance.DecideControlMode(decided_snap)
        if control_mode guidance.ControlMode.DETUMBLING:
            contorl.
        elif control_mode in (guidance.ControlMode.GPS_TRACKING_CLOSED
                              , guidance.ControlMode.GPS_TRACKING_OPEN
                              , guidance.ControlMode.DR_TRACKING_CLOSED
                              , guidance.ControlMode.DR_TRACKING_OPEN):
        
            L1_input = guidance.ProduceL1Input()
            L1_output = guidance.produceL1output()
            ctrl_input = control.ProduceCtrlInput(L1_output, control_mode)
            ctrl_output = control.ProduceCtrlOutput(ctrl_input)
            control.MoveServo(ctrl_output)
        elif control_mode in (guidance.ControlMode.FAIL,):
            if PI is not None:
                control.WriteOff(PI)
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