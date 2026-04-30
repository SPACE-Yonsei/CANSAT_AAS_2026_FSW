import math
import os
import signal
import threading
import time
import types
from datetime import datetime
from multiprocessing import connection
from typing import Optional
from lib import appargs, msgstructure, events
from Sensor_Motor import motor_guidance, motor_control, Motor_Release, Motor_Egg
from Sensor_Motor.motor_logger import MotorLogger

running: bool = True
motor_enabled: bool = True
pi = None
logger: Optional[MotorLogger] = None

target = types.SimpleNamespace(
    lat  = None,  # Optional[float] — deg, decimal degrees
    lon  = None,  # Optional[float] — deg, decimal degrees
)

altitude = types.SimpleNamespace(
    yaw     = None,   # Optional[float] — deg, 0-360
    gyrz    = None,   # Optional[float] — rad/s (pre-filtered in imu.py)
    healthy = False,  # bool — False when IMU reports stale/fault
)
baro_m: Optional[float] = None  # m, 기압계 고도

GpsVector = types.SimpleNamespace(
    lat    = None,  # Optional[float] — deg, decimal degrees
    lon    = None,  # Optional[float] — deg, decimal degrees
    speed  = None,  # Optional[float] — m/s
    course = None,  # Optional[float] — deg, 0-360
)
GpsFidelity = types.SimpleNamespace(
    rmc_status    = None,  # Optional[str]  — "A"(active) / "V"(void)
    fix_quality   = None,  # Optional[int]  — 0=no fix, 1=GPS, 2=DGPS
    sats          = None,  # Optional[int]  — 위성 수
    jump_rejected = True,  # bool — True until is_gps_jump() clears it
)

state: int = 0
_start_point_locked: bool = False  # True once a valid-GPS start_point is committed

GYRZ_RUNAWAY_THRESHOLD: float = math.radians(100.0) # rad/s (=100°/s), 제어 불능 판정

GPS_STALE_TIMEOUT: float       = 10.0   # s — drop test max valid gap was 9.0 s
GPS_MAX_PLAUSIBLE_SPEED: float = 15.0   # m/s — parafoil physical airspeed ceiling
_gps_last_received_time: float = 0.0    # epoch, 0 = never received
_last_valid_gps_speed: float   = 1.0    # m/s, hold-last on implausible GPS speed
FORCE_GPS_SPEED_MPS: Optional[float] = None  # e.g. 4.0 to force fixed speed for debugging
_force_speed_logged: bool = False

threads: dict[str, threading.Thread] = {}
update_lock = threading.Lock()
CONTROL_LOG_INTERVAL = 0.1
DEBUG_GUIDANCE = True  # guidance 디버그 프린트 on/off

APP = appargs.MotorAppArg.AppName


def log(msg: str, level=events.EventType.info):
    events.LogEvent(APP, level, msg)


def handle_terminate(data: str):
    global running
    log("Termination detected")
    running = False


def handle_gps(data: str):
    global _start_point_locked, _gps_last_received_time, _last_valid_gps_speed, _force_speed_logged
    parts = data.split(",")
    if len(parts) != 7:
        log(f"GPS data format error: expected 7 fields, got {len(parts)}", events.EventType.error)
        return
    try:
        new_lat    = float(parts[0])
        new_lon    = float(parts[1])
        new_speed  = float(parts[2])
        new_course = float(parts[3])
        new_fix    = int(parts[4])
        new_sats   = int(parts[5])
        new_rmc    = parts[6].strip()
    except (ValueError, IndexError) as e:
        log(f"GPS parse error: {e} raw={data!r}", events.EventType.error)
        return
    if not (math.isfinite(new_lat) and math.isfinite(new_lon)
            and math.isfinite(new_speed) and math.isfinite(new_course)):
        log(f"GPS NaN/Inf rejected: lat={new_lat} lon={new_lon} spd={new_speed} crs={new_course}",
            events.EventType.error)
        return
    # GPS speed plausibility gate: parafoil cannot exceed GPS_MAX_PLAUSIBLE_SPEED
    if new_speed > GPS_MAX_PLAUSIBLE_SPEED:
        log(f"GPS speed implausible ({new_speed:.1f} m/s > {GPS_MAX_PLAUSIBLE_SPEED} m/s), holding last={_last_valid_gps_speed:.1f} m/s",
            events.EventType.warning)
        new_speed = _last_valid_gps_speed
    else:
        _last_valid_gps_speed = new_speed
    if FORCE_GPS_SPEED_MPS is not None:
        new_speed = FORCE_GPS_SPEED_MPS
        if not _force_speed_logged:
            log(f"DEBUG: forcing GPS speed to fixed {FORCE_GPS_SPEED_MPS:.2f} m/s", events.EventType.warning)
            _force_speed_logged = True
    _gps_last_received_time = time.time()
    # Evaluate GPS jump before acquiring the lock (pure computation, no shared state write)
    new_jump_rejected = motor_guidance.is_gps_jump(new_lat, new_lon)
    with update_lock:
        GpsVector.lat    = new_lat
        GpsVector.lon    = new_lon
        GpsVector.speed  = new_speed
        GpsVector.course = new_course
        GpsFidelity.fix_quality   = new_fix
        GpsFidelity.sats          = new_sats
        GpsFidelity.rmc_status    = new_rmc
        GpsFidelity.jump_rejected = new_jump_rejected
        # state=3 진입 후 유효 GPS가 처음 도착하면 start_point 확정
        if state == 3 and not _start_point_locked and motor_guidance.is_gps_valid(GpsVector, GpsFidelity):
            motor_guidance.set_start_coordinates(GpsVector.lat, GpsVector.lon)
            _start_point_locked = True
            log(f"start_point confirmed: ({GpsVector.lat:.6f}, {GpsVector.lon:.6f}) "
                f"fix={GpsFidelity.fix_quality} sats={GpsFidelity.sats}")


def handle_imu(data: str):
    parts = data.split(",")
    if len(parts) != 3:
        log(f"IMU data format error: expected 3 fields, got {len(parts)}", events.EventType.error)
        return
    try:
        new_yaw    = float(parts[0])
        new_gyrz   = float(parts[1])
        new_health = parts[2].strip() == "1"
    except (ValueError, IndexError) as e:
        log(f"IMU parse error: {e} raw={data!r}", events.EventType.error)
        return
    if not (math.isfinite(new_yaw) and math.isfinite(new_gyrz)):
        log(f"IMU NaN/Inf rejected: yaw={new_yaw} gyrz={new_gyrz}", events.EventType.error)
        return
    with update_lock:
        altitude.yaw     = new_yaw
        altitude.gyrz    = new_gyrz   # already spike-filtered + EMA'd in imu.py
        altitude.healthy = new_health


def handle_barometer(data: str):
    global baro_m
    try:
        parts = data.split(",")
        with update_lock:
            baro_m = float(parts[0])
    except (ValueError, IndexError):
        log("Barometer data format error", events.EventType.error)


def handle_target_coord(data: str):
    parts = data.split(",")
    if len(parts) != 2:
        log(f"Target coords format error: expected 2 fields, got {len(parts)}", events.EventType.error)
        return
    try:
        new_lat = float(parts[0])
        new_lon = float(parts[1])
    except (ValueError, IndexError) as e:
        log(f"Target coord parse error: {e} raw={data!r}", events.EventType.error)
        return
    if not (math.isfinite(new_lat) and math.isfinite(new_lon)):
        log(f"Target NaN/Inf rejected: lat={new_lat} lon={new_lon}", events.EventType.error)
        return
    if not (-90.0 <= new_lat <= 90.0 and -180.0 <= new_lon <= 180.0):
        log(f"Target coord out of range: lat={new_lat} lon={new_lon}", events.EventType.error)
        return
    with update_lock:
        target.lat, target.lon = new_lat, new_lon
    motor_guidance.set_target_coord(new_lat, new_lon)
    log(f"Target set: ({new_lat:.6f}, {new_lon:.6f})")


def handle_flight_state(data: str):
    global state, _start_point_locked
    try:
        new_state = int(data.strip())
    except (ValueError, AttributeError) as e:
        log(f"Flight state parse error: {e} raw={data!r}", events.EventType.error)
        return
    if not (0 <= new_state <= 5):
        log(f"Flight state out of range: {new_state}", events.EventType.error)
        return
    with update_lock:
        state = new_state
        if state == 3:
            _start_point_locked = False  # 새 state=3 진입마다 재확정 허용
            if motor_guidance.is_gps_valid(GpsVector, GpsFidelity):
                motor_guidance.set_start_coordinates(GpsVector.lat, GpsVector.lon)
                _start_point_locked = True
                log(f"start_point set: ({GpsVector.lat:.6f}, {GpsVector.lon:.6f}) "
                    f"fix={GpsFidelity.fix_quality} sats={GpsFidelity.sats} rmc={GpsFidelity.rmc_status}")
            else:
                log("state=3: GPS not valid yet, waiting for first valid fix to set start_point",
                    events.EventType.warning)
            if target.lat is None or target.lon is None:
                log("CRITICAL: state=3 (RELEASE) but target coordinates not set — "
                    "GNC will failsafe. Send 'CMD,XXXX,TC,lat,lon' or set prevstate.txt TARGET_LAT/LON",
                    events.EventType.error)
    log(f"Flight state: {state}")


def handle_release():
    log("Activating burnwire")
    Motor_Release.activate_burnwire()


def handle_egg_drop():
    log("Activating solenoid")
    Motor_Egg.activate_solenoid()


def handle_mec(data: str):
    global motor_enabled
    log(f"MEC command: {data}")
    with update_lock:
        if data == "ON":
            motor_enabled = True
        elif data == "OFF":
            motor_enabled = False
        else:
            log(f"Invalid MEC option: {data}", events.EventType.error)


MSG_HANDLERS = {
    appargs.MainAppArg.MID_TerminateProcess:       handle_terminate,
    appargs.GpsAppArg.MID_motor_gps:               handle_gps,
    appargs.ImuAppArg.MID_motor_imu:               handle_imu,
    appargs.BarometerAppArg.MID_motor_alt:         handle_barometer,
    appargs.FlightlogicAppArg.MID_motor_TargetCor: handle_target_coord,
    appargs.FlightlogicAppArg.MID_motor_state:     handle_flight_state,
    appargs.FlightlogicAppArg.MID_motor_burnwire:  lambda d: handle_release(),
    appargs.FlightlogicAppArg.MID_motor_EggDrop:   lambda d: handle_egg_drop(),
    appargs.CommAppArg.MID_RouteCmd_MEC:           handle_mec,
}


def dispatch(msg: msgstructure.MsgStructure):
    handler = MSG_HANDLERS.get(msg.MsgID)
    if handler:
        handler(msg.data)
    else:
        log(f"Unknown MID: {msg.MsgID}", events.EventType.error)



def _snapshot_sensors() -> types.SimpleNamespace:
    with update_lock:
        return types.SimpleNamespace(
            state         = state,
            motor_enabled = motor_enabled,
            baro_m        = baro_m,
            gps           = types.SimpleNamespace(
                lat=GpsVector.lat, lon=GpsVector.lon,
                speed=GpsVector.speed, course=GpsVector.course),
            gps_fidelity  = types.SimpleNamespace(
                rmc_status=GpsFidelity.rmc_status,
                fix_quality=GpsFidelity.fix_quality,
                sats=GpsFidelity.sats,
                jump_rejected=GpsFidelity.jump_rejected),
            imu           = types.SimpleNamespace(
                yaw=altitude.yaw, gyrz=altitude.gyrz,
                healthy=altitude.healthy),
            target        = types.SimpleNamespace(
                lat=target.lat, lon=target.lon),
        )


def _check_fdir(snap: types.SimpleNamespace) -> Optional[str]:
    # FDIR-0: 센서 미수신 / NaN·Inf
    gps_missing  = (snap.gps.lat is None or snap.gps.lon is None
                    or not math.isfinite(snap.gps.lat) or not math.isfinite(snap.gps.lon))
    imu_missing  = (snap.imu.yaw is None or snap.imu.gyrz is None
                    or not math.isfinite(snap.imu.yaw) or not math.isfinite(snap.imu.gyrz))
    baro_missing = (snap.baro_m is None or not math.isfinite(snap.baro_m))
    if gps_missing or imu_missing or baro_missing:
        missing = (["GPS"] if gps_missing else []) + \
                  (["IMU"] if imu_missing else []) + \
                  (["BARO"] if baro_missing else [])
        return f"No data received: {'+'.join(missing)}"

    # FDIR-1: IMU 센서-보고 건강 상태 (imu.py의 read_imu_data 루프가 판정)
    if not snap.imu.healthy:
        return "IMU stale (sensor-reported)"

    # FDIR-2: GPS 수신 freshness (마지막 수신 후 GPS_STALE_TIMEOUT 초 초과)
    if _gps_last_received_time > 0:
        gps_age = time.time() - _gps_last_received_time
        if gps_age > GPS_STALE_TIMEOUT:
            return f"GPS stale ({gps_age:.1f}s since last fix, limit={GPS_STALE_TIMEOUT}s)"

    # FDIR-2b: GPS 무결성
    if not motor_guidance.is_gps_valid(snap.gps, snap.gps_fidelity):
        return (f"GPS invalid (lat={snap.gps.lat}, lon={snap.gps.lon}, "
                f"fix={snap.gps_fidelity.fix_quality}, sats={snap.gps_fidelity.sats}, "
                f"rmc={snap.gps_fidelity.rmc_status})")

    # FDIR-2c: GPS 순간 이동 거부
    if snap.gps_fidelity.jump_rejected:
        return (f"GPS jump rejected (lat={snap.gps.lat:.6f}, lon={snap.gps.lon:.6f})")

    # FDIR-3: 극한 회전
    if abs(snap.imu.gyrz) > GYRZ_RUNAWAY_THRESHOLD:
        return (f"|gyrz|={abs(snap.imu.gyrz):.2f} rad/s "
                f"({math.degrees(abs(snap.imu.gyrz)):.1f}°/s) > "
                f"{GYRZ_RUNAWAY_THRESHOLD:.2f} rad/s")

    # FDIR-4: 기압계 고도
    if snap.baro_m <= 0.0:
        return f"Baro altitude invalid ({snap.baro_m:.1f}m)"

    # FDIR-5: 목표 좌표 미수신
    if snap.target.lat is None or snap.target.lon is None:
        return "No target coordinates received"

    return None


def ctrl_paragldr():
    motors_off        = False
    _fdir_last_reason = None
    _fdir_repeat_count = 0
    _ctrl_tick        = 0

    while running:
        try:
            snap = _snapshot_sensors()

            if snap.state >= 3 and snap.motor_enabled:

                if snap.state == 5:
                    if not motors_off:
                        motor_control.set_motors_off(pi)
                        motors_off = True
                        log("State 5: motors off", events.EventType.warning)
                    time.sleep(CONTROL_LOG_INTERVAL)
                    continue

                motors_off = False

                reason = _check_fdir(snap)

                if reason is not None:
                    if reason != _fdir_last_reason:
                        _fdir_last_reason  = reason
                        _fdir_repeat_count = 1
                        log(f"Failsafe: {reason}", events.EventType.error)
                    else:
                        _fdir_repeat_count += 1
                        if _fdir_repeat_count % 10 == 0:
                            log(f"Failsafe x{_fdir_repeat_count}: {reason}", events.EventType.error)
                    motor_control.set_neutral(pi)
                else:
                    if DEBUG_GUIDANCE:
                        logger.guidance_in(snap.state, snap.baro_m, snap.imu, snap.gps, snap.gps_fidelity, snap.target)
                    result       = motor_guidance.guidance(snap.imu, snap.gps, snap.gps_fidelity, snap.target, baro_m=snap.baro_m)
                    motor_result = motor_control.control(pi, result.commanded_yaw_rate)
                    if DEBUG_GUIDANCE and motor_result is not None:
                        logger.motor_out(motor_result)
                    logger.control(result, motor_result)

            _ctrl_tick += 1
            if _ctrl_tick % 10 == 0:
                log(f"[HB] tick={_ctrl_tick} state={snap.state} baro={snap.baro_m} motor={snap.motor_enabled}")

            time.sleep(CONTROL_LOG_INTERVAL)

        except Exception as e:
            log(f"ctrl_paragldr CRASHED: {e}", events.EventType.error)
            try:
                motor_control.set_neutral(pi)
            except Exception:
                pass
            time.sleep(CONTROL_LOG_INTERVAL)


def init() -> bool:
    global pi, running, logger

    signal.signal(signal.SIGINT, signal.SIG_IGN)
    log("Initializing motorapp")

    try:
        logger = MotorLogger(
            sim_log_path=os.getenv("CANSAT_SIM_LOG", datetime.now().strftime("%m%d_sim.txt")),
        )
        motor_guidance.init_guidance(logger)
        pi = motor_control.init_control(logger)
        # RPi.GPIO 릴레이 핀은 pigpio(pi) 연결 이후에 설정 (초기화 순서로 레벨이 흔들리는 것 방지)
        Motor_Release.init_burnwire()
        Motor_Egg.init_solenoid()
        threads["ControlLog_Thread"] = threading.Thread(
            target=ctrl_paragldr,
            name="ControlLog_Thread",
            daemon=True
        )
        threads["ControlLog_Thread"].start()
        log("Motors initialized (parafoil, burnwire, solenoid)")
        return True
    except Exception as e:
        log(f"Init failed: {e}", events.EventType.error)
        running = False
        return False


def terminate():
    global running
    running = False
    log("Terminating motorapp")

    if logger:
        logger.close()

    if pi:
        motor_control.terminate_parafoil_motor(pi)

    Motor_Release.terminate_burnwire()
    Motor_Egg.terminate_solenoid()

    for name, thread in threads.items():
        log(f"Joining thread: {name}")
        thread.join()

    log("Motorapp terminated")


def motorapp_main(main_pipe: connection.Connection):
    global running
    running = True

    if not init():
        return

    try:
        while running:
            recv_msg = main_pipe.recv()
            unpacked_msg = msgstructure.unpack_msg(recv_msg)

            if unpacked_msg == False:
                continue

            if unpacked_msg.receiver_app in (appargs.MotorAppArg.AppID,
                                              appargs.MainAppArg.AppID):
                dispatch(unpacked_msg)

    except Exception as e:
        log(f"Error: {e}", events.EventType.error)

    finally:
        terminate()
