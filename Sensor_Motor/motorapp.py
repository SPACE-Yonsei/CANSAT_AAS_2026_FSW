import math
import os
import signal
import threading
import time
import types
from datetime import datetime
from multiprocessing import connection
from lib import appargs, msgstructure, events
from Sensor_Motor import motor_guidance, motor_control, Motor_Release, Motor_Egg


log_dir = "./sensorlogs"
if not os.path.exists(log_dir):
    os.makedirs(log_dir)
controllogfile = open(os.path.join(log_dir, "control.txt"), "a")
simlogfile = open("0320_sim.txt", "a")

def _dbg(line: str):
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    full = f"[{ts}] {line}"
    print(full)
    simlogfile.write(full + "\n")
    simlogfile.flush()

def log_control(g, m):
    t = datetime.now().isoformat(sep=" ", timespec="milliseconds")

    if m is None:
        m = types.SimpleNamespace(
            left_cmd_deg=0.0, right_cmd_deg=0.0,
            actual_delta_deg=0.0, expected_yaw_rate=0.0,
            left_pulse=0, right_pulse=0
        )

    # g 데이터는 state, distance, commanded_yaw_rate 3개만 존재함
    line = (
        f"{t},"
        f"state:{g.state},"
        f"dist:{g.distance:.2f},"
        f"cmd_yr:{g.commanded_yaw_rate:.2f},"
        f"L_deg:{m.left_cmd_deg:.1f},"
        f"R_deg:{m.right_cmd_deg:.1f},"
        f"L_pw:{m.left_pulse},"
        f"R_pw:{m.right_pulse}\n"
    )
    controllogfile.write(line)
    controllogfile.flush()

from typing import Optional

running: bool = True
motor_enabled: bool = True
pi = None

target = types.SimpleNamespace(
    lat  = None,  # Optional[float] — deg, decimal degrees
    lon  = None,  # Optional[float] — deg, decimal degrees
)

altitude = types.SimpleNamespace(
    yaw  = None,  # Optional[float] — deg, 0-360
    gyrz = None,  # Optional[float] — rad/s
)
baro_m: Optional[float] = None  # m, 기압계 고도

GpsVector = types.SimpleNamespace(
    lat    = None,  # Optional[float] — deg, decimal degrees
    lon    = None,  # Optional[float] — deg, decimal degrees
    speed  = None,  # Optional[float] — m/s
    course = None,  # Optional[float] — deg, 0-360
)
GpsFidelity = types.SimpleNamespace(
    rmc_status   = None,  # Optional[str]  — "A"(active) / "V"(void)
    fix_quality  = None,  # Optional[int]  — 0=no fix, 1=GPS, 2=DGPS
    sats         = None,  # Optional[int]  — 위성 수
)

state: int = 0

last_gps_time: Optional[float] = None  # s, time.time() epoch
last_imu_time: Optional[float] = None  # s, time.time() epoch
STALE_THRESHOLD: float = 1.5           # s, 이상 갱신 없으면 stale 판정

GYRZ_SPIKE_THRESHOLD: float = math.radians(45.0)   # rad/s (=45°/s), 틱 간 최대 허용 델타
_prev_gyrz: Optional[float] = None                  # rad/s

GYRZ_RUNAWAY_THRESHOLD: float = math.radians(100.0) # rad/s (=100°/s), 제어 불능 판정

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
    global last_gps_time
    parts = data.split(",")
    if len(parts) == 7:
        with update_lock:
            GpsVector.lat    = float(parts[0])
            GpsVector.lon    = float(parts[1])
            GpsVector.speed  = float(parts[2])
            GpsVector.course = float(parts[3])
            GpsFidelity.fix_quality = int(parts[4])
            GpsFidelity.sats        = int(parts[5])
            GpsFidelity.rmc_status  = parts[6]
            last_gps_time = time.time()
    else:
        log("GPS data format error", events.EventType.error)


def handle_imu(data: str):
    global last_imu_time, _prev_gyrz
    parts = data.split(",")
    if len(parts) == 2:
        with update_lock:
            new_yaw  = float(parts[0])
            new_gyrz = float(parts[1])
            if _prev_gyrz is None or abs(new_gyrz - _prev_gyrz) <= GYRZ_SPIKE_THRESHOLD:
                altitude.gyrz = new_gyrz
                _prev_gyrz = new_gyrz
            else:
                log(f"gyrz spike rejected: {new_gyrz:.2f} rad/s (prev={_prev_gyrz:.2f})",
                    events.EventType.warning)
            altitude.yaw  = new_yaw
            last_imu_time = time.time()
    else:
        log("IMU data format error", events.EventType.error)


def handle_barometer(data: str):
    global baro_m
    try:
        parts = data.split(",")
        with update_lock:
            baro_m = 400.0  # DEBUG: fixed altitude for L_DISTANCE_HIGH test (original: float(parts[0]))
    except (ValueError, IndexError):
        log("Barometer data format error", events.EventType.error)


def handle_target_coord(data: str):
    parts = data.split(",")
    if len(parts) == 2:
        with update_lock:
            target.lat, target.lon = float(parts[0]), float(parts[1])
        motor_guidance.set_target_coord(target.lat, target.lon)
        log(f"Target set: ({target.lat:.6f}, {target.lon:.6f})")
    else:
        log("Target coords format error", events.EventType.error)


def handle_flight_state(data: str):
    global state
    with update_lock:
        state = int(data)
        if state == 3 and GpsVector.lat is not None and GpsVector.lon is not None:
            motor_guidance.set_start_coordinates(GpsVector.lat, GpsVector.lon)
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


def _resolve_patterned(flight_state: int, alt_m: float) -> bool:
    """고도와 state로 8자 비행 여부 결정."""
    return False  # 패턴 비활성화 — 호밍 제어만 사용
    # if flight_state == 4:
    #     return alt_m > 10.0   # EGG: 10m 초과 → 8자, 10m 이하 → 당근 (Final)
    # return False               # state 3: 당근 제어 (호밍)


def ctrl_paragldr():
    motors_off = False
    while running:
        with update_lock:
            _state       = state
            _motor_enabled    = motor_enabled
            _baro_m      = baro_m
            _GpsVector         = types.SimpleNamespace(
                lat=GpsVector.lat, lon=GpsVector.lon,
                speed=GpsVector.speed, course=GpsVector.course)
            _GpsFidelity    = types.SimpleNamespace(
                rmc_status=GpsFidelity.rmc_status,
                fix_quality=GpsFidelity.fix_quality,
                sats=GpsFidelity.sats)
            _altitude         = types.SimpleNamespace(
                yaw=altitude.yaw, gyrz=altitude.gyrz)
            _target      = types.SimpleNamespace(
                lat=target.lat, lon=target.lon)
            _last_gps_t  = last_gps_time
            _last_imu_t  = last_imu_time

        if _state >= 3 and _motor_enabled:

            # State 5: 서보 신호 완전 차단 후 루프 유지 (재진입 방지)
            if _state == 5:
                if not motors_off:
                    motor_control.set_motors_off(pi)
                    motors_off = True
                    log("State 5: motors off", events.EventType.warning)
                time.sleep(CONTROL_LOG_INTERVAL)
                continue

            motors_off = False

            # FDIR 게이트
            failsafe_reason = None

            # FDIR-0: 센서 수신 여부
            if _GpsVector.lat is None or _GpsVector.lon is None or _altitude.yaw is None or _baro_m is None:

                missing = []
                if _GpsVector.lat is None:  missing.append("GPS")
                if _GpsVector.lon is None:  missing.append("GPS")
                if _altitude.yaw is None:  missing.append("IMU")
                if _baro_m is None:   missing.append("BARO")
                failsafe_reason = f"No data received: {'+'.join(missing)}"

            # FDIR-1: 센서 통신 타임아웃
            if failsafe_reason is None:
                now = time.time()
                gps_stale = (now - _last_gps_t) > STALE_THRESHOLD if _last_gps_t is not None else True
                imu_stale = (now - _last_imu_t) > STALE_THRESHOLD if _last_imu_t is not None else True

                if gps_stale and imu_stale:
                    failsafe_reason = "Sensor timeout: GPS+IMU stale"
                elif gps_stale:
                    failsafe_reason = "Sensor timeout: GPS stale"
                elif imu_stale:
                    failsafe_reason = "Sensor timeout: IMU stale"

            # FDIR-2: GPS 무결성
            if failsafe_reason is None:
                if not motor_guidance.is_gps_valid(_GpsVector, _GpsFidelity):
                    failsafe_reason = (
                        f"GPS invalid (lat={_GpsVector.lat}, lon={_GpsVector.lon}, "
                        f"fix={_GpsFidelity.fix_quality}, sats={_GpsFidelity.sats}, "
                        f"rmc={_GpsFidelity.rmc_status})")

            # FDIR-3: 극한 회전 상태
            if failsafe_reason is None:
                if abs(_altitude.gyrz) > GYRZ_RUNAWAY_THRESHOLD:
                    failsafe_reason = (
                        f"|gyrz|={abs(_altitude.gyrz):.2f} rad/s "
                        f"({math.degrees(abs(_altitude.gyrz)):.1f}°/s) > "
                        f"{GYRZ_RUNAWAY_THRESHOLD:.2f} rad/s")

            # FDIR-4: 기압계 고도
            if failsafe_reason is None:
                if _baro_m <= 0.0:
                    failsafe_reason = f"Baro altitude invalid ({_baro_m:.1f}m)"

            # FDIR-5: Target 좌표 수신
            if failsafe_reason is None:
                if _target.lat is None or _target.lon is None:
                    failsafe_reason = "No target coordinates received"

            # Failsafe 분기
            if failsafe_reason is not None:
                log(f"Failsafe: {failsafe_reason}", events.EventType.error)
                motor_control.set_neutral(pi)
            else:
                _patterned = _resolve_patterned(_state, _baro_m)

                if DEBUG_GUIDANCE:
                    _dbg(
                        f"[GUIDANCE IN ] "
                        f"state={_state} baro={_baro_m:.1f}m patterned={_patterned} | "
                        f"yaw={_altitude.yaw:.1f}° gyrz={_altitude.gyrz:.2f} | "
                        f"gps=({_GpsVector.lat:.6f},{_GpsVector.lon:.6f}) spd={_GpsVector.speed:.1f} crs={_GpsVector.course:.1f} | "
                        f"fix={_GpsFidelity.fix_quality} sats={_GpsFidelity.sats} rmc={_GpsFidelity.rmc_status} | "
                        f"target=({_target.lat:.6f},{_target.lon:.6f})"
                    )

                result = motor_guidance.guidance(
                    _altitude, _GpsVector, _GpsFidelity, _target,
                    baro_m=_baro_m, patterned=_patterned
                )

                motor_result = motor_control.control(pi, result.commanded_yaw_rate)

                if DEBUG_GUIDANCE and motor_result is not None:
                    _dbg(
                        f"[MOTOR] "
                        f"L: {motor_result.left_cmd_deg:6.1f}°  pw={motor_result.left_pulse} | "
                        f"R: {motor_result.right_cmd_deg:6.1f}°  pw={motor_result.right_pulse} | "
                        f"delta={motor_result.actual_delta_deg:+.1f}°  exp_yr={motor_result.expected_yaw_rate:+.2f}°/s"
                    )
                    simlogfile.write("---\n")
                    simlogfile.flush()

                log_control(result, motor_result)

        time.sleep(CONTROL_LOG_INTERVAL)


def init() -> bool:
    global pi, running

    signal.signal(signal.SIGINT, signal.SIG_IGN)
    log("Initializing motorapp")

    try:
        Motor_Release.init_burnwire()
        motor_guidance.init_guidance()
        pi = motor_control.init_control()
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

    try:
        controllogfile.close()
    except Exception:
        pass

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
