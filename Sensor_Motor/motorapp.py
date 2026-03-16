import os
import signal
import threading
import types
from datetime import datetime
from multiprocessing import connection

from lib import appargs, msgstructure, events

from Sensor_Motor import Motor_Parafoil, Motor_Release, Motor_Egg, Motor_Parafoil_Calculate

# =============================================================================
# 제어 로그 (control.txt)
# =============================================================================

log_dir = "./sensorlogs"
if not os.path.exists(log_dir):
    os.makedirs(log_dir)
controllogfile = open(os.path.join(log_dir, "control.txt"), "a")


def log_control(target_lat, target_lon, target_azimuth, distance, error, effective_error,
                p_term, i_term, d_term, pid_integral, pid_output, left_pulse, right_pulse):
    """제어 목표, 오차, PID 상태, 모터 펄스를 control.txt에 기록 (imu.txt 형식)"""
    t = datetime.now().isoformat(sep=" ", timespec="milliseconds")
    line = (f"{t},{target_lat:.6f},{target_lon:.6f},{target_azimuth:.4f},{distance:.2f},"
            f"{error:.4f},{effective_error:.4f},{p_term:.4f},{i_term:.4f},{d_term:.4f},"
            f"{pid_integral:.4f},{pid_output:.4f},{left_pulse},{right_pulse}\n")
    controllogfile.write(line)
    controllogfile.flush()


# =============================================================================
# 상태 변수
# =============================================================================

running = True
motor_enabled = True
arms_pulled = False  # EGG 진입 시 모터 암 중립(당김) 유지
pi = None  # pigpio instance

# 센서 데이터
altitude = types.SimpleNamespace(yaw=0.0, gyrz=0.0)
GpsVector = types.SimpleNamespace(lat=0.0, lon=0.0, speed=0.0, course=0.0)
GpsFidelity = types.SimpleNamespace(rmc_status="V", fix_quality=0, sats=0)
state = 0  # 0=LAUNCHPAD, 1=ASCENT, 2=APOGEE, 3=DESCENT, 4=EGG_RELEASE, 5=LANDED

threads: dict[str, threading.Thread] = {}
update_lock = threading.Lock()
CONTROL_LOG_INTERVAL = 0.1  # 10Hz 주기 로깅 (GPS/IMU 의존 없이)

APP = appargs.MotorAppArg.AppName

def log(msg: str, level=events.EventType.info):
    events.LogEvent(APP, level, msg)

# =============================================================================
# 메시지 핸들러
# =============================================================================

def handle_terminate(data: str):
    global running
    log("Termination detected")
    running = False

def handle_gps(data: str):
    parts = data.split(",")
    if len(parts) == 7:
        GpsVector.lat = float(parts[0])
        GpsVector.lon = float(parts[1])
        GpsVector.speed = float(parts[2])  # m/s
        GpsVector.course = float(parts[3])
        GpsFidelity.fix_quality = int(parts[4])
        GpsFidelity.sats = int(parts[5])
        GpsFidelity.rmc_status = parts[6]
    else:
        log("GPS data format error", events.EventType.error)

def handle_imu(data: str):
    parts = data.split(",")
    if len(parts) == 2:
        altitude.yaw = float(parts[0])
        altitude.gyrz = float(parts[1])
    else:
        log("IMU data format error", events.EventType.error)

def handle_target_coords(data: str):
    parts = data.split(",")
    if len(parts) == 2:
        lat, lon = float(parts[0]), float(parts[1])
        Motor_Parafoil_Calculate.set_target_coordinates(lat, lon)
        log(f"Target set: ({lat:.6f}, {lon:.6f})")
    else:
        log("Target coords format error", events.EventType.error)

def handle_flight_state(data: str):
    global state, arms_pulled
    state = int(data)
    if state == 4:  # EGG 진입 시 초기화
        arms_pulled = False
    log(f"Flight state: {state}")

def handle_pull_arms():
    global arms_pulled
    arms_pulled = True
    log("Motor arms pull (neutral)")

def handle_release():
    log("Activating burnwire")
    Motor_Release.activate_burnwire()

def handle_egg_drop():
    log("Activating solenoid")
    Motor_Egg.activate_solenoid()

def handle_mec(data: str):
    global motor_enabled
    log(f"MEC command: {data}")
    if data == "ON":
        motor_enabled = True
    elif data == "OFF":
        motor_enabled = False
    else:
        log(f"Invalid MEC option: {data}", events.EventType.error)

MSG_HANDLERS = {
    appargs.MainAppArg.MID_TerminateProcess: handle_terminate,
    appargs.GpsAppArg.MID_motor_gps: handle_gps,
    appargs.ImuAppArg.MID_motor_imu: handle_imu,
    appargs.FlightlogicAppArg.MID_motor_TargetCor: handle_target_coords,
    appargs.FlightlogicAppArg.MID_motor_state: handle_flight_state,
    appargs.FlightlogicAppArg.MID_motor_burnwire: lambda d: handle_release(),
    appargs.FlightlogicAppArg.MID_motor_EggDrop: lambda d: handle_egg_drop(),
    appargs.FlightlogicAppArg.MID_motor_PullArms: lambda d: handle_pull_arms(),
    appargs.CommAppArg.MID_RouteCmd_MEC: handle_mec
}

def dispatch(msg: msgstructure.MsgStructure):
    handler = MSG_HANDLERS.get(msg.MsgID) # MID에 해당하는 함수 가져오기 (pipe와 상관 없음)
    if handler:
        handler(msg.data)
    else:
        log(f"Unknown MID: {msg.MsgID}", events.EventType.error)

# =============================================================================
# 파라포일 제어
# =============================================================================

def control_payload():
    """10Hz로 파라포일 제어 및 로깅"""
    import time
    while running:
        with update_lock:
            if state >= 3 and motor_enabled and pi is not None:
                if arms_pulled:
                    ctrl = Motor_Parafoil.pull_both_arms(pi)
                    error, target_azimuth, distance = 0.0, 0.0, 0.0
                    
                else:
                    error, target_azimuth, distance = Motor_Parafoil_Calculate.calculate_raw_error(altitude.yaw, GpsVector.lat, GpsVector.lon)
                    ctrl = Motor_Parafoil.rotate_parafoil_motor(pi, altitude.yaw, error) #gyro_Z, and added gps datas
                target_lat, target_lon = Motor_Parafoil_Calculate.get_target_coordinates()
                log_control(
                    target_lat, target_lon, target_azimuth, distance,
                    ctrl["error"], ctrl["effective_error"],
                    ctrl["p_term"], ctrl["i_term"], ctrl["d_term"],
                    ctrl["pid_integral"], ctrl["pid_output"],
                    ctrl["left_pulse"], ctrl["right_pulse"],
                )#need to fix

                if state == 5:
                    log("Stopping motors", events.EventType.warning)
                    Motor_Parafoil.rotate_parafoil_motor(pi, 0.0, 0.0)

        time.sleep(CONTROL_LOG_INTERVAL)

# =============================================================================
# 초기화 / 종료
# =============================================================================

def init() -> bool:
    global pi, running
    
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    log("Initializing motorapp")
    
    try:
        Motor_Parafoil_Calculate.init_parafoil_control()
        pi = Motor_Parafoil.init_parafoil_motor()
        Motor_Release.init_burnwire()
        Motor_Egg.init_solenoid()
        threads["ControlLog_Thread"] = threading.Thread(target=control_payload, name="ControlLog_Thread", daemon=True)
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

    # 모터 종료
    if pi:
        Motor_Parafoil.terminate_parafoil_motor(pi)
        pi.stop()
    Motor_Release.terminate_burnwire()
    Motor_Egg.terminate_solenoid()
    
    # 스레드 종료
    for name, thread in threads.items():
        log(f"Joining thread: {name}")
        thread.join()
    
    log("Motorapp terminated")

# =============================================================================
# 메인 루프
# =============================================================================

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

            if unpacked_msg.receiver_app in (appargs.MotorAppArg.AppID, appargs.MainAppArg.AppID):
                dispatch(unpacked_msg)
    
    except Exception as e:
        log(f"Error: {e}", events.EventType.error)
    
    finally:
        terminate()
