import signal
import threading
from multiprocessing import connection

from lib import appargs, msgstructure, events

from Sensor_Motor import Motor_Parafoil, Motor_Release, Motor_Egg, Motor_Parafoil_Calculate

# =============================================================================
# 상태 변수
# =============================================================================

running = True
motor_enabled = True
pi = None  # pigpio instance

# 센서 데이터
yaw = 0.0
lat = 0.0
lon = 0.0
state = 0  # 0=LAUNCHPAD, 1=ASCENT, 2=APOGEE, 3=DESCENT, 4=EGG_RELEASE, 5=LANDED

threads: dict[str, threading.Thread] = {}

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

def handle_gps_data(data: str):
    global lat, lon
    parts = data.split(",")
    if len(parts) == 2:
        lat, lon = float(parts[0]), float(parts[1])
        update_parafoil()
    else:
        log("GPS data format error", events.EventType.error)

def handle_imu_data(data: str):
    global yaw
    yaw = float(data)
    update_parafoil()

def handle_target_coords(data: str):
    global lat, lon
    parts = data.split(",")
    if len(parts) == 2:
        lat, lon = float(parts[0]), float(parts[1])
        Motor_Parafoil_Calculate.set_target_coordinates(lat, lon)
        log(f"Target set: ({lat:.6f}, {lon:.6f})")
    else:
        log("Target coords format error", events.EventType.error)

def handle_flight_state(data: str):
    global state
    state = int(data)
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
    if data == "ON":
        motor_enabled = True
    elif data == "OFF":
        motor_enabled = False
    else:
        log(f"Invalid MEC option: {data}", events.EventType.error)

MSG_HANDLERS = {
    appargs.MainAppArg.MID_TerminateProcess: handle_terminate,
    appargs.GpsAppArg.MID_motor_MyCor: handle_gps_data,
    appargs.ImuAppArg.MID_motor_yaw: handle_imu_data,
    appargs.FlightlogicAppArg.MID_motor_TargetCor: handle_target_coords,
    appargs.FlightlogicAppArg.MID_motor_state: handle_flight_state,
    appargs.FlightlogicAppArg.MID_motor_burnwire: lambda d: handle_release(),
    appargs.FlightlogicAppArg.MID_motor_EggDrop: lambda d: handle_egg_drop(),
    appargs.CommAppArg.MID_RouteCmd_MEC: handle_mec
}


def dispatch(msg: msgstructure.MsgStructure):
    handler = MSG_HANDLERS.get(msg.MsgID)
    if handler:
        handler(msg.data)
    else:
        log(f"Unknown MID: {msg.MsgID}", events.EventType.error)

# =============================================================================
# 파라포일 제어
# =============================================================================

def update_parafoil():
    if state < 3 or not motor_enabled:
        return
    
    error = Motor_Parafoil_Calculate.calculate_motor_control(yaw, lat, lon)
    Motor_Parafoil.rotate_parafoil_motor(pi, error)

    if state == 5:
        log("Stopping motors", events.EventType.warning)
        Motor_Parafoil.rotate_parafoil_motor(pi, 0.0)

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
            raw = main_pipe.recv()
            msg = msgstructure.unpack_msg(raw)

            if msg == False:
                continue

            if msg.receiver_app in (appargs.MotorAppArg.AppID, appargs.MainAppArg.AppID):
                dispatch(msg)
    
    except Exception as e:
        log(f"Error: {e}", events.EventType.error)
    
    finally:
        terminate()
