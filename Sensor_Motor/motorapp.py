import signal
import threading
import time
from multiprocessing import Queue, connection

from lib import appargs, msgstructure, events

from Sensor_Motor import Motor_Parafoil, Motor_Release, Motor_Egg, parafoil_control

# =============================================================================
# 상태 변수
# =============================================================================

_running = True
_motor_enabled = True
_pi = None  # pigpio instance

# 센서 데이터
_yaw = 0.0
_lat = 0.0
_lon = 0.0
_state = 0  # 0=LAUNCHPAD, 1=ASCENT, 2=APOGEE, 3=DESCENT, 4=EGG_RELEASE, 5=LANDED

_threads: dict[str, threading.Thread] = {}

APP = appargs.MotorAppArg.AppName

def _log(msg: str, level=events.EventType.info):
    events.LogEvent(APP, level, msg)

# =============================================================================
# 메시지 핸들러
# =============================================================================

def _handle_terminate(data: str):
    global _running
    _log("Termination detected")
    _running = False

def _handle_gps_data(data: str):
    global _lat, _lon
    parts = data.split(",")
    if len(parts) == 2:
        _lat, _lon = float(parts[0]), float(parts[1])
        _update_parafoil()
    else:
        _log("GPS data format error", events.EventType.error)

def _handle_imu_data(data: str):
    global _yaw
    _yaw = float(data)
    _update_parafoil()

def _handle_target_coords(data: str):
    parts = data.split(",")
    if len(parts) == 2:
        lat, lon = float(parts[0]), float(parts[1])
        parafoil_control.set_target_coordinates(lat, lon)
        _log(f"Target set: ({lat:.6f}, {lon:.6f})")
    else:
        _log("Target coords format error", events.EventType.error)

def _handle_flight_state(data: str):
    global _state
    _state = int(data)
    _log(f"Flight state: {_state}")

def _handle_release():
    _log("Activating burnwire")
    Motor_Release.activate_burnwire()

def _handle_egg_drop():
    _log("Activating solenoid")
    Motor_Egg.activate_solenoid()

def _handle_mec(data: str):
    global _motor_enabled
    _log(f"MEC command: {data}")
    if data == "ON":
        _motor_enabled = True
    elif data == "OFF":
        _motor_enabled = False
    else:
        _log(f"Invalid MEC option: {data}", events.EventType.error)

_MSG_HANDLERS = {
    appargs.MainAppArg.MID_TerminateProcess: _handle_terminate,
    appargs.FlightlogicAppArg.MID_SendGpsMotorData: _handle_gps_data,
    appargs.FlightlogicAppArg.MID_SendImuMotorData: _handle_imu_data,
    appargs.FlightlogicAppArg.MID_motor_TargetCor: _handle_target_coords,
    appargs.FlightlogicAppArg.MID_motor_state: _handle_flight_state,
    appargs.FlightlogicAppArg.MID_motor_burnwire: lambda d: _handle_release(),
    appargs.FlightlogicAppArg.MID_motor_EggDrop: lambda d: _handle_egg_drop(),
    appargs.CommAppArg.MID_RouteCmd_MEC: _handle_mec
}


def _dispatch(msg: msgstructure.MsgStructure):
    handler = _MSG_HANDLERS.get(msg.MsgID)
    if handler:
        handler(msg.data)
    else:
        _log(f"Unknown MID: {msg.MsgID}", events.EventType.error)

# =============================================================================
# 파라포일 제어
# =============================================================================

def _update_parafoil():
    if _state < 3 or not _motor_enabled:
        return
    
    turn = parafoil_control.calculate_motor_control(_yaw, _lat, _lon)
    Motor_Parafoil.rotate_parafoil_motor(_pi, turn)

    if _state == 5:
        _log("Stopping motors")
        Motor_Parafoil.rotate_parafoil_motor(_pi, 0.0)

# =============================================================================
# 초기화 / 종료
# =============================================================================

def _init() -> bool:
    global _pi, _running
    
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    _log("Initializing motorapp")
    
    try:
        parafoil_control.init_parafoil_control()
        _pi = Motor_Parafoil.init_parafoil_motor()
        Motor_Release.init_burnwire()
        Motor_Egg.init_solenoid()
        _log("Motors initialized (parafoil, burnwire, solenoid)")
        return True
    except Exception as e:
        _log(f"Init failed: {e}", events.EventType.error)
        _running = False
        return False

def _terminate():
    global _running
    _running = False
    _log("Terminating motorapp")
    
    # 모터 종료
    if _pi:
        Motor_Parafoil.terminate_parafoil_motor(_pi)
        _pi.stop()
    Motor_Release.terminate_burnwire()
    Motor_Egg.terminate_solenoid()
    
    # 스레드 종료
    for name, thread in _threads.items():
        _log(f"Joining thread: {name}")
        thread.join()
    
    _log("Motorapp terminated")

# =============================================================================
# 메인 루프
# =============================================================================

def motorapp_main(main_queue: Queue, main_pipe: connection.Connection):
    global _running
    _running = True
    
    if not _init():
        return

    try:
        while _running:
            raw = main_pipe.recv()
            msg = msgstructure.MsgStructure()
            
            if not msgstructure.unpack_msg(msg, raw):
                msgstructure.unpack_msg(msg, raw)
                continue
            
            if msg.receiver_app in (appargs.MotorAppArg.AppID, appargs.MainAppArg.AppID):
                _dispatch(msg)
    
    except Exception as e:
        _log(f"Error: {e}", events.EventType.error)
    
    finally:
        _terminate()
