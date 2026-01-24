
"""
Flight Logic App - 비행 상태 관리 및 상태 전이
Author: Jeongmin Park
"""

import signal
import threading
import time
import math
from multiprocessing import Queue, connection

from lib import appargs, msgstructure, events, prevstate, config
from Sensor_Motor import parafoil_control

# =============================================================================
# 상수
# =============================================================================

STATE = {
    "LAUNCH_PAD": 0,
    "ASCENT": 1,
    "APOGEE": 2,
    "RELEASE": 3,
    "EGG": 4,
    "LANDED": 5,
}

STATE_NAMES = ["LAUNCH_PAD", "ASCENT", "APOGEE", "RELEASE", "EGG", "LANDED"]

# 고도 임계값
EGG_DROP_ALT = 2.0          # 계란 사출 고도 (m)
SOLENOID_ALT_MIN = 3.0      # 솔레노이드 안전 작동 시작 고도
SOLENOID_ALT_MAX = 4.0      # 솔레노이드 안전 작동 종료 고도
TARGET_RADIUS = 50.0        # 목표 도달 반경 (m)

APP = appargs.FlightlogicAppArg.AppName

# =============================================================================
# 상태 변수
# =============================================================================

_running = True
_state = 0
_max_alt = 0.0
_recent_alt = []

# 시뮬레이션
_sim_enable = False
_sim_active = False

# 카운터
_cnt_ascent = 0
_cnt_apogee = 0
_cnt_descent = 0
_cnt_release = 0
_cnt_landed = 0
_cnt_egg_drop = 0

# 플래그
_egg_activated = False
_solenoid_count = 0
_solenoid_done = False
_target_reached = False

# 목표 좌표
_target_lat = 0.0
_target_lon = 0.0

# Queue
_main_queue: Queue = None

_threads: dict[str, threading.Thread] = {}


def _log(msg: str, level=events.EventType.info):
    events.LogEvent(APP, level, msg)


# =============================================================================
# 메시지 핸들러
# =============================================================================

def _handle_terminate(data: str):
    global _running
    _log("Termination detected")
    _running = False


def _handle_sim(data: str):
    global _sim_enable, _sim_active
    if data == "ENABLE":
        _sim_enable = True
        _log("Simulation enabled")
    elif data == "ACTIVATE":
        _sim_active = True
        _log("Simulation activated")
    elif data == "DISABLE":
        _sim_enable = _sim_active = False
        _log("Simulation disabled")
    # 시뮬레이션 상태 전송
    status = "S" if (_sim_enable and _sim_active) else "F"
    msgstructure.send_msg(_main_queue, appargs.FlightlogicAppArg.AppID, appargs.CommAppArg.AppID, appargs.FlightlogicAppArg.MID_comm_sim, status)


def _handle_simp(data: str):
    if _sim_enable and _sim_active:
        _barometer_logic(float(data))


def _handle_barometer(data: str):
    if not (_sim_enable and _sim_active):
        _barometer_logic(float(data))


def _handle_gps(data: str):
    global _target_reached
    if _sim_enable and _sim_active:
        return
    
    parts = data.split(",")
    if len(parts) != 2:
        _log("GPS data format error", events.EventType.error)
        return
    
    lat, lon = float(parts[0]), float(parts[1])
    
    # motorapp으로 전달
    msgstructure.send_msg(_main_queue, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_SendGpsMotorData, f"{lat},{lon}")
    
    # 목표 도달 체크
    if (_target_lat != 0.0 or _target_lon != 0.0) and parafoil_control.is_gps_valid(lat, lon):
        dist = parafoil_control.calculate_distance_haversine(lat, lon, _target_lat, _target_lon)
        if not _target_reached and dist <= TARGET_RADIUS:
            _target_reached = True
            _log(f"Target reached! Distance: {dist:.2f}m")


def _handle_imu(data: str):
    if _sim_enable and _sim_active:
        return
    yaw = float(data)
    msgstructure.send_msg(_main_queue, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_SendImuMotorData, str(yaw))


def _handle_ss(data: str):
    """Set State 명령 처리"""
    state = int(data)
    transitions = [
        _to_launch_pad, _to_ascent, _to_apogee, _to_release, _to_egg, _to_landed
    ]
    if 0 <= state < len(transitions):
        transitions[state](force=True)
    else:
        _log(f"Invalid state: {state}", events.EventType.error)


def _handle_reset_alt(data: str):
    global _max_alt, _recent_alt
    _max_alt = 0
    _recent_alt.clear()


_MSG_HANDLERS = {
    appargs.MainAppArg.MID_TerminateProcess: _handle_terminate,
    appargs.CommAppArg.MID_RouteCmd_SIM: _handle_sim,
    appargs.CommAppArg.MID_RouteCmd_SIMP: _handle_simp,
    appargs.BarometerAppArg.MID_SendBarometerFlightLogicData: _handle_barometer,
    appargs.GpsAppArg.MID_SendGpsFlightLogicData: _handle_gps,
    appargs.ImuAppArg.MID_SendImuFlightLogicData: _handle_imu,
    appargs.CommAppArg.MID_RouteCmd_SS: _handle_ss,
    appargs.BarometerAppArg.MID_ResetBarometerMaxAlt: _handle_reset_alt,
}


def _dispatch(msg: msgstructure.MsgStructure):
    handler = _MSG_HANDLERS.get(msg.MsgID)
    if handler:
        handler(msg.data)
    else:
        _log(f"Unknown MID: {msg.MsgID}", events.EventType.error)


# =============================================================================
# 기압계 로직
# =============================================================================

def _barometer_logic(alt: float):
    global _max_alt, _recent_alt, _state
    global _cnt_ascent, _cnt_apogee, _cnt_descent, _cnt_release, _cnt_landed, _cnt_egg_drop
    global _egg_activated, _solenoid_count, _solenoid_done
    
    # 최근 고도 기록
    _recent_alt.append(alt)
    if len(_recent_alt) > 3:
        _recent_alt.pop(0)
    
    # 최대 고도 갱신
    if len(_recent_alt) > 2:
        second_max = sorted(_recent_alt, reverse=True)[1]
        if second_max > _max_alt:
            _max_alt = second_max
            prevstate.update_maxalt(second_max)
    
    if len(_recent_alt) < 3:
        return
    
    # 카운터 하한 보정(0 밑으로 x)
    _cnt_ascent = max(0, _cnt_ascent)
    _cnt_apogee = max(0, _cnt_apogee)
    _cnt_descent = max(0, _cnt_descent)
    _cnt_release = max(0, _cnt_release)
    _cnt_landed = max(0, _cnt_landed)
    
    # === LAUNCH_PAD (0) ===
    if _state == STATE["LAUNCH_PAD"]:
        if alt > 50:
            _cnt_ascent += 1
        else:
            _cnt_ascent -= 2
        if _cnt_ascent >= 3:
            _to_ascent()
    
    # === ASCENT (1) ===
    elif _state == STATE["ASCENT"]:
        if alt <= _max_alt - 20:
            _cnt_descent += 1
        else:
            _cnt_descent -= 2
        
        if _max_alt - 20 < alt < _max_alt - 0.25:
            _cnt_apogee += 1
        else:
            _cnt_apogee -= 2
        
        if _cnt_descent >= 2:
            _to_release()
        elif _cnt_apogee >= 2:
            _to_apogee()
    
    # === APOGEE (2) ===
    elif _state == STATE["APOGEE"]:
        if alt <= _max_alt - 20:
            _cnt_descent += 1
        else:
            _cnt_descent -= 2
        if _cnt_descent >= 2:
            _to_release()
    
    # === RELEASE (3) ===
    elif _state == STATE["RELEASE"]:
        if alt <= _max_alt * 0.80:
            _cnt_release += 1
        else:
            _cnt_release -= 2
        if _cnt_release >= 2:
            _to_egg()
    
    # === EGG (4) ===
    elif _state == STATE["EGG"]:
        # 솔레노이드 안전 작동 (3~4m)
        if not _solenoid_done and SOLENOID_ALT_MIN <= alt <= SOLENOID_ALT_MAX:
            if _solenoid_count < 6:
                _log(f"Safety solenoid ({_solenoid_count + 1}/6) at {alt:.2f}m")
                msgstructure.send_msg(_main_queue, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_Motor_Egg_Drop_Activate, "")
                _solenoid_count += 1
                if _solenoid_count >= 5:
                    _solenoid_done = True
                    _log(f"Safety solenoid complete ({_solenoid_count} times)")
        
        # 계란 사출 (목표 도달 + 2m 이하)
        if not _egg_activated and _target_reached and alt <= EGG_DROP_ALT:
            _cnt_egg_drop += 1
        else:
            _cnt_egg_drop = max(0, _cnt_egg_drop - 2)
        
        if not _egg_activated and _target_reached and _cnt_egg_drop >= 2:
            _log(f"Egg drop at {alt:.2f}m")
            msgstructure.send_msg(_main_queue, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_Motor_Egg_Drop_Activate, "")
            _egg_activated = True
        elif not _target_reached and alt <= EGG_DROP_ALT:
            _log(f"At drop altitude ({alt:.2f}m) but target not reached")
        
        # 착륙 감지
        if alt <= 15:
            _cnt_landed += 1
        else:
            _cnt_landed -= 2
        if _cnt_landed >= 3:
            _to_landed()


# =============================================================================
# 상태 변환
# =============================================================================

def _can_transition(force: bool) -> bool:
    return force or config.STATE_OVERRIDE is None


def _to_launch_pad(force: bool = False):
    global _state, _max_alt, _recent_alt
    if not _can_transition(force):
        return
    _state = STATE["LAUNCH_PAD"]
    _max_alt = 0
    _recent_alt.clear()
    _log("STATE → LAUNCH_PAD")
    prevstate.update_prevstate(_state)
    msgstructure.send_msg(_main_queue, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_SendFlightStateToMotor, str(_state))


def _to_ascent(force: bool = False):
    global _state
    if not _can_transition(force):
        return
    _state = STATE["ASCENT"]
    _log("STATE → ASCENT")
    prevstate.update_prevstate(_state)
    msgstructure.send_msg(_main_queue, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_SendFlightStateToMotor, str(_state))
    msgstructure.send_msg(_main_queue, appargs.FlightlogicAppArg.AppID, appargs.CameraAppArg.AppID, appargs.FlightlogicAppArg.MID_cam_activate, "")


def _to_apogee(force: bool = False):
    global _state
    if not _can_transition(force):
        return
    _state = STATE["APOGEE"]
    _log("STATE → APOGEE")
    prevstate.update_prevstate(_state)
    msgstructure.send_msg(_main_queue, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_SendFlightStateToMotor, str(_state))
    msgstructure.send_msg(_main_queue, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_Motor_Parafoil_Activate, "")


def _to_release(force: bool = False):
    global _state
    if not _can_transition(force):
        return
    _state = STATE["RELEASE"]
    _log("STATE → RELEASE (burnwire activate)")
    prevstate.update_prevstate(_state)
    msgstructure.send_msg(_main_queue, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_SendFlightStateToMotor, str(_state))
    msgstructure.send_msg(_main_queue, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_Motor_Release_Activate, "")


def _to_egg(force: bool = False):
    global _state
    if not _can_transition(force):
        return
    _state = STATE["EGG"]
    _log("STATE → EGG")
    prevstate.update_prevstate(_state)
    msgstructure.send_msg(_main_queue, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_SendFlightStateToMotor, str(_state))


def _to_landed(force: bool = False):
    global _state
    if not _can_transition(force):
        return
    _state = STATE["LANDED"]
    _log("STATE → LANDED (motors stop)")
    prevstate.update_prevstate(_state)
    msgstructure.send_msg(_main_queue, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_SendFlightStateToMotor, str(_state))
    msgstructure.send_msg(_main_queue, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_PayloadMotorStop, "")


# =============================================================================
# 주기적 전송
# =============================================================================

def _send_hk():
    while _running:
        msgstructure.send_msg(_main_queue, appargs.FlightlogicAppArg.AppID, appargs.HkAppArg.AppID, appargs.FlightlogicAppArg.MID_SendHK, str(_running))
        time.sleep(1)


def _send_current_state():
    while _running:
        msgstructure.send_msg(_main_queue, appargs.FlightlogicAppArg.AppID, appargs.CommAppArg.AppID, appargs.FlightlogicAppArg.MID_comm_state, STATE_NAMES[_state])
        time.sleep(1)


# =============================================================================
# 초기화 / 종료
# =============================================================================

def _init():
    global _state, _max_alt, _target_lat, _target_lon
    
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    _log("Initializing flightlogicapp")
    
    try:
        # 상태 복원
        if config.STATE_OVERRIDE is not None:
            _state = int(config.STATE_OVERRIDE)
            _log(f"Using STATE_OVERRIDE: {_state}")
        else:
            _state = int(prevstate.PREV_STATE)
            _log(f"Using prev state: {_state}")
        
        # 상태 전이 수행
        transitions = [_to_launch_pad, _to_ascent, _to_apogee, _to_release, _to_egg, _to_landed]
        if 0 <= _state < len(transitions):
            transitions[_state](force=True)
        
        # 이전 데이터 복원
        _max_alt = float(prevstate.PREV_MAX_ALT)
        _target_lat = float(prevstate.Target_lat)
        _target_lon = float(prevstate.Target_lon)
        
        # 목표 좌표 전송
        if _target_lat != 0.0 or _target_lon != 0.0:
            msgstructure.send_msg(_main_queue, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_motor_TargetCor, f"{_target_lat},{_target_lon}")
        
        _log(f"Initialized with state={_state}")
        
    except Exception as e:
        _log(f"Init error: {e}", events.EventType.error)


def _terminate():
    global _running
    _running = False
    _log("Terminating flightlogicapp")
    
    for name, thread in _threads.items():
        _log(f"Joining thread: {name}")
        thread.join()
    
    _log("Flightlogicapp terminated")


# =============================================================================
# 메인 루프
# =============================================================================

def flightlogicapp_main(main_queue: Queue, main_pipe: connection.Connection):
    global _running, _main_queue
    _running = True
    _main_queue = main_queue
    
    _init()
    
    # 스레드 시작
    _threads["HK"] = threading.Thread(target=_send_hk, daemon=True)
    _threads["State"] = threading.Thread(target=_send_current_state, daemon=True)
    for t in _threads.values():
        t.start()
    
    try:
        while _running:
            raw = main_pipe.recv()
            msg = msgstructure.MsgStructure()
            
            if not msgstructure.unpack_msg(msg, raw):
                continue
            
            if msg.receiver_app in (appargs.FlightlogicAppArg.AppID, appargs.MainAppArg.AppID):
                _dispatch(msg)
    
    except Exception as e:
        _log(f"Error: {e}", events.EventType.error)
    
    finally:
        _terminate()
