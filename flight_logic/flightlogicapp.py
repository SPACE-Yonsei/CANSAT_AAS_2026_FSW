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

EGG_DROP_ALT = 4.0      # 계란 사출 고도 (m, barometer)
RELEASE_TO_EGG_ALT = 50.0   # RELEASE → EGG 전환 고도 (m, barometer) 원래는 50이였음.

SOLENOID_DISTANCE_TARGET = 2500  # 솔레노이드 작동 목표 거리 (250cm = 2500mm)
SOLENOID_COUNT_MAX = 3      # 솔레노이드 작동 횟수 (3번)

APP = appargs.FlightlogicAppArg.AppName

# =============================================================================
# 상태 변수
# =============================================================================

running = True
state = 0
max_alt = 0.0
recent_alt = []

# 시뮬레이션
sim_enable = False
sim_active = False

# 카운터
cnt_ascent = 0
cnt_apogee = 0
cnt_release = 0
cnt_landed = 0
cnt_egg_drop = 0

# 플래그
egg_activated = False
solenoid_count = 0
solenoid_done = False

# TF-Luna 거리 센서 데이터
distance_mm = 0
recent_distance = []

# 목표 좌표
target_lat = 0.0
target_lon = 0.0

threads: dict[str, threading.Thread] = {}


def log(msg: str, level=events.EventType.info):
    events.LogEvent(APP, level, msg)


# =============================================================================
# 메시지 핸들러
# =============================================================================

def handleterminate(data: str, queue: Queue):
    global running
    log("Termination detected")
    running = False


def handle_sim(data: str, queue: Queue):
    global sim_enable, sim_active
    if data == "ENABLE":
        sim_enable = True
        log("Simulation enabled")
    elif data == "ACTIVATE":
        sim_active = True
        log("Simulation activated")
    elif data == "DISABLE":
        sim_enable = sim_active = False
        log("Simulation disabled")

    # Send sim status directly
    status = "S" if (sim_enable and sim_active) else "F"
    msgstructure.send_msg(queue, appargs.FlightlogicAppArg.AppID, appargs.CommAppArg.AppID, appargs.FlightlogicAppArg.MID_comm_sim, status)


def handle_simp(data: str, queue: Queue):
    if sim_enable and sim_active:
        barometer_logic(queue, float(data))


def handle_barometer(data: str, queue: Queue):
    if not (sim_enable and sim_active):
        barometer_logic(queue, float(data))


def handle_distance(data: str, queue: Queue):
    """TF-Luna 거리 센서 데이터 처리"""
    global distance_mm, recent_distance, state, solenoid_done
    
    if sim_enable and sim_active:
        return
    
    try:
        distance_mm = int(float(data))
        distance_mm = distance_mm
        
        # 최근 거리 기록 (3회)
        recent_distance.append(distance_mm)
        if len(recent_distance) > 3:
            recent_distance.pop(0)
        
        # EGG 상태에서 솔레노이드 작동 로직
        if state == STATE["EGG"] and not solenoid_done:
            solenoid_logic(queue, distance_mm)
            
    except (ValueError, TypeError) as e:
        log(f"Distance data parse error: {e}", events.EventType.error)


def handle_ss(data: str, queue: Queue):
    """Set State 명령 처리"""
    state = int(data)
    transitions = [
        to_launch_pad, to_ascent, to_apogee, to_release, to_egg, to_landed
    ]
    if 0 <= state < len(transitions):
        transitions[state](queue, force=True)
    else:
        log(f"Invalid state: {state}", events.EventType.error)


def handle_reset_alt(data: str, queue: Queue):
    global max_alt, recent_alt
    max_alt = 0
    recent_alt.clear()
    prevstate.update_maxalt(0)


def handle_target_coord(data: str, queue: Queue):
    """지상국 TC 명령: 'lat,lon' → prevstate 저장 + motor 전달."""
    global target_lat, target_lon
    parts = data.split(",")
    if len(parts) != 2:
        log(f"TC format error: expected 'lat,lon', got {data!r}", events.EventType.error)
        return
    try:
        new_lat = float(parts[0])
        new_lon = float(parts[1])
    except ValueError as e:
        log(f"TC parse error: {e} raw={data!r}", events.EventType.error)
        return
    if not (-90.0 <= new_lat <= 90.0 and -180.0 <= new_lon <= 180.0):
        log(f"TC out of range: lat={new_lat} lon={new_lon}", events.EventType.error)
        return
    target_lat = new_lat
    target_lon = new_lon
    prevstate.update_target_gps(new_lat, new_lon)
    msgstructure.send_msg(queue, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID,
                          appargs.FlightlogicAppArg.MID_motor_TargetCor, f"{new_lat},{new_lon}")
    log(f"Target set: ({new_lat:.6f}, {new_lon:.6f})")


MSG_HANDLERS = {
    appargs.MainAppArg.MID_TerminateProcess: handleterminate,
    appargs.CommAppArg.MID_RouteCmd_SIM: handle_sim,
    appargs.CommAppArg.MID_RouteCmd_SIMP: handle_simp,
    appargs.BarometerAppArg.MID_flight_alt: handle_barometer,
    #appargs.ImuAppArg.MID_flight_yaw: handle_imu,
    appargs.DistanceAppArg.MID_flight_dis: handle_distance,
    appargs.CommAppArg.MID_RouteCmd_SS: handle_ss,
    appargs.BarometerAppArg.MID_flight_ResetMaxAlt: handle_reset_alt,
    appargs.CommAppArg.MID_RouteCmd_TC: handle_target_coord,
}


def dispatch(msg: msgstructure.MsgStructure, queue: Queue):
    Handler = MSG_HANDLERS.get(msg.MsgID)
    if Handler:
        Handler(msg.data, queue)
    else:
        log(f"Unknown MID: {msg.MsgID}", events.EventType.error)


# =============================================================================
# 솔레노이드 로직
# =============================================================================

def solenoid_logic(queue: Queue, distance_mm: int):
    global solenoid_count, solenoid_done
    
    if solenoid_done:
        return
    
    # 거리가 250cm (2500mm) 이하일 때 작동
    if distance_mm <= SOLENOID_DISTANCE_TARGET and distance_mm > 0:
        if solenoid_count < SOLENOID_COUNT_MAX:
            solenoid_count += 1
            distance_cm = distance_mm / 10.0
            log(f"Solenoid activation ({solenoid_count}/{SOLENOID_COUNT_MAX}) at {distance_cm:.1f}cm (TF-Luna)")
            msgstructure.send_msg(queue, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_motor_EggDrop, "")

            if solenoid_count >= SOLENOID_COUNT_MAX:
                solenoid_done = True
                log(f"Solenoid complete ({solenoid_count} times)")
            time.sleep(0.1)

# =============================================================================
# 기압계 로직
# =============================================================================

def barometer_logic(queue: Queue, alt: float):
    global max_alt, recent_alt, state
    global cnt_ascent, cnt_apogee, cnt_release, cnt_landed, cnt_egg_drop
    global egg_activated, solenoid_count, solenoid_done
    
    # 최근 고도 기록
    recent_alt.append(alt)
    if len(recent_alt) > 3:
        recent_alt.pop(0)
    
    # 최대 고도 갱신
    if len(recent_alt) > 2:
        second_max = sorted(recent_alt, reverse=True)[1]
        if second_max > max_alt:
            max_alt = second_max
            prevstate.update_maxalt(second_max)
    
    if len(recent_alt) < 3:
        return
    
    # 카운터 하한 보정(0 밑으로 x)
    cnt_ascent = max(0, cnt_ascent)
    cnt_apogee = max(0, cnt_apogee)
    cnt_release = max(0, cnt_release)
    cnt_landed = max(0, cnt_landed)
    
    # === LAUNCH_PAD (0) ===
    if state == STATE["LAUNCH_PAD"]:
        if alt > 200:
            cnt_ascent += 1
        else:
            cnt_ascent -= 2
        if cnt_ascent >= 3:
            to_ascent(queue)
    
    # === ASCENT (1) ===
    elif state == STATE["ASCENT"]:
        if max_alt > 0 and alt <= max_alt * 0.8:
            cnt_release += 1
        else:
            cnt_release -= 2
        
        if max_alt * 0.8 < alt < max_alt - 0.25:
            cnt_apogee += 1
        else:
            cnt_apogee -= 2
        
        if cnt_release >= 3:
            to_release(queue)
        elif cnt_apogee >= 2:
            to_apogee(queue)
    
    # === APOGEE (2) ===
    elif state == STATE["APOGEE"]:
        if max_alt > 0 and alt <= max_alt * 0.8:
            cnt_release += 1
        else:
            cnt_release -= 2
        if cnt_release >= 3:
            to_release(queue)
    
    # === RELEASE (3) ===
    elif state == STATE["RELEASE"]:
        if alt <= RELEASE_TO_EGG_ALT:
            cnt_release += 1
        else:
            cnt_release = max(0, cnt_release - 2)
        if cnt_release >= 3:
            #to_egg(queue)
            to_egg(queue)
    
    # === EGG (4) ===
    elif state == STATE["EGG"]:
        # 솔레노이드 작동은 TF-Luna 거리 센서로 판별 (handle_distance에서 처리)
        # 여기서는 barometer 기반 계란 사출 및 착륙 감지만 처리

        # 계란 사출 (2m 이하, barometer)
        if not egg_activated and alt <= EGG_DROP_ALT:
            cnt_egg_drop += 1
        else:
            cnt_egg_drop = max(0, cnt_egg_drop - 2)
        
        if not egg_activated and cnt_egg_drop >= 2:
            log(f"Egg drop at {alt:.2f}m")
            msgstructure.send_msg(queue, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_motor_EggDrop, "")
            egg_activated = True
        
        # 착륙 감지
        if alt <= 10:
            cnt_landed += 1
        else:
            cnt_landed -= 2
        if cnt_landed >= 100:
            to_landed(queue)


# =============================================================================
# 상태 변환환
# =============================================================================

def can_transition(force: bool) -> bool:
    return force or config.STATE_OVERRIDE is None


def to_launch_pad(queue: Queue, force: bool = False):
    global state, max_alt, recent_alt
    if not can_transition(force):
        return
    state = STATE["LAUNCH_PAD"]
    max_alt = 0
    recent_alt.clear()
    log("STATE → LAUNCH_PAD")
    prevstate.update_prevstate(state)
    prevstate.update_maxalt(0)
    msgstructure.send_msg(queue, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_motor_state, str(state))


def to_ascent(queue: Queue, force: bool = False):
    global state
    if not can_transition(force):
        return
    state = STATE["ASCENT"]
    log("STATE → ASCENT")
    prevstate.update_prevstate(state)
    msgstructure.send_msg(queue, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_motor_state, str(state))
    msgstructure.send_msg(queue, appargs.FlightlogicAppArg.AppID, appargs.CameraAppArg.AppID, appargs.FlightlogicAppArg.MID_cam_activate, "")


def to_apogee(queue: Queue, force: bool = False):
    global state
    if not can_transition(force):
        return
    state = STATE["APOGEE"]
    log("STATE → APOGEE")
    prevstate.update_prevstate(state)
    msgstructure.send_msg(queue, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_motor_state, str(state))

def to_release(queue: Queue, force: bool = False):
    global state
    if not can_transition(force):
        return
    state = STATE["RELEASE"]
    log("STATE → RELEASE (burnwire activate)")
    prevstate.update_prevstate(state)
    msgstructure.send_msg(queue, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_motor_state, str(state))
    msgstructure.send_msg(queue, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_motor_burnwire, "")
    # target 좌표 재전송: motor 프로세스가 init 메시지를 놓쳤거나 재시작된 경우를 대비
    if target_lat != 0.0 or target_lon != 0.0:
        msgstructure.send_msg(queue, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID,
                              appargs.FlightlogicAppArg.MID_motor_TargetCor, f"{target_lat},{target_lon}")
        log(f"Target resent on RELEASE: ({target_lat:.6f}, {target_lon:.6f})")
    else:
        log("CRITICAL: RELEASE without target coordinates — GNC will failsafe. "
            "Set target before flight via 'CMD,XXXX,TC,lat,lon'", events.EventType.error)


def to_egg(queue: Queue, force: bool = False):
    global state, solenoid_count, solenoid_done, recent_distance
    #if not can_transition(force): 낙하테스트 + 솔레노이드 사출 테스트 용
    #    return
    state = STATE["EGG"]
    # 솔레노이드 관련 변수 초기화
    solenoid_count = 0
    solenoid_done = False
    recent_distance.clear()
    log("STATE → EGG (TF-Luna distance sensor ready for solenoid)")
    prevstate.update_prevstate(state)
    msgstructure.send_msg(queue, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_motor_state, str(state))
    msgstructure.send_msg(queue, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_motor_PullArms, "")


def to_landed(queue: Queue, force: bool = False):
    global state
    if not can_transition(force):
        return
    state = STATE["LANDED"]
    log("STATE → LANDED (motors stop)")
    prevstate.update_prevstate(state)
    msgstructure.send_msg(queue, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_motor_state, str(state))


# =============================================================================
# 주기적 전송
# =============================================================================

def send_current_state_thread(queue: Queue):
    while running:
        msgstructure.send_msg(queue, appargs.FlightlogicAppArg.AppID, appargs.CommAppArg.AppID, appargs.FlightlogicAppArg.MID_comm_state, STATE_NAMES[state])
        time.sleep(1)


# =============================================================================
# 초기화 / 종료
# =============================================================================

def init(queue: Queue):
    global state, max_alt, target_lat, target_lon
    
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    log("Initializing flightlogicapp")
    
    try:
        # 비행 로직은 별도 프로세스에서 실행되므로, 여기서 다시 읽어야 main과 동일한
        # lib/prevstate.txt 내용을 쓴다(spawn 방식·경로 혼동 시에도 일치).
        prevstate.init_prevstate()

        # 상태 복원
        if config.STATE_OVERRIDE is not None:
            state = int(config.STATE_OVERRIDE)
            log(f"Using STATE_OVERRIDE: {state}")
        else:
            state = int(prevstate.PREV_STATE)
            log(f"Using prev state: {state}")
        
        # 상태 전이 수행
        transitions = [to_launch_pad, to_ascent, to_apogee, to_release, to_egg, to_landed]
        if 0 <= state < len(transitions):
            transitions[state](queue, force=True)
        
        # 이전 데이터 복원
        if state > 0:
            max_alt = float(prevstate.PREV_MAX_ALT)
            target_lat = float(prevstate.Target_lat)
            target_lon = float(prevstate.Target_lon)
        
        # 목표 좌표 전송
        if target_lat != 0.0 or target_lon != 0.0:
            msgstructure.send_msg(queue, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID,
                                  appargs.FlightlogicAppArg.MID_motor_TargetCor, f"{target_lat},{target_lon}")
            log(f"Target restored from prevstate: ({target_lat:.6f}, {target_lon:.6f})")
        else:
            log("WARNING: Target coordinates are (0,0) — GNC will failsafe at RELEASE. "
                "Send 'CMD,XXXX,TC,lat,lon' before launch or set TARGET_LAT/LON in prevstate.txt",
                events.EventType.warning)

        log(f"Initialized with state={state}")
        
    except Exception as e:
        log(f"Init error: {e}", events.EventType.error)


def terminate():
    global running
    running = False
    log("Terminating flightlogicapp")
    
    for name, thread in threads.items():
        log(f"Joining thread: {name}")
        thread.join()
    
    log("Flightlogicapp terminated")


# =============================================================================
# 메인 루프
# =============================================================================

def flightlogicapp_main(main_queue: Queue, main_pipe: connection.Connection):
    global running
    running = True
    
    init(main_queue)
    
    # 스레드 시작
    threads["State"] = threading.Thread(target=send_current_state_thread, args=(main_queue,), daemon=True)
    for t in threads.values():
        t.start()
    
    try:
        while running:
            raw = main_pipe.recv()
            msg = msgstructure.unpack_msg(raw)

            if msg == False:
                continue

            if msg.receiver_app in (appargs.FlightlogicAppArg.AppID, appargs.MainAppArg.AppID):
                dispatch(msg, main_queue)
    
    except Exception as e:
        log(f"Error: {e}", events.EventType.error)
    
    finally:
        terminate()
