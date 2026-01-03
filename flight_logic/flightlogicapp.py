# Python FSW V3 Flightlogic App
# Author : Jeongmin Park

from lib import appargs
from lib import msgstructure
from lib import logging
from lib import events
from lib import types
from lib import prevstate
from lib import config
import math

import signal
from multiprocessing import Queue, connection
import threading
import time

# Runstatus of application. Application is terminated when false
FLIGHTLOGICAPP_RUNSTATUS = True

######################################################
## FUNDEMENTAL METHODS                              ##
######################################################


# Handles received message
def command_handler (recv_msg : msgstructure.MsgStructure, Main_Queue:Queue):
    global FLIGHTLOGICAPP_RUNSTATUS
    global SIMULATION_ENABLE
    global SIMULATION_ACTIVATE
    global MAX_ALT
    global recent_alt

    if recv_msg.MsgID == appargs.MainAppArg.MID_TerminateProcess:
        # Change Runstatus to false to start termination process
        events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, f"FLIGHTLOGICAPP TERMINATION DETECTED")
        FLIGHTLOGICAPP_RUNSTATUS = False

    elif recv_msg.MsgID == appargs.CommAppArg.MID_RouteCmd_SIM:
        # When simulation command is input
        option = recv_msg.data
        if option == "ENABLE":
            SIMULATION_ENABLE = True
            events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, "Simulation Enabled")
        if option == "ACTIVATE":
            SIMULATION_ACTIVATE = True
            events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, "Simulation Activated")
        if option == "DISABLE":
            SIMULATION_ACTIVATE = False
            SIMULATION_ENABLE = False
            events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, "Simulation Disabled")
            
        check_simulation_status(Main_Queue)

    # Simulation pressure data
    elif recv_msg.MsgID == appargs.CommAppArg.MID_RouteCmd_SIMP:
        if SIMULATION_ACTIVATE and SIMULATION_ENABLE:

            simulated_altitude = float(recv_msg.data)
            
            barometer_logic(Main_Queue, simulated_altitude)
        else:
            return

    # Receive Sensor data
    # Barometer Data input at 10Hz
    elif recv_msg.MsgID == appargs.BarometerAppArg.MID_SendBarometerFlightLogicData:
        # Ignore the barometer data when simulation is activated
        if SIMULATION_ENABLE and SIMULATION_ACTIVATE:
            return
        
        recv_altitude = float(recv_msg.data)

        # Perform barometer logic
        barometer_logic(Main_Queue, recv_altitude)

    elif recv_msg.MsgID == appargs.GpsAppArg.MID_SendGpsFlightLogicData:
        # Ignore the gps data when simulation is activated
        if SIMULATION_ENABLE and SIMULATION_ACTIVATE:
            return
        
        sep_data = recv_msg.data.split(",")
        
        # Check the length of separated data
        if (len(sep_data) != 2):
            events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.error, f"ERROR receiving GPS flight logic data, expected 2 fields")
            return
        
        recv_lat = float(sep_data[0])
        recv_lon = float(sep_data[1])

        # Perform GPS logic
        gps_logic(Main_Queue, recv_lat, recv_lon)
    
    elif recv_msg.MsgID == appargs.ImuAppArg.MID_SendImuFlightLogicData:
        # Ignore the IMU data when simulation is activated
        if SIMULATION_ENABLE and SIMULATION_ACTIVATE:
            return
        
        recv_yaw = float(recv_msg.data)

        # Perform IMU logic
        imu_logic(Main_Queue, recv_yaw)

    elif recv_msg.MsgID == appargs.CommAppArg.MID_RouteCmd_SS:
        # When received Set State command
        recv_state = int(recv_msg.data)

        # LAUNCHPAD
        if recv_state == 0:
            launchpad_state_transition(Main_Queue)

        # ASCENT
        elif recv_state == 1:
            ascent_state_transition(Main_Queue)

        # APOGEE
        elif recv_state == 2:
            apogee_state_transition(Main_Queue)

        # DESCENT
        elif recv_state == 3:
            descent_state_transition(Main_Queue)

        # PROBE RELEASE
        elif recv_state == 4:
            probe_release_state_transition(Main_Queue)

        # LANDED
        elif recv_state == 5:
            landed_state_transition(Main_Queue)
            
        # Received State out of state range
        else: 
            events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.error, f"Error receiving SS message, Expected state value of 0 ~ {len(STATE_LIST) - 1}, received {recv_state}")

    # When reset message from barometer app is received, set the max alt to 0
    elif recv_msg.MsgID == appargs.BarometerAppArg.MID_ResetBarometerMaxAlt:
        MAX_ALT = 0
        recent_alt.clear()

    else:
        events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.error, f"MID {recv_msg.MsgID} not handled")
    return

def send_hk(Main_Queue : Queue):
    global FLIGHTLOGICAPP_RUNSTATUS
    while FLIGHTLOGICAPP_RUNSTATUS:
        flightlogicHK = msgstructure.MsgStructure()
        msgstructure.send_msg(Main_Queue, flightlogicHK, appargs.FlightlogicAppArg.AppID, appargs.HkAppArg.AppID, appargs.FlightlogicAppArg.MID_SendHK, str(FLIGHTLOGICAPP_RUNSTATUS))
        time.sleep(1)
    return

######################################################
## INITIALIZATION, TERMINATION                      ##
######################################################

# Initialization
def flightlogicapp_init(Main_Queue : Queue):
    try:
        global FLIGHTLOGICAPP_RUNSTATUS
        global CURRENT_STATE
        global MAX_ALT
        global Target_lat
        global Target_lon

        # Disable Keyboardinterrupt since Termination is handled by parent process
        signal.signal(signal.SIGINT, signal.SIG_IGN)

        events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, "Initializating flightlogicapp")
        ## User Defined Initialization goes HERE

        # Check if STATE_OVERRIDE is set in config.txt
        if config.STATE_OVERRIDE is not None:
            # Use STATE_OVERRIDE if specified in config
            CURRENT_STATE = int(config.STATE_OVERRIDE)
            events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, f"Using STATE_OVERRIDE from config: {CURRENT_STATE}")
        else:
            # For recovery, set the prev state as the current state.
            CURRENT_STATE = int(prevstate.PREV_STATE)
            events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, f"Using prev state: {CURRENT_STATE}")

        # Perform state transition according to state
        if CURRENT_STATE == 0:
            launchpad_state_transition(Main_Queue)
        elif CURRENT_STATE == 1:
            ascent_state_transition(Main_Queue)
        elif CURRENT_STATE == 2:
            apogee_state_transition(Main_Queue)
        elif CURRENT_STATE == 3:
            descent_state_transition(Main_Queue)
        elif CURRENT_STATE == 4:
            probe_release_state_transition(Main_Queue)
        elif CURRENT_STATE == 5:
            landed_state_transition(Main_Queue)
            
        # For recovery set the max altitude
        MAX_ALT = float(prevstate.PREV_MAX_ALT)
        Target_lat = float(prevstate.Target_lat)
        Target_lon = float(prevstate.Target_lon)

        events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, f"Setting Current state to {CURRENT_STATE}")

        events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, "Flightlogicapp Initialization Complete")
    except Exception as e:
        events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.error, f"Error during initialization: {e}")
        FLIGHTLOGICAPP_RUNSTATUS = False

# Termination
def flightlogicapp_terminate():
    global FLIGHTLOGICAPP_RUNSTATUS

    FLIGHTLOGICAPP_RUNSTATUS = False
    events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, "Terminating flightlogicapp")
    # Termination Process Comes Here

    # Join Each Thread to make sure all threads terminates
    for thread_name in thread_dict:
        events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, f"Terminating thread {thread_name}")
        thread_dict[thread_name].join()
        events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, f"Terminating thread {thread_name} Complete")

    # The termination flag should switch to false AFTER ALL TERMINATION PROCESS HAS ENDED
    events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, "Terminating flightlogicapp complete")
    return

######################################################
## USER METHOD                                      ##
######################################################

STATE_LIST = ["LAUNCH_PAD",
              "ASCENT",
              "APOGEE",
              "DESCENT",
              "PROBE_RELEASE",
              "LANDED"]

CURRENT_STATE = 0

# Simulation Mode Flags, both ENABLE and ACTIVATE should be true to use simulation mode
SIMULATION_ENABLE = False
SIMULATION_ACTIVATE = False

# Variable used in state determination
MAX_ALT = 0
PAYLOAD_SEP_ALT_THRESHOLD = 0
EGG_DROP_ALT_THRESHOLD = 2.0  # Drop egg at 2m above ground
TARGET_REACHED_RADIUS = 50.0  # Target reached radius in meters

BAROMETER_ASCENT_COUNTER = 0
BAROMETER_APOGEE_COUNTER = 0
BAROMETER_DESCENT_COUNTER = 0
BAROMETER_PROBE_RELEASE_COUNTER = 0
BAROMETER_LANDED_COUNTER = 0
BAROMETER_EGG_DROP_COUNTER = 0

# Flag to track if egg motor has been activated (prevent multiple activations)
EGG_MOTOR_ACTIVATED = False
TARGET_REACHED = False  # Flag to track if target GPS location has been reached

recent_alt = []

def barometer_logic(Main_Queue:Queue, altitude:float):
    global MAX_ALT
    global PAYLOAD_SEP_ALT_THRESHOLD
    global CURRENT_STATE
    global BAROMETER_ASCENT_COUNTER
    global BAROMETER_DESCENT_COUNTER
    global BAROMETER_APOGEE_COUNTER
    global BAROMETER_PROBE_RELEASE_COUNTER
    global BAROMETER_LANDED_COUNTER
    
    global recent_alt

    recent_alt.append(altitude)
    if len(recent_alt) > 3:
        recent_alt.pop(0)

    if len(recent_alt) > 2:
        sorted_alts = sorted(recent_alt, reverse=True)
        second_max = sorted_alts[1]

        if sorted_alts[1] > MAX_ALT:
            MAX_ALT = second_max
            prevstate.update_maxalt(second_max)

    PAYLOAD_SEP_ALT_THRESHOLD = MAX_ALT * 0.75
    
    if len(recent_alt) < 3:
        # Do not perform any logic before filter is enabled
        return

    if BAROMETER_ASCENT_COUNTER <= 0 :
        BAROMETER_ASCENT_COUNTER = 0

    if BAROMETER_APOGEE_COUNTER <= 0 :
        BAROMETER_APOGEE_COUNTER = 0

    if BAROMETER_DESCENT_COUNTER <= 0 :
        BAROMETER_DESCENT_COUNTER = 0
    
    if BAROMETER_PROBE_RELEASE_COUNTER <= 0:
        BAROMETER_PROBE_RELEASE_COUNTER = 0

    if BAROMETER_LANDED_COUNTER <= 0 :
        BAROMETER_LANDED_COUNTER = 0
        
    # At Standby State
    if CURRENT_STATE == 0:

        # When Altitude is higher than 75 meters
        if (altitude > 75):
            BAROMETER_ASCENT_COUNTER += 1
        else:
            BAROMETER_ASCENT_COUNTER -= 2
        
        # When the counter is larger than 3 ; when the altitude is higher than 50 meter 3 times in a row
        if BAROMETER_ASCENT_COUNTER >= 3:
            ascent_state_transition(Main_Queue)
    
    # At Ascent State
    if CURRENT_STATE == 1:

        # When Altitude is 20 meter lower than max altitude

        if (altitude <= MAX_ALT - 20):
            BAROMETER_DESCENT_COUNTER += 1
        else:
            BAROMETER_DESCENT_COUNTER -= 2
        
        # 0.25 meter is the resolution of BMP390
        if (altitude < MAX_ALT - 0.25 and altitude > MAX_ALT - 20):
            BAROMETER_APOGEE_COUNTER += 1
        else:
            BAROMETER_APOGEE_COUNTER -= 2

        # When the counter is larger than 2; When the altitude is 2 meter lower than max altitude 2 times in a row
        if BAROMETER_DESCENT_COUNTER >= 2:
            descent_state_transition(Main_Queue)

        if BAROMETER_APOGEE_COUNTER >= 2:
            apogee_state_transition(Main_Queue)

    # At Apogee State
    if CURRENT_STATE == 2:
        
        # Check for descent
        if (altitude <= MAX_ALT - 20):
            BAROMETER_DESCENT_COUNTER += 1
        else:
            BAROMETER_DESCENT_COUNTER -= 2

        # When the counter is larger than 3; When the altitude is 2 meter lower than max altitude 3 times in a row
        if BAROMETER_DESCENT_COUNTER >= 2:
            descent_state_transition(Main_Queue)

    # At Descent State
    if CURRENT_STATE == 3:
        # When Altitude is 75% of max altitude

        if (altitude <= MAX_ALT * 0.75):
            BAROMETER_PROBE_RELEASE_COUNTER += 1
        else:
            BAROMETER_PROBE_RELEASE_COUNTER -= 2

        # When the counter is larger than 3; When the altitude is 75% of max altitude 2 times in a row
        if BAROMETER_PROBE_RELEASE_COUNTER >= 2:
            probe_release_state_transition(Main_Queue)
    
    # At Probe Release State

    if CURRENT_STATE == 4:
        global EGG_MOTOR_ACTIVATED
        global BAROMETER_EGG_DROP_COUNTER
        global TARGET_REACHED
        
        # Only activate egg motor if target has been reached AND altitude is 2m above ground
        # Mission: Reach target GPS location, then drop egg at 2m altitude
        if not EGG_MOTOR_ACTIVATED and TARGET_REACHED and altitude <= EGG_DROP_ALT_THRESHOLD:
            BAROMETER_EGG_DROP_COUNTER += 1
        else:
            BAROMETER_EGG_DROP_COUNTER -= 2
        
        # Activate egg motor when target reached AND altitude is 2m or below (2 times in a row)
        if not EGG_MOTOR_ACTIVATED and TARGET_REACHED and BAROMETER_EGG_DROP_COUNTER >= 2:
            events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, f"Target reached and egg drop altitude reached ({altitude:.2f}m), activating egg ejection motor")
            PayloadEggMotorActivateMsg = msgstructure.MsgStructure()
            msgstructure.send_msg(Main_Queue, PayloadEggMotorActivateMsg, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_PayloadEggMotorActivate, "")
            EGG_MOTOR_ACTIVATED = True
        elif not TARGET_REACHED and altitude <= EGG_DROP_ALT_THRESHOLD:
            # Log that we're at drop altitude but haven't reached target yet
            events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, f"At drop altitude ({altitude:.2f}m) but target not yet reached - waiting for target")
        
        # When altitude is almost zero (landed)
        if altitude <= 15:
            BAROMETER_LANDED_COUNTER += 1
        else:
            BAROMETER_LANDED_COUNTER -= 2

        if BAROMETER_LANDED_COUNTER >= 3:
            landed_state_transition(Main_Queue)
        
    return

Target_Yaw = 0
Recent_lat = 0.0
Recent_lon = 0.0
GPS_VALID = False
Last_Valid_GPS_Angle = None  # 마지막 유효한 GPS 방위각 (GPS 손실 시 사용)

def imu_logic(Main_Queue:Queue, recent_yaw:float):
    global Target_Yaw
    global Recent_lat
    global Recent_lon
    global GPS_VALID
    global Last_Valid_GPS_Angle
    global CURRENT_STATE

    Target_Yaw = recent_yaw
    
    # 파라포일 모터 제어는 DESCENT(3) 또는 PROBE_RELEASE(4) 상태에서만 활성화
    # simulation과 동일하게 동작하도록 모든 상태에서 모터 제어 가능하도록 설정
    # (필요시 특정 상태에서만 제어하려면 아래 주석 해제)
    # if CURRENT_STATE < 3:  # DESCENT 이전 상태에서는 모터 제어 안 함
    #     return
    
    # IMU 데이터가 올 때마다 (100Hz) 최신 GPS 좌표로 모터 제어
    # GPS 유효성 검사 후 처리
    if GPS_VALID and is_gps_valid(Recent_lat, Recent_lon):
        # GPS가 유효한 경우: GPS 기반 모터 제어 (simulation과 동일)
        control_motor_with_gps(Main_Queue, Recent_lat, Recent_lon)
    elif Last_Valid_GPS_Angle is not None:
        # GPS가 유효하지 않지만 이전에 유효한 GPS 방위각이 있는 경우
        # 마지막 유효한 방위각을 유지하도록 모터 제어
        control_motor_with_heading(Main_Queue, Last_Valid_GPS_Angle)
    else:
        # GPS가 없고 이전 유효한 방위각도 없는 경우: 모터 정지
        control_motor_stop(Main_Queue)
    
    return

def is_gps_valid(lat: float, lon: float) -> bool:
    """
    Check if GPS coordinates are valid.
    Returns True if GPS data is valid (not 0.0, 0.0 and within reasonable ranges).
    """
    # Check if coordinates are not zero (invalid GPS reading)
    if lat == 0.0 and lon == 0.0:
        return False
    
    if abs(lat) > 90.0 or abs(lon) > 180.0:
        return False
    
    return True

def calculate_distance_haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate distance between two GPS coordinates using Haversine formula.
    Returns distance in meters.
    """
    # Earth radius in meters
    R = 6371000  # meters
    
    # Convert latitude and longitude from degrees to radians
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    # Haversine formula
    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    distance = R * c
    return distance

def control_motor_with_gps(Main_Queue:Queue, recent_lat:float, recent_lon:float):
    global Target_lat
    global Target_lon
    global Target_Yaw
    global TARGET_REACHED
    global TARGET_REACHED_RADIUS
    global Last_Valid_GPS_Angle

    # Skip motor control if target coordinates are not set (0, 0)
    if Target_lat == 0.0 and Target_lon == 0.0:
        return

    # Calculate bearing angle to target (simulation의 calculate_bearing_angle과 동일)
    # atan2(y, x) returns angle from x-axis, so we use (lat_diff, lon_diff)
    # Convert to degrees and normalize to 0-360 range
    gps_angle_rad = math.atan2(Target_lat - recent_lat, Target_lon - recent_lon)
    gps_angle_deg = math.degrees(gps_angle_rad)  # Convert radians to degrees
    
    # Normalize gps_angle to 0-360 degrees
    if gps_angle_deg < 0:
        gps_angle_deg += 360
    
    Last_Valid_GPS_Angle = gps_angle_deg
    
    # Calculate turn angle (simulation의 calculate_turn_angle과 동일)
    # current_yaw - gps_bearing: positive = turn right, negative = turn left
    angle_diff = Target_Yaw - gps_angle_deg
    
    # Normalize angle difference to -180 to +180 range (shortest path)
    while angle_diff > 180:
        angle_diff -= 360
    while angle_diff < -180:
        angle_diff += 360
    
    # turn > 0 means need to turn right (use right motor)
    # turn < 0 means need to turn left (use left motor)
    turn = angle_diff

    # Convert turn value to string (send_msg requires str, not list)
    turn_data = str(turn)

    # Send motor control command (simulation과 동일하게 모터 제어)
    SendPayloadMotorRotation = msgstructure.MsgStructure()
    msgstructure.send_msg(Main_Queue, SendPayloadMotorRotation, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_SendPayloadMotorRatation, turn_data)
    return

def control_motor_with_heading(Main_Queue:Queue, target_heading:float):
    """
    GPS가 없을 때 마지막 유효한 방위각을 유지하도록 모터 제어.
    도심 등 GPS 수신 불가 상황에서 사용.
    """
    global Target_Yaw
    
    # 현재 yaw와 목표 방위각의 차이 계산
    angle_diff = Target_Yaw - target_heading
    
    # Normalize angle difference to -180 to +180 range (shortest path)
    while angle_diff > 180:
        angle_diff -= 360
    while angle_diff < -180:
        angle_diff += 360
    
    turn = angle_diff
    
    # Convert turn value to string
    turn_data = str(turn)
    
    SendPayloadMotorRotation = msgstructure.MsgStructure()
    msgstructure.send_msg(Main_Queue, SendPayloadMotorRotation, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_SendPayloadMotorRatation, turn_data)
    return

def control_motor_stop(Main_Queue:Queue):
    """
    GPS가 없고 이전 유효한 방위각도 없을 때 모터 정지.
    안전을 위해 모터를 정지시킴.
    """
    # turn = 0 means stop motors
    turn_data = "0.0"
    
    SendPayloadMotorRotation = msgstructure.MsgStructure()
    msgstructure.send_msg(Main_Queue, SendPayloadMotorRotation, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_SendPayloadMotorRatation, turn_data)
    return

def gps_logic(Main_Queue:Queue, recent_lat:float, recent_lon:float):
    global Target_lat
    global Target_lon
    global Target_Yaw
    global TARGET_REACHED
    global TARGET_REACHED_RADIUS
    global Recent_lat
    global Recent_lon
    global GPS_VALID

    # GPS 데이터 유효성 검사
    gps_valid = is_gps_valid(recent_lat, recent_lon)
    
    # GPS 유효성 상태 변경 시 로그 출력
    if gps_valid != GPS_VALID:
        if gps_valid:
            events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, f"GPS signal acquired: Lat={recent_lat:.6f}, Lon={recent_lon:.6f}")
        else:
            events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.warning, f"GPS signal lost or invalid: Lat={recent_lat:.6f}, Lon={recent_lon:.6f} (도심 등 GPS 수신 불가)")
    
    GPS_VALID = gps_valid
    
    # 최신 GPS 좌표 저장 (IMU 기반 모터 제어용)
    # 유효하지 않은 GPS도 저장하되, 유효성 플래그로 구분
    Recent_lat = recent_lat
    Recent_lon = recent_lon

    # Skip GPS logic if target coordinates are not set (0, 0)
    if Target_lat == 0.0 and Target_lon == 0.0:
        return

    # GPS가 유효한 경우에만 거리 계산 및 목표 도달 체크
    if GPS_VALID:
        # Calculate distance to target using Haversine formula (in meters)
        distance_to_target = calculate_distance_haversine(recent_lat, recent_lon, Target_lat, Target_lon)
        
        # Check if target has been reached (within target radius)
        if not TARGET_REACHED and distance_to_target <= TARGET_REACHED_RADIUS:
            TARGET_REACHED = True
            events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, f"Target reached! Distance: {distance_to_target:.2f}m (within {TARGET_REACHED_RADIUS}m radius)")
        
        control_motor_with_gps(Main_Queue, recent_lat, recent_lon)
    
    return

def launchpad_state_transition(Main_Queue : Queue):
    global CURRENT_STATE
    global MAX_ALT
    global recent_alt

    # Set the Current State to 0 ; Standby
    CURRENT_STATE = 0
    # Set the max alt to 0 and clear the recent alt list
    MAX_ALT = 0
    recent_alt.clear()

    events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, "CHANGED STATE TO STANDBY")
    
    # Store the current state to prev state file
    prevstate.update_prevstate(CURRENT_STATE)

    # Reset the mechanism depending on the FSW config
    if config.FSW_CONF == config.CONF_CONTAINER:
        PayloadReleaseMotorStandbyMsg = msgstructure.MsgStructure()
        msgstructure.send_msg(Main_Queue, PayloadReleaseMotorStandbyMsg, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_PayloadReleaseMotorStandby, "")

    return

def ascent_state_transition(Main_Queue : Queue):
    global CURRENT_STATE
    
    # Set the Current State to 1 ; Ascent
    CURRENT_STATE = 1
    events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, "CHANGED STATE TO ASCENT")

    # Store the current state to prev state file
    prevstate.update_prevstate(CURRENT_STATE)

    # Perform Action for Ascent State

    ActivateCameraToCamappMsg = msgstructure.MsgStructure()
    msgstructure.send_msg(Main_Queue, ActivateCameraToCamappMsg, appargs.FlightlogicAppArg.AppID, appargs.CameraAppArg.AppID, appargs.FlightlogicAppArg.MID_SendCameraActivateToCam, "")

    return

def apogee_state_transition(Main_Queue : Queue):
    global CURRENT_STATE

    # Set the Current State to 2 ; Apogee
    CURRENT_STATE = 2

    events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, "CHANGED STATE TO APOGEE")
    prevstate.update_prevstate(CURRENT_STATE)


def descent_state_transition(Main_Queue:Queue):
    global CURRENT_STATE
    
    # Set the Current State to 3 ; Deploy
    CURRENT_STATE = 3
    events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, "CHANGED STATE TO DESCENT")

    # Store the current state to prev state file
    prevstate.update_prevstate(CURRENT_STATE)

    # Perform Action for Deploy State

    ActivateCameraToCamappMsg = msgstructure.MsgStructure()
    msgstructure.send_msg(Main_Queue, ActivateCameraToCamappMsg, appargs.FlightlogicAppArg.AppID, appargs.CameraAppArg.AppID, appargs.FlightlogicAppArg.MID_SendCameraActivateToCam, "")


    return

def probe_release_state_transition(Main_Queue:Queue):
    global CURRENT_STATE
    
    # Set the Current State to 4 ; Payload Sep
    CURRENT_STATE = 4
    events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, "CHANGED STATE TO PROBE RELEASE")

    # Store the current state to prev state file
    prevstate.update_prevstate(CURRENT_STATE)

    # Perform Action for Payload Separation State

    ActivateCameraToCamappMsg = msgstructure.MsgStructure()
    msgstructure.send_msg(Main_Queue, ActivateCameraToCamappMsg, appargs.FlightlogicAppArg.AppID, appargs.CameraAppArg.AppID, appargs.FlightlogicAppArg.MID_SendCameraActivateToCam, "")

    PayloadReleaseMotorActivateMsg = msgstructure.MsgStructure()
    msgstructure.send_msg(Main_Queue, PayloadReleaseMotorActivateMsg, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_PayloadReleaseMotorActivate, "")
    
    # Container -> Activate Motor to release payload

    return

def landed_state_transition(Main_Queue : Queue):
    global CURRENT_STATE
    
    # Set the Current State to 5 ; Landing
    CURRENT_STATE = 5
    events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, "CHANGED STATE TO LANDED")

    # Store the current state to prev state file
    prevstate.update_prevstate(CURRENT_STATE)

    # Perform Action for Landing State

    return

def check_simulation_status(Main_Queue:Queue):
    simulation_status = "F"

    if SIMULATION_ENABLE and SIMULATION_ACTIVATE:
        simulation_status = "S"
        
    SimulationStatusToTlmMsg = msgstructure.MsgStructure()
    msgstructure.send_msg(Main_Queue, SimulationStatusToTlmMsg, appargs.FlightlogicAppArg.AppID, appargs.CommAppArg.AppID, appargs.FlightlogicAppArg.MID_SendSimulationStatustoTlm, simulation_status)   

# Put user-defined methods here!

def send_current_state(Main_Queue:Queue):
    global FLIGHTLOGICAPP_RUNSTATUS
    global CURRENT_STATE

    SendCurrentStateMsg = msgstructure.MsgStructure()
    while FLIGHTLOGICAPP_RUNSTATUS:
        msgstructure.send_msg(Main_Queue, SendCurrentStateMsg, appargs.FlightlogicAppArg.AppID, appargs.CommAppArg.AppID, appargs.FlightlogicAppArg.MID_SendCurrentStateToTlm, STATE_LIST[CURRENT_STATE])
        time.sleep(1)
    
    return
######################################################
## MAIN METHOD                                      ##
######################################################

thread_dict = dict[str, threading.Thread]()

# This method is called from main app. Initialization, runloop process
def flightlogicapp_main(Main_Queue : Queue, Main_Pipe : connection.Connection):
    global FLIGHTLOGICAPP_RUNSTATUS
    FLIGHTLOGICAPP_RUNSTATUS = True

    # Initialization Process
    flightlogicapp_init(Main_Queue)

    # Spawn SB Message Listner Thread
    thread_dict["HKSender_Thread"] = threading.Thread(target=send_hk, args=(Main_Queue, ), name="HKSender_Thread")
    thread_dict["SendCurrentState_Thread"] = threading.Thread(target=send_current_state, args=(Main_Queue, ), name="SendCurrentState_Thread")

    # Spawn Each Threads
    for thread_name in thread_dict:
        thread_dict[thread_name].start()

    try:
        while FLIGHTLOGICAPP_RUNSTATUS:
            # Receive Message From Pipe
            message = Main_Pipe.recv()
            recv_msg = msgstructure.MsgStructure()

            # Unpack Message, Skip this message if unpacked message is not valid
            if msgstructure.unpack_msg(recv_msg, message) == False:
                continue
            
            # Validate Message, Skip this message if target AppID different from flightlogicapp's AppID
            # Exception when the message is from main app
            if recv_msg.receiver_app == appargs.FlightlogicAppArg.AppID or recv_msg.receiver_app == appargs.MainAppArg.AppID:
                # Handle Command According to Message ID
                command_handler(recv_msg, Main_Queue)
            else:
                events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.error, "Receiver MID does not match with flightlogicapp MID")

    # If error occurs, terminate app
    except Exception as e:
        events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.error, f"flightlogicapp error : {e}")
        FLIGHTLOGICAPP_RUNSTATUS = False

    # Termination Process after runloop
    flightlogicapp_terminate()

    return
