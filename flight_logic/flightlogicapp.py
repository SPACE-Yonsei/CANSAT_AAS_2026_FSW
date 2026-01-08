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

# Import parafoil_control for GPS/IMU logic
from Sensor_Motor import parafoil_control

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

        # Send GPS data to motorapp
        SendGpsMotorDataMsg = msgstructure.MsgStructure()
        msgstructure.send_msg(Main_Queue, SendGpsMotorDataMsg, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_SendGpsMotorData, f"{recv_lat},{recv_lon}")
        
        # GPS logic for target reached check (using parafoil_control)
        global Target_lat, Target_lon, TARGET_REACHED, TARGET_REACHED_RADIUS
        if Target_lat != 0.0 or Target_lon != 0.0:
            if parafoil_control.is_gps_valid(recv_lat, recv_lon):
                distance_to_target = parafoil_control.calculate_distance_haversine(recv_lat, recv_lon, Target_lat, Target_lon)
                if not TARGET_REACHED and distance_to_target <= TARGET_REACHED_RADIUS:
                    TARGET_REACHED = True
                    events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, f"Target reached! Distance: {distance_to_target:.2f}m (within {TARGET_REACHED_RADIUS}m radius)")    
    
    elif recv_msg.MsgID == appargs.ImuAppArg.MID_SendImuFlightLogicData:
        # Ignore the IMU data when simulation is activated
        if SIMULATION_ENABLE and SIMULATION_ACTIVATE:
            return
        events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, f"Received IMU data: {recv_msg.data}")
        
        recv_yaw = float(recv_msg.data)
        SendImuMotorDataMsg = msgstructure.MsgStructure()
        msgstructure.send_msg(Main_Queue, SendImuMotorDataMsg, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_SendImuMotorData, f"{recv_yaw}")

    elif recv_msg.MsgID == appargs.CommAppArg.MID_RouteCmd_SS:
        # When received Set State command
        recv_state = int(recv_msg.data)

        # LAUNCHPAD (force=True for SS command)
        if recv_state == 0:
            launchpad_state_transition(Main_Queue, force=True)

        # ASCENT
        elif recv_state == 1:
            Ascent_state_transition(Main_Queue, force=True)

        # APOGEE
        elif recv_state == 2:
            Apogee_state_transition(Main_Queue, force=True)

        # DESCENT
        elif recv_state == 3:
            Release_state_transition(Main_Queue, force=True)

        # PROBE RELEASE
        elif recv_state == 4:
            Egg_state_transition(Main_Queue, force=True)

        # LANDED
        elif recv_state == 5:
            landed_state_transition(Main_Queue, force=True)
            
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

        # Perform state transition according to state (force=True for initialization)
        if CURRENT_STATE == 0:
            launchpad_state_transition(Main_Queue, force=True)
        elif CURRENT_STATE == 1:
            Ascent_state_transition(Main_Queue, force=True)
        elif CURRENT_STATE == 2:
            Apogee_state_transition(Main_Queue, force=True)
        elif CURRENT_STATE == 3:
            Release_state_transition(Main_Queue, force=True)
        elif CURRENT_STATE == 4:
            Egg_state_transition(Main_Queue, force=True)
        elif CURRENT_STATE == 5:
            landed_state_transition(Main_Queue, force=True)
            
        # For recovery set the max altitude
        MAX_ALT = float(prevstate.PREV_MAX_ALT)
        Target_lat = float(prevstate.Target_lat)
        Target_lon = float(prevstate.Target_lon)
        
        # Set target coordinates to motorapp
        if Target_lat != 0.0 or Target_lon != 0.0:
            SetTargetCoordsMsg = msgstructure.MsgStructure()
            target_coords_data = f"{Target_lat},{Target_lon}"
            msgstructure.send_msg(Main_Queue, SetTargetCoordsMsg, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_SetTargetCoordinates, target_coords_data)

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
              "RELEASE",
              "EGG",
              "LANDED"]

CURRENT_STATE = 0

# Simulation Mode Flags, both ENABLE and ACTIVATE should be true to use simulation mode
SIMULATION_ENABLE = False
SIMULATION_ACTIVATE = False

# Variable used in state determination
MAX_ALT = 0
PAYLOAD_SEP_ALT_THRESHOLD = 0
EGG_DROP_ALT_THRESHOLD = 2.0  # Drop egg at 2m above ground
SOLENOID_SAFETY_ALT_MIN = 3.0  # Start solenoid safety activation at 3m
SOLENOID_SAFETY_ALT_MAX = 4.0  # End solenoid safety activation at 4m
TARGET_REACHED_RADIUS = 50.0  # Target reached radius in meters

BAROMETER_ASCENT_COUNTER = 0
BAROMETER_APOGEE_COUNTER = 0
BAROMETER_DESCENT_COUNTER = 0
BAROMETER_PROBE_RELEASE_COUNTER = 0
BAROMETER_LANDED_COUNTER = 0
BAROMETER_EGG_DROP_COUNTER = 0

# Flag to track if egg motor has been activated (prevent multiple activations)
EGG_MOTOR_ACTIVATED = False
SOLENOID_ACTIVATION_COUNT = 0  # Count of solenoid activations for safety (3-4m)
SOLENOID_SAFETY_COMPLETE = False  # Flag to track if safety solenoid activation is complete
TARGET_REACHED = False  # Flag to track if target GPS location has been reached

# Target GPS coordinates (initialized from prevstate)
Target_lat = 0.0
Target_lon = 0.0

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

    PAYLOAD_SEP_ALT_THRESHOLD = MAX_ALT * 0.80
    
    if len(recent_alt) < 3:
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
        
    # At LaunchPad State 
    if CURRENT_STATE == 0:
        if (altitude > 50):
            BAROMETER_ASCENT_COUNTER += 1
        else:
            BAROMETER_ASCENT_COUNTER -= 2
        
        # When the counter is larger than 3 ; when the altitude is higher than 150 meter 3 times in a row
        if BAROMETER_ASCENT_COUNTER >= 3:
            Ascent_state_transition(Main_Queue)
    
    # At Fly State
    if CURRENT_STATE == 1:
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
            Release_state_transition(Main_Queue)

        if BAROMETER_APOGEE_COUNTER >= 2:
            Apogee_state_transition(Main_Queue)

    # At Apogee State
    if CURRENT_STATE == 2:
        if (altitude <= MAX_ALT - 20):
            BAROMETER_DESCENT_COUNTER += 1
        else:
            BAROMETER_DESCENT_COUNTER -= 2

        # When the counter is larger than 3; When the altitude is 2 meter lower than max altitude 3 times in a row
        if BAROMETER_DESCENT_COUNTER >= 2:
            Release_state_transition(Main_Queue)

    # At Descent State
    if CURRENT_STATE == 3:
        if (altitude <= MAX_ALT * 0.80):
            BAROMETER_PROBE_RELEASE_COUNTER += 1
        else:
            BAROMETER_PROBE_RELEASE_COUNTER -= 2

        # When the counter is larger than 3; When the altitude is 75% of max altitude 2 times in a row
        if BAROMETER_PROBE_RELEASE_COUNTER >= 2:
            Egg_state_transition(Main_Queue)
    
    # At Probe Release State

    if CURRENT_STATE == 4:
        global EGG_MOTOR_ACTIVATED
        global BAROMETER_EGG_DROP_COUNTER
        global TARGET_REACHED
        global SOLENOID_ACTIVATION_COUNT
        global SOLENOID_SAFETY_COMPLETE
        
        # Safety solenoid activation: 3-4m altitude, activate 5-6 times repeatedly
        if not SOLENOID_SAFETY_COMPLETE and SOLENOID_SAFETY_ALT_MIN <= altitude <= SOLENOID_SAFETY_ALT_MAX:
            if SOLENOID_ACTIVATION_COUNT < 6:  # Activate up to 6 times
                events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, f"Safety solenoid activation ({SOLENOID_ACTIVATION_COUNT + 1}/6) at altitude {altitude:.2f}m")
                SolenoidActivateMsg = msgstructure.MsgStructure()
                msgstructure.send_msg(Main_Queue, SolenoidActivateMsg, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_SolenoidActivate, "")
                SOLENOID_ACTIVATION_COUNT += 1
                if SOLENOID_ACTIVATION_COUNT >= 5:  # Complete after 5-6 activations
                    SOLENOID_SAFETY_COMPLETE = True
                    events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, f"Safety solenoid activation complete ({SOLENOID_ACTIVATION_COUNT} times)")
        
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

def launchpad_state_transition(Main_Queue : Queue, force: bool = False):
    global CURRENT_STATE
    global MAX_ALT
    global recent_alt

    # STATE_OVERRIDE가 설정되어 있으면 강제 호출이 아닌 경우 상태 변화 차단
    if config.STATE_OVERRIDE is not None and not force:
        return

    # Set the Current State to 0 ; Standby
    CURRENT_STATE = 0
    # Set the max alt to 0 and clear the recent alt list
    MAX_ALT = 0
    recent_alt.clear()

    events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, "CHANGED STATE TO STANDBY")
    
    # Store the current state to prev state file
    prevstate.update_prevstate(CURRENT_STATE)
    
    # Send state to motorapp
    send_flight_state_to_motor(Main_Queue, CURRENT_STATE)

    return

def Ascent_state_transition(Main_Queue : Queue, force: bool = False):
    global CURRENT_STATE
    
    if config.STATE_OVERRIDE is not None and not force:
        return

    CURRENT_STATE = 1
    events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, "CHANGED STATE TO ASCENT")

    # Store the current state to prev state file
    prevstate.update_prevstate(CURRENT_STATE)
    
    # Send state to motorapp
    send_flight_state_to_motor(Main_Queue, CURRENT_STATE)
    ActivateCameraToCamappMsg = msgstructure.MsgStructure()
    msgstructure.send_msg(Main_Queue, ActivateCameraToCamappMsg, appargs.FlightlogicAppArg.AppID, appargs.CameraAppArg.AppID, appargs.FlightlogicAppArg.MID_SendCameraActivateToCam, "")

    return

def Apogee_state_transition(Main_Queue : Queue, force: bool = False):
    global CURRENT_STATE

    if config.STATE_OVERRIDE is not None and not force:
        return
    CURRENT_STATE = 2
    prevstate.update_prevstate(CURRENT_STATE)

    events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, "CHANGED STATE TO APOGEE")
    
    # Send state to motorapp
    send_flight_state_to_motor(Main_Queue, CURRENT_STATE)
    MotorParafoilActivateMsg = msgstructure.MsgStructure()
    msgstructure.send_msg(Main_Queue, MotorParafoilActivateMsg, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_Motor_Parafoil_Activate, "")
    


def Release_state_transition(Main_Queue:Queue, force: bool = False):
    # 이때 번와이어로 컨테이너-페이로드 사출
    # 파라포일 모터 전개 시작작
    global CURRENT_STATE
    
    if config.STATE_OVERRIDE is not None and not force:
        return

    CURRENT_STATE = 3
    prevstate.update_prevstate(CURRENT_STATE)
    events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, "CHANGED STATE TO RELEASE")
    
    # Send state to motorapp
    send_flight_state_to_motor(Main_Queue, CURRENT_STATE)


    MotorReleaseActivateMsg = msgstructure.MsgStructure()
    msgstructure.send_msg(Main_Queue, MotorReleaseActivateMsg, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_Motor_Release_Activate, "")


    return

def Egg_state_transition(Main_Queue:Queue, force: bool = False):
    # 상공 2m에서 계란 사출
    global CURRENT_STATE
    
    if config.STATE_OVERRIDE is not None and not force:
        return
    CURRENT_STATE = 4
    prevstate.update_prevstate(CURRENT_STATE)

    events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, "CHANGED STATE TO EGG DROP")
    
    # Send state to motorapp
    send_flight_state_to_motor(Main_Queue, CURRENT_STATE)
    MotorEggDropActivateMsg = msgstructure.MsgStructure()
    msgstructure.send_msg(Main_Queue, MotorEggDropActivateMsg, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_Motor_Egg_Drop_Activate, "")

    return

def landed_state_transition(Main_Queue : Queue, force: bool = False):
    global CURRENT_STATE
    
    if config.STATE_OVERRIDE is not None and not force:
        return
    
    CURRENT_STATE = 5
    events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, "CHANGED STATE TO LANDED")

    # Store the current state to prev state file
    prevstate.update_prevstate(CURRENT_STATE)
    
    # Send state to motorapp
    send_flight_state_to_motor(Main_Queue, CURRENT_STATE)

    # Perform Action for Landing State
    # 모든 모터 정지
    PayloadMotorStopMsg = msgstructure.MsgStructure()
    msgstructure.send_msg(Main_Queue, PayloadMotorStopMsg, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_PayloadMotorStop, "")
    events.LogEvent(appargs.FlightlogicAppArg.AppName, events.EventType.info, "All motors stopped (LANDED state)")

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

def send_flight_state_to_motor(Main_Queue:Queue, state:int):
    """
    비행 상태를 motorapp으로 전달합니다.
    """
    SendFlightStateMsg = msgstructure.MsgStructure()
    msgstructure.send_msg(Main_Queue, SendFlightStateMsg, appargs.FlightlogicAppArg.AppID, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_SendFlightStateToMotor, str(state))
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
