# Python FSW V2 motor App
# Author : Hyeon Lee

from lib import appargs
from lib import msgstructure
from lib import logging
from lib import events
from lib import types
from lib import config

import signal
from multiprocessing import Queue, connection
import threading
import time

# Import Motor Libraries
from Sensor_Motor import parafoil_motor
from Sensor_Motor import Motor_Release
from Sensor_Motor import Motor_Egg
from Sensor_Motor import parafoil_control

MOTORAPP_RUNSTATUS = True
PAYLOAD_MOTOR_ENABLE = True

# Current sensor data for motor control
CURRENT_YAW = 0.0
CURRENT_LAT = 0.0
CURRENT_LON = 0.0
CURRENT_STATE = 0  # Flight state (0=LAUNCHPAD, 1=ASCENT, 2=APOGEE, 3=DESCENT, 4=EGG_RELEASE, 5=LANDED)

######################################################
## FUNDAMENTAL METHODS                              ##
######################################################

# Handles received message
def command_handler (recv_msg : msgstructure.MsgStructure, motor_instance):
    global MOTORAPP_RUNSTATUS
    global PAYLOAD_MOTOR_ENABLE

    if recv_msg.MsgID == appargs.MainAppArg.MID_TerminateProcess:
        events.LogEvent(appargs.MotorAppArg.AppName, events.EventType.info, f"MOTORAPP TERMINATION DETECTED")
        MOTORAPP_RUNSTATUS = False

    # On receiving GPS data for motor control
    elif recv_msg.MsgID == appargs.FlightlogicAppArg.MID_SendGpsMotorData:
        global CURRENT_LAT, CURRENT_LON
        sep_data = recv_msg.data.split(",")
        if len(sep_data) == 2:
            CURRENT_LAT = float(sep_data[0])
            CURRENT_LON = float(sep_data[1])
            # Update motor control when GPS data is received
            update_motor_control(motor_instance)
        else:
            events.LogEvent(appargs.MotorAppArg.AppName, events.EventType.error, f"ERROR receiving GPS motor data, expected 2 fields")
    
    # On receiving IMU data for motor control
    elif recv_msg.MsgID == appargs.FlightlogicAppArg.MID_SendImuMotorData:
        global CURRENT_YAW
        CURRENT_YAW = float(recv_msg.data)
        # Update motor control when IMU data is received (100Hz)
        update_motor_control(motor_instance)
    
    # On receiving target coordinates
    elif recv_msg.MsgID == appargs.FlightlogicAppArg.MID_SetTargetCoordinates:
        sep_data = recv_msg.data.split(",")
        if len(sep_data) == 2:
            target_lat = float(sep_data[0])
            target_lon = float(sep_data[1])
            parafoil_control.set_target_coordinates(target_lat, target_lon)
            events.LogEvent(appargs.MotorAppArg.AppName, events.EventType.info, f"Target coordinates set: Lat={target_lat:.6f}, Lon={target_lon:.6f}")
        else:
            events.LogEvent(appargs.MotorAppArg.AppName, events.EventType.error, f"ERROR receiving target coordinates, expected 2 fields")
    
    # On receiving flight state
    elif recv_msg.MsgID == appargs.FlightlogicAppArg.MID_SendFlightStateToMotor:
        global CURRENT_STATE
        CURRENT_STATE = int(recv_msg.data)
        events.LogEvent(appargs.MotorAppArg.AppName, events.EventType.info, f"Flight state updated to: {CURRENT_STATE}")
    
    # On Container-Payload release activation command (burnwire)
    elif recv_msg.MsgID == appargs.FlightlogicAppArg.MID_Motor_Release_Activate:
        activate_burnwire_release()

    # On Payload-Egg drop activation command (solenoid)
    elif recv_msg.MsgID == appargs.FlightlogicAppArg.MID_Motor_Egg_Drop_Activate:
        activate_egg_drop_solenoid()

    # On Payload Motor Stop command (for LANDED state)
    elif recv_msg.MsgID == appargs.FlightlogicAppArg.MID_PayloadMotorStop:
        stop_payload_motor(motor_instance)

    elif recv_msg.MsgID == appargs.CommAppArg.MID_RouteCmd_MEC:
        events.LogEvent(appargs.MotorAppArg.AppName, events.EventType.info, f"MEC : Current Option : {recv_msg.data}")
        if recv_msg.data == "ON":
            PAYLOAD_MOTOR_ENABLE = True
        elif recv_msg.data == "OFF":
            PAYLOAD_MOTOR_ENABLE = False
        else:
            events.LogEvent(appargs.MotorAppArg.AppName, events.EventType.error, f"Error setting motor enable, invalid option : {recv_msg.data}")
            
    else:
        events.LogEvent(appargs.MotorAppArg.AppName, events.EventType.error, f"MID {recv_msg.MsgID} not handled")
    return

def send_hk(Main_Queue : Queue):
    global MOTORAPP_RUNSTATUS
    while MOTORAPP_RUNSTATUS:
        motorHK = msgstructure.MsgStructure()
        msgstructure.send_msg(Main_Queue, motorHK, appargs.MotorAppArg.AppID, appargs.HkAppArg.AppID, appargs.MotorAppArg.MID_SendHK, str(MOTORAPP_RUNSTATUS))
        time.sleep(1)
    return

######################################################
## INITIALIZATION, TERMINATION                      ##
######################################################

# Initialization
def motorapp_init():
    global MOTORAPP_RUNSTATUS
    try:
        # Disable Keyboardinterrupt since Termination is handled by parent process
        signal.signal(signal.SIGINT, signal.SIG_IGN)

        events.LogEvent(appargs.MotorAppArg.AppName, events.EventType.info, "Initializating motorapp")
        ## User Defined Initialization goes HERE
        motor_instance = None

        # Initialize parafoil control module (load target coordinates from prevstate)
        parafoil_control.init_parafoil_control()
        # Initialize parafoil motor (GPIO 12, 13)
        parafoil_instance = parafoil_motor.init_parafoil_motor()
        # Initialize burnwire for container-payload release (GPIO 6)
        Motor_Release.init_burnwire()
        # Initialize solenoid for egg drop (GPIO 5)
        Motor_Egg.init_solenoid()
        # Store parafoil motor in a dictionary
        motor_instance = {
            'parafoil': parafoil_instance
        }
        events.LogEvent(appargs.MotorAppArg.AppName, events.EventType.info, "Payload motors (parafoil), burnwire, and solenoid standby")

        events.LogEvent(appargs.MotorAppArg.AppName, events.EventType.info, "motorapp Initialization Complete")
        return motor_instance
    
    except Exception as e:
        events.LogEvent(appargs.MotorAppArg.AppName, events.EventType.error, f"Error during initialization: {e}")
        MOTORAPP_RUNSTATUS = False
        return None
    
# Termination
def motorapp_terminate(motor_instance):
    global MOTORAPP_RUNSTATUS

    MOTORAPP_RUNSTATUS = False
    events.LogEvent(appargs.MotorAppArg.AppName, events.EventType.info, "Terminating motorapp")
    # Termination Process Comes Here

    # Terminate each motor
    if isinstance(motor_instance, dict):
        parafoil_motor.terminate_parafoil_motor(motor_instance['parafoil'])
        # Stop pigpio instance (shared between motors)
        if motor_instance['parafoil'] is not None:
            motor_instance['parafoil'].stop()
        # Terminate release mechanisms
        Motor_Release.terminate_burnwire()
        Motor_Egg.terminate_solenoid()
    else:
        parafoil_motor.terminate_parafoil_motor(motor_instance)

    for thread_name in thread_dict:
        events.LogEvent(appargs.MotorAppArg.AppName, events.EventType.info, f"Terminating thread {thread_name}")
        thread_dict[thread_name].join()
        events.LogEvent(appargs.MotorAppArg.AppName, events.EventType.info, f"Terminating thread {thread_name} Complete")

    events.LogEvent(appargs.MotorAppArg.AppName, events.EventType.info, "Terminating motorapp complete")
    return

######################################################
## USER METHOD                                      ##
######################################################

def activate_burnwire_release():
    """Activate burnwire to release payload from container (번와이어로 컨테이너-페이로드 사출)."""
    events.LogEvent(appargs.MotorAppArg.AppName, events.EventType.info, "Activating Burnwire (container-payload release)")
    Motor_Release.activate_burnwire()
    return

def activate_egg_drop_solenoid():
    """Activate solenoid to drop payload-egg (솔레노이드로 계란 사출)."""
    events.LogEvent(appargs.MotorAppArg.AppName, events.EventType.info, "Activating Solenoid (payload-egg drop)")
    Motor_Egg.activate_solenoid()
    return

def stop_payload_motor(motor_instance):
    """Stop all payload motors (parafoil motors)."""
    events.LogEvent(appargs.MotorAppArg.AppName, events.EventType.info, "Stopping all payload motors")
    if isinstance(motor_instance, dict):
        # Stop parafoil motors by sending turn=0
        parafoil_motor.rotate_parafoil_motor(motor_instance['parafoil'], 0.0)
    else:
        parafoil_motor.rotate_parafoil_motor(motor_instance, 0.0)
    return

def update_motor_control(motor_instance):
    """
    Calculate motor control direction based on current GPS/IMU data and control motors.
    Motor control is only activated in DESCENT(3) or EGG_RELEASE(4) states.
    """
    global CURRENT_STATE, CURRENT_YAW, CURRENT_LAT, CURRENT_LON, PAYLOAD_MOTOR_ENABLE
    
    # Parafoil motor control is only activated in DESCENT(3) or EGG_RELEASE(4) states
    # In APOGEE(2) state, parafoil algorithm starts but motors do not operate
    if CURRENT_STATE < 3:  # No motor control before DESCENT state (LAUNCHPAD, ASCENT, APOGEE)
        return
    
    # Motor is disabled
    if not PAYLOAD_MOTOR_ENABLE:
        return
    
    # Calculate motor control direction (turn angle)
    turn_angle = parafoil_control.calculate_motor_control(
        current_yaw=CURRENT_YAW,
        current_lat=CURRENT_LAT,
        current_lon=CURRENT_LON
    )
    
    # Execute motor control
    if isinstance(motor_instance, dict):
        parafoil_motor.rotate_parafoil_motor(motor_instance['parafoil'], turn_angle)
    else:
        parafoil_motor.rotate_parafoil_motor(motor_instance, turn_angle)
    
    return

######################################################
## MAIN METHOD                                      ##
######################################################

thread_dict = dict[str, threading.Thread]()

def motorapp_main(Main_Queue : Queue, Main_Pipe : connection.Connection):
    global MOTORAPP_RUNSTATUS
    MOTORAPP_RUNSTATUS = True

    # Initialization Process
    motor_instance = motorapp_init()
    
    # Check if initialization failed (motor_instance can be None or dict)
    if motor_instance is None:
        events.LogEvent(appargs.MotorAppArg.AppName, events.EventType.error, "Motor initialization failed, terminating motorapp")
        MOTORAPP_RUNSTATUS = False
        return

    # Spawn SB Message Listner Thread
    thread_dict["HKSender_Thread"] = threading.Thread(target=send_hk, args=(Main_Queue, ), name="HKSender_Thread")

    # Spawn Each Threads
    for thread_name in thread_dict:
        thread_dict[thread_name].start()

    try:
        while MOTORAPP_RUNSTATUS:
            message = Main_Pipe.recv()
            recv_msg = msgstructure.MsgStructure()

            if msgstructure.unpack_msg(recv_msg, message) == False:
                continue

            if recv_msg.receiver_app == appargs.MotorAppArg.AppID or recv_msg.receiver_app == appargs.MainAppArg.AppID:
                # Handle Command According to Message ID
                command_handler(recv_msg, motor_instance)
            else:
                events.LogEvent(appargs.MotorAppArg.AppName, events.EventType.error, "Receiver MID does not match with motorapp MID")

    # If error occurs, terminate app
    except Exception as e:
        events.LogEvent(appargs.MotorAppArg.AppName, events.EventType.error, f"motorapp error : {e}")
        MOTORAPP_RUNSTATUS = False

    # Termination Process after runloop
    motorapp_terminate(motor_instance)

    return
