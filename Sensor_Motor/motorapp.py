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
from Sensor_Motor import container_motor
from Sensor_Motor import payload_egg_motor

MOTORAPP_RUNSTATUS = True
PAYLOAD_MOTOR_ENABLE = True
PAYLOAD_MODES = (config.CONF_PAYLOAD, config.CONF_PAYLOAD_DESCENT)

######################################################
## FUNDEMENTAL METHODS                              ##
######################################################

# Handles received message
def command_handler (recv_msg : msgstructure.MsgStructure, motor_instance):
    global MOTORAPP_RUNSTATUS
    global PAYLOAD_MOTOR_ENABLE

    if recv_msg.MsgID == appargs.MainAppArg.MID_TerminateProcess:
        events.LogEvent(appargs.motorAppArg.AppName, events.EventType.info, f"MOTORAPP TERMINATION DETECTED")
        MOTORAPP_RUNSTATUS = False

    # On receiving yaw data
    elif recv_msg.MsgID == appargs.FlightlogicAppArg.MID_SendPayloadMotorRatation:
        if config.FSW_CONF in PAYLOAD_MODES and PAYLOAD_MOTOR_ENABLE == True:
            recv_turn = float(recv_msg.data)
            if isinstance(motor_instance, dict):
                parafoil_motor.rotate_parafoil_motor(motor_instance['parafoil'], recv_turn)
            else:
                parafoil_motor.rotate_parafoil_motor(motor_instance, recv_turn)
        else:
            return
    
    # On Payload-Egg motor activation command
    elif recv_msg.MsgID == appargs.FlightlogicAppArg.MID_PayloadEggMotorActivate:
        if config.FSW_CONF in PAYLOAD_MODES:
            activateeggmotor(motor_instance)
        else:
            events.LogEvent(appargs.motorAppArg.AppName, events.EventType.info, f"Not Performing Payload-Egg Motor Activation, current conf : {config.FSW_CONF}")    

    # On Payload Release motor activation command
    elif recv_msg.MsgID == appargs.FlightlogicAppArg.MID_PayloadReleaseMotorActivate:

        if config.FSW_CONF == config.CONF_CONTAINER:
            activatepayloadreleasemotor(motor_instance)
        else:
            events.LogEvent(appargs.motorAppArg.AppName, events.EventType.info, f"Not Performing Payload Release Motor Activation, current conf : {config.FSW_CONF}")

    # On Payload Release motor standby command
    elif recv_msg.MsgID == appargs.FlightlogicAppArg.MID_PayloadReleaseMotorStandby:

        if config.FSW_CONF == config.CONF_CONTAINER:
            standbypayloadreleasemotor(motor_instance)
        else:
            events.LogEvent(appargs.motorAppArg.AppName, events.EventType.info, f"Not Performing Payload Release Motor Standby, current conf : {config.FSW_CONF}")


    elif recv_msg.MsgID == appargs.CommAppArg.MID_RouteCmd_MEC:
        if config.FSW_CONF == config.CONF_CONTAINER:
            events.LogEvent(appargs.motorAppArg.AppName, events.EventType.info, f"MEC : Current conf : container, Current Option : {recv_msg.data}...")
            if recv_msg.data == "ON":
                activatepayloadreleasemotor(motor_instance)
            elif recv_msg.data == "OFF":
                freepayloadreleasemotor(motor_instance)
            else:
                events.LogEvent(appargs.motorAppArg.AppName, events.EventType.error, f"Error Activating container motor, invalid option : {recv_msg.data}")

        elif config.FSW_CONF in PAYLOAD_MODES:
            events.LogEvent(appargs.motorAppArg.AppName, events.EventType.info, f"MEC : Current conf : payload, Current Option : {recv_msg.data}")
            if recv_msg.data == "ON":
                PAYLOAD_MOTOR_ENABLE = True
            elif recv_msg.data == "OFF":
                PAYLOAD_MOTOR_ENABLE = False
            else:
                events.LogEvent(appargs.motorAppArg.AppName, events.EventType.error, f"Error Activating payload motor, invalid option : {recv_msg.data}")
            
    else:
        events.LogEvent(appargs.motorAppArg.AppName, events.EventType.error, f"MID {recv_msg.MsgID} not handled")
    return

def send_hk(Main_Queue : Queue):
    global MOTORAPP_RUNSTATUS
    while MOTORAPP_RUNSTATUS:
        motorHK = msgstructure.MsgStructure()
        msgstructure.send_msg(Main_Queue, motorHK, appargs.motorAppArg.AppID, appargs.HkAppArg.AppID, appargs.motorAppArg.MID_SendHK, str(MOTORAPP_RUNSTATUS))
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

        events.LogEvent(appargs.motorAppArg.AppName, events.EventType.info, "Initializating motorapp")
        ## User Defined Initialization goes HERE
        motor_instance = None

        if config.FSW_CONF in PAYLOAD_MODES:
            # Initialize parafoil motor (GPIO 12, 13)
            parafoil_instance = parafoil_motor.init_parafoil_motor()
            # Initialize payload-egg ejection motor (GPIO 6)
            egg_motor_instance = payload_egg_motor.init_MG92B()
            # Store both motors in a dictionary
            motor_instance = {
                'parafoil': parafoil_instance,
                'egg_motor': egg_motor_instance
            }
            events.LogEvent(appargs.motorAppArg.AppName, events.EventType.info, "Payload motors (parafoil + egg) standby")

        elif config.FSW_CONF == config.CONF_CONTAINER:
            motor_instance = container_motor.init_MG996R()
            events.LogEvent(appargs.motorAppArg.AppName, events.EventType.info, "Container motor standby")
            standbypayloadreleasemotor(motor_instance)

        else:
            events.LogEvent(appargs.motorAppArg.AppName, events.EventType.info, "No Valid configuration!")
            MOTORAPP_RUNSTATUS = False
            return None

        events.LogEvent(appargs.motorAppArg.AppName, events.EventType.info, "motorapp Initialization Complete")
        return motor_instance
    
    except Exception as e:
        events.LogEvent(appargs.motorAppArg.AppName, events.EventType.error, f"Error during initialization: {e}")
        MOTORAPP_RUNSTATUS = False
        return None
    
# Termination
def motorapp_terminate(motor_instance):
    global MOTORAPP_RUNSTATUS

    MOTORAPP_RUNSTATUS = False
    events.LogEvent(appargs.motorAppArg.AppName, events.EventType.info, "Terminating motorapp")
    # Termination Process Comes Here

    # Terminate each motor
    if config.FSW_CONF in PAYLOAD_MODES:
        if isinstance(motor_instance, dict):
            parafoil_motor.terminate_parafoil_motor(motor_instance['parafoil'])
            payload_egg_motor.terminate_MG92B(motor_instance['egg_motor'])
            # Stop pigpio instance (shared between motors)
            if motor_instance['parafoil'] is not None:
                motor_instance['parafoil'].stop()
        else:
            parafoil_motor.terminate_parafoil_motor(motor_instance)
    elif config.FSW_CONF == config.CONF_CONTAINER:
        container_motor.terminate_MG996R(motor_instance)

    for thread_name in thread_dict:
        events.LogEvent(appargs.motorAppArg.AppName, events.EventType.info, f"Terminating thread {thread_name}")
        thread_dict[thread_name].join()
        events.LogEvent(appargs.motorAppArg.AppName, events.EventType.info, f"Terminating thread {thread_name} Complete")

    events.LogEvent(appargs.motorAppArg.AppName, events.EventType.info, "Terminating motorapp complete")
    return

######################################################
## USER METHOD                                      ##
######################################################

def activatepayloadreleasemotor(motor_instance):
    events.LogEvent(appargs.motorAppArg.AppName, events.EventType.info, "Activating Payload Release Motor")
    container_motor.container_release(motor_instance)
    return

def standbypayloadreleasemotor(motor_instance):
    events.LogEvent(appargs.motorAppArg.AppName, events.EventType.info, "Standby Payload Release Motor")
    container_motor.container_initial(motor_instance)
    return

def freepayloadreleasemotor(motor_instance):
    events.LogEvent(appargs.motorAppArg.AppName, events.EventType.info, "Free Payload Release Motor")
    container_motor.container_free(motor_instance)
    return

def activateeggmotor(motor_instance):
    """Activate payload-egg ejection motor."""
    events.LogEvent(appargs.motorAppArg.AppName, events.EventType.info, "Activating Payload-Egg Ejection Motor")
    if isinstance(motor_instance, dict):
        payload_egg_motor.egg_motor_release(motor_instance['egg_motor'])
    else:
        events.LogEvent(appargs.motorAppArg.AppName, events.EventType.error, "Motor instance type error for egg motor")
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
        events.LogEvent(appargs.motorAppArg.AppName, events.EventType.error, "Motor initialization failed, terminating motorapp")
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

            if recv_msg.receiver_app == appargs.motorAppArg.AppID or recv_msg.receiver_app == appargs.MainAppArg.AppID:
                # Handle Command According to Message ID
                command_handler(recv_msg, motor_instance)
            else:
                events.LogEvent(appargs.motorAppArg.AppName, events.EventType.error, "Receiver MID does not match with motorapp MID")

    # If error occurs, terminate app
    except Exception as e:
        events.LogEvent(appargs.motorAppArg.AppName, events.EventType.error, f"motorapp error : {e}")
        MOTORAPP_RUNSTATUS = False

    # Termination Process after runloop
    motorapp_terminate(motor_instance)

    return