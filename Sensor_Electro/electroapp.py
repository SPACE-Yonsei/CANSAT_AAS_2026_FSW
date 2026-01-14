from lib import appargs
from lib import msgstructure
from lib import logging
from lib import events
from lib import types

import signal
from multiprocessing import Queue, connection
import threading
import time

from Sensor_Electro import electro

# Runstatus of application. Application is terminated when false
ELECTROAPP_RUNSTATUS = True

######################################################
## FUNDEMENTAL METHODS                              ##
######################################################

# Handles received message
def command_handler (recv_msg : msgstructure.MsgStructure):
    global ELECTROAPP_RUNSTATUS

    if recv_msg.MsgID == appargs.MainAppArg.MID_TerminateProcess:
        # Change Runstatus to false to start termination process
        events.LogEvent(appargs.ElectroAppArg.AppName, events.EventType.info, f"ELECTROAPP TERMINATION DETECTED")
        ELECTROAPP_RUNSTATUS = False

    else:
        events.LogEvent(appargs.ElectroAppArg.AppName, events.EventType.error, f"MID {recv_msg.MsgID} not handled")
    return

def send_hk(Main_Queue : Queue):
    global ELECTROAPP_RUNSTATUS
    while ELECTROAPP_RUNSTATUS:
        electroHK = msgstructure.MsgStructure()
        msgstructure.send_msg(Main_Queue, electroHK, appargs.ElectroAppArg.AppID, appargs.HkAppArg.AppID, appargs.ElectroAppArg.MID_SendHK, str(ELECTROAPP_RUNSTATUS))
        time.sleep(1)
    return

######################################################
## INITIALIZATION, TERMINATION                      ##
######################################################

# Initialization
def electroapp_init():
    global ELECTROAPP_RUNSTATUS
    try:
        # Disable Keyboardinterrupt since Termination is handled by parent process
        signal.signal(signal.SIGINT, signal.SIG_IGN)

        events.LogEvent(appargs.ElectroAppArg.AppName, events.EventType.info, "Initializating electroapp")
        ## User Defined Initialization goes HERE

        # Initialize voltage sensor
        electro_reader = electro.init_INA228()
        
        events.LogEvent(appargs.ElectroAppArg.AppName, events.EventType.info, "Electroapp Initialization Complete")
        
        return electro_reader
    
    except Exception as e:
        events.LogEvent(appargs.ElectroAppArg.AppName, events.EventType.error, f"Error during initialization: {e}")
        ELECTROAPP_RUNSTATUS = False
        return None

# Termination
def electroapp_terminate():
    global ELECTROAPP_RUNSTATUS

    ELECTROAPP_RUNSTATUS = False
    events.LogEvent(appargs.ElectroAppArg.AppName, events.EventType.info, "Terminating electroapp")
    # Termination Process Comes Here

    # Join Each Thread to make sure all threads terminates
    for thread_name in thread_dict:
        events.LogEvent(appargs.ElectroAppArg.AppName, events.EventType.info, f"Terminating thread {thread_name}")
        thread_dict[thread_name].join()
        events.LogEvent(appargs.ElectroAppArg.AppName, events.EventType.info, f"Terminating thread {thread_name} Complete")

    # The termination flag should switch to false AFTER ALL TERMINATION PROCESS HAS ENDED
    events.LogEvent(appargs.ElectroAppArg.AppName, events.EventType.info, "Terminating electroapp complete")
    return

######################################################
## USER METHOD                                      ##
######################################################

ELECTRO_VOLTAGE : float = 0.0
ELECTRO_POWER : float = 0.0
ELECTRO_CURRENT : float = 0.0

def read_electro_data(electro_reader):
    global ELECTRO_VOLTAGE
    global ELECTRO_CURRENT
    global ELECTRO_POWER
    global ELECTROAPP_RUNSTATUS

    while ELECTROAPP_RUNSTATUS:
        # Check if voltage_reader is valid
        if electro_reader is None:
            time.sleep(1)
            continue
        
        try:
            ELECTRO_VOLTAGE = electro.read_voltage(electro_reader)
            ELECTRO_CURRENT = electro.read_current(electro_reader)
            ELECTRO_POWER = electro.read_power(electro_reader)
        except (AttributeError, OSError, RuntimeError) as e:
            if not ELECTROAPP_RUNSTATUS:
                break
            events.LogEvent(appargs.ElectroAppArg.AppName, events.EventType.error, f"Error reading electro data: {e}")
            time.sleep(1)
            continue
        
        time.sleep(1)
    
    return

def send_electro_data(Main_Queue : Queue):
    global ELECTRO_VOLTAGE
    global ELECTRO_CURRENT
    global ELECTRO_POWER
    global ELECTROAPP_RUNSTATUS

    ElectroDataToTlmMsg = msgstructure.MsgStructure()
    while ELECTROAPP_RUNSTATUS:
        status = msgstructure.send_msg(Main_Queue,
                                       ElectroDataToTlmMsg,
                                       appargs.ElectroAppArg.AppID,
                                       appargs.CommAppArg.AppID,
                                       appargs.ElectroAppArg.MID_SendElectroTlmData,
                                       f"{ELECTRO_VOLTAGE:.2f},{ELECTRO_CURRENT:.2f},{ELECTRO_POWER:.2f}")
        if status == False:
            events.LogEvent(appargs.ElectroAppArg.AppName, events.EventType.error, "Error when sending electro telemetry data")
        time.sleep(1)
    
    return

# Put user-defined methods here!

######################################################
## MAIN METHOD                                      ##
######################################################

thread_dict = dict[str, threading.Thread]()

# This method is called from main app. Initialization, runloop process
def electroapp_main(Main_Queue : Queue, Main_Pipe : connection.Connection):
    global ELECTROAPP_RUNSTATUS
    ELECTROAPP_RUNSTATUS = True

    # Initialization Process
    electro_reader = electroapp_init()
    
    # Check if initialization failed
    if electro_reader is None:
        events.LogEvent(appargs.ElectroAppArg.AppName, events.EventType.error, "Electro sensor initialization failed, terminating electroapp")
        ELECTROAPP_RUNSTATUS = False
        return

    # Spawn SB Message Listner Thread
    thread_dict["HKSender_Thread"] = threading.Thread(target=send_hk, args=(Main_Queue, ), name="HKSender_Thread")
    thread_dict["ElectroReader_Thread"] = threading.Thread(target=read_electro_data, args=(electro_reader, ), name="ElectroReader_Thread")
    thread_dict["ElectroSender_Thread"] = threading.Thread(target=send_electro_data, args=(Main_Queue, ), name="ElectroSender_Thread")


    # Spawn Each Threads
    for thread_name in thread_dict:
        thread_dict[thread_name].start()

    try:
        while ELECTROAPP_RUNSTATUS:
            # Receive Message From Pipe
            message = Main_Pipe.recv()
            recv_msg = msgstructure.MsgStructure()

            # Unpack Message, Skip this message if unpacked message is not valid
            if msgstructure.unpack_msg(recv_msg, message) == False:
                continue
            
            # Validate Message, Skip this message if target AppID different from voltageapp's AppID
            # Exception when the message is from main app
            if recv_msg.receiver_app == appargs.ElectroAppArg.AppID or recv_msg.receiver_app == appargs.MainAppArg.AppID:
                # Handle Command According to Message ID
                command_handler(recv_msg)
            else:
                events.LogEvent(appargs.ElectroAppArg.AppName, events.EventType.error, "Receiver MID does not match with electroapp MID")

    # If error occurs, terminate app
    except Exception as e:
        events.LogEvent(appargs.ElectroAppArg.AppName, events.EventType.error, f"electroapp error : {e}")
        ELECTROAPP_RUNSTATUS = False

    # Termination Process after runloop
    electroapp_terminate()

    return