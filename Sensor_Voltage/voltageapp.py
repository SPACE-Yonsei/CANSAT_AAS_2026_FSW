# Python FSW V2 Voltage App
# Author : Hyeon Lee

from lib import appargs
from lib import msgstructure
from lib import logging
from lib import events
from lib import types

import signal
from multiprocessing import Queue, connection
import threading
import time

from Sensor_Voltage import INA238

# Runstatus of application. Application is terminated when false
VOLTAGEAPP_RUNSTATUS = True

######################################################
## FUNDEMENTAL METHODS                              ##
######################################################

# SB Methods
# Methods for sending/receiving/handling SB messages

# Handles received message
def command_handler (recv_msg : msgstructure.MsgStructure):
    global VOLTAGEAPP_RUNSTATUS

    if recv_msg.MsgID == appargs.MainAppArg.MID_TerminateProcess:
        # Change Runstatus to false to start termination process
        events.LogEvent(appargs.VoltageAppArg.AppName, events.EventType.info, f"VOLTAGEAPP TERMINATION DETECTED")
        VOLTAGEAPP_RUNSTATUS = False

    else:
        events.LogEvent(appargs.VoltageAppArg.AppName, events.EventType.error, f"MID {recv_msg.MsgID} not handled")
    return

def send_hk(Main_Queue : Queue):
    global VOLTAGEAPP_RUNSTATUS
    while VOLTAGEAPP_RUNSTATUS:
        voltageHK = msgstructure.MsgStructure()
        msgstructure.send_msg(Main_Queue, voltageHK, appargs.VoltageAppArg.AppID, appargs.HkAppArg.AppID, appargs.VoltageAppArg.MID_SendHK, str(VOLTAGEAPP_RUNSTATUS))
        time.sleep(1)
    return

######################################################
## INITIALIZATION, TERMINATION                      ##
######################################################

# Initialization
def voltageapp_init():
    global VOLTAGEAPP_RUNSTATUS
    try:
        # Disable Keyboardinterrupt since Termination is handled by parent process
        signal.signal(signal.SIGINT, signal.SIG_IGN)

        events.LogEvent(appargs.VoltageAppArg.AppName, events.EventType.info, "Initializating voltageapp")
        ## User Defined Initialization goes HERE

        # Initialize voltage sensor
        voltage_reader = INA238.init_INA238()
        
        events.LogEvent(appargs.VoltageAppArg.AppName, events.EventType.info, "Voltageapp Initialization Complete")
        
        return voltage_reader
    
    except Exception as e:
        events.LogEvent(appargs.VoltageAppArg.AppName, events.EventType.error, f"Error during initialization: {e}")
        VOLTAGEAPP_RUNSTATUS = False
        return None

# Termination
def voltageapp_terminate():
    global VOLTAGEAPP_RUNSTATUS

    VOLTAGEAPP_RUNSTATUS = False
    events.LogEvent(appargs.VoltageAppArg.AppName, events.EventType.info, "Terminating voltageapp")
    # Termination Process Comes Here

    # Join Each Thread to make sure all threads terminates
    for thread_name in thread_dict:
        events.LogEvent(appargs.VoltageAppArg.AppName, events.EventType.info, f"Terminating thread {thread_name}")
        thread_dict[thread_name].join()
        events.LogEvent(appargs.VoltageAppArg.AppName, events.EventType.info, f"Terminating thread {thread_name} Complete")

    # The termination flag should switch to false AFTER ALL TERMINATION PROCESS HAS ENDED
    events.LogEvent(appargs.VoltageAppArg.AppName, events.EventType.info, "Terminating voltageapp complete")
    return

######################################################
## USER METHOD                                      ##
######################################################

INA238_VOLTAGE : float = 0.0
INA238_POWER : float = 0.0
INA238_CURRENT : float = 0.0

def read_voltage_data(voltage_reader):
    global INA238_VOLTAGE
    global INA238_CURRENT
    global INA238_POWER
    global VOLTAGEAPP_RUNSTATUS

    while VOLTAGEAPP_RUNSTATUS:
        # Check if voltage_reader is valid
        if voltage_reader is None:
            time.sleep(1)
            continue
        
        try:
            INA238_VOLTAGE = INA238.read_voltage(voltage_reader)
            INA238_CURRENT = INA238.read_current(voltage_reader)
            INA238_POWER = INA238.read_power(voltage_reader)
        except (AttributeError, OSError, RuntimeError) as e:
            if not VOLTAGEAPP_RUNSTATUS:
                break
            events.LogEvent(appargs.VoltageAppArg.AppName, events.EventType.error, f"Error reading voltage data: {e}")
            time.sleep(1)
            continue
        
        time.sleep(1)
    
    return

def send_voltage_data(Main_Queue : Queue):
    global INA238_VOLTAGE
    global INA238_CURRENT
    global INA238_POWER
    global VOLTAGEAPP_RUNSTATUS

    VoltageDataToTlmMsg = msgstructure.MsgStructure()
    while VOLTAGEAPP_RUNSTATUS:
        status = msgstructure.send_msg(Main_Queue,
                                       VoltageDataToTlmMsg,
                                       appargs.VoltageAppArg.AppID,
                                       appargs.CommAppArg.AppID,
                                       appargs.VoltageAppArg.MID_SendVoltageTlmData,
                                       f"{INA238_VOLTAGE:.2f},{INA238_CURRENT:.2f},{INA238_POWER:.2f}")
        if status == False:
            events.LogEvent(appargs.VoltageAppArg.AppName, events.EventType.error, "Error when sending voltage telemetry data")
        time.sleep(1)
    
    return

# Put user-defined methods here!

######################################################
## MAIN METHOD                                      ##
######################################################

thread_dict = dict[str, threading.Thread]()

# This method is called from main app. Initialization, runloop process
def voltageapp_main(Main_Queue : Queue, Main_Pipe : connection.Connection):
    global VOLTAGEAPP_RUNSTATUS
    VOLTAGEAPP_RUNSTATUS = True

    # Initialization Process
    voltage_reader = voltageapp_init()
    
    # Check if initialization failed
    if voltage_reader is None:
        events.LogEvent(appargs.VoltageAppArg.AppName, events.EventType.error, "Voltage sensor initialization failed, terminating voltageapp")
        VOLTAGEAPP_RUNSTATUS = False
        return

    # Spawn SB Message Listner Thread
    thread_dict["HKSender_Thread"] = threading.Thread(target=send_hk, args=(Main_Queue, ), name="HKSender_Thread")
    thread_dict["VoltageReader_Thread"] = threading.Thread(target=read_voltage_data, args=(voltage_reader, ), name="VoltageReader_Thread")
    thread_dict["VoltageSender_Thread"] = threading.Thread(target=send_voltage_data, args=(Main_Queue, ), name="VoltageSender_Thread")


    # Spawn Each Threads
    for thread_name in thread_dict:
        thread_dict[thread_name].start()

    try:
        while VOLTAGEAPP_RUNSTATUS:
            # Receive Message From Pipe
            message = Main_Pipe.recv()
            recv_msg = msgstructure.MsgStructure()

            # Unpack Message, Skip this message if unpacked message is not valid
            if msgstructure.unpack_msg(recv_msg, message) == False:
                continue
            
            # Validate Message, Skip this message if target AppID different from voltageapp's AppID
            # Exception when the message is from main app
            if recv_msg.receiver_app == appargs.VoltageAppArg.AppID or recv_msg.receiver_app == appargs.MainAppArg.AppID:
                # Handle Command According to Message ID
                command_handler(recv_msg)
            else:
                events.LogEvent(appargs.VoltageAppArg.AppName, events.EventType.error, "Receiver MID does not match with voltageapp MID")

    # If error occurs, terminate app
    except Exception as e:
        events.LogEvent(appargs.VoltageAppArg.AppName, events.EventType.error, f"voltageapp error : {e}")
        VOLTAGEAPP_RUNSTATUS = False

    # Termination Process after runloop
    voltageapp_terminate()

    return