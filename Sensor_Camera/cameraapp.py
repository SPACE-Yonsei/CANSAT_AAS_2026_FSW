# Python FSW V2 Camera App
# Author : Hyeon Lee

# for priority management, import os
import os

from lib import appargs
from lib import msgstructure
from lib import logging
from lib import events

import signal
from multiprocessing import Queue, connection
import threading
import time

# Runstatus of application. Application is terminated when false
CAMERAAPP_RUNSTATUS = True

######################################################
## FUNDEMENTAL METHODS                              ##
######################################################

# SB Methods
# Methods for sending/receiving/handling SB messages

# Handles received message
def command_handler (recv_msg : msgstructure.MsgStructure):
    global CAMERAAPP_RUNSTATUS

    if recv_msg.MsgID == appargs.MainAppArg.MID_TerminateProcess:
        # Change Runstatus to false to start termination process
        events.LogEvent(appargs.CameraAppArg.AppName, events.EventType.info, f"CAMERAAPP TERMINATION DETECTED")
        CAMERAAPP_RUNSTATUS = False

    # When received activate camera
    elif recv_msg.MsgID == appargs.CommAppArg.MID_RouteCmd_CAM:
        events.LogEvent(appargs.CameraAppArg.AppName, events.EventType.info, f"CAMERA {recv_msg.data} COMMAND RECEIVED")
        if recv_msg.data == "ON":
            picam_start_recording()
        elif recv_msg.data == "OFF":
            picam_stop_recording()
    
    elif recv_msg.MsgID == appargs.FlightlogicAppArg.MID_cam_activate:
        # events.LogEvent(appargs.CameraAppArg.AppName, events.EventType.info, f"CAMERA ACTIVATION BY LOGIC")
        picam_start_recording()

    else:
        events.LogEvent(appargs.CameraAppArg.AppName, events.EventType.error, f"MID {recv_msg.MsgID} not handled")
        
    return


######################################################
## INITIALIZATION, TERMINATION                      ##
######################################################

# Initialization
def cameraapp_init():
    global CAMERAAPP_RUNSTATUS

    picam_instance = None
    picamencoder_instance = None

    # Disable Keyboardinterrupt since Termination is handled by parent process
    signal.signal(signal.SIGINT, signal.SIG_IGN)

    events.LogEvent(appargs.CameraAppArg.AppName, events.EventType.info, "Initializating cameraapp")

    ## User Defined Initialization goes HERE

    # Initialization of picamera
    try:
        picam_instance, picamencoder_instance = picam.init_cam()
    except Exception as e:
        events.LogEvent(appargs.CameraAppArg.AppName, events.EventType.error, f"Error Initializing picam : {e}")

    # Initially start the cameras
    picam_start_recording() # Turns the recording flag to True

    events.LogEvent(appargs.CameraAppArg.AppName, events.EventType.info, "Cameraapp Initialization Complete")

    return picam_instance, picamencoder_instance

# Termination
def cameraapp_terminate(picam_instance):
    global CAMERAAPP_RUNSTATUS

    CAMERAAPP_RUNSTATUS = False
    events.LogEvent(appargs.CameraAppArg.AppName, events.EventType.info, "Terminating cameraapp")

    # Termination Process Comes Here
    picam_stop_recording()
    
    # Terminating picam
    picam.terminate(picam_instance)

    # Join Each Thread to make sure all threads terminates
    for thread_name in thread_dict:
        events.LogEvent(appargs.CameraAppArg.AppName, events.EventType.info, f"Terminating thread {thread_name}")
        thread_dict[thread_name].join()
        events.LogEvent(appargs.CameraAppArg.AppName, events.EventType.info, f"Terminating thread {thread_name} Complete")

    # The termination flag should switch to false AFTER ALL TERMINATION PROCESS HAS ENDED
    events.LogEvent(appargs.CameraAppArg.AppName, events.EventType.info, "Terminating cameraapp complete")
    return

######################################################
## USER METHOD                                      ##
######################################################
from Sensor_Camera import picam

CAMERA_RECORD_SEC = 7

PICAM_RECORDING = False

# Simple wrapup function for managing camera
def picam_start_recording():
    global PICAM_RECORDING
    PICAM_RECORDING = True
    picam.PICAM_RECORDING = True
    
    # events.LogEvent(appargs.CameraAppArg.AppName, events.EventType.info, f"Picam recording flag is TRUE")
    return

def picam_stop_recording():
    global PICAM_RECORDING
    PICAM_RECORDING = False
    picam.PICAM_RECORDING = False

    # events.LogEvent(appargs.CameraAppArg.AppName, events.EventType.info, f"Picam recording flag is FALSE")
    return

def picam_record_thread(picam_instance, picamencoder_instance):
    global CAMERAAPP_RUNSTATUS

    if picam_instance == None:
        events.LogEvent(appargs.CameraAppArg.AppName, events.EventType.error, f"Picam not initialized! Terminating thread")
        return  # Early return to prevent using None instance

    while CAMERAAPP_RUNSTATUS:
        if PICAM_RECORDING == True:
            try:
                # events.LogEvent(appargs.CameraAppArg.AppName, events.EventType.info, f"Picam Recording Start")
                picam.record(picam_instance, picamencoder_instance, CAMERA_RECORD_SEC)
                # events.LogEvent(appargs.CameraAppArg.AppName, events.EventType.info, f"Picam Recording End")

            except Exception as e:
                events.LogEvent(appargs.CameraAppArg.AppName, events.EventType.error, f"Error Recording picam : {e}")
                time.sleep(1)
        else:
            # Wait until Picam recording flag
            time.sleep(0.1)

    return
    
# Put user-defined methods here!

######################################################
## MAIN METHOD                                      ##
######################################################

thread_dict = dict[str, threading.Thread]()

from lib import config

# This method is called from main app. Initialization, runloop process
def cameraapp_main(Main_Queue : Queue, Main_Pipe : connection.Connection):
    
    global CAMERAAPP_RUNSTATUS
    CAMERAAPP_RUNSTATUS = True

    # Initialization Process
    picam_instance, picamencoder_instance = cameraapp_init()

    # Spawn SB Message Listner Thread
    thread_dict["PicamRecorder_Thread"]  = threading.Thread(target=picam_record_thread, args=(picam_instance, picamencoder_instance), name="PicamRecorder_Thread")

    # Spawn Each Threads
    for thread_name in thread_dict:
        thread_dict[thread_name].start()

    try:
        while CAMERAAPP_RUNSTATUS:
            # Receive Message From Pipe
            message = Main_Pipe.recv()
            recv_msg = msgstructure.unpack_msg(message)

            # Unpack Message, Skip this message if unpacked message is not valid
            if recv_msg == False:
                continue

            # Validate Message, Skip this message if target AppID different from cameraapp's AppID
            # Exception when the message is from main app
            if recv_msg.receiver_app == appargs.CameraAppArg.AppID or recv_msg.receiver_app == appargs.MainAppArg.AppID:
                # Handle Command According to Message ID
                command_handler(recv_msg)
            else:
                events.LogEvent(appargs.CameraAppArg.AppName, events.EventType.error, "Receiver MID does not match with cameraapp MID")

    # If error occurs, terminate app
    except Exception as e:
        events.LogEvent(appargs.CameraAppArg.AppName, events.EventType.error, f"cameraapp error : {e}")
        CAMERAAPP_RUNSTATUS = False

    # Termination Process after runloop
    cameraapp_terminate(picam_instance)

    return
