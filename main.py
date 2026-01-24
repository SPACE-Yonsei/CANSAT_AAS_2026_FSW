# Main Flight Software Code for Cansat Mission
# Author : Hyeon Lee

# Sys library is needed to exit app
import sys

MAINAPP_RUNSTATUS = True

# Custum libraries
from lib import appargs
from lib import msgstructure
from lib import events

# Multiprocessing Library is used on Python FSW V2
# Each application should have its own runloop
# Import the application and execute the runloop here.

from multiprocessing import Process, Queue, Pipe, connection

# Initialize logging system FIRST (before any LogEvent calls)
log_queue = events.init_events_main_process()

# Load configuration files
from lib import config
if config.FSW_CONF == config.CONF_NONE:
    events.LogEvent(appargs.MainAppArg.AppName, events.EventType.error, "CONFIG IS SELECTED AS NONE, TERMINATING FSW")
    sys.exit(0)

# Read prev state, altitude calibration for recovery
from lib import prevstate
prevstate.init_prevstate()

# Define the multiprocessing queue structure
# Every runloop should take this queue as an argument
# for message routing
# Set maxsize to prevent unbounded memory growth (1000 messages ~ few MB)
main_queue = Queue(maxsize=1000)

# When the main app receives the message entry from the queue
# It checks the message ID and destination application then routes the message
# Each application should establish a pipe with main app to receive routed message

# App element stores main process of app, pipe that can send SB message to app
class app_elements:
    process : Process = None
    pipe : connection.Connection = None
# The app dictionary has key as AppID, app elements as Value
app_dict = dict[app_elements]()

#########################################################
# Lazy Import Wrapper Functions                         #
# Each subprocess imports its own module on startup     #
# This enables parallel imports for faster boot time    #
# log_queue is passed for multiprocessing-safe logging  #
#########################################################

def barometerapp_launcher(queue, pipe, log_queue):
    from lib import events
    events.init_events_subprocess(log_queue)
    from Sensor_Barometer import barometerapp
    barometerapp.barometerapp_main(queue, pipe)

def cameraapp_launcher(queue, pipe, log_queue):
    from lib import events
    events.init_events_subprocess(log_queue)
    from Sensor_Camera import cameraapp
    cameraapp.cameraapp_main(queue, pipe)

def gpsapp_launcher(queue, pipe, log_queue):
    from lib import events
    events.init_events_subprocess(log_queue)
    from Sensor_Gps import gpsapp
    gpsapp.gpsapp_main(queue, pipe)

def imuapp_launcher(queue, pipe, log_queue):
    from lib import events
    events.init_events_subprocess(log_queue)
    from Sensor_Imu import imuapp
    imuapp.imuapp_main(queue, pipe)

def commapp_launcher(queue, pipe, log_queue):
    from lib import events
    events.init_events_subprocess(log_queue)
    from comm import commapp
    commapp.commapp_main(queue, pipe)

def electroapp_launcher(queue, pipe, log_queue):
    from lib import events
    events.init_events_subprocess(log_queue)
    from Sensor_Electro import electroapp
    electroapp.electroapp_main(queue, pipe)

def flightlogicapp_launcher(queue, pipe, log_queue):
    from lib import events
    events.init_events_subprocess(log_queue)
    from flight_logic import flightlogicapp
    flightlogicapp.flightlogicapp_main(queue, pipe)

def motorapp_launcher(queue, pipe, log_queue):
    from lib import events
    events.init_events_subprocess(log_queue)
    from Sensor_Motor import motorapp
    motorapp.motorapp_main(queue, pipe)

def distanceapp_launcher(queue, pipe, log_queue):
    from lib import events
    events.init_events_subprocess(log_queue)
    from Sensor_Distance import distanceapp
    distanceapp.distanceapp_main(queue, pipe)

#########################################################
# BarometerApp                                          #
#########################################################
parent_pipe, child_pipe = Pipe()
barometerapp_elements = app_elements()
barometerapp_elements.process = Process(target=barometerapp_launcher, args=(main_queue, child_pipe, log_queue))
barometerapp_elements.pipe = parent_pipe
app_dict[appargs.BarometerAppArg.AppID] = barometerapp_elements

#########################################################
# CameraApp                                             #
#########################################################
parent_pipe, child_pipe = Pipe()
cameraapp_elements = app_elements()
cameraapp_elements.process = Process(target=cameraapp_launcher, args=(main_queue, child_pipe, log_queue))
cameraapp_elements.pipe = parent_pipe
app_dict[appargs.CameraAppArg.AppID] = cameraapp_elements

#########################################################
# GpsApp                                                #
#########################################################
parent_pipe, child_pipe = Pipe()
gpsapp_elements = app_elements()
gpsapp_elements.process = Process(target=gpsapp_launcher, args=(main_queue, child_pipe, log_queue))
gpsapp_elements.pipe = parent_pipe
app_dict[appargs.GpsAppArg.AppID] = gpsapp_elements

#########################################################
# ImuApp                                                #
#########################################################
parent_pipe, child_pipe = Pipe()
imuapp_elements = app_elements()
imuapp_elements.process = Process(target=imuapp_launcher, args=(main_queue, child_pipe, log_queue))
imuapp_elements.pipe = parent_pipe
app_dict[appargs.ImuAppArg.AppID] = imuapp_elements

#########################################################
# CommApp                                               #
#########################################################
parent_pipe, child_pipe = Pipe()
commapp_elements = app_elements()
commapp_elements.process = Process(target=commapp_launcher, args=(main_queue, child_pipe, log_queue))
commapp_elements.pipe = parent_pipe
app_dict[appargs.CommAppArg.AppID] = commapp_elements

#########################################################
# ElectroApp                                            #
#########################################################
parent_pipe, child_pipe = Pipe()
electroapp_elements = app_elements()
electroapp_elements.process = Process(target=electroapp_launcher, args=(main_queue, child_pipe, log_queue))
electroapp_elements.pipe = parent_pipe
app_dict[appargs.ElectroAppArg.AppID] = electroapp_elements

#########################################################
# FlightlogicApp                                        #
#########################################################
parent_pipe, child_pipe = Pipe()
flightlogicapp_elements = app_elements()
flightlogicapp_elements.process = Process(target=flightlogicapp_launcher, args=(main_queue, child_pipe, log_queue))
flightlogicapp_elements.pipe = parent_pipe
app_dict[appargs.FlightlogicAppArg.AppID] = flightlogicapp_elements

#########################################################
# Gimbalmotorapp                                        #
#########################################################
parent_pipe, child_pipe = Pipe()
motorapp_elements = app_elements()
motorapp_elements.process = Process(target=motorapp_launcher, args=(main_queue, child_pipe, log_queue))
motorapp_elements.pipe = parent_pipe
app_dict[appargs.MotorAppArg.AppID] = motorapp_elements

#########################################################
# DistanceApp (VL53L1CX ToF Sensor)                     #
#########################################################
parent_pipe, child_pipe = Pipe()
distanceapp_elements = app_elements()
distanceapp_elements.process = Process(target=distanceapp_launcher, args=(main_queue, child_pipe, log_queue))
distanceapp_elements.pipe = parent_pipe
app_dict[appargs.DistanceAppArg.AppID] = distanceapp_elements

#########################################################
# Add Apps HERE                                         #
#########################################################


#########################################################
# Application Management                                #
# Functions for (re)starting, terminating applications  # 
#########################################################
def run_app(AppID: int):
    if AppID in app_dict:
        app_process : Process = app_dict[AppID].process
        app_process.start()
    else:
        events.LogEvent(appargs.MainAppArg.AppName, events.EventType.error, f"AppID {AppID} not in app dictionary")

#########################################################
# Termination Process                                   #
# Jobs need to be done when terminating process         # 
#########################################################

def terminate_FSW():
    global MAINAPP_RUNSTATUS
    # Set all Runstatus to false
    MAINAPP_RUNSTATUS = False

    msg=msgstructure.fill_msg(appargs.MainAppArg.AppID, appargs.MainAppArg.AppID, appargs.MainAppArg.MID_TerminateProcess, "")
    packed_msg = msgstructure.pack_msg(msg)

    # Send termination message to kill every process
    for appID in app_dict:
        events.LogEvent(appargs.MainAppArg.AppName, events.EventType.info, f"Terminating AppID {appID}")
        app_dict[appID].pipe.send(packed_msg)

    # Join all processes with timeout, force kill if not responding
    for appID in app_dict:
        events.LogEvent(appargs.MainAppArg.AppName, events.EventType.info, f"Joining AppID {appID}")
        app_dict[appID].process.join(timeout=3)  # 3초 타임아웃
        if app_dict[appID].process.is_alive():
            events.LogEvent(appargs.MainAppArg.AppName, events.EventType.warning, f"AppID {appID} not responding, force killing")
            app_dict[appID].process.terminate()
            app_dict[appID].process.join(timeout=1)
            if app_dict[appID].process.is_alive():
                app_dict[appID].process.kill()
        events.LogEvent(appargs.MainAppArg.AppName, events.EventType.info, f"Terminating AppID {appID} complete")

    events.LogEvent(appargs.MainAppArg.AppName, events.EventType.info, f"Manual termination! Resetting prev state file")
    prevstate.reset_prevstate()
    
    events.LogEvent(appargs.MainAppArg.AppName, events.EventType.info, f"All Termination Process complete, terminating FSW")
    
    # Shutdown logging system
    events.shutdown_events()
    
    sys.exit()
    return

# Check run status, restart correspoding app when run status is false
# TBD
def checkrunstatus():
    return

# Main Runloop
def runloop(Main_Queue : Queue):
    global MAINAPP_RUNSTATUS
    try:
        while MAINAPP_RUNSTATUS:
            # Recv Message from queue
            recv_msg = Main_Queue.get()

            # Unpack the message to Check receiver
            unpacked_msg = msgstructure.MsgStructure()
            msgstructure.unpack_msg(unpacked_msg, recv_msg)
            
            if unpacked_msg.receiver_app in app_dict:
                app_dict[unpacked_msg.receiver_app].pipe.send(recv_msg)
            else:
                #events.LogEvent(appargs.MainAppArg.AppName, events.EventType.error, "Error : Received MID in not in app dictionary")
                continue

    except KeyboardInterrupt:
        events.LogEvent(appargs.MainAppArg.AppName, events.EventType.info, "KeyboardInterrupt Detected, Terminating FSW")
        MAINAPP_RUNSTATUS = False

    try:
        terminate_FSW()
    except KeyboardInterrupt:
        events.LogEvent(appargs.MainAppArg.AppName, events.EventType.warning, "Force terminating all processes...")
        for appID in app_dict:
            if app_dict[appID].process.is_alive():
                app_dict[appID].process.kill()
        events.shutdown_events()
        sys.exit(1)
    return


# Operation starts HERE
if __name__ == '__main__':
    events.LogEvent(appargs.MainAppArg.AppName, events.EventType.info, "Starting FSW...")

    # Start each app's process
    for appID in app_dict:
        app_dict[appID].process.start()
        events.LogEvent(appargs.MainAppArg.AppName, events.EventType.info, f"Started AppID {appID}")

    events.LogEvent(appargs.MainAppArg.AppName, events.EventType.info, "All processes started. Entering main runloop.")
    
    # Main app runloop
    runloop(main_queue)
