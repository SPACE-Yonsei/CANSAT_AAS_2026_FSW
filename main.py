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
import threading
import time

APP_DICT_LOCK = threading.RLock()

# Initialize logging system FIRST (before any LogEvent calls)
log_queue = events.init_events_main_process()

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
app_dict = dict()

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

def cameraapp_launcher(pipe, log_queue):
    from lib import events
    events.init_events_subprocess(log_queue)
    from Sensor_Camera import cameraapp
    cameraapp.cameraapp_main(pipe)

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

def motorapp_launcher(pipe, log_queue):
    from lib import events
    events.init_events_subprocess(log_queue)
    from Sensor_Motor import motorapp
    motorapp.motorapp_main(pipe)

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
cameraapp_elements.process = Process(target=cameraapp_launcher, args=(child_pipe, log_queue))
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
# motorapp                                        #
#########################################################
parent_pipe, child_pipe = Pipe()
motorapp_elements = app_elements()
motorapp_elements.process = Process(target=motorapp_launcher, args=(child_pipe, log_queue))
motorapp_elements.pipe = parent_pipe
app_dict[appargs.MotorAppArg.AppID] = motorapp_elements

#########################################################
# DistanceApp (TF-Luna Sensor)                     #
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
# Launcher function mapping for process restart         #
#########################################################
app_launchers = {
    appargs.BarometerAppArg.AppID: barometerapp_launcher,
    appargs.CameraAppArg.AppID: cameraapp_launcher,
    appargs.GpsAppArg.AppID: gpsapp_launcher,
    appargs.ImuAppArg.AppID: imuapp_launcher,
    appargs.CommAppArg.AppID: commapp_launcher,
    appargs.ElectroAppArg.AppID: electroapp_launcher,
    appargs.FlightlogicAppArg.AppID: flightlogicapp_launcher,
    appargs.MotorAppArg.AppID: motorapp_launcher,
    appargs.DistanceAppArg.AppID: distanceapp_launcher,
}

# Launcher argument factory map
# Camera/Motor do not take main_queue, while others do.
app_launcher_args = {
    appargs.BarometerAppArg.AppID: lambda child_pipe: (main_queue, child_pipe, log_queue),
    appargs.CameraAppArg.AppID: lambda child_pipe: (child_pipe, log_queue),
    appargs.GpsAppArg.AppID: lambda child_pipe: (main_queue, child_pipe, log_queue),
    appargs.ImuAppArg.AppID: lambda child_pipe: (main_queue, child_pipe, log_queue),
    appargs.CommAppArg.AppID: lambda child_pipe: (main_queue, child_pipe, log_queue),
    appargs.ElectroAppArg.AppID: lambda child_pipe: (main_queue, child_pipe, log_queue),
    appargs.FlightlogicAppArg.AppID: lambda child_pipe: (main_queue, child_pipe, log_queue),
    appargs.MotorAppArg.AppID: lambda child_pipe: (child_pipe, log_queue),
    appargs.DistanceAppArg.AppID: lambda child_pipe: (main_queue, child_pipe, log_queue),
}


#########################################################
# Application Management                                #
# Functions for (re)starting, terminating applications  # 
#########################################################
def run_app(AppID: int):
    with APP_DICT_LOCK:
        app_entry = app_dict.get(AppID)
    if app_entry is not None:
        app_process : Process = app_entry.process
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

    msg = msgstructure.fill_msg(appargs.MainAppArg.AppID, appargs.MainAppArg.AppID, appargs.MainAppArg.MID_TerminateProcess, "")
    packed_msg = msgstructure.pack_msg(msg)

    # Send termination message to kill every process
    with APP_DICT_LOCK:
        app_ids = list(app_dict.keys())
    for appID in app_ids:
        events.LogEvent(appargs.MainAppArg.AppName, events.EventType.info, f"Terminating AppID {appID}")
        with APP_DICT_LOCK:
            app_entry = app_dict.get(appID)
        if app_entry is None:
            continue
        try:
            app_entry.pipe.send(packed_msg)
        except (OSError, BrokenPipeError):
             events.LogEvent(appargs.MainAppArg.AppName, events.EventType.warning, f"Pipe broken for AppID {appID}, process may have already exited")

    # Join all processes with timeout, force kill if not responding
    for appID in app_ids:
        events.LogEvent(appargs.MainAppArg.AppName, events.EventType.info, f"Joining AppID {appID}")
        with APP_DICT_LOCK:
            app_entry = app_dict.get(appID)
        if app_entry is None:
            continue
        app_entry.process.join(timeout=3)  # 3초 타임아웃
        if app_entry.process.is_alive():
            events.LogEvent(appargs.MainAppArg.AppName, events.EventType.warning, f"AppID {appID} not responding, force killing")
            app_entry.process.terminate()
            app_entry.process.join(timeout=1)
            if app_entry.process.is_alive():
                app_entry.process.kill()
        events.LogEvent(appargs.MainAppArg.AppName, events.EventType.info, f"Terminating AppID {appID} complete")

    events.LogEvent(appargs.MainAppArg.AppName, events.EventType.info, f"Manual termination! Resetting prev state file")
    prevstate.reset_prevstate()
    
    events.LogEvent(appargs.MainAppArg.AppName, events.EventType.info, f"All Termination Process complete, terminating FSW")
    
    # Shutdown logging system
    events.shutdown_events()
    
    sys.exit()
    return

#########################################################
# Process Monitoring and Restart                        #
#########################################################

# 프로세스 재시작 함수
def restart_app(appID: int):
    """죽은 프로세스를 재시작합니다."""
    global app_dict, app_launchers, app_launcher_args
    
    with APP_DICT_LOCK:
        has_app = appID in app_dict
    if not has_app or appID not in app_launchers:
        events.LogEvent(appargs.MainAppArg.AppName, events.EventType.error, 
                       f"Cannot restart AppID {appID}: not in dictionary")
        return False
    if appID not in app_launcher_args:
        events.LogEvent(appargs.MainAppArg.AppName, events.EventType.error,
                       f"Cannot restart AppID {appID}: launcher args not configured")
        return False
    
    try:
        # 새 파이프 생성
        parent_pipe, child_pipe = Pipe()
        
        # 새 프로세스 생성
        launcher = app_launchers[appID]
        launcher_args = app_launcher_args[appID](child_pipe)
        new_process = Process(target=launcher, args=launcher_args)
        
        # app_dict 업데이트
        new_elements = app_elements()
        new_elements.process = new_process
        new_elements.pipe = parent_pipe
        with APP_DICT_LOCK:
            old_entry = app_dict.get(appID)
            app_dict[appID] = new_elements
        if old_entry is not None:
            try:
                old_entry.pipe.close()
            except Exception:
                pass
        
        # 새 프로세스 시작
        new_process.start()
        
        events.LogEvent(appargs.MainAppArg.AppName, events.EventType.info, 
                       f"AppID {appID} restarted successfully")
        return True
        
    except Exception as e:
        events.LogEvent(appargs.MainAppArg.AppName, events.EventType.error, 
                       f"Failed to restart AppID {appID}: {e}")
        return False

# 프로세스 상태 확인 및 재시작 (주기적으로 호출)
def checkrunstatus():
    """모든 프로세스의 생존 여부를 확인하고, 죽은 프로세스를 재시작합니다."""
    global MAINAPP_RUNSTATUS, app_dict
    
    with APP_DICT_LOCK:
        app_items = [(app_id, app_dict[app_id].process) for app_id in list(app_dict.keys())]

    for appID, process in app_items:
        
        # 프로세스가 시작되었고, 더 이상 살아있지 않은 경우
        if process.pid is not None and not process.is_alive():
            exit_code = process.exitcode
            events.LogEvent(appargs.MainAppArg.AppName, events.EventType.warning, 
                           f"AppID {appID} died (exit code: {exit_code}), attempting restart...")
            
            # 재시작 시도
            success = restart_app(appID)
            if not success:
                events.LogEvent(appargs.MainAppArg.AppName, events.EventType.error, 
                               f"AppID {appID} restart failed, will retry next cycle")

# 프로세스 모니터링 스레드
def process_monitor():
    """백그라운드에서 프로세스 상태를 주기적으로 확인합니다."""
    global MAINAPP_RUNSTATUS
    
    MONITOR_INTERVAL = 3  # 5초마다 확인
    
    events.LogEvent(appargs.MainAppArg.AppName, events.EventType.info, 
                   "Process monitor started")
    
    while MAINAPP_RUNSTATUS:
        time.sleep(MONITOR_INTERVAL)
        
        if not MAINAPP_RUNSTATUS:
            break
            
        checkrunstatus()
    
    events.LogEvent(appargs.MainAppArg.AppName, events.EventType.info, 
                   "Process monitor stopped")

# Main Runloop
def runloop(Main_Queue : Queue):
    global MAINAPP_RUNSTATUS
    try:
        while MAINAPP_RUNSTATUS:
            # Recv Message from queue
            recv_msg = Main_Queue.get()

            # Unpack the message to Check receiver
            unpacked_msg = msgstructure.unpack_msg(recv_msg)

            if unpacked_msg == False:
                continue

            with APP_DICT_LOCK:
                app_entry = app_dict.get(unpacked_msg.receiver_app)
            if app_entry is None:
                continue
            try:
                app_entry.pipe.send(recv_msg)
            except (OSError, BrokenPipeError):
                events.LogEvent(
                    appargs.MainAppArg.AppName,
                    events.EventType.warning,
                    f"Pipe send failed to AppID {unpacked_msg.receiver_app}",
                )
                continue

    except KeyboardInterrupt:
        events.LogEvent(appargs.MainAppArg.AppName, events.EventType.info, "KeyboardInterrupt Detected, Terminating FSW")
        MAINAPP_RUNSTATUS = False

    try:
        terminate_FSW()
    except KeyboardInterrupt:
        events.LogEvent(appargs.MainAppArg.AppName, events.EventType.warning, "Force terminating all processes...")
        with APP_DICT_LOCK:
            app_entries = [app_dict[app_id] for app_id in app_dict]
        for app_entry in app_entries:
            if app_entry.process.is_alive():
                app_entry.process.kill()
        events.shutdown_events()
        sys.exit(1)
    return


# Operation starts HERE
if __name__ == '__main__':
    events.LogEvent(appargs.MainAppArg.AppName, events.EventType.info, "Starting FSW...")

    # Start each app's process
    with APP_DICT_LOCK:
        start_items = [(app_id, app_dict[app_id].process) for app_id in app_dict]
    for appID, app_process in start_items:
        app_process.start()
        events.LogEvent(appargs.MainAppArg.AppName, events.EventType.info, f"Started AppID {appID}")

    events.LogEvent(appargs.MainAppArg.AppName, events.EventType.info, "All processes started.")
    
    # Start process monitor thread
    monitor_thread = threading.Thread(target=process_monitor, name="ProcessMonitor", daemon=True)
    monitor_thread.start()
    events.LogEvent(appargs.MainAppArg.AppName, events.EventType.info, "Process monitor thread started. Entering main runloop.")
    
    # Main app runloop
    runloop(main_queue)
