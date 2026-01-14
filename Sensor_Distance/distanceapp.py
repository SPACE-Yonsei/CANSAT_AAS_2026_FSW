# Python FSW V2 Distance App (TF-Luna I2C Sensor)
# Author : Hyeon Lee

from lib import appargs
from lib import msgstructure
from lib import events

import signal
from multiprocessing import Queue, connection
import threading
import time

from Sensor_Distance import distance

# Runstatus of application. Application is terminated when false
DISTANCEAPP_RUNSTATUS = True

# Distance threshold for EGG_RELEASE (in mm)
EGG_RELEASE_THRESHOLD_MM = 2000  # 2m = 2000mm

######################################################
## FUNDEMENTAL METHODS                              ##
######################################################

# Handles received message
def command_handler(recv_msg: msgstructure.MsgStructure):
    global DISTANCEAPP_RUNSTATUS

    if recv_msg.MsgID == appargs.MainAppArg.MID_TerminateProcess:
        events.LogEvent(appargs.DistanceAppArg.AppName, events.EventType.info, "DISTANCEAPP TERMINATION DETECTED")
        DISTANCEAPP_RUNSTATUS = False

    else:
        events.LogEvent(appargs.DistanceAppArg.AppName, events.EventType.error, f"MID {recv_msg.MsgID} not handled")
    return


def send_hk(Main_Queue: Queue):
    global DISTANCEAPP_RUNSTATUS
    while DISTANCEAPP_RUNSTATUS:
        distanceHK = msgstructure.MsgStructure()
        msgstructure.send_msg(Main_Queue, distanceHK, appargs.DistanceAppArg.AppID, appargs.HkAppArg.AppID, appargs.DistanceAppArg.MID_SendHK, str(DISTANCEAPP_RUNSTATUS))
        time.sleep(1)
    return


######################################################
## INITIALIZATION, TERMINATION                      ##
######################################################

def distanceapp_init():
    global DISTANCEAPP_RUNSTATUS
    try:
        # Disable Keyboardinterrupt since Termination is handled by parent process
        signal.signal(signal.SIGINT, signal.SIG_IGN)

        events.LogEvent(appargs.DistanceAppArg.AppName, events.EventType.info, "Initializing distanceapp")

        # Initialize TF-Luna I2C sensor
        tof_sensor = distance.init_TFLuna()

        events.LogEvent(appargs.DistanceAppArg.AppName, events.EventType.info, "Distanceapp Initialization Complete")
        return tof_sensor

    except Exception as e:
        events.LogEvent(appargs.DistanceAppArg.AppName, events.EventType.error, f"Error during initialization: {e}")
        DISTANCEAPP_RUNSTATUS = False
        return None


def distanceapp_terminate(tof_sensor):
    global DISTANCEAPP_RUNSTATUS

    DISTANCEAPP_RUNSTATUS = False
    events.LogEvent(appargs.DistanceAppArg.AppName, events.EventType.info, "Terminating distanceapp")

    # Terminate sensor
    if tof_sensor is not None:
        distance.terminate_TFLuna(tof_sensor)

    # Join threads
    for thread_name in thread_dict:
        events.LogEvent(appargs.DistanceAppArg.AppName, events.EventType.info, f"Terminating thread {thread_name}")
        thread_dict[thread_name].join()
        events.LogEvent(appargs.DistanceAppArg.AppName, events.EventType.info, f"Terminating thread {thread_name} Complete")

    events.LogEvent(appargs.DistanceAppArg.AppName, events.EventType.info, "Terminating distanceapp complete")
    return


######################################################
## USER METHOD                                      ##
######################################################

DISTANCE_MM: int = 0


def read_distance_data(tof_sensor):
    """Read distance from TF-Luna I2C sensor."""
    global DISTANCE_MM
    global DISTANCEAPP_RUNSTATUS

    while DISTANCEAPP_RUNSTATUS:
        if tof_sensor is None:
            time.sleep(1)
            continue

        try:
            DISTANCE_MM = distance.read_distance(tof_sensor)
        except Exception as e:
            if not DISTANCEAPP_RUNSTATUS:
                break
            events.LogEvent(appargs.DistanceAppArg.AppName, events.EventType.error, f"Error reading distance: {e}")
            time.sleep(0.5)
            continue

        time.sleep(0.1)  # 10Hz reading

    return


def send_distance_data(Main_Queue: Queue):
    """Send distance data to flight logic and telemetry."""
    global DISTANCE_MM
    global DISTANCEAPP_RUNSTATUS

    DistanceToFlightLogicMsg = msgstructure.MsgStructure()
    DistanceToTlmMsg = msgstructure.MsgStructure()

    while DISTANCEAPP_RUNSTATUS:
        # Send to flight logic (for EGG_RELEASE trigger)
        msgstructure.send_msg(
            Main_Queue,
            DistanceToFlightLogicMsg,
            appargs.DistanceAppArg.AppID,
            appargs.FlightlogicAppArg.AppID,
            appargs.DistanceAppArg.MID_SendDistanceFlightLogicData,
            str(DISTANCE_MM)
        )

        # Send to telemetry
        msgstructure.send_msg(
            Main_Queue,
            DistanceToTlmMsg,
            appargs.DistanceAppArg.AppID,
            appargs.CommAppArg.AppID,
            appargs.DistanceAppArg.MID_SendDistanceTlmData,
            str(DISTANCE_MM)
        )

        time.sleep(0.2)  # 5Hz sending

    return


######################################################
## MAIN METHOD                                      ##
######################################################

thread_dict = dict[str, threading.Thread]()


def distanceapp_main(Main_Queue: Queue, Main_Pipe: connection.Connection):
    global DISTANCEAPP_RUNSTATUS
    DISTANCEAPP_RUNSTATUS = True

    # Initialization Process
    distance_sensor = distanceapp_init()

    # Check if initialization failed
    if distance_sensor is None:
        events.LogEvent(appargs.DistanceAppArg.AppName, events.EventType.error, "Distance sensor initialization failed, terminating distanceapp")
        DISTANCEAPP_RUNSTATUS = False
        return

    # Spawn threads
    thread_dict["HKSender_Thread"] = threading.Thread(target=send_hk, args=(Main_Queue,), name="HKSender_Thread")
    thread_dict["DistanceReader_Thread"] = threading.Thread(target=read_distance_data, args=(distance_sensor,), name="DistanceReader_Thread")
    thread_dict["DistanceSender_Thread"] = threading.Thread(target=send_distance_data, args=(Main_Queue,), name="DistanceSender_Thread")

    # Start threads
    for thread_name in thread_dict:
        thread_dict[thread_name].start()

    try:
        while DISTANCEAPP_RUNSTATUS:
            message = Main_Pipe.recv()
            recv_msg = msgstructure.MsgStructure()

            if msgstructure.unpack_msg(recv_msg, message) == False:
                continue

            if recv_msg.receiver_app == appargs.DistanceAppArg.AppID or recv_msg.receiver_app == appargs.MainAppArg.AppID:
                command_handler(recv_msg)
            else:
                events.LogEvent(appargs.DistanceAppArg.AppName, events.EventType.error, "Receiver MID does not match with distanceapp MID")

    except Exception as e:
        events.LogEvent(appargs.DistanceAppArg.AppName, events.EventType.error, f"distanceapp error: {e}")
        DISTANCEAPP_RUNSTATUS = False

    # Termination Process
    distanceapp_terminate(distance_sensor)

    return

