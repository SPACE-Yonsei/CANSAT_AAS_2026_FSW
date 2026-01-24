# Python FSW V2 Imu App
# Author : Hyeon Lee

from lib import appargs
from lib import msgstructure
from lib import logging
from lib import events

import signal
from multiprocessing import Queue, connection
import threading
import time

# Import IMU sensor library
from Sensor_Imu import imu

# Runstatus of application. Application is terminated when false
IMUAPP_RUNSTATUS = True

######################################################
## FUNDEMENTAL METHODS                              ##
######################################################


# Handles received message
def command_handler (recv_msg : msgstructure.MsgStructure):
    global IMUAPP_RUNSTATUS

    if recv_msg.MsgID == appargs.MainAppArg.MID_TerminateProcess:
        # Change Runstatus to false to start termination process
        events.LogEvent(appargs.ImuAppArg.AppName, events.EventType.info, f"IMUAPP TERMINATION DETECTED")
        IMUAPP_RUNSTATUS = False

    else:
        events.LogEvent(appargs.ImuAppArg.AppName, events.EventType.error, f"MID {recv_msg.MsgID} not handled")
    return


######################################################
## INITIALIZATION, TERMINATION                      ##
######################################################

# Initialization
def imuapp_init():
    global IMUAPP_RUNSTATUS
    try:
        # Disable Keyboardinterrupt since Termination is handled by parent process
        signal.signal(signal.SIGINT, signal.SIG_IGN)

        events.LogEvent(appargs.ImuAppArg.AppName, events.EventType.info, "Initializating imuapp")
        ## User Defined Initialization goes HERE
        
        #Initialize IMU Sensor
        i2c_instance, imu_instance = imu.init_imu()
        
        events.LogEvent(appargs.ImuAppArg.AppName, events.EventType.info, "Imuapp Initialization Complete")
        return i2c_instance, imu_instance
    
    except Exception as e:
        events.LogEvent(appargs.ImuAppArg.AppName, events.EventType.error, f"Error during initialization: {e}")
        IMUAPP_RUNSTATUS = False
        return None, None

# Termination
def imuapp_terminate(i2c_instance):
    global IMUAPP_RUNSTATUS

    IMUAPP_RUNSTATUS = False
    events.LogEvent(appargs.ImuAppArg.AppName, events.EventType.info, "Terminating imuapp")
    # Termination Process Comes Here

    imu.imu_terminate(i2c_instance)

    # Join Each Thread to make sure all threads terminates
    for thread_name in thread_dict:
        events.LogEvent(appargs.ImuAppArg.AppName, events.EventType.info, f"Terminating thread {thread_name}")
        thread_dict[thread_name].join()
        events.LogEvent(appargs.ImuAppArg.AppName, events.EventType.info, f"Terminating thread {thread_name} Complete")

    # The termination flag should switch to false AFTER ALL TERMINATION PROCESS HAS ENDED
    events.LogEvent(appargs.ImuAppArg.AppName, events.EventType.info, "Terminating imuapp complete")
    return

######################################################
## USER METHOD                                      ##
######################################################

IMU_ROLL: float = 0.0
IMU_PITCH: float = 0.0
IMU_YAW: float = 0.0
IMU_ACCX: float = 0.0
IMU_ACCY: float = 0.0
IMU_ACCZ: float = 0.0
IMU_MAGX: float = 0.0
IMU_MAGY: float = 0.0
IMU_MAGZ: float = 0.0
IMU_GYRX: float = 0.0
IMU_GYRY: float = 0.0
IMU_GYRZ: float = 0.0
IMU_TILT_ANGLE: float = 0.0  # 기울기 각도 (0-90도, 수직에서의 기울기)
IMU_TILT_DIRECTION: float = 0.0  # 기울기 방향 (0-360도)
IMU_GRAVITY_X: float = 0.0
IMU_GRAVITY_Y: float = 0.0
IMU_GRAVITY_Z: float = 0.0

def read_imu_data(imu_instance):

    global IMU_ROLL
    global IMU_PITCH
    global IMU_YAW
    global IMU_ACCX
    global IMU_ACCY
    global IMU_ACCZ
    global IMU_MAGX
    global IMU_MAGY
    global IMU_MAGZ
    global IMU_GYRX
    global IMU_GYRY
    global IMU_GYRZ
    global IMU_TILT_ANGLE
    global IMU_TILT_DIRECTION
    global IMU_GRAVITY_X
    global IMU_GRAVITY_Y
    global IMU_GRAVITY_Z
    
    global IMUAPP_RUNSTATUS
    while IMUAPP_RUNSTATUS:
        try:
            # Read data from IMU
            rcv_data = imu.read_sensor_data(imu_instance)

            # Continue if Quaternion data is empty
            if rcv_data == False:
                continue
            else:         
                #새로운 자세 정보 저장
                IMU_ROLL        = rcv_data[0]
                IMU_PITCH       = rcv_data[1]
                IMU_YAW         = rcv_data[2]
                
                IMU_ACCX        = rcv_data[3]
                IMU_ACCY        = rcv_data[4]
                IMU_ACCZ        = rcv_data[5]

                IMU_MAGX        = rcv_data[6]
                IMU_MAGY        = rcv_data[7]
                IMU_MAGZ        = rcv_data[8]

                IMU_GYRX        = rcv_data[9]
                IMU_GYRY        = rcv_data[10]
                IMU_GYRZ        = rcv_data[11]
                
                # 중력 벡터로부터 계산된 기울기 정보
                IMU_TILT_ANGLE      = rcv_data[12]  # 기울기 각도 (0-90도)
                IMU_TILT_DIRECTION  = rcv_data[13]  # 기울기 방향 (0-360도)
                IMU_GRAVITY_X       = rcv_data[14]  # 중력 벡터 X
                IMU_GRAVITY_Y       = rcv_data[15]  # 중력 벡터 Y
                IMU_GRAVITY_Z       = rcv_data[16]  # 중력 벡터 Z
        except (AttributeError, OSError, RuntimeError) as e:
            # Handle I2C errors during shutdown or communication issues
            if not IMUAPP_RUNSTATUS:
                # Normal shutdown, exit gracefully
                break
            # Log error but continue if still running
            events.LogEvent(appargs.ImuAppArg.AppName, events.EventType.error, f"Error reading IMU data: {e}")
            time.sleep(0.1)
            continue

        # The imu runs on 100Hz
        time.sleep(0.01)

    return

def send_imu_data(Main_Queue : Queue):
    global IMU_ROLL
    global IMU_PITCH
    global IMU_YAW
    global IMU_ACCX
    global IMU_ACCY
    global IMU_ACCZ
    global IMU_MAGX
    global IMU_MAGY
    global IMU_MAGZ
    global IMU_GYRX
    global IMU_GYRY
    global IMU_GYRZ
    global IMU_TILT_ANGLE
    global IMU_TILT_DIRECTION
    global IMU_GRAVITY_X
    global IMU_GRAVITY_Y
    global IMU_GRAVITY_Z

    global IMUAPP_RUNSTATUS

    send_counter = 0

    while IMUAPP_RUNSTATUS:

        send_counter += 1

        # Send Yaw data to FlightLogic
        status = msgstructure.send_msg(Main_Queue,
                                       appargs.ImuAppArg.AppID,
                                       appargs.FlightlogicAppArg.AppID,
                                       appargs.ImuAppArg.MID_flight_yaw,
                                       f"{IMU_YAW:.2f}")

        if send_counter >= 10 :
            # Send telemetry message to COMM app
            status = msgstructure.send_msg(Main_Queue,
                                        appargs.ImuAppArg.AppID,
                                        appargs.CommAppArg.AppID,
                                        appargs.ImuAppArg.MID_comm_euler,
                                        f"{IMU_ROLL:.2f},{IMU_PITCH:.2f},{IMU_YAW:.2f},{IMU_ACCX:.2f},{IMU_ACCY:.2f},{IMU_ACCZ:.2f},{IMU_MAGX:.2f},{IMU_MAGY:.2f},{IMU_MAGZ:.2f},{IMU_GYRX:.2f},{IMU_GYRY:.2f},{IMU_GYRZ:.2f}")
            if status == False:
                events.LogEvent(appargs.ImuAppArg.AppName, events.EventType.error, "Error When sending Imu Tlm Message")
            send_counter = 0

        # Sleep 1 second
        time.sleep(0.1)

    return  

######################################################
## MAIN METHOD                                      ##
######################################################

thread_dict = dict[str, threading.Thread]()

# This method is called from main app. Initialization, runloop process
def imuapp_main(Main_Queue : Queue, Main_Pipe : connection.Connection):
    global IMUAPP_RUNSTATUS
    IMUAPP_RUNSTATUS = True

    # Initialization Process
    i2c_instance, imu_instance = imuapp_init()
    
    # Check if initialization failed
    if i2c_instance is None or imu_instance is None:
        events.LogEvent(appargs.ImuAppArg.AppName, events.EventType.error, "IMU initialization failed, terminating imuapp")
        IMUAPP_RUNSTATUS = False
        return

    # Spawn SB Message Listner Thread
    thread_dict["ReadImuData_Thread"] = threading.Thread(target=read_imu_data, args=(imu_instance, ), name="ReadImuData_Thread")
    thread_dict["SendImuData_Thread"] = threading.Thread(target=send_imu_data, args=(Main_Queue, ), name="SendImuData_Thread")


    # Spawn Each Threads
    for thread_name in thread_dict:
        thread_dict[thread_name].start()

    try:
        while IMUAPP_RUNSTATUS:
            # Receive Message From Pipe
            message = Main_Pipe.recv()
            recv_msg = msgstructure.MsgStructure()

            # Unpack Message, Skip this message if unpacked message is not valid
            if msgstructure.unpack_msg(recv_msg, message) == False:
                continue
            
            # Validate Message, Skip this message if target AppID different from imuapp's AppID
            # Exception when the message is from main app
            if recv_msg.receiver_app == appargs.ImuAppArg.AppID or recv_msg.receiver_app == appargs.MainAppArg.AppID:
                # Handle Command According to Message ID
                command_handler(recv_msg)
            else:
                events.LogEvent(appargs.ImuAppArg.AppName, events.EventType.error, "Receiver MID does not match with imuapp MID")

    # If error occurs, terminate app
    except Exception as e:
        events.LogEvent(appargs.ImuAppArg.AppName, events.EventType.error, f"imuapp error : {e}")
        IMUAPP_RUNSTATUS = False

    # Termination Process after runloop
    imuapp_terminate(i2c_instance)

    return