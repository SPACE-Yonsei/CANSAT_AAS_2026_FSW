# Python FSW V2 Gps App
# Author : Hyeon Lee

from lib import appargs
from lib import msgstructure
from lib import logging
from lib import events

import signal
import sys
from multiprocessing import Queue, connection
import threading
import time

# Runstatus of application. Application is terminated when false
GPSAPP_RUNSTATUS = True
gps_instance = None
######################################################
## FUNDEMENTAL METHODS                              ##
######################################################

# SB Methods
# Methods for sending/receiving/handling SB messages

# Handles received message
def command_handler (recv_msg : msgstructure.MsgStructure):
    global GPSAPP_RUNSTATUS

    if recv_msg.MsgID == appargs.MainAppArg.MID_TerminateProcess:
        # Change Runstatus to false to start termination process
        events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.info, f"GPSAPP TERMINATION DETECTED")
        GPSAPP_RUNSTATUS = False

    else:
        events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.error, f"MID {recv_msg.MsgID} not handled")
    return

######################################################
## INITIALIZATION, TERMINATION                      ##
######################################################
# Initialization
def gpsapp_init():
    global GPSAPP_RUNSTATUS
    global gps_instance
    try:
        # Disable Keyboardinterrupt since Termination is handled by parent process
        signal.signal(signal.SIGINT, signal.SIG_IGN)

        events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.info, "Initializating gpsapp")
        ## User Defined Initialization goes HERE
        gps_instance = gps.init_gps()
        events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.info, "Gpsapp Initialization Complete")
        return gps_instance
    except Exception as e:
        events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.error, f"Error during initialization: {e}")
        GPSAPP_RUNSTATUS = False
        return None

# Termination
def gpsapp_terminate():
    global GPSAPP_RUNSTATUS
    global gps_instance
    GPSAPP_RUNSTATUS = False
    events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.info, "Terminating gpsapp")
    # Termination Process Comes Here

    # Join Each Thread to make sure all threads terminates
    for thread_name in thread_dict:
        events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.info, f"Terminating thread {thread_name}")
        thread_dict[thread_name].join()
        events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.info, f"Terminating thread {thread_name} Complete")

    if gps_instance is not None:
        gps.terminate_gps(gps_instance)
        gps_instance = None
        events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.info, "GPS connection terminated")

    # The termination flag should switch to false AFTER ALL TERMINATION PROCESS HAS ENDED
    events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.info, "Terminating gpsapp complete")
    return

######################################################
## USER METHOD                                      ##
######################################################
from Sensor_Gps import gps
def read_and_send_gps_data(Main_Queue: Queue, gps_instance):
    global GPSAPP_RUNSTATUS

    # 초기값 설정
    GPS_LAT = 0.0
    GPS_LON = 0.0
    GPS_ALT = 0.0
    GPS_TIME = "00:00:00"
    GPS_SATS = 0
    GPS_FIX_QUALITY = 0

    send_counter = 0

    while GPSAPP_RUNSTATUS:
        # Check if gps_instance is valid
        if gps_instance is None:
            time.sleep(0.04)
            continue
        
        try:
            rcv_data = gps.gps_readdata(gps_instance)
        except Exception as e:
            events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.error, f"Error reading GPS data: {e}")
            time.sleep(0.1)
            continue

        # 데이터가 유효할 때만 변수를 업데이트한다 (None이거나 유효하지 않으면 이전 값 유지)
        # gps.py에서 fix_quality를 추가했으므로 len이 5 또는 6일 수 있음
        if rcv_data and (len(rcv_data) == 5 or len(rcv_data) == 6):
            try:
                GPS_TIME = rcv_data[0]
                GPS_ALT  = float(rcv_data[1])
                GPS_LAT  = float(rcv_data[2])
                GPS_LON  = float(rcv_data[3])
                GPS_SATS = int(rcv_data[4])
                GPS_FIX_QUALITY = int(rcv_data[5]) if len(rcv_data) > 5 else 0
                # Print GPS data for debugging (disabled)
                # print(f"GPS: Time={GPS_TIME}, Lat={GPS_LAT:.6f}, Lon={GPS_LON:.6f}, Alt={GPS_ALT:.2f}, Sats={GPS_SATS}, FixQuality={GPS_FIX_QUALITY}")
                # sys.stdout.flush()
                # GPS 값이 유효한 경우에만 이벤트 로그 출력
                """
                if GPS_LAT != 0.0 or GPS_LON != 0.0 or GPS_SATS > 0:
                    events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.info, f"GPS data read: Lat={GPS_LAT:.6f}, Lon={GPS_LON:.6f}, Alt={GPS_ALT:.2f}, Sats={GPS_SATS}")
                # GPS fix가 없는 경우 (좌표가 0이지만 시간은 있는 경우) 주기적으로 로그 출력
                elif GPS_TIME != "00:00:00" and send_counter % 100 == 0:  # 10초마다
                    events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.warning, f"GPS reading but no fix: Time={GPS_TIME}, Sats={GPS_SATS}, FixQuality={GPS_FIX_QUALITY} (waiting for GPS fix...)")
                """
            except (ValueError, IndexError, TypeError) as e:
                # 파싱 에러 시 이전 값을 유지하고 로그만 남김
                events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.error, f"Error parsing GPS data: {e}, rcv_data={rcv_data}")
        else:
            # GPS 데이터가 없을 때 (None 또는 형식 불일치) - 이전 값 유지
            # 로그 출력 비활성화
            pass

        # FlightLogic으로 데이터 전송 (매 루프마다)
        if GPS_LAT != 0.0 or GPS_LON != 0.0:
            msgstructure.send_msg(
                Main_Queue,
                appargs.GpsAppArg.AppID, appargs.FlightlogicAppArg.AppID,
                appargs.GpsAppArg.MID_flight_MyCor,
                f"{GPS_LAT},{GPS_LON}"
            )

        send_counter += 1
        if send_counter >= 10:
            # Send telemetry message to COMM app
            # 이제 변수들이 0으로 리셋되지 않으므로 마지막 유효 좌표가 전송됨

            status = msgstructure.send_msg(
                Main_Queue,
                appargs.GpsAppArg.AppID,
                appargs.CommAppArg.AppID,
                appargs.GpsAppArg.MID_comm_gga,
                f"{GPS_TIME},{GPS_ALT},{GPS_LAT},{GPS_LON},{GPS_SATS}"
            )
            if status == False:
                events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.error, "Error When sending GPS Tlm Message")
            # GPS TLM 전송 성공 로그 비활성화
            send_counter = 0

        # GPS read rate: 10Hz (reduced from 25Hz to avoid I2C bus conflicts with other sensors)
        # read_gps() can take up to 1 second, so reading at 25Hz was causing overlapping calls
        time.sleep(0.1)

######################################################
## MAIN METHOD                                      ##
######################################################

thread_dict = dict[str, threading.Thread]()

# This method is called from main app. Initialization, runloop process
def gpsapp_main(Main_Queue : Queue, Main_Pipe : connection.Connection):
    global GPSAPP_RUNSTATUS
    GPSAPP_RUNSTATUS = True

    # Initialization Process
    gps_instance = gpsapp_init()
    
    # Check if initialization failed
    if gps_instance is None:
        events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.error, "GPS initialization failed, terminating gpsapp")
        GPSAPP_RUNSTATUS = False
        return

    # Spawn SB Message Listner Thread
    thread_dict["ReadAndSendGpsData_Thread"] = threading.Thread(target=read_and_send_gps_data, args=(Main_Queue, gps_instance,  ), name="ReadAndSendGpsData_Thread")

    # Spawn Each Threads
    for thread_name in thread_dict:
        thread_dict[thread_name].start()

    try:
        while GPSAPP_RUNSTATUS:
            # Receive Message From Pipe
            message = Main_Pipe.recv()
            recv_msg = msgstructure.MsgStructure()

            # Unpack Message, Skip this message if unpacked message is not valid
            if msgstructure.unpack_msg(recv_msg, message) == False:
                continue
            
            # Validate Message, Skip this message if target AppID different from gpsapp's AppID
            # Exception when the message is from main app
            if recv_msg.receiver_app == appargs.GpsAppArg.AppID or recv_msg.receiver_app == appargs.MainAppArg.AppID:
                # Handle Command According to Message ID
                command_handler(recv_msg)
            else:
                events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.error, "Receiver MID does not match with gpsapp MID")

    # If error occurs, terminate app
    except Exception as e:
        events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.error, f"gpsapp error : {e}")
        GPSAPP_RUNSTATUS = False

    # Termination Process after runloop
    gpsapp_terminate()

    return