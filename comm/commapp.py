# Python FSW V2 Comm App
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

from datetime import datetime, timedelta
from comm import uartserial
from comm import xbeereset

import os
# Runstatus of application. Application is terminated when false
COMMAPP_RUNSTATUS = True

# Team ID (AAS 2026)
TEAMID = 1070

# Timedelta for ST command, initially set to 0
ST_timedelta :timedelta = timedelta(seconds=0)

SIMP_OFFSET = 0

######################################################
## FUNDEMENTAL METHODS                              ##
######################################################

# SB Methods
# Methods for sending/receiving/handling SB messages

# Handles received message
def command_handler (recv_msg : msgstructure.MsgStructure):
    global COMMAPP_RUNSTATUS
    global tlm_data

    if recv_msg.MsgID == appargs.MainAppArg.MID_TerminateProcess:
        # Change Runstatus to false to start termination process
        events.LogEvent(appargs.CommAppArg.AppName, events.EventType.info, f"COMMAPP TERMINATION DETECTED")
        COMMAPP_RUNSTATUS = False

    # Receive Telemetry Data from Apps

    # Receive Barometer Data
    elif recv_msg.MsgID == appargs.BarometerAppArg.MID_SendBarometerTlmData:
        sep_data = recv_msg.data.split(",")
        
        # Check the length of separated data
        if (len(sep_data) != 3):
            events.LogEvent(appargs.CommAppArg.AppName, events.EventType.error, f"ERROR receiving barometer, expected 3 fields, got {len(sep_data)}")
            return
        
        try:
            # If simulation mode, ignore the pressure and altitude data
            if (tlm_data.mode == "F"):
                tlm_data.pressure = float(sep_data[0])
                tlm_data.altitude = float(sep_data[2])

            tlm_data.temperature = float(sep_data[1])
        except (ValueError, IndexError) as e:
            events.LogEvent(appargs.CommAppArg.AppName, events.EventType.error, f"ERROR parsing barometer data: {e}, data: {recv_msg.data}")
            return

    # Receive IMU Data
    elif recv_msg.MsgID == appargs.ImuAppArg.MID_SendImuTlmData:
        sep_data = recv_msg.data.split(",")

        # Check the length of separated data
        if (len(sep_data) != 12):
            events.LogEvent(appargs.CommAppArg.AppName, events.EventType.error, f"ERROR receiving IMU, expected 12 fields, got {len(sep_data)}")
            return
        
        try:
            tlm_data.filtered_roll = float(sep_data[0])
            tlm_data.filtered_pitch = float(sep_data[1])
            tlm_data.filtered_yaw = float(sep_data[2])
            
            tlm_data.acc_roll = float(sep_data[3])
            tlm_data.acc_pitch = float(sep_data[4])
            tlm_data.acc_yaw = float(sep_data[5])

            tlm_data.mag_roll = float(sep_data[6])
            tlm_data.mag_pitch = float(sep_data[7])
            tlm_data.mag_yaw = float(sep_data[8])
        
            tlm_data.gyro_roll = float(sep_data[9])
            tlm_data.gyro_pitch = float(sep_data[10])
            tlm_data.gyro_yaw = float(sep_data[11])
        except (ValueError, IndexError) as e:
            events.LogEvent(appargs.CommAppArg.AppName, events.EventType.error, f"ERROR parsing IMU data: {e}, data: {recv_msg.data}")
            return

    # Receive GPS Data
    elif recv_msg.MsgID == appargs.GpsAppArg.MID_SendGpsTlmData:
        sep_data = recv_msg.data.split(",")

        # Check the length of separated data
        if (len(sep_data) != 5):
            events.LogEvent(appargs.CommAppArg.AppName, events.EventType.error, f"ERROR receiving GPS, expected 5 fields, got {len(sep_data)}")
            return

        try:
            tlm_data.gps_time = str(sep_data[0])
            tlm_data.gps_alt = float(sep_data[1])
            tlm_data.gps_lat = float(sep_data[2])
            tlm_data.gps_lon = float(sep_data[3])
            tlm_data.gps_sats = int(sep_data[4])
            
            # GPS TLM 수신 로그 비활성화
            # if tlm_data.gps_lat != 0.0 and tlm_data.gps_lon != 0.0:
            #     events.LogEvent(appargs.CommAppArg.AppName, events.EventType.info, f"GPS TLM received: Lat={tlm_data.gps_lat:.6f}, Lon={tlm_data.gps_lon:.6f}, Alt={tlm_data.gps_alt:.2f}m, Sats={tlm_data.gps_sats}")
            # else:
            #     events.LogEvent(appargs.CommAppArg.AppName, events.EventType.info, f"GPS TLM received (no fix): Lat={tlm_data.gps_lat:.6f}, Lon={tlm_data.gps_lon:.6f}, Alt={tlm_data.gps_alt:.2f}m, Sats={tlm_data.gps_sats}, Time={tlm_data.gps_time}")
        except (ValueError, IndexError) as e:
            events.LogEvent(appargs.CommAppArg.AppName, events.EventType.error, f"ERROR parsing GPS data: {e}, data: {recv_msg.data}")
            return

    # Receive Electro Sensor Data
    elif recv_msg.MsgID == appargs.ElectroAppArg.MID_SendElectroTlmData:
        sep_data = recv_msg.data.split(",")
        
        # Check the length of separated data
        if (len(sep_data) != 3):
            events.LogEvent(appargs.CommAppArg.AppName, events.EventType.error, f"ERROR receiving voltage, expected 3 fields, got {len(sep_data)}")
            return
        
        try:
            tlm_data.voltage = float(sep_data[0])
            tlm_data.current = float(sep_data[1])
            tlm_data.power = float(sep_data[2])
        except (ValueError, IndexError) as e:
            events.LogEvent(appargs.CommAppArg.AppName, events.EventType.error, f"ERROR parsing voltage data: {e}, data: {recv_msg.data}")
            return
    
    elif recv_msg.MsgID == appargs.DistanceAppArg.MID_SendDistanceTlmData:
        try:
            tlm_data.distance = float(recv_msg.data)
        except (ValueError) as e:
            events.LogEvent(appargs.CommAppArg.AppName, events.EventType.error, f"ERROR parsing distance data: {e}, data: {recv_msg.data}")
            return
    
    elif recv_msg.MsgID == appargs.FlightlogicAppArg.MID_SendCurrentStateToTlm:
        tlm_data.state = recv_msg.data

    # Receive Simulation Status Data
    elif recv_msg.MsgID == appargs.FlightlogicAppArg.MID_SendSimulationStatustoTlm:
        tlm_data.mode = recv_msg.data

    else:
        events.LogEvent(appargs.CommAppArg.AppName, events.EventType.error, f"MID {recv_msg.MsgID} not handled")
    return

def send_hk(Main_Queue : Queue):
    global COMMAPP_RUNSTATUS
    while COMMAPP_RUNSTATUS:
        commHK = msgstructure.MsgStructure()
        msgstructure.send_msg(Main_Queue, commHK, appargs.CommAppArg.AppID, appargs.HkAppArg.AppID, appargs.CommAppArg.MID_SendHK, str(COMMAPP_RUNSTATUS))
        time.sleep(1)
    return

######################################################
## INITIALIZATION, TERMINATION                      ##
######################################################

# Initialization
def commapp_init():
    global COMMAPP_RUNSTATUS
    global TEAMID

    try:
        # Disable Keyboardinterrupt since Termination is handled by parent process
        signal.signal(signal.SIGINT, signal.SIG_IGN)

        events.LogEvent(appargs.CommAppArg.AppName, events.EventType.info, "Initializating commapp")
        ## User Defined Initialization goes HERE

        serial_instance = uartserial.init_serial()

        # Set Team ID (AAS 2026)
        tlm_data.team_id = TEAMID

        # Reset the XBee
        events.LogEvent(appargs.CommAppArg.AppName, events.EventType.info, "Sending XBEE reset pulse...")
        xbeereset.send_reset_pulse()

        # Initialize 
        events.LogEvent(appargs.CommAppArg.AppName, events.EventType.info, "Commapp Initialization Complete")

        return serial_instance
    
    except Exception as e:
        events.LogEvent(appargs.CommAppArg.AppName, events.EventType.error, f"Error during initialization : {e}")
        COMMAPP_RUNSTATUS = False

# Termination
def commapp_terminate(serial_instance):
    global COMMAPP_RUNSTATUS

    COMMAPP_RUNSTATUS = False
    events.LogEvent(appargs.CommAppArg.AppName, events.EventType.info, "Terminating commapp")
    # Termination Process Comes Here

    uartserial.terminate_serial(serial_instance)

    # Join Each Thread to make sure all threads terminates
    for thread_name in thread_dict:
        events.LogEvent(appargs.CommAppArg.AppName, events.EventType.info, f"Terminating thread {thread_name}")
        thread_dict[thread_name].join()
        events.LogEvent(appargs.CommAppArg.AppName, events.EventType.info, f"Terminating thread {thread_name} Complete")

    # The termination flag should switch to false AFTER ALL TERMINATION PROCESS HAS ENDED
    events.LogEvent(appargs.CommAppArg.AppName, events.EventType.info, "Terminating commapp complete")
    return

######################################################
## USER METHOD                                      ##
######################################################

# Put user-defined methods here!

class _tlm_data_format:
    team_id :int = 3139
    mission_time : str = "00:00:00"
    packet_count : int = 0
    mode : str = "F"
    state : str = "LAUNCH_PAD"
    altitude : float = 0.0
    temperature : float = 0.0
    pressure : float = 0.0
    voltage : float = 0.0
    current : float = 0.0
    power : float = 0.0
    ina238_temp : float = 0.0
    gyro_roll : float = 0.0
    gyro_pitch : float = 0.0
    gyro_yaw : float = 0.0
    acc_roll : float = 0.0
    acc_pitch : float = 0.0
    acc_yaw : float = 0.0
    mag_roll : float = 0.0
    mag_pitch : float = 0.0
    mag_yaw : float = 0.0
    rot_rate : float = 0.0
    gps_lat : float = 0.0
    gps_lon : float = 0.0
    gps_alt : float = 0.0
    gps_time : str = "00:00:00"
    gps_sats : float = 0.0
    filtered_roll : float = 0.0
    filtered_pitch : float = 0.0
    filtered_yaw : float = 0.0
    cmd_echo : str = "None"
    distance : float = 0.0

tlm_data = _tlm_data_format()
TELEMETRY_ENABLE = True

# This function Sends telemetry to Ground Station
def send_tlm(serial_instance):
    global COMMAPP_RUNSTATUS
    global tlm_data
    global TELEMETRY_ENABLE

    while COMMAPP_RUNSTATUS:
        current_time = get_current_time()
        tlm_data.mission_time = current_time.strftime("%H:%M:%S")

        if TELEMETRY_ENABLE:
            tlm_data.packet_count += 1

        tlm_to_send = ",".join([str(tlm_data.team_id),
                    tlm_data.mission_time,
                    str(tlm_data.packet_count),
                    tlm_data.mode,
                    tlm_data.state,
                    f"{tlm_data.altitude:.2f}",
                    f"{tlm_data.temperature:.2f}",
                    f"{0.1 * tlm_data.pressure:.2f}", # from hPa to kPa
                    f"{tlm_data.voltage:.2f}",
                    f"{tlm_data.current:.2f}",
                    f"{tlm_data.power:.2f}",
                    f"{tlm_data.gyro_roll:.4f}",
                    f"{tlm_data.gyro_pitch:.4f}",
                    f"{tlm_data.gyro_yaw:.4f}",
                    f"{tlm_data.acc_roll:.4f}",
                    f"{tlm_data.acc_pitch:.4f}",
                    f"{tlm_data.acc_yaw:.4f}",
                    f"{0.01 * tlm_data.mag_roll:.4f}", # from microT to G
                    f"{0.01 * tlm_data.mag_pitch:.4f}",
                    f"{0.01 * tlm_data.mag_yaw:.4f}",
                    f"{tlm_data.rot_rate:.2f}",
                    str(tlm_data.gps_time),
                    f"{tlm_data.gps_alt:.2f}",
                    f"{tlm_data.gps_lat:.2f}",
                    f"{tlm_data.gps_lon:.2f}",
                    f"{tlm_data.gps_sats:.2f}",
                    tlm_data.cmd_echo,
                    f','
                    f"{tlm_data.filtered_roll:.4f}",
                    f"{tlm_data.filtered_pitch:.4f}",
                    f"{tlm_data.filtered_yaw:.4f}"])+"\n"

        #events.LogEvent(appargs.CommAppArg.AppName, events.EventType.info,tlm_to_send)

        tlm_debug_text = f"\nID : {tlm_data.team_id} TIME : {tlm_data.mission_time}, PCK_CNT : {tlm_data.packet_count}, MODE : {tlm_data.mode}, STATE : {tlm_data.state}\n"\
                f"Barometer : {tlm_data.altitude} , {tlm_data.temperature}, {tlm_data.pressure}\n" \
                 f"Elctro : {tlm_data.voltage}, {tlm_data.current}, {tlm_data.power}\n" \
                 f"IMU : Gyro({tlm_data.gyro_roll}, {tlm_data.gyro_pitch}, {tlm_data.gyro_yaw}), " \
                 f"Accel({tlm_data.acc_roll}, {tlm_data.acc_pitch}, {tlm_data.acc_yaw}), " \
                 f"Mag({tlm_data.mag_roll}, {tlm_data.mag_pitch}, {tlm_data.mag_yaw})\n" \
                 f"Euler angle({tlm_data.filtered_roll:.4f}, {tlm_data.filtered_pitch:.4f}, {tlm_data.filtered_yaw:.4f}) \n" \
                 f"GPS : Lat({tlm_data.gps_lat:.6f}), Lon({tlm_data.gps_lon:.6f}), Alt({tlm_data.gps_alt:.2f}), " \
                 f"Time({tlm_data.gps_time}), Sats({tlm_data.gps_sats})\n"

        events.LogEvent(appargs.CommAppArg.AppName, events.EventType.debug, tlm_debug_text)

        # Only send telemetry when telemetry is enabled
        if TELEMETRY_ENABLE:
            uartserial.send_serial_data(serial_instance, tlm_to_send)

        time.sleep(1)
    return

# Import regular expression module
import re

def cmd_cx(option:str, Main_Queue:Queue):
    global TELEMETRY_ENABLE
    global tlm_data

    if option == "ON":
        events.LogEvent(appargs.CommAppArg.AppName, events.EventType.info, "Enabling Telemetry")
        TELEMETRY_ENABLE = True
    
    if option == "OFF":
        events.LogEvent(appargs.CommAppArg.AppName, events.EventType.info, "Disabling Telemetry")
        TELEMETRY_ENABLE = False
        tlm_data.packet_count = 0

    return

def cmd_st(option:str, Main_Queue:Queue):

    events.LogEvent(appargs.CommAppArg.AppName, events.EventType.info, "Setting Time")
    set_timedelta(option)

    return

def cmd_sim(option:str, Main_Queue:Queue):
    RouteSimCmdMsg = msgstructure.MsgStructure()

    # Route the simulation command to flightlogic app
    msgstructure.send_msg(Main_Queue, RouteSimCmdMsg, appargs.CommAppArg.AppID, appargs.FlightlogicAppArg.AppID, appargs.CommAppArg.MID_RouteCmd_SIM, option)
    return

def cmd_simp(option:str, Main_Queue:Queue):
    global tlm_data
    global SIMP_OFFSET

    RouteSimpCmdMsg = msgstructure.MsgStructure()
    
    sea_level_pressure = 1013.25

    recv_simp = float(option) / 100
    recv_alt = 44307.7 * (1 - (recv_simp / sea_level_pressure) ** 0.190284)

    tlm_data.pressure = recv_simp
    tlm_data.altitude = recv_alt - SIMP_OFFSET

    # Route the simulation pressure value to flightlogic app
    msgstructure.send_msg(Main_Queue, RouteSimpCmdMsg, appargs.CommAppArg.AppID, appargs.FlightlogicAppArg.AppID, appargs.CommAppArg.MID_RouteCmd_SIMP, str(tlm_data.altitude))
    
    return

def cmd_cal(option:str, Main_Queue:Queue):
    global tlm_data
    global SIMP_OFFSET

    # If flight mod
    if tlm_data.mode == "F":
        RouteCalCmdMsg = msgstructure.MsgStructure()

        # Route The Calibration command to Baromter app
        msgstructure.send_msg(Main_Queue, RouteCalCmdMsg, appargs.CommAppArg.AppID, appargs.BarometerAppArg.AppID, appargs.CommAppArg.MID_RouteCmd_CAL, "")
    
    # If simulation mode
    if tlm_data.mode == "S":
        SIMP_OFFSET = tlm_data.altitude
        ResetBarometerMaxAltCmd = msgstructure.MsgStructure()
        msgstructure.send_msg(Main_Queue, ResetBarometerMaxAltCmd, appargs.CommAppArg.AppID, appargs.FlightlogicAppArg.AppID, appargs.BarometerAppArg.MID_ResetBarometerMaxAlt, "")

    return

def cmd_mec(option:str, Main_Queue:Queue):
    RouteMecCmdMsg = msgstructure.MsgStructure()

    # Route the mechanism activation command to motor app
    msgstructure.send_msg(Main_Queue, RouteMecCmdMsg, appargs.CommAppArg.AppID, appargs.motorAppArg.AppID, appargs.CommAppArg.MID_RouteCmd_MEC, option)

    return

def cmd_ss(option:str, Main_Queue:Queue):

    RouteSsCmdMsg = msgstructure.MsgStructure()
    msgstructure.send_msg(Main_Queue, RouteSsCmdMsg, appargs.CommAppArg.AppID, appargs.FlightlogicAppArg.AppID, appargs.CommAppArg.MID_RouteCmd_SS, option)

    return

def cmd_rbt(option:str, Main_Queue:Queue):
    
    events.LogEvent(appargs.CommAppArg.AppName, events.EventType.info, "RBT command received. Restarting...")
    os.system('systemctl reboot -i')
    return

def cmd_cam(option:str, Main_Queue:Queue):

    RouteCamCmdMsg = msgstructure.MsgStructure()
    msgstructure.send_msg(Main_Queue, RouteCamCmdMsg, appargs.CommAppArg.AppID, appargs.CameraAppArg.AppID, appargs.CommAppArg.MID_RouteCmd_CAM, option)
    return

# This fuction reads command from Ground Station
def read_cmd(Main_Queue:Queue, serial_instance):
    global COMMAPP_RUNSTATUS
    global tlm_data
    global TEAMID

    # Regular expression for commands

    # Payload Telemetry ON/OFF
    cx_re_header = f"CMD,{TEAMID},CX," 
    cx_re_option = "(ON|OFF)$"

    # Set time
    st_re_header = f"CMD,{TEAMID},ST,"
    st_re_option = r"(([01]\d|2[0-3])(:[0-5]\d){2}|GPS)$"

    # Simulation Mode
    sim_re_header = f"CMD,{TEAMID},SIM," # Simulation mode control
    sim_re_option = "(ENABLE|ACTIVATE|DISABLE)$"

    # Simulated pressure
    simp_re_header = f"CMD,{TEAMID},SIMP,"
    simp_re_option = r"\d{5,6}$"

    # Altitude Calibration
    cal_re_header = f"CMD,{TEAMID},CAL"
    cal_re_option = "$"

    # Mechanism activation
    mec_re_header = f"CMD,{TEAMID},MEC,MOTOR,"
    mec_re_option = "(ON|OFF)$"

    # Set State
    ss_re_header = f"CMD,{TEAMID},SS,"
    ss_re_option = r"\d{1}$"
    
    rbt_re_header = f"CMD,{TEAMID},RBT"
    rbt_re_option = "$"

    # Camera Control
    cam_re_header = f"CMD,{TEAMID},CAM,"
    cam_re_option = "(ON|OFF)$"

    while COMMAPP_RUNSTATUS:
        try:
            # Check if serial instance is valid before receiving data
            if serial_instance is None:
                time.sleep(1)
                continue
                
            rcv_cmd = uartserial.receive_serial_data(serial_instance)

            # When Timeout Occurs ; empty data is read, continue
            if not rcv_cmd or rcv_cmd is None:
                continue
            
            # Ensure rcv_cmd is a string before processing
            if not isinstance(rcv_cmd, str):
                continue
            
            # Ignore XBEE responses like "OK", empty lines, etc.
            rcv_cmd = rcv_cmd.strip()
            if rcv_cmd in ("OK", "", "\r", "\n", "\r\n"):
                continue
            
            events.LogEvent(appargs.CommAppArg.AppName, events.EventType.info, f"Received Command : {rcv_cmd}")
            
            # Validate commmand using regex

            # cx
            if re.fullmatch(cx_re_header+cx_re_option, rcv_cmd):

                # Change the CMD Echo String
                set_cmdecho(rcv_cmd)

                # Parse the option
                option_match = re.search(cx_re_option, rcv_cmd)
                if option_match:
                    option = option_match.group()
                    cmd_cx(option, Main_Queue)
                else:
                    events.LogEvent(appargs.CommAppArg.AppName, events.EventType.error, f"Failed to parse option from command: {rcv_cmd}")

            # st
            elif re.fullmatch(st_re_header+st_re_option, rcv_cmd):

                # Change the CMD Echo String
                set_cmdecho(rcv_cmd)

                option_match = re.search(st_re_option, rcv_cmd)
                if option_match:
                    option = option_match.group()
                    cmd_st(option, Main_Queue)
                else:
                    events.LogEvent(appargs.CommAppArg.AppName, events.EventType.error, f"Failed to parse option from command: {rcv_cmd}")

            # sim
            elif re.fullmatch(sim_re_header+sim_re_option, rcv_cmd):

                # Change the CMD Echo String
                set_cmdecho(rcv_cmd)

                option_match = re.search(sim_re_option, rcv_cmd)
                if option_match:
                    option = option_match.group()
                    cmd_sim(option, Main_Queue)
                else:
                    events.LogEvent(appargs.CommAppArg.AppName, events.EventType.error, f"Failed to parse option from command: {rcv_cmd}")

            # simp
            elif re.fullmatch(simp_re_header+simp_re_option, rcv_cmd):

                # Change the CMD Echo String
                set_cmdecho(rcv_cmd)

                option_match = re.search(simp_re_option, rcv_cmd)
                if option_match:
                    option = option_match.group()
                    cmd_simp(option, Main_Queue)
                else:
                    events.LogEvent(appargs.CommAppArg.AppName, events.EventType.error, f"Failed to parse option from command: {rcv_cmd}")

            # cal
            elif re.fullmatch(cal_re_header+cal_re_option, rcv_cmd):

                # Change the CMD Echo String
                set_cmdecho(rcv_cmd)

                # Calibration command has no option
                option = ""
                cmd_cal(option, Main_Queue)

            # mec
            elif re.fullmatch(mec_re_header+mec_re_option, rcv_cmd):

                # Change the CMD Echo String
                set_cmdecho(rcv_cmd)

                option_match = re.search(mec_re_option, rcv_cmd)
                if option_match:
                    option = option_match.group()
                    cmd_mec(option, Main_Queue)
                else:
                    events.LogEvent(appargs.CommAppArg.AppName, events.EventType.error, f"Failed to parse option from command: {rcv_cmd}")
            
            #ss
            elif re.fullmatch(ss_re_header+ss_re_option, rcv_cmd):

                # Change the CMD Echo String
                set_cmdecho(rcv_cmd)

                option_match = re.search(ss_re_option, rcv_cmd)
                if option_match:
                    option = option_match.group()
                    cmd_ss(option, Main_Queue)
                else:
                    events.LogEvent(appargs.CommAppArg.AppName, events.EventType.error, f"Failed to parse option from command: {rcv_cmd}")

            elif re.fullmatch(rbt_re_header+rbt_re_option, rcv_cmd):
                set_cmdecho(rcv_cmd)

                # Reboot command has no option
                option = ""
                cmd_rbt(option, Main_Queue)

            elif re.fullmatch(cam_re_header+cam_re_option, rcv_cmd):
                set_cmdecho(rcv_cmd)

                option_match = re.search(cam_re_option, rcv_cmd)
                if option_match:
                    option = option_match.group()
                    # Activate Camera
                    cmd_cam(option, Main_Queue)
                else:
                    events.LogEvent(appargs.CommAppArg.AppName, events.EventType.error, f"Failed to parse option from command: {rcv_cmd}")

            else:
                events.LogEvent(appargs.CommAppArg.AppName, events.EventType.error, f"Invalid command {rcv_cmd}")

        except Exception as e:
            # Only log if we're not terminating (to avoid noise during shutdown)
            # But check COMMAPP_RUNSTATUS again after exception (it may have changed)
            if COMMAPP_RUNSTATUS:
                try:
                    events.LogEvent(appargs.CommAppArg.AppName, events.EventType.error, f"Error receiving command, Sleeping 1 second :  {e}")
                except:
                    # If logging fails (e.g., during shutdown), just continue
                    pass
            # Check run status before sleeping (if terminating, exit immediately)
            if not COMMAPP_RUNSTATUS:
                break
            try:
                time.sleep(1)
            except:
                # If sleep fails (shouldn't happen, but be safe), break
                break

    return

def set_cmdecho(cmd_str:str):
    global tlm_data
    #remove comma from command
    tlm_data.cmd_echo = cmd_str.replace(",","")

# Get the current time with timedelta applied
def get_current_time() -> datetime :
    global ST_timedelta

    current_time = datetime.now()

    return current_time - ST_timedelta

# Set the timedelta to given timestr
def set_timedelta(timestr:str) :
    global ST_timedelta
    global tlm_data

    curtime = datetime.now()
    # Timestr is always given in HH:MM:SS format or "GPS"
    if timestr == "GPS":
        # Check if GPS time is valid before parsing
        if tlm_data.gps_time is None or tlm_data.gps_time == "00:00:00" or ":" not in str(tlm_data.gps_time):
            events.LogEvent(appargs.CommAppArg.AppName, events.EventType.error, "Cannot set time from GPS: GPS time is not available")
            return
        
        try:
            h,m,s = map(int, tlm_data.gps_time.split(":"))
            gps_delta = curtime
            gps_delta = gps_delta.replace(hour=h, minute=m, second=s)

            ST_timedelta = curtime - gps_delta
        except (ValueError, AttributeError) as e:
            events.LogEvent(appargs.CommAppArg.AppName, events.EventType.error, f"Error parsing GPS time '{tlm_data.gps_time}': {e}")
            return

    else:
        try:
            h, m, s = map(int, timestr.split(":"))
            timestr_delta = curtime

            timestr_delta = timestr_delta.replace(hour=h, minute=m, second=s)

            ST_timedelta = curtime - timestr_delta
        except (ValueError, AttributeError) as e:
            events.LogEvent(appargs.CommAppArg.AppName, events.EventType.error, f"Error parsing time string '{timestr}': {e}")
            return
    return
        
######################################################
## MAIN METHOD                                      ##
######################################################

thread_dict = dict[str, threading.Thread]()

# This method is called from main app. Initialization, runloop process
def commapp_main(Main_Queue : Queue, Main_Pipe : connection.Connection):
    global COMMAPP_RUNSTATUS
    COMMAPP_RUNSTATUS = True

    # Initialization Process
    serial_instance = commapp_init()

    # Spawn SB Message Listner Thread
    thread_dict["HKSender_Thread"] = threading.Thread(target=send_hk, args=(Main_Queue, ), name="HKSender_Thread")
    thread_dict["TlmSender_Thread"] = threading.Thread(target=send_tlm, args=(serial_instance, ), name="TlmSender_Thread")
    thread_dict["CmdReader_Thread"] = threading.Thread(target=read_cmd, args=(Main_Queue, serial_instance), name="CmdReader_Thread")

    # Spawn Each Threads
    for thread_name in thread_dict:
        thread_dict[thread_name].start()

    try:
        while COMMAPP_RUNSTATUS:
            # Receive Message From Pipe
            message = Main_Pipe.recv()
            recv_msg = msgstructure.MsgStructure()

            # Unpack Message, Skip this message if unpacked message is not valid
            if msgstructure.unpack_msg(recv_msg, message) == False:
                continue
            
            # Validate Message, Skip this message if target AppID different from commapp's AppID
            # Exception when the message is from main app
            if recv_msg.receiver_app == appargs.CommAppArg.AppID or recv_msg.receiver_app == appargs.MainAppArg.AppID:
                # Handle Command According to Message ID
                command_handler(recv_msg)
            else:
                events.LogEvent(appargs.CommAppArg.AppName, events.EventType.error, "Receiver MID does not match with commapp MID")

    # If error occurs, terminate app
    except Exception as e:
        events.LogEvent(appargs.CommAppArg.AppName, events.EventType.error, f"commapp error : {e}")
        COMMAPP_RUNSTATUS = False

    # Termination Process after runloop
    commapp_terminate(serial_instance)

    return
