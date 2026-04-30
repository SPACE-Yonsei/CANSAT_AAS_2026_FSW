"""GPS app baseline with synthetic NMEA-derived data."""

from __future__ import annotations

import threading
import time

from lib import appargs, msgstructure


GPSAPP_RUNSTATUS = True
LAT = 37.5600
LON = 126.9300
ALT = 80.0


def command_handler(recv_msg: str) -> None:
    global GPSAPP_RUNSTATUS
    unpacked = msgstructure.unpack_msg(recv_msg)
    if unpacked is False:
        return
    if unpacked.msg_id == appargs.MainAppArg.MID_TerminateProcess:
        GPSAPP_RUNSTATUS = False


def read_and_send_gps_data(main_queue) -> None:
    global LAT, LON
    tick = 0
    while GPSAPP_RUNSTATUS:
        LAT += 0.000001
        LON += 0.000001
        payload_motor = f"{LAT},{LON},8.0,90.0,1,8,A,1"
        msgstructure.send_msg(
            main_queue,
            appargs.GpsAppArg.AppID,
            appargs.MotorAppArg.AppID,
            appargs.GpsAppArg.MID_motor_gps,
            payload_motor,
        )
        tick += 1
        if tick >= 10:
            tick = 0
            msgstructure.send_msg(
                main_queue,
                appargs.GpsAppArg.AppID,
                appargs.CommAppArg.AppID,
                appargs.GpsAppArg.MID_comm_gga,
                f"000000,{ALT},{LAT},{LON},8",
            )
        time.sleep(0.1)


def gpsapp_main(main_queue, main_pipe) -> None:
    t = threading.Thread(target=read_and_send_gps_data, args=(main_queue,), daemon=True)
    t.start()
    while GPSAPP_RUNSTATUS:
        if main_pipe.poll(0.1):
            command_handler(main_pipe.recv())
