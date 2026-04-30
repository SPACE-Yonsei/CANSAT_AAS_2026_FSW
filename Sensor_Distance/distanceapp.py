"""Distance app baseline."""

from __future__ import annotations

import threading
import time

from lib import appargs, msgstructure


DISTANCEAPP_RUNSTATUS = True
DISTANCE_MM = 5000.0


def command_handler(recv_msg: str) -> None:
    global DISTANCEAPP_RUNSTATUS
    unpacked = msgstructure.unpack_msg(recv_msg)
    if unpacked is False:
        return
    if unpacked.msg_id == appargs.MainAppArg.MID_TerminateProcess:
        DISTANCEAPP_RUNSTATUS = False


def read_distance_data() -> None:
    global DISTANCE_MM
    while DISTANCEAPP_RUNSTATUS:
        DISTANCE_MM = max(500.0, DISTANCE_MM - 5.0)
        time.sleep(0.1)


def send_distance_data(main_queue) -> None:
    while DISTANCEAPP_RUNSTATUS:
        msgstructure.send_msg(
            main_queue,
            appargs.DistanceAppArg.AppID,
            appargs.FlightlogicAppArg.AppID,
            appargs.DistanceAppArg.MID_flight_dis,
            f"{DISTANCE_MM}",
        )
        msgstructure.send_msg(
            main_queue,
            appargs.DistanceAppArg.AppID,
            appargs.CommAppArg.AppID,
            appargs.DistanceAppArg.MID_comm_dis,
            f"{DISTANCE_MM}",
        )
        time.sleep(0.2)


def distanceapp_main(main_queue, main_pipe) -> None:
    t1 = threading.Thread(target=read_distance_data, daemon=True)
    t2 = threading.Thread(target=send_distance_data, args=(main_queue,), daemon=True)
    t1.start()
    t2.start()
    while DISTANCEAPP_RUNSTATUS:
        if main_pipe.poll(0.1):
            command_handler(main_pipe.recv())
