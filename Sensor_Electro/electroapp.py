"""Electro app baseline."""

from __future__ import annotations

import threading
import time

from lib import appargs, msgstructure


ELECTROAPP_RUNSTATUS = True
VOLT = 7.4
CURR = 0.8
PWR = 5.92


def command_handler(recv_msg: str) -> None:
    global ELECTROAPP_RUNSTATUS
    unpacked = msgstructure.unpack_msg(recv_msg)
    if unpacked is False:
        return
    if unpacked.msg_id == appargs.MainAppArg.MID_TerminateProcess:
        ELECTROAPP_RUNSTATUS = False


def read_electro_data() -> None:
    while ELECTROAPP_RUNSTATUS:
        time.sleep(1.0)


def send_electro_data(main_queue) -> None:
    while ELECTROAPP_RUNSTATUS:
        msgstructure.send_msg(
            main_queue,
            appargs.ElectroAppArg.AppID,
            appargs.CommAppArg.AppID,
            appargs.ElectroAppArg.MID_comm_volt,
            f"{VOLT},{CURR},{PWR}",
        )
        time.sleep(1.0)


def electroapp_main(main_queue, main_pipe) -> None:
    t1 = threading.Thread(target=read_electro_data, daemon=True)
    t2 = threading.Thread(target=send_electro_data, args=(main_queue,), daemon=True)
    t1.start()
    t2.start()
    while ELECTROAPP_RUNSTATUS:
        if main_pipe.poll(0.1):
            command_handler(main_pipe.recv())
