"""IMU app baseline with synthetic attitude stream."""

from __future__ import annotations

import threading
import time

from lib import appargs, msgstructure


IMUAPP_RUNSTATUS = True
YAW = 0.0
GYRZ = 0.0
HEALTH = 1


def command_handler(recv_msg: str) -> None:
    global IMUAPP_RUNSTATUS
    unpacked = msgstructure.unpack_msg(recv_msg)
    if unpacked is False:
        return
    if unpacked.msg_id == appargs.MainAppArg.MID_TerminateProcess:
        IMUAPP_RUNSTATUS = False


def read_imu_data() -> None:
    global YAW, GYRZ
    while IMUAPP_RUNSTATUS:
        YAW = (YAW + 0.5) % 360.0
        GYRZ = 0.5
        time.sleep(0.01)


def send_imu_data(main_queue) -> None:
    tick = 0
    while IMUAPP_RUNSTATUS:
        msgstructure.send_msg(
            main_queue,
            appargs.ImuAppArg.AppID,
            appargs.MotorAppArg.AppID,
            appargs.ImuAppArg.MID_motor_imu,
            f"{YAW},{GYRZ},{HEALTH}",
        )
        tick += 1
        if tick >= 10:
            tick = 0
            msgstructure.send_msg(
                main_queue,
                appargs.ImuAppArg.AppID,
                appargs.CommAppArg.AppID,
                appargs.ImuAppArg.MID_comm_euler,
                f"0,0,{YAW},0,0,0,0,0,0,0,0,{GYRZ}",
            )
        time.sleep(0.1)


def imuapp_main(main_queue, main_pipe) -> None:
    t1 = threading.Thread(target=read_imu_data, daemon=True)
    t2 = threading.Thread(target=send_imu_data, args=(main_queue,), daemon=True)
    t1.start()
    t2.start()
    while IMUAPP_RUNSTATUS:
        if main_pipe.poll(0.1):
            command_handler(main_pipe.recv())
