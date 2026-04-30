"""Barometer app baseline with synthetic telemetry."""

from __future__ import annotations

import time
import threading

from lib import appargs, msgstructure, prevstate


BAROMETERAPP_RUNSTATUS = True
ALTITUDE = 0.0
TEMPERATURE = 20.0
PRESSURE = 1013.25


def command_handler(main_queue, recv_msg: str, _barometer_instance=None) -> None:
    global BAROMETERAPP_RUNSTATUS, ALTITUDE
    unpacked = msgstructure.unpack_msg(recv_msg)
    if unpacked is False:
        return
    if unpacked.msg_id == appargs.MainAppArg.MID_TerminateProcess:
        BAROMETERAPP_RUNSTATUS = False
    elif unpacked.msg_id == appargs.CommAppArg.MID_RouteCmd_CAL:
        parts = [x.strip() for x in unpacked.data.split(",")]
        if len(parts) == 2:
            try:
                ALTITUDE += float(parts[1])
            except ValueError:
                pass
        else:
            ALTITUDE = 0.0
        prevstate.update_altcal(ALTITUDE)
        msgstructure.send_msg(
            main_queue,
            appargs.BarometerAppArg.AppID,
            appargs.FlightlogicAppArg.AppID,
            appargs.CommAppArg.MID_RouteCmd_CAL,
            "RESET",
        )


def read_barometer_data() -> None:
    global ALTITUDE
    while BAROMETERAPP_RUNSTATUS:
        ALTITUDE += 0.01
        time.sleep(0.1)


def send_barometer_data(main_queue) -> None:
    tick = 0
    while BAROMETERAPP_RUNSTATUS:
        msgstructure.send_msg(
            main_queue,
            appargs.BarometerAppArg.AppID,
            appargs.FlightlogicAppArg.AppID,
            appargs.BarometerAppArg.MID_flight_alt,
            f"{ALTITUDE}",
        )
        msgstructure.send_msg(
            main_queue,
            appargs.BarometerAppArg.AppID,
            appargs.MotorAppArg.AppID,
            appargs.BarometerAppArg.MID_motor_alt,
            f"{ALTITUDE}",
        )
        tick += 1
        if tick >= 10:
            tick = 0
            msgstructure.send_msg(
                main_queue,
                appargs.BarometerAppArg.AppID,
                appargs.CommAppArg.AppID,
                appargs.BarometerAppArg.MID_comm_alt,
                f"{PRESSURE},{TEMPERATURE},{ALTITUDE}",
            )
        time.sleep(0.1)


def barometerapp_main(main_queue, main_pipe) -> None:
    t1 = threading.Thread(target=read_barometer_data, daemon=True)
    t2 = threading.Thread(target=send_barometer_data, args=(main_queue,), daemon=True)
    t1.start()
    t2.start()
    while BAROMETERAPP_RUNSTATUS:
        if main_pipe.poll(0.1):
            recv_msg = main_pipe.recv()
            command_handler(main_queue, recv_msg, None)
