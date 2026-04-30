"""Camera app baseline."""

from __future__ import annotations

import threading
import time

from lib import appargs, msgstructure
from Sensor_Camera import picam


CAMERAAPP_RUNSTATUS = True
PICAM_RECORDING = False


def picam_start_recording() -> None:
    global PICAM_RECORDING
    PICAM_RECORDING = True


def picam_stop_recording() -> None:
    global PICAM_RECORDING
    PICAM_RECORDING = False


def command_handler(recv_msg: str) -> None:
    global CAMERAAPP_RUNSTATUS
    unpacked = msgstructure.unpack_msg(recv_msg)
    if unpacked is False:
        return
    if unpacked.msg_id == appargs.MainAppArg.MID_TerminateProcess:
        CAMERAAPP_RUNSTATUS = False
    elif unpacked.msg_id == appargs.CommAppArg.MID_RouteCmd_CAM:
        if unpacked.data.strip().upper() == "ON":
            picam_start_recording()
        else:
            picam_stop_recording()
    elif unpacked.msg_id == appargs.CameraAppArg.MID_cam_activate:
        picam_start_recording()


def picam_record_thread(cam, enc) -> None:
    while CAMERAAPP_RUNSTATUS:
        if PICAM_RECORDING:
            picam.record(cam, enc, 1)
        else:
            time.sleep(0.1)


def cameraapp_main(main_pipe) -> None:
    cam, enc = picam.init_cam()
    t = threading.Thread(target=picam_record_thread, args=(cam, enc), daemon=True)
    t.start()
    while CAMERAAPP_RUNSTATUS:
        if main_pipe.poll(0.1):
            command_handler(main_pipe.recv())
    picam.terminate(cam)
