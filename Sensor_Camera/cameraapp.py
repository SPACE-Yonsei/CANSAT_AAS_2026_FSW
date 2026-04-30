"""Camera app baseline."""

from __future__ import annotations

import threading
import time

from lib import appargs, msgstructure
from Sensor_Camera import picam


CAMERAAPP_RUNSTATUS = True
PICAM_RECORDING = False
CAMERA_HEALTH = 0
SEGMENT_SEC = 1.0


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
        elif unpacked.data.strip().upper() == "OFF":
            picam_stop_recording()
    elif unpacked.msg_id == appargs.CameraAppArg.MID_cam_activate:
        picam_start_recording()


def picam_record_thread(cam, enc) -> None:
    global CAMERA_HEALTH
    while CAMERAAPP_RUNSTATUS:
        if PICAM_RECORDING:
            out = picam.record(cam, enc, SEGMENT_SEC)
            CAMERA_HEALTH = 1 if out is not None else 0
        else:
            time.sleep(0.1)


def cameraapp_main(main_pipe) -> None:
    cam, enc = picam.init_cam()
    if cam is not None:
        global CAMERA_HEALTH
        CAMERA_HEALTH = 1
    t = threading.Thread(target=picam_record_thread, args=(cam, enc), daemon=True)
    t.start()
    try:
        while CAMERAAPP_RUNSTATUS:
            try:
                has_msg = main_pipe.poll(0.1)
            except (KeyboardInterrupt, EOFError, OSError):
                break
            if has_msg:
                command_handler(main_pipe.recv())
    except KeyboardInterrupt:
        pass
    finally:
        picam.terminate(cam)
