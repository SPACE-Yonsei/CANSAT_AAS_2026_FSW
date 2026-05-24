"""Camera app baseline."""

from __future__ import annotations

import logging
import threading
import time

from lib import appargs, msgstructure
from Sensor_Camera import picam


CAMERAAPP_RUNSTATUS = True
PICAM_RECORDING = False
CAMERA_HEALTH = 0
SEGMENT_SEC = 7.0
logger = logging.getLogger(__name__)


def picam_start_recording() -> None:
    global PICAM_RECORDING
    if not PICAM_RECORDING:
        logger.info("Camera recording START")
    PICAM_RECORDING = True


def picam_stop_recording() -> None:
    global PICAM_RECORDING
    if PICAM_RECORDING:
        logger.info("Camera recording STOP")
    PICAM_RECORDING = False


def command_handler(recv_msg: str) -> None:
    global CAMERAAPP_RUNSTATUS
    unpacked = msgstructure.unpack_msg(recv_msg)
    if unpacked is False:
        return
    if unpacked.msg_id == appargs.MainAppArg.MID_TerminateProcess:
        CAMERAAPP_RUNSTATUS = False
        logger.info("Camera terminate command received")
    elif unpacked.msg_id == appargs.CommAppArg.MID_RouteCmd_CAM:
        if unpacked.data.strip().upper() == "ON":
            logger.info("Camera command CAM,ON received from Comm")
            picam_start_recording()
        elif unpacked.data.strip().upper() == "OFF":
            logger.info("Camera command CAM,OFF received from Comm")
            picam_stop_recording()
    elif unpacked.msg_id == appargs.CameraAppArg.MID_cam_activate:
        logger.info("Camera activation message received from FlightLogic")
        picam_start_recording()


def picam_record_thread(cam, enc) -> None:
    global CAMERA_HEALTH
    while CAMERAAPP_RUNSTATUS:
        if PICAM_RECORDING:
            out = picam.record(cam, enc, SEGMENT_SEC)
            is_real = out is not None and out.suffix != ".txt"
            CAMERA_HEALTH = 1 if is_real else 0
            if is_real:
                logger.debug("Camera segment saved: %s", out)
            else:
                logger.warning("Camera segment save failed (placeholder: %s)", out)
        else:
            time.sleep(0.1)


def cameraapp_main(main_pipe) -> None:
    cam, enc = picam.init_cam()
    global CAMERA_HEALTH
    CAMERA_HEALTH = 1 if getattr(cam, "available", False) else 0
    # Start recording immediately on FSW boot; CAM OFF can disable later.
    picam_start_recording()
    logger.info("Camera app started")
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
        logger.info("Camera app terminating")
        t.join(timeout=2.0)
        picam.terminate(cam)
