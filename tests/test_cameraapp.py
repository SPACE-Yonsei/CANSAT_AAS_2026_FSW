import tempfile
import unittest
from pathlib import Path

from Sensor_Camera import cameraapp, picam
from lib import appargs, msgstructure


class TestCameraApp(unittest.TestCase):
    def setUp(self):
        cameraapp.CAMERAAPP_RUNSTATUS = True
        cameraapp.PICAM_RECORDING = False

    def test_command_handler_on_off(self):
        on_msg = msgstructure.pack_msg(
            msgstructure.fill_msg(
                appargs.CommAppArg.AppID,
                appargs.CameraAppArg.AppID,
                appargs.CommAppArg.MID_RouteCmd_CAM,
                "ON",
            )
        )
        off_msg = msgstructure.pack_msg(
            msgstructure.fill_msg(
                appargs.CommAppArg.AppID,
                appargs.CameraAppArg.AppID,
                appargs.CommAppArg.MID_RouteCmd_CAM,
                "OFF",
            )
        )
        cameraapp.command_handler(on_msg)
        self.assertTrue(cameraapp.PICAM_RECORDING)
        cameraapp.command_handler(off_msg)
        self.assertFalse(cameraapp.PICAM_RECORDING)

    def test_terminate_message(self):
        term = msgstructure.pack_msg(
            msgstructure.fill_msg(
                appargs.MainAppArg.AppID,
                appargs.CameraAppArg.AppID,
                appargs.MainAppArg.MID_TerminateProcess,
                "",
            )
        )
        cameraapp.command_handler(term)
        self.assertFalse(cameraapp.CAMERAAPP_RUNSTATUS)


class TestPicam(unittest.TestCase):
    def test_fallback_record_creates_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            handle = picam.CameraHandle(cam=None, encoder=None, available=False, output_dir=Path(tmp))
            out = picam.record(handle, None, 0.0)
            self.assertIsNotNone(out)
            self.assertTrue(Path(out).exists())


if __name__ == "__main__":
    unittest.main()
