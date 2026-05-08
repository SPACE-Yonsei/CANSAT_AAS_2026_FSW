import queue
import unittest

from Sensor_Electro import electroapp
from lib import appargs, msgstructure


class TestElectroApp(unittest.TestCase):
    def setUp(self):
        electroapp.ELECTRO_HEALTH = 0
        electroapp.VOLT = 0.0
        electroapp.CURR = 0.0
        electroapp.PWR = 0.0
        electroapp._reader = None

    def test_synthetic_read_shape(self):
        v, c, p = electroapp._synthetic_read()
        self.assertEqual((v, c, p), (0.0, 0.0, 0.0))

    def test_read_sensor_fallback(self):
        sample = electroapp._read_sensor()
        self.assertIsNotNone(sample)
        self.assertEqual(len(sample), 3)

    def test_command_handler_terminate(self):
        electroapp.ELECTROAPP_RUNSTATUS = True
        packed = msgstructure.pack_msg(
            msgstructure.fill_msg(
                appargs.MainAppArg.AppID,
                appargs.ElectroAppArg.AppID,
                appargs.MainAppArg.MID_TerminateProcess,
                "",
            )
        )
        electroapp.command_handler(packed)
        self.assertFalse(electroapp.ELECTROAPP_RUNSTATUS)


if __name__ == "__main__":
    unittest.main()
