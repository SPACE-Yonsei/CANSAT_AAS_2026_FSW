import queue
import unittest

from Sensor_Barometer import barometerapp
from lib import appargs, msgstructure


class TestBarometerApp(unittest.TestCase):
    def setUp(self):
        barometerapp.BAROMETER_OFFSET = 0.0
        barometerapp.ALTITUDE = 10.0

    def test_median(self):
        self.assertEqual(barometerapp._median([3, 1, 2]), 2.0)
        self.assertEqual(barometerapp._median([1, 2, 3, 4]), 2.5)

    def test_cal_command_updates_offset(self):
        q = queue.Queue()
        packed = msgstructure.pack_msg(
            msgstructure.fill_msg(
                appargs.CommAppArg.AppID,
                appargs.BarometerAppArg.AppID,
                appargs.CommAppArg.MID_RouteCmd_CAL,
                "CAL,5.0",
            )
        )
        barometerapp.command_handler(q, packed)
        self.assertAlmostEqual(barometerapp.BAROMETER_OFFSET, 5.0)
        self.assertFalse(q.empty())

    def test_synthetic_raw_shape(self):
        p, t, a = barometerapp._synthetic_raw()
        self.assertIsInstance(p, float)
        self.assertIsInstance(t, float)
        self.assertIsInstance(a, float)


if __name__ == "__main__":
    unittest.main()
