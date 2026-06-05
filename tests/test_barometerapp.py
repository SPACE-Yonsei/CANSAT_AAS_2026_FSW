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

    def test_cal_noarg_snaps_to_zero_repeatable(self):
        # No-arg CAL snaps current altitude to 0; repeated CAL must keep it at 0
        # (regression: offset must accumulate, not reset to relative ALTITUDE).
        q = queue.Queue()

        def _send_cal():
            packed = msgstructure.pack_msg(
                msgstructure.fill_msg(
                    appargs.CommAppArg.AppID,
                    appargs.BarometerAppArg.AppID,
                    appargs.CommAppArg.MID_RouteCmd_CAL,
                    "",
                )
            )
            barometerapp.command_handler(q, packed)

        raw = 300.0  # raw absolute altitude on the pad

        # First CAL: offset jumps to raw, relative altitude -> 0
        barometerapp.ALTITUDE = raw - barometerapp.BAROMETER_OFFSET
        _send_cal()
        self.assertAlmostEqual(barometerapp.BAROMETER_OFFSET, raw)
        self.assertAlmostEqual(raw - barometerapp.BAROMETER_OFFSET, 0.0)

        # Second CAL at the same raw altitude must keep it calibrated to 0.
        barometerapp.ALTITUDE = raw - barometerapp.BAROMETER_OFFSET
        _send_cal()
        self.assertAlmostEqual(barometerapp.BAROMETER_OFFSET, raw)
        self.assertAlmostEqual(raw - barometerapp.BAROMETER_OFFSET, 0.0)

    def test_synthetic_raw_shape(self):
        p, t, a = barometerapp._synthetic_raw()
        self.assertEqual((p, t, a), (0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
