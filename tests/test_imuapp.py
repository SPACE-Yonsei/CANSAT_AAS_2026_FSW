import unittest

from Sensor_Imu import imuapp
from lib import prevstate


class TestImuApp(unittest.TestCase):
    def test_wrap_deg(self):
        self.assertAlmostEqual(imuapp._wrap_deg(370.0), 10.0)
        self.assertAlmostEqual(imuapp._wrap_deg(-10.0), 350.0)

    def test_ema(self):
        self.assertAlmostEqual(imuapp._ema(None, 10.0), 10.0)
        self.assertAlmostEqual(imuapp._ema(0.0, 10.0, alpha=0.5), 5.0)

    def test_apply_yaw_offset(self):
        old_offset = prevstate.YAW_OFFSET
        try:
            prevstate.YAW_OFFSET = 15.0
            self.assertAlmostEqual(imuapp._apply_yaw_offset(350.0), 5.0)
        finally:
            prevstate.YAW_OFFSET = old_offset

    def test_synthetic_sample_shape(self):
        sample = imuapp._synthetic_sample()
        self.assertEqual(len(sample), 12)
        self.assertIsInstance(sample[2], float)  # yaw

    def test_read_sensor_sample_fallback(self):
        imuapp._imu_instance = None
        sample = imuapp._read_sensor_sample()
        self.assertIs(sample, False)


if __name__ == "__main__":
    unittest.main()
