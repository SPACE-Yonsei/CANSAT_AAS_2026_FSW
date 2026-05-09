import unittest

from Sensor_Imu import imuapp
from lib import prevstate


class TestImuApp(unittest.TestCase):
    def test_imu_read_period_uses_imu_rate(self):
        from lib import config

        old_imu = config.IMU_RATE_HZ
        old_baro = config.BAROMETER_RATE_HZ
        try:
            config.IMU_RATE_HZ = 20
            config.BAROMETER_RATE_HZ = 1
            self.assertAlmostEqual(imuapp._imu_read_period_sec(), 0.05)
        finally:
            config.IMU_RATE_HZ = old_imu
            config.BAROMETER_RATE_HZ = old_baro

    def test_wrap_deg(self):
        self.assertAlmostEqual(imuapp._wrap_deg(370.0), 10.0)
        self.assertAlmostEqual(imuapp._wrap_deg(-10.0), 350.0)

    def test_boot_yaw_alignment_maps_average_raw_to_desired(self):
        """Boot north: offset = wrap(desired - avg_raw) => wrap(avg + offset) == desired."""
        for avg, desired in ((42.0, 0.0), (350.0, 0.0), (10.0, 5.0)):
            off = imuapp._wrap_deg(desired - avg)
            self.assertAlmostEqual(imuapp._wrap_deg(avg + off), desired)

    def test_ema(self):
        self.assertAlmostEqual(imuapp._ema(None, 10.0), 10.0)
        self.assertAlmostEqual(imuapp._ema(0.0, 10.0, alpha=0.5), 5.0)

    def test_apply_yaw_offset(self):
        old_offset = prevstate.PREV_YAW_OFFSET
        try:
            prevstate.PREV_YAW_OFFSET = 15.0
            self.assertAlmostEqual(imuapp._apply_yaw_offset(350.0), 5.0)
        finally:
            prevstate.PREV_YAW_OFFSET = old_offset

    def test_synthetic_sample_shape(self):
        sample = imuapp._synthetic_sample()
        self.assertEqual(len(sample), 12)
        self.assertEqual(sample, (0.0,) * 12)

    def test_read_sensor_sample_fallback(self):
        imuapp._imu_instance = None
        sample = imuapp._read_sensor_sample()
        self.assertIs(sample, False)


if __name__ == "__main__":
    unittest.main()
