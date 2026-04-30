import time
import unittest

from Sensor_Gps import gpsapp


class TestGpsApp(unittest.TestCase):
    def setUp(self):
        gpsapp._last_fix = None
        gpsapp._last_update_ts = 0.0
        gpsapp.GPS_HEALTH = 0
        gpsapp.LAT = 37.56
        gpsapp.LON = 126.93

    def test_valid_fix(self):
        self.assertTrue(gpsapp._is_valid_fix(37.56, 126.93, 1, 8, "A"))
        self.assertFalse(gpsapp._is_valid_fix(0.0, 0.0, 0, 0, "V"))

    def test_jump_detection(self):
        now = time.time()
        self.assertFalse(gpsapp._is_jump(37.56, 126.93, now))
        # huge jump in tiny dt
        self.assertTrue(gpsapp._is_jump(38.56, 127.93, now + 0.01))

    def test_synthetic_read_shape(self):
        data = gpsapp._synthetic_read()
        self.assertGreaterEqual(len(data), 9)
        self.assertIsInstance(data[2], float)


if __name__ == "__main__":
    unittest.main()
