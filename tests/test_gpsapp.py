import time
import unittest

from Sensor_Gps import gpsapp


class TestGpsApp(unittest.TestCase):
    def setUp(self):
        gpsapp._last_fix = None
        gpsapp._last_update_ts = 0.0
        gpsapp._last_good_position = None
        gpsapp._last_good_motion = None
        gpsapp._anchor_position = None
        gpsapp._last_gps_time = None
        gpsapp._duplicate_gps_time_count = 0
        gpsapp.GPS_HEALTH = 0
        gpsapp.POS_HEALTH = 0
        gpsapp.MOTION_HEALTH = 0
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
        self.assertGreaterEqual(len(data), 12)
        self.assertIsInstance(data[2], float)

    def test_position_health_rejects_zero_and_bad_sats(self):
        now = time.time()
        health, _ = gpsapp._position_health(0.0, 126.93, 1, 8, "041559", 0.0, now)
        self.assertEqual(health, 0)
        health, _ = gpsapp._position_health(37.56, 126.93, 1, 2, "041559", 0.0, now)
        self.assertEqual(health, 0)

    def test_position_health_rejects_bad_time_and_stale(self):
        now = time.time()
        health, _ = gpsapp._position_health(37.56, 126.93, 1, 8, "04602.", 0.0, now)
        self.assertEqual(health, 0)
        health, _ = gpsapp._position_health(37.56, 126.93, 1, 8, "041559", 99.0, now)
        self.assertEqual(health, 0)

    def test_position_health_rejects_jump(self):
        now = time.time()
        health, _ = gpsapp._position_health(37.56, 126.93, 1, 8, "041559", 0.0, now)
        self.assertEqual(health, 1)
        gpsapp._hold_or_update_position(37.56, 126.93, health, now)
        health, _ = gpsapp._position_health(38.56, 127.93, 1, 8, "041600", 0.0, now + 0.2)
        self.assertEqual(health, 0)

    def test_motion_health_requires_rmc_and_valid_course(self):
        self.assertEqual(gpsapp._motion_health(8.0, 90.0, "V", False, 0.0, None), 0)
        self.assertEqual(gpsapp._motion_health(8.0, 361.0, "A", True, 0.0, None), 0)
        self.assertEqual(gpsapp._motion_health(8.0, 90.0, "A", True, 0.0, None), 1)

    def test_motion_health_rejects_speed_spike(self):
        gpsapp._hold_or_update_motion(5.0, 90.0, 1)
        self.assertEqual(gpsapp._motion_health(40.0, 90.0, "A", True, 0.0, None), 0)

    def test_ubx_position_health_uses_accuracy(self):
        now = time.time()
        health, _ = gpsapp._position_health(
            37.56,
            126.93,
            1,
            8,
            "041559",
            0.0,
            now,
            source="UBX_NAV_PVT",
            fix_type=3,
            h_acc=2.0,
        )
        self.assertEqual(health, 1)
        health, _ = gpsapp._position_health(
            37.56,
            126.93,
            1,
            8,
            "041600",
            0.0,
            now + 1.0,
            source="UBX_NAV_PVT",
            fix_type=3,
            h_acc=99.0,
        )
        self.assertEqual(health, 0)

    def test_ubx_motion_health_uses_accuracy(self):
        self.assertEqual(
            gpsapp._motion_health(
                8.0,
                90.0,
                "A",
                True,
                0.0,
                None,
                source="UBX_NAV_PVT",
                s_acc=0.5,
                head_acc=10.0,
            ),
            1,
        )
        self.assertEqual(
            gpsapp._motion_health(
                8.0,
                90.0,
                "A",
                True,
                0.0,
                None,
                source="UBX_NAV_PVT",
                s_acc=9.0,
                head_acc=10.0,
            ),
            0,
        )
        self.assertEqual(
            gpsapp._motion_health(
                8.0,
                90.0,
                "A",
                True,
                0.0,
                None,
                source="UBX_NAV_PVT",
                s_acc=0.5,
                head_acc=120.0,
            ),
            0,
        )


if __name__ == "__main__":
    unittest.main()
