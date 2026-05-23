import unittest
import sys
import types

sys.modules.setdefault("Sensor_Gps.gps", types.SimpleNamespace())
from Sensor_Gps import gpsapp


class TestCurrentGpsAppFidelity(unittest.TestCase):
    def setUp(self):
        gpsapp._prev_valid_lat = 0.0
        gpsapp._prev_valid_lon = 0.0
        gpsapp._prev_valid_ts = 0.0

    def test_position_fidelity_rejects_near_zero_placeholder(self):
        self.assertFalse(
            gpsapp._eval_pos_fidelity(
                lat=0.00001,
                lon=-0.00002,
                hdop=1.0,
                sats=8,
                fix_quality=1,
                now=100.0,
            )
        )

    def test_position_fidelity_accepts_valid_test_area_position(self):
        self.assertTrue(
            gpsapp._eval_pos_fidelity(
                lat=37.56,
                lon=126.93,
                hdop=1.0,
                sats=8,
                fix_quality=1,
                now=100.0,
            )
        )

    def test_position_fidelity_rejects_unexpected_longitude(self):
        self.assertFalse(
            gpsapp._eval_pos_fidelity(
                lat=37.56,
                lon=50.0,
                hdop=1.0,
                sats=8,
                fix_quality=1,
                now=100.0,
            )
        )


if __name__ == "__main__":
    unittest.main()
