"""motorapp-owned origin/target lock policy."""

import math
import unittest
from unittest.mock import patch

from Sensor_Motor import guidance, motorapp
from Sensor_Motor.sensor_types import _Cache


NOW = 1000.0


def _reset() -> None:
    guidance.reset()
    guidance._MISSION_t.target_lat = math.nan
    guidance._MISSION_t.target_lon = math.nan
    motorapp.STATE = 0
    motorapp._PREV_STATE = 0
    motorapp._CACHE_t = _Cache()
    motorapp._ORIGIN_LOCKED = False
    motorapp._TARGET_LOCKED = False


def _gps_msg(lat=37.560700, lon=126.930700, pos=1, ts=NOW) -> str:
    return f"{lat},{lon},{pos},{ts:.4f},90.0,5.0,1,{ts:.4f}"


class TestStartPointLock(unittest.TestCase):
    def setUp(self) -> None:
        _reset()

    @patch.object(motorapp.prevstate, "update_start_point")
    def test_handle_gps_locks_first_valid_origin_at_state3(self, mock_update) -> None:
        motorapp.STATE = 3
        motorapp.handle_gps(_gps_msg())
        motorapp.handle_gps(_gps_msg(lat=38.0, lon=127.5))

        mi = guidance._MISSION_t
        self.assertTrue(motorapp._ORIGIN_LOCKED)
        self.assertAlmostEqual(mi.origin_lat, 37.560700)
        self.assertAlmostEqual(mi.origin_lon, 126.930700)
        self.assertEqual(mock_update.call_count, 1)

    @patch.object(motorapp.prevstate, "update_start_point")
    def test_handle_gps_does_not_lock_before_state3(self, mock_update) -> None:
        motorapp.STATE = 2
        motorapp.handle_gps(_gps_msg())

        self.assertFalse(motorapp._ORIGIN_LOCKED)
        self.assertFalse(math.isfinite(guidance._MISSION_t.origin_lat))
        mock_update.assert_not_called()

    @patch.object(motorapp.prevstate, "FIX_TARGET_GPS", False)
    def test_target_coord_locks_first_valid_target(self) -> None:
        motorapp.handle_target_coord("37.500000,127.000000")
        motorapp.handle_target_coord("38.000000,128.000000")

        mi = guidance._MISSION_t
        self.assertTrue(motorapp._TARGET_LOCKED)
        self.assertAlmostEqual(mi.target_lat, 37.5)
        self.assertAlmostEqual(mi.target_lon, 127.0)


if __name__ == "__main__":
    unittest.main()
