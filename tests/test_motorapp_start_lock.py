"""Start-point lock behaviour for SIM / GCS telemetry (start_lat, start_lon)."""

import unittest
from unittest.mock import patch

from Sensor_Motor import motorapp


def _reset_motor_cache() -> None:
    motorapp.STATE = 0
    motorapp._START_POINT_LOCKED = False
    motorapp._CACHE = motorapp._Cache()


def _gps_msg(lat=37.560700, lon=126.930700, course=90.0, speed=5.0, ts=100.0) -> str:
    return f"{lat},{lon},{ts:.4f},{course},{speed},{ts:.4f}"


class TestStartPointLock(unittest.TestCase):
    def setUp(self) -> None:
        _reset_motor_cache()

    @patch.object(motorapp.prevstate, "update_start_point")
    def test_does_not_lock_when_position_rejected_even_with_valid_motion(self, mock_update) -> None:
        """Start-point locking follows MotorApp position sanity gates."""
        motorapp.STATE = 3
        motorapp.handle_gps(_gps_msg(lat=0.0, lon=0.0))
        self.assertFalse(motorapp._START_POINT_LOCKED)
        self.assertIsNone(motorapp._CACHE.start_lat)
        self.assertIsNone(motorapp._CACHE.start_lon)
        mock_update.assert_not_called()

    @patch.object(motorapp.prevstate, "update_start_point")
    def test_no_lock_invalid_motion_because_position_still_required(self, mock_update) -> None:
        motorapp.STATE = 3
        motorapp.handle_gps(_gps_msg(course=999.0))
        self.assertTrue(motorapp._START_POINT_LOCKED)
        self.assertAlmostEqual(motorapp._CACHE.start_lat, 37.560700)
        mock_update.assert_called_once()

    @patch.object(motorapp.prevstate, "update_start_point")
    def test_no_lock_placeholder_zero_zero(self, mock_update) -> None:
        motorapp.STATE = 3
        motorapp.handle_gps(_gps_msg(lat=0.0, lon=0.0))
        self.assertFalse(motorapp._START_POINT_LOCKED)
        self.assertIsNone(motorapp._CACHE.start_lat)
        mock_update.assert_not_called()

    @patch.object(motorapp.prevstate, "update_start_point")
    def test_state_below_three_no_lock(self, mock_update) -> None:
        motorapp.STATE = 2
        motorapp.handle_gps(_gps_msg())
        self.assertFalse(motorapp._START_POINT_LOCKED)
        mock_update.assert_not_called()


if __name__ == "__main__":
    unittest.main()
