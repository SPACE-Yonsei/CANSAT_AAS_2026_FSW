"""Start-point lock behaviour for SIM / GCS telemetry (start_lat, start_lon)."""

import unittest
from unittest.mock import patch

from Sensor_Motor import motorapp


def _reset_motor_cache() -> None:
    motorapp.STATE = 0
    motorapp._START_POINT_LOCKED = False
    motorapp._CACHE = motorapp._Cache()


class TestStartPointLock(unittest.TestCase):
    def setUp(self) -> None:
        _reset_motor_cache()

    @patch.object(motorapp.prevstate, "update_start_point")
    def test_does_not_lock_when_pos_health_false_even_with_valid_coords(self, mock_update) -> None:
        """Start-point locking follows the committed MotorApp pos_health gate."""
        motorapp.STATE = 3
        motorapp.handle_gps("37.560700,126.930700,90.0,5.0,0,0")
        self.assertFalse(motorapp._START_POINT_LOCKED)
        self.assertIsNone(motorapp._CACHE.start_lat)
        self.assertIsNone(motorapp._CACHE.start_lon)
        mock_update.assert_not_called()

    @patch.object(motorapp.prevstate, "update_start_point")
    def test_no_lock_placeholder_zero_zero(self, mock_update) -> None:
        motorapp.STATE = 3
        motorapp.handle_gps("0.0,0.0,0.0,0.0,0,0")
        self.assertFalse(motorapp._START_POINT_LOCKED)
        self.assertIsNone(motorapp._CACHE.start_lat)
        mock_update.assert_not_called()

    @patch.object(motorapp.prevstate, "update_start_point")
    def test_state_below_three_no_lock(self, mock_update) -> None:
        motorapp.STATE = 2
        motorapp.handle_gps("37.560700,126.930700,90.0,5.0,1,1")
        self.assertFalse(motorapp._START_POINT_LOCKED)
        mock_update.assert_not_called()


if __name__ == "__main__":
    unittest.main()
