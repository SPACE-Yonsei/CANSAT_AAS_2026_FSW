"""Origin (start point) lock now lives inside the guidance pipeline.

handle_gps / handle_flight_state only refresh _CACHE; the actual origin set
is performed by guidance.produceL1input from _CACHE snapshots during
ctrl_parafoil.  These tests verify the new contract.
"""

import time
import unittest
from unittest.mock import patch

from Sensor_Motor import motorapp, guidance


def _reset_motor_cache() -> None:
    motorapp.STATE = 0
    motorapp._ORIGIN_SAVED = False
    motorapp._CACHE = motorapp._Cache()
    motorapp._GUIDANCE_STATE = guidance.GuidanceState()
    motorapp._TARGET_LAT = None
    motorapp._TARGET_LON = None
    motorapp._START_LAT = None
    motorapp._START_LON = None


def _gps_msg(lat=37.560700, lon=126.930700, course=90.0, speed=5.0, ts=None) -> str:
    if ts is None:
        ts = time.monotonic()
    return f"{lat},{lon},{ts:.4f},{course},{speed},{ts:.4f}"


class TestStartPointLock(unittest.TestCase):
    def setUp(self) -> None:
        _reset_motor_cache()

    @patch.object(motorapp.prevstate, "update_start_point")
    def test_handle_gps_does_not_lock_origin(self, mock_update) -> None:
        """handle_gps must not lock origin or write to prevstate."""
        motorapp.STATE = 3
        motorapp.handle_gps(_gps_msg(course=999.0))
        self.assertIsNone(motorapp._CACHE.start_lat)
        self.assertFalse(motorapp._GUIDANCE_STATE.origin_ready)
        mock_update.assert_not_called()

    @patch.object(motorapp.prevstate, "update_start_point")
    def test_origin_set_via_guidance_pipeline(self, mock_update) -> None:
        """A full decidefresh → produceL1input cycle locks the origin."""
        motorapp.STATE = 3
        motorapp.handle_gps(_gps_msg())
        # Simulate one ctrl_parafoil cycle by invoking guidance directly
        snap = motorapp._cache_snapshot()
        now = time.monotonic()
        fresh = guidance.decidefresh(snap.latest_gps, snap.latest_imu,
                                      snap.latest_baro,
                                      motorapp._GUIDANCE_STATE, now)
        guidance.produceL1input(fresh, snap.latest_gps, snap.latest_imu,
                                 motorapp._GUIDANCE_STATE, motorapp.STATE, now)
        self.assertTrue(motorapp._GUIDANCE_STATE.origin_ready)
        self.assertAlmostEqual(motorapp._GUIDANCE_STATE.origin_lat, 37.560700)


if __name__ == "__main__":
    unittest.main()
