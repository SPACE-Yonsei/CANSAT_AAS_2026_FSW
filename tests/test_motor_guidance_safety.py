import time
import unittest
from types import SimpleNamespace

from Sensor_Motor import motor_guidance


class TestMotorGuidanceSafety(unittest.TestCase):
    def setUp(self):
        motor_guidance.init_guidance()
        self.imu = SimpleNamespace(yaw=10.0, gyrz=0.1)
        self.gps = SimpleNamespace(lat=37.55, lon=126.95, direction=90.0, velocity=8.0)
        self.fid = SimpleNamespace(pos_health=1, motion_health=1)

        # Bypass initial GPS jump warm-up for deterministic unit tests.
        motor_guidance._prev_gps.initialized = True
        motor_guidance._prev_gps.lat = self.gps.lat
        motor_guidance._prev_gps.lon = self.gps.lon
        motor_guidance._prev_gps.time = time.time() - 1.0
        motor_guidance._gps_stable_count = motor_guidance.GPS_STABLE_COUNT_REQUIRED

    def test_target_required(self):
        motor_guidance.set_start_coordinates(self.gps.lat, self.gps.lon)
        out = motor_guidance.guidance(
            self.imu,
            self.gps,
            self.fid,
            None,
            120.0,
        )
        self.assertEqual(out.state, "TARGET_UNSET")
        self.assertEqual(out.commanded_yaw_rate, 0.0)

    def test_start_point_checked_before_distance_logic(self):
        target = SimpleNamespace(lat=37.56, lon=126.96)
        out = motor_guidance.guidance(
            self.imu,
            self.gps,
            self.fid,
            target,
            120.0,
        )
        self.assertEqual(out.state, "START_UNSET")
        self.assertEqual(out.commanded_yaw_rate, 0.0)


if __name__ == "__main__":
    unittest.main()
