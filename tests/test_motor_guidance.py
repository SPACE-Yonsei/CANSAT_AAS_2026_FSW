import math
import time
import unittest
from types import SimpleNamespace

from Sensor_Motor import motor_guidance


class TestMotorGuidance(unittest.TestCase):
    def setUp(self):
        motor_guidance.init_guidance()

    @staticmethod
    def _good_inputs():
        imu = SimpleNamespace(yaw=10.0, gyrz=0.5)
        gps = SimpleNamespace(lat=37.55, lon=126.95, direction=90.0, velocity=12.0)
        fid = SimpleNamespace(pos_health=1, motion_health=1)
        tgt = SimpleNamespace(lat=37.56, lon=126.96)
        return imu, gps, fid, tgt

    def _prime_gps_jump_gate(self, gps):
        motor_guidance._PREV_GPS.initialized = True
        motor_guidance._PREV_GPS.lat = gps.lat
        motor_guidance._PREV_GPS.lon = gps.lon
        motor_guidance._PREV_GPS.time = time.time() - 1.0
        motor_guidance._GPS_STABLE_COUNT = motor_guidance.GPS_STABLE_COUNT_REQUIRED

    def test_invalid_gps_returns_gps_invalid(self):
        imu, gps, _, tgt = self._good_inputs()
        bad_fid = SimpleNamespace(pos_health=0, motion_health=0)
        out = motor_guidance.guidance(imu, gps, bad_fid, tgt, 100.0)
        self.assertEqual(out.state, "GPS_INVALID")

    def test_gps_jump_initially_invalid_then_valid(self):
        imu, gps, fid, tgt = self._good_inputs()
        motor_guidance.set_start_coordinates(gps.lat, gps.lon)
        out1 = motor_guidance.guidance(imu, gps, fid, tgt, 100.0)
        self.assertEqual(out1.state, "GPS_INVALID")
        out2 = motor_guidance.guidance(imu, gps, fid, tgt, 100.0)
        self.assertEqual(out2.state, "GPS_INVALID")

        motor_guidance._PREV_GPS.time = time.time() - 1.0
        gps2 = SimpleNamespace(lat=37.55005, lon=126.95005, direction=90.0, velocity=12.0)
        out3 = motor_guidance.guidance(imu, gps2, fid, tgt, 100.0)
        self.assertIn(out3.state, {"STRAIGHT", "TURNING", "PATTERN", "TARGET_REACHED"})

    def test_landing_command_limited(self):
        imu, gps, fid, tgt = self._good_inputs()
        motor_guidance.set_start_coordinates(gps.lat, gps.lon)
        self._prime_gps_jump_gate(gps)
        out = motor_guidance.guidance(imu, gps, fid, tgt, 5.0)
        self.assertLessEqual(abs(out.commanded_yaw_rate), motor_guidance.LANDING_YR_MAX + 1e-6)

    def test_guidance_output_is_finite(self):
        imu, gps, fid, tgt = self._good_inputs()
        motor_guidance.set_start_coordinates(gps.lat, gps.lon)
        self._prime_gps_jump_gate(gps)
        out = motor_guidance.guidance(imu, gps, fid, tgt, 120.0)
        self.assertTrue(math.isfinite(out.distance))
        self.assertTrue(math.isfinite(out.commanded_yaw_rate))

    def test_target_none_returns_target_unset(self):
        imu, gps, fid, _ = self._good_inputs()
        motor_guidance.set_start_coordinates(gps.lat, gps.lon)
        self._prime_gps_jump_gate(gps)
        out = motor_guidance.guidance(imu, gps, fid, None, 120.0)
        self.assertEqual(out.state, "TARGET_UNSET")

    def test_input_resolver_reports_nominal_lcsg_case(self):
        resolver = motor_guidance.GuidanceInputResolver()
        now = time.time()
        resolver.update_gnss(
            lat=37.55,
            lon=126.95,
            course_rad=math.radians(90.0),
            groundSpeed=12.0,
            posHealth=True,
            motionHealth=True,
            ts=now,
        )
        resolver.update_imu(gz=5.0, ts=now)

        guidance_input = resolver.resolve(now)

        self.assertEqual(guidance_input.lcsg_case, "LCSG")
        self.assertEqual(guidance_input.input_policy, "nominal_l1_with_yaw_rate_feedback")
        self.assertEqual(guidance_input.guidance_mode, motor_guidance.GuidanceMode.ACTIVE)

    def test_input_resolver_reports_feedforward_only_when_gyro_missing(self):
        resolver = motor_guidance.GuidanceInputResolver()
        now = time.time()
        resolver.update_gnss(
            lat=37.55,
            lon=126.95,
            course_rad=math.radians(90.0),
            groundSpeed=12.0,
            posHealth=True,
            motionHealth=True,
            ts=now,
        )

        guidance_input = resolver.resolve(now)

        self.assertEqual(guidance_input.lcsg_case, "LCS-")
        self.assertEqual(guidance_input.input_policy, "l1_valid_feedforward_only_no_gyro")
        self.assertEqual(guidance_input.guidance_mode, motor_guidance.GuidanceMode.ACTIVE)


if __name__ == "__main__":
    unittest.main()
