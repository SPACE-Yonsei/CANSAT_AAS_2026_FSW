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
        resolver = motor_guidance.make_resolver_state()
        now = time.time()
        motor_guidance.resolver_update_gnss(
            resolver,
            lat=37.55,
            lon=126.95,
            course_rad=math.radians(90.0),
            groundSpeed=12.0,
            posHealth=True,
            motionHealth=True,
            ts=now,
        )
        # imu_health=True required so GYRZ is classified FRESH → closed-loop
        motor_guidance.resolver_update_imu(resolver, gz=5.0, imu_health=True, ts=now)

        guidance_input = motor_guidance.resolver_resolve(resolver, now)

        self.assertEqual(guidance_input.lcsg_case, "LCSG")
        self.assertEqual(guidance_input.input_policy, "nominal_l1_with_yaw_rate_feedback")
        self.assertEqual(guidance_input.control_mode, motor_guidance.ControlMode.ACTIVE_CLOSED_LOOP)

    def test_input_resolver_reports_feedforward_only_when_gyro_missing(self):
        resolver = motor_guidance.make_resolver_state()
        now = time.time()
        motor_guidance.resolver_update_gnss(
            resolver,
            lat=37.55,
            lon=126.95,
            course_rad=math.radians(90.0),
            groundSpeed=12.0,
            posHealth=True,
            motionHealth=True,
            ts=now,
        )

        guidance_input = motor_guidance.resolver_resolve(resolver, now)

        self.assertEqual(guidance_input.lcsg_case, "LCS-")
        self.assertEqual(guidance_input.input_policy, "l1_valid_feedforward_only_no_gyro")
        self.assertEqual(guidance_input.control_mode, motor_guidance.ControlMode.ACTIVE_FEEDFORWARD)

    def test_fill_current_data_fresh(self):
        """pos_health=True, fresh data → FRESH status + values populated."""
        s = motor_guidance.make_resolver_state()
        now = time.time()
        motor_guidance.resolver_update_gnss(
            s, 37.55, 126.95, math.radians(90.0), 12.0, True, True, now,
        )
        motor_guidance.resolver_update_imu(s, gz=5.0, imu_health=True, ts=now)
        motor_guidance.resolver_update_baro(s, 100.0, now, baro_health=True)

        state = motor_guidance.GuidanceInput(timestamp=now)
        motor_guidance.fill_current_data(s, state, now)

        self.assertEqual(state.pos_status, motor_guidance.FieldStatus.FRESH)
        self.assertEqual(state.motion_health, motor_guidance.FieldStatus.FRESH)
        self.assertEqual(state.gyrz_health, motor_guidance.FieldStatus.FRESH)
        self.assertEqual(state.alt_health, motor_guidance.FieldStatus.FRESH)
        self.assertIsNotNone(state.pos_N)
        self.assertIsNotNone(state.course)
        self.assertIsNotNone(state.gyrz)
        self.assertIsNotNone(state.altitude)

    def test_fill_current_data_health_false_gives_stale(self):
        """health=False with fresh timestamps → STALE, values not yet filled."""
        s = motor_guidance.make_resolver_state()
        now = time.time()
        motor_guidance.resolver_update_gnss(
            s, 37.55, 126.95, math.radians(90.0), 12.0, False, False, now,
        )
        motor_guidance.resolver_update_imu(s, gz=5.0, imu_health=False, ts=now)
        motor_guidance.resolver_update_baro(s, 100.0, now, baro_health=False)

        state = motor_guidance.GuidanceInput(timestamp=now)
        motor_guidance.fill_current_data(s, state, now)

        self.assertEqual(state.pos_status, motor_guidance.FieldStatus.STALE)
        self.assertEqual(state.motion_health, motor_guidance.FieldStatus.STALE)
        self.assertEqual(state.gyrz_health, motor_guidance.FieldStatus.STALE)
        self.assertEqual(state.alt_health, motor_guidance.FieldStatus.STALE)
        # Values not yet written — fill_stale_data has not run
        self.assertIsNone(state.pos_N)
        self.assertIsNone(state.course)

    def test_fill_stale_data_fills_stale_fields(self):
        """fill_stale_data populates values for STALE fields."""
        s = motor_guidance.make_resolver_state()
        now = time.time()
        motor_guidance.resolver_update_gnss(
            s, 37.55, 126.95, math.radians(90.0), 12.0, False, False, now,
        )
        motor_guidance.resolver_update_imu(s, gz=5.0, imu_health=False, ts=now)
        motor_guidance.resolver_update_baro(s, 100.0, now, baro_health=False)

        state = motor_guidance.GuidanceInput(timestamp=now)
        motor_guidance.fill_current_data(s, state, now)
        motor_guidance.fill_stale_data(s, state, now)

        self.assertIsNotNone(state.course)
        self.assertIsNotNone(state.gyrz)
        self.assertIsNotNone(state.altitude)

    def test_decide_control_mode_active(self):
        """FRESH data → ACTIVE guidance mode."""
        s = motor_guidance.make_resolver_state()
        now = time.time()
        motor_guidance.resolver_update_gnss(
            s, 37.55, 126.95, math.radians(90.0), 12.0, True, True, now,
        )
        state = motor_guidance.GuidanceInput(timestamp=now)
        motor_guidance.fill_current_data(s, state, now)
        motor_guidance.fill_stale_data(s, state, now)
        motor_guidance.decide_control_mode(state)

        self.assertEqual(state.control_mode, motor_guidance.ControlMode.ACTIVE_FEEDFORWARD)

    def test_decide_control_mode_degraded_stale(self):
        """STALE data → DEGRADED guidance mode."""
        s = motor_guidance.make_resolver_state()
        now = time.time()
        motor_guidance.resolver_update_gnss(
            s, 37.55, 126.95, math.radians(90.0), 12.0, False, False, now,
        )
        motor_guidance.resolver_set_origin(s, 37.55, 126.95)
        state = motor_guidance.GuidanceInput(timestamp=now)
        motor_guidance.fill_current_data(s, state, now)
        motor_guidance.fill_stale_data(s, state, now)
        motor_guidance.decide_control_mode(state)

        self.assertEqual(state.control_mode, motor_guidance.ControlMode.DEGRADED_FEEDFORWARD)
        self.assertIn("stale", state.reason)


if __name__ == "__main__":
    unittest.main()
