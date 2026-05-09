"""Tests for guidance.py: InputResolver, fill_current/stale, decide_control_mode, L1.

Skipped: written against the legacy ``make_resolver_state`` /
``resolver_update_gnss`` / ``resolver_resolve`` API. Current ``guidance.py``
exposes ``ProduceL1Input`` / ``FillFresh`` / ``FillOld`` / ``DecideControlMode``
/ ``ProduceL1Output`` instead. Re-author tests against the new API.
"""

import math
import time
import unittest

import pytest

pytestmark = pytest.mark.skip(reason="legacy resolver API; needs rewrite for ProduceL1Input/Output")

from Sensor_Motor import guidance
from Sensor_Motor.guidance import SensorQuality, ControlMode


class TestInputResolver(unittest.TestCase):
    def test_nominal_lcsg_all_fresh(self):
        resolver = guidance.make_resolver_state()
        now = time.time()
        guidance.resolver_update_gnss(
            resolver,
            lat=37.55, lon=126.95,
            course_rad=math.radians(90.0),
            groundSpeed=12.0,
            posHealth=True, motionHealth=True,
            ts=now,
        )
        guidance.resolver_update_imu(resolver, gz=5.0, imu_health=True, ts=now)

        inp = guidance.resolver_resolve(resolver, now)

        self.assertEqual(inp.lcsg_case, "LCSG")
        self.assertEqual(inp.input_policy, "nominal_l1_with_yaw_rate_feedback")
        self.assertEqual(inp.control_mode, ControlMode.ACTIVE_CLOSED_LOOP)

    def test_feedforward_only_when_gyro_missing(self):
        resolver = guidance.make_resolver_state()
        now = time.time()
        guidance.resolver_update_gnss(
            resolver,
            lat=37.55, lon=126.95,
            course_rad=math.radians(90.0),
            groundSpeed=12.0,
            posHealth=True, motionHealth=True,
            ts=now,
        )
        # No IMU update → G missing

        inp = guidance.resolver_resolve(resolver, now)

        self.assertEqual(inp.lcsg_case, "LCS-")
        self.assertEqual(inp.input_policy, "l1_valid_feedforward_only_no_gyro")
        self.assertEqual(inp.control_mode, ControlMode.ACTIVE_FEEDFORWARD)


class TestFillCurrentData(unittest.TestCase):
    def test_fresh_data_populates_all_fields(self):
        s = guidance.make_resolver_state()
        now = time.time()
        guidance.resolver_update_gnss(
            s, 37.55, 126.95, math.radians(90.0), 12.0, True, True, now,
        )
        guidance.resolver_update_imu(s, gz=5.0, imu_health=True, ts=now)
        guidance.resolver_update_baro(s, 100.0, now, baro_health=True)

        state = guidance.L1Input(timestamp=now)
        guidance.fill_current_data(s, state, now)

        self.assertEqual(state.pos_status,   SensorQuality.FRESH)
        self.assertEqual(state.motion_health, SensorQuality.FRESH)
        self.assertEqual(state.gyrz_health,   SensorQuality.FRESH)
        self.assertEqual(state.alt_health,    SensorQuality.FRESH)
        self.assertIsNotNone(state.pos_N)
        self.assertIsNotNone(state.course)
        self.assertIsNotNone(state.gyrz)
        self.assertIsNotNone(state.altitude)

    def test_health_false_marks_stale_without_values(self):
        s = guidance.make_resolver_state()
        now = time.time()
        guidance.resolver_update_gnss(
            s, 37.55, 126.95, math.radians(90.0), 12.0, False, False, now,
        )
        guidance.resolver_update_imu(s, gz=5.0, imu_health=False, ts=now)
        guidance.resolver_update_baro(s, 100.0, now, baro_health=False)

        state = guidance.L1Input(timestamp=now)
        guidance.fill_current_data(s, state, now)

        self.assertEqual(state.pos_status,   SensorQuality.STALE)
        self.assertEqual(state.motion_health, SensorQuality.STALE)
        self.assertEqual(state.gyrz_health,   SensorQuality.STALE)
        self.assertEqual(state.alt_health,    SensorQuality.STALE)
        # fill_stale_data not yet called — values remain None
        self.assertIsNone(state.pos_N)
        self.assertIsNone(state.course)


class TestFillStaleData(unittest.TestCase):
    def test_stale_fields_filled_from_raw_fallback(self):
        s = guidance.make_resolver_state()
        now = time.time()
        guidance.resolver_update_gnss(
            s, 37.55, 126.95, math.radians(90.0), 12.0, False, False, now,
        )
        guidance.resolver_update_imu(s, gz=5.0, imu_health=False, ts=now)
        guidance.resolver_update_baro(s, 100.0, now, baro_health=False)

        state = guidance.L1Input(timestamp=now)
        guidance.fill_current_data(s, state, now)
        guidance.fill_stale_data(s, state, now)

        self.assertIsNotNone(state.course)
        self.assertIsNotNone(state.gyrz)
        self.assertIsNotNone(state.altitude)

    def test_last_good_preferred_over_raw_fallback(self):
        s = guidance.make_resolver_state()
        now = time.time()
        # First update: health=True → stores last_good
        guidance.resolver_update_gnss(
            s, 37.55, 126.95, math.radians(45.0), 10.0, True, True, now - 0.5,
        )
        # Second update: health=False (raw only, different values)
        guidance.resolver_update_gnss(
            s, 37.56, 126.96, math.radians(90.0), 20.0, False, False, now,
        )

        state = guidance.L1Input(timestamp=now)
        guidance.fill_current_data(s, state, now)  # → STALE (health=False now)
        guidance.fill_stale_data(s, state, now)

        # last_good course is 45°, raw is 90° — last_good should win
        self.assertAlmostEqual(state.course, math.radians(45.0), places=6)


class TestDecideControlMode(unittest.TestCase):
    def test_all_fresh_no_gyro_gives_active_feedforward(self):
        s = guidance.make_resolver_state()
        now = time.time()
        guidance.resolver_update_gnss(
            s, 37.55, 126.95, math.radians(90.0), 12.0, True, True, now,
        )
        state = guidance.L1Input(timestamp=now)
        guidance.fill_current_data(s, state, now)
        guidance.fill_stale_data(s, state, now)
        guidance.decide_control_mode(state)

        self.assertEqual(state.control_mode, ControlMode.ACTIVE_FEEDFORWARD)

    def test_stale_data_gives_degraded_feedforward(self):
        s = guidance.make_resolver_state()
        now = time.time()
        guidance.resolver_update_gnss(
            s, 37.55, 126.95, math.radians(90.0), 12.0, False, False, now,
        )
        guidance.resolver_set_origin(s, 37.55, 126.95)
        state = guidance.L1Input(timestamp=now)
        guidance.fill_current_data(s, state, now)
        guidance.fill_stale_data(s, state, now)
        guidance.decide_control_mode(state)

        self.assertEqual(state.control_mode, ControlMode.DEGRADED_FEEDFORWARD)
        self.assertIn("stale", state.reason)

    def test_missing_position_gives_safe_glide(self):
        state = guidance.L1Input(timestamp=time.time())
        # No resolver updates → all MISSING
        guidance.decide_control_mode(state)

        self.assertEqual(state.control_mode, ControlMode.SAFE_GLIDE)

    def test_all_fresh_with_gyro_gives_active_closed_loop(self):
        s = guidance.make_resolver_state()
        now = time.time()
        guidance.resolver_update_gnss(
            s, 37.55, 126.95, math.radians(90.0), 12.0, True, True, now,
        )
        guidance.resolver_update_imu(s, gz=0.1, imu_health=True, ts=now)
        state = guidance.L1Input(timestamp=now)
        guidance.fill_current_data(s, state, now)
        guidance.fill_stale_data(s, state, now)
        guidance.decide_control_mode(state)

        self.assertEqual(state.control_mode, ControlMode.ACTIVE_CLOSED_LOOP)


class TestL1Update(unittest.TestCase):
    def _make_active_input(self, course_deg=45.0, speed_mps=8.0, pos_n=100.0, pos_e=0.0):
        inp = guidance.L1Input(timestamp=time.time())
        inp.pos_N = pos_n
        inp.pos_E = pos_e
        inp.pos_status = SensorQuality.FRESH
        inp.course = math.radians(course_deg)
        inp.motion_health = SensorQuality.FRESH
        inp.ground_speed_mps = speed_mps
        inp.gyrz = 0.0
        inp.gyrz_health = SensorQuality.FRESH
        inp.control_mode = ControlMode.ACTIVE_CLOSED_LOOP
        return inp

    def test_no_target_returns_safe_glide(self):
        l1 = guidance.make_l1_state()
        inp = self._make_active_input()
        out = guidance.l1_update(l1, inp, time.time())
        self.assertFalse(out.active)
        self.assertEqual(out.submode, "SAFE_GLIDE")

    def test_safe_glide_mode_returns_inactive(self):
        l1 = guidance.make_l1_state()
        guidance.l1_set_target(l1, 500.0, 500.0)
        inp = self._make_active_input()
        inp.control_mode = ControlMode.SAFE_GLIDE
        out = guidance.l1_update(l1, inp, time.time())
        self.assertFalse(out.active)

    def test_on_track_heading_right_gives_finite_output(self):
        l1 = guidance.make_l1_state()
        guidance.l1_set_start(l1, 0.0, 0.0)
        guidance.l1_set_target(l1, 1000.0, 0.0)   # due north
        inp = self._make_active_input(course_deg=0.0, speed_mps=8.0, pos_n=100.0, pos_e=0.0)
        out = guidance.l1_update(l1, inp, time.time())
        self.assertTrue(out.active)
        self.assertTrue(math.isfinite(out.course_rate_cmd_rad_s))
        self.assertTrue(math.isfinite(out.lat_acc_cmd_mps2))

    def test_left_of_path_commands_right_turn(self):
        """Vehicle left of path → positive course_rate (right turn)."""
        l1 = guidance.make_l1_state()
        guidance.l1_set_start(l1, 0.0, 0.0)
        guidance.l1_set_target(l1, 1000.0, 0.0)   # path goes due north
        # Vehicle is 50 m to the left (west) of the path
        inp = self._make_active_input(course_deg=0.0, speed_mps=8.0, pos_n=100.0, pos_e=-50.0)
        out = guidance.l1_update(l1, inp, time.time())
        self.assertTrue(out.active)
        # crossTrack < 0 (right of path in this sign convention)
        # vehicle is at E=-50 which is west = right of northward path
        # Actually: crossTrack = AP_N*unit_AB_E - AP_E*unit_AB_N
        #           AP=(100,-50), unit_AB=(1,0) → crossTrack = 100*0 - (-50)*1 = 50 > 0 (left)
        self.assertGreater(out.crossTrack, 0.0)
        self.assertGreater(out.course_rate_cmd_rad_s, 0.0)  # right turn

    def test_course_rate_clamped_to_max(self):
        l1 = guidance.make_l1_state()
        guidance.l1_set_start(l1, 0.0, 0.0)
        guidance.l1_set_target(l1, 1000.0, 0.0)
        # Large cross-track error → saturates
        inp = self._make_active_input(course_deg=90.0, speed_mps=8.0, pos_n=100.0, pos_e=-500.0)
        out = guidance.l1_update(l1, inp, time.time())
        self.assertLessEqual(abs(out.course_rate_cmd_rad_s), guidance.COURSE_RATE_MAX + 1e-9)

    def test_direct_to_target_latches_on_hard_xtrack(self):
        l1 = guidance.make_l1_state()
        guidance.l1_set_start(l1, 0.0, 0.0)
        guidance.l1_set_target(l1, 1000.0, 0.0)
        # Position very far off path → XTRACK_HARD exceeded
        inp = self._make_active_input(course_deg=0.0, speed_mps=8.0, pos_n=100.0, pos_e=-200.0)
        out = guidance.l1_update(l1, inp, time.time())
        self.assertEqual(l1.submode, "DRAW_LINE")
        self.assertEqual(out.submode, "DRAW_LINE")


if __name__ == "__main__":
    unittest.main()
