"""Tests for guidance.py: FillFresh, DecideControlMode, ProduceL1Input/Output."""

import math
import time
import unittest
from types import SimpleNamespace

from Sensor_Motor import control, guidance
from Sensor_Motor.guidance import SensorQuality, ControlMode

ORIGIN_LAT = 37.55
ORIGIN_LON = 126.95


def _gps(lat, lon, course_rad=0.0, speed=8.0, age=0.0):
    ts = time.monotonic() - age
    return SimpleNamespace(
        lat=lat, lon=lon,
        course_rad=course_rad, speed_mps=speed,
        pos_ts=ts, motion_ts=ts,
    )


def _imu(gyrz_rad_s=0.1, age=0.0):
    ts = time.monotonic() - age
    return SimpleNamespace(gyrz_rad_s=gyrz_rad_s, ts=ts)


def _baro(alt_m=100.0, age=0.0):
    ts = time.monotonic() - age
    return SimpleNamespace(alt_m=alt_m, ts=ts)


def _fresh_l1_input():
    inp = guidance.L1Input()
    inp.origin_lat = ORIGIN_LAT
    inp.origin_lon = ORIGIN_LON
    return inp


class TestFillFresh(unittest.TestCase):
    def test_fresh_gps_fills_position(self):
        inp = _fresh_l1_input()
        gps = _gps(ORIGIN_LAT + 0.01, ORIGIN_LON, age=0.0)
        now = time.monotonic()
        guidance.FillFresh(inp, gps, None, None, ORIGIN_LAT, ORIGIN_LON, now)
        self.assertEqual(inp.pos_quality, SensorQuality.FRESH)
        self.assertIsNotNone(inp.pos_N)
        self.assertIsNotNone(inp.pos_E)
        self.assertGreater(inp.pos_N, 0.0)

    def test_fresh_gps_fills_motion(self):
        inp = _fresh_l1_input()
        gps = _gps(ORIGIN_LAT, ORIGIN_LON, course_rad=math.radians(90.0), speed=10.0)
        now = time.monotonic()
        guidance.FillFresh(inp, gps, None, None, ORIGIN_LAT, ORIGIN_LON, now)
        self.assertEqual(inp.motion_quality, SensorQuality.FRESH)
        self.assertAlmostEqual(inp.course, math.radians(90.0))
        self.assertAlmostEqual(inp.ground_speed_mps, 10.0)

    def test_stale_gps_skips_position(self):
        inp = _fresh_l1_input()
        gps = _gps(ORIGIN_LAT + 0.01, ORIGIN_LON, age=guidance.POS_FRESH_AGE + 0.5)
        now = time.monotonic()
        guidance.FillFresh(inp, gps, None, None, ORIGIN_LAT, ORIGIN_LON, now)
        self.assertNotEqual(inp.pos_quality, SensorQuality.FRESH)
        self.assertIsNone(inp.pos_N)

    def test_fresh_imu_fills_gyrz(self):
        inp = _fresh_l1_input()
        imu = _imu(gyrz_rad_s=0.3, age=0.0)
        now = time.monotonic()
        guidance.FillFresh(inp, None, imu, None, ORIGIN_LAT, ORIGIN_LON, now)
        self.assertEqual(inp.gyrz_quality, SensorQuality.FRESH)
        self.assertAlmostEqual(inp.gyrz, 0.3)

    def test_future_imu_timestamp_skips_gyrz(self):
        inp = _fresh_l1_input()
        now = time.monotonic()
        imu = SimpleNamespace(gyrz_rad_s=0.3, ts=now + 0.01)
        guidance.FillFresh(inp, None, imu, None, ORIGIN_LAT, ORIGIN_LON, now)
        self.assertNotEqual(inp.gyrz_quality, SensorQuality.FRESH)

    def test_fresh_baro_fills_alt(self):
        inp = _fresh_l1_input()
        baro = _baro(alt_m=250.0, age=0.0)
        now = time.monotonic()
        guidance.FillFresh(inp, None, None, baro, ORIGIN_LAT, ORIGIN_LON, now)
        self.assertEqual(inp.alt_quality, SensorQuality.FRESH)
        self.assertAlmostEqual(inp.alt, 250.0)

    def test_no_origin_skips_position(self):
        inp = guidance.L1Input()
        gps = _gps(ORIGIN_LAT + 0.01, ORIGIN_LON)
        now = time.monotonic()
        guidance.FillFresh(inp, gps, None, None, None, None, now)
        self.assertNotEqual(inp.pos_quality, SensorQuality.FRESH)


class TestDecideControlMode(unittest.TestCase):
    def _inp(self, pos_q, motion_q, gyrz_q=SensorQuality.STALE, gyrz_val=None):
        inp = guidance.L1Input()
        inp.pos_quality = pos_q
        inp.motion_quality = motion_q
        inp.gyrz_quality = gyrz_q
        inp.gyrz = gyrz_val if gyrz_val is not None else (0.1 if gyrz_q == SensorQuality.FRESH else None)
        return inp

    def test_fresh_pos_motion_no_gyrz_gives_active_feedforward(self):
        inp = self._inp(SensorQuality.FRESH, SensorQuality.FRESH)
        mode, _ = guidance.DecideControlMode(inp)
        self.assertEqual(mode, ControlMode.NOMINAL_FEEDFORWARD)

    def test_fresh_pos_motion_with_fresh_gyrz_gives_active_closed_loop(self):
        inp = self._inp(SensorQuality.FRESH, SensorQuality.FRESH, SensorQuality.FRESH)
        mode, _ = guidance.DecideControlMode(inp)
        self.assertEqual(mode, ControlMode.NOMINAL_CLOSED_LOOP)

    def test_stale_position_gives_fail(self):
        inp = self._inp(SensorQuality.STALE, SensorQuality.FRESH)
        mode, _ = guidance.DecideControlMode(inp)
        self.assertEqual(mode, ControlMode.FAIL)

    def test_stale_motion_gives_fail(self):
        inp = self._inp(SensorQuality.FRESH, SensorQuality.STALE)
        mode, _ = guidance.DecideControlMode(inp)
        self.assertEqual(mode, ControlMode.FAIL)

    def test_tumble_without_dominant_axis_reports_unstable_body(self):
        inp = self._inp(SensorQuality.FRESH, SensorQuality.FRESH, SensorQuality.FRESH)
        inp.tumble = 1
        inp.gyrx = math.radians(10.0)
        inp.gyry = math.radians(20.0)
        inp.gyrz = math.radians(5.0)
        mode, reason = guidance.DecideControlMode(inp)
        self.assertEqual(mode, ControlMode.FAIL)
        self.assertEqual(reason, guidance.FailReason.UNSTABLE_BODY)


class TestProduceL1Output(unittest.TestCase):
    TARGET_LAT = ORIGIN_LAT + 900.0 / 111_000.0  # ~900 m north
    TARGET_LON = ORIGIN_LON

    def _active_input(self, pos_n=100.0, pos_e=0.0, course_deg=0.0, speed=8.0):
        inp = guidance.L1Input()
        inp.pos_N = pos_n
        inp.pos_E = pos_e
        inp.pos_quality = SensorQuality.FRESH
        inp.course = math.radians(course_deg)
        inp.motion_quality = SensorQuality.FRESH
        inp.ground_speed_mps = speed
        inp.gyrz = 0.0
        inp.gyrz_quality = SensorQuality.FRESH
        inp.control_mode = ControlMode.NOMINAL_CLOSED_LOOP
        inp.origin_lat = ORIGIN_LAT
        inp.origin_lon = ORIGIN_LON
        return inp

    def _run(self, inp, mode=ControlMode.NOMINAL_CLOSED_LOOP,
             target_lat=None, target_lon=None):
        tgt_lat = target_lat if target_lat is not None else self.TARGET_LAT
        tgt_lon = target_lon if target_lon is not None else self.TARGET_LON
        return guidance.ProduceL1Output(
            inp, mode, ORIGIN_LAT, ORIGIN_LON, tgt_lat, tgt_lon, time.monotonic()
        )

    def test_fail_mode_returns_inactive(self):
        out = self._run(self._active_input(), mode=ControlMode.FAIL)
        self.assertFalse(out.nominal)
        self.assertEqual(out.reason, "FAIL")

    def test_no_target_returns_inactive(self):
        inp = self._active_input()
        out = guidance.ProduceL1Output(
            inp, ControlMode.NOMINAL_CLOSED_LOOP,
            ORIGIN_LAT, ORIGIN_LON, None, None, time.monotonic()
        )
        self.assertFalse(out.nominal)
        self.assertEqual(out.reason, "NO_POSITION")

    def test_no_origin_returns_inactive(self):
        inp = self._active_input()
        out = guidance.ProduceL1Output(
            inp, ControlMode.NOMINAL_CLOSED_LOOP,
            None, None, self.TARGET_LAT, self.TARGET_LON, time.monotonic()
        )
        self.assertFalse(out.nominal)
        self.assertEqual(out.reason, "NO_POSITION")

    def test_current_position_at_target_returns_inactive(self):
        inp = self._active_input(pos_n=0.0, pos_e=0.0)
        out = guidance.ProduceL1Output(
            inp, ControlMode.NOMINAL_CLOSED_LOOP,
            ORIGIN_LAT, ORIGIN_LON, ORIGIN_LAT, ORIGIN_LON, time.monotonic()
        )
        self.assertFalse(out.nominal)
        self.assertEqual(out.reason, "NO_POSITION")

    def test_on_track_returns_active_finite_output(self):
        inp = self._active_input(pos_n=100.0, pos_e=0.0, course_deg=0.0, speed=8.0)
        out = self._run(inp)
        self.assertTrue(out.nominal)
        self.assertTrue(math.isfinite(out.angular_velocity_cmd_rad_s))
        self.assertTrue(math.isfinite(out.lat_acc_cmd_mps2))

    def test_west_of_target_line_commands_right_turn(self):
        inp = self._active_input(pos_n=100.0, pos_e=-50.0, course_deg=0.0, speed=8.0)
        out = self._run(inp)
        self.assertTrue(out.nominal)
        self.assertAlmostEqual(out.crossTrack, 0.0)
        self.assertGreater(out.angular_velocity_cmd_rad_s, 0.0)

    def test_east_of_target_line_commands_left_turn(self):
        inp = self._active_input(pos_n=100.0, pos_e=50.0, course_deg=0.0, speed=8.0)
        out = self._run(inp)
        self.assertTrue(out.nominal)
        self.assertAlmostEqual(out.crossTrack, 0.0)
        self.assertLess(out.angular_velocity_cmd_rad_s, 0.0)

    def test_west_of_target_guidance_drives_right_brake_down(self):
        inp = self._active_input(pos_n=100.0, pos_e=-150.0, course_deg=0.0, speed=8.0)
        g_out = self._run(inp)
        cmd = control.ProduceCtrlOutput(
            control.MakeCtrler(),
            control.ProduceCtrlInput(g_out, g_out.timestamp),
            float("nan"),
            g_out.timestamp,
        )
        self.assertGreater(g_out.angular_velocity_cmd_rad_s, 0.0)
        self.assertGreater(cmd.delta_arm_deg, 0.0)
        self.assertLess(cmd.left_angle_deg, control.NEUTRAL_ARM_DEG)
        self.assertGreater(cmd.right_angle_deg, control.NEUTRAL_ARM_DEG)

    def test_east_of_target_guidance_drives_left_brake_down(self):
        inp = self._active_input(pos_n=100.0, pos_e=150.0, course_deg=0.0, speed=8.0)
        g_out = self._run(inp)
        cmd = control.ProduceCtrlOutput(
            control.MakeCtrler(),
            control.ProduceCtrlInput(g_out, g_out.timestamp),
            float("nan"),
            g_out.timestamp,
        )
        self.assertLess(g_out.angular_velocity_cmd_rad_s, 0.0)
        self.assertLess(cmd.delta_arm_deg, 0.0)
        self.assertGreater(cmd.left_angle_deg, control.NEUTRAL_ARM_DEG)
        self.assertLess(cmd.right_angle_deg, control.NEUTRAL_ARM_DEG)

    def test_nu_uses_fixed_target_bearing_error(self):
        inp = self._active_input(pos_n=100.0, pos_e=50.0, course_deg=0.0, speed=8.0)
        out = self._run(inp)
        self.assertTrue(out.nominal)
        self.assertAlmostEqual(out.crossTrack, 0.0, places=6)
        self.assertAlmostEqual(out.nu1, 0.0, places=6)
        self.assertLess(out.nu2, 0.0)
        self.assertAlmostEqual(out.angle_to_turn, out.nu2, places=6)

    def test_nu2_is_target_bearing_minus_course(self):
        """On a northward path, a 15-deg right course error gives nu2=-15 deg."""
        inp = self._active_input(pos_n=100.0, pos_e=0.0, course_deg=15.0, speed=8.0)
        out = self._run(inp)
        self.assertTrue(out.nominal)
        self.assertAlmostEqual(out.crossTrack, 0.0, places=6)
        self.assertAlmostEqual(out.nu1, 0.0, places=6)
        self.assertAlmostEqual(out.nu2, math.radians(-15.0), places=6)
        self.assertAlmostEqual(out.angle_to_turn, out.nu2, places=6)

    def test_fixed_target_is_the_carrot_point(self):
        inp = self._active_input(pos_n=100.0, pos_e=-50.0, course_deg=0.0, speed=8.0)
        out = self._run(inp)
        self.assertAlmostEqual(out.carrot_N, out.target_N)
        self.assertAlmostEqual(out.carrot_E, out.target_E)
        self.assertAlmostEqual(out.carrot_lat, out.target_lat)
        self.assertAlmostEqual(out.carrot_lon, out.target_lon)

    def test_course_rate_clamped_to_max(self):
        inp = self._active_input(pos_n=100.0, pos_e=-500.0, course_deg=90.0, speed=8.0)
        out = self._run(inp)
        self.assertLessEqual(abs(out.angular_velocity_cmd_rad_s), guidance.COURSE_RATE_MAX + 1e-9)

    def test_lat_acc_clamped_to_max(self):
        inp = self._active_input(pos_n=100.0, pos_e=-500.0, course_deg=90.0, speed=8.0)
        out = self._run(inp)
        self.assertLessEqual(abs(out.lat_acc_cmd_mps2), guidance.LAT_ACC_MAX + 1e-9)

    def test_degraded_mode_sets_degraded_flag(self):
        inp = self._active_input()
        out = self._run(inp, mode=ControlMode.DEGRADED_CLOSED_LOOP)
        self.assertTrue(out.nominal)
        self.assertTrue(out.degraded)

    def test_active_mode_degraded_false(self):
        inp = self._active_input()
        out = self._run(inp, mode=ControlMode.NOMINAL_CLOSED_LOOP)
        self.assertTrue(out.nominal)
        self.assertFalse(out.degraded)

    def test_angular_velocity_zero_when_inactive(self):
        inp = self._active_input()
        out = guidance.ProduceL1Output(
            inp, ControlMode.NOMINAL_CLOSED_LOOP,
            ORIGIN_LAT, ORIGIN_LON, None, None, time.monotonic()
        )
        self.assertAlmostEqual(out.angular_velocity_cmd_rad_s, 0.0)


class TestProduceL1Input(unittest.TestCase):
    def test_all_fresh_sensors_gives_active_closed_loop(self):
        gps = _gps(ORIGIN_LAT + 0.001, ORIGIN_LON + 0.001,
                   course_rad=math.radians(45.0), speed=8.0, age=0.0)
        imu = _imu(gyrz_rad_s=0.05, age=0.0)
        baro = _baro(alt_m=200.0, age=0.0)
        now = time.monotonic()
        l1_input, mode = guidance.ProduceL1Input(
            gps, imu, baro,
            ORIGIN_LAT, ORIGIN_LON, ORIGIN_LAT + 0.01, ORIGIN_LON + 0.01, now,
        )
        self.assertEqual(mode, ControlMode.NOMINAL_CLOSED_LOOP)
        self.assertEqual(l1_input.pos_quality, SensorQuality.FRESH)
        self.assertEqual(l1_input.motion_quality, SensorQuality.FRESH)
        self.assertEqual(l1_input.gyrz_quality, SensorQuality.FRESH)

    def test_no_imu_gives_active_feedforward(self):
        gps = _gps(ORIGIN_LAT + 0.001, ORIGIN_LON, course_rad=0.0, speed=8.0, age=0.0)
        now = time.monotonic()
        l1_input, mode = guidance.ProduceL1Input(
            gps, None, None,
            ORIGIN_LAT, ORIGIN_LON, ORIGIN_LAT + 0.01, ORIGIN_LON, now,
        )
        self.assertEqual(mode, ControlMode.NOMINAL_FEEDFORWARD)

    def test_no_origin_gives_fail(self):
        gps = _gps(ORIGIN_LAT + 0.001, ORIGIN_LON, speed=8.0, age=0.0)
        now = time.monotonic()
        l1_input, mode = guidance.ProduceL1Input(
            gps, None, None,
            None, None, ORIGIN_LAT + 0.01, ORIGIN_LON, now,
        )
        self.assertEqual(mode, ControlMode.FAIL)

    def test_stale_gps_gives_fail(self):
        gps = _gps(ORIGIN_LAT + 0.001, ORIGIN_LON, age=guidance.POS_FRESH_AGE + 0.5)
        now = time.monotonic()
        l1_input, mode = guidance.ProduceL1Input(
            gps, None, None,
            ORIGIN_LAT, ORIGIN_LON, ORIGIN_LAT + 0.01, ORIGIN_LON, now,
        )
        self.assertEqual(mode, ControlMode.FAIL)
        self.assertEqual(l1_input.pos_quality, SensorQuality.STALE)


if __name__ == "__main__":
    unittest.main()
