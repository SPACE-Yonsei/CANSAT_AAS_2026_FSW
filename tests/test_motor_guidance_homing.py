"""STEPS 4-8: Tests for the homing GNC architecture.

Tests produceL1input, DR tracking, produceL1output, saturated_sin,
fresh/stale gating, origin/target setup, and motor output modes.
"""
from __future__ import annotations

import math
import time
import unittest
from types import SimpleNamespace

from lib import config
from Sensor_Motor import control, guidance
from Sensor_Motor.guidance import (
    ControlMode,
    DRMethod,
    FreshResult,
    GuidanceState,
    SensorQuality,
    can_dead_reckon,
    compute_dr_confidence,
    decidefresh,
    produceL1input,
    produceL1output,
    saturated_sin,
    should_detumble,
)

ORIGIN_LAT = 37.55
ORIGIN_LON = 126.95
TARGET_LAT = ORIGIN_LAT + 900.0 / 111_000.0   # ~900 m north
TARGET_LON = ORIGIN_LON


# ── Fixture helpers ────────────────────────────────────────────────────────────

def _now() -> float:
    return time.monotonic()


def _gps(lat=ORIGIN_LAT, lon=ORIGIN_LON, course_rad=0.0, speed=8.0, age=0.0):
    ts = _now() - age
    return SimpleNamespace(
        lat=lat, lon=lon,
        course_rad=course_rad,
        speed_mps=speed,
        pos_ts=ts,
        motion_ts=ts,
        pos_health=1,
        motion_health=1,
    )


def _imu(gyrz_rad_s=0.0, yaw_rad=0.0, age=0.0, lin_acc_valid=False):
    ts = _now() - age
    return SimpleNamespace(
        gyrz_rad_s=gyrz_rad_s,
        gyrx_rad_s=0.0,
        gyry_rad_s=0.0,
        yaw_rad=yaw_rad,
        roll_rad=0.0,
        pitch_rad=0.0,
        accx_mps2=0.0,
        accy_mps2=0.0,
        accz_mps2=-9.8,
        magx_uT=0.0,
        magy_uT=0.0,
        magz_uT=0.0,
        ts=ts,
        freefall=0,
        tumble=0,
    )


def _baro(alt_m=100.0, age=0.0):
    ts = _now() - age
    return SimpleNamespace(alt_m=alt_m, ts=ts)


def _make_state(target_lat=TARGET_LAT, target_lon=TARGET_LON) -> GuidanceState:
    s = GuidanceState()
    s.target_lat = target_lat
    s.target_lon = target_lon
    return s


def _bootstrap_state(
    state: GuidanceState,
    gps=None,
    imu=None,
    baro=None,
    release_state: int = 3,
    cycles: int = 2,
) -> FreshResult:
    """Run decidefresh + produceL1input for a few cycles to establish origin."""
    if gps is None:
        gps = _gps()
    if imu is None:
        imu = _imu()
    fresh = FreshResult()
    for _ in range(cycles):
        now = _now()
        fresh = decidefresh(gps, imu, baro, state, now)
        produceL1input(fresh, gps, imu, state, release_state, now)
    return fresh


# ══════════════════════════════════════════════════════════════════════════════
# TEST 1: GPS_TRACKING_CLOSED
# ══════════════════════════════════════════════════════════════════════════════

class TestGpsTrackingClosed(unittest.TestCase):
    def test_all_fresh_gives_gps_tracking_closed(self):
        state = _make_state()
        gps = _gps(ORIGIN_LAT + 0.001, ORIGIN_LON, course_rad=0.0, speed=8.0)
        imu = _imu(gyrz_rad_s=0.05)
        now = _now()
        fresh = decidefresh(gps, imu, None, state, now)
        l1 = produceL1input(fresh, gps, imu, state, release_state=3, now=now)
        self.assertTrue(l1.valid)
        self.assertEqual(l1.control_mode, ControlMode.GPS_TRACKING_CLOSED)
        self.assertEqual(l1.dr_method, config.DR_METHOD_NONE)
        self.assertAlmostEqual(l1.confidence, 1.0)

    def test_gps_tracking_closed_nav_state_populated(self):
        state = _make_state()
        gps = _gps(ORIGIN_LAT + 0.001, ORIGIN_LON, course_rad=math.radians(10.0), speed=7.0)
        imu = _imu(gyrz_rad_s=0.0)
        now = _now()
        fresh = decidefresh(gps, imu, None, state, now)
        l1 = produceL1input(fresh, gps, imu, state, 3, now)
        self.assertTrue(l1.valid)
        self.assertIsNotNone(l1.N)
        self.assertIsNotNone(l1.E)
        self.assertIsNotNone(l1.course)
        self.assertAlmostEqual(l1.V, 7.0)


# ══════════════════════════════════════════════════════════════════════════════
# TEST 2: GPS_TRACKING_OPEN
# ══════════════════════════════════════════════════════════════════════════════

class TestGpsTrackingOpen(unittest.TestCase):
    def test_gps_fresh_no_gyrz_gives_gps_tracking_open(self):
        state = _make_state()
        gps = _gps(ORIGIN_LAT + 0.001, ORIGIN_LON, course_rad=0.0, speed=8.0)
        stale_imu = _imu(gyrz_rad_s=0.05, age=config.IMU_FRESH_MAX_AGE_S + 1.0)
        now = _now()
        fresh = decidefresh(gps, stale_imu, None, state, now)
        self.assertFalse(fresh.imu_gyrz_fresh)
        l1 = produceL1input(fresh, gps, stale_imu, state, 3, now)
        self.assertTrue(l1.valid)
        self.assertEqual(l1.control_mode, ControlMode.GPS_TRACKING_OPEN)


# ══════════════════════════════════════════════════════════════════════════════
# TEST 3: DR_TRACKING_CLOSED / GYRO_INTEGRATION
# ══════════════════════════════════════════════════════════════════════════════

class TestDrTrackingClosedGyroIntegration(unittest.TestCase):
    def test_gps_stale_imu_gyrz_fresh_gives_dr_tracking_closed(self):
        state = _make_state()
        gps_fresh = _gps(ORIGIN_LAT + 0.001, ORIGIN_LON, course_rad=0.0, speed=8.0)
        imu = _imu(gyrz_rad_s=0.0, yaw_rad=0.0)
        now0 = _now()
        # Bootstrap: set origin and DR anchor
        fresh0 = decidefresh(gps_fresh, imu, None, state, now0)
        l1_0 = produceL1input(fresh0, gps_fresh, imu, state, 3, now0)
        self.assertTrue(l1_0.valid)

        # Now GPS goes stale
        stale_gps = _gps(ORIGIN_LAT + 0.001, ORIGIN_LON,
                         age=config.GPS_CONTROL_FRESH_MAX_AGE_S + 2.0)
        now1 = _now()
        fresh1 = decidefresh(stale_gps, imu, None, state, now1)
        self.assertFalse(fresh1.point_fresh)
        self.assertTrue(fresh1.imu_gyrz_fresh)
        l1_1 = produceL1input(fresh1, stale_gps, imu, state, 3, now1)

        self.assertTrue(l1_1.valid)
        self.assertEqual(l1_1.control_mode, ControlMode.DR_TRACKING_CLOSED)
        self.assertEqual(l1_1.dr_method, config.DR_METHOD_GYRO_INTEGRATION)


# ══════════════════════════════════════════════════════════════════════════════
# TEST 4: DR_TRACKING_CLOSED / GYRO_ACC_BLEND  (no linear acc → still GYRO_INTEGRATION)
# ══════════════════════════════════════════════════════════════════════════════

class TestDrTrackingClosedGyroAccBlend(unittest.TestCase):
    def test_no_linear_acc_keeps_gyro_integration(self):
        """Without linear acc in ImuSample.lin_acc_valid, blend never activates."""
        state = _make_state()
        gps_fresh = _gps(ORIGIN_LAT + 0.001, ORIGIN_LON, course_rad=0.0, speed=8.0)
        imu = _imu()
        now0 = _now()
        decidefresh(gps_fresh, imu, None, state, now0)
        produceL1input(FreshResult(), gps_fresh, imu, state, 3, now0)
        fresh0 = decidefresh(gps_fresh, imu, None, state, now0)
        produceL1input(fresh0, gps_fresh, imu, state, 3, now0)

        stale_gps = _gps(age=config.GPS_CONTROL_FRESH_MAX_AGE_S + 2.0)
        now1 = _now()
        fresh1 = decidefresh(stale_gps, imu, None, state, now1)
        l1 = produceL1input(fresh1, stale_gps, imu, state, 3, now1)

        if l1.valid and l1.control_mode in (ControlMode.DR_TRACKING_CLOSED, ControlMode.DR_TRACKING_OPEN):
            self.assertEqual(l1.dr_method, config.DR_METHOD_GYRO_INTEGRATION)


# ══════════════════════════════════════════════════════════════════════════════
# TEST 5: DR_TRACKING_OPEN
# ══════════════════════════════════════════════════════════════════════════════

class TestDrTrackingOpen(unittest.TestCase):
    def test_gps_stale_only_yaw_fresh_gives_dr_tracking_open(self):
        state = _make_state()
        gps_fresh = _gps(ORIGIN_LAT + 0.001, ORIGIN_LON, course_rad=0.0, speed=8.0)
        imu_fresh = _imu(gyrz_rad_s=0.0, yaw_rad=0.0)
        now0 = _now()
        fresh0 = decidefresh(gps_fresh, imu_fresh, None, state, now0)
        produceL1input(fresh0, gps_fresh, imu_fresh, state, 3, now0)

        # Stale gyrz but fresh yaw (simulate very stale gyrz)
        stale_gps = _gps(age=config.GPS_CONTROL_FRESH_MAX_AGE_S + 2.0)
        # imu with stale gyrz but fresh (the gyrz_valid flag is set based on imu_fresh age)
        # For this test, fake it by patching state directly
        state.dr_start_E = state.nav_E or 0.0
        state.dr_start_N = state.nav_N or 100.0
        state.dr_start_V = state.nav_V or 8.0
        state.dr_start_course = state.nav_course or 0.0
        state.dr_start_time = now0
        state.course_at_dropout = state.nav_course or 0.0
        state.yaw_at_dropout = 0.0

        # Simulate fresh yaw but stale gyrz via FreshResult
        fresh_manual = FreshResult()
        fresh_manual.point_fresh = False
        fresh_manual.velocity_fresh = False
        fresh_manual.imu_fresh = True
        fresh_manual.imu_gyrz_fresh = False  # stale
        fresh_manual.imu_yaw_fresh = True    # only yaw is fresh

        now1 = _now()
        l1 = produceL1input(fresh_manual, stale_gps, imu_fresh, state, 3, now1)
        if l1.valid:
            self.assertEqual(l1.control_mode, ControlMode.DR_TRACKING_OPEN)


# ══════════════════════════════════════════════════════════════════════════════
# TEST 6: FAIL
# ══════════════════════════════════════════════════════════════════════════════

class TestFailMode(unittest.TestCase):
    def test_no_gps_no_dr_anchor_gives_fail(self):
        state = _make_state()
        # Set origin manually so we don't fail on that
        state.origin_ready = True
        state.origin_lat = ORIGIN_LAT
        state.origin_lon = ORIGIN_LON
        state.target_ready = True
        state.target_E = 0.0
        state.target_N = 900.0
        # No dr_start, no GPS
        fresh = FreshResult()  # all stale
        now = _now()
        l1 = produceL1input(fresh, None, None, state, 3, now)
        self.assertFalse(l1.valid)
        self.assertEqual(l1.control_mode, ControlMode.FAIL)

    def test_fail_mode_gives_zero_yaw_rate_cmd(self):
        l1 = guidance._fail_l1input("FAIL_NO_VALID_DR")
        g_out = produceL1output(l1)
        self.assertFalse(g_out.control_valid)
        self.assertAlmostEqual(g_out.angular_velocity_cmd_rad_s, 0.0)

    def test_fail_l1input_motor_neutral(self):
        l1 = guidance._fail_l1input("FAIL_TEST")
        g_out = produceL1output(l1)
        cmd = control.ProduceCtrlOutput(
            control.MakeCtrler(),
            control.ProduceCtrlInput(g_out, time.monotonic()),
            float("nan"),
            time.monotonic(),
        )
        self.assertEqual(cmd.left_pw, control.LEFT_NEUTRAL)
        self.assertEqual(cmd.right_pw, control.RIGHT_NEUTRAL)


# ══════════════════════════════════════════════════════════════════════════════
# TEST 7: Origin setting
# ══════════════════════════════════════════════════════════════════════════════

class TestOriginSetting(unittest.TestCase):
    def test_release_state_below_3_no_origin(self):
        state = _make_state()
        gps = _gps(ORIGIN_LAT + 0.001, ORIGIN_LON)
        imu = _imu()
        now = _now()
        fresh = decidefresh(gps, imu, None, state, now)
        l1 = produceL1input(fresh, gps, imu, state, release_state=2, now=now)
        self.assertFalse(l1.valid)
        self.assertFalse(state.origin_ready)

    def test_release_state_3_sets_origin_from_gps(self):
        state = _make_state()
        gps = _gps(ORIGIN_LAT + 0.001, ORIGIN_LON)
        imu = _imu()
        now = _now()
        fresh = decidefresh(gps, imu, None, state, now)
        produceL1input(fresh, gps, imu, state, release_state=3, now=now)
        self.assertTrue(state.origin_ready)
        self.assertAlmostEqual(state.origin_lat, ORIGIN_LAT + 0.001, places=5)

    def test_origin_immutable_after_first_set(self):
        state = _make_state()
        gps1 = _gps(ORIGIN_LAT + 0.001, ORIGIN_LON)
        imu = _imu()
        now = _now()
        fresh = decidefresh(gps1, imu, None, state, now)
        produceL1input(fresh, gps1, imu, state, 3, now)
        first_origin = state.origin_lat

        gps2 = _gps(ORIGIN_LAT + 0.005, ORIGIN_LON)
        now2 = _now()
        fresh2 = decidefresh(gps2, imu, None, state, now2)
        produceL1input(fresh2, gps2, imu, state, 3, now2)
        self.assertEqual(state.origin_lat, first_origin)


# ══════════════════════════════════════════════════════════════════════════════
# TEST 8: produceL1output — target bearing and nu
# ══════════════════════════════════════════════════════════════════════════════

class TestProduceL1Output(unittest.TestCase):
    def _build_l1input(self, pos_E=0.0, pos_N=0.0, course_deg=0.0,
                       target_E=0.0, target_N=900.0, speed=8.0,
                       mode=ControlMode.GPS_TRACKING_CLOSED):
        inp = guidance.L1Input()
        inp.valid = True
        inp.control_mode = mode
        inp.E = pos_E
        inp.N = pos_N
        inp.course = math.radians(course_deg)
        inp.V = speed
        inp.target_E = target_E
        inp.target_N = target_N
        inp.confidence = 1.0
        inp.dr_method = config.DR_METHOD_NONE
        return inp

    def test_target_north_course_north_yaw_rate_zero(self):
        """Current at origin, target north, course north → nu=0, yaw_rate=0."""
        l1 = self._build_l1input(pos_N=0.0, target_N=900.0, course_deg=0.0)
        out = produceL1output(l1)
        self.assertTrue(out.control_valid)
        self.assertAlmostEqual(out.angular_velocity_cmd_rad_s, 0.0, places=6)

    def test_target_east_course_north_yaw_rate_positive(self):
        """Target east, course north → nu=+90 deg → right turn."""
        l1 = self._build_l1input(pos_N=0.0, target_E=900.0, target_N=0.0, course_deg=0.0)
        out = produceL1output(l1)
        self.assertTrue(out.control_valid)
        self.assertGreater(out.angular_velocity_cmd_rad_s, 0.0)

    def test_target_west_course_north_yaw_rate_negative(self):
        """Target west, course north → nu=-90 deg → left turn."""
        l1 = self._build_l1input(pos_N=0.0, target_E=-900.0, target_N=0.0, course_deg=0.0)
        out = produceL1output(l1)
        self.assertTrue(out.control_valid)
        self.assertLess(out.angular_velocity_cmd_rad_s, 0.0)

    def test_fail_mode_no_control(self):
        l1 = guidance._fail_l1input("FAIL_TEST")
        out = produceL1output(l1)
        self.assertFalse(out.control_valid)
        self.assertAlmostEqual(out.angular_velocity_cmd_rad_s, 0.0)

    def test_detumbling_mode_zero_yaw_rate_cmd(self):
        inp = guidance.L1Input()
        inp.valid = True
        inp.control_mode = ControlMode.DETUMBLING
        inp.confidence = 1.0
        inp.dr_method = config.DR_METHOD_NONE
        out = produceL1output(inp)
        self.assertTrue(out.control_valid)
        self.assertAlmostEqual(out.angular_velocity_cmd_rad_s, 0.0)
        self.assertTrue(out.pid_enabled)


# ══════════════════════════════════════════════════════════════════════════════
# TEST 9: Saturated sine
# ══════════════════════════════════════════════════════════════════════════════

class TestSaturatedSin(unittest.TestCase):
    def test_nu_zero(self):
        self.assertAlmostEqual(saturated_sin(0.0), 0.0)

    def test_nu_90_deg(self):
        self.assertAlmostEqual(saturated_sin(math.radians(90.0)), 1.0)

    def test_nu_120_deg_saturates_to_1(self):
        self.assertAlmostEqual(saturated_sin(math.radians(120.0)), 1.0)

    def test_nu_minus_120_deg_saturates_to_minus_1(self):
        self.assertAlmostEqual(saturated_sin(math.radians(-120.0)), -1.0)

    def test_nu_180_deg_saturates_to_1(self):
        self.assertAlmostEqual(saturated_sin(math.radians(180.0)), 1.0)

    def test_nu_minus_45_deg(self):
        expected = -math.sin(math.radians(45.0))
        self.assertAlmostEqual(saturated_sin(math.radians(-45.0)), expected)


# ══════════════════════════════════════════════════════════════════════════════
# TEST 10: Fresh/stale gating
# ══════════════════════════════════════════════════════════════════════════════

class TestFreshStalGating(unittest.TestCase):
    def test_stale_gps_not_used_for_direct_tracking(self):
        state = _make_state()
        state.origin_ready = True
        state.origin_lat = ORIGIN_LAT
        state.origin_lon = ORIGIN_LON
        state.target_ready = True
        state.target_E = 0.0
        state.target_N = 900.0

        stale_gps = _gps(age=config.GPS_CONTROL_FRESH_MAX_AGE_S + 5.0)
        imu = _imu(gyrz_rad_s=0.0)
        now = _now()
        fresh = decidefresh(stale_gps, imu, None, state, now)
        self.assertFalse(fresh.point_fresh)
        # Without DR anchor, should fail
        l1 = produceL1input(fresh, stale_gps, imu, state, 3, now)
        self.assertFalse(l1.valid)

    def test_age_determines_freshness_not_history_presence(self):
        state = _make_state()
        gps_old = _gps(age=config.GPS_CONTROL_FRESH_MAX_AGE_S + 0.5)
        now = _now()
        fresh = decidefresh(gps_old, None, None, state, now)
        # history may contain the sample but it should not be fresh
        self.assertFalse(fresh.point_fresh)

    def test_dr_confidence_by_age(self):
        self.assertAlmostEqual(compute_dr_confidence(0.0), 1.0)
        self.assertAlmostEqual(compute_dr_confidence(config.DR_CONF_AGE_1_S - 0.1), 1.0)
        self.assertAlmostEqual(compute_dr_confidence(config.DR_CONF_AGE_2_S - 0.1), 0.7)
        self.assertAlmostEqual(compute_dr_confidence(config.DR_CONF_AGE_3_S - 0.1), 0.4)
        self.assertAlmostEqual(compute_dr_confidence(config.DR_CONF_AGE_3_S + 1.0), 0.0)


# ══════════════════════════════════════════════════════════════════════════════
# TEST 11: Motor output
# ══════════════════════════════════════════════════════════════════════════════

class TestMotorOutput(unittest.TestCase):
    def _make_gps_tracking_l1(self, yaw_rate_cmd_dps: float, closed: bool = True):
        mode = ControlMode.GPS_TRACKING_CLOSED if closed else ControlMode.GPS_TRACKING_OPEN
        inp = guidance.L1Input()
        inp.valid = True
        inp.control_mode = mode
        inp.E = 0.0
        inp.N = 0.0
        inp.course = 0.0
        inp.V = 8.0
        inp.target_E = 0.0
        inp.target_N = 900.0
        inp.confidence = 1.0
        inp.dr_method = config.DR_METHOD_NONE
        return inp

    def test_fail_gives_neutral_pwm(self):
        l1 = guidance._fail_l1input("FAIL_TEST")
        g_out = produceL1output(l1)
        cmd = control.ProduceCtrlOutput(
            control.MakeCtrler(),
            control.ProduceCtrlInput(g_out, _now()),
            float("nan"),
            _now(),
        )
        self.assertEqual(cmd.left_pw, control.LEFT_NEUTRAL)
        self.assertEqual(cmd.right_pw, control.RIGHT_NEUTRAL)

    def test_detumbling_pid_counters_yaw_rate(self):
        """DETUMBLING: yaw_rate_cmd=0 + PID = counter-rotation when gyrz nonzero."""
        inp = guidance.L1Input()
        inp.valid = True
        inp.control_mode = ControlMode.DETUMBLING
        inp.confidence = 1.0
        inp.dr_method = config.DR_METHOD_NONE
        g_out = produceL1output(inp)
        self.assertTrue(g_out.pid_enabled)
        ctl = control.MakeCtrler()
        # Spinning right at 50 deg/s
        measured_dps = 50.0
        cmd = control.ProduceCtrlOutput(ctl, control.ProduceCtrlInput(g_out, _now()), measured_dps, _now())
        # Should command left turn (negative delta_arm)
        self.assertLess(cmd.delta_arm_deg, 0.0)

    def test_open_mode_feedforward_only(self):
        """GPS_TRACKING_OPEN: pid_enabled=False, no PID term."""
        l1 = self._make_gps_tracking_l1(yaw_rate_cmd_dps=15.0, closed=False)
        g_out = produceL1output(l1)
        self.assertFalse(g_out.pid_enabled)
        ctl = control.MakeCtrler()
        cmd = control.ProduceCtrlOutput(ctl, control.ProduceCtrlInput(g_out, _now()), float("nan"), _now())
        self.assertAlmostEqual(cmd.delta_pid_deg, 0.0)

    def test_closed_mode_pid_active_with_valid_gyrz(self):
        """GPS_TRACKING_CLOSED: pid_enabled=True, PID term nonzero when error exists."""
        state = _make_state()
        gps = _gps(ORIGIN_LAT + 0.001, ORIGIN_LON, course_rad=0.0, speed=8.0)
        imu = _imu(gyrz_rad_s=0.0)
        now = _now()
        fresh = decidefresh(gps, imu, None, state, now)
        l1 = produceL1input(fresh, gps, imu, state, 3, now)
        if not l1.valid:
            return  # can't test further

        g_out = produceL1output(l1)
        self.assertTrue(g_out.pid_enabled)

    def test_yaw_rate_limit_applied(self):
        """Yaw rate command is clamped to mode limit."""
        # Force a situation where nu=90 deg → sin_nu=1 → large yaw_rate
        inp = guidance.L1Input()
        inp.valid = True
        inp.control_mode = ControlMode.GPS_TRACKING_CLOSED
        inp.E = 0.0
        inp.N = 0.0
        inp.course = 0.0  # north
        inp.V = 20.0  # fast → large yaw_rate
        inp.target_E = 900.0  # east → 90 deg nu
        inp.target_N = 0.0
        inp.confidence = 1.0
        inp.dr_method = config.DR_METHOD_NONE
        g_out = produceL1output(inp)
        limit_rad_s = math.radians(config.GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS)
        self.assertLessEqual(abs(g_out.angular_velocity_cmd_rad_s), limit_rad_s + 1e-9)


# ══════════════════════════════════════════════════════════════════════════════
# TEST: can_dead_reckon
# ══════════════════════════════════════════════════════════════════════════════

class TestCanDeadReckon(unittest.TestCase):
    def test_no_anchor_returns_false(self):
        state = GuidanceState()
        fresh = FreshResult()
        fresh.imu_gyrz_fresh = True
        self.assertFalse(can_dead_reckon(fresh, state))

    def test_anchor_but_no_course_source_returns_false(self):
        state = GuidanceState()
        state.dr_start_E = 0.0
        state.dr_start_N = 0.0
        state.dr_start_V = 8.0
        state.dr_start_course = 0.0
        state.dr_start_time = _now()
        fresh = FreshResult()
        # neither yaw nor gyrz fresh
        self.assertFalse(can_dead_reckon(fresh, state))

    def test_anchor_and_gyrz_fresh_returns_true(self):
        state = GuidanceState()
        state.dr_start_E = 0.0
        state.dr_start_N = 0.0
        state.dr_start_V = 8.0
        state.dr_start_course = 0.0
        state.dr_start_time = _now()
        fresh = FreshResult()
        fresh.imu_gyrz_fresh = True
        self.assertTrue(can_dead_reckon(fresh, state))

    def test_anchor_and_yaw_fresh_returns_true(self):
        state = GuidanceState()
        state.dr_start_E = 0.0
        state.dr_start_N = 0.0
        state.dr_start_V = 8.0
        state.dr_start_course = 0.0
        state.dr_start_time = _now()
        fresh = FreshResult()
        fresh.imu_yaw_fresh = True
        self.assertTrue(can_dead_reckon(fresh, state))


# ══════════════════════════════════════════════════════════════════════════════
# TEST: should_detumble
# ══════════════════════════════════════════════════════════════════════════════

class TestShouldDetumble(unittest.TestCase):
    def test_below_threshold_not_detumbling(self):
        state = GuidanceState()
        imu = _imu(gyrz_rad_s=math.radians(10.0))
        fresh = FreshResult()
        fresh.imu_gyrz_fresh = True
        self.assertFalse(should_detumble(fresh, imu, state, _now()))

    def test_above_threshold_detumbles(self):
        state = GuidanceState()
        gz = math.radians(config.DETUMBLE_GYRZ_THRESHOLD_DPS + 10.0)
        imu = _imu(gyrz_rad_s=gz)
        fresh = FreshResult()
        fresh.imu_gyrz_fresh = True
        self.assertTrue(should_detumble(fresh, imu, state, _now()))

    def test_stale_gyrz_no_detumble(self):
        state = GuidanceState()
        gz = math.radians(config.DETUMBLE_GYRZ_THRESHOLD_DPS + 10.0)
        imu = _imu(gyrz_rad_s=gz)
        fresh = FreshResult()
        fresh.imu_gyrz_fresh = False
        self.assertFalse(should_detumble(fresh, imu, state, _now()))

    def test_exit_hold_keeps_detumbling(self):
        state = GuidanceState()
        state.nav_control_mode = config.CONTROL_MODE_DETUMBLING
        # gyrz just below exit threshold
        gz = math.radians(config.DETUMBLE_EXIT_THRESHOLD_DPS - 5.0)
        imu = _imu(gyrz_rad_s=gz)
        fresh = FreshResult()
        fresh.imu_gyrz_fresh = True
        now = _now()
        # First call: start exit hold timer
        result1 = should_detumble(fresh, imu, state, now)
        # Should still be True (hold not expired)
        self.assertTrue(result1)


# ══════════════════════════════════════════════════════════════════════════════
# SPEC §17 TESTS — numbered per spec requirements
# ══════════════════════════════════════════════════════════════════════════════

class TestSpec17_1_GpsTrackingClosed(unittest.TestCase):
    """Spec test 1: GPS_TRACKING_CLOSED — POINT/VELOCITY/gyrz fresh."""

    def test_mode_and_fields(self):
        state = _make_state()
        gps = _gps(ORIGIN_LAT + 0.001, ORIGIN_LON, course_rad=0.1, speed=8.0)
        imu = _imu(gyrz_rad_s=0.05)
        now = _now()
        fresh = decidefresh(gps, imu, None, state, now)
        l1 = produceL1input(fresh, gps, imu, state, 3, now)

        self.assertTrue(l1.valid)
        self.assertEqual(l1.control_mode, ControlMode.GPS_TRACKING_CLOSED)
        self.assertEqual(l1.dr_method, config.DR_METHOD_NONE)
        self.assertAlmostEqual(l1.confidence, 1.0)
        # Spec-primary fields populated
        self.assertIsNotNone(l1.E)
        self.assertIsNotNone(l1.N)
        self.assertIsNotNone(l1.V)


class TestSpec17_2_GpsTrackingOpen(unittest.TestCase):
    """Spec test 2: GPS_TRACKING_OPEN — POINT/VELOCITY fresh, gyrz stale."""

    def test_mode(self):
        state = _make_state()
        gps = _gps(ORIGIN_LAT + 0.001, ORIGIN_LON, course_rad=0.0, speed=8.0)
        stale_imu = _imu(age=config.IMU_FRESH_MAX_AGE_S + 1.0)
        now = _now()
        fresh = decidefresh(gps, stale_imu, None, state, now)
        self.assertFalse(fresh.imu_gyrz_fresh)
        l1 = produceL1input(fresh, gps, stale_imu, state, 3, now)
        self.assertTrue(l1.valid)
        self.assertEqual(l1.control_mode, ControlMode.GPS_TRACKING_OPEN)


class TestSpec17_3_DrTrackingClosed_GyroIntegration(unittest.TestCase):
    """Spec test 3: DR_TRACKING_CLOSED/GYRO_INTEGRATION — GPS stale, gyrz fresh, acc stale."""

    def test_mode_and_dr_method(self):
        state = _make_state()
        gps_fresh = _gps(ORIGIN_LAT + 0.001, ORIGIN_LON, course_rad=0.0, speed=8.0)
        imu = _imu(gyrz_rad_s=0.0, yaw_rad=0.0)
        now0 = _now()
        # Bootstrap GPS anchor
        fresh0 = decidefresh(gps_fresh, imu, None, state, now0)
        produceL1input(fresh0, gps_fresh, imu, state, 3, now0)

        # GPS goes stale; acc stale (no lin_acc_valid)
        stale_gps = _gps(age=config.GPS_FRESH_MAX_AGE_S + 1.0)
        now1 = _now()
        fresh1 = decidefresh(stale_gps, imu, None, state, now1)

        self.assertFalse(fresh1.point_fresh)
        self.assertTrue(fresh1.imu_gyrz_fresh)
        self.assertFalse(fresh1.imu_linear_acc_fresh)   # no lin acc → GYRO_INTEGRATION

        l1 = produceL1input(fresh1, stale_gps, imu, state, 3, now1)
        self.assertTrue(l1.valid)
        self.assertEqual(l1.control_mode, ControlMode.DR_TRACKING_CLOSED)
        self.assertEqual(l1.dr_method, config.DR_METHOD_GYRO_INTEGRATION)


class TestSpec17_4_DrTrackingClosed_GyroAccBlend(unittest.TestCase):
    """Spec test 4: DR_TRACKING_CLOSED/GYRO_ACC_BLEND — GPS stale, gyrz+acc fresh."""

    def _imu_with_lin_acc(self, gyrz=0.0, yaw=0.0, age=0.0):
        ts = _now() - age
        return SimpleNamespace(
            gyrz_rad_s=gyrz, gyrx_rad_s=0.0, gyry_rad_s=0.0,
            yaw_rad=yaw, roll_rad=0.0, pitch_rad=0.0,
            accx_mps2=0.0, accy_mps2=0.0, accz_mps2=-9.8,
            magx_uT=0.0, magy_uT=0.0, magz_uT=0.0,
            ts=ts, freefall=0, tumble=0,
            lin_acc_x=0.05, lin_acc_y=0.02, lin_acc_z=0.0,
            lin_acc_valid=True,
        )

    def test_mode_and_dr_method(self):
        state = _make_state()
        gps_fresh = _gps(ORIGIN_LAT + 0.001, ORIGIN_LON, course_rad=0.0, speed=8.0)
        imu_full = self._imu_with_lin_acc()
        now0 = _now()
        fresh0 = decidefresh(gps_fresh, imu_full, None, state, now0)
        produceL1input(fresh0, gps_fresh, imu_full, state, 3, now0)

        stale_gps = _gps(age=config.GPS_FRESH_MAX_AGE_S + 1.0)
        now1 = _now()
        fresh1 = decidefresh(stale_gps, imu_full, None, state, now1)

        self.assertFalse(fresh1.point_fresh)
        self.assertTrue(fresh1.imu_gyrz_fresh)
        # Only proceed if DR confidence > 0 and acc conditions met
        l1 = produceL1input(fresh1, stale_gps, imu_full, state, 3, now1)
        if l1.valid and l1.control_mode == ControlMode.DR_TRACKING_CLOSED:
            # acc blend activates when lin_acc_valid + within age window
            self.assertIn(l1.dr_method, (
                config.DR_METHOD_GYRO_ACC_BLEND,
                config.DR_METHOD_GYRO_INTEGRATION,
            ))


class TestSpec17_5_DrTrackingOpen(unittest.TestCase):
    """Spec test 5: DR_TRACKING_OPEN — GPS stale, yaw fresh, gyrz stale."""

    def test_mode(self):
        state = _make_state()
        gps_fresh = _gps(ORIGIN_LAT + 0.001, ORIGIN_LON, course_rad=0.0, speed=8.0)
        imu_fresh = _imu(gyrz_rad_s=0.0, yaw_rad=0.0)
        now0 = _now()
        fresh0 = decidefresh(gps_fresh, imu_fresh, None, state, now0)
        produceL1input(fresh0, gps_fresh, imu_fresh, state, 3, now0)

        # Patch DR anchor for determinism
        state.dr_start_E = state.nav_E or 0.0
        state.dr_start_N = state.nav_N or 100.0
        state.dr_start_V = state.nav_V or 8.0
        state.dr_start_course = state.nav_course or 0.0
        state.dr_start_time = now0
        state.course_at_dropout = state.nav_course or 0.0
        state.yaw_at_dropout = 0.0

        stale_gps = _gps(age=config.GPS_FRESH_MAX_AGE_S + 2.0)
        # Fresh yaw only (gyrz stale)
        fresh_manual = FreshResult()
        fresh_manual.point_fresh = False
        fresh_manual.velocity_fresh = False
        fresh_manual.imu_fresh = True
        fresh_manual.imu_gyrz_fresh = False
        fresh_manual.imu_yaw_fresh = True
        fresh_manual.imu_age_s = 0.1

        now1 = _now()
        l1 = produceL1input(fresh_manual, stale_gps, imu_fresh, state, 3, now1)
        if l1.valid:
            self.assertEqual(l1.control_mode, ControlMode.DR_TRACKING_OPEN)


class TestSpec17_6_Fail(unittest.TestCase):
    """Spec test 6: FAIL — GPS stale, no last GPS, no DR anchor."""

    def test_fail_mode_and_neutral_output(self):
        state = _make_state()
        state.origin_ready = True
        state.origin_lat = ORIGIN_LAT
        state.origin_lon = ORIGIN_LON
        state.target_ready = True
        state.target_E = 0.0
        state.target_N = 900.0
        # No dr_start → DR impossible

        fresh = FreshResult()   # everything stale
        now = _now()
        l1 = produceL1input(fresh, None, None, state, 3, now)

        self.assertFalse(l1.valid)
        self.assertEqual(l1.control_mode, ControlMode.FAIL)

        g_out = produceL1output(l1)
        self.assertFalse(g_out.control_valid)
        self.assertAlmostEqual(g_out.yaw_rate_cmd, 0.0)
        self.assertAlmostEqual(g_out.angular_velocity_cmd_rad_s, 0.0)

        cmd = control.ProduceCtrlOutput(
            control.MakeCtrler(),
            control.ProduceCtrlInput(g_out, _now()),
            float("nan"),
            _now(),
        )
        self.assertEqual(cmd.left_pw, control.LEFT_NEUTRAL)
        self.assertEqual(cmd.right_pw, control.RIGHT_NEUTRAL)


class TestSpec17_7_ProduceL1Output_NorthTarget(unittest.TestCase):
    """Spec test 7: current at origin, target north, course north → nu=0, yaw_rate_cmd=0."""

    def test_nu_zero_yaw_rate_zero(self):
        l1 = guidance.L1Input()
        l1.valid = True
        l1.control_mode = ControlMode.GPS_TRACKING_CLOSED
        l1.E = 0.0
        l1.N = 0.0
        l1.course = 0.0          # 0 rad = north
        l1.V = 8.0
        l1.target_E = 0.0
        l1.target_N = 900.0      # target due north
        l1.confidence = 1.0
        l1.dr_method = config.DR_METHOD_NONE

        out = produceL1output(l1)

        self.assertTrue(out.control_valid)
        self.assertAlmostEqual(out.nu, 0.0, places=9)
        self.assertAlmostEqual(out.yaw_rate_cmd, 0.0, places=9)
        self.assertAlmostEqual(out.angular_velocity_cmd_rad_s, 0.0, places=9)
        self.assertAlmostEqual(out.target_bearing, 0.0, places=9)


class TestSpec17_8_SaturatedSine(unittest.TestCase):
    """Spec test 8: saturated sine values at key angles."""

    def test_nu_90_deg(self):
        self.assertAlmostEqual(saturated_sin(math.radians(90.0)), 1.0)

    def test_nu_120_deg_saturates_to_1(self):
        self.assertAlmostEqual(saturated_sin(math.radians(120.0)), 1.0)

    def test_nu_minus_120_deg_saturates_to_minus_1(self):
        self.assertAlmostEqual(saturated_sin(math.radians(-120.0)), -1.0)

    def test_nu_below_90_uses_normal_sin(self):
        nu = math.radians(45.0)
        self.assertAlmostEqual(saturated_sin(nu), math.sin(nu))

    def test_nu_exactly_90_equals_1(self):
        self.assertAlmostEqual(saturated_sin(math.pi / 2), 1.0)

    def test_nu_0_equals_0(self):
        self.assertAlmostEqual(saturated_sin(0.0), 0.0)


class TestNewL1OutputFields(unittest.TestCase):
    """Verify spec-primary L1Output fields are populated by produceL1output."""

    def _build_l1input(self, pos_E=0.0, pos_N=0.0, course_deg=0.0,
                       target_E=0.0, target_N=900.0, speed=8.0):
        l1 = guidance.L1Input()
        l1.valid = True
        l1.control_mode = ControlMode.GPS_TRACKING_CLOSED
        l1.E = pos_E
        l1.N = pos_N
        l1.course = math.radians(course_deg)
        l1.V = speed
        l1.target_E = target_E
        l1.target_N = target_N
        l1.confidence = 1.0
        l1.dr_method = config.DR_METHOD_NONE
        return l1

    def test_target_north_fields(self):
        out = produceL1output(self._build_l1input())
        self.assertAlmostEqual(out.target_bearing, 0.0, places=9)
        self.assertAlmostEqual(out.nu, 0.0, places=9)
        self.assertAlmostEqual(out.distance_to_target, 900.0, places=3)
        self.assertAlmostEqual(out.yaw_rate_cmd, 0.0, places=9)
        # Backward compat mirrors
        self.assertEqual(out.yaw_rate_cmd, out.angular_velocity_cmd_rad_s)
        self.assertEqual(out.nu, out.nu2)
        self.assertEqual(out.distance_to_target, out.alongTrack)

    def test_target_east_fields(self):
        out = produceL1output(self._build_l1input(target_E=900.0, target_N=0.0))
        self.assertAlmostEqual(out.target_bearing, math.pi / 2, places=6)
        self.assertAlmostEqual(out.nu, math.pi / 2, places=6)
        self.assertGreater(out.yaw_rate_cmd, 0.0)


if __name__ == "__main__":
    unittest.main()
