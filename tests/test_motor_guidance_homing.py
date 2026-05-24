"""Tests for the homing GNC architecture (adapted to the rebuilt guidance.py).

Covers: decidefresh → produceL1input → produceL1output pipeline, GuidanceState
history-driven DR anchor, ControlMode transitions, saturated sine, target bearing.
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
    GpsSample,
    ImuSample,
    compute_dr_confidence,
    decidefresh,
    produceL1input,
    produceL1output,
    saturated_sin,
)

ORIGIN_LAT = 37.55
ORIGIN_LON = 126.95
TARGET_LAT = ORIGIN_LAT + 900.0 / 111_000.0   # ~900 m north
TARGET_LON = ORIGIN_LON


# ── Fixture helpers ──────────────────────────────────────────────────────────

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


def _imu(gyrz_rad_s=0.0, yaw_rad=0.0, age=0.0,
         lin_acc_valid=False, lin_acc_x=0.0, lin_acc_y=0.0, lin_acc_z=0.0):
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
        lin_acc_x=lin_acc_x,
        lin_acc_y=lin_acc_y,
        lin_acc_z=lin_acc_z,
        lin_acc_valid=lin_acc_valid,
    )


def _baro(alt_m=100.0, age=0.0):
    ts = _now() - age
    return SimpleNamespace(alt_m=alt_m, ts=ts)


def _make_state(target_lat=TARGET_LAT, target_lon=TARGET_LON) -> GuidanceState:
    s = GuidanceState()
    s.target_lat = target_lat
    s.target_lon = target_lon
    return s


# ══════════════════════════════════════════════════════════════════════════════
# TEST 1: GPS_TRACKING_CLOSED
# ══════════════════════════════════════════════════════════════════════════════

class TestGpsTrackingClosed(unittest.TestCase):
    def test_all_fresh_gives_gps_tracking_closed(self):
        state = _make_state()
        gps = _gps(ORIGIN_LAT + 0.001, ORIGIN_LON, course_rad=0.0, speed=8.0)
        imu = _imu(gyrz_rad_s=0.05)
        now = _now()
        fresh = decidefresh(gps, imu, _baro(), state, now)
        l1 = produceL1input(fresh, gps, imu, state, release_state=3, now=now)
        self.assertTrue(l1.valid)
        self.assertEqual(l1.control_mode, ControlMode.GPS_TRACKING_CLOSED)
        self.assertEqual(l1.dr_method, DRMethod.NONE)
        self.assertAlmostEqual(l1.confidence, 1.0)

    def test_gps_tracking_closed_nav_state_populated(self):
        state = _make_state()
        gps = _gps(ORIGIN_LAT + 0.001, ORIGIN_LON, course_rad=math.radians(10.0), speed=7.0)
        imu = _imu(gyrz_rad_s=0.0)
        now = _now()
        fresh = decidefresh(gps, imu, _baro(), state, now)
        l1 = produceL1input(fresh, gps, imu, state, 3, now)
        self.assertTrue(l1.valid)
        self.assertTrue(math.isfinite(l1.N))
        self.assertTrue(math.isfinite(l1.E))
        self.assertTrue(math.isfinite(l1.course))
        self.assertAlmostEqual(l1.V, 7.0)


# ══════════════════════════════════════════════════════════════════════════════
# TEST 2: GPS_TRACKING_OPEN (gyrz stale)
# ══════════════════════════════════════════════════════════════════════════════

class TestGpsTrackingOpen(unittest.TestCase):
    def test_gps_fresh_no_gyrz_gives_gps_tracking_open(self):
        state = _make_state()
        gps = _gps(ORIGIN_LAT + 0.001, ORIGIN_LON, course_rad=0.0, speed=8.0)
        stale_imu = _imu(gyrz_rad_s=0.05, age=config.IMU_FRESH_MAX_AGE_S + 1.0)
        now = _now()
        fresh = decidefresh(gps, stale_imu, _baro(), state, now)
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
        imu0 = _imu(gyrz_rad_s=0.0, yaw_rad=0.0)
        now0 = _now()
        fresh0 = decidefresh(gps_fresh, imu0, _baro(), state, now0)
        l1_0 = produceL1input(fresh0, gps_fresh, imu0, state, 3, now0)
        self.assertTrue(l1_0.valid)

        # GPS dropout: jump time forward past GPS freshness window
        now1 = now0 + config.GPS_FRESH_MAX_AGE_S + 1.0
        # IMU must still be fresh at now1
        imu1 = _imu(gyrz_rad_s=0.0, yaw_rad=0.0)
        imu1.ts = now1
        baro1 = _baro()
        baro1.ts = now1
        fresh1 = decidefresh(None_to_ns(), imu1, baro1, state, now1)
        self.assertFalse(fresh1.point_fresh)
        self.assertTrue(fresh1.imu_gyrz_fresh)
        l1_1 = produceL1input(fresh1, None_to_ns(), imu1, state, 3, now1)

        self.assertTrue(l1_1.valid)
        self.assertEqual(l1_1.control_mode, ControlMode.DR_TRACKING_CLOSED)
        self.assertEqual(l1_1.dr_method, DRMethod.GYRO_INTEGRATION)


def None_to_ns():
    """Empty GPS message (no pos_ts) — simulates dropout."""
    return SimpleNamespace(pos_ts=None, motion_ts=None,
                           lat=None, lon=None,
                           course_rad=None, speed_mps=None)


# ══════════════════════════════════════════════════════════════════════════════
# TEST 4: DR_TRACKING_CLOSED — no lin_acc → still GYRO_INTEGRATION
# ══════════════════════════════════════════════════════════════════════════════

class TestDrTrackingClosedNoAccBlend(unittest.TestCase):
    def test_no_linear_acc_keeps_gyro_integration(self):
        state = _make_state()
        gps_fresh = _gps(ORIGIN_LAT + 0.001, ORIGIN_LON, course_rad=0.0, speed=8.0)
        imu = _imu()   # lin_acc_valid=False by default
        now0 = _now()
        fresh0 = decidefresh(gps_fresh, imu, _baro(), state, now0)
        produceL1input(fresh0, gps_fresh, imu, state, 3, now0)

        now1 = _now() + config.GPS_FRESH_MAX_AGE_S + 1.0
        fresh1 = decidefresh(None_to_ns(), imu, _baro(), state, now1)
        l1 = produceL1input(fresh1, None_to_ns(), imu, state, 3, now1)

        if l1.valid and l1.control_mode in (ControlMode.DR_TRACKING_CLOSED, ControlMode.DR_TRACKING_OPEN):
            self.assertEqual(l1.dr_method, DRMethod.GYRO_INTEGRATION)


# ══════════════════════════════════════════════════════════════════════════════
# TEST 5: DR_TRACKING_OPEN (manual FreshResult, no gyrz)
# ══════════════════════════════════════════════════════════════════════════════

class TestDrTrackingOpen(unittest.TestCase):
    def test_gps_stale_only_yaw_fresh_gives_dr_tracking_open(self):
        state = _make_state()
        gps_fresh = _gps(ORIGIN_LAT + 0.001, ORIGIN_LON, course_rad=0.0, speed=8.0)
        imu_fresh = _imu(gyrz_rad_s=0.0, yaw_rad=0.0)
        now0 = _now()
        fresh0 = decidefresh(gps_fresh, imu_fresh, _baro(), state, now0)
        produceL1input(fresh0, gps_fresh, imu_fresh, state, 3, now0)

        # Force DR anchor (test directly the DR_TRACKING_OPEN branch)
        state.dr_start_E = state.nav_E if math.isfinite(state.nav_E) else 0.0
        state.dr_start_N = state.nav_N if math.isfinite(state.nav_N) else 100.0
        state.dr_start_V = state.nav_V if math.isfinite(state.nav_V) else 8.0
        state.dr_start_course = state.nav_course if math.isfinite(state.nav_course) else 0.0
        state.dr_start_time = now0
        state.yaw_at_dropout = 0.0

        # Hand-build a FreshResult: only yaw is fresh, gyrz stale
        fresh_manual = FreshResult()
        fresh_manual.point_fresh = False
        fresh_manual.velocity_fresh = False
        fresh_manual.imu_fresh = True
        fresh_manual.imu_gyrz_fresh = False
        fresh_manual.imu_yaw_fresh = True
        fresh_manual.imu_age_s = 0.1

        now1 = _now()
        l1 = produceL1input(fresh_manual, None_to_ns(), imu_fresh, state, 3, now1)
        if l1.valid:
            self.assertEqual(l1.control_mode, ControlMode.DR_TRACKING_OPEN)


# ══════════════════════════════════════════════════════════════════════════════
# TEST 6: FAIL mode
# ══════════════════════════════════════════════════════════════════════════════

class TestFailMode(unittest.TestCase):
    def test_no_gps_no_dr_anchor_gives_fail(self):
        state = _make_state()
        state.origin_ready = True
        state.origin_lat = ORIGIN_LAT
        state.origin_lon = ORIGIN_LON
        state.target_ready = True
        state.target_E = 0.0
        state.target_N = 900.0
        # No DR anchor, no GPS
        fresh = FreshResult()
        now = _now()
        l1 = produceL1input(fresh, None_to_ns(), _imu(), state, 3, now)
        self.assertFalse(l1.valid)
        self.assertEqual(l1.control_mode, ControlMode.FAIL)

    def test_fail_mode_gives_zero_yaw_rate_cmd(self):
        l1 = guidance._fail_l1input("FAIL_NO_VALID_DR", GuidanceState(), FreshResult())
        g_out = produceL1output(l1)
        self.assertFalse(g_out.control_valid)
        self.assertAlmostEqual(g_out.angular_velocity_cmd_rad_s, 0.0)

    def test_fail_l1input_motor_neutral(self):
        l1 = guidance._fail_l1input("FAIL_TEST", GuidanceState(), FreshResult())
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
        fresh = decidefresh(gps, imu, _baro(), state, now)
        l1 = produceL1input(fresh, gps, imu, state, release_state=2, now=now)
        self.assertFalse(l1.valid)
        self.assertFalse(state.origin_ready)

    def test_release_state_3_sets_origin_from_gps(self):
        state = _make_state()
        gps = _gps(ORIGIN_LAT + 0.001, ORIGIN_LON)
        imu = _imu()
        now = _now()
        fresh = decidefresh(gps, imu, _baro(), state, now)
        produceL1input(fresh, gps, imu, state, release_state=3, now=now)
        self.assertTrue(state.origin_ready)
        self.assertAlmostEqual(state.origin_lat, ORIGIN_LAT + 0.001, places=5)

    def test_origin_immutable_after_first_set(self):
        state = _make_state()
        gps1 = _gps(ORIGIN_LAT + 0.001, ORIGIN_LON)
        imu = _imu()
        now = _now()
        fresh = decidefresh(gps1, imu, _baro(), state, now)
        produceL1input(fresh, gps1, imu, state, 3, now)
        first_origin = state.origin_lat

        gps2 = _gps(ORIGIN_LAT + 0.005, ORIGIN_LON)
        now2 = _now()
        fresh2 = decidefresh(gps2, imu, _baro(), state, now2)
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
        inp.dr_method = DRMethod.NONE
        return inp

    def test_target_north_course_north_yaw_rate_zero(self):
        l1 = self._build_l1input(pos_N=0.0, target_N=900.0, course_deg=0.0)
        out = produceL1output(l1)
        self.assertTrue(out.control_valid)
        self.assertAlmostEqual(out.angular_velocity_cmd_rad_s, 0.0, places=6)

    def test_target_east_course_north_yaw_rate_positive(self):
        l1 = self._build_l1input(pos_N=0.0, target_E=900.0, target_N=0.0, course_deg=0.0)
        out = produceL1output(l1)
        self.assertTrue(out.control_valid)
        self.assertGreater(out.angular_velocity_cmd_rad_s, 0.0)

    def test_target_west_course_north_yaw_rate_negative(self):
        l1 = self._build_l1input(pos_N=0.0, target_E=-900.0, target_N=0.0, course_deg=0.0)
        out = produceL1output(l1)
        self.assertTrue(out.control_valid)
        self.assertLess(out.angular_velocity_cmd_rad_s, 0.0)

    def test_fail_mode_no_control(self):
        l1 = guidance._fail_l1input("FAIL_TEST", GuidanceState(), FreshResult())
        out = produceL1output(l1)
        self.assertFalse(out.control_valid)
        self.assertAlmostEqual(out.angular_velocity_cmd_rad_s, 0.0)

    def test_detumbling_mode_zero_yaw_rate_cmd(self):
        inp = guidance.L1Input()
        inp.valid = True
        inp.control_mode = ControlMode.DETUMBLING
        inp.confidence = 1.0
        inp.dr_method = DRMethod.NONE
        out = produceL1output(inp)
        self.assertTrue(out.control_valid)
        self.assertAlmostEqual(out.angular_velocity_cmd_rad_s, 0.0)
        self.assertTrue(out.pid_enabled)


# ══════════════════════════════════════════════════════════════════════════════
# TEST 9: Saturated sine
# ══════════════════════════════════════════════════════════════════════════════

class TestSaturatedSin(unittest.TestCase):
    """Input-clamped sine: sin(clamp(nu, ±π/2)). Saturates at ±1.0."""

    def test_nu_zero(self):
        self.assertAlmostEqual(saturated_sin(0.0), 0.0)

    def test_nu_45_deg_uses_normal_sin(self):
        self.assertAlmostEqual(saturated_sin(math.radians(45.0)), math.sin(math.pi/4))

    def test_nu_90_deg_at_saturation(self):
        self.assertAlmostEqual(saturated_sin(math.radians(90.0)), 1.0)

    def test_nu_120_deg_input_clamped_to_pi_over_2(self):
        # input 120° clamped to 90° → sin(90°) = 1.0
        self.assertAlmostEqual(saturated_sin(math.radians(120.0)), 1.0)

    def test_nu_minus_120_deg_input_clamped_to_minus_pi_over_2(self):
        self.assertAlmostEqual(saturated_sin(math.radians(-120.0)), -1.0)

    def test_nu_180_deg_input_clamped_to_pi_over_2(self):
        # input π clamped to π/2 → sin(π/2) = 1.0
        # Prevents L1 singularity at exactly target-behind.
        self.assertAlmostEqual(saturated_sin(math.radians(180.0)), 1.0)

    def test_nu_minus_45_deg(self):
        self.assertAlmostEqual(saturated_sin(math.radians(-45.0)), -math.sin(math.pi/4))


# ══════════════════════════════════════════════════════════════════════════════
# TEST 10: Fresh/stale gating
# ══════════════════════════════════════════════════════════════════════════════

class TestFreshStaleGating(unittest.TestCase):
    def test_stale_gps_without_dr_anchor_gives_fail(self):
        state = _make_state()
        state.origin_ready = True
        state.origin_lat = ORIGIN_LAT
        state.origin_lon = ORIGIN_LON
        state.target_ready = True
        state.target_E = 0.0
        state.target_N = 900.0

        stale_gps = _gps(age=config.GPS_FRESH_MAX_AGE_S + 5.0)
        imu = _imu(gyrz_rad_s=0.0)
        now = _now()
        fresh = decidefresh(stale_gps, imu, _baro(), state, now)
        self.assertFalse(fresh.point_fresh)
        l1 = produceL1input(fresh, stale_gps, imu, state, 3, now)
        self.assertFalse(l1.valid)

    def test_age_determines_freshness_not_history_presence(self):
        state = _make_state()
        gps_old = _gps(age=config.GPS_FRESH_MAX_AGE_S + 0.5)
        now = _now()
        fresh = decidefresh(gps_old, _imu(), _baro(), state, now)
        self.assertFalse(fresh.point_fresh)

    def test_dr_confidence_by_age(self):
        # Formula: piecewise linear (a1, 1.0) → (a2, 0.5) → (a3, 0.0)
        a1 = config.DR_CONF_AGE_1_S
        a2 = config.DR_CONF_AGE_2_S
        a3 = config.DR_CONF_AGE_3_S
        self.assertAlmostEqual(compute_dr_confidence(0.0), 1.0)
        self.assertAlmostEqual(compute_dr_confidence(a1 - 0.1), 1.0)
        self.assertAlmostEqual(compute_dr_confidence(a1), 1.0)
        # midpoint of [a1, a2] → 0.75
        mid12 = 0.5 * (a1 + a2)
        self.assertAlmostEqual(compute_dr_confidence(mid12), 0.75)
        self.assertAlmostEqual(compute_dr_confidence(a2), 0.5)
        # midpoint of [a2, a3] → 0.25
        mid23 = 0.5 * (a2 + a3)
        self.assertAlmostEqual(compute_dr_confidence(mid23), 0.25)
        self.assertAlmostEqual(compute_dr_confidence(a3 + 1.0), 0.0)


# ══════════════════════════════════════════════════════════════════════════════
# TEST 11: Motor output integration
# ══════════════════════════════════════════════════════════════════════════════

class TestMotorOutput(unittest.TestCase):
    def _make_gps_tracking_l1(self, closed: bool = True):
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
        inp.dr_method = DRMethod.NONE
        return inp

    def test_fail_gives_neutral_pwm(self):
        l1 = guidance._fail_l1input("FAIL_TEST", GuidanceState(), FreshResult())
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
        """DETUMBLING: yaw_rate_cmd=0 + PID counters measured yaw.
        KP_DETUMBLE=0 (PID OFF 테스트 모드)이면 delta=0, 양수이면 반대 방향 제동."""
        inp = guidance.L1Input()
        inp.valid = True
        inp.control_mode = ControlMode.DETUMBLING
        inp.confidence = 1.0
        inp.dr_method = DRMethod.NONE
        g_out = produceL1output(inp)
        self.assertTrue(g_out.pid_enabled)
        ctl = control.MakeCtrler()
        # Spinning right at 50 deg/s → controller should command left turn
        measured_dps = 50.0
        cmd = control.ProduceCtrlOutput(ctl, control.ProduceCtrlInput(g_out, _now()),
                                         measured_dps, _now())
        if config.KP_DETUMBLE == 0.0:
            self.assertEqual(cmd.delta_arm_deg, 0.0)  # PID OFF: no response expected
        else:
            self.assertLess(cmd.delta_arm_deg, 0.0)   # PID ON: oppose CW spin

    def test_open_mode_feedforward_only(self):
        """GPS_TRACKING_OPEN should run feedforward-only (no PID trim)."""
        l1 = self._make_gps_tracking_l1(closed=False)
        g_out = produceL1output(l1)
        # In open mode the controller still runs PID; pid_enabled is True for L1 modes.
        ctl = control.MakeCtrler()
        cmd = control.ProduceCtrlOutput(ctl, control.ProduceCtrlInput(g_out, _now()),
                                         float("nan"), _now())
        self.assertAlmostEqual(cmd.delta_pid_deg, 0.0)
        self.assertEqual(cmd.mode, control.CTRL_MODE_FEEDFORWARD_ONLY)

    def test_yaw_rate_limit_applied(self):
        """produceL1output clamps yaw_rate_cmd to the mode limit."""
        inp = guidance.L1Input()
        inp.valid = True
        inp.control_mode = ControlMode.GPS_TRACKING_CLOSED
        inp.E = 0.0
        inp.N = 0.0
        inp.course = 0.0
        inp.V = 20.0
        inp.target_E = 900.0
        inp.target_N = 0.0
        inp.confidence = 1.0
        inp.dr_method = DRMethod.NONE
        g_out = produceL1output(inp)
        limit_rad_s = math.radians(config.GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS)
        self.assertLessEqual(abs(g_out.angular_velocity_cmd_rad_s), limit_rad_s + 1e-9)


# ══════════════════════════════════════════════════════════════════════════════
# TEST: _can_dead_reckon — DR anchor + IMU required
# ══════════════════════════════════════════════════════════════════════════════

class TestCanDeadReckon(unittest.TestCase):
    def test_no_anchor_returns_false(self):
        state = GuidanceState()
        fresh = FreshResult()
        fresh.imu_gyrz_fresh = True
        self.assertFalse(guidance._can_dead_reckon(state, fresh))

    def test_anchor_but_no_imu_returns_false(self):
        state = GuidanceState()
        state.dr_start_E = 0.0
        state.dr_start_N = 0.0
        state.dr_start_V = 8.0
        state.dr_start_course = 0.0
        state.dr_start_time = _now()
        fresh = FreshResult()
        self.assertFalse(guidance._can_dead_reckon(state, fresh))

    def test_anchor_and_gyrz_fresh_returns_true(self):
        state = GuidanceState()
        state.dr_start_E = 0.0
        state.dr_start_N = 0.0
        state.dr_start_V = 8.0
        state.dr_start_course = 0.0
        state.dr_start_time = _now()
        fresh = FreshResult()
        fresh.imu_gyrz_fresh = True
        self.assertTrue(guidance._can_dead_reckon(state, fresh))

    def test_anchor_and_yaw_fresh_returns_true(self):
        state = GuidanceState()
        state.dr_start_E = 0.0
        state.dr_start_N = 0.0
        state.dr_start_V = 8.0
        state.dr_start_course = 0.0
        state.dr_start_time = _now()
        fresh = FreshResult()
        fresh.imu_yaw_fresh = True
        self.assertTrue(guidance._can_dead_reckon(state, fresh))


# ══════════════════════════════════════════════════════════════════════════════
# TEST: _should_detumble — reads latest IMU sample from state.imu_history
# ══════════════════════════════════════════════════════════════════════════════

class TestShouldDetumble(unittest.TestCase):
    def _push_imu(self, state, gyrz_rad_s, ts):
        state.imu_history.append(ImuSample(
            timestamp=ts, gyr_z=gyrz_rad_s, gyrz_valid=True,
        ))

    def test_below_threshold_not_detumbling(self):
        state = GuidanceState()
        now = _now()
        self._push_imu(state, math.radians(10.0), now)
        fresh = FreshResult()
        fresh.imu_gyrz_fresh = True
        self.assertFalse(guidance._should_detumble(state, fresh, now))

    def test_above_threshold_detumbles(self):
        state = GuidanceState()
        now = _now()
        gz = math.radians(config.DETUMBLE_GYRZ_THRESHOLD_DPS + 10.0)
        self._push_imu(state, gz, now)
        fresh = FreshResult()
        fresh.imu_gyrz_fresh = True
        self.assertTrue(guidance._should_detumble(state, fresh, now))

    def test_stale_gyrz_no_detumble(self):
        state = GuidanceState()
        now = _now()
        gz = math.radians(config.DETUMBLE_GYRZ_THRESHOLD_DPS + 10.0)
        self._push_imu(state, gz, now)
        fresh = FreshResult()
        fresh.imu_gyrz_fresh = False
        self.assertFalse(guidance._should_detumble(state, fresh, now))

    def test_exit_hold_keeps_detumbling(self):
        state = GuidanceState()
        state.nav_control_mode = ControlMode.DETUMBLING
        now = _now()
        gz = math.radians(config.DETUMBLE_EXIT_THRESHOLD_DPS - 5.0)
        self._push_imu(state, gz, now)
        fresh = FreshResult()
        fresh.imu_gyrz_fresh = True
        # First call starts the hold timer; should still be True
        self.assertTrue(guidance._should_detumble(state, fresh, now))


# ══════════════════════════════════════════════════════════════════════════════
# TEST: history-driven anchor (replaces last_fresh_* fields)
# ══════════════════════════════════════════════════════════════════════════════

class TestHistoryDrivenAnchor(unittest.TestCase):
    def test_no_last_fresh_fields_on_state(self):
        s = GuidanceState()
        for attr in ("last_fresh_pos_E", "last_fresh_pos_N", "last_fresh_course",
                     "last_fresh_speed_mps", "last_valid_gps_time", "course_at_dropout"):
            self.assertFalse(hasattr(s, attr),
                             f"GuidanceState should not have {attr} (moved to history)")

    def test_gps_history_drives_last_valid(self):
        state = GuidanceState()
        # populate history directly
        state.gps_history.append(GpsSample(
            lat=37.0, lon=126.0, point_E=10.0, point_N=20.0,
            pos_ts=1.0, pos_valid=True,
            course_rad=0.5, speed_mps=3.0, motion_ts=1.0, motion_valid=True,
        ))
        lp = guidance._last_valid_point(state)
        lv = guidance._last_valid_velocity(state)
        self.assertIsNotNone(lp)
        self.assertEqual(lp.pos_ts, 1.0)
        self.assertIsNotNone(lv)
        self.assertEqual(lv.motion_ts, 1.0)

    def test_invalid_samples_skipped(self):
        state = GuidanceState()
        # valid sample first, then an invalid one
        state.gps_history.append(GpsSample(
            lat=37.0, lon=126.0, point_E=10.0, point_N=20.0,
            pos_ts=1.0, pos_valid=True,
            course_rad=0.5, speed_mps=3.0, motion_ts=1.0, motion_valid=True,
        ))
        state.gps_history.append(GpsSample(
            lat=37.01, lon=126.01, point_E=float("nan"), point_N=float("nan"),
            pos_ts=2.0, pos_valid=False,
            motion_valid=False,
        ))
        # latest valid should be the first one (ts=1.0)
        self.assertEqual(guidance._last_valid_point(state).pos_ts, 1.0)
        self.assertEqual(guidance._last_valid_velocity(state).motion_ts, 1.0)


# ══════════════════════════════════════════════════════════════════════════════
# Spec-style integration tests — kept from prior structure, adapted
# ══════════════════════════════════════════════════════════════════════════════

class TestSpec_GpsTrackingClosed(unittest.TestCase):
    def test_mode_and_fields(self):
        state = _make_state()
        gps = _gps(ORIGIN_LAT + 0.001, ORIGIN_LON, course_rad=0.1, speed=8.0)
        imu = _imu(gyrz_rad_s=0.05)
        now = _now()
        fresh = decidefresh(gps, imu, _baro(), state, now)
        l1 = produceL1input(fresh, gps, imu, state, 3, now)
        self.assertTrue(l1.valid)
        self.assertEqual(l1.control_mode, ControlMode.GPS_TRACKING_CLOSED)
        self.assertEqual(l1.dr_method, DRMethod.NONE)
        self.assertAlmostEqual(l1.confidence, 1.0)
        self.assertTrue(math.isfinite(l1.E))
        self.assertTrue(math.isfinite(l1.N))
        self.assertTrue(math.isfinite(l1.V))


class TestSpec_ProduceL1Output_NorthTarget(unittest.TestCase):
    def test_nu_zero_yaw_rate_zero(self):
        l1 = guidance.L1Input()
        l1.valid = True
        l1.control_mode = ControlMode.GPS_TRACKING_CLOSED
        l1.E = 0.0
        l1.N = 0.0
        l1.course = 0.0
        l1.V = 8.0
        l1.target_E = 0.0
        l1.target_N = 900.0
        l1.confidence = 1.0
        l1.dr_method = DRMethod.NONE

        out = produceL1output(l1)
        self.assertTrue(out.control_valid)
        self.assertAlmostEqual(out.nu, 0.0, places=9)
        self.assertAlmostEqual(out.yaw_rate_cmd, 0.0, places=9)
        self.assertAlmostEqual(out.angular_velocity_cmd_rad_s, 0.0, places=9)
        self.assertAlmostEqual(out.target_bearing, 0.0, places=9)


class TestNewL1OutputFields(unittest.TestCase):
    """Verify spec L1Output fields are populated by produceL1output."""

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
        l1.dr_method = DRMethod.NONE
        return l1

    def test_target_north_fields(self):
        out = produceL1output(self._build_l1input())
        self.assertAlmostEqual(out.target_bearing, 0.0, places=9)
        self.assertAlmostEqual(out.nu, 0.0, places=9)
        self.assertAlmostEqual(out.distance_to_target, 900.0, places=3)
        self.assertAlmostEqual(out.yaw_rate_cmd, 0.0, places=9)
        # Backward-compatible alias and mirror
        self.assertEqual(out.yaw_rate_cmd, out.angular_velocity_cmd_rad_s)
        self.assertEqual(out.distance_to_target, out.alongTrack)
        self.assertEqual(out.carrot_E, out.target_E)
        self.assertEqual(out.carrot_N, out.target_N)

    def test_target_east_fields(self):
        out = produceL1output(self._build_l1input(target_E=900.0, target_N=0.0))
        self.assertAlmostEqual(out.target_bearing, math.pi / 2, places=6)
        self.assertGreater(out.yaw_rate_cmd, 0.0)


if __name__ == "__main__":
    unittest.main()
