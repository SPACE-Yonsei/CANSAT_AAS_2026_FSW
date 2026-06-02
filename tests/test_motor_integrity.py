"""Integrity & data-flow tests across guidance / control / motorapp.

Covers gaps not addressed by other test files:
  - motorapp helpers: _send_diag, _measured_yaw_rate_dps,
    _sync_origin_to_prevstate, _sleep_for_period, _manual_steer_command
  - handlers: handle_fac, handle_cmc, handle_release/egg, dispatch terminate
  - guidance: lat/lon round-trip, _circular_mean, target projection edges,
    reset_guidance_state_for_flight full reset, DR acc-blend branch
  - end-to-end: GPS → DR → recover one full cycle
"""

from __future__ import annotations

import math
import threading
import time
import unittest
from types import SimpleNamespace
from unittest import mock

from lib import config
from Sensor_Motor import control, guidance, motorapp
from Sensor_Motor.motorapp import _Cache


# ════════════════════════════════════════════════════════════════════════════
# Fixtures
# ════════════════════════════════════════════════════════════════════════════

ORIGIN_LAT = 37.55
ORIGIN_LON = 126.95
TARGET_LAT = ORIGIN_LAT + 900.0 / 111_000.0   # ~900 m north


def _reset_motorapp():
    motorapp.MOTORAPP_RUNSTATUS = True
    motorapp.MOTOR_ENABLED = True
    motorapp.STATE = 0
    motorapp._PREV_STATE = -1
    motorapp._ORIGIN_SAVED = False
    motorapp.MANUAL_STEER_MODE = config.MOTOR_MANUAL_NEUTRAL
    motorapp.RELEASE_ACTION_ENABLED = True
    motorapp.EGG_ACTION_ENABLED = True
    motorapp.PI = None
    control.reset()
    motorapp._CACHE = _Cache()
    motorapp._GUIDANCE_STATE = guidance.GuidanceState()
    motorapp._TARGET_LAT = None
    motorapp._TARGET_LON = None
    motorapp._START_LAT = None
    motorapp._START_LON = None


def _gps_ns(lat=ORIGIN_LAT, lon=ORIGIN_LON, course_rad=0.0, speed=8.0,
            pos_ts=None, motion_ts=None):
    now = time.monotonic()
    return SimpleNamespace(
        lat=lat, lon=lon,
        course_rad=course_rad, speed_mps=speed,
        pos_ts=pos_ts if pos_ts is not None else now,
        motion_ts=motion_ts if motion_ts is not None else now,
    )


def _imu_ns(gyrz_rad_s=0.0, yaw_rad=0.0, ts=None,
            lin_acc_valid=False, lin_acc_x=0.0, lin_acc_y=0.0):
    return SimpleNamespace(
        roll_rad=0.0, pitch_rad=0.0, yaw_rad=yaw_rad,
        accx_mps2=0.0, accy_mps2=0.0, accz_mps2=-9.81,
        gyrx_rad_s=0.0, gyry_rad_s=0.0, gyrz_rad_s=gyrz_rad_s,
        magx_uT=0.0, magy_uT=0.0, magz_uT=0.0,
        ts=ts if ts is not None else time.monotonic(),
        lin_acc_x=lin_acc_x, lin_acc_y=lin_acc_y, lin_acc_z=0.0,
        lin_acc_valid=lin_acc_valid,
        freefall=0, tumble=0,
    )


def _baro_ns(alt_m=150.0, ts=None):
    return SimpleNamespace(alt_m=alt_m, ts=ts if ts is not None else time.monotonic())


# ════════════════════════════════════════════════════════════════════════════
# motorapp helpers
# ════════════════════════════════════════════════════════════════════════════

class TestMeasuredYawRateDps(unittest.TestCase):
    """_measured_yaw_rate_dps gating logic."""

    def setUp(self):
        _reset_motorapp()

    def test_pid_disabled_returns_nan(self):
        g_out = guidance.L1Output(pid_enabled=False)
        fresh = guidance.FreshResult(imu_gyrz_fresh=True)
        imu = _imu_ns(gyrz_rad_s=0.5)
        self.assertTrue(math.isnan(motorapp._measured_yaw_rate_dps(g_out, fresh, imu)))

    def test_gyrz_stale_returns_nan(self):
        g_out = guidance.L1Output(pid_enabled=True)
        fresh = guidance.FreshResult(imu_gyrz_fresh=False)
        imu = _imu_ns(gyrz_rad_s=0.5)
        self.assertTrue(math.isnan(motorapp._measured_yaw_rate_dps(g_out, fresh, imu)))

    def test_none_imu_returns_nan(self):
        g_out = guidance.L1Output(pid_enabled=True)
        fresh = guidance.FreshResult(imu_gyrz_fresh=True)
        self.assertTrue(math.isnan(motorapp._measured_yaw_rate_dps(g_out, fresh, None)))

    def test_gyrz_none_returns_nan(self):
        g_out = guidance.L1Output(pid_enabled=True)
        fresh = guidance.FreshResult(imu_gyrz_fresh=True)
        imu = motorapp._ImuFromApp()   # gyrz_rad_s=None by default
        self.assertTrue(math.isnan(motorapp._measured_yaw_rate_dps(g_out, fresh, imu)))

    def test_gyrz_nan_returns_nan(self):
        g_out = guidance.L1Output(pid_enabled=True)
        fresh = guidance.FreshResult(imu_gyrz_fresh=True)
        imu = _imu_ns(gyrz_rad_s=float("nan"))
        self.assertTrue(math.isnan(motorapp._measured_yaw_rate_dps(g_out, fresh, imu)))

    def test_valid_path_returns_deg_with_sign(self):
        g_out = guidance.L1Output(pid_enabled=True)
        fresh = guidance.FreshResult(imu_gyrz_fresh=True)
        imu = _imu_ns(gyrz_rad_s=math.radians(30.0))
        # config.GYRZ_SIGN should be 1.0 in normal config
        result = motorapp._measured_yaw_rate_dps(g_out, fresh, imu)
        self.assertAlmostEqual(result, 30.0 * config.GYRZ_SIGN, places=4)


class TestSyncOriginToPrevstate(unittest.TestCase):
    def setUp(self):
        _reset_motorapp()

    def test_origin_not_ready_returns_false(self):
        with mock.patch.object(motorapp.prevstate, "update_start_point") as m:
            self.assertFalse(motorapp._sync_origin_to_prevstate())
            m.assert_not_called()

    def test_origin_ready_syncs_cache_and_prevstate(self):
        motorapp._GUIDANCE_STATE.origin_ready = True
        motorapp._GUIDANCE_STATE.origin_lat = 37.55
        motorapp._GUIDANCE_STATE.origin_lon = 126.95
        with mock.patch.object(motorapp.prevstate, "update_start_point") as m:
            self.assertTrue(motorapp._sync_origin_to_prevstate())
            m.assert_called_once_with(37.55, 126.95, True)
        self.assertAlmostEqual(motorapp._START_LAT, 37.55)
        self.assertAlmostEqual(motorapp._START_LON, 126.95)


class TestSleepForPeriod(unittest.TestCase):
    def test_short_cycle_sleeps_remainder(self):
        period = 0.05
        cycle_start = time.monotonic()
        # immediately call _sleep_for_period — should sleep ~period
        t0 = time.monotonic()
        motorapp._sleep_for_period(cycle_start, period)
        elapsed = time.monotonic() - t0
        # allow generous tolerance for CI scheduling
        self.assertGreaterEqual(elapsed, period * 0.5)
        self.assertLess(elapsed, period * 3.0)

    def test_long_cycle_does_not_sleep(self):
        period = 0.05
        cycle_start = time.monotonic() - 1.0   # already over period
        t0 = time.monotonic()
        motorapp._sleep_for_period(cycle_start, period)
        elapsed = time.monotonic() - t0
        self.assertLess(elapsed, 0.005)


class TestManualSteerCommandHelper(unittest.TestCase):
    def setUp(self):
        _reset_motorapp()

    def test_left_negative_delta(self):
        cmd = motorapp._manual_steer_command(123.4, config.MOTOR_MANUAL_LEFT)
        self.assertLess(cmd.delta_arm_deg, 0.0)
        self.assertGreater(cmd.left_angle_deg, control.NEUTRAL_ARM_DEG)
        self.assertLess(cmd.right_angle_deg, control.NEUTRAL_ARM_DEG)
        self.assertTrue(cmd.valid)
        self.assertEqual(cmd.mode, "MANUAL_LEFT")
        self.assertEqual(cmd.fallback_mode, "MANUAL_LEFT")
        self.assertAlmostEqual(cmd.angular_velocity_cmd_deg_s, 0.0)

    def test_right_positive_delta(self):
        cmd = motorapp._manual_steer_command(123.4, config.MOTOR_MANUAL_RIGHT)
        self.assertGreater(cmd.delta_arm_deg, 0.0)
        self.assertLess(cmd.left_angle_deg, control.NEUTRAL_ARM_DEG)
        self.assertGreater(cmd.right_angle_deg, control.NEUTRAL_ARM_DEG)
        self.assertTrue(cmd.valid)

    def test_unknown_mode_returns_neutral_safe(self):
        cmd = motorapp._manual_steer_command(0.0, "BOGUS")
        self.assertEqual(cmd.left_pw, control.LEFT_NEUTRAL)
        self.assertEqual(cmd.right_pw, control.RIGHT_NEUTRAL)


class TestSendDiagPayload(unittest.TestCase):
    """_send_diag is the primary telemetry channel — verify schema/format."""

    def setUp(self):
        _reset_motorapp()

    def _fake_queue(self):
        sent = []
        class _Q:
            def put_nowait(self, m): sent.append(m)
            def put(self, m, *a, **kw): sent.append(m)
        return _Q(), sent

    def test_skips_when_queue_is_none(self):
        snap = motorapp._cache_snapshot()
        g_out = guidance.L1Output()
        cmd   = control.WriteNeutral(123.0)
        # Should not raise
        motorapp._send_diag(None, cmd, g_out, "TEST", snap)

    def test_payload_field_count_and_format(self):
        snap = motorapp._cache_snapshot()
        motorapp._TARGET_LAT = 37.55
        motorapp._TARGET_LON = 126.95
        motorapp._START_LAT  = 37.54
        motorapp._START_LON  = 126.94
        g_out = guidance.L1Output(
            current_heading_rad=math.radians(45.0),
            crossTrack=0.0, alongTrack=900.0,
        )
        cmd = control.CtrlOutput(timestamp=123.0)
        cmd.left_pw = 1500; cmd.right_pw = 1600
        captured = []
        def _grab(queue, src, dst, mid, payload):
            captured.append(payload)
        with mock.patch.object(motorapp.msgstructure, "send_msg", side_effect=_grab):
            motorapp._send_diag(object(), cmd, g_out, "TEST_STATE", snap)
        self.assertEqual(len(captured), 1)
        fields = captured[0].split(",")
        # Verify schema: 29 fixed fields
        self.assertEqual(len(fields), 29)
        # Spot-check key fields
        self.assertEqual(fields[0], "1500")        # left_pw
        self.assertEqual(fields[1], "1600")        # right_pw
        self.assertEqual(fields[4], "37.550000")   # target_lat
        self.assertEqual(fields[5], "126.950000")  # target_lon
        self.assertEqual(fields[9], "TEST_STATE")  # diag_state
        self.assertEqual(fields[-1], "NEUTRAL")    # mode (default)

    def test_nan_heading_emits_nan_string(self):
        snap = motorapp._cache_snapshot()
        g_out = guidance.L1Output()   # current_heading_rad=nan
        cmd = control.WriteNeutral(0.0)
        captured = []
        with mock.patch.object(motorapp.msgstructure, "send_msg",
                                side_effect=lambda *a, **k: captured.append(a[-1])):
            motorapp._send_diag(object(), cmd, g_out, "X", snap)
        self.assertIn("nan", captured[0].split(",")[8])


# ════════════════════════════════════════════════════════════════════════════
# Handlers
# ════════════════════════════════════════════════════════════════════════════

class TestHandleFac(unittest.TestCase):
    def setUp(self):
        _reset_motorapp()

    def test_all_off(self):
        motorapp.handle_fac("OFF")
        self.assertFalse(motorapp.RELEASE_ACTION_ENABLED)
        self.assertFalse(motorapp.EGG_ACTION_ENABLED)

    def test_all_on(self):
        motorapp.RELEASE_ACTION_ENABLED = False
        motorapp.EGG_ACTION_ENABLED = False
        motorapp.handle_fac("ON")
        self.assertTrue(motorapp.RELEASE_ACTION_ENABLED)
        self.assertTrue(motorapp.EGG_ACTION_ENABLED)

    def test_rel_only(self):
        motorapp.handle_fac("REL,OFF")
        self.assertFalse(motorapp.RELEASE_ACTION_ENABLED)
        self.assertTrue(motorapp.EGG_ACTION_ENABLED)

    def test_egg_only(self):
        motorapp.handle_fac("EGG,OFF")
        self.assertTrue(motorapp.RELEASE_ACTION_ENABLED)
        self.assertFalse(motorapp.EGG_ACTION_ENABLED)

    def test_garbage_no_change(self):
        motorapp.handle_fac("FOO,BAR,BAZ")
        self.assertTrue(motorapp.RELEASE_ACTION_ENABLED)
        self.assertTrue(motorapp.EGG_ACTION_ENABLED)

    def test_lowercase_and_spaces(self):
        motorapp.handle_fac(" rel , off ")
        self.assertFalse(motorapp.RELEASE_ACTION_ENABLED)


class TestHandleCmc(unittest.TestCase):
    def setUp(self):
        _reset_motorapp()

    def test_valid_modes_no_crash(self):
        for m in (config.MOTOR_CTRL_MODE_GPS_GUIDED,
                  config.MOTOR_CTRL_MODE_GPS_ONLY,
                  config.MOTOR_CTRL_MODE_IMU_HEADING):
            motorapp.handle_cmc(m)   # should not raise

    def test_unknown_mode_no_crash(self):
        motorapp.handle_cmc("BOGUS")

    def test_empty_no_crash(self):
        motorapp.handle_cmc("")
        motorapp.handle_cmc(None)


class TestHandleReleaseEggGating(unittest.TestCase):
    def setUp(self):
        _reset_motorapp()

    def test_release_disabled_skips_thread(self):
        motorapp.RELEASE_ACTION_ENABLED = False
        with mock.patch.object(threading, "Thread") as m:
            motorapp.handle_release("X")
            m.assert_not_called()

    def test_egg_disabled_skips_thread(self):
        motorapp.EGG_ACTION_ENABLED = False
        with mock.patch.object(threading, "Thread") as m:
            motorapp.handle_egg_drop()
            m.assert_not_called()


class TestDispatch(unittest.TestCase):
    def setUp(self):
        _reset_motorapp()

    def test_terminate_message_stops_runstatus(self):
        from lib import appargs, msgstructure
        m = msgstructure.MsgStructure(
            sender_app   = appargs.MainAppArg.AppID,
            receiver_app = appargs.MotorAppArg.AppID,
            msg_id       = appargs.MainAppArg.MID_TerminateProcess,
            data         = "",
        )
        motorapp.dispatch(msgstructure.pack_msg(m))
        self.assertFalse(motorapp.MOTORAPP_RUNSTATUS)

    def test_bad_message_no_crash(self):
        motorapp.dispatch("not_a_real_message")


class TestHandleTargetCoordZero(unittest.TestCase):
    """Verify (0,0) sentinel is rejected by handle_target_coord (matches init)."""

    def setUp(self):
        _reset_motorapp()

    def test_zero_zero_rejected(self):
        motorapp.handle_target_coord("0,0")
        self.assertIsNone(motorapp._CACHE.target_lat)
        self.assertIsNone(motorapp._CACHE.target_lon)

    def test_near_zero_rejected(self):
        motorapp.handle_target_coord("0.0,0.0")
        self.assertIsNone(motorapp._CACHE.target_lat)


# ════════════════════════════════════════════════════════════════════════════
# guidance — utility & reset coverage
# ════════════════════════════════════════════════════════════════════════════

class TestLatLonRoundTrip(unittest.TestCase):
    def test_round_trip_close(self):
        lat0, lon0 = 37.55, 126.95
        for dlat_m, dlon_m in [(0, 0), (100, 0), (0, 100), (500, 500),
                                (-300, 200), (-1000, -800)]:
            # Approximate degrees per meter
            lat = lat0 + dlat_m / 111_320.0
            lon = lon0 + dlon_m / (111_320.0 * math.cos(math.radians(lat0)))
            N, E = guidance.latlon_to_ne(lat, lon, lat0, lon0)
            lat2, lon2 = guidance.ne_to_latlon(N, E, lat0, lon0)
            self.assertAlmostEqual(lat, lat2, places=8)
            self.assertAlmostEqual(lon, lon2, places=8)

    def test_convert_local_en_returns_E_first(self):
        # 1 degree north at equator ~ 111 km
        E, N = guidance.convert_latlon_to_local_en(38.0, 127.0, 37.0, 127.0)
        self.assertAlmostEqual(E, 0.0, places=3)
        self.assertGreater(N, 100_000.0)


class TestWrapPi(unittest.TestCase):
    def test_zero(self):
        self.assertAlmostEqual(guidance.wrap_pi(0.0), 0.0)

    def test_positive_pi(self):
        self.assertAlmostEqual(guidance.wrap_pi(math.pi), -math.pi, places=9)

    def test_above_2pi(self):
        self.assertAlmostEqual(guidance.wrap_pi(3 * math.pi), -math.pi, places=9)

    def test_negative_3pi(self):
        self.assertAlmostEqual(guidance.wrap_pi(-3 * math.pi), -math.pi, places=9)


class TestCircularMean(unittest.TestCase):
    def test_same_angle(self):
        self.assertAlmostEqual(guidance._circular_mean(0.5, 0.5), 0.5, places=9)

    def test_opposite_angles_wraps(self):
        # mean of 170° and -170° should be ±180°, NOT 0°
        m = guidance._circular_mean(math.radians(170), math.radians(-170))
        self.assertAlmostEqual(abs(m), math.pi, places=6)

    def test_acute_pair(self):
        m = guidance._circular_mean(math.radians(10), math.radians(30))
        self.assertAlmostEqual(math.degrees(m), 20.0, places=4)


class TestConvertTargetEdges(unittest.TestCase):
    def test_origin_not_ready_skips(self):
        s = guidance.GuidanceState()
        s.target_lat = 37.55; s.target_lon = 126.95
        guidance.convert_target_to_local_en_if_possible(s)
        self.assertFalse(s.target_ready)

    def test_target_already_ready_skips(self):
        s = guidance.GuidanceState()
        s.origin_ready = True
        s.origin_lat = 37.55; s.origin_lon = 126.95
        s.target_ready = True
        s.target_E = 999.0; s.target_N = 999.0
        guidance.convert_target_to_local_en_if_possible(s)
        self.assertEqual(s.target_E, 999.0)   # unchanged

    def test_zero_zero_target_rejected(self):
        s = guidance.GuidanceState()
        s.origin_ready = True
        s.origin_lat = 37.55; s.origin_lon = 126.95
        s.target_lat = 0.0; s.target_lon = 0.0
        guidance.convert_target_to_local_en_if_possible(s)
        self.assertFalse(s.target_ready)


class TestResetGuidanceStateFullReset(unittest.TestCase):
    """reset_guidance_state_for_flight must clear ALL flight state."""

    def test_target_en_and_ready_reset(self):
        s = guidance.GuidanceState()
        s.target_lat = 37.55; s.target_lon = 126.95
        s.target_E = 100.0;   s.target_N = 200.0
        s.target_ready = True
        s.origin_lat = 37.55; s.origin_lon = 126.95
        s.origin_ready = True
        guidance.reset_guidance_state_for_flight(s)
        self.assertFalse(s.target_ready)
        self.assertTrue(math.isnan(s.target_E))
        self.assertTrue(math.isnan(s.target_N))
        # target_lat/lon preserved
        self.assertAlmostEqual(s.target_lat, 37.55)
        self.assertAlmostEqual(s.target_lon, 126.95)

    def test_all_dr_state_cleared(self):
        s = guidance.GuidanceState()
        s.dr_start_E = 1.0; s.dr_start_N = 2.0
        s.dr_start_V = 3.0; s.dr_start_course = 0.5; s.dr_start_time = 100.0
        s.yaw_at_dropout = 0.7; s.gyro_integral_since_dropout = 0.9
        s.last_dr_update_time = 100.5; s.detumble_exit_start = 99.0
        guidance.reset_guidance_state_for_flight(s)
        for attr in ("dr_start_E", "dr_start_N", "dr_start_V", "dr_start_course",
                     "dr_start_time", "yaw_at_dropout",
                     "last_dr_update_time", "detumble_exit_start"):
            self.assertTrue(math.isnan(getattr(s, attr)), f"{attr} not nan")
        self.assertEqual(s.gyro_integral_since_dropout, 0.0)

    def test_histories_cleared(self):
        s = guidance.GuidanceState()
        s.gps_history.append(guidance.GpsSample(pos_ts=1.0))
        s.imu_history.append(guidance.ImuSample(timestamp=1.0))
        s.baro_history.append(guidance.BarometerSample(timestamp=1.0))
        guidance.reset_guidance_state_for_flight(s)
        self.assertEqual(s.gps_history, [])
        self.assertEqual(s.imu_history, [])
        self.assertEqual(s.baro_history, [])

    def test_nav_state_cleared(self):
        s = guidance.GuidanceState()
        s.nav_E = 5.0; s.nav_N = 6.0; s.nav_V = 8.0; s.nav_course = 0.1
        s.nav_vE = 0.1; s.nav_vN = 0.2
        s.nav_confidence = 0.7; s.nav_dr_age = 3.0
        s.nav_control_mode = guidance.ControlMode.GPS_TRACKING_CLOSED
        s.nav_dr_method = guidance.DRMethod.GYRO_INTEGRATION
        guidance.reset_guidance_state_for_flight(s)
        self.assertTrue(all(math.isnan(getattr(s, a))
                            for a in ("nav_E", "nav_N", "nav_V", "nav_course",
                                       "nav_vE", "nav_vN")))
        self.assertEqual(s.nav_confidence, 0.0)
        self.assertEqual(s.nav_dr_age, 0.0)
        self.assertEqual(s.nav_control_mode, guidance.ControlMode.FAIL)
        self.assertEqual(s.nav_dr_method, guidance.DRMethod.NONE)


# ════════════════════════════════════════════════════════════════════════════
# DR acc-blend branch
# ════════════════════════════════════════════════════════════════════════════

class TestDeadReckoningAccBlend(unittest.TestCase):
    def _bootstrap_gps_anchor(self, state):
        now = time.monotonic()
        gps = _gps_ns()
        imu = _imu_ns(gyrz_rad_s=0.0, yaw_rad=0.0)
        fresh = guidance.decidefresh(gps, imu, _baro_ns(), state, now)
        guidance.produceL1input(fresh, gps, imu, state, 3, now)
        return now

    def test_gyro_acc_blend_method_selected_when_lin_acc_fresh(self):
        s = guidance.GuidanceState()
        s.target_lat = TARGET_LAT; s.target_lon = ORIGIN_LON
        now0 = self._bootstrap_gps_anchor(s)
        # GPS dropout: jump forward, IMU fresh with linear acc
        now1 = now0 + config.GPS_FRESH_MAX_AGE_S + 0.5
        imu = _imu_ns(gyrz_rad_s=0.0, yaw_rad=0.0,
                      lin_acc_valid=True, lin_acc_x=0.1, lin_acc_y=0.05)
        imu.ts = now1
        # Empty GPS to force DR
        empty_gps = SimpleNamespace(pos_ts=None, motion_ts=None,
                                     lat=None, lon=None,
                                     course_rad=None, speed_mps=None)
        fresh = guidance.decidefresh(empty_gps, imu, _baro_ns(ts=now1), s, now1)
        l1 = guidance.produceL1input(fresh, empty_gps, imu, s, 3, now1)
        if l1.valid and l1.control_mode == guidance.ControlMode.DR_TRACKING_CLOSED:
            self.assertEqual(l1.dr_method, guidance.DRMethod.GYRO_ACC_BLEND)

    def test_acc_above_limit_falls_back_to_gyro_only(self):
        s = guidance.GuidanceState()
        s.target_lat = TARGET_LAT; s.target_lon = ORIGIN_LON
        now0 = self._bootstrap_gps_anchor(s)
        now1 = now0 + config.GPS_FRESH_MAX_AGE_S + 0.5
        # Acc magnitude well above ACC_LIMIT_MPS2
        imu = _imu_ns(gyrz_rad_s=0.0, yaw_rad=0.0,
                      lin_acc_valid=True, lin_acc_x=10.0, lin_acc_y=10.0)
        imu.ts = now1
        empty_gps = SimpleNamespace(pos_ts=None, motion_ts=None,
                                     lat=None, lon=None,
                                     course_rad=None, speed_mps=None)
        fresh = guidance.decidefresh(empty_gps, imu, _baro_ns(ts=now1), s, now1)
        l1 = guidance.produceL1input(fresh, empty_gps, imu, s, 3, now1)
        if l1.valid:
            self.assertEqual(l1.dr_method, guidance.DRMethod.GYRO_INTEGRATION)


# ════════════════════════════════════════════════════════════════════════════
# End-to-end one-cycle pipeline (no IPC)
# ════════════════════════════════════════════════════════════════════════════

class TestEndToEndPipeline(unittest.TestCase):
    """Push sensor data via handlers, then invoke the guidance + control chain
    the way ctrl_parafoil does. Verify full data flow without spinning the loop."""

    def setUp(self):
        _reset_motorapp()
        motorapp.STATE = 3

    def _push_sensors(self, ts):
        motorapp.handle_gps(f"{ORIGIN_LAT},{ORIGIN_LON},{ts:.4f},0.0,8.0,{ts:.4f}")
        motorapp.handle_imu(
            f"0,0,0,0,0,-9.81,0,0,0,0,0,2.5,{ts:.4f},0,0,1"
        )
        motorapp.handle_barometer(f"150.0,{ts:.4f},nan")
        motorapp.handle_target_coord(f"{TARGET_LAT},{ORIGIN_LON}")

    def test_one_full_cycle_yields_valid_pwm(self):
        ts = time.monotonic()
        self._push_sensors(ts)
        snap = motorapp._cache_snapshot()
        now = time.monotonic()
        fresh = guidance.decidefresh(snap.latest_gps, snap.latest_imu,
                                      snap.latest_baro, motorapp._GUIDANCE_STATE, now)
        self.assertTrue(fresh.point_fresh)
        self.assertTrue(fresh.velocity_fresh)
        self.assertTrue(fresh.imu_fresh)
        l1_in = guidance.produceL1input(fresh, snap.latest_gps, snap.latest_imu,
                                         motorapp._GUIDANCE_STATE, motorapp.STATE, now)
        self.assertTrue(l1_in.valid)
        self.assertEqual(l1_in.control_mode, guidance.ControlMode.GPS_TRACKING_CLOSED)
        # Origin and target should now be set
        self.assertTrue(motorapp._GUIDANCE_STATE.origin_ready)
        self.assertTrue(motorapp._GUIDANCE_STATE.target_ready)

        g_out = guidance.produceL1output(l1_in)
        g_out.timestamp = now
        self.assertTrue(g_out.control_valid)

        measured = motorapp._measured_yaw_rate_dps(g_out, fresh, snap.latest_imu)
        self.assertFalse(math.isnan(measured))   # PID-enabled + gyrz fresh
        ctrl_in = control.ProduceCtrlInput(g_out, now)
        cmd = control.step(ctrl_in, measured, now)
        self.assertTrue(cmd.valid)
        self.assertEqual(cmd.mode, control.CTRL_MODE_CLOSED_LOOP)
        self.assertGreaterEqual(cmd.left_pw,  control.LEFT_MIN_PULSE)
        self.assertLessEqual(   cmd.left_pw,  control.LEFT_MAX_PULSE)
        self.assertGreaterEqual(cmd.right_pw, control.RIGHT_MIN_PULSE)
        self.assertLessEqual(   cmd.right_pw, control.RIGHT_MAX_PULSE)

    def test_gps_dropout_transitions_to_dr(self):
        ts0 = time.monotonic()
        self._push_sensors(ts0)
        snap = motorapp._cache_snapshot()
        now0 = time.monotonic()
        fresh0 = guidance.decidefresh(snap.latest_gps, snap.latest_imu,
                                       snap.latest_baro, motorapp._GUIDANCE_STATE, now0)
        l0 = guidance.produceL1input(fresh0, snap.latest_gps, snap.latest_imu,
                                      motorapp._GUIDANCE_STATE, 3, now0)
        self.assertEqual(l0.control_mode, guidance.ControlMode.GPS_TRACKING_CLOSED)

        # Jump time past GPS freshness; refresh IMU only
        now1 = now0 + config.GPS_FRESH_MAX_AGE_S + 1.0
        motorapp.handle_imu(f"0,0,0,0,0,-9.81,0,0,0,0,0,2.5,{now1:.4f},0,0,1")
        snap1 = motorapp._cache_snapshot()
        fresh1 = guidance.decidefresh(snap1.latest_gps, snap1.latest_imu,
                                       snap1.latest_baro, motorapp._GUIDANCE_STATE, now1)
        self.assertFalse(fresh1.point_fresh)
        self.assertTrue(fresh1.imu_gyrz_fresh)
        l1 = guidance.produceL1input(fresh1, snap1.latest_gps, snap1.latest_imu,
                                      motorapp._GUIDANCE_STATE, 3, now1)
        self.assertEqual(l1.control_mode, guidance.ControlMode.DR_TRACKING_CLOSED)
        # confidence should have decayed (dr_age ~ 3s)
        self.assertLess(l1.confidence, 1.0)


# ════════════════════════════════════════════════════════════════════════════
# init() origin restore behaviour
# ════════════════════════════════════════════════════════════════════════════

class TestInitOriginRestore(unittest.TestCase):
    """init() should mark _ORIGIN_SAVED=True when restoring origin from prevstate."""

    def setUp(self):
        _reset_motorapp()

    def test_restore_sets_origin_saved_flag(self):
        with mock.patch.object(motorapp.prevstate, "init_prevstate"), \
             mock.patch.object(motorapp.prevstate, "is_motor_enabled", return_value=True), \
             mock.patch.object(motorapp.prevstate, "get_target_gps", return_value=(0.0, 0.0)), \
             mock.patch.object(motorapp.prevstate, "get_start_point",
                                return_value=(37.55, 126.95)), \
             mock.patch.object(motorapp.control, "init_control", return_value=None):
            motorapp.init()
        self.assertTrue(motorapp._ORIGIN_SAVED)
        self.assertTrue(motorapp._GUIDANCE_STATE.origin_ready)
        self.assertAlmostEqual(motorapp._GUIDANCE_STATE.origin_lat, 37.55)

    def test_no_restore_keeps_flag_false(self):
        with mock.patch.object(motorapp.prevstate, "init_prevstate"), \
             mock.patch.object(motorapp.prevstate, "is_motor_enabled", return_value=True), \
             mock.patch.object(motorapp.prevstate, "get_target_gps", return_value=(0.0, 0.0)), \
             mock.patch.object(motorapp.prevstate, "get_start_point", return_value=None), \
             mock.patch.object(motorapp.control, "init_control", return_value=None):
            motorapp.init()
        self.assertFalse(motorapp._ORIGIN_SAVED)
        self.assertFalse(motorapp._GUIDANCE_STATE.origin_ready)


if __name__ == "__main__":
    unittest.main()
