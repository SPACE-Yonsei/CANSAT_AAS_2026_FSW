"""ProduceL1Input: GPS_TRACKING / DR_M_* / DR_PM_* L1Input generation.

Covers the cases enumerated in the ProduceL1Input spec (A-H): GPS tracking,
motion-only DR (P from GPS, M estimated), full DR (P+M estimated), and the
defensive NO_SPEED_SOURCE / NO_COURSE_SOURCE rejections.
"""
import math
import unittest
from math import isfinite, nan, radians

from Sensor_Motor import guidance
from Sensor_Motor.sensor_types import _BaroFromApp, _GpsFromApp, _ImuFromApp
from lib import config


NOW = 100.0
# baro sink=3.0 → 추정 수평속도 = sink * gain, V_MAX_DR로 clamp (gain 튜닝에 무관하게)
SINK_V = min(3.0 * config.DR_SINK_TO_HSPEED_GAIN, config.V_MAX_DR_MPS)


def _prepare_mission():
    guidance.reset()
    origin_lat = 37.0
    origin_lon = 127.0
    target_lon = origin_lon + math.degrees(
        100.0 / (guidance.EARTH_RADIUS_M * math.cos(math.radians(origin_lat)))
    )
    guidance.set_origin_point(origin_lat, origin_lon)
    guidance.set_target_point(origin_lat, target_lon)


def _gps(pos=True, motion=True, age=0.0):
    ts = NOW - age
    return _GpsFromApp(
        lat=37.00001,
        lon=127.00001,
        course_rad=math.radians(90.0),
        speed_mps=5.0,
        pos_ts=ts,
        motion_ts=ts,
        pos_health=1 if pos else 0,
        motion_health=1 if motion else 0,
    )


def _imu(gyrz=False, yaw=False, acc=False, age=0.0):
    return _ImuFromApp(
        yaw_rad=math.radians(90.0) if yaw else None,
        gyrz_rad_s=math.radians(3.0) if gyrz else None,
        ts=NOW - age,
        lin_acc_x=0.2 if acc else None,
        lin_acc_y=0.0 if acc else None,
        lin_acc_valid=acc,
        health=1 if (gyrz or yaw or acc) else 0,
    )


def _baro(sink=False, age=0.0):
    return _BaroFromApp(
        alt_m=120.0,
        sink_rate=3.0 if sink else None,
        rx_ts=NOW - age,
        health=1 if sink else 0,
    )


def _seed_dr():
    guidance.dr_lock(
        guidance._STATE_t.dr,
        E=10.0,
        N=20.0,
        V=4.0,
        course=math.radians(80.0),
        yaw=math.radians(80.0),
        now=NOW - 0.1,
    )


class TestProduceL1Input(unittest.TestCase):
    def setUp(self):
        _prepare_mission()

    def _decide(self, gps=None, imu=None, baro=None):
        return guidance.DecideControlMode(gps, imu, baro, NOW)

    # ── Case A: GPS tracking ────────────────────────────────────────────────
    def test_case_a_gps_tracking_valid(self):
        mode = self._decide(_gps(True, True), _imu(gyrz=True), _baro())
        self.assertEqual(mode, guidance.ControlMode.GPS_TRACKING_CLOSED)
        l1 = guidance.ProduceL1Input(NOW)
        self.assertTrue(l1.valid)
        self.assertEqual(l1.control_mode, guidance.ControlMode.GPS_TRACKING_CLOSED)
        self.assertEqual(l1.reason, "GPS_NAV")
        self.assertEqual(l1.confidence, 1.0)
        self.assertTrue(isfinite(l1.E) and isfinite(l1.N))
        self.assertTrue(isfinite(l1.vE) and isfinite(l1.vN))
        self.assertAlmostEqual(l1.V, 5.0, places=6)

    # ── Case B: DR_M_GB_CLOSED — P=GPS, course=gyro, V=baro ──────────────────
    def test_case_b_dr_m_gb_closed(self):
        _seed_dr()
        mode = self._decide(_gps(True, motion=False), _imu(gyrz=True), _baro(sink=True))
        self.assertEqual(mode, guidance.ControlMode.DR_M_GB_CLOSED)
        l1 = guidance.ProduceL1Input(NOW)
        self.assertTrue(l1.valid, l1.reason)
        self.assertEqual(l1.E, guidance._STATE_t.gps.E)
        self.assertEqual(l1.N, guidance._STATE_t.gps.N)
        self.assertAlmostEqual(math.degrees(l1.course), 80.0, delta=2.0)
        self.assertAlmostEqual(l1.V, SINK_V, delta=0.2)
        self.assertEqual(l1.dr_method, guidance.DRMethod.GYRO_INTEGRATION)

    # ── Case C: DR_M_YB_OPEN — P=GPS, course=yaw, V=baro ─────────────────────
    def test_case_c_dr_m_yb_open(self):
        mode = self._decide(_gps(True, motion=False), _imu(yaw=True), _baro(sink=True))
        self.assertEqual(mode, guidance.ControlMode.DR_M_YB_OPEN)
        l1 = guidance.ProduceL1Input(NOW)
        self.assertTrue(l1.valid, l1.reason)
        self.assertEqual(l1.E, guidance._STATE_t.gps.E)
        self.assertAlmostEqual(math.degrees(l1.course), 90.0, delta=2.0)
        self.assertAlmostEqual(l1.V, SINK_V, delta=0.2)

    # ── Case D: DR_PM_GBA_CLOSED — propagate P, acc-blended ──────────────────
    def test_case_d_dr_pm_gba_closed(self):
        _seed_dr()
        mode = self._decide(None, _imu(gyrz=True, acc=True), _baro(sink=True))
        self.assertEqual(mode, guidance.ControlMode.DR_PM_GBA_CLOSED)
        l1 = guidance.ProduceL1Input(NOW)
        self.assertTrue(l1.valid, l1.reason)
        # position was propagated from the seeded current (E=10, N=20)
        self.assertNotEqual(l1.E, 10.0)
        self.assertEqual(l1.dr_method, guidance.DRMethod.GYRO_ACC_BLEND)

    # ── Case E: DR_PM_G_CLOSED — speed from last velocity ────────────────────
    def test_case_e_dr_pm_g_closed_last_v(self):
        _seed_dr()
        mode = self._decide(None, _imu(gyrz=True), _baro())
        self.assertEqual(mode, guidance.ControlMode.DR_PM_G_CLOSED)
        l1 = guidance.ProduceL1Input(NOW)
        self.assertTrue(l1.valid, l1.reason)
        self.assertAlmostEqual(l1.V, 4.0, delta=0.1)

    # ── Case F: DR_PM_Y_OPEN — speed from last velocity ──────────────────────
    def test_case_f_dr_pm_y_open_last_v(self):
        _seed_dr()
        mode = self._decide(None, _imu(yaw=True), _baro())
        self.assertEqual(mode, guidance.ControlMode.DR_PM_Y_OPEN)
        l1 = guidance.ProduceL1Input(NOW)
        self.assertTrue(l1.valid, l1.reason)
        self.assertAlmostEqual(l1.V, 4.0, delta=0.1)

    # ── Case G: no speed source → invalid NO_SPEED_SOURCE ────────────────────
    def test_case_g_no_speed_source(self):
        # establish flags (gyrz fresh, baro stale, gps stale); mode falls to FAIL
        self._decide(None, _imu(gyrz=True), _baro())
        dr = guidance._STATE_t.dr
        dr.current_E = 10.0
        dr.current_N = 20.0
        dr.current_course = radians(80.0)
        dr.current_time = NOW - 0.1
        dr.current_V = nan
        dr.anchor_V = nan
        dr.anchor_course = radians(80.0)
        dr.anchor_time = NOW - 0.1
        dr.yaw_at_anchor = nan
        dr.last_step_time = NOW - 0.1
        dr.gyro_integral = 0.0
        guidance._STATE_t.nav.control_mode = guidance.ControlMode.DR_PM_G_CLOSED
        l1 = guidance.ProduceL1Input(NOW)
        self.assertFalse(l1.valid)
        self.assertEqual(l1.reason, "NO_SPEED_SOURCE")

    # ── Case H: yaw stale in a Y mode → invalid NO_COURSE_SOURCE ─────────────
    def test_case_h_no_course_source(self):
        _seed_dr()
        # yaw absent → imu_yaw_fresh False; natural mode would be DR_PM_GB
        self._decide(None, _imu(gyrz=True), _baro(sink=True))
        guidance._STATE_t.nav.control_mode = guidance.ControlMode.DR_PM_YB_OPEN
        l1 = guidance.ProduceL1Input(NOW)
        self.assertFalse(l1.valid)
        self.assertEqual(l1.reason, "NO_COURSE_SOURCE")

    # ── Guard: FAIL short-circuit ────────────────────────────────────────────
    def test_fail_mode_invalid(self):
        guidance._STATE_t.nav.control_mode = guidance.ControlMode.FAIL
        l1 = guidance.ProduceL1Input(NOW)
        self.assertFalse(l1.valid)
        self.assertEqual(l1.reason, "FAIL")

if __name__ == "__main__":
    unittest.main()
