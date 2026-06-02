"""Candidate-origin policy and DR safety guards (guidance.py)."""
import math
import unittest
from math import radians

from Sensor_Motor import guidance
from Sensor_Motor.sensor_types import _BaroFromApp, _GpsFromApp, _ImuFromApp
from lib import config

NOW = 1000.0
ControlMode = guidance.ControlMode


def _gps(pos=True, motion=True, age=0.0):
    ts = NOW - age
    return _GpsFromApp(lat=37.5, lon=127.0, course_rad=radians(90.0), speed_mps=5.0,
                       pos_ts=ts, motion_ts=ts,
                       pos_health=1 if pos else 0, motion_health=1 if motion else 0)


def _imu(gyrz_dps=None, yaw=False, acc=False, age=0.0):
    return _ImuFromApp(
        yaw_rad=radians(90.0) if yaw else None,
        gyrz_rad_s=radians(gyrz_dps) if gyrz_dps is not None else None,
        ts=NOW - age, lin_acc_x=0.2 if acc else None, lin_acc_y=0.0 if acc else None,
        lin_acc_valid=acc, health=1)


def _baro(sink=3.0, age=0.0):
    return _BaroFromApp(alt_m=120.0, sink_rate=sink, rx_ts=NOW - age,
                        health=1 if sink is not None else 0)


def _mission():
    guidance.reset(keep_candidate_origin=False)
    mi = guidance._MISSION_t
    mi.origin_lat, mi.origin_lon, mi.origin_ready = 37.5, 127.0, True
    mi.target_E, mi.target_N, mi.target_ready = 100.0, 0.0, True


class TestCandidateOrigin(unittest.TestCase):
    def setUp(self):
        guidance.reset(keep_candidate_origin=False)

    def test_update_candidate_in_area(self):
        self.assertTrue(guidance.update_candidate_origin(37.5, 127.0, ts=100.0))
        self.assertTrue(guidance._MISSION_t.candidate_origin_valid)

    def test_update_candidate_rejects_outside_area(self):
        self.assertFalse(guidance.update_candidate_origin(0.0, 0.0, ts=100.0))
        self.assertFalse(guidance._MISSION_t.candidate_origin_valid)

    def test_pre_release_lock(self):
        guidance.update_candidate_origin(37.5, 127.0, ts=100.0)
        guidance.note_state3_entry(110.0)
        src = guidance.try_lock_origin_from_candidate(120.0)   # age 20 <= 30
        self.assertEqual(src, "PRE_RELEASE_GPS")
        self.assertTrue(guidance._MISSION_t.origin_ready)
        self.assertEqual(guidance._MISSION_t.origin_lock_source, "PRE_RELEASE_GPS")

    def test_release_lock_window(self):
        guidance.note_state3_entry(100.0)
        guidance.update_candidate_origin(37.5, 127.0, ts=101.0)   # within +2s
        self.assertEqual(guidance.try_lock_origin_from_candidate(102.0), "RELEASE_GPS")

    def test_late_lock(self):
        guidance.note_state3_entry(100.0)
        guidance.update_candidate_origin(37.5, 127.0, ts=105.0)   # after +2s
        self.assertEqual(guidance.try_lock_origin_from_candidate(106.0), "LATE_GPS")

    def test_too_old_candidate_rejected(self):
        guidance.note_state3_entry(200.0)
        guidance.update_candidate_origin(37.5, 127.0, ts=100.0)   # age 100 > 30
        self.assertIsNone(guidance.try_lock_origin_from_candidate(200.0))
        self.assertFalse(guidance._MISSION_t.origin_ready)

    def test_no_overwrite_when_ready(self):
        guidance.lock_origin(37.5, 127.0, source="PRE_RELEASE_GPS")
        guidance.update_candidate_origin(37.6, 127.1, ts=999.0)
        self.assertIsNone(guidance.try_lock_origin_from_candidate(1000.0))
        self.assertEqual(guidance._MISSION_t.origin_lat, 37.5)  # unchanged

    def test_candidate_survives_reset(self):
        guidance.update_candidate_origin(37.5, 127.0, ts=100.0)
        guidance.reset()   # default keep_candidate_origin=True
        self.assertTrue(guidance._MISSION_t.candidate_origin_valid)


class TestDRSafetyGuards(unittest.TestCase):
    def setUp(self):
        _mission()

    def _decide(self, gps, imu, baro):
        return guidance.DecideControlMode(gps, imu, baro, NOW)

    def _seed_dr(self, age=0.1, V=4.0):
        guidance.dr_lock(guidance._STATE_t.dr, E=10.0, N=20.0, V=V,
                         course=radians(80.0), yaw=radians(80.0), now=NOW - age)

    def test_dr_timeout_to_fail(self):
        self._seed_dr(age=config.DR_MAX_AGE_S + 10.0)
        mode = self._decide(None, _imu(gyrz_dps=3.0), _baro(sink=3.0))
        self.assertEqual(mode, ControlMode.FAIL)
        l1 = guidance.ProduceL1Input(NOW)
        self.assertFalse(l1.valid)
        self.assertEqual(l1.reason, "DR_TIMEOUT")

    def test_baro_sink_spike_rejected(self):
        self._seed_dr()
        self._decide(None, _imu(gyrz_dps=3.0), _baro(sink=config.DR_BARO_SINK_MAX_MPS + 5.0))
        self.assertFalse(guidance._STATE_t.flags.baro_sink_fresh)
        self.assertTrue(guidance._STATE_t.nav.baro_sink_spike)

    def test_gyro_excessive_forces_open(self):
        # GPS full-fresh, but gyrz way over the guidance limit -> OPEN (no gyro feedback)
        mode = self._decide(_gps(True, True),
                            _imu(gyrz_dps=config.DR_MAX_YAW_RATE_DPS_FOR_CONTROL + 50.0),
                            _baro())
        self.assertEqual(mode, ControlMode.GPS_TRACKING_OPEN)

    def test_speed_clamped_flag(self):
        # DR_PM_G with a huge last velocity -> clamped to V_MAX_DR_MPS, flag set
        self._seed_dr(V=100.0)
        mode = self._decide(None, _imu(gyrz_dps=3.0), _baro(sink=None))
        self.assertTrue(mode.value.startswith("DR_PM_"))
        l1 = guidance.ProduceL1Input(NOW)
        self.assertTrue(l1.valid)
        self.assertLessEqual(l1.V, config.V_MAX_DR_MPS + 1e-9)
        self.assertTrue(guidance._STATE_t.nav.speed_clamped)

    def test_position_jump_rejected(self):
        self._seed_dr()
        self._decide(None, _imu(gyrz_dps=3.0), _baro(sink=3.0))
        saved = config.DR_MAX_POSITION_JUMP_M
        try:
            config.DR_MAX_POSITION_JUMP_M = 0.001   # force any propagation to exceed
            l1 = guidance.ProduceL1Input(NOW + 0.2)
            self.assertFalse(l1.valid)
            self.assertEqual(l1.reason, "DR_POSITION_JUMP")
        finally:
            config.DR_MAX_POSITION_JUMP_M = saved

    def test_low_confidence_zeroes_output(self):
        l1in = guidance.L1Input(valid=True, reason="DR", control_mode=ControlMode.DR_PM_G_CLOSED,
                                confidence=config.DR_MIN_CONFIDENCE_FOR_CONTROL - 0.05,
                                E=0.0, N=0.0, V=5.0, course=0.0, vE=0.0, vN=5.0,
                                target_E=50.0, target_N=86.6)
        out = guidance.ProduceL1Output(l1in)
        self.assertFalse(out.valid)
        self.assertEqual(out.reason, "LOW_DR_CONFIDENCE")
        self.assertEqual(out.yaw_rate_cmd, 0.0)


if __name__ == "__main__":
    unittest.main()
