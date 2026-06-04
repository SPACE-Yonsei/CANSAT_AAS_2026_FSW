"""Origin-lock policy and DR safety guards (guidance.py / motorapp.py).

Origin policy: no candidate. The first valid GPS fix received at STATE >= 3
(DESCENT) is locked as origin; later fixes never overwrite it.
"""
import math
import unittest
from math import radians
from unittest.mock import patch

from Sensor_Motor import guidance, motorapp
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
    guidance.reset()
    origin_lat = 37.5
    origin_lon = 127.0
    target_lon = origin_lon + math.degrees(
        100.0 / (guidance.EARTH_RADIUS_M * math.cos(math.radians(origin_lat)))
    )
    guidance.set_origin_point(origin_lat, origin_lon)
    guidance.set_target_point(origin_lat, target_lon)


def _origin_ready() -> bool:
    mi = guidance._MISSION_t
    return math.isfinite(mi.origin_lat) and math.isfinite(mi.origin_lon)


class TestDescentOriginLock(unittest.TestCase):
    """DESCENT(state>=3) 진입 후 처음 들어오는 유효 GPS가 무조건 origin이 된다."""

    def setUp(self):
        guidance.reset()
        motorapp.STATE = 0
        motorapp._ORIGIN_LOCKED = False

    def _send(self, lat, lon, pos=1, course=90.0, speed=5.0, ts=None):
        ts = NOW if ts is None else ts
        # 8 fields: lat,lon,pos_health,pos_ts,course,speed,motion_health,motion_ts
        motorapp.handle_gps(f"{lat},{lon},{pos},{ts:.4f},{course},{speed},1,{ts:.4f}")

    @patch.object(motorapp.prevstate, "update_start_point")
    def test_no_lock_below_state3(self, mock_update):
        motorapp.STATE = 2          # APOGEE: 아직 origin 안 잡음
        self._send(37.5, 127.0)
        self.assertFalse(_origin_ready())
        self.assertFalse(motorapp._ORIGIN_LOCKED)
        mock_update.assert_not_called()

    @patch.object(motorapp.prevstate, "update_start_point")
    def test_first_valid_fix_at_descent_locks(self, mock_update):
        motorapp.STATE = 3          # DESCENT 첫 유효 좌표
        self._send(38.86, -104.79)   # US site (any coordinate works)
        mi = guidance._MISSION_t
        self.assertTrue(_origin_ready())
        self.assertTrue(motorapp._ORIGIN_LOCKED)
        self.assertAlmostEqual(mi.origin_lat, 38.86)
        self.assertAlmostEqual(mi.origin_lon, -104.79)
        mock_update.assert_called_once()

    @patch.object(motorapp.prevstate, "update_start_point")
    def test_first_fix_wins_no_overwrite(self, mock_update):
        motorapp.STATE = 3
        self._send(38.86, -104.79)
        self._send(40.00, -105.00)   # later fix must be ignored
        mi = guidance._MISSION_t
        self.assertAlmostEqual(mi.origin_lat, 38.86)
        self.assertAlmostEqual(mi.origin_lon, -104.79)
        self.assertEqual(mock_update.call_count, 1)

    @patch.object(motorapp.prevstate, "update_start_point")
    def test_origin_persists_into_release(self, mock_update):
        motorapp.STATE = 3
        self._send(38.86, -104.79)   # DESCENT에서 lock
        motorapp.STATE = 4           # PAYLOAD_RELEASE 진입: 덮어쓰지 않음
        self._send(40.00, -105.00)
        self.assertAlmostEqual(guidance._MISSION_t.origin_lat, 38.86)

    @patch.object(motorapp.prevstate, "update_start_point")
    def test_invalid_fix_does_not_lock(self, mock_update):
        motorapp.STATE = 3
        self._send(37.5, 127.0, pos=0)   # pos_health=0 → not a valid fix
        self.assertFalse(_origin_ready())
        self.assertFalse(motorapp._ORIGIN_LOCKED)
        mock_update.assert_not_called()

    @patch.object(motorapp.prevstate, "update_start_point")
    def test_relock_after_reset(self, mock_update):
        motorapp.STATE = 3
        self._send(38.86, -104.79)
        self.assertTrue(_origin_ready())
        guidance.reset()                 # state<4 전이 시 motorapp이 호출
        motorapp._ORIGIN_LOCKED = False
        self.assertFalse(_origin_ready())
        self._send(40.00, -105.00)       # 재진입 후 첫 좌표로 재잠금
        self.assertAlmostEqual(guidance._MISSION_t.origin_lat, 40.00)


class TestTargetLock(unittest.TestCase):
    def setUp(self):
        guidance.reset()
        guidance._MISSION_t.target_lat = math.nan
        guidance._MISSION_t.target_lon = math.nan
        motorapp._TARGET_LOCKED = False

    @patch.object(motorapp.prevstate, "FIX_TARGET_GPS", False)
    def test_first_valid_target_wins_no_overwrite(self):
        motorapp.handle_target_coord("37.500000,127.000000")
        motorapp.handle_target_coord("38.000000,128.000000")
        mi = guidance._MISSION_t
        self.assertTrue(motorapp._TARGET_LOCKED)
        self.assertAlmostEqual(mi.target_lat, 37.5)
        self.assertAlmostEqual(mi.target_lon, 127.0)

    @patch.object(motorapp.prevstate, "FIX_TARGET_GPS", False)
    def test_invalid_target_does_not_lock(self):
        motorapp.handle_target_coord("0,0")
        self.assertFalse(motorapp._TARGET_LOCKED)
        self.assertFalse(math.isfinite(guidance._MISSION_t.target_lat))


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
