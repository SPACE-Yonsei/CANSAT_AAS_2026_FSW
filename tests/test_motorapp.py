"""Unit tests for motorapp: handler parsing, cache updates, state transitions, snapshot."""

import math
import time
import unittest

from Sensor_Motor import motorapp
from Sensor_Motor.motorapp import _Cache


def _reset():
    motorapp.MOTORAPP_RUNSTATUS = True
    motorapp.MOTOR_ENABLED = True
    motorapp.STATE = 0
    motorapp._PREV_STATE = -1
    motorapp._START_POINT_LOCKED = False
    motorapp._CONTROLLER = None
    motorapp._L1_STATE = None
    motorapp.PI = None
    motorapp._CACHE = _Cache()


def _gps_msg(lat=37.55, lon=126.95, course=90.0, speed=12.0, pos=True, motion=True, ts=None):
    ts = time.monotonic() if ts is None else ts
    lat_s = f"{lat}" if pos else "nan"
    lon_s = f"{lon}" if pos else "nan"
    course_s = f"{course}" if motion else "nan"
    speed_s = f"{speed}" if motion else "nan"
    motion_ts = f"{ts:.4f}" if motion else "nan"
    return f"{lat_s},{lon_s},{ts:.4f},{course_s},{speed_s},{motion_ts}"


def _imu_msg(gyrz=2.5, health=1, ts=None):
    ts = time.monotonic() if ts is None else ts
    return f"1.0,2.0,45.0,0.1,0.2,0.3,0.4,0.5,0.6,0.0,0.0,{gyrz},{ts:.4f},0,0,{health}"


def _baro_msg(alt=200.5, health=1, sink=1.2, ts=None):
    ts = time.monotonic() if ts is None else ts
    return f"{alt},{ts:.4f},{sink},{health}"


class TestHandleMec(unittest.TestCase):
    def setUp(self):
        _reset()

    def test_mec_off_disables_motor(self):
        motorapp.handle_mec("OFF")
        self.assertFalse(motorapp.MOTOR_ENABLED)

    def test_mec_on_enables_motor(self):
        motorapp.handle_mec("OFF")
        motorapp.handle_mec("ON")
        self.assertTrue(motorapp.MOTOR_ENABLED)

    def test_mec_unknown_no_crash(self):
        motorapp.handle_mec("MAYBE")


class TestHandleGps(unittest.TestCase):
    def setUp(self):
        _reset()

    def test_valid_6_fields_updates_latest_gps(self):
        motorapp.handle_gps(_gps_msg())
        gps = motorapp._CACHE.latest_gps
        self.assertAlmostEqual(gps.lat, 37.55)
        self.assertAlmostEqual(gps.lon, 126.95)
        self.assertAlmostEqual(gps.course_rad, math.radians(90.0))
        self.assertAlmostEqual(gps.speed_mps, 12.0)
        self.assertTrue(gps.pos_health)
        self.assertTrue(gps.motion_health)

    def test_valid_payload_uses_position_ts(self):
        motorapp.handle_gps(_gps_msg(ts=100.5))
        gps = motorapp._CACHE.latest_gps
        self.assertAlmostEqual(gps.pos_ts, 100.5, places=3)

    def test_too_few_fields_no_update(self):
        motorapp.handle_gps("37.55,126.95")
        self.assertIsNone(motorapp._CACHE.latest_gps.lat)

    def test_bad_float_no_update(self):
        motorapp.handle_gps("not_a_float,126.95,90.0,90.0,12.0,90.0")
        self.assertIsNone(motorapp._CACHE.latest_gps.lat)

    def test_fresh_latest_moves_to_history_on_next_gps(self):
        t0 = time.monotonic() - 0.1
        motorapp.handle_gps(_gps_msg(ts=t0))
        self.assertEqual(len(motorapp._CACHE.gps_history), 0)
        motorapp.handle_gps(_gps_msg(lat=37.55001, lon=126.95001, course=91.0, speed=12.1, ts=t0 + 0.1))
        self.assertEqual(len(motorapp._CACHE.gps_history), 1)
        self.assertAlmostEqual(motorapp._CACHE.gps_history[-1].lat, 37.55)
        self.assertTrue(motorapp._CACHE.gps_history[-1].pos_health)

    def test_invalid_position_does_not_enter_history(self):
        t0 = time.monotonic() - 0.1
        motorapp.handle_gps(_gps_msg(lat=37.55, lon=50.0, ts=t0))
        motorapp.handle_gps(_gps_msg(ts=t0 + 0.1))
        self.assertEqual(len(motorapp._CACHE.gps_history), 0)

    def test_unexpected_longitude_disables_position_and_motion(self):
        motorapp.handle_gps(_gps_msg(lat=37.55, lon=50.0))
        gps = motorapp._CACHE.latest_gps
        self.assertFalse(gps.pos_health)
        self.assertFalse(gps.motion_health)

    def test_invalid_motion_stays_out_of_latest_motion_fields(self):
        motorapp.handle_gps(_gps_msg(course=999.0))
        self.assertTrue(motorapp._CACHE.latest_gps.pos_health)
        self.assertFalse(motorapp._CACHE.latest_gps.motion_health)
        self.assertIsNone(motorapp._CACHE.latest_gps.course_rad)

    def test_motion_health_requires_position_health(self):
        motorapp.handle_gps(_gps_msg(lat=37.55, lon=50.0))
        self.assertFalse(motorapp._CACHE.latest_gps.pos_health)
        self.assertFalse(motorapp._CACHE.latest_gps.motion_health)

    def test_history_preserves_freshed_motion(self):
        t0 = time.monotonic() - 0.1
        motorapp.handle_gps(_gps_msg(ts=t0))
        motorapp.handle_gps(_gps_msg(lat=37.55001, lon=126.95001, course=91.0, speed=12.1, ts=t0 + 0.1))
        freshed = motorapp._freshed_gps_from_history(motorapp._CACHE.gps_history)
        self.assertAlmostEqual(freshed.course_rad, math.radians(90.0))
        self.assertAlmostEqual(freshed.speed_mps, 12.0)

    def test_first_gps_stays_latest_not_history(self):
        motorapp.handle_gps(_gps_msg())
        self.assertEqual(len(motorapp._CACHE.gps_history), 0)


class TestHandleGpsStartPointLocking(unittest.TestCase):
    """Tests for the start-point locking path in handle_gps (STATE >= 3)."""

    def setUp(self):
        _reset()

    def test_state_less_than_3_does_not_lock(self):
        motorapp.STATE = 2
        motorapp.handle_gps(_gps_msg())
        self.assertFalse(motorapp._START_POINT_LOCKED)
        self.assertIsNone(motorapp._CACHE.start_lat)

    def test_state3_healthy_gps_locks_start_point(self):
        motorapp.STATE = 3
        motorapp.handle_gps(_gps_msg())
        self.assertTrue(motorapp._START_POINT_LOCKED)
        self.assertAlmostEqual(motorapp._CACHE.start_lat, 37.55)
        self.assertAlmostEqual(motorapp._CACHE.start_lon, 126.95)

    def test_state3_unexpected_longitude_does_not_lock_start_point(self):
        motorapp.STATE = 3
        motorapp.handle_gps(_gps_msg(lat=37.55, lon=50.0))
        self.assertFalse(motorapp._START_POINT_LOCKED)
        self.assertIsNone(motorapp._CACHE.start_lon)


class TestHandleImu(unittest.TestCase):
    def setUp(self):
        _reset()

    def test_valid_payload(self):
        motorapp.handle_imu(_imu_msg())
        imu = motorapp._CACHE.latest_imu
        self.assertAlmostEqual(imu.gyrz_rad_s, math.radians(2.5))
        self.assertTrue(imu.health)

    def test_valid_payload_uses_sample_ts(self):
        motorapp.handle_imu(_imu_msg(ts=99.9))
        self.assertAlmostEqual(motorapp._CACHE.latest_imu.ts, 99.9, places=3)

    def test_bad_data_no_update(self):
        motorapp.handle_imu("bad")
        self.assertIsNone(motorapp._CACHE.latest_imu.gyrz_rad_s)

    def test_too_few_fields_no_update(self):
        motorapp.handle_imu("1.0,2.0,3.0")
        self.assertIsNone(motorapp._CACHE.latest_imu.gyrz_rad_s)

    def test_fresh_latest_moves_to_imu_history_on_next_imu(self):
        t0 = time.monotonic() - 0.1
        motorapp.handle_imu(_imu_msg(gyrz=2.5, ts=t0))
        self.assertEqual(len(motorapp._CACHE.imu_history), 0)
        motorapp.handle_imu(_imu_msg(gyrz=3.0, ts=t0 + 0.05))
        self.assertEqual(len(motorapp._CACHE.imu_history), 1)
        self.assertAlmostEqual(motorapp._CACHE.imu_history[-1].gyrz_rad_s, math.radians(2.5))

    def test_unhealthy_imu_skips_history(self):
        t0 = time.monotonic() - 0.1
        motorapp.handle_imu(_imu_msg(gyrz=2.5, health=0, ts=t0))
        motorapp.handle_imu(_imu_msg(gyrz=3.0, ts=t0 + 0.05))
        self.assertEqual(len(motorapp._CACHE.imu_history), 0)


class TestHandleBarometer(unittest.TestCase):
    def setUp(self):
        _reset()

    def test_valid_payload(self):
        motorapp.handle_barometer(_baro_msg(alt=150.0))
        self.assertAlmostEqual(motorapp._CACHE.latest_baro.alt_m, 150.0)
        self.assertTrue(motorapp._CACHE.latest_baro.health)

    def test_alt_and_health(self):
        motorapp.handle_barometer(_baro_msg(alt=200.5, health=1))
        self.assertAlmostEqual(motorapp._CACHE.latest_baro.alt_m, 200.5)
        self.assertTrue(motorapp._CACHE.latest_baro.health)

    def test_alt_health_and_ts(self):
        motorapp.handle_barometer(_baro_msg(alt=200.5, health=1, ts=77.7))
        self.assertAlmostEqual(motorapp._CACHE.latest_baro.alt_m, 200.5)
        self.assertAlmostEqual(motorapp._CACHE.latest_baro.ts, 77.7, places=3)

    def test_bad_data_no_update(self):
        motorapp.handle_barometer("NOPE")
        self.assertIsNone(motorapp._CACHE.latest_baro.alt_m)

    def test_fresh_latest_moves_to_baro_history_on_next_baro(self):
        t0 = time.monotonic() - 0.1
        motorapp.handle_barometer(_baro_msg(alt=200.5, health=1, ts=t0))
        self.assertEqual(len(motorapp._CACHE.baro_history), 0)
        motorapp.handle_barometer(_baro_msg(alt=201.0, health=1, ts=t0 + 0.1))
        self.assertEqual(len(motorapp._CACHE.baro_history), 1)
        self.assertAlmostEqual(motorapp._CACHE.baro_history[-1].alt_m, 200.5)

    def test_unhealthy_baro_skips_history(self):
        t0 = time.monotonic() - 0.1
        motorapp.handle_barometer(_baro_msg(alt=200.5, health=0, ts=t0))
        motorapp.handle_barometer(_baro_msg(alt=201.0, health=1, ts=t0 + 0.1))
        self.assertEqual(len(motorapp._CACHE.baro_history), 0)


class TestHandleTargetCoord(unittest.TestCase):
    def setUp(self):
        _reset()

    def test_valid_coord_updates_cache(self):
        motorapp.handle_target_coord("37.55,126.95")
        self.assertAlmostEqual(motorapp._CACHE.target_lat, 37.55)
        self.assertAlmostEqual(motorapp._CACHE.target_lon, 126.95)

    def test_out_of_range_lat_no_update(self):
        motorapp.handle_target_coord("999.0,126.95")
        self.assertIsNone(motorapp._CACHE.target_lat)

    def test_out_of_range_lon_no_update(self):
        motorapp.handle_target_coord("37.55,999.0")
        self.assertIsNone(motorapp._CACHE.target_lat)

    def test_bad_float_no_update(self):
        motorapp.handle_target_coord("not,valid")
        self.assertIsNone(motorapp._CACHE.target_lat)

    def test_wrong_field_count_no_update(self):
        motorapp.handle_target_coord("37.55")
        self.assertIsNone(motorapp._CACHE.target_lat)


class TestHandleFlightState(unittest.TestCase):
    def setUp(self):
        _reset()

    def test_state_transition(self):
        motorapp.handle_flight_state("3")
        self.assertEqual(motorapp.STATE, 3)

    def test_same_state_no_prev_update(self):
        motorapp.STATE = 2
        motorapp._PREV_STATE = 1
        motorapp.handle_flight_state("2")
        self.assertEqual(motorapp._PREV_STATE, 1)

    def test_bad_value_no_change(self):
        motorapp.STATE = 2
        motorapp.handle_flight_state("abc")
        self.assertEqual(motorapp.STATE, 2)

    def test_prev_state_recorded(self):
        motorapp.STATE = 2
        motorapp.handle_flight_state("4")
        self.assertEqual(motorapp._PREV_STATE, 2)

    def test_below_3_clears_start_point(self):
        motorapp._CACHE.start_lat = 37.55
        motorapp._CACHE.start_lon = 126.95
        motorapp._START_POINT_LOCKED = True
        motorapp.STATE = 3
        motorapp.handle_flight_state("2")
        self.assertIsNone(motorapp._CACHE.start_lat)
        self.assertFalse(motorapp._START_POINT_LOCKED)

    def test_state3_with_healthy_gps_locks_start_point(self):
        motorapp._CACHE.latest_gps.lat = 37.55
        motorapp._CACHE.latest_gps.lon = 126.95
        motorapp._CACHE.latest_gps.pos_ts = time.monotonic()
        motorapp._CACHE.latest_gps.pos_health = True
        motorapp.handle_flight_state("3")
        self.assertTrue(motorapp._START_POINT_LOCKED)
        self.assertAlmostEqual(motorapp._CACHE.start_lat, 37.55)
        self.assertAlmostEqual(motorapp._CACHE.start_lon, 126.95)

    def test_state3_without_healthy_gps_skips_start_lock(self):
        motorapp._CACHE.latest_gps.lat = None
        motorapp._CACHE.latest_gps.pos_health = False
        motorapp.handle_flight_state("3")
        self.assertFalse(motorapp._START_POINT_LOCKED)


class TestCacheSnapshot(unittest.TestCase):
    def setUp(self):
        _reset()

    def test_snapshot_copies_target(self):
        motorapp._CACHE.target_lat = 37.6
        motorapp._CACHE.target_lon = 127.0
        snap = motorapp._cache_snapshot()
        self.assertAlmostEqual(snap.target_lat, 37.6)
        self.assertAlmostEqual(snap.target_lon, 127.0)

    def test_snapshot_target_none_when_unset(self):
        snap = motorapp._cache_snapshot()
        self.assertIsNone(snap.target_lat)

    def test_snapshot_copies_start_point(self):
        motorapp._CACHE.start_lat = 37.55
        motorapp._CACHE.start_lon = 126.95
        snap = motorapp._cache_snapshot()
        self.assertAlmostEqual(snap.start_lat, 37.55)
        self.assertAlmostEqual(snap.start_lon, 126.95)

    def test_snapshot_copies_latest_gps(self):
        motorapp.handle_gps(_gps_msg())
        snap = motorapp._cache_snapshot()
        self.assertAlmostEqual(snap.latest_gps.lat, 37.55)
        self.assertAlmostEqual(snap.latest_gps.lon, 126.95)

    def test_snapshot_copies_latest_imu(self):
        motorapp.handle_imu(_imu_msg())
        snap = motorapp._cache_snapshot()
        self.assertAlmostEqual(snap.latest_imu.gyrz_rad_s, math.radians(2.5))

    def test_snapshot_copies_latest_baro(self):
        motorapp.handle_barometer(_baro_msg(alt=333.0))
        snap = motorapp._cache_snapshot()
        self.assertAlmostEqual(snap.latest_baro.alt_m, 333.0)

    def test_snapshot_is_independent_copy(self):
        motorapp._CACHE.target_lat = 10.0
        snap = motorapp._cache_snapshot()
        motorapp._CACHE.target_lat = 99.0
        self.assertAlmostEqual(snap.target_lat, 10.0)

    def test_snapshot_copies_history(self):
        t0 = time.monotonic() - 0.1
        motorapp.handle_gps(_gps_msg(ts=t0))
        motorapp.handle_gps(_gps_msg(lat=37.55001, lon=126.95001, ts=t0 + 0.1))
        snap = motorapp._cache_snapshot()
        self.assertEqual(len(snap.gps_history), 1)
        self.assertAlmostEqual(snap.gps_history[-1].lat, 37.55)


if __name__ == "__main__":
    unittest.main()
