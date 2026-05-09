"""Unit tests for motorapp: handler parsing, cache updates, state transitions, snapshot."""

import math
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
        motorapp.handle_gps("37.55,126.95,90.0,12.0,1,1")
        gps = motorapp._CACHE.latest_gps
        self.assertAlmostEqual(gps.lat, 37.55)
        self.assertAlmostEqual(gps.lon, 126.95)
        self.assertAlmostEqual(gps.course_rad, math.radians(90.0))
        self.assertAlmostEqual(gps.speed_mps, 12.0)
        self.assertTrue(gps.pos_health)
        self.assertTrue(gps.motion_health)

    def test_valid_7_fields_uses_sample_ts(self):
        motorapp.handle_gps("37.55,126.95,90.0,12.0,1,1,100.5")
        gps = motorapp._CACHE.latest_gps
        self.assertAlmostEqual(gps.sample_ts, 100.5)

    def test_too_few_fields_no_update(self):
        motorapp.handle_gps("37.55,126.95")
        self.assertIsNone(motorapp._CACHE.latest_gps.lat)

    def test_bad_float_no_update(self):
        motorapp.handle_gps("not_a_float,126.95,90.0,12.0,1,1")
        self.assertIsNone(motorapp._CACHE.latest_gps.lat)

    def test_healthy_pos_updates_last_gps(self):
        motorapp.handle_gps("37.55,126.95,90.0,12.0,1,1")
        self.assertAlmostEqual(motorapp._CACHE.last_gps.lat, 37.55)
        self.assertTrue(motorapp._CACHE.last_gps.pos_health)

    def test_unhealthy_pos_skips_last_gps_position(self):
        motorapp.handle_gps("37.55,126.95,90.0,12.0,0,0")
        self.assertIsNone(motorapp._CACHE.last_gps.lat)

    def test_healthy_motion_updates_last_gps_motion(self):
        motorapp.handle_gps("37.55,126.95,90.0,12.0,1,1")
        self.assertAlmostEqual(motorapp._CACHE.last_gps.course_rad, math.radians(90.0))
        self.assertAlmostEqual(motorapp._CACHE.last_gps.speed_mps, 12.0)

    def test_gps_appended_to_history(self):
        motorapp.handle_gps("37.55,126.95,90.0,12.0,1,1")
        self.assertEqual(len(motorapp._CACHE.gps_history), 1)


class TestHandleGpsStartPointLocking(unittest.TestCase):
    """Tests for the start-point locking path in handle_gps (STATE >= 3)."""

    def setUp(self):
        _reset()

    def test_state_less_than_3_does_not_lock(self):
        motorapp.STATE = 2
        motorapp.handle_gps("37.55,126.95,90.0,12.0,1,1")
        self.assertFalse(motorapp._START_POINT_LOCKED)
        self.assertIsNone(motorapp._CACHE.start_lat)

    def test_state3_healthy_gps_locks_start_point(self):
        motorapp.STATE = 3
        motorapp.handle_gps("37.55,126.95,90.0,12.0,1,1")
        self.assertTrue(motorapp._START_POINT_LOCKED)
        self.assertAlmostEqual(motorapp._CACHE.start_lat, 37.55)
        self.assertAlmostEqual(motorapp._CACHE.start_lon, 126.95)


class TestHandleImu(unittest.TestCase):
    def setUp(self):
        _reset()

    def test_valid_13_fields(self):
        motorapp.handle_imu("1.0,2.0,45.0,0.1,0.2,0.3,0.4,0.5,0.6,0.0,0.0,2.5,1")
        imu = motorapp._CACHE.latest_imu
        self.assertAlmostEqual(imu.gyrz_rad_s, math.radians(2.5))
        self.assertTrue(imu.health)

    def test_valid_14_fields_uses_sample_ts(self):
        motorapp.handle_imu("1.0,2.0,45.0,0.1,0.2,0.3,0.4,0.5,0.6,0.0,0.0,2.5,1,99.9")
        self.assertAlmostEqual(motorapp._CACHE.latest_imu.sample_ts, 99.9)

    def test_bad_data_no_update(self):
        motorapp.handle_imu("bad")
        self.assertIsNone(motorapp._CACHE.latest_imu.gyrz_rad_s)

    def test_too_few_fields_no_update(self):
        motorapp.handle_imu("1.0,2.0,3.0")
        self.assertIsNone(motorapp._CACHE.latest_imu.gyrz_rad_s)

    def test_healthy_imu_updates_last_imu(self):
        motorapp.handle_imu("1.0,2.0,45.0,0.1,0.2,0.3,0.4,0.5,0.6,0.0,0.0,2.5,1")
        self.assertAlmostEqual(motorapp._CACHE.last_imu.gyrz_rad_s, math.radians(2.5))

    def test_unhealthy_imu_skips_last_imu(self):
        motorapp.handle_imu("1.0,2.0,45.0,0.1,0.2,0.3,0.4,0.5,0.6,0.0,0.0,2.5,0")
        self.assertIsNone(motorapp._CACHE.last_imu.gyrz_rad_s)


class TestHandleBarometer(unittest.TestCase):
    def setUp(self):
        _reset()

    def test_alt_only_field(self):
        motorapp.handle_barometer("150.0")
        self.assertAlmostEqual(motorapp._CACHE.latest_baro.alt_m, 150.0)
        self.assertTrue(motorapp._CACHE.latest_baro.health)

    def test_alt_and_health(self):
        motorapp.handle_barometer("200.5,1")
        self.assertAlmostEqual(motorapp._CACHE.latest_baro.alt_m, 200.5)
        self.assertTrue(motorapp._CACHE.latest_baro.health)

    def test_alt_health_and_ts(self):
        motorapp.handle_barometer("200.5,1,77.7")
        self.assertAlmostEqual(motorapp._CACHE.latest_baro.alt_m, 200.5)
        self.assertAlmostEqual(motorapp._CACHE.latest_baro.sample_ts, 77.7)

    def test_bad_data_no_update(self):
        motorapp.handle_barometer("NOPE")
        self.assertIsNone(motorapp._CACHE.latest_baro.alt_m)

    def test_healthy_baro_updates_last_baro(self):
        motorapp.handle_barometer("200.5,1")
        self.assertAlmostEqual(motorapp._CACHE.last_baro.alt_m, 200.5)

    def test_unhealthy_baro_skips_last_baro(self):
        motorapp.handle_barometer("200.5,0")
        self.assertIsNone(motorapp._CACHE.last_baro.alt_m)


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
        motorapp.handle_gps("37.55,126.95,90.0,12.0,1,1")
        snap = motorapp._cache_snapshot()
        self.assertAlmostEqual(snap.latest_gps.lat, 37.55)
        self.assertAlmostEqual(snap.latest_gps.lon, 126.95)

    def test_snapshot_copies_latest_imu(self):
        motorapp.handle_imu("1.0,2.0,45.0,0.1,0.2,0.3,0.4,0.5,0.6,0.0,0.0,2.5,1")
        snap = motorapp._cache_snapshot()
        self.assertAlmostEqual(snap.latest_imu.gyrz_rad_s, math.radians(2.5))

    def test_snapshot_copies_latest_baro(self):
        motorapp.handle_barometer("333.0,1")
        snap = motorapp._cache_snapshot()
        self.assertAlmostEqual(snap.latest_baro.alt_m, 333.0)

    def test_snapshot_is_independent_copy(self):
        motorapp._CACHE.target_lat = 10.0
        snap = motorapp._cache_snapshot()
        motorapp._CACHE.target_lat = 99.0
        self.assertAlmostEqual(snap.target_lat, 10.0)

    def test_snapshot_copies_last_gps(self):
        motorapp.handle_gps("37.55,126.95,90.0,12.0,1,1")
        snap = motorapp._cache_snapshot()
        self.assertAlmostEqual(snap.last_gps.lat, 37.55)


if __name__ == "__main__":
    unittest.main()
