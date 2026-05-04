"""Unit tests for motorapp: handler parsing, FDIR logic, state transitions."""

import time
import unittest
from types import SimpleNamespace

from Sensor_Motor import motorapp, motor_guidance


def _reset_motorapp():
    """Reset all motorapp module-level state before each test."""
    motor_guidance.init_guidance()
    motorapp.MOTORAPP_RUNSTATUS = True
    motorapp.MOTOR_ENABLED      = True
    motorapp.STATE              = 0
    motorapp._PREV_STATE        = -1
    motorapp.TARGET             = None
    motorapp._LAST_FDIR_REASON  = None
    motorapp._LAST_FDIR_LOG_TS  = 0.0
    motorapp._START_POINT_LOCKED = False
    motorapp._LAST_GYRZ_FOR_FDIR = None

    motorapp.SENSOR.yaw         = 10.0
    motorapp.SENSOR.gyrz        = 1.0
    motorapp.SENSOR.imu_health  = 1
    motorapp.SENSOR.lat         = 37.5
    motorapp.SENSOR.lon         = 126.9
    motorapp.SENSOR.speed       = 10.0
    motorapp.SENSOR.course      = 90.0
    motorapp.SENSOR.fix_quality = 1
    motorapp.SENSOR.sats        = 7
    motorapp.SENSOR.rmc_status  = "A"
    motorapp.SENSOR.gps_health  = 1
    motorapp.SENSOR.alt      = 120.0

    motorapp.TARGET = SimpleNamespace(lat=37.6, lon=127.0)

    now = time.time()
    motorapp._LAST_GPS_UPDATE  = now
    motorapp._LAST_IMU_UPDATE  = now
    motorapp._LAST_BARO_UPDATE = now

    # Prime GPS jump validator so _check_fdir can evaluate beyond FDIR-4
    motor_guidance.is_gps_jump(motorapp.SENSOR.lat, motorapp.SENSOR.lon)
    motor_guidance.is_gps_jump(motorapp.SENSOR.lat, motorapp.SENSOR.lon)


class TestFDIR(unittest.TestCase):
    def setUp(self):
        _reset_motorapp()

    def test_fdir_ok(self):
        snap = motorapp._snapshot_sensors()
        self.assertIsNone(motorapp._check_fdir(snap))

    def test_fdir_target_none(self):
        motorapp.TARGET = None
        snap = motorapp._snapshot_sensors()
        result = motorapp._check_fdir(snap)
        self.assertIsNotNone(result)
        self.assertIn("target", result)

    def test_fdir_imu_unhealthy(self):
        motorapp.SENSOR.imu_health = 0
        snap = motorapp._snapshot_sensors()
        self.assertIn("imu", motorapp._check_fdir(snap))

    def test_fdir_gps_stale(self):
        motorapp._LAST_GPS_UPDATE = time.time() - 999
        snap = motorapp._snapshot_sensors()
        self.assertIn("gps stale", motorapp._check_fdir(snap))

    def test_fdir_imu_stale(self):
        motorapp._LAST_IMU_UPDATE = time.time() - 999
        snap = motorapp._snapshot_sensors()
        self.assertIn("imu stale", motorapp._check_fdir(snap))

    def test_fdir_baro_stale(self):
        motorapp._LAST_BARO_UPDATE = time.time() - 999
        snap = motorapp._snapshot_sensors()
        self.assertIn("baro stale", motorapp._check_fdir(snap))

    def test_fdir_yaw_rate_implausible(self):
        motorapp.SENSOR.gyrz = 600.0
        snap = motorapp._snapshot_sensors()
        self.assertIn("yaw-rate", motorapp._check_fdir(snap))

    def test_fdir_moderate_motoroff_yaw_rate_not_hard_fault(self):
        motorapp.SENSOR.gyrz = 180.0
        snap = motorapp._snapshot_sensors()
        self.assertIsNone(motorapp._check_fdir(snap))

    def test_fdir_negative_altitude(self):
        motorapp.SENSOR.alt = -1.0
        snap = motorapp._snapshot_sensors()
        self.assertIn("altitude", motorapp._check_fdir(snap))

    def test_fdir_never_received_gps(self):
        motorapp._LAST_GPS_UPDATE = 0.0
        snap = motorapp._snapshot_sensors()
        result = motorapp._check_fdir(snap)
        self.assertIsNotNone(result)
        self.assertIn("gps stale", result)


class TestHandlers(unittest.TestCase):
    def setUp(self):
        _reset_motorapp()

    def test_handle_mec_off_on(self):
        motorapp.handle_mec("OFF")
        self.assertFalse(motorapp.MOTOR_ENABLED)
        motorapp.handle_mec("ON")
        self.assertTrue(motorapp.MOTOR_ENABLED)

    def test_handle_mec_unknown(self):
        motorapp.handle_mec("MAYBE")    # must not crash

    def test_handle_gps_valid(self):
        motorapp.handle_gps("37.55,126.95,12.0,180.0,1,8,A,1")
        self.assertAlmostEqual(motorapp.SENSOR.lat, 37.55)
        self.assertAlmostEqual(motorapp.SENSOR.lon, 126.95)
        self.assertGreater(motorapp._LAST_GPS_UPDATE, 0)

    def test_handle_gps_too_few_fields(self):
        old_lat = motorapp.SENSOR.lat
        motorapp.handle_gps("37.55,126.95")    # only 2 fields
        self.assertEqual(motorapp.SENSOR.lat, old_lat)

    def test_handle_gps_bad_value(self):
        old_lat = motorapp.SENSOR.lat
        # float('NaN') parses successfully and IS stored (NaN propagates to FDIR-0 check)
        motorapp.handle_gps("NaN,126.95,12.0,180.0,1,8,A,1")
        import math
        self.assertTrue(math.isnan(motorapp.SENSOR.lat))

        # Restore and test a true parse error (non-numeric)
        motorapp.SENSOR.lat = old_lat
        motorapp.handle_gps("not_a_float,126.95,12.0,180.0,1,8,A,1")
        self.assertEqual(motorapp.SENSOR.lat, old_lat)   # unchanged on parse error

    def test_handle_imu_valid(self):
        motorapp.handle_imu("45.0,2.5,1")
        self.assertAlmostEqual(motorapp.SENSOR.yaw,  45.0)
        self.assertAlmostEqual(motorapp.SENSOR.gyrz, 2.5)
        self.assertGreater(motorapp._LAST_IMU_UPDATE, 0)

    def test_handle_imu_bad_data(self):
        old_yaw = motorapp.SENSOR.yaw
        motorapp.handle_imu("bad")
        self.assertEqual(motorapp.SENSOR.yaw, old_yaw)

    def test_handle_barometer_valid(self):
        motorapp.handle_barometer("200.5,1013.2")
        self.assertAlmostEqual(motorapp.SENSOR.alt, 200.5)
        self.assertGreater(motorapp._LAST_BARO_UPDATE, 0)

    def test_handle_barometer_bad_data(self):
        old_baro = motorapp.SENSOR.alt
        motorapp.handle_barometer("NOPE")
        self.assertEqual(motorapp.SENSOR.alt, old_baro)

    def test_handle_target_coord_valid(self):
        motorapp.handle_target_coord("37.55,126.95")
        self.assertIsNotNone(motorapp.TARGET)
        self.assertAlmostEqual(motorapp.TARGET.lat, 37.55)
        self.assertAlmostEqual(motorapp.TARGET.lon, 126.95)

    def test_handle_target_coord_out_of_range(self):
        motorapp.TARGET = None
        motorapp.handle_target_coord("999.0,126.95")
        self.assertIsNone(motorapp.TARGET)

    def test_handle_target_coord_bad_data(self):
        motorapp.TARGET = None
        motorapp.handle_target_coord("not,valid")
        self.assertIsNone(motorapp.TARGET)

    def test_handle_flight_state_transition(self):
        motorapp.STATE = 0
        motorapp.handle_flight_state("3")
        self.assertEqual(motorapp.STATE, 3)

    def test_handle_flight_state_same_no_change(self):
        motorapp.STATE = 2
        motorapp._PREV_STATE = 1
        motorapp.handle_flight_state("2")
        self.assertEqual(motorapp._PREV_STATE, 1)   # unchanged

    def test_handle_flight_state_bad(self):
        old = motorapp.STATE
        motorapp.handle_flight_state("abc")
        self.assertEqual(motorapp.STATE, old)

    def test_state3_locks_start_point(self):
        motorapp.SENSOR.lat = 37.55
        motorapp.SENSOR.lon = 126.95
        motorapp.SENSOR.fix_quality = 1
        motorapp.SENSOR.sats = 8
        motorapp.SENSOR.rmc_status = "A"
        motorapp.SENSOR.gps_health = 1
        motorapp.handle_flight_state("3")
        self.assertAlmostEqual(motor_guidance._start_point.lat, 37.55)
        self.assertAlmostEqual(motor_guidance._start_point.lon, 126.95)

    def test_state3_defers_start_point_until_valid_gps(self):
        motorapp.SENSOR.lat = 37.55
        motorapp.SENSOR.lon = 0.0
        motorapp.SENSOR.fix_quality = 0
        motorapp.SENSOR.sats = 0
        motorapp.SENSOR.rmc_status = "V"
        motorapp.SENSOR.gps_health = 0
        motorapp.handle_flight_state("3")
        self.assertFalse(motorapp._START_POINT_LOCKED)
        motorapp.handle_gps("37.55,126.95,12.0,90.0,1,8,A,1")
        self.assertTrue(motorapp._START_POINT_LOCKED)
        self.assertAlmostEqual(motor_guidance._start_point.lon, 126.95)


class TestSnapshot(unittest.TestCase):
    def setUp(self):
        _reset_motorapp()

    def test_snapshot_target_none(self):
        motorapp.TARGET = None
        snap = motorapp._snapshot_sensors()
        self.assertIsNone(snap.target)

    def test_snapshot_target_present(self):
        motorapp.TARGET = SimpleNamespace(lat=37.6, lon=127.0)
        snap = motorapp._snapshot_sensors()
        self.assertIsNotNone(snap.target)
        self.assertAlmostEqual(snap.target.lat, 37.6)


if __name__ == "__main__":
    unittest.main()

