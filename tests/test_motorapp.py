"""Unit tests for motorapp: handler parsing, FDIR logic, state transitions."""

import time
import unittest
from types import SimpleNamespace

from Sensor_Motor import motorapp, motor_guidance


def _reset_motorapp():
    """Reset all motorapp module-level state before each test."""
    motor_guidance.init_guidance()
    motorapp.MOTORAPP_RUNSTATUS = True
    motorapp.motor_enabled      = True
    motorapp.state              = 0
    motorapp._prev_state        = -1
    motorapp.target             = None
    motorapp._last_fdir_reason  = None
    motorapp._last_fdir_log_ts  = 0.0

    motorapp.sensor.yaw         = 10.0
    motorapp.sensor.gyrz        = 1.0
    motorapp.sensor.imu_health  = 1
    motorapp.sensor.lat         = 37.5
    motorapp.sensor.lon         = 126.9
    motorapp.sensor.speed       = 10.0
    motorapp.sensor.course      = 90.0
    motorapp.sensor.fix_quality = 1
    motorapp.sensor.sats        = 7
    motorapp.sensor.rmc_status  = "A"
    motorapp.sensor.gps_health  = 1
    motorapp.sensor.baro_m      = 120.0

    motorapp.target = SimpleNamespace(lat=37.6, lon=127.0)

    now = time.time()
    motorapp.last_gps_update  = now
    motorapp.last_imu_update  = now
    motorapp.last_baro_update = now

    # Prime GPS jump validator so _check_fdir can evaluate beyond FDIR-4
    motor_guidance.is_gps_jump(motorapp.sensor.lat, motorapp.sensor.lon)
    motor_guidance.is_gps_jump(motorapp.sensor.lat, motorapp.sensor.lon)


class TestFDIR(unittest.TestCase):
    def setUp(self):
        _reset_motorapp()

    def test_fdir_ok(self):
        snap = motorapp._snapshot_sensors()
        self.assertIsNone(motorapp._check_fdir(snap))

    def test_fdir_target_none(self):
        motorapp.target = None
        snap = motorapp._snapshot_sensors()
        result = motorapp._check_fdir(snap)
        self.assertIsNotNone(result)
        self.assertIn("target", result)

    def test_fdir_imu_unhealthy(self):
        motorapp.sensor.imu_health = 0
        snap = motorapp._snapshot_sensors()
        self.assertIn("imu", motorapp._check_fdir(snap))

    def test_fdir_gps_stale(self):
        motorapp.last_gps_update = time.time() - 999
        snap = motorapp._snapshot_sensors()
        self.assertIn("gps stale", motorapp._check_fdir(snap))

    def test_fdir_imu_stale(self):
        motorapp.last_imu_update = time.time() - 999
        snap = motorapp._snapshot_sensors()
        self.assertIn("imu stale", motorapp._check_fdir(snap))

    def test_fdir_baro_stale(self):
        motorapp.last_baro_update = time.time() - 999
        snap = motorapp._snapshot_sensors()
        self.assertIn("baro stale", motorapp._check_fdir(snap))

    def test_fdir_yaw_rate_implausible(self):
        motorapp.sensor.gyrz = 500.0
        snap = motorapp._snapshot_sensors()
        self.assertIn("yaw-rate", motorapp._check_fdir(snap))

    def test_fdir_negative_altitude(self):
        motorapp.sensor.baro_m = -1.0
        snap = motorapp._snapshot_sensors()
        self.assertIn("altitude", motorapp._check_fdir(snap))

    def test_fdir_never_received_gps(self):
        motorapp.last_gps_update = 0.0
        snap = motorapp._snapshot_sensors()
        result = motorapp._check_fdir(snap)
        self.assertIsNotNone(result)
        self.assertIn("gps stale", result)


class TestHandlers(unittest.TestCase):
    def setUp(self):
        _reset_motorapp()

    def test_handle_mec_off_on(self):
        motorapp.handle_mec("OFF")
        self.assertFalse(motorapp.motor_enabled)
        motorapp.handle_mec("ON")
        self.assertTrue(motorapp.motor_enabled)

    def test_handle_mec_unknown(self):
        motorapp.handle_mec("MAYBE")    # must not crash

    def test_handle_gps_valid(self):
        motorapp.handle_gps("37.55,126.95,12.0,180.0,1,8,A,1")
        self.assertAlmostEqual(motorapp.sensor.lat, 37.55)
        self.assertAlmostEqual(motorapp.sensor.lon, 126.95)
        self.assertGreater(motorapp.last_gps_update, 0)

    def test_handle_gps_too_few_fields(self):
        old_lat = motorapp.sensor.lat
        motorapp.handle_gps("37.55,126.95")    # only 2 fields
        self.assertEqual(motorapp.sensor.lat, old_lat)

    def test_handle_gps_bad_value(self):
        old_lat = motorapp.sensor.lat
        # float('NaN') parses successfully and IS stored (NaN propagates to FDIR-0 check)
        motorapp.handle_gps("NaN,126.95,12.0,180.0,1,8,A,1")
        import math
        self.assertTrue(math.isnan(motorapp.sensor.lat))

        # Restore and test a true parse error (non-numeric)
        motorapp.sensor.lat = old_lat
        motorapp.handle_gps("not_a_float,126.95,12.0,180.0,1,8,A,1")
        self.assertEqual(motorapp.sensor.lat, old_lat)   # unchanged on parse error

    def test_handle_imu_valid(self):
        import math
        motorapp.handle_imu("45.0,2.5,1")
        self.assertAlmostEqual(motorapp.sensor.yaw,  45.0)
        # 2.5 rad/s converted to deg/s
        self.assertAlmostEqual(motorapp.sensor.gyrz, 2.5 * (180.0 / math.pi), places=4)
        self.assertGreater(motorapp.last_imu_update, 0)

    def test_handle_imu_bad_data(self):
        old_yaw = motorapp.sensor.yaw
        motorapp.handle_imu("bad")
        self.assertEqual(motorapp.sensor.yaw, old_yaw)

    def test_handle_barometer_valid(self):
        motorapp.handle_barometer("200.5,1013.2")
        self.assertAlmostEqual(motorapp.sensor.baro_m, 200.5)
        self.assertGreater(motorapp.last_baro_update, 0)

    def test_handle_barometer_bad_data(self):
        old_baro = motorapp.sensor.baro_m
        motorapp.handle_barometer("NOPE")
        self.assertEqual(motorapp.sensor.baro_m, old_baro)

    def test_handle_target_coord_valid(self):
        motorapp.handle_target_coord("37.55,126.95")
        self.assertIsNotNone(motorapp.target)
        self.assertAlmostEqual(motorapp.target.lat, 37.55)
        self.assertAlmostEqual(motorapp.target.lon, 126.95)

    def test_handle_target_coord_out_of_range(self):
        motorapp.target = None
        motorapp.handle_target_coord("999.0,126.95")
        self.assertIsNone(motorapp.target)

    def test_handle_target_coord_bad_data(self):
        motorapp.target = None
        motorapp.handle_target_coord("not,valid")
        self.assertIsNone(motorapp.target)

    def test_handle_flight_state_transition(self):
        motorapp.state = 0
        motorapp.handle_flight_state("3")
        self.assertEqual(motorapp.state, 3)

    def test_handle_flight_state_same_no_change(self):
        motorapp.state = 2
        motorapp._prev_state = 1
        motorapp.handle_flight_state("2")
        self.assertEqual(motorapp._prev_state, 1)   # unchanged

    def test_handle_flight_state_bad(self):
        old = motorapp.state
        motorapp.handle_flight_state("abc")
        self.assertEqual(motorapp.state, old)

    def test_state3_locks_start_point(self):
        motorapp.sensor.lat = 37.55
        motorapp.sensor.lon = 126.95
        motorapp.handle_flight_state("3")
        self.assertAlmostEqual(motor_guidance._start_point.lat, 37.55)
        self.assertAlmostEqual(motor_guidance._start_point.lon, 126.95)


class TestSnapshot(unittest.TestCase):
    def setUp(self):
        _reset_motorapp()

    def test_snapshot_target_none(self):
        motorapp.target = None
        snap = motorapp._snapshot_sensors()
        self.assertIsNone(snap.target)

    def test_snapshot_target_present(self):
        motorapp.target = SimpleNamespace(lat=37.6, lon=127.0)
        snap = motorapp._snapshot_sensors()
        self.assertIsNotNone(snap.target)
        self.assertAlmostEqual(snap.target.lat, 37.6)


if __name__ == "__main__":
    unittest.main()
