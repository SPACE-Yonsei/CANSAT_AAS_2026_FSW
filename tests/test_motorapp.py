"""Unit tests for motorapp: handler parsing, state transitions, snapshot."""

import math
import unittest
from types import SimpleNamespace

from Sensor_Motor import motorapp, motor_guidance


def _reset_motorapp():
    """Reset all motorapp module-level state before each test."""
    motor_guidance.init_guidance()
    motorapp.MOTORAPP_RUNSTATUS  = True
    motorapp.MOTOR_ENABLED       = True
    motorapp.STATE               = 0
    motorapp._PREV_STATE         = -1
    motorapp.TARGET              = None
    motorapp._START_POINT_LOCKED = False

    motorapp.IMU.yaw        = 10.0
    motorapp.IMU.gyrz       = 1.0
    motorapp.IMU.imu_health = 1

    motorapp.GPS_VECTOR.lat       = 37.5
    motorapp.GPS_VECTOR.lon       = 126.9
    motorapp.GPS_VECTOR.direction = 90.0
    motorapp.GPS_VECTOR.velocity  = 10.0

    motorapp.GPS_HEALTH.pos_health    = 1
    motorapp.GPS_HEALTH.motion_health = 1

    motorapp.ALT = 120.0

    motorapp.TARGET = SimpleNamespace(lat=37.6, lon=127.0)

    motor_guidance.is_gps_jump(motorapp.GPS_VECTOR.lat, motorapp.GPS_VECTOR.lon)
    motor_guidance.is_gps_jump(motorapp.GPS_VECTOR.lat, motorapp.GPS_VECTOR.lon)


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
        motorapp.handle_gps("37.55,126.95,180.0,12.0,1,1")
        self.assertAlmostEqual(motorapp.GPS_VECTOR.lat,       37.55)
        self.assertAlmostEqual(motorapp.GPS_VECTOR.lon,       126.95)
        self.assertAlmostEqual(motorapp.GPS_VECTOR.direction, 180.0)
        self.assertAlmostEqual(motorapp.GPS_VECTOR.velocity,  12.0)
        self.assertEqual(motorapp.GPS_HEALTH.pos_health,    1)
        self.assertEqual(motorapp.GPS_HEALTH.motion_health, 1)

    def test_handle_gps_too_few_fields(self):
        old_lat = motorapp.GPS_VECTOR.lat
        motorapp.handle_gps("37.55,126.95")    # only 2 fields
        self.assertEqual(motorapp.GPS_VECTOR.lat, old_lat)

    def test_handle_gps_bad_value(self):
        old_lat = motorapp.GPS_VECTOR.lat
        # NaN parses successfully and IS stored
        motorapp.handle_gps("NaN,126.95,180.0,12.0,1,1")
        self.assertTrue(math.isnan(motorapp.GPS_VECTOR.lat))

        # Restore and test a true parse error
        motorapp.GPS_VECTOR.lat = old_lat
        motorapp.handle_gps("not_a_float,126.95,180.0,12.0,1,1")
        self.assertEqual(motorapp.GPS_VECTOR.lat, old_lat)

    def test_handle_imu_valid(self):
        motorapp.handle_imu("45.0,2.5,1")
        self.assertAlmostEqual(motorapp.IMU.yaw,  45.0)
        self.assertAlmostEqual(motorapp.IMU.gyrz, 2.5)
        self.assertEqual(motorapp.IMU.imu_health, 1)

    def test_handle_imu_bad_data(self):
        old_yaw = motorapp.IMU.yaw
        motorapp.handle_imu("bad")
        self.assertEqual(motorapp.IMU.yaw, old_yaw)

    def test_handle_barometer_valid(self):
        motorapp.handle_barometer("200.5,1013.2")
        self.assertAlmostEqual(motorapp.ALT, 200.5)

    def test_handle_barometer_bad_data(self):
        old_baro = motorapp.ALT
        motorapp.handle_barometer("NOPE")
        self.assertEqual(motorapp.ALT, old_baro)

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
        self.assertEqual(motorapp._PREV_STATE, 1)

    def test_handle_flight_state_bad(self):
        old = motorapp.STATE
        motorapp.handle_flight_state("abc")
        self.assertEqual(motorapp.STATE, old)

    def test_state3_locks_start_point(self):
        motorapp.GPS_VECTOR.lat        = 37.55
        motorapp.GPS_VECTOR.lon        = 126.95
        motorapp.GPS_HEALTH.pos_health = 1
        motorapp.handle_flight_state("3")
        self.assertAlmostEqual(motor_guidance.START_POINT.lat, 37.55)
        self.assertAlmostEqual(motor_guidance.START_POINT.lon, 126.95)

    def test_state3_defers_start_point_until_valid_gps(self):
        motorapp.GPS_VECTOR.lat        = 37.55
        motorapp.GPS_VECTOR.lon        = 0.0
        motorapp.GPS_HEALTH.pos_health = 0
        motorapp.handle_flight_state("3")
        self.assertFalse(motorapp._START_POINT_LOCKED)
        motorapp.handle_gps("37.55,126.95,90.0,12.0,1,1")
        self.assertTrue(motorapp._START_POINT_LOCKED)
        self.assertAlmostEqual(motor_guidance.START_POINT.lon, 126.95)


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

    def test_snapshot_reflects_imu(self):
        motorapp.IMU.yaw  = 99.0
        motorapp.IMU.gyrz = 5.5
        snap = motorapp._snapshot_sensors()
        self.assertAlmostEqual(snap.yaw,  99.0)
        self.assertAlmostEqual(snap.gyrz, 5.5)

    def test_snapshot_reflects_gps(self):
        motorapp.GPS_VECTOR.lat = 11.1
        motorapp.GPS_VECTOR.lon = 22.2
        snap = motorapp._snapshot_sensors()
        self.assertAlmostEqual(snap.lat, 11.1)
        self.assertAlmostEqual(snap.lon, 22.2)

    def test_snapshot_reflects_alt(self):
        motorapp.ALT = 333.0
        snap = motorapp._snapshot_sensors()
        self.assertAlmostEqual(snap.alt, 333.0)


if __name__ == "__main__":
    unittest.main()
