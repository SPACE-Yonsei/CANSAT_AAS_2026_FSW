import time
import unittest
from types import SimpleNamespace

from Sensor_Motor import motorapp
from Sensor_Motor import motor_guidance


class TestMotorApp(unittest.TestCase):
    def setUp(self):
        motor_guidance.init_guidance()
        motorapp.last_gps_update = time.time()
        motorapp.sensor.yaw = 10.0
        motorapp.sensor.gyrz = 1.0
        motorapp.sensor.imu_health = 1
        motorapp.sensor.lat = 37.5
        motorapp.sensor.lon = 126.9
        motorapp.sensor.speed = 10.0
        motorapp.sensor.course = 90.0
        motorapp.sensor.fix_quality = 1
        motorapp.sensor.sats = 7
        motorapp.sensor.rmc_status = "A"
        motorapp.sensor.gps_health = 1
        motorapp.sensor.baro_m = 120.0
        motorapp.target.lat = 37.6
        motorapp.target.lon = 127.0
        # Prime GPS jump validator so _check_fdir can evaluate next conditions.
        motor_guidance.is_gps_jump(motorapp.sensor.lat, motorapp.sensor.lon)
        motor_guidance.is_gps_jump(motorapp.sensor.lat, motorapp.sensor.lon)

    def test_check_fdir_ok(self):
        snap = motorapp._snapshot_sensors()
        self.assertIsNone(motorapp._check_fdir(snap))

    def test_check_fdir_target_missing(self):
        motorapp.target.lat = 0.0
        motorapp.target.lon = 0.0
        snap = motorapp._snapshot_sensors()
        self.assertEqual(motorapp._check_fdir(snap), "FDIR-5 target missing")

    def test_handle_mec(self):
        motorapp.motor_enabled = True
        motorapp.handle_mec("OFF")
        self.assertFalse(motorapp.motor_enabled)
        motorapp.handle_mec("ON")
        self.assertTrue(motorapp.motor_enabled)

    def test_handle_target_coord_validation(self):
        motorapp.target.lat = 0.0
        motorapp.target.lon = 0.0
        motorapp.handle_target_coord("37.55,126.95")
        self.assertAlmostEqual(motorapp.target.lat, 37.55)
        self.assertAlmostEqual(motorapp.target.lon, 126.95)


if __name__ == "__main__":
    unittest.main()
