import time
import unittest
from types import SimpleNamespace

from Sensor_Motor import motor_guidance


class TestMotorGuidance(unittest.TestCase):
    def setUp(self):
        motor_guidance.init_guidance()

    def _good_inputs(self):
        imu = SimpleNamespace(yaw=10.0, gyrz=0.5)
        gps = SimpleNamespace(lat=37.55, lon=126.95, speed=12.0, course=90.0)
        fid = SimpleNamespace(fix_quality=1, sats=8, rmc_status="A")
        tgt = SimpleNamespace(lat=37.56, lon=126.96)
        return imu, gps, fid, tgt

    def test_invalid_gps_returns_fdir(self):
        imu, gps, _, tgt = self._good_inputs()
        bad_fid = SimpleNamespace(fix_quality=0, sats=1, rmc_status="V")
        out = motor_guidance.guidance(imu, gps, bad_fid, tgt, 100.0)
        self.assertEqual(out.state, "FDIR")

    def test_gps_jump_initially_fdir_then_valid(self):
        imu, gps, fid, tgt = self._good_inputs()
        out1 = motor_guidance.guidance(imu, gps, fid, tgt, 100.0)
        self.assertEqual(out1.state, "FDIR")

        time.sleep(0.02)
        gps2 = SimpleNamespace(lat=37.550001, lon=126.950001, speed=12.0, course=90.0)
        out2 = motor_guidance.guidance(imu, gps2, fid, tgt, 100.0)
        self.assertIn(out2.state, {"HOMING", "PATTERN", "LANDING"})

    def test_landing_command_limited(self):
        imu, gps, fid, tgt = self._good_inputs()
        # warm-up for gps jump stabilizer
        motor_guidance.guidance(imu, gps, fid, tgt, 100.0)
        time.sleep(0.02)
        out = motor_guidance.guidance(imu, SimpleNamespace(lat=37.550001, lon=126.950001, speed=9.0, course=80.0), fid, tgt, 5.0)
        self.assertLessEqual(abs(out.commanded_yaw_rate), motor_guidance.LANDING_YR_MAX + 1e-6)


if __name__ == "__main__":
    unittest.main()
