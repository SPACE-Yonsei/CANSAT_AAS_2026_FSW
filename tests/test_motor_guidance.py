import time
import unittest
from types import SimpleNamespace

from Sensor_Motor import motor_guidance
from Sensor_Motor.motor_guidance import GpsFidelity, GpsVector


class TestMotorGuidance(unittest.TestCase):
    def setUp(self):
        motor_guidance.init_guidance()

    def _good_inputs(self):
        imu = SimpleNamespace(yaw=10.0, gyrz=0.5)
        gps = GpsVector(lat=37.55, lon=126.95, speed=12.0, course=90.0)
        fid = GpsFidelity(fix_quality=1, sats=8, rmc_status="A", gps_health=1)
        tgt = SimpleNamespace(lat=37.56, lon=126.96)
        return imu, gps, fid, tgt

    def test_invalid_gps_returns_fdir(self):
        imu, gps, _, tgt = self._good_inputs()
        bad_fid = GpsFidelity(fix_quality=0, sats=1, rmc_status="V", gps_health=0)
        out = motor_guidance.guidance(imu, gps, bad_fid, tgt, 100.0)
        self.assertEqual(out.state, "FDIR")

    def test_gps_health_zero_returns_fdir(self):
        imu, gps, _, tgt = self._good_inputs()
        bad_fid = GpsFidelity(fix_quality=1, sats=8, rmc_status="A", gps_health=0)
        out = motor_guidance.guidance(imu, gps, bad_fid, tgt, 100.0)
        self.assertEqual(out.state, "FDIR")

    def test_gps_jump_initially_fdir_then_valid(self):
        imu, gps, fid, tgt = self._good_inputs()
        out1 = motor_guidance.guidance(imu, gps, fid, tgt, 100.0)
        self.assertEqual(out1.state, "FDIR")

        time.sleep(0.02)
        gps2 = GpsVector(lat=37.550001, lon=126.950001, speed=12.0, course=90.0)
        out2 = motor_guidance.guidance(imu, gps2, fid, tgt, 100.0)
        self.assertIn(out2.state, {"HOMING", "PATTERN", "LANDING"})

    def test_landing_command_limited(self):
        imu, gps, fid, tgt = self._good_inputs()
        motor_guidance.guidance(imu, gps, fid, tgt, 100.0)
        time.sleep(0.02)
        gps2 = GpsVector(lat=37.550001, lon=126.950001, speed=9.0, course=80.0)
        out = motor_guidance.guidance(imu, gps2, fid, tgt, 5.0)
        self.assertLessEqual(abs(out.commanded_yaw_rate), motor_guidance.LANDING_YR_MAX + 1e-6)

    def test_guidance_debug_fields_finite(self):
        imu, gps, fid, tgt = self._good_inputs()
        motor_guidance.set_start_coordinates(gps.lat, gps.lon)
        motor_guidance.guidance(imu, gps, fid, tgt, 100.0)
        time.sleep(0.02)
        gps2 = GpsVector(lat=37.550001, lon=126.950001, speed=12.0, course=90.0)
        out = motor_guidance.guidance(imu, gps2, fid, tgt, 120.0)
        self.assertIn(out.state, {"HOMING", "PATTERN", "LANDING"})
        import math
        for name in (
            "crosstrack_error",
            "along_track",
            "lookahead",
            "heading_error",
            "desired_yaw_rate",
            "commanded_yaw_rate",
        ):
            self.assertTrue(math.isfinite(getattr(out, name)), name)

    def test_gps_vector_is_namedtuple(self):
        gps = GpsVector(lat=37.55, lon=126.95, speed=10.0, course=90.0)
        self.assertEqual(gps.lat,    37.55)
        self.assertEqual(gps.speed,  10.0)

    def test_gps_fidelity_is_namedtuple(self):
        fid = GpsFidelity(fix_quality=1, sats=7, rmc_status="A", gps_health=1)
        self.assertEqual(fid.rmc_status, "A")
        self.assertEqual(fid.gps_health,  1)


if __name__ == "__main__":
    unittest.main()
