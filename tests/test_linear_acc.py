import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from Sensor_Motor import motorapp


class LinearAccTests(unittest.TestCase):
    def test_flat_static_removes_gravity(self):
        result = motorapp._compute_linear_acc(0.0, 0.0, 0.0, 0.0, 9.81)

        self.assertTrue(result.valid)
        self.assertEqual(result.reject_reason, "OK")
        self.assertAlmostEqual(result.x, 0.0, places=6)
        self.assertAlmostEqual(result.y, 0.0, places=6)
        self.assertAlmostEqual(result.z, 0.0, places=6)

    def test_pitch_static_removes_gravity(self):
        pitch_deg = 10.0
        pitch = math.radians(pitch_deg)
        ax = -math.sin(pitch) * 9.81
        az = math.cos(pitch) * 9.81

        result = motorapp._compute_linear_acc(0.0, pitch_deg, ax, 0.0, az)

        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.x, 0.0, places=6)
        self.assertAlmostEqual(result.y, 0.0, places=6)
        self.assertAlmostEqual(result.z, 0.0, places=6)

    def test_roll_static_removes_gravity(self):
        roll_deg = 10.0
        roll = math.radians(roll_deg)
        ay = math.sin(roll) * 9.81
        az = math.cos(roll) * 9.81

        result = motorapp._compute_linear_acc(roll_deg, 0.0, 0.0, ay, az)

        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.x, 0.0, places=6)
        self.assertAlmostEqual(result.y, 0.0, places=6)
        self.assertAlmostEqual(result.z, 0.0, places=6)

    def test_rejects_raw_acc_spike(self):
        result = motorapp._compute_linear_acc(0.0, 0.0, 0.0, 0.0, 20.0)

        self.assertFalse(result.valid)
        self.assertEqual(result.reject_reason, "RAW_ACC_SPIKE")

    def test_rejects_fast_gyro(self):
        result = motorapp._compute_linear_acc(
            0.0, 0.0, 0.0, 0.0, 9.81, gyrz_deg_s=90.0
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.reject_reason, "GYRO_TOO_FAST")

    def test_rejects_large_linear_xy_acc(self):
        result = motorapp._compute_linear_acc(0.0, 0.0, 2.0, 0.0, 9.81)

        self.assertFalse(result.valid)
        self.assertEqual(result.reject_reason, "LIN_ACC_TOO_LARGE")

    def test_rejects_stale_sample(self):
        result = motorapp._compute_linear_acc(
            0.0, 0.0, 0.0, 0.0, 9.81, sample_age_s=0.2
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.reject_reason, "IMU_STALE")


if __name__ == "__main__":
    unittest.main()
