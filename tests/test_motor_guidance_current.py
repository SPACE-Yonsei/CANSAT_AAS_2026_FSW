import math
import unittest

from Sensor_Motor import guidance


class TestCurrentGuidanceOutput(unittest.TestCase):
    def test_near_target_still_uses_bearing_guidance(self):
        l1_input = guidance.L1Input(
            valid=True,
            control_mode=guidance.ControlMode.GPS_TRACKING_CLOSED,
            confidence=1.0,
            E=0.0,
            N=902.5,
            V=6.0,
            course=0.0,
            target_E=0.0,
            target_N=900.0,
        )

        out = guidance.ProduceL1Output(l1_input)

        self.assertTrue(out.control_valid)
        self.assertTrue(out.pid_enabled)
        self.assertNotEqual(out.reason, "TARGET_REACHED")
        self.assertAlmostEqual(out.distance_to_target, 2.5)
        self.assertAlmostEqual(math.degrees(out.nu), -180.0)
        self.assertLess(out.yaw_rate_cmd, 0.0)


if __name__ == "__main__":
    unittest.main()
