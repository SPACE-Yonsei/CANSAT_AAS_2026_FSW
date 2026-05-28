import math
import unittest

from lib import config
from Sensor_Motor import control, guidance


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


class TestCurrentDetumblingOutput(unittest.TestCase):
    def _detumble_output(self, gyrz_deg_s):
        return control.ProduceDetumbleOutput(100.0, gyrz_deg_s)

    def test_left_rotation_commands_max_right_damping(self):
        out = self._detumble_output(-250.0)

        self.assertEqual(out.mode, config.CONTROL_MODE_DETUMBLING)
        self.assertAlmostEqual(out.delta_arm_deg, control.DELTA_ARM_MAX_DEG)
        self.assertAlmostEqual(out.left_angle_deg, control.ARM_MIN_DEG)
        self.assertAlmostEqual(out.right_angle_deg, control.ARM_MAX_DEG)

    def test_right_rotation_commands_max_left_damping(self):
        out = self._detumble_output(250.0)

        self.assertEqual(out.mode, config.CONTROL_MODE_DETUMBLING)
        self.assertAlmostEqual(out.delta_arm_deg, -control.DELTA_ARM_MAX_DEG)
        self.assertAlmostEqual(out.left_angle_deg, control.ARM_MAX_DEG)
        self.assertAlmostEqual(out.right_angle_deg, control.ARM_MIN_DEG)


if __name__ == "__main__":
    unittest.main()
