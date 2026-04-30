import unittest

from Sensor_Motor import motor_control


class TestMotorControl(unittest.TestCase):
    def test_actuator_mixer_clamp(self):
        l, r, _ = motor_control.actuator_mixer(9999.0)
        self.assertEqual(l, motor_control.LEFT_MAX)
        self.assertEqual(r, motor_control.RIGHT_MAX)

        l, r, _ = motor_control.actuator_mixer(-9999.0)
        self.assertEqual(l, motor_control.LEFT_MIN)
        self.assertEqual(r, motor_control.RIGHT_MIN)

    def test_neutral_and_off(self):
        handle = motor_control.init_control()
        backend = handle.pi
        self.assertEqual(backend.pulses[motor_control.LEFT_GPIO], motor_control.LEFT_NEUTRAL)
        self.assertEqual(backend.pulses[motor_control.RIGHT_GPIO], motor_control.RIGHT_NEUTRAL)

        motor_control.set_motors_off(handle)
        self.assertEqual(backend.pulses[motor_control.LEFT_GPIO], 0)
        self.assertEqual(backend.pulses[motor_control.RIGHT_GPIO], 0)

    def test_control_returns_feedback(self):
        handle = motor_control.init_control()
        fb = motor_control.control(handle, 5.0)
        self.assertTrue(hasattr(fb, "left_pulse"))
        self.assertTrue(hasattr(fb, "right_pulse"))
        self.assertGreaterEqual(fb.left_pulse, motor_control.LEFT_MIN)
        self.assertLessEqual(fb.right_pulse, motor_control.RIGHT_MAX)


if __name__ == "__main__":
    unittest.main()
