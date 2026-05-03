"""Tests for motor_control: mixer clamp, neutral/off, feedback object, signs."""

import unittest

from Sensor_Motor import motor_control


class TestActuatorMixer(unittest.TestCase):
    def test_clamp_high(self):
        l, r, _ = motor_control.actuator_mixer(9999.0)
        self.assertEqual(l, motor_control.LEFT_MAX)
        # RIGHT_SIGN = -1: large positive rate -> RIGHT_NEUTRAL - large offset -> RIGHT_MIN
        self.assertEqual(r, motor_control.RIGHT_MIN)

    def test_clamp_low(self):
        l, r, _ = motor_control.actuator_mixer(-9999.0)
        self.assertEqual(l, motor_control.LEFT_MIN)
        self.assertEqual(r, motor_control.RIGHT_MAX)

    def test_zero_is_neutral(self):
        l, r, offset = motor_control.actuator_mixer(0.0)
        self.assertEqual(l, motor_control.LEFT_NEUTRAL)
        self.assertEqual(r, motor_control.RIGHT_NEUTRAL)
        self.assertAlmostEqual(offset, 0.0)

    def test_differential_direction(self):
        """Positive yaw-rate: left brake pulled (increases), right released (decreases)."""
        l_pos, r_pos, _ = motor_control.actuator_mixer(10.0)
        l_neu, r_neu, _ = motor_control.actuator_mixer(0.0)
        self.assertGreater(l_pos, l_neu)   # LEFT_SIGN = +1
        self.assertLess(r_pos,   r_neu)    # RIGHT_SIGN = -1

    def test_output_within_bounds(self):
        for rate in [-50, -10, 0, 10, 50]:
            l, r, _ = motor_control.actuator_mixer(rate)
            self.assertGreaterEqual(l, motor_control.LEFT_MIN)
            self.assertLessEqual(l,   motor_control.LEFT_MAX)
            self.assertGreaterEqual(r, motor_control.RIGHT_MIN)
            self.assertLessEqual(r,   motor_control.RIGHT_MAX)


class TestInitAndSetters(unittest.TestCase):
    def setUp(self):
        self.handle  = motor_control.init_control()
        self.backend = self.handle.pi

    def test_init_sets_neutral(self):
        self.assertEqual(self.backend.pulses[motor_control.LEFT_GPIO],  motor_control.LEFT_NEUTRAL)
        self.assertEqual(self.backend.pulses[motor_control.RIGHT_GPIO], motor_control.RIGHT_NEUTRAL)

    def test_set_neutral(self):
        motor_control.set_motors_off(self.handle)
        motor_control.set_neutral(self.handle)
        self.assertEqual(self.backend.pulses[motor_control.LEFT_GPIO],  motor_control.LEFT_NEUTRAL)
        self.assertEqual(self.backend.pulses[motor_control.RIGHT_GPIO], motor_control.RIGHT_NEUTRAL)

    def test_set_motors_off(self):
        motor_control.set_motors_off(self.handle)
        self.assertEqual(self.backend.pulses[motor_control.LEFT_GPIO],  0)
        self.assertEqual(self.backend.pulses[motor_control.RIGHT_GPIO], 0)


class TestControlFeedback(unittest.TestCase):
    def setUp(self):
        self.handle = motor_control.init_control()

    def test_returns_feedback_namespace(self):
        fb = motor_control.control(self.handle, 5.0)
        self.assertTrue(hasattr(fb, "left_pulse"))
        self.assertTrue(hasattr(fb, "right_pulse"))
        self.assertTrue(hasattr(fb, "expected_yaw_rate"))

    def test_pulse_within_bounds(self):
        fb = motor_control.control(self.handle, 20.0)
        self.assertGreaterEqual(fb.left_pulse,  motor_control.LEFT_MIN)
        self.assertLessEqual(fb.left_pulse,     motor_control.LEFT_MAX)
        self.assertGreaterEqual(fb.right_pulse, motor_control.RIGHT_MIN)
        self.assertLessEqual(fb.right_pulse,    motor_control.RIGHT_MAX)

    def test_gpio_pins_match_config(self):
        from lib import config
        self.assertEqual(motor_control.LEFT_GPIO,  config.PARAFOIL_LEFT_GPIO)
        self.assertEqual(motor_control.RIGHT_GPIO, config.PARAFOIL_RIGHT_GPIO)


if __name__ == "__main__":
    unittest.main()
