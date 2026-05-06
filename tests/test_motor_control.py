"""Tests for motor_control: mixer clamp, neutral/off, feedback object, signs."""

import sys
import unittest
from unittest import mock

from Sensor_Motor import motor_control


class _FakePi:
    def __init__(self):
        self.pulses = {}
        self.connected = 1

    def set_servo_pulsewidth(self, pin, width):
        self.pulses[pin] = width

    def stop(self):
        return None


class _FakePigpio:
    @staticmethod
    def pi():
        return _FakePi()


class TestActuatorMixer(unittest.TestCase):
    def test_clamp_high(self):
        l, r, *_ = motor_control.actuator_mixer(9999.0)
        self.assertLess(l, motor_control.LEFT_NEUTRAL)
        self.assertGreater(r, motor_control.RIGHT_NEUTRAL)

    def test_clamp_low(self):
        l, r, *_ = motor_control.actuator_mixer(-9999.0)
        self.assertGreater(l, motor_control.LEFT_NEUTRAL)
        self.assertLess(r, motor_control.RIGHT_NEUTRAL)

    def test_zero_is_neutral(self):
        l, r, _, _, _, offset = motor_control.actuator_mixer(0.0)
        self.assertEqual(l, motor_control.LEFT_NEUTRAL)
        self.assertEqual(r, motor_control.RIGHT_NEUTRAL)
        self.assertAlmostEqual(offset, 0.0)

    def test_differential_direction(self):
        l_pos, r_pos, *_ = motor_control.actuator_mixer(10.0)
        l_neu, r_neu, *_ = motor_control.actuator_mixer(0.0)
        self.assertLess(l_pos, l_neu)
        self.assertGreater(r_pos, r_neu)

    def test_output_within_bounds(self):
        for rate in [-50, -10, 0, 10, 50]:
            l, r, *_ = motor_control.actuator_mixer(rate)
            self.assertGreaterEqual(l, motor_control.PULSE_MIN)
            self.assertLessEqual(l, motor_control.LEFT_MAX_PULSE)
            self.assertGreaterEqual(r, motor_control.RIGHT_MIN_PULSE)
            self.assertLessEqual(r, motor_control.PULSE_MAX)


class TestInitAndSetters(unittest.TestCase):
    def setUp(self):
        self._patch = mock.patch.dict(sys.modules, {"pigpio": _FakePigpio()})
        self._patch.start()
        self.handle = motor_control.init_control()
        self.backend = self.handle

    def tearDown(self):
        self._patch.stop()

    def test_init_sets_neutral(self):
        self.assertEqual(self.backend.pulses[motor_control.PARAFOIL_LEFT_MOTOR_PIN], motor_control.LEFT_NEUTRAL)
        self.assertEqual(self.backend.pulses[motor_control.PARAFOIL_RIGHT_MOTOR_PIN], motor_control.RIGHT_NEUTRAL)

    def test_set_neutral(self):
        motor_control.set_motors_off(self.handle)
        motor_control.set_neutral(self.handle)
        self.assertEqual(self.backend.pulses[motor_control.PARAFOIL_LEFT_MOTOR_PIN], motor_control.LEFT_NEUTRAL)
        self.assertEqual(self.backend.pulses[motor_control.PARAFOIL_RIGHT_MOTOR_PIN], motor_control.RIGHT_NEUTRAL)

    def test_set_motors_off(self):
        motor_control.set_motors_off(self.handle)
        self.assertEqual(self.backend.pulses[motor_control.PARAFOIL_LEFT_MOTOR_PIN], 0)
        self.assertEqual(self.backend.pulses[motor_control.PARAFOIL_RIGHT_MOTOR_PIN], 0)


class TestControlFeedback(unittest.TestCase):
    def setUp(self):
        self._patch = mock.patch.dict(sys.modules, {"pigpio": _FakePigpio()})
        self._patch.start()
        self.handle = motor_control.init_control()

    def tearDown(self):
        self._patch.stop()

    def test_returns_feedback_namespace(self):
        fb = motor_control.control(self.handle, 5.0)
        self.assertTrue(hasattr(fb, "left_pulse"))
        self.assertTrue(hasattr(fb, "right_pulse"))
        self.assertTrue(hasattr(fb, "expected_yaw_rate"))

    def test_pulse_within_bounds(self):
        fb = motor_control.control(self.handle, 20.0)
        self.assertGreaterEqual(fb.left_pulse, motor_control.PULSE_MIN)
        self.assertLessEqual(fb.left_pulse, motor_control.LEFT_MAX_PULSE)
        self.assertGreaterEqual(fb.right_pulse, motor_control.RIGHT_MIN_PULSE)
        self.assertLessEqual(fb.right_pulse, motor_control.PULSE_MAX)

    def test_gpio_pins_match_config(self):
        from lib import config
        self.assertEqual(motor_control.PARAFOIL_LEFT_MOTOR_PIN, config.PARAFOIL_LEFT_GPIO)
        self.assertEqual(motor_control.PARAFOIL_RIGHT_MOTOR_PIN, config.PARAFOIL_RIGHT_GPIO)


if __name__ == "__main__":
    unittest.main()
