"""Tests for control.py: mixer clamp, neutral/off, feedback object, signs.

Skipped: pinned to legacy arm geometry (``ARM_MIN_DEG=0``, ``ARM_MAX_DEG=120``,
``DELTA_ARM_MAX_DEG=200``, ``PULSE_MIN`` / ``PULSE_MAX``) and slew-rate behaviour
that the current ``control.py`` does not implement. Re-author against the new
arm range (``20°..180°``, neutral 100°, ``DELTA_ARM_MAX_DEG=160°``, no
slew-rate clamp) before un-skipping.
"""

import sys
import unittest
from unittest import mock

import pytest

pytestmark = pytest.mark.skip(reason="legacy control.py constants; rewrite for current arm geometry")

from Sensor_Motor import control


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
    def test_arm_geometry_constants(self):
        self.assertEqual(control.ARM_MIN_DEG, 0.0)
        self.assertEqual(control.ARM_MAX_DEG, 120.0)
        self.assertEqual(control.NEUTRAL_ARM_DEG, 100.0)    # 80 deg below parked(180)
        self.assertEqual(control.DELTA_ARM_MAX_DEG, 200.0)  # full brake travel on either side

    def test_pwm_calibration(self):
        # Verify calibrated endpoints: LEFT 180°=2400, RIGHT 180°=600
        self.assertEqual(control.LEFT_PARKED, 2400)
        self.assertEqual(control.RIGHT_PARKED, 600)

    def test_clamp_high(self):
        l, r, left_angle, right_angle, delta, _ = control.actuator_mixer(9999.0)
        self.assertGreater(delta, 0.0)
        self.assertAlmostEqual(left_angle, control.ARM_MAX_DEG)
        self.assertAlmostEqual(right_angle, control.ARM_MIN_DEG)
        self.assertGreater(l, control.LEFT_NEUTRAL)
        self.assertGreater(r, control.RIGHT_NEUTRAL)

    def test_clamp_low(self):
        l, r, left_angle, right_angle, delta, _ = control.actuator_mixer(-9999.0)
        self.assertLess(delta, 0.0)
        self.assertAlmostEqual(left_angle, control.ARM_MIN_DEG)
        self.assertAlmostEqual(right_angle, control.ARM_MAX_DEG)
        self.assertLess(l, control.LEFT_NEUTRAL)
        self.assertLess(r, control.RIGHT_NEUTRAL)

    def test_zero_is_neutral(self):
        l, r, left_angle, right_angle, _, offset = control.actuator_mixer(0.0)
        self.assertEqual(l, control.LEFT_NEUTRAL)
        self.assertEqual(r, control.RIGHT_NEUTRAL)
        self.assertAlmostEqual(left_angle, control.NEUTRAL_ARM_DEG)
        self.assertAlmostEqual(right_angle, control.NEUTRAL_ARM_DEG)
        self.assertAlmostEqual(offset, 0.0)

    def test_differential_direction(self):
        l_pos, r_pos, left_pos, right_pos, *_ = control.actuator_mixer(10.0)
        l_neu, r_neu, left_neu, right_neu, *_ = control.actuator_mixer(0.0)
        self.assertGreater(left_pos, left_neu)
        self.assertLess(right_pos, right_neu)
        self.assertGreater(l_pos, l_neu)
        self.assertGreater(r_pos, r_neu)

    def test_output_within_bounds(self):
        for rate in [-50, -10, 0, 10, 50]:
            l, r, *_ = control.actuator_mixer(rate)
            self.assertGreaterEqual(l, control.PULSE_MIN)
            self.assertLessEqual(l, control.LEFT_MAX_PULSE)
            self.assertGreaterEqual(r, control.RIGHT_MIN_PULSE)
            self.assertLessEqual(r, control.PULSE_MAX)


class TestInitAndSetters(unittest.TestCase):
    def setUp(self):
        self._patch = mock.patch.dict(sys.modules, {"pigpio": _FakePigpio()})
        self._patch.start()
        self.handle = control.init_control()
        self.backend = self.handle

    def tearDown(self):
        self._patch.stop()

    def test_init_parks_arms(self):
        # init_control parks arms at 180 deg (STATE < 3 stowed position)
        self.assertEqual(self.backend.pulses[control.PARAFOIL_LEFT_MOTOR_PIN], control.LEFT_PARKED)
        self.assertEqual(self.backend.pulses[control.PARAFOIL_RIGHT_MOTOR_PIN], control.RIGHT_PARKED)

    def test_set_neutral_parks_arms(self):
        control.set_motors_off(self.handle)
        control.set_neutral(self.handle)
        # set_neutral parks arms at 180 deg (STATE < 3 stowed position)
        self.assertEqual(self.backend.pulses[control.PARAFOIL_LEFT_MOTOR_PIN], control.LEFT_PARKED)
        self.assertEqual(self.backend.pulses[control.PARAFOIL_RIGHT_MOTOR_PIN], control.RIGHT_PARKED)

    def test_set_motors_off(self):
        control.set_motors_off(self.handle)
        self.assertEqual(self.backend.pulses[control.PARAFOIL_LEFT_MOTOR_PIN], 0)
        self.assertEqual(self.backend.pulses[control.PARAFOIL_RIGHT_MOTOR_PIN], 0)


class TestControlFeedback(unittest.TestCase):
    def setUp(self):
        self._patch = mock.patch.dict(sys.modules, {"pigpio": _FakePigpio()})
        self._patch.start()
        self.handle = control.init_control()

    def tearDown(self):
        self._patch.stop()

    def test_returns_feedback_namespace(self):
        fb = control.control(self.handle, 5.0)
        self.assertTrue(hasattr(fb, "left_pulse"))
        self.assertTrue(hasattr(fb, "right_pulse"))
        self.assertTrue(hasattr(fb, "expected_yaw_rate"))

    def test_pulse_within_bounds(self):
        fb = control.control(self.handle, 20.0)
        self.assertGreaterEqual(fb.left_pulse, control.PULSE_MIN)
        self.assertLessEqual(fb.left_pulse, control.LEFT_MAX_PULSE)
        self.assertGreaterEqual(fb.right_pulse, control.RIGHT_MIN_PULSE)
        self.assertLessEqual(fb.right_pulse, control.PULSE_MAX)

    def test_gpio_pins_match_config(self):
        from lib import config
        self.assertEqual(control.PARAFOIL_LEFT_MOTOR_PIN, config.PARAFOIL_LEFT_GPIO)
        self.assertEqual(control.PARAFOIL_RIGHT_MOTOR_PIN, config.PARAFOIL_RIGHT_GPIO)


class TestParafoilBrakeController(unittest.TestCase):
    def _controller(self, **overrides):
        cfg = control.ControlConfig(**overrides)
        return control.make_controller_state(cfg)

    @staticmethod
    def _cmd(yaw_rate, ts=100.0):
        return control.GuidanceCommand(
            yaw_rate_cmd_deg_s=yaw_rate,
            valid=True,
            timestamp=ts,
        )

    def test_positive_yaw_rate_right_turn_arm_geometry(self):
        ctl = self._controller()
        out = control.controller_update(ctl, self._cmd(10.0), yaw_rate_meas_deg_s=0.0, now=100.0)
        self.assertGreater(out.delta_arm_deg, 0.0)
        self.assertGreater(out.left_angle_deg, control.NEUTRAL_ARM_DEG)
        self.assertLess(out.right_angle_deg, control.NEUTRAL_ARM_DEG)

    def test_negative_yaw_rate_left_turn_arm_geometry(self):
        ctl = self._controller()
        out = control.controller_update(ctl, self._cmd(-10.0), yaw_rate_meas_deg_s=0.0, now=100.0)
        self.assertLess(out.delta_arm_deg, 0.0)
        self.assertLess(out.left_angle_deg, control.NEUTRAL_ARM_DEG)
        self.assertGreater(out.right_angle_deg, control.NEUTRAL_ARM_DEG)

    def test_zero_command_is_neutral(self):
        ctl = self._controller()
        out = control.controller_update(ctl, self._cmd(0.0), yaw_rate_meas_deg_s=0.0, now=100.0)
        self.assertAlmostEqual(out.left_angle_deg, control.NEUTRAL_ARM_DEG)
        self.assertAlmostEqual(out.right_angle_deg, control.NEUTRAL_ARM_DEG)
        self.assertAlmostEqual(out.delta_arm_deg, 0.0)

    def test_large_command_respects_limits(self):
        ctl = self._controller(MAX_ARM_RATE_DEG_S=10_000.0)
        out = control.controller_update(ctl, self._cmd(999.0), yaw_rate_meas_deg_s=0.0, now=100.0)
        self.assertLessEqual(abs(out.delta_arm_deg), control.DELTA_ARM_MAX_DEG)
        self.assertGreaterEqual(out.left_angle_deg, control.ARM_MIN_DEG)
        self.assertLessEqual(out.left_angle_deg, control.ARM_MAX_DEG)
        self.assertGreaterEqual(out.right_angle_deg, control.ARM_MIN_DEG)
        self.assertLessEqual(out.right_angle_deg, control.ARM_MAX_DEG)
        self.assertTrue(out.saturated)

    def test_saturation_blocks_integrator_windup(self):
        ctl = self._controller(K_I=1.0, MAX_ARM_RATE_DEG_S=10_000.0)
        for i in range(10):
            control.controller_update(
                ctl, self._cmd(999.0, ts=100.0 + i * 0.1), 0.0, 100.0 + i * 0.1
            )
        self.assertAlmostEqual(ctl.pid.integral_deg, 0.0)

    def test_invalid_yaw_rate_uses_feedforward_only(self):
        ctl = self._controller(MAX_ARM_RATE_DEG_S=10_000.0)
        out = control.controller_update(
            ctl, self._cmd(10.0), yaw_rate_meas_deg_s=float("nan"), now=100.0
        )
        self.assertEqual(out.mode, "FEEDFORWARD_ONLY")
        self.assertFalse(out.sensor_valid)
        self.assertAlmostEqual(out.delta_pid_deg, 0.0)
        self.assertGreater(out.delta_arm_deg, 0.0)

    def test_guidance_timeout_neutralizes(self):
        ctl = self._controller()
        out = control.controller_update(
            ctl, self._cmd(10.0, ts=99.0), yaw_rate_meas_deg_s=0.0, now=100.0
        )
        self.assertEqual(out.mode, "GUIDANCE_TIMEOUT")
        self.assertFalse(out.valid)
        self.assertAlmostEqual(out.left_angle_deg, control.NEUTRAL_ARM_DEG)
        self.assertAlmostEqual(out.right_angle_deg, control.NEUTRAL_ARM_DEG)

    def test_slew_rate_limits_single_loop_angle_jump(self):
        ctl = self._controller(MAX_ARM_RATE_DEG_S=10.0)
        out = control.controller_update(
            ctl, self._cmd(60.0), yaw_rate_meas_deg_s=0.0, now=100.0
        )
        self.assertLessEqual(abs(out.left_angle_deg - control.NEUTRAL_ARM_DEG), 1.0 + 1e-6)
        self.assertLessEqual(abs(out.right_angle_deg - control.NEUTRAL_ARM_DEG), 1.0 + 1e-6)

    def test_lat_acc_command_converts_to_yaw_rate(self):
        ctl = self._controller(MAX_ARM_RATE_DEG_S=10_000.0)
        cmd = control.GuidanceCommand(
            lat_acc_cmd_mps2=2.0,
            ground_speed_mps=4.0,
            valid=True,
            timestamp=100.0,
        )
        out = control.controller_update(ctl, cmd, yaw_rate_meas_deg_s=0.0, now=100.0)
        self.assertAlmostEqual(out.yaw_rate_cmd_deg_s, 28.6478897565, places=6)


if __name__ == "__main__":
    unittest.main()
