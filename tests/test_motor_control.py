"""Tests for control.py: ConnectRoMo, controller_update, servo setters, constants."""

import math
import sys
import time
import unittest
from unittest import mock

from Sensor_Motor import control, guidance


class _FakePi:
    def __init__(self):
        self.pulses = {}
        self.connected = 1

    def set_servo_pulsewidth(self, pin, width):
        self.pulses[pin] = width


class _FakePigpio:
    @staticmethod
    def pi():
        return _FakePi()


class TestConstants(unittest.TestCase):
    def test_arm_geometry(self):
        self.assertEqual(control.ARM_MIN_DEG, 0.0)
        self.assertEqual(control.ARM_MAX_DEG, 160.0)
        self.assertEqual(control.NEUTRAL_ARM_DEG, 80.0)
        self.assertEqual(control.DELTA_ARM_MAX_DEG, 160.0)

    def test_pwm_zero_positions(self):
        self.assertEqual(control.LEFT_ZERO, 2480)
        self.assertEqual(control.RIGHT_ZERO, 636)

    def test_neutral_pulse_derivation(self):
        expected_left = int(control.LEFT_ZERO - control.NEUTRAL_ARM_DEG * control.PULSE_PER_DEG)
        expected_right = int(control.RIGHT_ZERO + control.NEUTRAL_ARM_DEG * control.PULSE_PER_DEG)
        self.assertEqual(control.LEFT_NEUTRAL, expected_left)
        self.assertEqual(control.RIGHT_NEUTRAL, expected_right)

    def test_pulse_bounds_ordering(self):
        self.assertLess(control.LEFT_MIN_PULSE, control.LEFT_MAX_PULSE)
        self.assertLess(control.RIGHT_MIN_PULSE, control.RIGHT_MAX_PULSE)


class TestConnectRoMo(unittest.TestCase):
    def test_zero_gives_neutral(self):
        lp, rp, la, ra, delta = control.ConnectRoMo(0.0)
        self.assertEqual(lp, control.LEFT_NEUTRAL)
        self.assertEqual(rp, control.RIGHT_NEUTRAL)
        self.assertAlmostEqual(la, control.NEUTRAL_ARM_DEG)
        self.assertAlmostEqual(ra, control.NEUTRAL_ARM_DEG)
        self.assertAlmostEqual(delta, 0.0)

    def test_positive_cmd_right_turn_geometry(self):
        """Positive angular_velocity -> right turn: left arm up (angle down), right arm down (angle up)."""
        lp, rp, la, ra, delta = control.ConnectRoMo(10.0)
        self.assertGreater(delta, 0.0)
        self.assertLess(la, control.NEUTRAL_ARM_DEG)
        self.assertGreater(ra, control.NEUTRAL_ARM_DEG)

    def test_negative_cmd_left_turn_geometry(self):
        """Negative angular_velocity -> left turn: left arm down (angle up), right arm up (angle down)."""
        lp, rp, la, ra, delta = control.ConnectRoMo(-10.0)
        self.assertLess(delta, 0.0)
        self.assertGreater(la, control.NEUTRAL_ARM_DEG)
        self.assertLess(ra, control.NEUTRAL_ARM_DEG)

    def test_large_positive_clamps_arm_to_limits(self):
        lp, rp, la, ra, delta = control.ConnectRoMo(9999.0)
        self.assertAlmostEqual(la, control.ARM_MIN_DEG)
        self.assertAlmostEqual(ra, control.ARM_MAX_DEG)

    def test_large_negative_clamps_arm_to_limits(self):
        lp, rp, la, ra, delta = control.ConnectRoMo(-9999.0)
        self.assertAlmostEqual(la, control.ARM_MAX_DEG)
        self.assertAlmostEqual(ra, control.ARM_MIN_DEG)

    def test_output_always_within_pulse_bounds(self):
        for rate in (-200, -50, -10, 0, 10, 50, 200):
            lp, rp, *_ = control.ConnectRoMo(rate)
            self.assertGreaterEqual(lp, control.LEFT_MIN_PULSE,
                                    msg=f"left pulse out of bounds at rate={rate}")
            self.assertLessEqual(lp, control.LEFT_MAX_PULSE,
                                 msg=f"left pulse out of bounds at rate={rate}")
            self.assertGreaterEqual(rp, control.RIGHT_MIN_PULSE,
                                    msg=f"right pulse out of bounds at rate={rate}")
            self.assertLessEqual(rp, control.RIGHT_MAX_PULSE,
                                 msg=f"right pulse out of bounds at rate={rate}")


class TestInitAndSetters(unittest.TestCase):
    def setUp(self):
        self._patch = mock.patch.dict(sys.modules, {"pigpio": _FakePigpio()})
        self._patch.start()
        self.pi = control.init_control()

    def tearDown(self):
        self._patch.stop()

    def test_init_sets_zero_pulse(self):
        self.assertEqual(self.pi.pulses[control.PARAFOIL_LEFT_MOTOR_PIN], control.LEFT_ZERO_PULSE)
        self.assertEqual(self.pi.pulses[control.PARAFOIL_RIGHT_MOTOR_PIN], control.RIGHT_ZERO_PULSE)

    def test_set_zero_restores_zero_pulse(self):
        control.WriteZero(self.pi)
        self.assertEqual(self.pi.pulses[control.PARAFOIL_LEFT_MOTOR_PIN], control.LEFT_ZERO_PULSE)
        self.assertEqual(self.pi.pulses[control.PARAFOIL_RIGHT_MOTOR_PIN], control.RIGHT_ZERO_PULSE)

    def test_set_motors_off_sends_zero_width(self):
        control.WriteOff(self.pi)
        self.assertEqual(self.pi.pulses[control.PARAFOIL_LEFT_MOTOR_PIN], 0)
        self.assertEqual(self.pi.pulses[control.PARAFOIL_RIGHT_MOTOR_PIN], 0)

    def test_set_180_sends_max_angle_pulse(self):
        control.Set180(self.pi)
        self.assertEqual(self.pi.pulses[control.PARAFOIL_LEFT_MOTOR_PIN], control.LEFT_MIN_PULSE)
        self.assertEqual(self.pi.pulses[control.PARAFOIL_RIGHT_MOTOR_PIN], control.RIGHT_MAX_PULSE)

    def test_set_brake_command_sends_cmd_pulses(self):
        cmd = control.CtrlOutput(timestamp=time.monotonic(), left_pw=1700, right_pw=1300)
        control.ProducePulse(self.pi, cmd)
        self.assertEqual(self.pi.pulses[control.PARAFOIL_LEFT_MOTOR_PIN], 1700)
        self.assertEqual(self.pi.pulses[control.PARAFOIL_RIGHT_MOTOR_PIN], 1300)

    def test_gpio_pins_match_config(self):
        from lib import config
        self.assertEqual(control.PARAFOIL_LEFT_MOTOR_PIN, config.PARAFOIL_LEFT_GPIO)
        self.assertEqual(control.PARAFOIL_RIGHT_MOTOR_PIN, config.PARAFOIL_RIGHT_GPIO)

    def test_set_zero_none_no_crash(self):
        control.WriteZero(None)

    def test_set_motors_off_none_no_crash(self):
        control.WriteOff(None)

    def test_set_brake_command_none_no_crash(self):
        control.ProducePulse(None, control.WriteNeutral(time.monotonic()))


class TestNeutralCommand(unittest.TestCase):
    def test_neutral_command_pulses(self):
        cmd = control.WriteNeutral(time.monotonic())
        self.assertEqual(cmd.left_pw, control.LEFT_NEUTRAL)
        self.assertEqual(cmd.right_pw, control.RIGHT_NEUTRAL)
        self.assertAlmostEqual(cmd.left_angle_deg, control.NEUTRAL_ARM_DEG)
        self.assertAlmostEqual(cmd.right_angle_deg, control.NEUTRAL_ARM_DEG)

    def test_neutral_command_control_mode_label(self):
        cmd = control.WriteNeutral(time.monotonic(), guidance.ControlMode.FAIL)
        self.assertEqual(cmd.control_mode, guidance.ControlMode.FAIL)


class TestGuidanceCommandFromL1(unittest.TestCase):
    def test_converts_rad_to_deg(self):
        from types import SimpleNamespace
        l1 = SimpleNamespace(
            yaw_rate_cmd=0.5,
            ground_speed_mps=7.0,
            nominal=True,
            timestamp=100.0,
        )
        gcmd = control.ProduceCtrlInput(l1, 100.0)
        self.assertAlmostEqual(gcmd.angular_velocity_cmd_deg_s, math.degrees(0.5), places=5)
        self.assertTrue(gcmd.valid)
        self.assertAlmostEqual(gcmd.ground_speed_mps, 7.0)

    def test_non_nominal_l1_gives_invalid_cmd(self):
        from types import SimpleNamespace
        l1 = SimpleNamespace(
            yaw_rate_cmd=0.0,
            ground_speed_mps=0.0,
            nominal=False,
            timestamp=100.0,
        )
        gcmd = control.ProduceCtrlInput(l1, 100.0)
        self.assertFalse(gcmd.valid)


class TestControllerUpdate(unittest.TestCase):
    @staticmethod
    def _cmd(angular_velocity_deg_s=0.0, ts=100.0, speed=0.0):
        return control.CtrlInput(
            angular_velocity_cmd_deg_s=angular_velocity_deg_s,
            ground_speed_mps=speed,
            valid=True,
            timestamp=ts,
        )

    def _ctl(self, **kwargs):
        control.reset()

    def test_zero_cmd_neutral_angles(self):
        self._ctl()
        out = control.step(self._cmd(0.0), float("nan"), 100.0)
        self.assertAlmostEqual(out.left_angle_deg, control.NEUTRAL_ARM_DEG)
        self.assertAlmostEqual(out.right_angle_deg, control.NEUTRAL_ARM_DEG)
        self.assertAlmostEqual(out.delta_arm_deg, 0.0)

    def test_positive_angular_velocity_right_turn(self):
        """Positive cmd -> right turn: left_angle < NEUTRAL, right_angle > NEUTRAL."""
        self._ctl()
        out = control.step(self._cmd(10.0), float("nan"), 100.0)
        self.assertGreater(out.delta_arm_deg, 0.0)
        self.assertLess(out.left_angle_deg, control.NEUTRAL_ARM_DEG)
        self.assertGreater(out.right_angle_deg, control.NEUTRAL_ARM_DEG)

    def test_negative_angular_velocity_left_turn(self):
        """Negative cmd -> left turn: left_angle > NEUTRAL, right_angle < NEUTRAL."""
        self._ctl()
        out = control.step(self._cmd(-10.0), float("nan"), 100.0)
        self.assertLess(out.delta_arm_deg, 0.0)
        self.assertGreater(out.left_angle_deg, control.NEUTRAL_ARM_DEG)
        self.assertLess(out.right_angle_deg, control.NEUTRAL_ARM_DEG)

    def test_large_cmd_saturates_within_delta_max(self):
        self._ctl()
        out = control.step(self._cmd(999.0), float("nan"), 100.0)
        self.assertLessEqual(abs(out.delta_arm_deg), control.DELTA_ARM_MAX_DEG)
        self.assertGreaterEqual(out.left_angle_deg, control.ARM_MIN_DEG)
        self.assertLessEqual(out.left_angle_deg, control.ARM_MAX_DEG)
        self.assertTrue(out.saturated)

    def test_saturation_resets_integrator(self):
        self._ctl()
        for i in range(10):
            control.step(self._cmd(999.0, ts=100.0 + i * 0.1), 0.0, 100.0 + i * 0.1)
        self.assertAlmostEqual(control._integral_deg, 0.0)

    def test_no_gyro_preserves_guidance_control_mode(self):
        self._ctl()
        out = control.step(self._cmd(10.0), float("nan"), 100.0)
        self.assertEqual(out.control_mode, guidance.ControlMode.FAIL)
        self.assertFalse(out.sensor_valid)
        self.assertAlmostEqual(out.delta_pid_deg, 0.0)
        self.assertGreater(out.delta_arm_deg, 0.0)

    def test_valid_gyro_closed_loop(self):
        self._ctl()
        out = control.step(self._cmd(10.0), 5.0, 100.0)
        self.assertEqual(out.control_mode, guidance.ControlMode.FAIL)
        self.assertTrue(out.sensor_valid)

    def test_command_timestamp_does_not_neutralize(self):
        self._ctl()
        out = control.step(self._cmd(10.0, ts=-9999.0), 0.0, 100.0)
        self.assertTrue(out.valid)
        self.assertNotAlmostEqual(out.delta_arm_deg, 0.0)

    def test_command_timestamp_does_not_attenuate(self):
        self._ctl()
        out = control.step(self._cmd(10.0, ts=-9999.0), 0.0, 100.0)
        self.assertTrue(out.valid)
        self.assertFalse(out.gyro_rejected)
        self.assertAlmostEqual(out.angular_velocity_cmd_deg_s, 10.0)

    def test_angular_velocity_cmd_zero_stays_zero(self):
        # lat_acc fallback was removed (P8): control uses angular_velocity_cmd_deg_s as-is.
        # Guidance is responsible for converting lat_acc to angular_velocity.
        self._ctl()
        out = control.step(self._cmd(angular_velocity_deg_s=0.0, speed=4.0), float("nan"), 100.0)
        self.assertAlmostEqual(out.angular_velocity_cmd_deg_s, 0.0, places=5)

    def test_controller_reset_clears_pid(self):
        self._ctl()
        for i in range(5):
            control.step(self._cmd(5.0, ts=100.0 + i * 0.1), 0.0, 100.0 + i * 0.1)
        control.reset()
        self.assertAlmostEqual(control._integral_deg, 0.0)
        self.assertAlmostEqual(control._prev_error_deg, 0.0)
        self.assertAlmostEqual(control._prev_left_angle_deg, control.NEUTRAL_ARM_DEG)
        self.assertAlmostEqual(control._prev_right_angle_deg, control.NEUTRAL_ARM_DEG)


if __name__ == "__main__":
    unittest.main()
