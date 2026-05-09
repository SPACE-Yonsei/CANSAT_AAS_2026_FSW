"""Tests for Motor_Release and Motor_Egg: dummy backend, try/finally safety."""

import os
import unittest

from Sensor_Motor import Motor_Egg, Motor_Release
from lib import config


def _reset_release():
    Motor_Release.GPIO = None
    Motor_Release.BURNWIRE_READY = False
    Motor_Release.BURNWIRE_DURATION_SEC = 0.001


def _reset_egg():
    Motor_Egg.GPIO = None
    Motor_Egg.SOLENOID_READY = False
    Motor_Egg.SOLENOID_REPEAT  = 1
    Motor_Egg.SOLENOID_ON_SEC  = 0.001
    Motor_Egg.SOLENOID_OFF_SEC = 0.001


class TestBurnwire(unittest.TestCase):
    def setUp(self):
        _reset_release()

    def test_init_creates_dummy_gpio(self):
        Motor_Release.init_burnwire()
        self.assertTrue(Motor_Release.BURNWIRE_READY)
        self.assertIsNotNone(Motor_Release.GPIO)

    def test_activate_deactivates_after_burn(self):
        Motor_Release.init_burnwire()
        Motor_Release.activate_burnwire()
        gpio = Motor_Release.GPIO
        self.assertEqual(gpio.state[Motor_Release.BURNWIRE_GPIO], Motor_Release._RELEASE_DEACTIVATE_LEVEL)

    def test_terminate_clears_ready(self):
        Motor_Release.init_burnwire()
        Motor_Release.terminate_burnwire()
        self.assertFalse(Motor_Release.BURNWIRE_READY)

    def test_activate_without_init_auto_inits(self):
        self.assertFalse(Motor_Release.BURNWIRE_READY)
        Motor_Release.activate_burnwire()   # must not raise
        gpio = Motor_Release.GPIO
        self.assertEqual(gpio.state[Motor_Release.BURNWIRE_GPIO], Motor_Release._RELEASE_DEACTIVATE_LEVEL)

    def test_deactivate_guaranteed_on_exception(self):
        """Simulate an exception during sleep: relay must still be deactivated."""
        Motor_Release.init_burnwire()
        gpio = Motor_Release.GPIO

        original_sleep = __import__("time").sleep

        def raise_after_activate(duration):
            # Allow very short sleeps (atexit, etc.) through
            if duration > 0.0005:
                raise RuntimeError("simulated interrupt")
            original_sleep(duration)

        import time as _time
        import Sensor_Motor.Motor_Release as _mr
        _mr_sleep = _mr.__dict__.get("time")

        import unittest.mock as mock
        with mock.patch("Sensor_Motor.Motor_Release.time") as mock_time:
            mock_time.sleep.side_effect = raise_after_activate
            try:
                Motor_Release.activate_burnwire()
            except RuntimeError:
                pass
        # Relay must be deactivated despite the exception
        self.assertEqual(gpio.state[Motor_Release.BURNWIRE_GPIO], Motor_Release._RELEASE_DEACTIVATE_LEVEL)


class TestSolenoid(unittest.TestCase):
    def setUp(self):
        _reset_egg()

    def test_init_creates_dummy_gpio(self):
        Motor_Egg.init_solenoid()
        self.assertTrue(Motor_Egg.SOLENOID_READY)
        self.assertIsNotNone(Motor_Egg.GPIO)

    def test_activate_deactivates_after_all_pulses(self):
        Motor_Egg.init_solenoid()
        Motor_Egg.activate_solenoid()
        gpio = Motor_Egg.GPIO
        self.assertEqual(gpio.state[Motor_Egg.SOLENOID_GPIO], config.RELAY_DEACTIVATE_LEVEL)

    def test_activate_multiple_repeats(self):
        Motor_Egg.SOLENOID_REPEAT = 3
        Motor_Egg.init_solenoid()
        Motor_Egg.activate_solenoid()
        gpio = Motor_Egg.GPIO
        self.assertEqual(gpio.state[Motor_Egg.SOLENOID_GPIO], config.RELAY_DEACTIVATE_LEVEL)

    def test_terminate_clears_ready(self):
        Motor_Egg.init_solenoid()
        Motor_Egg.terminate_solenoid()
        self.assertFalse(Motor_Egg.SOLENOID_READY)

    def test_activate_without_init_auto_inits(self):
        self.assertFalse(Motor_Egg.SOLENOID_READY)
        Motor_Egg.activate_solenoid()
        gpio = Motor_Egg.GPIO
        self.assertEqual(gpio.state[Motor_Egg.SOLENOID_GPIO], config.RELAY_DEACTIVATE_LEVEL)

    def test_deactivate_guaranteed_on_exception(self):
        """Relay must be deactivated even if sleep raises during a pulse."""
        Motor_Egg.init_solenoid()
        gpio = Motor_Egg.GPIO

        import unittest.mock as mock
        call_count = [0]

        def raise_on_second(duration):
            call_count[0] += 1
            if call_count[0] == 1:
                raise RuntimeError("simulated interrupt")

        with mock.patch("Sensor_Motor.Motor_Egg.time") as mock_time:
            mock_time.sleep.side_effect = raise_on_second
            try:
                Motor_Egg.activate_solenoid()
            except RuntimeError:
                pass
        self.assertEqual(gpio.state[Motor_Egg.SOLENOID_GPIO], config.RELAY_DEACTIVATE_LEVEL)


if __name__ == "__main__":
    unittest.main()
