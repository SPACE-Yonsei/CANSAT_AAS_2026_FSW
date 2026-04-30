import os
import unittest

from Sensor_Motor import Motor_Egg, Motor_Release
from lib import config


class TestMotorActuators(unittest.TestCase):
    def setUp(self):
        # Speed up test execution
        os.environ["BURNWIRE_DURATION_SEC"] = "0.001"
        os.environ["SOLENOID_REPEAT"] = "1"
        os.environ["SOLENOID_ON_SEC"] = "0.001"
        os.environ["SOLENOID_OFF_SEC"] = "0.001"

        # reset module runtime state
        Motor_Release.GPIO = None
        Motor_Release.BURNWIRE_READY = False
        Motor_Release.BURNWIRE_DURATION_SEC = 0.001
        Motor_Egg.GPIO = None
        Motor_Egg.SOLENOID_READY = False
        Motor_Egg.SOLENOID_REPEAT = 1
        Motor_Egg.SOLENOID_ON_SEC = 0.001
        Motor_Egg.SOLENOID_OFF_SEC = 0.001

    def test_burnwire_cycle(self):
        Motor_Release.init_burnwire()
        Motor_Release.activate_burnwire()
        gpio = Motor_Release.GPIO
        self.assertEqual(gpio.state[Motor_Release.BURNWIRE_GPIO], config.RELAY_DEACTIVATE_LEVEL)
        Motor_Release.terminate_burnwire()
        self.assertFalse(Motor_Release.BURNWIRE_READY)

    def test_solenoid_cycle(self):
        Motor_Egg.init_solenoid()
        Motor_Egg.activate_solenoid()
        gpio = Motor_Egg.GPIO
        self.assertEqual(gpio.state[Motor_Egg.SOLENOID_GPIO], config.RELAY_DEACTIVATE_LEVEL)
        Motor_Egg.terminate_solenoid()
        self.assertFalse(Motor_Egg.SOLENOID_READY)


if __name__ == "__main__":
    unittest.main()
