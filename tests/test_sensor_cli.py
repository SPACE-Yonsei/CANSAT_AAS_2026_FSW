import os
import unittest

from lib import sensor_cli


class TestSensorCli(unittest.TestCase):
    def test_cli_period(self):
        os.environ["SENSOR_CLI_HZ"] = "10"
        self.assertAlmostEqual(sensor_cli.cli_period_sec(), 0.1)
        os.environ["SENSOR_CLI_HZ"] = "5"
        self.assertAlmostEqual(sensor_cli.cli_period_sec(), 0.2)
        os.environ.pop("SENSOR_CLI_HZ", None)
        self.assertAlmostEqual(sensor_cli.cli_period_sec(), 0.1)


if __name__ == "__main__":
    unittest.main()
