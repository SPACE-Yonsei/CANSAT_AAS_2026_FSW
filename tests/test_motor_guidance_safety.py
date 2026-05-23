"""Safety gate tests: guidance inactive when target/origin missing or path degenerate."""

import math
import time
import unittest

from Sensor_Motor import guidance
from Sensor_Motor.guidance import SensorQuality, ControlMode

ORIGIN_LAT = 37.55
ORIGIN_LON = 126.95
TARGET_LAT = 37.56
TARGET_LON = 126.96


def _active_input(pos_n=100.0, pos_e=0.0):
    inp = guidance.L1Input()
    inp.pos_N = pos_n
    inp.pos_E = pos_e
    inp.pos_quality = SensorQuality.FRESH
    inp.course = math.radians(45.0)
    inp.motion_quality = SensorQuality.FRESH
    inp.ground_speed_mps = 8.0
    inp.gyrz = 0.1
    inp.gyrz_quality = SensorQuality.FRESH
    inp.control_mode = ControlMode.NOMINAL_CLOSED_LOOP
    inp.origin_lat = ORIGIN_LAT
    inp.origin_lon = ORIGIN_LON
    return inp


class TestGuidanceSafetyGates(unittest.TestCase):
    def test_no_target_returns_inactive(self):
        out = guidance.ProduceL1Output(
            _active_input(), ControlMode.NOMINAL_CLOSED_LOOP,
            ORIGIN_LAT, ORIGIN_LON, None, None, time.monotonic(),
        )
        self.assertFalse(out.nominal)
        self.assertEqual(out.reason, "NO_POSITION")

    def test_partial_target_lon_missing_returns_inactive(self):
        out = guidance.ProduceL1Output(
            _active_input(), ControlMode.NOMINAL_CLOSED_LOOP,
            ORIGIN_LAT, ORIGIN_LON, TARGET_LAT, None, time.monotonic(),
        )
        self.assertFalse(out.nominal)
        self.assertEqual(out.reason, "NO_POSITION")

    def test_no_origin_returns_inactive(self):
        out = guidance.ProduceL1Output(
            _active_input(), ControlMode.NOMINAL_CLOSED_LOOP,
            None, None, TARGET_LAT, TARGET_LON, time.monotonic(),
        )
        self.assertFalse(out.nominal)
        self.assertEqual(out.reason, "NO_POSITION")

    def test_fail_mode_returns_inactive(self):
        out = guidance.ProduceL1Output(
            _active_input(), ControlMode.FAIL,
            ORIGIN_LAT, ORIGIN_LON, TARGET_LAT, TARGET_LON, time.monotonic(),
        )
        self.assertFalse(out.nominal)
        self.assertEqual(out.reason, "FAIL")

    def test_origin_equals_target_returns_invalid_path(self):
        out = guidance.ProduceL1Output(
            _active_input(), ControlMode.NOMINAL_CLOSED_LOOP,
            ORIGIN_LAT, ORIGIN_LON, ORIGIN_LAT, ORIGIN_LON, time.monotonic(),
        )
        self.assertFalse(out.nominal)
        self.assertEqual(out.reason, "NO_POSITION")

    def test_inactive_output_has_zero_angular_velocity(self):
        out = guidance.ProduceL1Output(
            _active_input(), ControlMode.NOMINAL_CLOSED_LOOP,
            ORIGIN_LAT, ORIGIN_LON, None, None, time.monotonic(),
        )
        self.assertAlmostEqual(out.angular_velocity_cmd_rad_s, 0.0)

    def test_inactive_output_has_zero_lat_acc(self):
        out = guidance.ProduceL1Output(
            _active_input(), ControlMode.NOMINAL_CLOSED_LOOP,
            ORIGIN_LAT, ORIGIN_LON, None, None, time.monotonic(),
        )
        self.assertAlmostEqual(out.lat_acc_cmd_mps2, 0.0)

    def test_no_origin_in_produce_l1_input_causes_fail(self):
        """Without origin, position cannot be computed → FAIL mode."""
        from types import SimpleNamespace
        now = time.monotonic()
        gps = SimpleNamespace(
            lat=ORIGIN_LAT + 0.001, lon=ORIGIN_LON + 0.001,
            course_rad=0.0, speed_mps=8.0,
            pos_health=True, motion_health=True,
            pos_ts=now, motion_ts=now,
        )
        imu = SimpleNamespace(gyrz_rad_s=0.1, ts=now, health=True)
        baro = SimpleNamespace(alt_m=100.0, ts=now, health=True)
        _, mode = guidance.ProduceL1Input(
            gps, imu, baro,
            None, None, TARGET_LAT, TARGET_LON, now,
        )
        self.assertEqual(mode, ControlMode.FAIL)


if __name__ == "__main__":
    unittest.main()
