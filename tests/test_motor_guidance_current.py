import math
import unittest

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
        self.assertAlmostEqual(out.distance_to_target, 2.5)
        self.assertAlmostEqual(math.degrees(out.nu), -180.0)
        self.assertLess(out.yaw_rate_cmd, 0.0)


class TestPosOnlyDrAnchorInit(unittest.TestCase):
    def setUp(self):
        guidance.reset()
        origin_lat = 37.0
        origin_lon = 127.0
        target_lon = origin_lon + math.degrees(
            100.0 / (guidance.EARTH_RADIUS_M * math.cos(math.radians(origin_lat)))
        )
        guidance.set_origin_point(origin_lat, origin_lon)
        guidance.set_target_point(origin_lat, target_lon)

    def _set_pos_only_state(self, now):
        state = guidance._STATE_t
        state.nav.control_mode = guidance.ControlMode.DR_TRACKING_OPEN
        state.gps.E = 10.0
        state.gps.N = 20.0
        state.gps.pos_ts = now
        state.gps.pos_valid = True
        state.imu.ts = now
        return state

    def test_gyrz_alone_does_not_initialize_dr_anchor(self):
        now = 100.0
        state = self._set_pos_only_state(now)
        state.imu.gyr_z = math.radians(5.0)
        state.imu.gyrz_valid = True

        l1_input = guidance.ProduceL1Input(now)

        self.assertFalse(l1_input.valid)
        self.assertEqual(l1_input.reason, "NO_HEADING_SOURCE")
        self.assertFalse(guidance.dr_anchor_valid(state.dr))

    def test_yaw_initializes_pos_only_dr_anchor(self):
        now = 100.0
        state = self._set_pos_only_state(now)
        state.imu.yaw = math.radians(30.0)
        state.imu.yaw_valid = True

        l1_input = guidance.ProduceL1Input(now)

        self.assertTrue(l1_input.valid)
        self.assertEqual(l1_input.reason, "DR_TRACKING")
        self.assertTrue(guidance.dr_anchor_valid(state.dr))
        self.assertAlmostEqual(state.dr.anchor_course, math.radians(30.0))


if __name__ == "__main__":
    unittest.main()
