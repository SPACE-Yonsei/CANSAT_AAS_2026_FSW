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
        self.assertNotEqual(out.reason, "TARGET_REACHED")
        self.assertAlmostEqual(out.distance_to_target, 2.5)
        self.assertAlmostEqual(math.degrees(out.nu), -180.0)
        self.assertLess(out.yaw_rate_cmd, 0.0)


class TestPosOnlyDrAnchorInit(unittest.TestCase):
    def setUp(self):
        guidance.reset()
        mission = guidance._MISSION_t
        mission.origin_ready = True
        mission.target_ready = True
        mission.target_E = 100.0
        mission.target_N = 0.0

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
        self.assertFalse(guidance.dr_is_valid(state.dr))

    def test_yaw_initializes_pos_only_dr_anchor(self):
        now = 100.0
        state = self._set_pos_only_state(now)
        state.imu.yaw = math.radians(30.0)
        state.imu.yaw_valid = True

        l1_input = guidance.ProduceL1Input(now)

        self.assertTrue(l1_input.valid)
        self.assertEqual(l1_input.reason, "DR_TRACKING")
        self.assertTrue(guidance.dr_is_valid(state.dr))
        self.assertAlmostEqual(state.dr.anchor_course, math.radians(30.0))


class TestCurrentDetumblingOutput(unittest.TestCase):
    def _detumble_output(self, gyrz_deg_s):
        return control.ProduceDetumbleOutput(100.0, gyrz_deg_s)

    def test_left_rotation_commands_max_right_damping(self):
        out = self._detumble_output(-250.0)

        self.assertEqual(out.control_mode, guidance.ControlMode.DETUMBLING)
        self.assertAlmostEqual(out.delta_arm_deg, control.DELTA_ARM_MAX_DEG)
        self.assertAlmostEqual(
            out.left_angle_deg,
            control.NEUTRAL_ARM_DEG - control.DELTA_ARM_MAX_DEG / 2.0,
        )
        self.assertAlmostEqual(out.right_angle_deg, control.ARM_MAX_DEG)

    def test_right_rotation_commands_max_left_damping(self):
        out = self._detumble_output(250.0)

        self.assertEqual(out.control_mode, guidance.ControlMode.DETUMBLING)
        self.assertAlmostEqual(out.delta_arm_deg, -control.DELTA_ARM_MAX_DEG)
        self.assertAlmostEqual(out.left_angle_deg, control.ARM_MAX_DEG)
        self.assertAlmostEqual(
            out.right_angle_deg,
            control.NEUTRAL_ARM_DEG - control.DELTA_ARM_MAX_DEG / 2.0,
        )


if __name__ == "__main__":
    unittest.main()
