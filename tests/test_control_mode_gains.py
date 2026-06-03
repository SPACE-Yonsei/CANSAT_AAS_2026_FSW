"""control.step: per-mode gain / FF-scale / PID-enable behavior.

Covers spec cases 1-7 for the new ControlMode taxonomy. All angular-rate
quantities are deg/s (ProduceCtrlInput converts L1Output rad/s → deg/s).
"""
import math
import unittest

from Sensor_Motor import control
from Sensor_Motor.guidance import ControlMode
from lib import config

CMD_DPS = 20.0  # > FF and PID deadbands (5 dps), below GPS limit (60)


def _cmd(mode, *, pid_enabled, valid=True, cmd_dps=CMD_DPS):
    return control.CtrlInput(
        angular_velocity_cmd_deg_s=cmd_dps,
        ground_speed_mps=5.0,
        valid=valid,
        timestamp=0.0,
        pid_enabled=pid_enabled,
        control_mode=mode,
    )


class TestControlModeGains(unittest.TestCase):
    def setUp(self):
        control.reset()

    def _step(self, mode, *, pid_enabled, meas=0.0, valid=True, cmd_dps=CMD_DPS, now=1.0):
        return control.ProduceCtrlOutput(_cmd(mode, pid_enabled=pid_enabled, valid=valid, cmd_dps=cmd_dps),
                            meas, now)

    # ── Case 1: GPS_TRACKING_CLOSED → FF + PID ──────────────────────────────
    def test_case_1_gps_closed_ff_plus_pid(self):
        out = self._step(ControlMode.GPS_TRACKING_CLOSED, pid_enabled=True, meas=0.0)
        self.assertTrue(out.valid)
        self.assertTrue(out.sensor_valid)
        self.assertEqual(out.ff_scale, 1.0)
        self.assertNotEqual(out.delta_ff_deg, 0.0)
        self.assertNotEqual(out.delta_pid_deg, 0.0)
        self.assertAlmostEqual(out.kp_used, config.KP_GPS_CLOSED, places=9)

    # ── Case 2: GPS_TRACKING_OPEN → FF only ─────────────────────────────────
    def test_case_2_gps_open_ff_only(self):
        out = self._step(ControlMode.GPS_TRACKING_OPEN, pid_enabled=False, meas=0.0)
        self.assertTrue(out.valid)
        self.assertEqual(out.ff_scale, 1.0)
        self.assertNotEqual(out.delta_ff_deg, 0.0)
        self.assertEqual(out.delta_pid_deg, 0.0)

    # ── Case 3: DR_M_GB_CLOSED → FF scaled + PID (KP_DR_M_CLOSED) ────────────
    def test_case_3_dr_m_gb_closed(self):
        gps = self._step(ControlMode.GPS_TRACKING_CLOSED, pid_enabled=True, meas=0.0)
        control.reset()
        out = self._step(ControlMode.DR_M_GB_CLOSED, pid_enabled=True, meas=0.0)
        self.assertTrue(out.valid)
        self.assertAlmostEqual(out.ff_scale, config.DR_M_FF_SCALE, places=9)
        self.assertAlmostEqual(out.kp_used, config.KP_DR_M_CLOSED, places=9)
        self.assertNotEqual(out.delta_pid_deg, 0.0)
        # FF rides the same curve at reduced authority
        self.assertAlmostEqual(out.delta_ff_deg, gps.delta_ff_deg * config.DR_M_FF_SCALE, places=6)

    # ── Case 4: DR_PM_GB_CLOSED → FF scaled + PID (KP_DR_PM_CLOSED) ──────────
    def test_case_4_dr_pm_gb_closed(self):
        out = self._step(ControlMode.DR_PM_GB_CLOSED, pid_enabled=True, meas=0.0)
        self.assertTrue(out.valid)
        self.assertAlmostEqual(out.ff_scale, config.DR_PM_FF_SCALE, places=9)
        self.assertAlmostEqual(out.kp_used, config.KP_DR_PM_CLOSED, places=9)
        self.assertNotEqual(out.delta_pid_deg, 0.0)

    # ── Case 5: DR_PM_YB_OPEN → FF only, no PID ─────────────────────────────
    def test_case_5_dr_pm_yb_open_ff_only(self):
        out = self._step(ControlMode.DR_PM_YB_OPEN, pid_enabled=False, meas=0.0)
        self.assertTrue(out.valid)
        self.assertAlmostEqual(out.ff_scale, config.DR_PM_FF_SCALE, places=9)
        self.assertNotEqual(out.delta_ff_deg, 0.0)
        self.assertEqual(out.delta_pid_deg, 0.0)

    # ── Case 6: FAIL → neutral ──────────────────────────────────────────────
    def test_case_6_fail_neutral(self):
        out = self._step(ControlMode.FAIL, pid_enabled=True, meas=0.0)
        self.assertFalse(out.valid)
        self.assertEqual(out.reason, "FAIL")
        self.assertEqual(out.left_pw, control.LEFT_NEUTRAL)
        self.assertEqual(out.right_pw, control.RIGHT_NEUTRAL)
        self.assertEqual(out.delta_arm_deg, 0.0)

    # ── Case 7: meas NaN in a closed mode → PID off, FF only ────────────────
    def test_case_7_nan_meas_closed_ff_only(self):
        out = self._step(ControlMode.GPS_TRACKING_CLOSED, pid_enabled=True,
                         meas=float("nan"))
        self.assertTrue(out.valid)          # FF still steers
        self.assertFalse(out.sensor_valid)
        self.assertEqual(out.delta_pid_deg, 0.0)
        self.assertNotEqual(out.delta_ff_deg, 0.0)

    # ── Guard: invalid command → neutral ────────────────────────────────────
    def test_invalid_cmd_neutral(self):
        out = self._step(ControlMode.GPS_TRACKING_CLOSED, pid_enabled=True, valid=False)
        self.assertFalse(out.valid)
        self.assertEqual(out.reason, "INVALID_CMD")
        self.assertEqual(out.left_pw, control.LEFT_NEUTRAL)

    # ── Guard: DR_M weaker FF than GPS, DR_PM weaker than DR_M ───────────────
    def test_ff_scale_ordering(self):
        self.assertGreater(config.DR_M_FF_SCALE, config.DR_PM_FF_SCALE)
        self.assertGreater(1.0, config.DR_M_FF_SCALE)
        self.assertGreater(config.KP_GPS_CLOSED, config.KP_DR_M_CLOSED)
        self.assertGreater(config.KP_DR_M_CLOSED, config.KP_DR_PM_CLOSED)


if __name__ == "__main__":
    unittest.main()
