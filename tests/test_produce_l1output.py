"""ProduceL1Output: per-mode yaw-rate limit, pid_enabled, confidence scaling.

Covers the spec cases 1-7: GPS closed/open, DR_M / DR_PM limits + confidence
scaling, low-confidence gate, nu clamp, and V-too-small handling.
"""
import math
import unittest

from Sensor_Motor import guidance
from lib import config

ControlMode = guidance.ControlMode
DRMethod = guidance.DRMethod


def _l1in(mode, *, E=0.0, N=0.0, V=5.0, course=0.0,
          bearing_deg=0.0, dist=100.0, confidence=1.0, valid=True,
          reason="TEST"):
    """Build a valid-by-default L1Input with target placed at bearing_deg/dist."""
    target_E = E + dist * math.sin(math.radians(bearing_deg))
    target_N = N + dist * math.cos(math.radians(bearing_deg))
    return guidance.L1Input(
        valid=valid, reason=reason, control_mode=mode,
        dr_method=DRMethod.NONE, confidence=confidence,
        E=E, N=N, V=V, course=course,
        vE=V * math.sin(course), vN=V * math.cos(course),
        target_E=target_E, target_N=target_N,
    )


class TestProduceL1Output(unittest.TestCase):
    # ── Case 1: GPS_TRACKING_CLOSED ─────────────────────────────────────────
    def test_case_1_gps_closed(self):
        out = guidance.ProduceL1Output(_l1in(ControlMode.GPS_TRACKING_CLOSED, bearing_deg=30.0))
        self.assertTrue(out.valid and out.control_valid)
        self.assertTrue(out.pid_enabled)
        self.assertAlmostEqual(out.yaw_rate_limit_dps,
                               config.GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS, places=6)

    # ── Case 2: GPS_TRACKING_OPEN ───────────────────────────────────────────
    def test_case_2_gps_open(self):
        out = guidance.ProduceL1Output(_l1in(ControlMode.GPS_TRACKING_OPEN, bearing_deg=30.0))
        self.assertTrue(out.valid)
        self.assertFalse(out.pid_enabled)
        self.assertAlmostEqual(out.yaw_rate_limit_dps,
                               config.GPS_TRACKING_OPEN_YAW_RATE_LIMIT_DPS, places=6)

    # ── Case 3: DR_M_GB_CLOSED — limit + confidence scaling ─────────────────
    def test_case_3_dr_m_gb_closed(self):
        full = guidance.ProduceL1Output(
            _l1in(ControlMode.DR_M_GB_CLOSED, bearing_deg=30.0, confidence=1.0))
        half = guidance.ProduceL1Output(
            _l1in(ControlMode.DR_M_GB_CLOSED, bearing_deg=30.0, confidence=0.5))
        self.assertTrue(half.valid)
        self.assertTrue(half.pid_enabled)
        self.assertAlmostEqual(half.yaw_rate_limit_dps,
                               config.DR_M_GB_YAW_RATE_LIMIT_DPS, places=6)
        # confidence 0.5 halves the (un-clamped) command
        self.assertAlmostEqual(half.yaw_rate_cmd, 0.5 * full.yaw_rate_cmd, places=9)
        self.assertGreater(abs(full.yaw_rate_cmd), 0.0)

    # ── Case 4: DR_PM_YB_OPEN — limit + confidence scaling, no PID ───────────
    def test_case_4_dr_pm_yb_open(self):
        out = guidance.ProduceL1Output(
            _l1in(ControlMode.DR_PM_YB_OPEN, bearing_deg=30.0, confidence=0.5))
        self.assertTrue(out.valid)
        self.assertFalse(out.pid_enabled)
        self.assertAlmostEqual(out.yaw_rate_limit_dps,
                               config.DR_PM_YB_YAW_RATE_LIMIT_DPS, places=6)
        self.assertGreater(abs(out.yaw_rate_cmd), 0.0)

    # ── Case 5: confidence below threshold → invalid + reason ───────────────
    def test_case_5_low_confidence(self):
        out = guidance.ProduceL1Output(
            _l1in(ControlMode.DR_PM_G_CLOSED, bearing_deg=30.0,
                  confidence=config.DR_MIN_CONFIDENCE_FOR_CONTROL - 0.05))
        self.assertFalse(out.valid)
        self.assertFalse(out.control_valid)
        self.assertEqual(out.reason, "LOW_DR_CONFIDENCE")
        self.assertEqual(out.yaw_rate_cmd, 0.0)

    # ── Case 6: nu > 90deg → clamped in command ─────────────────────────────
    def test_case_6_nu_clamp(self):
        out = guidance.ProduceL1Output(
            _l1in(ControlMode.GPS_TRACKING_CLOSED, V=5.0, bearing_deg=135.0))
        # raw nu exceeds 90 deg
        self.assertGreater(abs(out.nu), math.pi / 2.0)
        # command uses sin(clamp(nu, ±pi/2)) = 1 → 2*V/L (below GPS closed limit)
        expected = 2.0 * 5.0 / config.L_GAIN_M  # sin clamped == 1.0, conf == 1.0
        lim = math.radians(config.GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS)
        self.assertAlmostEqual(out.yaw_rate_cmd, min(expected, lim), places=9)

    # ── Case 7: V too small → invalid ───────────────────────────────────────
    def test_case_7_v_too_small(self):
        out = guidance.ProduceL1Output(
            _l1in(ControlMode.GPS_TRACKING_CLOSED, V=config.V_MIN_MPS - 0.1, bearing_deg=30.0))
        self.assertFalse(out.valid)
        self.assertEqual(out.reason, "V_TOO_SMALL")

    # ── Guards: invalid input, FAIL, DETUMBLING, target reached ─────────────
    def test_invalid_input_passthrough(self):
        out = guidance.ProduceL1Output(
            _l1in(ControlMode.GPS_TRACKING_CLOSED, valid=False, reason="GPS_NAV_INVALID"))
        self.assertFalse(out.valid)
        self.assertEqual(out.reason, "GPS_NAV_INVALID")

    def test_fail_mode(self):
        out = guidance.ProduceL1Output(_l1in(ControlMode.FAIL))
        self.assertFalse(out.valid)
        self.assertEqual(out.reason, "FAIL")

    def test_detumbling_mode(self):
        out = guidance.ProduceL1Output(_l1in(ControlMode.DETUMBLING))
        self.assertTrue(out.control_valid)
        self.assertFalse(out.nominal)
        self.assertFalse(out.pid_enabled)
        self.assertEqual(out.reason, "DETUMBLING")
        self.assertEqual(out.yaw_rate_cmd, 0.0)

    def test_target_reached(self):
        out = guidance.ProduceL1Output(
            _l1in(ControlMode.GPS_TRACKING_CLOSED, dist=config.TARGET_RADIUS_M - 1.0))
        self.assertFalse(out.valid)
        self.assertEqual(out.reason, "TARGET_REACHED")
        self.assertEqual(out.yaw_rate_cmd, 0.0)


if __name__ == "__main__":
    unittest.main()
