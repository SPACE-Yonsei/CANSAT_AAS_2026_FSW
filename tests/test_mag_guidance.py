"""Unit tests for Sensor_Motor/mag_guidance.py (GPS-free bearing-hold guidance)."""

import math
import unittest

from Sensor_Motor.mag_guidance import (
    MagGuidanceConfig,
    MagGuidanceInput,
    MagGuidanceOutput,
    ProduceMagGuidance,
    MODE_NOMINAL,
    MODE_DEGRADED,
    MODE_NO_BEARING,
    MODE_IMU_FAIL,
)


# ── helpers ───────────────────────────────────────────────────────────────────
_NOW = 1000.0  # fixed monotonic reference


def _inp(
    yaw_deg=0.0,
    gyrz_deg_s=0.0,
    alt_m=100.0,
    bearing_deg=0.0,
    imu_age_s=0.1,
    baro_age_s=0.1,
    imu_health=1,
    baro_health=1,
    bearing_nan=False,
    yaw_none=False,
):
    """Build a MagGuidanceInput with sensible defaults."""
    return MagGuidanceInput(
        yaw_rad=None if yaw_none else math.radians(yaw_deg),
        gyrz_rad_s=math.radians(gyrz_deg_s),
        alt_m=alt_m,
        target_bearing_rad=float("nan") if bearing_nan else math.radians(bearing_deg),
        timestamp=_NOW,
        imu_ts=_NOW - imu_age_s,
        baro_ts=_NOW - baro_age_s,
        imu_health=imu_health,
        baro_health=baro_health,
    )


def _run(inp, **cfg_kwargs):
    cfg = MagGuidanceConfig(**cfg_kwargs) if cfg_kwargs else MagGuidanceConfig()
    return ProduceMagGuidance(inp, cfg)


# ── failure modes ─────────────────────────────────────────────────────────────
class TestFailureModes(unittest.TestCase):

    def test_no_bearing_returns_no_bearing_mode(self):
        out = _run(_inp(bearing_nan=True))
        self.assertEqual(out.mode, MODE_NO_BEARING)
        self.assertFalse(out.valid)
        self.assertAlmostEqual(out.angular_velocity_cmd_deg_s, 0.0)

    def test_imu_health_zero_returns_imu_fail(self):
        out = _run(_inp(imu_health=0))
        self.assertEqual(out.mode, MODE_IMU_FAIL)
        self.assertFalse(out.valid)

    def test_yaw_none_returns_imu_fail(self):
        out = _run(_inp(yaw_none=True))
        self.assertEqual(out.mode, MODE_IMU_FAIL)
        self.assertFalse(out.valid)

    def test_imu_stale_returns_imu_fail(self):
        cfg = MagGuidanceConfig(imu_timeout_s=0.5)
        out = ProduceMagGuidance(_inp(imu_age_s=1.0), cfg)
        self.assertEqual(out.mode, MODE_IMU_FAIL)
        self.assertFalse(out.valid)

    def test_imu_just_within_timeout_is_valid(self):
        cfg = MagGuidanceConfig(imu_timeout_s=1.0)
        out = ProduceMagGuidance(_inp(imu_age_s=0.99), cfg)
        self.assertTrue(out.valid)

    def test_bearing_nan_takes_priority_over_imu_fail(self):
        """NaN bearing should short-circuit before IMU check."""
        out = _run(_inp(bearing_nan=True, imu_health=0))
        self.assertEqual(out.mode, MODE_NO_BEARING)


# ── nominal / degraded modes ───────────────────────────────────────────────────
class TestModeSelection(unittest.TestCase):

    def test_both_fresh_gives_nominal(self):
        out = _run(_inp(baro_health=1, baro_age_s=0.1))
        self.assertEqual(out.mode, MODE_NOMINAL)
        self.assertTrue(out.valid)

    def test_baro_stale_gives_degraded_but_still_valid(self):
        cfg = MagGuidanceConfig(baro_timeout_s=0.5)
        out = ProduceMagGuidance(_inp(baro_age_s=1.0), cfg)
        self.assertEqual(out.mode, MODE_DEGRADED)
        self.assertTrue(out.valid)   # steering continues without baro

    def test_baro_unhealthy_gives_degraded(self):
        out = _run(_inp(baro_health=0))
        self.assertEqual(out.mode, MODE_DEGRADED)
        self.assertTrue(out.valid)


# ── turn direction ─────────────────────────────────────────────────────────────
class TestTurnDirection(unittest.TestCase):

    def test_zero_error_gives_zero_cmd(self):
        out = _run(_inp(yaw_deg=90.0, bearing_deg=90.0, gyrz_deg_s=0.0))
        self.assertAlmostEqual(out.angular_velocity_cmd_deg_s, 0.0, places=6)
        self.assertAlmostEqual(out.heading_error_deg, 0.0, places=6)

    def test_positive_error_gives_right_turn(self):
        """bearing > yaw (target clockwise of current) → cmd > 0 (right turn)."""
        out = _run(_inp(yaw_deg=60.0, bearing_deg=90.0, gyrz_deg_s=0.0))
        self.assertGreater(out.angular_velocity_cmd_deg_s, 0.0)
        self.assertAlmostEqual(out.heading_error_deg, 30.0, places=4)

    def test_negative_error_gives_left_turn(self):
        """bearing < yaw (target counter-clockwise) → cmd < 0 (left turn)."""
        out = _run(_inp(yaw_deg=120.0, bearing_deg=90.0, gyrz_deg_s=0.0))
        self.assertLess(out.angular_velocity_cmd_deg_s, 0.0)
        self.assertAlmostEqual(out.heading_error_deg, -30.0, places=4)

    def test_wrap_pi_right_through_north(self):
        """yaw=350°, bearing=10° → error=+20° → right turn."""
        out = _run(_inp(yaw_deg=350.0, bearing_deg=10.0, gyrz_deg_s=0.0))
        self.assertGreater(out.angular_velocity_cmd_deg_s, 0.0)
        self.assertAlmostEqual(out.heading_error_deg, 20.0, places=4)

    def test_wrap_pi_left_through_north(self):
        """yaw=10°, bearing=350° → error=-20° → left turn."""
        out = _run(_inp(yaw_deg=10.0, bearing_deg=350.0, gyrz_deg_s=0.0))
        self.assertLess(out.angular_velocity_cmd_deg_s, 0.0)
        self.assertAlmostEqual(out.heading_error_deg, -20.0, places=4)

    def test_180_degree_error_saturates_cmd(self):
        """180° error hits max_cmd clamp."""
        cfg = MagGuidanceConfig(max_cmd_deg_s=45.0)
        out = ProduceMagGuidance(_inp(yaw_deg=0.0, bearing_deg=180.0), cfg)
        self.assertAlmostEqual(abs(out.angular_velocity_cmd_deg_s), 45.0, places=4)


# ── proportional gain ─────────────────────────────────────────────────────────
class TestProportionalGain(unittest.TestCase):

    def test_cmd_proportional_to_error(self):
        """30° error produces twice the cmd of 15° error (no gyrz, no clamp)."""
        cfg = MagGuidanceConfig(Kp_rad_s_per_rad=1.0, Kd_damping=0.0, max_cmd_deg_s=180.0)
        out30 = ProduceMagGuidance(_inp(yaw_deg=0.0, bearing_deg=30.0), cfg)
        out15 = ProduceMagGuidance(_inp(yaw_deg=0.0, bearing_deg=15.0), cfg)
        self.assertAlmostEqual(
            out30.angular_velocity_cmd_deg_s,
            2.0 * out15.angular_velocity_cmd_deg_s,
            places=4,
        )

    def test_cmd_clamped_to_max(self):
        cfg = MagGuidanceConfig(max_cmd_deg_s=20.0)
        out = ProduceMagGuidance(_inp(yaw_deg=0.0, bearing_deg=90.0), cfg)
        self.assertAlmostEqual(out.angular_velocity_cmd_deg_s, 20.0, places=4)

    def test_cmd_clamped_negative_max(self):
        cfg = MagGuidanceConfig(max_cmd_deg_s=20.0)
        out = ProduceMagGuidance(_inp(yaw_deg=90.0, bearing_deg=0.0), cfg)
        self.assertAlmostEqual(out.angular_velocity_cmd_deg_s, -20.0, places=4)


# ── derivative damping ────────────────────────────────────────────────────────
class TestDerivativeDamping(unittest.TestCase):

    def test_gyrz_in_turn_direction_reduces_cmd(self):
        """Turning right (gyrz > 0) while error > 0: damping reduces cmd."""
        cfg = MagGuidanceConfig(Kp_rad_s_per_rad=1.0, Kd_damping=1.0, max_cmd_deg_s=180.0)
        no_damp = ProduceMagGuidance(
            _inp(yaw_deg=60.0, bearing_deg=90.0, gyrz_deg_s=0.0), cfg
        )
        with_damp = ProduceMagGuidance(
            _inp(yaw_deg=60.0, bearing_deg=90.0, gyrz_deg_s=15.0), cfg
        )
        self.assertLess(with_damp.angular_velocity_cmd_deg_s,
                        no_damp.angular_velocity_cmd_deg_s)

    def test_gyrz_opposite_turn_direction_increases_cmd(self):
        """Drifting left (gyrz < 0) while error > 0: damping increases cmd."""
        cfg = MagGuidanceConfig(Kp_rad_s_per_rad=1.0, Kd_damping=1.0, max_cmd_deg_s=180.0)
        no_damp = ProduceMagGuidance(
            _inp(yaw_deg=60.0, bearing_deg=90.0, gyrz_deg_s=0.0), cfg
        )
        opp_damp = ProduceMagGuidance(
            _inp(yaw_deg=60.0, bearing_deg=90.0, gyrz_deg_s=-15.0), cfg
        )
        self.assertGreater(opp_damp.angular_velocity_cmd_deg_s,
                           no_damp.angular_velocity_cmd_deg_s)

    def test_kd_zero_ignores_gyrz(self):
        cfg = MagGuidanceConfig(Kp_rad_s_per_rad=1.0, Kd_damping=0.0, max_cmd_deg_s=180.0)
        out_a = ProduceMagGuidance(_inp(yaw_deg=60.0, bearing_deg=90.0, gyrz_deg_s=0.0), cfg)
        out_b = ProduceMagGuidance(_inp(yaw_deg=60.0, bearing_deg=90.0, gyrz_deg_s=30.0), cfg)
        self.assertAlmostEqual(
            out_a.angular_velocity_cmd_deg_s,
            out_b.angular_velocity_cmd_deg_s,
            places=6,
        )


# ── output fields ─────────────────────────────────────────────────────────────
class TestOutputFields(unittest.TestCase):

    def test_output_echoes_timestamp(self):
        out = _run(_inp())
        self.assertAlmostEqual(out.timestamp, _NOW)

    def test_output_echoes_bearing_deg(self):
        out = _run(_inp(yaw_deg=0.0, bearing_deg=127.5))
        self.assertAlmostEqual(out.target_bearing_deg, 127.5, places=4)

    def test_output_echoes_current_yaw_deg(self):
        out = _run(_inp(yaw_deg=45.0, bearing_deg=90.0))
        self.assertAlmostEqual(out.current_yaw_deg, 45.0, places=4)

    def test_output_echoes_alt_m(self):
        out = _run(_inp(alt_m=250.0))
        self.assertAlmostEqual(out.alt_m, 250.0)

    def test_no_bearing_output_has_nan_bearing_deg(self):
        out = _run(_inp(bearing_nan=True))
        self.assertTrue(math.isnan(out.target_bearing_deg))

    def test_imu_fail_output_preserves_bearing_deg(self):
        out = _run(_inp(imu_health=0, bearing_deg=55.0))
        self.assertAlmostEqual(out.target_bearing_deg, 55.0, places=4)


if __name__ == "__main__":
    unittest.main()
