from __future__ import annotations

import math
import sys
from collections import Counter
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

import sim_closed_loop_yaw_rate_current as sim  # noqa: E402
from Sensor_Motor import control, guidance  # noqa: E402
from lib import config  # noqa: E402


def test_closed_loop_yaw_rate_current_acceptance(tmp_path):
    sim.reload_modules()
    rows = sim.run_all_cases()
    out_path = tmp_path / "closed_loop_yaw_rate_current.csv"
    sim.write_csv(rows, out_path)

    fail_reasons = Counter(
        reason
        for row in rows
        for reason in str(row["fail_reason"]).split("|")
        if reason
    )

    assert out_path.exists()
    assert len(rows) == 3 * 3 * 3 * 4 * 3
    assert all(not row["diverged"] for row in rows)
    assert all(not row["opposite_sign"] for row in rows)
    assert all(row["pass"] for row in rows), dict(fail_reasons)


def test_l1output_limit_is_transferred_to_control_input_and_output():
    sim.reload_modules()
    rows = sim.run_all_cases()

    for row in rows:
        assert row["limit_transferred"] is True
        assert math.isclose(
            float(row["l1_yaw_rate_limit_dps"]),
            float(row["ctrl_in_yaw_rate_limit_dps"]),
            rel_tol=0.0,
            abs_tol=1.0e-9,
        )
        assert math.isclose(
            float(row["l1_yaw_rate_limit_dps"]),
            float(row["ctrl_out_yaw_rate_limit_dps"]),
            rel_tol=0.0,
            abs_tol=1.0e-9,
        )


def test_feedforward_normalization_uses_mode_limit():
    sim.reload_modules()
    rows = sim.run_all_cases()

    assert all(row["ff_uses_mode_limit"] for row in rows)

    dr_pm_at_limit = [
        row for row in rows
        if row["mode"] == guidance.ControlMode.DR_PM_G_CLOSED.value
        and abs(float(row["command_dps"])) == config.DR_PM_G_YAW_RATE_LIMIT_DPS
    ]
    assert dr_pm_at_limit
    for row in dr_pm_at_limit:
        expected = (
            control.angular_velocity_to_delta_ff(
                float(row["command_dps"]),
                config.DR_PM_G_YAW_RATE_LIMIT_DPS,
            )
            * config.DR_PM_FF_SCALE
        )
        gps_fallback = (
            control.angular_velocity_to_delta_ff(
                float(row["command_dps"]),
                config.GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS,
            )
            * config.DR_PM_FF_SCALE
        )
        assert math.isclose(float(row["delta_ff_deg"]), expected, rel_tol=0.0, abs_tol=1.0e-6)
        assert not math.isclose(float(row["delta_ff_deg"]), gps_fallback, rel_tol=0.0, abs_tol=1.0e-6)


def test_final_error_and_saturation_bounds_are_reasonable():
    sim.reload_modules()
    rows = sim.run_all_cases()

    for row in rows:
        assert abs(float(row["final_error_dps"])) <= float(row["final_error_abs_limit_dps"])
        assert float(row["saturation_ratio"]) < 0.50
        assert abs(float(row["delta_ff_deg"])) <= control.DELTA_ARM_MAX_DEG + 1.0e-9
        assert abs(float(row["final_yaw_rate_dps"])) < 1000.0
