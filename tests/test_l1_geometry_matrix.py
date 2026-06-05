import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

import inspect_l1_geometry_matrix as geom  # noqa: E402


def assert_output_mirrors_input(result):
    l1_in = result["l1_in"]
    l1_out = result["l1_out"]
    assert l1_out.valid == l1_out.control_valid
    assert l1_out.nav_E == l1_in.E
    assert l1_out.nav_N == l1_in.N
    assert l1_out.V == l1_in.V
    assert l1_out.course == l1_in.course


def assert_sign_deadband_and_limit(result, expected_sign):
    l1_out = result["l1_out"]
    yaw_rate_cmd_dps = math.degrees(l1_out.yaw_rate_cmd)
    assert abs(yaw_rate_cmd_dps) <= l1_out.yaw_rate_limit_dps + 1e-9
    assert l1_out.valid is True
    assert l1_out.control_valid is True
    if expected_sign > 0:
        assert math.degrees(l1_out.nu) > 0.0
        assert yaw_rate_cmd_dps > 0.0
    elif expected_sign < 0:
        assert math.degrees(l1_out.nu) < 0.0
        assert yaw_rate_cmd_dps < 0.0
    else:
        assert abs(math.degrees(l1_out.nu)) < geom.config.NU_DEADBAND_DEG
        assert yaw_rate_cmd_dps == 0.0


def test_direct_positive_nu_yields_positive_yaw_rate():
    result = geom.run_direct_case("direct_positive_nu", course_deg=0.0, nu_deg=30.0)
    assert_sign_deadband_and_limit(result, expected_sign=1)
    assert_output_mirrors_input(result)


def test_direct_negative_nu_yields_negative_yaw_rate():
    result = geom.run_direct_case("direct_negative_nu", course_deg=0.0, nu_deg=-30.0)
    assert_sign_deadband_and_limit(result, expected_sign=-1)
    assert_output_mirrors_input(result)


def test_direct_deadband_zeroes_yaw_rate():
    result = geom.run_direct_case(
        "direct_deadband_nu",
        course_deg=0.0,
        nu_deg=geom.config.NU_DEADBAND_DEG * 0.5,
    )
    assert_sign_deadband_and_limit(result, expected_sign=0)
    assert_output_mirrors_input(result)


def test_direct_dr_confidence_scales_yaw_rate():
    full = geom.run_direct_case(
        "direct_dr_conf_full",
        mode=geom.guidance.ControlMode.DR_M_G_CLOSED,
        course_deg=0.0,
        nu_deg=20.0,
        confidence=1.0,
    )
    half = geom.run_direct_case(
        "direct_dr_conf_half",
        mode=geom.guidance.ControlMode.DR_M_G_CLOSED,
        course_deg=0.0,
        nu_deg=20.0,
        confidence=0.5,
    )
    assert full["l1_out"].valid is True
    assert half["l1_out"].valid is True
    assert math.degrees(full["l1_out"].yaw_rate_cmd) > 0.0
    assert math.isclose(
        half["l1_out"].yaw_rate_cmd,
        0.5 * full["l1_out"].yaw_rate_cmd,
        rel_tol=0.0,
        abs_tol=1e-12,
    )


def test_direct_invalid_input_still_has_finite_yaw_rate_limit():
    result = geom.run_direct_case("direct_invalid_input", course_deg=0.0, nu_deg=30.0, valid=False)
    l1_out = result["l1_out"]
    assert l1_out.valid is False
    assert l1_out.control_valid is False
    assert math.isfinite(l1_out.yaw_rate_limit_dps)


def test_pipeline_generated_l1input_positive_nu():
    result = geom.run_pipeline_case("pipeline_positive_nu", course_deg=0.0, nu_deg=30.0)
    assert result["mode"] == geom.guidance.ControlMode.GPS_TRACKING_CLOSED
    assert result["l1_in"].valid is True
    assert_sign_deadband_and_limit(result, expected_sign=1)
    assert_output_mirrors_input(result)


def test_pipeline_generated_l1input_negative_nu():
    result = geom.run_pipeline_case("pipeline_negative_nu", course_deg=0.0, nu_deg=-30.0)
    assert result["mode"] == geom.guidance.ControlMode.GPS_TRACKING_CLOSED
    assert result["l1_in"].valid is True
    assert_sign_deadband_and_limit(result, expected_sign=-1)
    assert_output_mirrors_input(result)


def test_pipeline_generated_l1input_deadband_zeroes_yaw_rate():
    result = geom.run_pipeline_case(
        "pipeline_deadband_nu",
        course_deg=0.0,
        nu_deg=geom.config.NU_DEADBAND_DEG * 0.5,
    )
    assert result["mode"] == geom.guidance.ControlMode.GPS_TRACKING_CLOSED
    assert result["l1_in"].valid is True
    assert_sign_deadband_and_limit(result, expected_sign=0)
    assert_output_mirrors_input(result)
