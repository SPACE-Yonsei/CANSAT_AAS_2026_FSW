import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools import inspect_l1input_contract_current as inspect_contract


def assert_finite(*values):
    for value in values:
        assert math.isfinite(float(value))


def assert_l1_matches_nav(result):
    nav = result["nav"]
    l1_in = result["l1_in"]
    assert l1_in.E == nav.E
    assert l1_in.N == nav.N
    assert l1_in.V == nav.V
    assert l1_in.course == nav.course


def test_a_no_origin():
    result = inspect_contract.run_contract_case("A_NO_ORIGIN")
    guidance = result["guidance"]
    assert result["decide_calls"] == 1
    assert result["mode"] == guidance.ControlMode.FAIL
    assert result["nav"].valid is False
    assert result["l1_in"].valid is False
    assert result["l1_in"].reason.startswith("NO_ORIGIN")


def test_b_no_target():
    result = inspect_contract.run_contract_case("B_NO_TARGET")
    guidance = result["guidance"]
    assert result["decide_calls"] == 1
    assert result["mode"] == guidance.ControlMode.FAIL
    assert result["l1_in"].valid is False
    assert result["l1_in"].reason.startswith("NO_TARGET")


def test_c_gps_tracking_closed_nav_to_l1input():
    result = inspect_contract.run_contract_case("C_GPS_TRACKING_CLOSED")
    guidance = result["guidance"]
    nav = result["nav"]
    l1_in = result["l1_in"]

    assert result["decide_calls"] == 1
    assert result["mode"] == guidance.ControlMode.GPS_TRACKING_CLOSED
    assert nav.valid is True
    assert l1_in.valid is True
    assert l1_in.reason == "GPS_NAV"
    assert_finite(
        l1_in.E,
        l1_in.N,
        l1_in.V,
        l1_in.course,
        l1_in.vE,
        l1_in.vN,
        l1_in.target_E,
        l1_in.target_N,
    )
    assert l1_in.confidence == 1.0
    assert l1_in.dr_method == guidance.DRMethod.NONE
    assert_l1_matches_nav(result)


def test_d_gps_tracking_open_uses_gps_nav():
    result = inspect_contract.run_contract_case("D_GPS_TRACKING_OPEN")
    guidance = result["guidance"]
    l1_in = result["l1_in"]

    assert result["decide_calls"] == 1
    assert result["mode"] == guidance.ControlMode.GPS_TRACKING_OPEN
    assert l1_in.valid is True
    assert l1_in.confidence == 1.0
    assert l1_in.reason == "GPS_NAV"
    assert_l1_matches_nav(result)


def test_e_dr_m_gba_position_from_gps_motion_from_dr():
    result = inspect_contract.run_contract_case("E_DR_M_GBA")
    guidance = result["guidance"]
    nav = result["nav"]
    l1_in = result["l1_in"]
    gps = guidance._STATE_t.gps

    assert result["decide_calls"] == 1
    assert result["mode"] == guidance.ControlMode.DR_M_GBA_CLOSED
    assert nav.valid is True
    assert l1_in.valid is True
    assert l1_in.E == gps.E
    assert l1_in.N == gps.N
    assert l1_in.V == nav.V
    assert l1_in.course == nav.course
    assert l1_in.confidence == guidance._STATE_t.dr.confidence
    assert l1_in.dr_method == guidance._STATE_t.dr.method
    assert l1_in.dr_method in (
        guidance.DRMethod.GYRO_ACC_BLEND,
        guidance.DRMethod.GYRO_INTEGRATION,
    )
    assert_l1_matches_nav(result)


def test_f_dr_pm_gba_position_and_motion_from_dr_current():
    result = inspect_contract.run_contract_case("F_DR_PM_GBA")
    guidance = result["guidance"]
    nav = result["nav"]
    dr = result["dr"]
    l1_in = result["l1_in"]

    assert result["decide_calls"] == 1
    assert result["mode"] == guidance.ControlMode.DR_PM_GBA_CLOSED
    assert nav.E == dr.current_E
    assert nav.N == dr.current_N
    assert l1_in.E == nav.E
    assert l1_in.N == nav.N
    assert l1_in.V == nav.V
    assert l1_in.course == nav.course
    assert l1_in.confidence == dr.confidence


def test_g_nav_invalid_when_selected_nav_speed_is_too_small():
    result = inspect_contract.run_contract_case("G_NAV_INVALID")
    guidance = result["guidance"]
    nav = result["nav"]
    l1_in = result["l1_in"]

    assert result["decide_calls"] == 1
    assert result["mode"] in (
        guidance.ControlMode.GPS_TRACKING_CLOSED,
        guidance.ControlMode.GPS_TRACKING_OPEN,
    )
    assert nav.V < inspect_contract.config.V_MIN_MPS
    assert l1_in.valid is False
    assert l1_in.reason in ("NAV_INVALID", "V_TOO_SMALL")


def test_h_valid_l1input_to_l1output():
    result = inspect_contract.run_contract_case("H_L1INPUT_TO_L1OUTPUT")
    l1_in = result["l1_in"]
    l1_out = result["l1_out"]

    assert result["decide_calls"] == 1
    assert l1_in.valid is True
    assert l1_out.valid is True
    assert_finite(l1_out.distance_to_target, l1_out.target_bearing, l1_out.nu)
    yaw_rate_cmd_dps = math.degrees(l1_out.yaw_rate_cmd)
    assert abs(yaw_rate_cmd_dps) <= l1_out.yaw_rate_limit_dps + 1e-9
    assert l1_out.nav_E == l1_in.E
    assert l1_out.nav_N == l1_in.N
    assert l1_out.V == l1_in.V
    assert l1_out.course == l1_in.course


def test_i_valid_l1output_to_control():
    result = inspect_contract.run_contract_case("I_L1OUTPUT_TO_CONTROL")
    control = result["control"]
    ctrl_in = result["ctrl_in"]
    ctrl_out = result["ctrl_out"]
    l1_out = result["l1_out"]

    assert result["decide_calls"] == 1
    assert l1_out.valid is True
    assert ctrl_in is not None
    assert ctrl_in.yaw_rate_limit_dps == l1_out.yaw_rate_limit_dps
    assert abs(ctrl_out.delta_arm_deg) <= control.DELTA_ARM_MAX_DEG + 1e-9
