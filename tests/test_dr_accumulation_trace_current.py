import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

import trace_dr_accumulation_current as trace  # noqa: E402


def rows_for(rows, scenario):
    return [row for row in rows if row["scenario"] == scenario]


def ratio(rows, key):
    return sum(1 for row in rows if row[key]) / len(rows)


def test_trace_scenarios_and_single_decide_call():
    rows = trace.run_all_scenarios()
    scenarios = {row["scenario"] for row in rows}
    assert {
        "A_GPS_TO_DR_M_GBA",
        "B_GPS_TO_DR_PM_GBA",
        "C_DR_PM_G_FALLBACK",
        "D_DR_PM_YBA_OPEN",
        "E_DR_TIMEOUT",
        "F_DR_POSITION_JUMP",
        "G_BARO_SPIKE_DEGRADATION",
        "H_GYRO_SPIKE_DEGRADATION",
        "I_TARGET_POINTING_TRUTH_NAV_SPLIT",
    } <= scenarios
    assert {row["decide_calls"] for row in rows} == {1}
    assert not any(row["produce_l1input_mutated_dr"] for row in rows)


def test_dr_m_uses_gps_position_without_dr_pm_style_position_integration():
    row = rows_for(trace.run_all_scenarios(), "A_GPS_TO_DR_M_GBA")[0]
    assert row["mode"] == trace.guidance.ControlMode.DR_M_GBA_CLOSED.value
    assert row["gps_pos_fresh"] is True
    assert row["gps_motion_fresh"] is False
    assert math.isclose(row["nav_E"], row["truth_E"], abs_tol=1e-6)
    assert math.isclose(row["nav_N"], row["truth_N"], abs_tol=1e-6)


def test_dr_pm_accumulates_current_position_over_time():
    rows = rows_for(trace.run_all_scenarios(), "B_GPS_TO_DR_PM_GBA")
    assert rows[0]["mode"] == trace.guidance.ControlMode.DR_PM_GBA_CLOSED.value
    dr_e = [row["dr_current_E"] for row in rows]
    assert dr_e[-1] > dr_e[0]
    assert any(rows[i]["dr_current_E"] > rows[i - 1]["dr_current_E"] for i in range(1, len(rows)))


def test_dr_confidence_non_increasing_with_age():
    rows = rows_for(trace.run_all_scenarios(), "B_GPS_TO_DR_PM_GBA")
    conf = [row["dr_confidence"] for row in rows]
    assert all(conf[i] <= conf[i - 1] + 1e-12 for i in range(1, len(conf)))


def test_dr_mode_yaw_rate_uses_confidence_scaling():
    rows = rows_for(trace.run_all_scenarios(), "B_GPS_TO_DR_PM_GBA")
    checked = False
    for row in rows:
        if row["l1out_valid"] and abs(row["nu_est_deg"]) > trace.config.NU_DEADBAND_DEG:
            _v_floor = max(trace.config.V_MIN_MPS,
                           getattr(trace.config, "L1_STEER_V_FLOOR_MPS", trace.config.V_MIN_MPS))
            v_eff = min(max(row["nav_V"], _v_floor), trace.config.V_MAX_DR_MPS)
            nu_eff = max(-math.pi / 2.0, min(math.pi / 2.0, math.radians(row["nu_est_deg"])))
            unscaled = 2.0 * v_eff / trace.config.L_GAIN_M * math.sin(nu_eff)
            scaled = unscaled * row["dr_confidence"]
            limit = math.radians(row["yaw_rate_limit_dps"])
            expected = max(-limit, min(limit, scaled))
            assert math.isclose(math.radians(row["yaw_rate_cmd_dps"]), expected, abs_tol=1e-9)
            checked = True
            break
    assert checked


def test_dr_pm_g_fallback_and_yba_open_modes():
    rows = trace.run_all_scenarios()
    c = rows_for(rows, "C_DR_PM_G_FALLBACK")[0]
    d = rows_for(rows, "D_DR_PM_YBA_OPEN")[0]
    assert c["mode"] == trace.guidance.ControlMode.DR_PM_G_CLOSED.value
    assert c["dr_speed_source"] == "SPEED_LASTV"
    assert d["mode"] == trace.guidance.ControlMode.DR_PM_YBA_OPEN.value


def test_timeout_and_position_jump_guards():
    rows = trace.run_all_scenarios()
    timeout = rows_for(rows, "E_DR_TIMEOUT")[0]
    jump = rows_for(rows, "F_DR_POSITION_JUMP")[0]
    assert timeout["mode"] == trace.guidance.ControlMode.FAIL.value
    assert timeout["fail_reason"] == "DR_TIMEOUT"
    assert jump["mode"] == trace.guidance.ControlMode.FAIL.value
    assert jump["fail_reason"] == "DR_POSITION_JUMP"
    assert jump["jump_current_unchanged"] is True
    assert jump["ctrl_valid"] is False


def test_baro_and_gyro_spike_degradation():
    rows = trace.run_all_scenarios()
    baro = rows_for(rows, "G_BARO_SPIKE_DEGRADATION")[0]
    gyro = rows_for(rows, "H_GYRO_SPIKE_DEGRADATION")[0]
    assert baro["baro_sink_spike"] is True
    assert baro["baro_sink_fresh"] is False
    assert baro["mode"] == trace.guidance.ControlMode.DR_M_G_CLOSED.value
    assert baro["dr_speed_source"] == "SPEED_LASTV"
    assert gyro["imu_gyrz_fresh"] is True
    assert gyro["mode"] == trace.guidance.ControlMode.DR_PM_YBA_OPEN.value
    assert not gyro["mode"].endswith("_CLOSED")


def test_target_pointing_truth_nav_split_metrics():
    rows = rows_for(trace.run_all_scenarios(), "I_TARGET_POINTING_TRUTH_NAV_SPLIT")
    early_5s = [row for row in rows if row["t"] - trace.DR_START_T <= 5.0 and row["cycle"] > 0]
    early_10s = [row for row in rows if row["t"] - trace.DR_START_T <= 10.0]
    assert ratio(early_5s, "distance_decreasing_truth") >= 0.70
    assert ratio(early_10s, "yaw_cmd_sign_correct_truth") >= 0.80
    assert any(abs(row["distance_error_m"]) > 0.1 for row in rows)

    sustained = 0.0
    max_sustained = 0.0
    for row in rows:
        if row["abs_nu_truth_deg"] > 120.0:
            sustained += trace.DT
            max_sustained = max(max_sustained, sustained)
        else:
            sustained = 0.0
    assert max_sustained < 1.0
    assert all(row["target_pointing_ok"] for row in rows)
