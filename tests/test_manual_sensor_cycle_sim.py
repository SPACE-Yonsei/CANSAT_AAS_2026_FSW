"""Tests for tools/manual_sensor_cycle_sim.py.

These exercise the full production guidance/control pipeline through the SIL
harness and assert the behaviours the simulator is meant to demonstrate:
mode auto-selection, GPS dropout → DR transitions, DR accumulation, the
ProduceL1Input read-only contract, command/arm limits, sign consistency, and
the timeout / baro-spike / gyro-spike safety paths.
"""
import math
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

import manual_sensor_cycle_sim as sim  # noqa: E402
from lib import config  # noqa: E402
from Sensor_Motor import control  # noqa: E402


def _sign(x: float, eps: float = 1e-6) -> int:
    if x > eps:
        return 1
    if x < -eps:
        return -1
    return 0


# ── 1. GPS_CLOSED_RIGHT: GPS_TRACKING_CLOSED, all sides RIGHT ──────────────────

def test_gps_closed_right_modes_and_sides():
    rows = sim.run_preset("GPS_CLOSED_RIGHT", cycles=10, sample_cycles=set())
    assert len(rows) == 10
    for r in rows:
        assert r["mode"] == "GPS_TRACKING_CLOSED"
        assert r["target_side"] == r["cmd_side"] == r["arm_side"] == "RIGHT"
        assert r["target_pointing_ok"] is True


# ── 2. GPS_CLOSED_LEFT: all sides LEFT ────────────────────────────────────────

def test_gps_closed_left_modes_and_sides():
    rows = sim.run_preset("GPS_CLOSED_LEFT", cycles=10, sample_cycles=set())
    assert len(rows) == 10
    for r in rows:
        assert r["mode"] == "GPS_TRACKING_CLOSED"
        assert r["target_side"] == r["cmd_side"] == r["arm_side"] == "LEFT"
        assert r["target_pointing_ok"] is True


# ── 3. GPS dropout sequence: CLOSED → DR_M → DR_PM ────────────────────────────

def test_gps_dropout_sequence_mode_progression():
    rows = sim.run_preset("GPS_DROPOUT_SEQUENCE_RIGHT", cycles=10, sample_cycles=set())
    by_cycle = {r["cycle"]: r for r in rows}
    for c in (1, 2):
        assert by_cycle[c]["mode"] == "GPS_TRACKING_CLOSED"
    for c in (3, 4, 5):
        assert by_cycle[c]["mode"] == "DR_M_GBA_CLOSED"
    for c in (6, 7, 8, 9, 10):
        assert by_cycle[c]["mode"] == "DR_PM_GBA_CLOSED"
    # every cycle keeps target pointing consistent
    for r in rows:
        assert r["target_side"] == r["cmd_side"] == r["arm_side"]


# ── 4. DR_PM accumulates current position over time ───────────────────────────

def test_dr_pm_accumulates_current_position():
    rows = sim.run_preset("DR_PM_GBA_RIGHT", cycles=10, sample_cycles=set())
    assert all(r["mode"] == "DR_PM_GBA_CLOSED" for r in rows)
    n_after = [r["dr_current_N_after_l1input"] for r in rows]
    # course is north for this preset, so N must grow monotonically
    assert n_after[-1] > n_after[0]
    assert all(n_after[i] >= n_after[i - 1] - 1e-12 for i in range(1, len(n_after)))
    # the accumulated distance from the start is strictly increasing
    assert any(n_after[i] > n_after[i - 1] for i in range(1, len(n_after)))


# ── 5. ProduceL1Input must not mutate dr.current_E/N ──────────────────────────

def test_produce_l1input_does_not_reintegrate_dr():
    for name in ("DR_PM_GBA_RIGHT", "DR_PM_G_RIGHT", "DR_M_GBA_RIGHT"):
        rows = sim.run_preset(name, cycles=10, sample_cycles=set())
        for r in rows:
            assert r["dr_current_E_before_l1input"] == r["dr_current_E_after_l1input"]
            assert r["dr_current_N_before_l1input"] == r["dr_current_N_after_l1input"]
            assert r["dr_double_integrated_by_l1input"] is False


# ── 6. yaw_rate_cmd within the per-mode limit ─────────────────────────────────

def test_yaw_rate_cmd_within_limit_all_presets():
    for name in sim.PRESETS:
        rows = sim.run_preset(name, sample_cycles=set())
        for r in rows:
            if r["l1out_valid"] and math.isfinite(r["yaw_rate_limit_dps"]):
                assert abs(r["yaw_rate_cmd_dps"]) <= r["yaw_rate_limit_dps"] + 1e-9


# ── 7. delta_arm within DELTA_ARM_MAX_DEG ─────────────────────────────────────

def test_delta_arm_within_max_all_presets():
    dam = control.DELTA_ARM_MAX_DEG
    for name in sim.PRESETS:
        rows = sim.run_preset(name, sample_cycles=set())
        for r in rows:
            assert abs(r["delta_arm_deg"]) <= dam + 1e-9


# ── 8. yaw_rate_cmd sign matches right_minus_left sign on valid cycles ─────────

def test_yaw_rate_and_arm_sign_consistent():
    for name in sim.PRESETS:
        rows = sim.run_preset(name, sample_cycles=set())
        for r in rows:
            if r["l1out_valid"] and r["ctrl_valid"]:
                sa = _sign(r["yaw_rate_cmd_dps"])
                sb = _sign(r["right_minus_left_angle_deg"])
                # never opposite; when the command is acting, the arm follows it
                assert not (sa == 1 and sb == -1)
                assert not (sa == -1 and sb == 1)
                if sa != 0:
                    assert sb == sa


# ── 9. DR_TIMEOUT → FAIL then neutral arms ────────────────────────────────────

def test_dr_timeout_then_neutral_arms():
    rows = sim.run_preset("DR_TIMEOUT_RIGHT", sample_cycles=set())
    fails = [r for r in rows if r["mode"] == "FAIL"]
    assert fails, "expected DR to time out into FAIL"
    assert all(r["fail_reason"] == "DR_TIMEOUT" for r in fails)
    for r in fails:
        assert r["right_minus_left_angle_deg"] == 0.0
        assert r["delta_arm_deg"] == 0.0
        assert r["ctrl_valid"] is False
    # earlier cycles were actually doing DR before the timeout
    assert any(r["mode"] == "DR_PM_GBA_CLOSED" for r in rows)


# ── 10. BARO spike is detected and degrades gracefully ────────────────────────

def test_baro_spike_detected_no_blowup():
    rows = sim.run_preset("BARO_SPIKE_RIGHT", sample_cycles=set())
    spike_rows = [r for r in rows if r["baro_sink_spike"]]
    assert spike_rows, "expected at least one baro_sink_spike cycle"
    for r in spike_rows:
        assert r["baro_sink_fresh"] is False
        # speed falls back to last-known velocity, not the spike
        assert r["dr_speed_source"] == "SPEED_LASTV"
        # no NaN / blow-up in the guidance + control outputs
        assert math.isfinite(r["nu_deg"])
        assert math.isfinite(r["target_distance_m"])
        assert math.isfinite(r["delta_arm_deg"])


# ── 11. GYRO spike is not used as CLOSED-loop PID feedback ─────────────────────

def test_gyro_spike_not_used_in_closed_pid():
    rows = sim.run_preset("GYRO_SPIKE_RIGHT", sample_cycles=set())
    by_cycle = {r["cycle"]: r for r in rows}
    spike = by_cycle[5]
    # the spike cycle must leave the CLOSED PID path (OPEN) so the bad gyro
    # value can never drive closed-loop feedback
    assert spike["mode"].endswith("_OPEN")
    assert spike["pid_enabled"] is False
    assert spike["delta_pid_deg"] == 0.0
    # neighbouring cycles are the normal CLOSED DR mode (shows it is the spike)
    assert by_cycle[4]["mode"].endswith("_CLOSED")
    # 신 계약: DR closed는 기본 FF-only(PID 제거). 롤백(DR_PID_ENABLED=True) 시에만 PID.
    assert by_cycle[4]["pid_enabled"] is bool(sim.config.DR_PID_ENABLED)
    assert by_cycle[6]["mode"].endswith("_CLOSED")


# ── Pipeline contract: exactly one DecideControlMode per cycle ─────────────────

def test_exactly_one_decide_control_mode_call_per_cycle():
    import manual_sensor_cycle_sim as s
    s.reload_modules()
    premise = s.Premise(target_E=80.0, target_N=20.0)
    s.configure_mission(premise)

    calls = {"n": 0}
    real = s.guidance.DecideControlMode

    def counting(*args, **kwargs):
        calls["n"] += 1
        return real(*args, **kwargs)

    s.guidance.DecideControlMode = counting
    try:
        cyc = s.base_cycle_from_premise(premise)
        s.run_cycle(cyc, premise, 1000.0, 0.05, 1, "main", "TEST")
    finally:
        s.guidance.DecideControlMode = real
    assert calls["n"] == 1


# ── Input parser ──────────────────────────────────────────────────────────────

def test_parse_kv_line_and_apply_overrides():
    premise = sim.Premise(target_E=80.0, target_N=20.0)
    cyc = sim.base_cycle_from_premise(premise)
    overrides = sim.parse_kv_line("gps_motion=off speed_mps=6 target_E=50")
    sim.apply_overrides(cyc, premise, overrides)
    assert cyc.gps_motion == "off"
    assert cyc.speed_mps == 6.0
    assert premise.target_E == 50.0


def test_blank_interactive_line_keeps_previous():
    assert sim.parse_kv_line("") == {}
    assert sim.parse_kv_line("   ") == {}


def test_unknown_key_is_rejected():
    premise = sim.Premise()
    cyc = sim.base_cycle_from_premise(premise)
    with pytest.raises(ValueError):
        sim.apply_overrides(cyc, premise, {"banana": "1"})


def test_bad_state_value_is_rejected():
    premise = sim.Premise()
    cyc = sim.base_cycle_from_premise(premise)
    with pytest.raises(ValueError):
        sim.apply_overrides(cyc, premise, {"gps_pos": "sometimes"})


# ── Scripted CSV reproduces the dropout sequence ──────────────────────────────

def test_scripted_csv_matches_dropout_sequence(tmp_path):
    csv_path = tmp_path / "sim_input.csv"
    lines = ["cycle,gps_pos,gps_motion,yaw,gyrz,baro,acc,payload_E,payload_N,course_deg,speed_mps,yaw_deg,gyrz_dps,baro_sink"]
    for c in range(1, 11):
        if c <= 2:
            gp, gm = "on", "on"
        elif c <= 5:
            gp, gm = "on", "off"
        else:
            gp, gm = "off", "off"
        lines.append(f"{c},{gp},{gm},on,on,on,on,0,0,0,4,0,0,2")
    csv_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    premise = sim.Premise(target_E=80.0, target_N=20.0)
    rows = sim.run_script(csv_path, premise, cycles=10, sample_cycles=set())
    by_cycle = {r["cycle"]: r for r in rows}
    assert by_cycle[1]["mode"] == "GPS_TRACKING_CLOSED"
    assert by_cycle[3]["mode"] == "DR_M_GBA_CLOSED"
    assert by_cycle[10]["mode"] == "DR_PM_GBA_CLOSED"


# ── CSV column contract ───────────────────────────────────────────────────────

def test_csv_columns_present(tmp_path):
    rows = sim.run_preset("GPS_DROPOUT_SEQUENCE_RIGHT", cycles=10, sample_cycles=set())
    out = tmp_path / "trace.csv"
    sim.write_csv(rows, out)
    header = out.read_text(encoding="utf-8").splitlines()[0].split(",")
    for required in (
        "cycle", "mode", "fail_reason", "target_distance_m", "nu_deg",
        "yaw_rate_cmd_dps", "right_minus_left_angle_deg", "delta_arm_deg",
        "dr_age_s", "dr_confidence", "dr_speed_source", "pid_enabled",
        "baro_sink_spike", "speed_clamped", "saturated",
        "target_side", "cmd_side", "arm_side", "target_pointing_ok",
        "dr_current_E_before_l1input", "dr_current_E_after_l1input",
    ):
        assert required in header
