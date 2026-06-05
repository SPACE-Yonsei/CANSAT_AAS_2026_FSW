from __future__ import annotations

import math
import sys
from collections import Counter
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

import replay_flight_log_sil_current as replay_sil  # noqa: E402
from Sensor_Motor import control  # noqa: E402


RAW_MOTOR_LOG = ROOT / "motorlogs" / "motor_control_20260604_161105.csv"


def _b(value) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _f(value) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return math.nan
    return out if math.isfinite(out) else math.nan


def test_actual_flight_log_replay_sil_contract(tmp_path):
    events = replay_sil.load_events(raw_motor=RAW_MOTOR_LOG)
    rows = replay_sil.replay_events(events)
    out_path = tmp_path / "flight_log_replay_sil_current.csv"
    replay_sil.write_output(rows, out_path)

    assert out_path.exists()
    replay_sil.assert_replay_contract(rows)

    print(f"mode_counts={dict(Counter(row['mode'] for row in rows))}")
    print(f"fail_counts={dict(Counter(row['fail_reason'] for row in rows if row['mode'] == 'FAIL'))}")


def test_replay_records_reason_chain_and_limits():
    events = replay_sil.load_events(raw_motor=RAW_MOTOR_LOG)
    rows = replay_sil.replay_events(events)

    assert rows
    assert all("fail_reason" in row for row in rows)
    assert all("l1_reason" in row for row in rows)
    assert all("l1out_reason" in row for row in rows)

    valid_rows = [row for row in rows if _b(row["l1_valid"]) and _b(row["l1out_valid"])]
    assert valid_rows
    assert any(row["l1_reason"] == "GPS_NAV" for row in valid_rows)

    for row in rows:
        if _f(row["yaw_rate_limit_dps"]) > 0.0:
            assert abs(_f(row["yaw_rate_cmd_dps"])) <= _f(row["yaw_rate_limit_dps"]) + 1.0e-9
        assert abs(_f(row["delta_arm_deg"])) <= control.DELTA_ARM_MAX_DEG + 1.0e-9


def test_dr_pm_not_double_integrated_by_l1input():
    events = replay_sil.load_events(raw_motor=RAW_MOTOR_LOG)
    rows = replay_sil.replay_events(events)
    dr_pm_rows = [row for row in rows if str(row["mode"]).startswith("DR_PM")]

    for row in dr_pm_rows:
        assert not _b(row["dr_pm_double_update"])

    assert all(int(row["decide_call_count"]) == 1 for row in rows if int(row["flight_state"]) >= 4)
    assert all(int(row["produce_l1input_call_count"]) == 1 for row in rows if int(row["flight_state"]) >= 4)


def test_fail_reason_allowlist_and_saturation_ratio():
    events = replay_sil.load_events(raw_motor=RAW_MOTOR_LOG)
    rows = replay_sil.replay_events(events)

    fail_rows = [row for row in rows if row["mode"] == "FAIL"]
    for row in fail_rows:
        assert row["fail_reason"] in replay_sil.ALLOWED_FAIL_REASONS

    saturation_ratio = sum(1 for row in rows if _b(row["saturated"])) / len(rows)
    assert saturation_ratio < 0.30
