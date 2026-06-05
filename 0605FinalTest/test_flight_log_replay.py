import math
from collections import Counter

from Sensor_Motor import control
import replay_flight_log


RAW_MOTOR_LOG = replay_flight_log.ROOT / "motorlogs" / "motor_control_20260604_161105.csv"


def _b(value) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def test_replay_actual_motor_log_without_exceptions(tmp_path):
    events = replay_flight_log.load_events(raw_motor=RAW_MOTOR_LOG)
    rows = replay_flight_log.replay_events(events)
    out_path = tmp_path / "replay_control_output.csv"
    replay_flight_log.write_output(rows, out_path)

    assert out_path.exists()
    assert rows

    state4_under50_valid = [
        row
        for row in rows
        if int(row["flight_state"]) == 4
        and math.isfinite(float(row["distance_to_target"]))
        and float(row["distance_to_target"]) <= 50.0
        and _b(row["l1_valid"])
    ]
    assert state4_under50_valid

    for row in rows:
        limit = float(row["yaw_rate_limit_dps"])
        cmd = float(row["yaw_rate_cmd_dps"])
        if limit > 0.0:
            assert abs(cmd) <= limit + 1.0e-9
        assert abs(float(row["delta_arm_deg"])) <= control.DELTA_ARM_MAX_DEG + 1.0e-9
        if row["mode"] == "FAIL":
            assert row["fail_reason"] in replay_flight_log.ALLOWED_FAIL_REASONS

    total = len(rows)
    saturated_count = sum(1 for row in rows if _b(row["saturated"]))
    gyro_rejected_count = sum(1 for row in rows if _b(row["gyro_rejected"]))
    baro_spike_count = sum(1 for row in rows if _b(row["baro_sink_spike"]))
    saturation_ratio = saturated_count / total

    print(f"mode_counts={dict(Counter(row['mode'] for row in rows))}")
    print(f"fail_counts={dict(Counter(row['fail_reason'] for row in rows if row['mode'] == 'FAIL'))}")
    print(f"baro_sink_spike_count={baro_spike_count}")
    print(f"gyro_rejected_count={gyro_rejected_count}")
    print(f"saturation_ratio={saturation_ratio:.3f}")

    assert saturation_ratio < 0.30
