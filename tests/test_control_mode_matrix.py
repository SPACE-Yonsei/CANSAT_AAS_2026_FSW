import math
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

import sim_control_modes as sim  # noqa: E402


@pytest.fixture(scope="module")
def matrix_rows():
    sim.apply_tuned_config()
    return sim.run_all_cases(sim.NOW)


def _row(rows, **criteria):
    matches = [
        row for row in rows
        if all(row.get(key) == value for key, value in criteria.items())
    ]
    assert matches, f"no row for {criteria}"
    return matches[0]


def test_every_control_mode_is_reachable(matrix_rows):
    reached = {row["mode"] for row in matrix_rows}
    missing = {mode.value for mode in sim.guidance.ControlMode} - reached
    assert missing == set()


def test_decide_control_mode_called_once_per_case(matrix_rows):
    assert {row["decide_calls"] for row in matrix_rows} == {1}


def test_no_origin_or_no_target_always_fails(matrix_rows):
    for row in matrix_rows:
        if not row["origin_ready"] or not row["target_ready"]:
            assert row["mode"] == sim.guidance.ControlMode.FAIL.value
            if not row["origin_ready"]:
                assert row["fail_reason"] == "NO_ORIGIN"
            else:
                assert row["fail_reason"] == "NO_TARGET"


def test_full_gps_and_good_gyro_goes_gps_closed(matrix_rows):
    modes = {
        row["mode"] for row in matrix_rows
        if row["origin_ready"]
        and row["target_ready"]
        and row["gps_pos"]
        and row["gps_motion"]
        and row["imu_gyrz"]
        and not row["gyro_spike"]
    }
    assert modes == {sim.guidance.ControlMode.GPS_TRACKING_CLOSED.value}


def test_full_gps_but_gyro_missing_or_spike_goes_gps_open(matrix_rows):
    modes = {
        row["mode"] for row in matrix_rows
        if row["origin_ready"]
        and row["target_ready"]
        and row["gps_pos"]
        and row["gps_motion"]
        and (not row["imu_gyrz"] or row["gyro_spike"])
    }
    assert modes == {sim.guidance.ControlMode.GPS_TRACKING_OPEN.value}


@pytest.mark.parametrize(
    ("expected_mode", "criteria"),
    [
        (
            sim.guidance.ControlMode.DR_M_GBA_CLOSED.value,
            dict(gps_pos=True, gps_motion=False, imu_gyrz=True, imu_yaw=False,
                 baro_sink=True, acc=True, dr_ready=True, dr_old=False,
                 gyro_spike=False, baro_spike=False),
        ),
        (
            sim.guidance.ControlMode.DR_M_GB_CLOSED.value,
            dict(gps_pos=True, gps_motion=False, imu_gyrz=True, imu_yaw=False,
                 baro_sink=True, acc=False, dr_ready=True, dr_old=False,
                 gyro_spike=False, baro_spike=False),
        ),
        (
            sim.guidance.ControlMode.DR_M_G_CLOSED.value,
            dict(gps_pos=True, gps_motion=False, imu_gyrz=True, imu_yaw=False,
                 baro_sink=False, acc=False, dr_ready=True, dr_old=False,
                 gyro_spike=False, baro_spike=False),
        ),
        (
            sim.guidance.ControlMode.DR_M_YBA_OPEN.value,
            dict(gps_pos=True, gps_motion=False, imu_gyrz=False, imu_yaw=True,
                 baro_sink=True, acc=True, dr_ready=True, dr_old=False,
                 gyro_spike=False, baro_spike=False),
        ),
        (
            sim.guidance.ControlMode.DR_M_YB_OPEN.value,
            dict(gps_pos=True, gps_motion=False, imu_gyrz=False, imu_yaw=True,
                 baro_sink=True, acc=False, dr_ready=True, dr_old=False,
                 gyro_spike=False, baro_spike=False),
        ),
        (
            sim.guidance.ControlMode.DR_M_Y_OPEN.value,
            dict(gps_pos=True, gps_motion=False, imu_gyrz=False, imu_yaw=True,
                 baro_sink=False, acc=False, dr_ready=True, dr_old=False,
                 gyro_spike=False, baro_spike=False),
        ),
        (
            sim.guidance.ControlMode.DR_PM_GBA_CLOSED.value,
            dict(gps_pos=False, gps_motion=False, imu_gyrz=True, imu_yaw=False,
                 baro_sink=True, acc=True, dr_ready=True, dr_old=False,
                 gyro_spike=False, baro_spike=False),
        ),
        (
            sim.guidance.ControlMode.DR_PM_GB_CLOSED.value,
            dict(gps_pos=False, gps_motion=False, imu_gyrz=True, imu_yaw=False,
                 baro_sink=True, acc=False, dr_ready=True, dr_old=False,
                 gyro_spike=False, baro_spike=False),
        ),
        (
            sim.guidance.ControlMode.DR_PM_G_CLOSED.value,
            dict(gps_pos=False, gps_motion=False, imu_gyrz=True, imu_yaw=False,
                 baro_sink=False, acc=False, dr_ready=True, dr_old=False,
                 gyro_spike=False, baro_spike=False),
        ),
        (
            sim.guidance.ControlMode.DR_PM_YBA_OPEN.value,
            dict(gps_pos=False, gps_motion=False, imu_gyrz=False, imu_yaw=True,
                 baro_sink=True, acc=True, dr_ready=True, dr_old=False,
                 gyro_spike=False, baro_spike=False),
        ),
        (
            sim.guidance.ControlMode.DR_PM_YB_OPEN.value,
            dict(gps_pos=False, gps_motion=False, imu_gyrz=False, imu_yaw=True,
                 baro_sink=True, acc=False, dr_ready=True, dr_old=False,
                 gyro_spike=False, baro_spike=False),
        ),
        (
            sim.guidance.ControlMode.DR_PM_Y_OPEN.value,
            dict(gps_pos=False, gps_motion=False, imu_gyrz=False, imu_yaw=True,
                 baro_sink=False, acc=False, dr_ready=True, dr_old=False,
                 gyro_spike=False, baro_spike=False),
        ),
    ],
)
def test_dr_modes_are_reachable_by_sensor_schedule(matrix_rows, expected_mode, criteria):
    row = _row(
        matrix_rows,
        origin_ready=True,
        target_ready=True,
        **criteria,
    )
    assert row["mode"] == expected_mode


def test_old_dr_goes_fail_dr_timeout(matrix_rows):
    row = _row(
        matrix_rows,
        origin_ready=True,
        target_ready=True,
        gps_pos=False,
        gps_motion=False,
        imu_gyrz=True,
        imu_yaw=False,
        baro_sink=False,
        acc=False,
        dr_ready=True,
        dr_old=True,
        gyro_spike=False,
        baro_spike=False,
    )
    assert row["mode"] == sim.guidance.ControlMode.FAIL.value
    assert row["fail_reason"] == "DR_TIMEOUT"


def test_baro_spike_removes_baro_source(matrix_rows):
    row = _row(
        matrix_rows,
        origin_ready=True,
        target_ready=True,
        gps_pos=True,
        gps_motion=False,
        imu_gyrz=True,
        imu_yaw=False,
        baro_sink=True,
        acc=True,
        dr_ready=True,
        dr_old=False,
        gyro_spike=False,
        baro_spike=True,
    )
    assert row["baro_sink_fresh"] is False
    assert row["baro_sink_spike"] is True
    assert row["mode"] == sim.guidance.ControlMode.DR_M_G_CLOSED.value


def test_l1input_reason_matches_current_contract(matrix_rows):
    for row in matrix_rows:
        assert row["l1_in_reason"] == row["l1_in_reason_expected"]
        if row["mode"].startswith("GPS_TRACKING_"):
            assert row["l1_in_reason"] == "GPS_NAV"
        elif row["mode"].startswith("DR_"):
            assert row["l1_in_reason"] == row["mode"]
        elif row["mode"] == sim.guidance.ControlMode.FAIL.value:
            assert row["l1_in_reason"] == row["fail_reason"]


def test_invalid_l1output_yaw_rate_limit_is_finite(matrix_rows):
    for row in matrix_rows:
        if not row["l1_out_valid"]:
            assert math.isfinite(float(row["yaw_rate_limit_dps"]))


def test_closed_modes_have_pid_gain(matrix_rows):
    for row in matrix_rows:
        mode = row["mode"]
        if mode == sim.guidance.ControlMode.GPS_TRACKING_CLOSED.value:
            assert math.isclose(row["kp_used"], sim.config.KP_GPS_CLOSED)
        elif mode.startswith("DR_M_") and mode.endswith("_CLOSED"):
            assert math.isclose(row["kp_used"], sim.config.KP_DR_M_CLOSED)
        elif mode.startswith("DR_PM_") and mode.endswith("_CLOSED"):
            assert math.isclose(row["kp_used"], sim.config.KP_DR_PM_CLOSED)
        elif mode.endswith("_OPEN") or mode == sim.guidance.ControlMode.FAIL.value:
            assert row["delta_pid_deg"] == 0.0
            assert row["kp_used"] == 0.0


def test_fail_outputs_neutral(matrix_rows):
    for row in matrix_rows:
        if row["mode"] == sim.guidance.ControlMode.FAIL.value:
            assert row["left_pw"] == sim.control.LEFT_NEUTRAL
            assert row["right_pw"] == sim.control.RIGHT_NEUTRAL
            assert row["delta_arm_deg"] == 0.0
