import math

import pytest

import sim_control_modes as sim


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
    allowed_missing = set()
    assert missing == allowed_missing


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


def test_full_gps_but_gyro_spike_goes_gps_open(matrix_rows):
    modes = {
        row["mode"] for row in matrix_rows
        if row["origin_ready"]
        and row["target_ready"]
        and row["gps_pos"]
        and row["gps_motion"]
        and row["imu_gyrz"]
        and row["gyro_spike"]
    }
    assert modes == {sim.guidance.ControlMode.GPS_TRACKING_OPEN.value}


def test_gps_pos_only_with_dr_and_gba_goes_dr_m_gba(matrix_rows):
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
        baro_spike=False,
    )
    assert row["mode"] == sim.guidance.ControlMode.DR_M_GBA_CLOSED.value


def test_no_gps_with_dr_and_gba_goes_dr_pm_gba(matrix_rows):
    row = _row(
        matrix_rows,
        origin_ready=True,
        target_ready=True,
        gps_pos=False,
        gps_motion=False,
        imu_gyrz=True,
        imu_yaw=False,
        baro_sink=True,
        acc=True,
        dr_ready=True,
        dr_old=False,
        gyro_spike=False,
        baro_spike=False,
    )
    assert row["mode"] == sim.guidance.ControlMode.DR_PM_GBA_CLOSED.value


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
