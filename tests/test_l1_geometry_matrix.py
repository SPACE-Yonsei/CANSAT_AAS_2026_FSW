import math

import pytest

from lib import config
from Sensor_Motor import guidance


COURSE_DEG_CASES = [-90.0, 0.0, 45.0]
NU_OFFSET_DEG_CASES = [-30.0, -10.0, -2.0, 0.0, 2.0, 10.0, 30.0]
MODE_CASES = [
    guidance.ControlMode.GPS_TRACKING_CLOSED,
    guidance.ControlMode.GPS_TRACKING_OPEN,
    guidance.ControlMode.DR_M_G_CLOSED,
    guidance.ControlMode.DR_PM_G_CLOSED,
]
SPEED_CASES = [config.V_MIN_MPS, 5.0]


def _target_from_bearing(bearing_deg: float, distance_m: float = 100.0):
    bearing = math.radians(bearing_deg)
    return distance_m * math.sin(bearing), distance_m * math.cos(bearing)


def _l1_input(course_deg: float, target_bearing_deg: float, speed_mps: float, mode):
    target_e, target_n = _target_from_bearing(target_bearing_deg)
    return guidance.L1Input(
        valid=True,
        reason="TEST_GEOMETRY",
        control_mode=mode,
        confidence=1.0,
        E=0.0,
        N=0.0,
        V=speed_mps,
        course=math.radians(course_deg),
        vE=speed_mps * math.sin(math.radians(course_deg)),
        vN=speed_mps * math.cos(math.radians(course_deg)),
        target_E=target_e,
        target_N=target_n,
    )


@pytest.mark.parametrize("mode", MODE_CASES)
@pytest.mark.parametrize("course_deg", COURSE_DEG_CASES)
@pytest.mark.parametrize("nu_offset_deg", NU_OFFSET_DEG_CASES)
@pytest.mark.parametrize("speed_mps", SPEED_CASES)
def test_yaw_rate_sign_deadband_and_limit(mode, course_deg, nu_offset_deg, speed_mps):
    l1in = _l1_input(course_deg, course_deg + nu_offset_deg, speed_mps, mode)
    out = guidance.ProduceL1Output(l1in)

    assert out.valid is True
    yaw_rate_cmd_dps = math.degrees(out.yaw_rate_cmd)
    assert abs(yaw_rate_cmd_dps) <= out.yaw_rate_limit_dps + 1.0e-9

    if abs(nu_offset_deg) < config.NU_DEADBAND_DEG:
        assert yaw_rate_cmd_dps == 0.0
    elif nu_offset_deg > 0.0:
        assert yaw_rate_cmd_dps > 0.0
    else:
        assert yaw_rate_cmd_dps < 0.0


@pytest.mark.parametrize("mode", MODE_CASES)
def test_high_speed_is_clamped_before_l1_command(mode):
    max_speed = config.V_MAX_DR_MPS if str(mode.value).startswith("DR_") else config.V_MAX_MPS
    course_deg = 0.0
    target_bearing_deg = 20.0
    out_at_max = guidance.ProduceL1Output(
        _l1_input(course_deg, target_bearing_deg, max_speed, mode)
    )
    out_above_max = guidance.ProduceL1Output(
        _l1_input(course_deg, target_bearing_deg, max_speed + 10.0, mode)
    )

    assert out_at_max.valid is True
    assert out_above_max.valid is True
    assert math.isclose(
        out_above_max.yaw_rate_cmd,
        out_at_max.yaw_rate_cmd,
        rel_tol=0.0,
        abs_tol=1.0e-12,
    )


@pytest.mark.parametrize("mode", MODE_CASES)
def test_speed_below_min_is_rejected_by_production_guard(mode):
    out = guidance.ProduceL1Output(
        _l1_input(0.0, 20.0, config.V_MIN_MPS - 0.01, mode)
    )

    assert out.valid is False
    assert out.reason == "V_TOO_SMALL"
