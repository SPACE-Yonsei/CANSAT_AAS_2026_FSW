import math

import pytest

from lib import config
from Sensor_Motor import control
from Sensor_Motor.guidance import ControlMode


CMD_DPS_CASES = [-120.0, -60.0, -30.0, -10.0, 0.0, 10.0, 30.0, 60.0, 120.0]
GYRZ_MEAS_DPS_CASES = [-120.0, -60.0, -10.0, 0.0, 10.0, 60.0, 120.0]
MODE_CASES = [
    ControlMode.GPS_TRACKING_CLOSED,
    ControlMode.GPS_TRACKING_OPEN,
    ControlMode.DR_M_G_CLOSED,
    ControlMode.DR_PM_G_CLOSED,
    ControlMode.FAIL,
]


def _limit_for_mode(mode):
    if mode == ControlMode.GPS_TRACKING_OPEN:
        return config.GPS_TRACKING_OPEN_YAW_RATE_LIMIT_DPS
    if mode == ControlMode.DR_M_G_CLOSED:
        return config.DR_M_G_YAW_RATE_LIMIT_DPS
    if mode == ControlMode.DR_PM_G_CLOSED:
        return config.DR_PM_G_YAW_RATE_LIMIT_DPS
    return config.GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS


def _kp_for_mode(mode):
    if mode == ControlMode.GPS_TRACKING_CLOSED:
        return config.KP_GPS_CLOSED
    if mode == ControlMode.DR_M_G_CLOSED:
        return config.KP_DR_M_CLOSED
    if mode == ControlMode.DR_PM_G_CLOSED:
        return config.KP_DR_PM_CLOSED
    return 0.0


def _ctrl_input(mode, cmd_dps):
    return control.CtrlInput(
        angular_velocity_cmd_deg_s=cmd_dps,
        ground_speed_mps=5.0,
        valid=True,
        timestamp=1.0,
        pid_enabled=(
            mode
            in {
                ControlMode.GPS_TRACKING_CLOSED,
                ControlMode.DR_M_G_CLOSED,
                ControlMode.DR_PM_G_CLOSED,
            }
        ),
        control_mode=mode,
        yaw_rate_limit_dps=_limit_for_mode(mode),
    )


@pytest.mark.parametrize("mode", MODE_CASES)
@pytest.mark.parametrize("cmd_dps", CMD_DPS_CASES)
@pytest.mark.parametrize("gyrz_meas_dps", GYRZ_MEAS_DPS_CASES)
def test_control_command_grid_signs_gains_and_limits(mode, cmd_dps, gyrz_meas_dps):
    control.reset()
    out = control.ProduceCtrlOutput(_ctrl_input(mode, cmd_dps), gyrz_meas_dps, 1.0)

    if mode == ControlMode.FAIL:
        assert out.valid is False
        assert out.left_pw == control.LEFT_NEUTRAL
        assert out.right_pw == control.RIGHT_NEUTRAL
        assert out.delta_arm_deg == 0.0
        return

    assert out.valid is True
    assert abs(out.delta_arm_deg) <= control.DELTA_ARM_MAX_DEG + 1.0e-9

    if cmd_dps > 0.0:
        assert out.delta_ff_deg > 0.0
    elif cmd_dps < 0.0:
        assert out.delta_ff_deg < 0.0
    else:
        assert out.delta_ff_deg == 0.0

    if mode == ControlMode.GPS_TRACKING_OPEN:
        assert out.delta_pid_deg == 0.0
        assert out.kp_used == 0.0
    else:
        assert math.isclose(out.kp_used, _kp_for_mode(mode), rel_tol=0.0, abs_tol=1.0e-12)

    if out.delta_arm_deg > 0.0:
        assert out.right_angle_deg > control.NEUTRAL_ARM_DEG
        assert out.left_angle_deg < control.NEUTRAL_ARM_DEG
    elif out.delta_arm_deg < 0.0:
        assert out.right_angle_deg < control.NEUTRAL_ARM_DEG
        assert out.left_angle_deg > control.NEUTRAL_ARM_DEG
