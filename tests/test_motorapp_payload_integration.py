import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

import inspect_motorapp_payload_integration as inspect_motorapp  # noqa: E402


def test_payload_handlers_parse_gps_imu_baro():
    result = inspect_motorapp.run_valid_case()
    gps = result["cached_gps"]
    imu = result["cached_imu"]
    baro = result["cached_baro"]

    assert gps.pos_health == 1
    assert gps.motion_health == 1
    assert math.isclose(gps.lat, inspect_motorapp.ORIGIN_LAT)
    assert math.isclose(gps.lon, inspect_motorapp.ORIGIN_LON)
    assert math.isclose(gps.course_rad, 0.0)
    assert math.isclose(gps.speed_mps, 5.0)

    assert math.isclose(imu.gyrz_rad_s, math.radians(-12.0))
    assert imu.lin_acc_valid is True
    assert imu.lin_acc_reject_reason == "OK"

    assert math.isclose(baro.sink_rate, 3.0)
    assert baro.health == 1


def test_ctrl_cycle_valid_guidance_pipeline_and_fakepi_direction():
    result = inspect_motorapp.run_valid_case()
    guidance = result["guidance"]
    control = result["control"]
    out = result["out"]
    l1_in = result["l1_in"]
    l1_out = result["l1_out"]

    assert result["decide_call_count"] == 1
    assert result["l1input_call_count"] == 1
    assert result["l1output_call_count"] == 1
    assert result["ctrlinput_call_count"] == 1
    assert result["ctrloutput_call_count"] == 1
    assert result["log_call_count"] == 1

    assert result["mode"] == guidance.ControlMode.GPS_TRACKING_CLOSED
    assert l1_in is not None and l1_in.valid is True
    assert l1_out is not None and l1_out.valid is True
    assert out.valid is True
    assert out.control_mode == guidance.ControlMode.GPS_TRACKING_CLOSED
    assert math.isclose(guidance._STATE_t.imu.gyr_z, math.radians(-12.0))
    assert math.isclose(guidance._STATE_t.baro.sink_rate, 3.0)

    assert out.delta_arm_deg > 0.0
    assert out.right_angle_deg > control.NEUTRAL_ARM_DEG
    assert out.left_angle_deg < control.NEUTRAL_ARM_DEG
    assert out.right_pw > control.RIGHT_NEUTRAL
    assert out.left_pw > control.LEFT_NEUTRAL
    assert result["last_pwm"] == [
        (control.PARAFOIL_LEFT_MOTOR_PIN, out.left_pw),
        (control.PARAFOIL_RIGHT_MOTOR_PIN, out.right_pw),
    ]


def test_ctrl_cycle_invalid_l1input_goes_neutral_and_logs():
    result = inspect_motorapp.run_invalid_neutral_case()
    guidance = result["guidance"]
    control = result["control"]
    out = result["out"]
    l1_in = result["l1_in"]

    assert result["decide_call_count"] == 1
    assert result["l1input_call_count"] == 1
    assert result["l1output_call_count"] == 0
    assert result["ctrlinput_call_count"] == 0
    assert result["ctrloutput_call_count"] == 0
    assert result["log_call_count"] == 1

    assert result["mode"] == guidance.ControlMode.FAIL
    assert l1_in is not None and l1_in.valid is False
    assert l1_in.reason == "NO_TARGET"
    assert out.valid is False
    assert out.reason == "NO_TARGET"
    assert out.left_pw == control.LEFT_NEUTRAL
    assert out.right_pw == control.RIGHT_NEUTRAL
    assert result["last_pwm"] == [
        (control.PARAFOIL_LEFT_MOTOR_PIN, out.left_pw),
        (control.PARAFOIL_RIGHT_MOTOR_PIN, out.right_pw),
    ]


def test_sensorlog_log_motor_raw_receives_l1input_metadata():
    valid = inspect_motorapp.run_valid_case()
    invalid = inspect_motorapp.run_invalid_neutral_case()

    valid_args, valid_kwargs = valid["log_calls"][-1]
    invalid_args, invalid_kwargs = invalid["log_calls"][-1]

    assert len(valid_args) >= 4
    assert valid_kwargs["event"] == "GPS_TRACKING_CLOSED"
    assert valid_kwargs["l1_in"].valid is True

    assert len(invalid_args) >= 4
    assert invalid_kwargs["event"] == "FAIL"
    assert invalid_kwargs["l1_in"].valid is False
