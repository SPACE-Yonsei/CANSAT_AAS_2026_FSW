import importlib
import math
from unittest.mock import patch

from lib import config
from Sensor_Motor import control, guidance, motorapp


NOW = 1000.0
ORIGIN_LAT = 37.0
ORIGIN_LON = 127.0
TARGET_E_M = 100.0


class FakePi:
    def __init__(self):
        self.calls = []

    def set_servo_pulsewidth(self, pin, pulsewidth):
        self.calls.append((pin, pulsewidth))


def _latlon_from_ne(north_m: float, east_m: float) -> tuple[float, float]:
    lat = ORIGIN_LAT + math.degrees(north_m / guidance.EARTH_RADIUS_M)
    lon = ORIGIN_LON + math.degrees(
        east_m / (guidance.EARTH_RADIUS_M * math.cos(math.radians(ORIGIN_LAT)))
    )
    return lat, lon


def _reload_stack():
    global control, guidance, motorapp
    guidance = importlib.reload(guidance)
    control = importlib.reload(control)
    motorapp = importlib.reload(motorapp)
    control.reset()


def test_payload_handlers_parse_sign_acc_and_servo_direction():
    _reload_stack()
    fake_pi = FakePi()
    target_lat, target_lon = _latlon_from_ne(0.0, TARGET_E_M)

    motorapp.STATE = 4
    motorapp.MOTOR_ENABLED = True
    motorapp.PI = fake_pi
    motorapp._STEER_MODE = ""
    motorapp._ORIGIN_LOCKED = True
    guidance.set_origin_point(ORIGIN_LAT, ORIGIN_LON)
    guidance.set_target_point(target_lat, target_lon)

    with (
        patch.object(motorapp.time, "monotonic", return_value=NOW),
        patch.object(motorapp.sensorlog, "log_motor_raw", return_value=None),
        patch.object(motorapp.prevstate, "update_start_point", return_value=None),
    ):
        motorapp.handle_gps(
            f"{ORIGIN_LAT},{ORIGIN_LON},1,{NOW},0.0,5.0,1,{NOW}"
        )
        motorapp.handle_imu(
            f"0.0,0.0,0.0,0.0,0.0,9.81,0.0,0.0,12.0,1,{NOW},0.0"
        )
        motorapp.handle_barometer("120.0,3.0,1")
        out = motorapp._ctrl_cycle(None, NOW)

    cached_imu = motorapp._CACHE_t.latest_imu
    assert math.isclose(cached_imu.gyrz_rad_s, math.radians(-12.0))
    assert cached_imu.lin_acc_valid is True
    assert cached_imu.lin_acc_reject_reason == "OK"

    assert out is not None
    assert out.valid is True
    assert out.control_mode == guidance.ControlMode.GPS_TRACKING_CLOSED
    assert out.delta_ff_deg > 0.0
    assert out.delta_arm_deg > 0.0
    assert math.isclose(guidance._STATE_t.imu.gyr_z, math.radians(-12.0))

    assert out.right_angle_deg > control.NEUTRAL_ARM_DEG
    assert out.left_angle_deg < control.NEUTRAL_ARM_DEG
    assert out.right_pw > control.RIGHT_NEUTRAL
    assert out.left_pw > control.LEFT_NEUTRAL
    assert fake_pi.calls[-2:] == [
        (control.PARAFOIL_LEFT_MOTOR_PIN, out.left_pw),
        (control.PARAFOIL_RIGHT_MOTOR_PIN, out.right_pw),
    ]

    assert abs(out.delta_arm_deg) <= control.DELTA_ARM_MAX_DEG
    assert out.kp_used == config.KP_GPS_CLOSED
