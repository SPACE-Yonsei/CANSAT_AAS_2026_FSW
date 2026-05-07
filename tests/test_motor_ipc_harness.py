"""IPC harness for motor app routing and guidance contracts."""

import sys
import time
import unittest
from types import SimpleNamespace
from unittest import mock

from lib import appargs
from Sensor_Motor import motorapp, motor_control, motor_guidance


class _FakePi:
    def __init__(self):
        self.pulses = {}
        self.connected = 1

    def set_servo_pulsewidth(self, pin, width):
        self.pulses[pin] = width

    def stop(self):
        return None


class _FakePigpio:
    @staticmethod
    def pi():
        return _FakePi()


def _dispatch(sender_id: int, mid: int, data: str) -> None:
    msg = f"{sender_id}|{appargs.MotorAppArg.AppID}|{mid}|{data}"
    motorapp.dispatch(msg)


def _reset() -> None:
    motor_guidance.init_guidance()
    motorapp.MOTORAPP_RUNSTATUS = True
    motorapp.MOTOR_ENABLED = True
    motorapp.STATE = 0
    motorapp._PREV_STATE = -1
    motorapp.TARGET = None
    motorapp._START_POINT_LOCKED = False
    motorapp.IMU.yaw = 0.0
    motorapp.IMU.gyrz = 0.0
    motorapp.IMU.imu_health = 1
    motorapp.GPS_VECTOR.lat = 0.0
    motorapp.GPS_VECTOR.lon = 0.0
    motorapp.GPS_VECTOR.direction = 0.0
    motorapp.GPS_VECTOR.velocity = 0.0
    motorapp.GPS_HEALTH.pos_health = 0
    motorapp.GPS_HEALTH.motion_health = 0
    motorapp.ALT = 0.0


class TestMessageRouting(unittest.TestCase):
    def setUp(self):
        _reset()

    def test_gps_updates_vector(self):
        _dispatch(appargs.GpsAppArg.AppID, appargs.GpsAppArg.MID_motor_gps, "37.55,126.95,90.0,10.0,1,1")
        self.assertAlmostEqual(motorapp.GPS_VECTOR.lat, 37.55)
        self.assertAlmostEqual(motorapp.GPS_VECTOR.lon, 126.95)
        self.assertEqual(motorapp.GPS_HEALTH.pos_health, 1)

    def test_state3_locks_start_point(self):
        motorapp.GPS_VECTOR.lat = 37.55
        motorapp.GPS_VECTOR.lon = 126.95
        motorapp.GPS_HEALTH.pos_health = 1
        _dispatch(appargs.FlightlogicAppArg.AppID, appargs.FlightlogicAppArg.MID_motor_state, "3")
        self.assertAlmostEqual(motor_guidance.START_POINT.lat, 37.55)
        self.assertAlmostEqual(motor_guidance.START_POINT.lon, 126.95)

    def test_terminate_message_stops_runstatus(self):
        _dispatch(appargs.MainAppArg.AppID, appargs.MainAppArg.MID_TerminateProcess, "")
        self.assertFalse(motorapp.MOTORAPP_RUNSTATUS)


class TestGuidanceAndActuator(unittest.TestCase):
    def setUp(self):
        _reset()
        _dispatch(appargs.GpsAppArg.AppID, appargs.GpsAppArg.MID_motor_gps, "37.55,126.95,90.0,10.0,1,1")
        _dispatch(appargs.ImuAppArg.AppID, appargs.ImuAppArg.MID_motor_imu, "45.0,1.0,1")
        _dispatch(appargs.BarometerAppArg.AppID, appargs.BarometerAppArg.MID_motor_alt, "200.0")
        _dispatch(appargs.FlightlogicAppArg.AppID, appargs.FlightlogicAppArg.MID_motor_TargetCor, "37.56,126.96")
        motor_guidance.set_start_coordinates(37.55, 126.95)
        motor_guidance._PREV_GPS.initialized = True
        motor_guidance._PREV_GPS.lat = 37.55
        motor_guidance._PREV_GPS.lon = 126.95
        motor_guidance._PREV_GPS.time = time.time() - 1.0
        motor_guidance._GPS_STABLE_COUNT = motor_guidance.GPS_STABLE_COUNT_REQUIRED

    def test_guidance_output_finite(self):
        snap = motorapp._snapshot_sensors()
        imu = SimpleNamespace(yaw=45.0, gyrz=1.0)
        gps = SimpleNamespace(lat=37.55, lon=126.95, direction=90.0, velocity=10.0)
        fid = SimpleNamespace(pos_health=1, motion_health=1)
        result = motor_guidance.guidance(imu, gps, fid, snap.target, 200.0)
        self.assertIn(result.state, {"STRAIGHT", "TURNING", "PATTERN", "TARGET_REACHED"})
        self.assertTrue(abs(result.commanded_yaw_rate) <= motor_guidance.CASCADE_PI.MAX_CMD + 1e-6)

    def test_actuator_paths_without_hardware(self):
        with mock.patch.dict(sys.modules, {"pigpio": _FakePigpio()}):
            handle = motor_control.init_control()
            fb = motor_control.control(handle, 9999.0)
            self.assertLessEqual(fb.left_pulse, motor_control.LEFT_MAX_PULSE)
            self.assertGreaterEqual(fb.right_pulse, motor_control.RIGHT_MIN_PULSE)
            motor_control.set_motors_off(handle)
            self.assertEqual(handle.pulses[motor_control.PARAFOIL_LEFT_MOTOR_PIN], 0)
            self.assertEqual(handle.pulses[motor_control.PARAFOIL_RIGHT_MOTOR_PIN], 0)


if __name__ == "__main__":
    unittest.main()
