"""IPC harness: motorapp dispatch routing and end-to-end guidance/actuator flow."""

import math
import sys
import time
import unittest
from unittest import mock

from lib import appargs
from Sensor_Motor import motorapp, control, guidance
from Sensor_Motor.motorapp import _Cache


class _FakePi:
    def __init__(self):
        self.pulses = {}
        self.connected = 1

    def set_servo_pulsewidth(self, pin, width):
        self.pulses[pin] = width


class _FakePigpio:
    @staticmethod
    def pi():
        return _FakePi()


def _dispatch(sender_id: int, mid: int, data: str) -> None:
    msg = f"{sender_id}|{appargs.MotorAppArg.AppID}|{mid}|{data}"
    motorapp.dispatch(msg)


def _gps_msg(lat=37.55, lon=126.95, course=90.0, speed=10.0, ts=None):
    ts = time.monotonic() if ts is None else ts
    return f"{lat},{lon},{ts:.4f},{course},{speed},{ts:.4f}"


def _imu_msg(gyrz=2.5, ts=None):
    ts = time.monotonic() if ts is None else ts
    return f"1.0,2.0,45.0,0.1,0.2,0.3,0.4,0.5,0.6,0.0,0.0,{gyrz},{ts:.4f},0,0"


def _baro_msg(alt=200.0, sink=1.0, ts=None):
    ts = time.monotonic() if ts is None else ts
    return f"{alt},{ts:.4f},{sink}"


def _reset() -> None:
    motorapp.MOTORAPP_RUNSTATUS = True
    motorapp.MOTOR_ENABLED = True
    motorapp.MANUAL_STEER_MODE = "NEUTRAL"
    motorapp.STATE = 0
    motorapp._PREV_STATE = -1
    motorapp._ORIGIN_SAVED = False
    motorapp._CONTROLLER = None
    motorapp.PI = None
    motorapp._CACHE = _Cache()
    motorapp._GUIDANCE_STATE = guidance.GuidanceState()
    motorapp._TARGET_LAT = None
    motorapp._TARGET_LON = None
    motorapp._START_LAT = None
    motorapp._START_LON = None


class TestMessageRouting(unittest.TestCase):
    def setUp(self):
        _reset()

    def test_terminate_message_stops_runstatus(self):
        _dispatch(appargs.MainAppArg.AppID, appargs.MainAppArg.MID_TerminateProcess, "")
        self.assertFalse(motorapp.MOTORAPP_RUNSTATUS)

    def test_gps_message_updates_cache(self):
        _dispatch(appargs.GpsAppArg.AppID, appargs.GpsAppArg.MID_motor_gps,
                  _gps_msg())
        self.assertAlmostEqual(motorapp._CACHE.latest_gps.lat, 37.55)
        self.assertAlmostEqual(motorapp._CACHE.latest_gps.lon, 126.95)
        self.assertTrue(motorapp._CACHE.latest_gps.pos_health)

    def test_imu_message_updates_cache(self):
        _dispatch(appargs.ImuAppArg.AppID, appargs.ImuAppArg.MID_motor_imu,
                  _imu_msg())
        self.assertAlmostEqual(
            motorapp._CACHE.latest_imu.gyrz_rad_s, math.radians(-2.5), places=5  # negated: IMU gz+ = CCW
        )

    def test_baro_message_updates_cache(self):
        _dispatch(appargs.BarometerAppArg.AppID, appargs.BarometerAppArg.MID_motor_alt, _baro_msg())
        self.assertAlmostEqual(motorapp._CACHE.latest_baro.alt_m, 200.0)

    def test_target_coord_message_updates_cache(self):
        _dispatch(appargs.FlightlogicAppArg.AppID,
                  appargs.FlightlogicAppArg.MID_motor_TargetCor, "37.56,126.96")
        self.assertAlmostEqual(motorapp._CACHE.target_lat, 37.56)
        self.assertAlmostEqual(motorapp._CACHE.target_lon, 126.96)

    def test_state3_alone_does_not_set_origin(self):
        """Origin lock now happens only inside ctrl_parafoil's guidance call,
        not as a side effect of handle_gps or handle_flight_state."""
        _dispatch(appargs.GpsAppArg.AppID, appargs.GpsAppArg.MID_motor_gps,
                  _gps_msg())
        _dispatch(appargs.FlightlogicAppArg.AppID,
                  appargs.FlightlogicAppArg.MID_motor_state, "3")
        self.assertIsNone(motorapp._CACHE.start_lat)
        self.assertFalse(motorapp._GUIDANCE_STATE.origin_ready)

    def test_mec_off_disables_motor(self):
        _dispatch(appargs.CommAppArg.AppID, appargs.CommAppArg.MID_RouteCmd_MEC, "OFF")
        self.assertFalse(motorapp.MOTOR_ENABLED)

    def test_mec_on_enables_motor(self):
        motorapp.MOTOR_ENABLED = False
        _dispatch(appargs.CommAppArg.AppID, appargs.CommAppArg.MID_RouteCmd_MEC, "ON")
        self.assertTrue(motorapp.MOTOR_ENABLED)

    def test_mtr_right_updates_manual_mode(self):
        _dispatch(appargs.CommAppArg.AppID, appargs.CommAppArg.MID_RouteCmd_MTR, "RIGHT")
        self.assertEqual(motorapp.MANUAL_STEER_MODE, "RIGHT")

    def test_invalid_message_no_crash(self):
        motorapp.dispatch("bad_message_no_pipes")
        motorapp.dispatch("1|2|3|4|5")  # too many fields

    def test_state_transition_via_dispatch(self):
        _dispatch(appargs.FlightlogicAppArg.AppID,
                  appargs.FlightlogicAppArg.MID_motor_state, "4")
        self.assertEqual(motorapp.STATE, 4)


class TestGuidanceAndActuatorIntegration(unittest.TestCase):
    def setUp(self):
        _reset()
        _dispatch(appargs.GpsAppArg.AppID, appargs.GpsAppArg.MID_motor_gps,
                  _gps_msg())
        _dispatch(appargs.ImuAppArg.AppID, appargs.ImuAppArg.MID_motor_imu,
                  _imu_msg())
        _dispatch(appargs.BarometerAppArg.AppID, appargs.BarometerAppArg.MID_motor_alt,
                  _baro_msg())
        _dispatch(appargs.FlightlogicAppArg.AppID,
                  appargs.FlightlogicAppArg.MID_motor_TargetCor, "37.56,126.96")
        _dispatch(appargs.FlightlogicAppArg.AppID,
                  appargs.FlightlogicAppArg.MID_motor_state, "3")

    def test_snapshot_reflects_all_dispatched_data(self):
        snap = motorapp._cache_snapshot()
        self.assertAlmostEqual(snap.latest_gps.lat, 37.55)
        self.assertAlmostEqual(snap.latest_imu.gyrz_rad_s, math.radians(-2.5), places=5)  # negated
        self.assertAlmostEqual(snap.latest_baro.alt_m, 200.0)
        self.assertAlmostEqual(motorapp._TARGET_LAT, 37.56)
        # _START_LAT stays None until the ctrl_parafoil loop runs and triggers
        # the origin-sync one-shot (guidance pipeline sets _GUIDANCE_STATE.origin
        # → _START_LAT/_START_LON); the dispatch-only setup never runs that loop.
        self.assertIsNone(motorapp._START_LAT)

    def test_neutral_command_outputs_to_servo(self):
        with mock.patch.dict(sys.modules, {"pigpio": _FakePigpio()}):
            pi = control.init_control()
            cmd = control.WriteNeutral(time.monotonic())
            control.ProducePulse(pi, cmd)
            self.assertEqual(pi.pulses[control.PARAFOIL_LEFT_MOTOR_PIN], control.LEFT_NEUTRAL)
            self.assertEqual(pi.pulses[control.PARAFOIL_RIGHT_MOTOR_PIN], control.RIGHT_NEUTRAL)

    def test_set_motors_off_clears_pulses(self):
        with mock.patch.dict(sys.modules, {"pigpio": _FakePigpio()}):
            pi = control.init_control()
            control.WriteOff(pi)
            self.assertEqual(pi.pulses[control.PARAFOIL_LEFT_MOTOR_PIN], 0)
            self.assertEqual(pi.pulses[control.PARAFOIL_RIGHT_MOTOR_PIN], 0)

    def test_controller_update_produces_valid_pulses(self):
        with mock.patch.dict(sys.modules, {"pigpio": _FakePigpio()}):
            pi = control.init_control()
            control.reset()
            gcmd = control.CtrlInput(
                angular_velocity_cmd_deg_s=10.0, valid=True, timestamp=time.monotonic()
            )
            cmd = control.ProduceCtrlOutput(gcmd, float("nan"), time.monotonic())
            control.MoveServo(pi, cmd)
            self.assertGreaterEqual(pi.pulses[control.PARAFOIL_LEFT_MOTOR_PIN],
                                    control.LEFT_MIN_PULSE)
            self.assertLessEqual(pi.pulses[control.PARAFOIL_LEFT_MOTOR_PIN],
                                 control.LEFT_MAX_PULSE)

    def test_manual_steer_left_command_outputs_turning_pulses(self):
        cmd = motorapp._manual_steer_command(time.monotonic(), "LEFT")
        self.assertNotEqual(cmd.left_pw, control.LEFT_NEUTRAL)
        self.assertNotEqual(cmd.right_pw, control.RIGHT_NEUTRAL)
        self.assertLess(cmd.delta_arm_deg, 0.0)
        self.assertGreater(cmd.left_angle_deg, control.NEUTRAL_ARM_DEG)
        self.assertLess(cmd.right_angle_deg, control.NEUTRAL_ARM_DEG)

    def test_manual_steer_right_command_outputs_turning_pulses(self):
        cmd = motorapp._manual_steer_command(time.monotonic(), "RIGHT")
        self.assertNotEqual(cmd.left_pw, control.LEFT_NEUTRAL)
        self.assertNotEqual(cmd.right_pw, control.RIGHT_NEUTRAL)
        self.assertGreater(cmd.delta_arm_deg, 0.0)
        self.assertLess(cmd.left_angle_deg, control.NEUTRAL_ARM_DEG)
        self.assertGreater(cmd.right_angle_deg, control.NEUTRAL_ARM_DEG)


if __name__ == "__main__":
    unittest.main()
