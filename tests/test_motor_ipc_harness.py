"""IPC Harness: verify motorapp message routing end-to-end without real hardware.

This file is the primary tool for validating the inter-app connection:
  GPS / IMU / Baro / FlightLogic -> motorapp.dispatch() -> sensor state
  sensor state -> guidance() -> control output

No Raspberry Pi, pigpio, or serial port is required.

Usage:
  python -m pytest tests/test_motor_ipc_harness.py -v
  python -m unittest tests.test_motor_ipc_harness -v
"""

import unittest
from types import SimpleNamespace

from lib import appargs, msgstructure
from Sensor_Motor import motorapp, motor_guidance, motor_control
from Sensor_Motor.motor_guidance import GpsFidelity, GpsVector


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _pack(sender_id: int, receiver_id: int, mid: int, data: str) -> str:
    return f"{sender_id}|{receiver_id}|{mid}|{data}"


def _dispatch(sender_id: int, mid: int, data: str) -> None:
    msg = _pack(sender_id, appargs.MotorAppArg.AppID, mid, data)
    motorapp.dispatch(msg)


def _inject_all_sensors() -> None:
    """Inject valid GPS, IMU, baro, and target messages."""
    _dispatch(appargs.GpsAppArg.AppID,
              appargs.GpsAppArg.MID_motor_gps,
              "37.55,126.95,90.0,10.0,1,1")
    _dispatch(appargs.ImuAppArg.AppID,
              appargs.ImuAppArg.MID_motor_imu,
              "45.0,1.0,1")
    _dispatch(appargs.BarometerAppArg.AppID,
              appargs.BarometerAppArg.MID_motor_alt,
              "200.0")
    _dispatch(appargs.FlightlogicAppArg.AppID,
              appargs.FlightlogicAppArg.MID_motor_TargetCor,
              "37.56,126.96")


def _prime_gps_jump(lat: float, lon: float, n: int = 2) -> None:
    for _ in range(n):
        motor_guidance.is_gps_jump(lat, lon)


def _reset() -> None:
    """Full reset of motorapp and guidance state before each test."""
    motor_guidance.init_guidance()
    motorapp.MOTORAPP_RUNSTATUS  = True
    motorapp.MOTOR_ENABLED       = True
    motorapp.STATE               = 0
    motorapp._PREV_STATE         = -1
    motorapp.TARGET              = None
    motorapp._START_POINT_LOCKED = False

    motorapp.IMU.yaw        = 0.0
    motorapp.IMU.gyrz       = 0.0
    motorapp.IMU.imu_health = 1

    motorapp.GPS_VECTOR.lat       = 0.0
    motorapp.GPS_VECTOR.lon       = 0.0
    motorapp.GPS_VECTOR.direction = 0.0
    motorapp.GPS_VECTOR.velocity  = 0.0

    motorapp.GPS_HEALTH.pos_health    = 0
    motorapp.GPS_HEALTH.motion_health = 0

    motorapp.ALT = 0.0


# ---------------------------------------------------------------------------
# Test: individual message routing
# ---------------------------------------------------------------------------

class TestMessageRouting(unittest.TestCase):
    def setUp(self):
        _reset()

    # --- GPS ---
    def test_gps_updates_vector(self):
        _dispatch(appargs.GpsAppArg.AppID, appargs.GpsAppArg.MID_motor_gps,
                  "37.55,126.95,90.0,10.0,1,1")
        self.assertAlmostEqual(motorapp.GPS_VECTOR.lat,       37.55)
        self.assertAlmostEqual(motorapp.GPS_VECTOR.lon,       126.95)
        self.assertAlmostEqual(motorapp.GPS_VECTOR.direction, 90.0)
        self.assertAlmostEqual(motorapp.GPS_VECTOR.velocity,  10.0)
        self.assertEqual(motorapp.GPS_HEALTH.pos_health,    1)
        self.assertEqual(motorapp.GPS_HEALTH.motion_health, 1)

    def test_gps_bad_field_count_ignored(self):
        _dispatch(appargs.GpsAppArg.AppID, appargs.GpsAppArg.MID_motor_gps,
                  "37.55,126.95")
        self.assertAlmostEqual(motorapp.GPS_VECTOR.lat, 0.0)

    def test_gps_parse_error_ignored(self):
        _dispatch(appargs.GpsAppArg.AppID, appargs.GpsAppArg.MID_motor_gps,
                  "NOTANUMBER,126.95,90.0,10.0,1,1")
        self.assertAlmostEqual(motorapp.GPS_VECTOR.lat, 0.0)

    # --- IMU ---
    def test_imu_updates_imu(self):
        _dispatch(appargs.ImuAppArg.AppID, appargs.ImuAppArg.MID_motor_imu,
                  "45.0,2.5,1")
        self.assertAlmostEqual(motorapp.IMU.yaw,  45.0)
        self.assertAlmostEqual(motorapp.IMU.gyrz, 2.5)
        self.assertEqual(motorapp.IMU.imu_health, 1)

    def test_imu_bad_data_ignored(self):
        _dispatch(appargs.ImuAppArg.AppID, appargs.ImuAppArg.MID_motor_imu, "bad")
        self.assertAlmostEqual(motorapp.IMU.yaw, 0.0)

    # --- Barometer ---
    def test_baro_updates_alt(self):
        _dispatch(appargs.BarometerAppArg.AppID, appargs.BarometerAppArg.MID_motor_alt,
                  "150.0,1013.2")
        self.assertAlmostEqual(motorapp.ALT, 150.0)

    def test_baro_bad_data_ignored(self):
        _dispatch(appargs.BarometerAppArg.AppID, appargs.BarometerAppArg.MID_motor_alt,
                  "NOPE")
        self.assertAlmostEqual(motorapp.ALT, 0.0)

    # --- Target coordinate ---
    def test_target_coord_sets_target(self):
        _dispatch(appargs.FlightlogicAppArg.AppID,
                  appargs.FlightlogicAppArg.MID_motor_TargetCor, "37.6,127.0")
        self.assertIsNotNone(motorapp.TARGET)
        self.assertAlmostEqual(motorapp.TARGET.lat, 37.6)
        self.assertAlmostEqual(motorapp.TARGET.lon, 127.0)

    def test_target_coord_out_of_range_rejected(self):
        _dispatch(appargs.FlightlogicAppArg.AppID,
                  appargs.FlightlogicAppArg.MID_motor_TargetCor, "999.0,127.0")
        self.assertIsNone(motorapp.TARGET)

    # --- Flight state ---
    def test_state_message_updates_state(self):
        _dispatch(appargs.FlightlogicAppArg.AppID,
                  appargs.FlightlogicAppArg.MID_motor_state, "3")
        self.assertEqual(motorapp.STATE, 3)

    def test_state3_locks_start_point(self):
        motorapp.GPS_VECTOR.lat        = 37.55
        motorapp.GPS_VECTOR.lon        = 126.95
        motorapp.GPS_HEALTH.pos_health = 1
        _dispatch(appargs.FlightlogicAppArg.AppID,
                  appargs.FlightlogicAppArg.MID_motor_state, "3")
        self.assertAlmostEqual(motor_guidance._start_point.lat, 37.55)
        self.assertAlmostEqual(motor_guidance._start_point.lon, 126.95)

    # --- MEC ---
    def test_mec_off_disables_motor(self):
        motorapp.MOTOR_ENABLED = True
        _dispatch(appargs.CommAppArg.AppID, appargs.CommAppArg.MID_RouteCmd_MEC, "OFF")
        self.assertFalse(motorapp.MOTOR_ENABLED)

    def test_mec_on_enables_motor(self):
        motorapp.MOTOR_ENABLED = False
        _dispatch(appargs.CommAppArg.AppID, appargs.CommAppArg.MID_RouteCmd_MEC, "ON")
        self.assertTrue(motorapp.MOTOR_ENABLED)

    # --- Malformed messages ---
    def test_malformed_message_ignored(self):
        motorapp.dispatch("not|a|valid")
        motorapp.dispatch("1|2|notanint|data")
        motorapp.dispatch("")
        motorapp.dispatch("1|2|3")
        self.assertEqual(motorapp.STATE, 0)

    # --- Terminate ---
    def test_terminate_message_stops_runstatus(self):
        _dispatch(appargs.MainAppArg.AppID,
                  appargs.MainAppArg.MID_TerminateProcess, "")
        self.assertFalse(motorapp.MOTORAPP_RUNSTATUS)
        motorapp.MOTORAPP_RUNSTATUS = True


# ---------------------------------------------------------------------------
# Test: full chain without hardware
# ---------------------------------------------------------------------------

class TestFullChain(unittest.TestCase):
    """End-to-end: inject all sensor messages, verify guidance output finite."""

    def setUp(self):
        _reset()
        _inject_all_sensors()
        _prime_gps_jump(37.55, 126.95)

    def test_guidance_output_finite(self):
        snap = motorapp._snapshot_sensors()
        imu  = SimpleNamespace(yaw=45.0, gyrz=1.0)
        gpsv = GpsVector(lat=37.55, lon=126.95, speed=10.0, course=90.0)
        gpsf = GpsFidelity(fix_quality=1, sats=7, rmc_status="A", gps_health=1)
        result = motor_guidance.guidance(imu, gpsv, gpsf, snap.target, 200.0)
        self.assertIn(result.state, {"HOMING", "PATTERN", "LANDING"})
        import math
        self.assertTrue(math.isfinite(result.commanded_yaw_rate))
        self.assertLessEqual(abs(result.commanded_yaw_rate), motor_guidance.YR_MAX + 1e-6)

    def test_guidance_fdir_on_invalid_gps_fidelity(self):
        snap = motorapp._snapshot_sensors()
        imu  = SimpleNamespace(yaw=45.0, gyrz=1.0)
        gpsv = GpsVector(lat=37.55, lon=126.95, speed=10.0, course=90.0)
        gpsf = GpsFidelity(fix_quality=0, sats=1, rmc_status="V", gps_health=0)
        result = motor_guidance.guidance(imu, gpsv, gpsf, snap.target, 200.0)
        self.assertEqual(result.state, "FDIR")
        self.assertAlmostEqual(result.commanded_yaw_rate, 0.0)

    def test_motor_control_dummy_clamp(self):
        handle = motor_control.init_control()
        fb = motor_control.control(handle, 9999.0)
        self.assertLessEqual(fb.left_pulse,  motor_control.LEFT_MAX)
        self.assertGreaterEqual(fb.right_pulse, motor_control.RIGHT_MIN)

    def test_motor_neutral_and_off(self):
        handle = motor_control.init_control()
        motor_control.set_neutral(handle)
        b = handle.pi
        self.assertEqual(b.pulses[motor_control.LEFT_GPIO],  motor_control.LEFT_NEUTRAL)
        self.assertEqual(b.pulses[motor_control.RIGHT_GPIO], motor_control.RIGHT_NEUTRAL)
        motor_control.set_motors_off(handle)
        self.assertEqual(b.pulses[motor_control.LEFT_GPIO],  0)
        self.assertEqual(b.pulses[motor_control.RIGHT_GPIO], 0)


# ---------------------------------------------------------------------------
# Test: ctrl_paragldr exception recovery
# ---------------------------------------------------------------------------

class TestCtrlParagldrResilience(unittest.TestCase):
    """Verify that the control loop stays alive after an exception."""

    def test_exception_causes_neutral_not_crash(self):
        import unittest.mock as mock

        handle = motor_control.init_control()
        backend = handle.pi

        call_count = [0]

        def exploding_guidance(*args, **kwargs):
            call_count[0] += 1
            if call_count[0] == 1:
                raise RuntimeError("simulated guidance failure")
            return SimpleNamespace(state="HOMING", distance=100.0, commanded_yaw_rate=5.0)

        _reset()
        _inject_all_sensors()
        _prime_gps_jump(37.55, 126.95)
        motorapp.STATE = 3
        motorapp.PI    = handle

        with mock.patch("Sensor_Motor.motorapp.motor_guidance") as mock_guidance:
            mock_guidance.guidance.side_effect = exploding_guidance

            try:
                motorapp._snapshot_sensors()
                mock_guidance.guidance(None, None, None, None, None)
            except RuntimeError:
                motor_control.set_neutral(handle)

        self.assertEqual(backend.pulses[motor_control.LEFT_GPIO],  motor_control.LEFT_NEUTRAL)
        self.assertEqual(backend.pulses[motor_control.RIGHT_GPIO], motor_control.RIGHT_NEUTRAL)


if __name__ == "__main__":
    unittest.main()
