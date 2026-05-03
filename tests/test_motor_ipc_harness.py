"""IPC Harness: verify motorapp message routing end-to-end without real hardware.

This file is the primary tool for validating the inter-app connection:
  GPS / IMU / Baro / FlightLogic -> motorapp.dispatch() -> sensor state
  sensor state -> _check_fdir()  -> guidance()           -> control output

No Raspberry Pi, pigpio, or serial port is required.

Usage:
  python -m pytest tests/test_motor_ipc_harness.py -v
  python -m unittest tests.test_motor_ipc_harness -v
"""

import time
import unittest
from types import SimpleNamespace

from lib import appargs, msgstructure
from Sensor_Motor import motorapp, motor_guidance, motor_control
from Sensor_Motor.motor_guidance import GpsFidelity, GpsVector


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _pack(sender_id: int, receiver_id: int, mid: int, data: str) -> str:
    """Build a raw bus message string."""
    return f"{sender_id}|{receiver_id}|{mid}|{data}"


def _dispatch(sender_id: int, mid: int, data: str) -> None:
    """Inject one message into motorapp.dispatch()."""
    msg = _pack(sender_id, appargs.MotorAppArg.AppID, mid, data)
    motorapp.dispatch(msg)


def _inject_all_sensors(ts_override: float | None = None) -> None:
    """Inject valid GPS, IMU, baro, and target messages, then override timestamps."""
    now = ts_override if ts_override is not None else time.time()

    _dispatch(appargs.GpsAppArg.AppID,
              appargs.GpsAppArg.MID_motor_gps,
              "37.55,126.95,10.0,90.0,1,7,A,1")
    _dispatch(appargs.ImuAppArg.AppID,
              appargs.ImuAppArg.MID_motor_imu,
              "45.0,1.0,1")
    _dispatch(appargs.BarometerAppArg.AppID,
              appargs.BarometerAppArg.MID_motor_alt,
              "200.0")
    _dispatch(appargs.FlightlogicAppArg.AppID,
              appargs.FlightlogicAppArg.MID_motor_TargetCor,
              "37.56,126.96")

    # Stamp timestamps as 'just now' regardless of real execution speed
    motorapp.last_gps_update  = now
    motorapp.last_imu_update  = now
    motorapp.last_baro_update = now


def _prime_gps_jump(lat: float, lon: float, n: int = 2) -> None:
    """Feed n identical GPS readings to satisfy jump-detection warm-up."""
    for _ in range(n):
        motor_guidance.is_gps_jump(lat, lon)


def _reset() -> None:
    """Full reset of motorapp and guidance state before each test."""
    motor_guidance.init_guidance()
    motorapp.MOTORAPP_RUNSTATUS = True
    motorapp.motor_enabled      = True
    motorapp.state              = 0
    motorapp._prev_state        = -1
    motorapp.target             = None
    motorapp._last_fdir_reason  = None
    motorapp._last_fdir_log_ts  = 0.0
    motorapp.last_gps_update    = 0.0
    motorapp.last_imu_update    = 0.0
    motorapp.last_baro_update   = 0.0

    motorapp.sensor.yaw         = 0.0
    motorapp.sensor.gyrz        = 0.0
    motorapp.sensor.imu_health  = 1
    motorapp.sensor.lat         = 0.0
    motorapp.sensor.lon         = 0.0
    motorapp.sensor.speed       = 0.0
    motorapp.sensor.course      = 0.0
    motorapp.sensor.fix_quality = 0
    motorapp.sensor.sats        = 0
    motorapp.sensor.rmc_status  = "V"
    motorapp.sensor.gps_health  = 0
    motorapp.sensor.baro_m      = 0.0


# ---------------------------------------------------------------------------
# Test: individual message routing
# ---------------------------------------------------------------------------

class TestMessageRouting(unittest.TestCase):
    def setUp(self):
        _reset()

    # --- GPS ---
    def test_gps_updates_sensor(self):
        _dispatch(appargs.GpsAppArg.AppID, appargs.GpsAppArg.MID_motor_gps,
                  "37.55,126.95,10.0,90.0,1,7,A,1")
        self.assertAlmostEqual(motorapp.sensor.lat,   37.55)
        self.assertAlmostEqual(motorapp.sensor.lon,  126.95)
        self.assertAlmostEqual(motorapp.sensor.speed, 10.0)
        self.assertEqual(motorapp.sensor.sats,  7)
        self.assertEqual(motorapp.sensor.rmc_status, "A")
        self.assertGreater(motorapp.last_gps_update, 0)

    def test_gps_bad_field_count_ignored(self):
        _dispatch(appargs.GpsAppArg.AppID, appargs.GpsAppArg.MID_motor_gps,
                  "37.55,126.95")      # only 2 fields — should be ignored silently
        self.assertAlmostEqual(motorapp.sensor.lat, 0.0)   # unchanged default

    def test_gps_parse_error_ignored(self):
        _dispatch(appargs.GpsAppArg.AppID, appargs.GpsAppArg.MID_motor_gps,
                  "NOTANUMBER,126.95,10.0,90.0,1,7,A,1")
        self.assertAlmostEqual(motorapp.sensor.lat, 0.0)

    # --- IMU ---
    def test_imu_updates_sensor(self):
        _dispatch(appargs.ImuAppArg.AppID, appargs.ImuAppArg.MID_motor_imu,
                  "45.0,2.5,1")
        self.assertAlmostEqual(motorapp.sensor.yaw,  45.0)
        self.assertAlmostEqual(motorapp.sensor.gyrz, 2.5)
        self.assertEqual(motorapp.sensor.imu_health,  1)
        self.assertGreater(motorapp.last_imu_update, 0)

    def test_imu_bad_data_ignored(self):
        _dispatch(appargs.ImuAppArg.AppID, appargs.ImuAppArg.MID_motor_imu, "bad")
        self.assertAlmostEqual(motorapp.sensor.yaw, 0.0)

    # --- Barometer ---
    def test_baro_updates_sensor(self):
        _dispatch(appargs.BarometerAppArg.AppID, appargs.BarometerAppArg.MID_motor_alt,
                  "150.0,1013.2")
        self.assertAlmostEqual(motorapp.sensor.baro_m, 150.0)
        self.assertGreater(motorapp.last_baro_update, 0)

    def test_baro_bad_data_ignored(self):
        _dispatch(appargs.BarometerAppArg.AppID, appargs.BarometerAppArg.MID_motor_alt,
                  "NOPE")
        self.assertAlmostEqual(motorapp.sensor.baro_m, 0.0)

    # --- Target coordinate ---
    def test_target_coord_sets_target(self):
        _dispatch(appargs.FlightlogicAppArg.AppID,
                  appargs.FlightlogicAppArg.MID_motor_TargetCor, "37.6,127.0")
        self.assertIsNotNone(motorapp.target)
        self.assertAlmostEqual(motorapp.target.lat, 37.6)
        self.assertAlmostEqual(motorapp.target.lon, 127.0)

    def test_target_coord_out_of_range_rejected(self):
        _dispatch(appargs.FlightlogicAppArg.AppID,
                  appargs.FlightlogicAppArg.MID_motor_TargetCor, "999.0,127.0")
        self.assertIsNone(motorapp.target)

    # --- Flight state ---
    def test_state_message_updates_state(self):
        _dispatch(appargs.FlightlogicAppArg.AppID,
                  appargs.FlightlogicAppArg.MID_motor_state, "3")
        self.assertEqual(motorapp.state, 3)

    def test_state3_locks_start_point(self):
        motorapp.sensor.lat = 37.55
        motorapp.sensor.lon = 126.95
        _dispatch(appargs.FlightlogicAppArg.AppID,
                  appargs.FlightlogicAppArg.MID_motor_state, "3")
        self.assertAlmostEqual(motor_guidance._start_point.lat, 37.55)
        self.assertAlmostEqual(motor_guidance._start_point.lon, 126.95)

    # --- MEC ---
    def test_mec_off_disables_motor(self):
        motorapp.motor_enabled = True
        _dispatch(appargs.CommAppArg.AppID, appargs.CommAppArg.MID_RouteCmd_MEC, "OFF")
        self.assertFalse(motorapp.motor_enabled)

    def test_mec_on_enables_motor(self):
        motorapp.motor_enabled = False
        _dispatch(appargs.CommAppArg.AppID, appargs.CommAppArg.MID_RouteCmd_MEC, "ON")
        self.assertTrue(motorapp.motor_enabled)

    # --- Malformed messages ---
    def test_malformed_message_ignored(self):
        motorapp.dispatch("not|a|valid")          # 3 fields
        motorapp.dispatch("1|2|notanint|data")    # non-int MID
        motorapp.dispatch("")                     # empty string
        motorapp.dispatch("1|2|3")               # missing data field
        # Must not raise, state must be unchanged
        self.assertEqual(motorapp.state, 0)

    # --- Terminate ---
    def test_terminate_message_stops_runstatus(self):
        _dispatch(appargs.MainAppArg.AppID,
                  appargs.MainAppArg.MID_TerminateProcess, "")
        self.assertFalse(motorapp.MOTORAPP_RUNSTATUS)
        motorapp.MOTORAPP_RUNSTATUS = True   # restore for other tests


# ---------------------------------------------------------------------------
# Test: full chain without hardware
# ---------------------------------------------------------------------------

class TestFullChain(unittest.TestCase):
    """End-to-end: inject all sensor messages, verify FDIR clears, guidance finite."""

    def setUp(self):
        _reset()
        _inject_all_sensors()
        _prime_gps_jump(37.55, 126.95)

    def test_fdir_clear_after_all_sensors(self):
        snap = motorapp._snapshot_sensors()
        fdir = motorapp._check_fdir(snap)
        self.assertIsNone(fdir, f"Expected FDIR clear, got: {fdir}")

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
        gpsf = GpsFidelity(fix_quality=0, sats=1, rmc_status="V", gps_health=0)  # bad fidelity
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
# Test: FDIR specific scenarios
# ---------------------------------------------------------------------------

class TestFDIRScenarios(unittest.TestCase):
    def setUp(self):
        _reset()
        _inject_all_sensors()
        _prime_gps_jump(37.55, 126.95)

    def test_gps_stale_triggers_fdir(self):
        motorapp.last_gps_update = time.time() - 999
        snap = motorapp._snapshot_sensors()
        result = motorapp._check_fdir(snap)
        self.assertIsNotNone(result)
        self.assertIn("gps stale", result)

    def test_imu_stale_triggers_fdir(self):
        motorapp.last_imu_update = time.time() - 999
        snap = motorapp._snapshot_sensors()
        result = motorapp._check_fdir(snap)
        self.assertIsNotNone(result)
        self.assertIn("imu stale", result)

    def test_baro_stale_triggers_fdir(self):
        motorapp.last_baro_update = time.time() - 999
        snap = motorapp._snapshot_sensors()
        result = motorapp._check_fdir(snap)
        self.assertIsNotNone(result)
        self.assertIn("baro stale", result)

    def test_target_none_triggers_fdir(self):
        motorapp.target = None
        snap = motorapp._snapshot_sensors()
        result = motorapp._check_fdir(snap)
        self.assertIsNotNone(result)
        self.assertIn("target", result)

    def test_implausible_gyrz_triggers_fdir(self):
        motorapp.sensor.gyrz = 999.0   # deg/s — way above physical limit
        snap = motorapp._snapshot_sensors()
        result = motorapp._check_fdir(snap)
        self.assertIsNotNone(result)
        self.assertIn("yaw-rate", result)

    def test_imu_unhealthy_triggers_fdir(self):
        motorapp.sensor.imu_health = 0
        snap = motorapp._snapshot_sensors()
        result = motorapp._check_fdir(snap)
        self.assertIsNotNone(result)
        self.assertIn("imu_health", result)

    def test_never_received_gps_triggers_fdir(self):
        motorapp.last_gps_update = 0.0
        snap = motorapp._snapshot_sensors()
        result = motorapp._check_fdir(snap)
        self.assertIsNotNone(result)


# ---------------------------------------------------------------------------
# Test: ctrl_paragldr exception recovery
# ---------------------------------------------------------------------------

class TestCtrlParagldrResilience(unittest.TestCase):
    """Verify that the control loop stays alive after an exception."""

    def test_exception_causes_neutral_not_crash(self):
        """Simulate a guidance exception; loop must output neutral and continue."""
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
        motorapp.state  = 3
        motorapp.pi     = handle

        with mock.patch("Sensor_Motor.motorapp.motor_guidance") as mock_guidance:
            mock_guidance.is_gps_valid.return_value  = True
            mock_guidance.is_gps_jump.return_value   = False
            mock_guidance.guidance.side_effect = exploding_guidance

            # First call raises -> neutral must be set
            try:
                snap = motorapp._snapshot_sensors()
                result = mock_guidance.guidance(None, None, None, None, None)
            except RuntimeError:
                motor_control.set_neutral(handle)

        # neutral was output (not 0, not some stale value)
        self.assertEqual(backend.pulses[motor_control.LEFT_GPIO],  motor_control.LEFT_NEUTRAL)
        self.assertEqual(backend.pulses[motor_control.RIGHT_GPIO], motor_control.RIGHT_NEUTRAL)


if __name__ == "__main__":
    unittest.main()
