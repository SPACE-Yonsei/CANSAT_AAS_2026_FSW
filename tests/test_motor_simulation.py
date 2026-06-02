"""Prevstate-driven simulation tests.

These tests load a realistic prevstate JSON dict, run motorapp.init(), then
drive ctrl_parafoil one cycle at a time via the extracted _ctrl_cycle helper.
They verify the full data path:

    prevstate.json → init() → handlers → _CACHE → guidance pipeline
        → control pipeline → PWM commands + diag telemetry

No threads, no IPC, no pigpio — entirely deterministic.
"""

from __future__ import annotations

import json
import math
import time
import unittest
from pathlib import Path
from unittest import mock

from lib import appargs, config, msgstructure, prevstate
from Sensor_Motor import control, guidance, motorapp
from Sensor_Motor.motorapp import _Cache


# ════════════════════════════════════════════════════════════════════════════
# Helpers
# ════════════════════════════════════════════════════════════════════════════

# User-supplied prevstate snapshot — realistic boot-from-disk content
PREVSTATE_SAMPLE = {
    "PREV_STATE": 0,
    "PREV_ALT_CAL": 125,
    "PREV_MAX_ALT": 0,
    "PREV_TARGET_LAT": 37,
    "PREV_TARGET_LON": 35,
    "PREV_PACKET_COUNT": 0,
    "PREV_ST_TIMEDELTA": 0.0,
    "PREV_YAW_OFFSET": 0.0,
    "PREV_MOTOR_ENABLED": 0,
    "PREV_SOLENOID_COUNT": 0,
    "PREV_SOLENOID_DONE": 0,
    "PREV_START_LAT": 1,
    "PREV_START_LON": 1,
    "PREV_START_LOCKED": 0,
    "PREV_BEARING": None,
}


def _reset_motorapp():
    motorapp.MOTORAPP_RUNSTATUS = True
    motorapp.MOTOR_ENABLED = True
    motorapp.STATE = 0
    motorapp._PREV_STATE = -1
    motorapp._ORIGIN_SAVED = False
    motorapp.MANUAL_STEER_MODE = config.MOTOR_MANUAL_NEUTRAL
    motorapp.RELEASE_ACTION_ENABLED = True
    motorapp.EGG_ACTION_ENABLED = True
    motorapp.PI = None
    control.reset()
    motorapp._CACHE = _Cache()
    motorapp._GUIDANCE_STATE = guidance.GuidanceState()
    motorapp._TARGET_LAT = None
    motorapp._TARGET_LON = None
    motorapp._START_LAT = None
    motorapp._START_LON = None


def _init_with_prevstate(payload):
    """Run motorapp.init() with prevstate primed from `payload`."""
    with mock.patch.object(prevstate, "init_prevstate"), \
         mock.patch.object(prevstate, "is_motor_enabled",
                            return_value=payload.get("PREV_MOTOR_ENABLED", 0) == 1), \
         mock.patch.object(prevstate, "get_target_gps",
                            return_value=(float(payload.get("PREV_TARGET_LAT", 0)),
                                           float(payload.get("PREV_TARGET_LON", 0)))), \
         mock.patch.object(prevstate, "get_start_point",
                            return_value=(float(payload.get("PREV_START_LAT", 0)),
                                           float(payload.get("PREV_START_LON", 0)))
                            if payload.get("PREV_START_LOCKED", 0) == 1 else None), \
         mock.patch.object(control, "init_control", return_value=None):
        motorapp.init()


def _gps_msg(lat, lon, course_deg=0.0, speed=8.0, ts=None) -> str:
    ts = time.monotonic() if ts is None else ts
    return f"{lat},{lon},{ts:.4f},{course_deg},{speed},{ts:.4f}"


def _imu_msg(gyrz_deg=0.0, yaw_deg=0.0, ts=None, health=1) -> str:
    ts = time.monotonic() if ts is None else ts
    return (f"0,0,{yaw_deg},0,0,-9.81,0,0,0,0,0,{gyrz_deg},"
            f"{ts:.4f},0,0,{health}")


def _baro_msg(alt=150.0, ts=None) -> str:
    ts = time.monotonic() if ts is None else ts
    return f"{alt},{ts:.4f},nan"


def _capture_diag():
    """Return (queue_sentinel, list_to_receive_payloads)."""
    captured = []
    queue = object()
    def _grab(_queue, _src, _dst, _mid, payload):
        captured.append(payload)
    return queue, captured, _grab


# ════════════════════════════════════════════════════════════════════════════
# init() behaviour with the user-supplied prevstate
# ════════════════════════════════════════════════════════════════════════════

class TestInitWithUserPrevstate(unittest.TestCase):
    """Verify init() correctly interprets the supplied prevstate JSON."""

    def setUp(self):
        _reset_motorapp()
        _init_with_prevstate(PREVSTATE_SAMPLE)

    def test_motor_starts_disabled(self):
        self.assertFalse(motorapp.MOTOR_ENABLED)

    def test_target_loaded_as_float(self):
        # (37, 35) integers → become floats inside GuidanceState
        self.assertAlmostEqual(motorapp._GUIDANCE_STATE.target_lat, 37.0)
        self.assertAlmostEqual(motorapp._GUIDANCE_STATE.target_lon, 35.0)
        self.assertAlmostEqual(motorapp._CACHE.target_lat, 37.0)

    def test_start_point_not_locked_so_origin_not_restored(self):
        # PREV_START_LOCKED = 0 → get_start_point returns None
        self.assertFalse(motorapp._GUIDANCE_STATE.origin_ready)
        self.assertIsNone(motorapp._CACHE.start_lat)
        self.assertFalse(motorapp._ORIGIN_SAVED)

    def test_state_starts_at_zero(self):
        self.assertEqual(motorapp.STATE, 0)

    def test_controller_initialized(self):
        self.assertIsNotNone(motorapp._CONTROLLER)


class TestInitWithLockedStartPoint(unittest.TestCase):
    """Variant where PREV_START_LOCKED=1: origin should be restored."""

    def test_locked_start_point_restores_origin(self):
        payload = dict(PREVSTATE_SAMPLE)
        payload["PREV_START_LAT"] = 37.55
        payload["PREV_START_LON"] = 126.95
        payload["PREV_START_LOCKED"] = 1
        _reset_motorapp()
        _init_with_prevstate(payload)
        self.assertTrue(motorapp._GUIDANCE_STATE.origin_ready)
        self.assertAlmostEqual(motorapp._GUIDANCE_STATE.origin_lat, 37.55)
        self.assertTrue(motorapp._ORIGIN_SAVED)


class TestInitWithZeroTarget(unittest.TestCase):
    """Variant where target is the (0,0) sentinel: should NOT be loaded."""

    def test_zero_target_not_loaded(self):
        payload = dict(PREVSTATE_SAMPLE)
        payload["PREV_TARGET_LAT"] = 0
        payload["PREV_TARGET_LON"] = 0
        _reset_motorapp()
        _init_with_prevstate(payload)
        # init() rejects (0,0): cache and state untouched
        self.assertIsNone(motorapp._CACHE.target_lat)


# ════════════════════════════════════════════════════════════════════════════
# Simulated boot → flight → guidance sequence
# ════════════════════════════════════════════════════════════════════════════

class TestBootToFlightSequence(unittest.TestCase):
    """Full boot from prevstate, then operator commands + sensor flow."""

    def setUp(self):
        _reset_motorapp()
        _init_with_prevstate(PREVSTATE_SAMPLE)

    def test_disabled_motor_first_cycle_zero_pwm_only(self):
        """Motor disabled (per prevstate): cycle should not produce a command."""
        now = time.monotonic()
        cmd = motorapp._ctrl_cycle(None, now)
        self.assertIsNone(cmd)   # gate 1 short-circuited

    def test_operator_enable_then_state3_then_sensors_yields_l1_command(self):
        # 1. Operator sends MEC ON
        motorapp.handle_mec("ON")
        self.assertTrue(motorapp.MOTOR_ENABLED)

        # 2. Realistic target for Korea drop zone
        motorapp.handle_target_coord("37.560000,126.950000")
        # also push the start landmark (origin will lock from first GPS fix)
        ts0 = time.monotonic()
        motorapp.handle_gps(_gps_msg(37.551, 126.950, ts=ts0))
        motorapp.handle_imu(_imu_msg(gyrz_deg=2.0, yaw_deg=0.0, ts=ts0))
        motorapp.handle_barometer(_baro_msg(ts=ts0))

        # 3. Flightlogic enters state 3 (release armed)
        motorapp.handle_flight_state("3")
        self.assertEqual(motorapp.STATE, 3)

        # 4. One ctrl cycle
        queue, captured, grab = _capture_diag()
        with mock.patch.object(msgstructure, "send_msg", side_effect=grab):
            cmd = motorapp._ctrl_cycle(queue, ts0 + 0.05)
        self.assertIsNotNone(cmd)
        self.assertTrue(cmd.valid)
        # origin must now be locked and saved
        self.assertTrue(motorapp._GUIDANCE_STATE.origin_ready)
        self.assertTrue(motorapp._ORIGIN_SAVED)
        # diag emitted
        self.assertEqual(len(captured), 1)
        fields = captured[0].split(",")
        self.assertEqual(len(fields), 29)
        self.assertEqual(fields[10], "1")   # MOTOR_ENABLED column

    def test_state_5_landed_no_command(self):
        motorapp.handle_mec("ON")
        motorapp.handle_flight_state("5")
        cmd = motorapp._ctrl_cycle(None, time.monotonic())
        self.assertIsNone(cmd)

    def test_manual_steer_takes_priority_over_guidance(self):
        motorapp.handle_mec("ON")
        motorapp.handle_target_coord("37.560000,126.950000")
        ts0 = time.monotonic()
        motorapp.handle_gps(_gps_msg(37.551, 126.950, ts=ts0))
        motorapp.handle_imu(_imu_msg(gyrz_deg=2.0, ts=ts0))
        motorapp.handle_barometer(_baro_msg(ts=ts0))
        motorapp.handle_flight_state("3")
        motorapp.handle_mtr("RIGHT")

        queue, captured, grab = _capture_diag()
        with mock.patch.object(msgstructure, "send_msg", side_effect=grab):
            cmd = motorapp._ctrl_cycle(queue, ts0 + 0.05)
        self.assertIsNotNone(cmd)
        self.assertGreater(cmd.delta_arm_deg, 0.0)   # RIGHT = positive
        self.assertEqual(cmd.mode, "MANUAL_RIGHT")


# ════════════════════════════════════════════════════════════════════════════
# Multi-cycle simulation: GPS dropout → DR → recover
# ════════════════════════════════════════════════════════════════════════════

class TestMultiCycleSimulation(unittest.TestCase):
    """Drive several _ctrl_cycle iterations to exercise mode transitions."""

    def setUp(self):
        _reset_motorapp()
        # Start with a usable prevstate (motor enabled, target set)
        payload = dict(PREVSTATE_SAMPLE)
        payload["PREV_MOTOR_ENABLED"] = 1
        payload["PREV_TARGET_LAT"] = 37.560
        payload["PREV_TARGET_LON"] = 126.950
        _init_with_prevstate(payload)
        motorapp.handle_flight_state("3")

    def _push_fresh(self, ts, lat=37.551, lon=126.950):
        motorapp.handle_gps(_gps_msg(lat, lon, ts=ts))
        motorapp.handle_imu(_imu_msg(gyrz_deg=2.0, ts=ts))
        motorapp.handle_barometer(_baro_msg(ts=ts))

    def test_gps_tracking_then_dropout_then_recover(self):
        # ── Cycle 0–2: GPS tracking ─────────────────────────────────────────
        t = time.monotonic()
        modes_seen = []
        for k in range(3):
            self._push_fresh(t)
            motorapp._ctrl_cycle(None, t + 0.001)
            modes_seen.append(motorapp._GUIDANCE_STATE.nav_control_mode)
            t += 0.05
        self.assertEqual(modes_seen[-1], guidance.ControlMode.GPS_TRACKING_CLOSED)

        # ── Cycle 3–4: GPS dropout (only IMU updates) ───────────────────────
        for k in range(2):
            t += config.GPS_FRESH_MAX_AGE_S + 1.0
            motorapp.handle_imu(_imu_msg(gyrz_deg=0.5, ts=t))
            motorapp._ctrl_cycle(None, t + 0.001)
        self.assertIn(motorapp._GUIDANCE_STATE.nav_control_mode,
                      (guidance.ControlMode.DR_TRACKING_CLOSED,
                       guidance.ControlMode.DR_TRACKING_OPEN,
                       guidance.ControlMode.FAIL))

        # ── Cycle 5: GPS recovers ───────────────────────────────────────────
        t += 0.05
        self._push_fresh(t)
        motorapp._ctrl_cycle(None, t + 0.001)
        self.assertEqual(motorapp._GUIDANCE_STATE.nav_control_mode,
                          guidance.ControlMode.GPS_TRACKING_CLOSED)

    def test_state_demotion_resets_origin_and_controller(self):
        """Going state>=3 → <3 must reset everything."""
        ts0 = time.monotonic()
        self._push_fresh(ts0)
        motorapp._ctrl_cycle(None, ts0 + 0.001)
        self.assertTrue(motorapp._GUIDANCE_STATE.origin_ready)
        self.assertTrue(motorapp._ORIGIN_SAVED)

        # Demote to pre-deploy
        with mock.patch.object(prevstate, "clear_start_point") as m:
            motorapp.handle_flight_state("2")
            m.assert_called_once()
        self.assertFalse(motorapp._GUIDANCE_STATE.origin_ready)
        self.assertFalse(motorapp._ORIGIN_SAVED)
        self.assertIsNone(motorapp._CACHE.start_lat)
        # Histories cleared
        self.assertEqual(motorapp._GUIDANCE_STATE.gps_history, [])
        self.assertEqual(motorapp._GUIDANCE_STATE.imu_history, [])


# ════════════════════════════════════════════════════════════════════════════
# Real prevstate JSON round-trip (file-based)
# ════════════════════════════════════════════════════════════════════════════

class TestPrevstateJsonOnDisk(unittest.TestCase):
    """Write the supplied JSON to a temp file, call prevstate.init_prevstate,
    and verify the loaded values match."""

    def test_file_roundtrip_with_sample_payload(self):
        import tempfile, os, pathlib
        with tempfile.TemporaryDirectory() as tmpdir:
            target = pathlib.Path(tmpdir) / "prevstate.json"
            target.write_text(json.dumps(PREVSTATE_SAMPLE, indent=2))
            with mock.patch.object(prevstate, "_STATE_FILE", target):
                prevstate.init_prevstate()
                self.assertEqual(prevstate.PREV_STATE, 0)
                self.assertEqual(prevstate.PREV_TARGET_LAT, 37.0)
                self.assertEqual(prevstate.PREV_TARGET_LON, 35.0)
                self.assertEqual(prevstate.PREV_MOTOR_ENABLED, 0)
                self.assertEqual(prevstate.PREV_START_LOCKED, 0)
                # PREV_BEARING null → NaN
                self.assertTrue(math.isnan(prevstate.PREV_BEARING))
                # get_start_point honors lock flag
                self.assertIsNone(prevstate.get_start_point())
                # get_target_gps returns the raw values
                lat, lon = prevstate.get_target_gps()
                self.assertEqual((lat, lon), (37.0, 35.0))


if __name__ == "__main__":
    unittest.main()
