"""IPC integration tests for mag-guidance bearing dispatch (GPS-free branch)."""

import math
import time
import unittest

from lib import appargs
from Sensor_Motor import motorapp
from Sensor_Motor.motorapp import _Cache


# ── reset / message helpers ───────────────────────────────────────────────────
def _reset():
    motorapp.MOTORAPP_RUNSTATUS = True
    motorapp.MOTOR_ENABLED = True
    motorapp.MANUAL_STEER_MODE = "NEUTRAL"
    motorapp.STATE = 0
    motorapp._PREV_STATE = -1
    motorapp._START_POINT_LOCKED = False
    motorapp._CONTROLLER = None
    motorapp._L1_STATE = None
    motorapp.PI = None
    motorapp._CACHE = _Cache()


def _dispatch(sender_id: int, mid: int, data: str) -> None:
    msg = f"{sender_id}|{appargs.MotorAppArg.AppID}|{mid}|{data}"
    motorapp.dispatch(msg)


def _imu_msg(yaw_deg=90.0, gyrz=0.0, health=1, ts=None):
    ts = time.monotonic() if ts is None else ts
    return (
        f"1.0,2.0,{yaw_deg},0.1,0.2,0.3,"
        f"0.4,0.5,0.6,0.0,0.0,{gyrz},"
        f"{ts:.4f},0,0,{health}"
    )


def _baro_msg(alt=100.0, health=1, sink=1.0, ts=None):
    ts = time.monotonic() if ts is None else ts
    return f"{alt},{ts:.4f},{sink},{health}"


# ── bearing dispatch tests ────────────────────────────────────────────────────
class TestBearingDispatch(unittest.TestCase):

    def setUp(self):
        _reset()

    # basic routing
    def test_bearing_message_updates_cache(self):
        _dispatch(appargs.FlightlogicAppArg.AppID,
                  appargs.FlightlogicAppArg.MID_motor_bearing, "127.5")
        self.assertAlmostEqual(
            math.degrees(motorapp._CACHE.target_bearing_rad), 127.5, places=4
        )

    def test_bearing_default_is_nan(self):
        self.assertTrue(math.isnan(motorapp._CACHE.target_bearing_rad))

    def test_bearing_overwrite_with_new_value(self):
        _dispatch(appargs.FlightlogicAppArg.AppID,
                  appargs.FlightlogicAppArg.MID_motor_bearing, "45.0")
        _dispatch(appargs.FlightlogicAppArg.AppID,
                  appargs.FlightlogicAppArg.MID_motor_bearing, "270.0")
        self.assertAlmostEqual(
            math.degrees(motorapp._CACHE.target_bearing_rad), 270.0, places=4
        )

    def test_bearing_zero_north_is_valid(self):
        _dispatch(appargs.FlightlogicAppArg.AppID,
                  appargs.FlightlogicAppArg.MID_motor_bearing, "0.0")
        self.assertAlmostEqual(motorapp._CACHE.target_bearing_rad, 0.0, places=6)

    def test_bearing_359_is_valid(self):
        _dispatch(appargs.FlightlogicAppArg.AppID,
                  appargs.FlightlogicAppArg.MID_motor_bearing, "359.9")
        self.assertAlmostEqual(
            math.degrees(motorapp._CACHE.target_bearing_rad), 359.9, places=3
        )

    def test_bearing_bad_string_no_crash_and_no_update(self):
        motorapp._CACHE.target_bearing_rad = math.radians(55.0)
        _dispatch(appargs.FlightlogicAppArg.AppID,
                  appargs.FlightlogicAppArg.MID_motor_bearing, "notanumber")
        # cache must stay at previous value
        self.assertAlmostEqual(
            math.degrees(motorapp._CACHE.target_bearing_rad), 55.0, places=4
        )

    def test_bearing_empty_string_no_crash(self):
        _dispatch(appargs.FlightlogicAppArg.AppID,
                  appargs.FlightlogicAppArg.MID_motor_bearing, "")
        # still NaN (default), no exception
        self.assertTrue(math.isnan(motorapp._CACHE.target_bearing_rad))

    # snapshot propagation
    def test_snapshot_includes_bearing(self):
        _dispatch(appargs.FlightlogicAppArg.AppID,
                  appargs.FlightlogicAppArg.MID_motor_bearing, "180.0")
        snap = motorapp._cache_snapshot()
        self.assertAlmostEqual(
            math.degrees(snap.target_bearing_rad), 180.0, places=4
        )

    def test_snapshot_bearing_independent_copy(self):
        _dispatch(appargs.FlightlogicAppArg.AppID,
                  appargs.FlightlogicAppArg.MID_motor_bearing, "90.0")
        snap = motorapp._cache_snapshot()
        motorapp._CACHE.target_bearing_rad = math.radians(0.0)
        # snapshot must not change
        self.assertAlmostEqual(math.degrees(snap.target_bearing_rad), 90.0, places=4)


# ── end-to-end: imu + baro + bearing populate guidance input ──────────────────
class TestMagGuidanceEndToEnd(unittest.TestCase):

    def setUp(self):
        _reset()
        _dispatch(appargs.ImuAppArg.AppID,
                  appargs.ImuAppArg.MID_motor_imu, _imu_msg(yaw_deg=45.0, gyrz=0.0))
        _dispatch(appargs.BarometerAppArg.AppID,
                  appargs.BarometerAppArg.MID_motor_alt, _baro_msg(alt=150.0))
        _dispatch(appargs.FlightlogicAppArg.AppID,
                  appargs.FlightlogicAppArg.MID_motor_bearing, "90.0")

    def test_imu_yaw_stored_correctly(self):
        self.assertAlmostEqual(
            math.degrees(motorapp._CACHE.latest_imu.yaw_rad), 45.0, places=4
        )

    def test_baro_alt_stored_correctly(self):
        self.assertAlmostEqual(motorapp._CACHE.latest_baro.alt_m, 150.0)

    def test_guidance_input_from_cache_snap_is_complete(self):
        from Sensor_Motor.mag_guidance import MagGuidanceInput, ProduceMagGuidance
        snap = motorapp._cache_snapshot()
        inp = MagGuidanceInput(
            yaw_rad=snap.latest_imu.yaw_rad,
            gyrz_rad_s=snap.latest_imu.gyrz_rad_s or 0.0,
            alt_m=snap.latest_baro.alt_m,
            target_bearing_rad=snap.target_bearing_rad,
            timestamp=time.monotonic(),
            imu_ts=snap.latest_imu.ts,
            baro_ts=snap.latest_baro.ts,
            imu_health=snap.latest_imu.health,
            baro_health=snap.latest_baro.health,
        )
        out = ProduceMagGuidance(inp)
        self.assertTrue(out.valid)
        # yaw=45°, bearing=90° → error=+45° → right turn (cmd > 0)
        self.assertGreater(out.angular_velocity_cmd_deg_s, 0.0)
        self.assertAlmostEqual(out.heading_error_deg, 45.0, places=3)

    def test_guidance_no_bearing_gives_invalid(self):
        from Sensor_Motor.mag_guidance import MagGuidanceInput, ProduceMagGuidance
        import math as _math
        snap = motorapp._cache_snapshot()
        inp = MagGuidanceInput(
            yaw_rad=snap.latest_imu.yaw_rad,
            target_bearing_rad=_math.nan,
            timestamp=time.monotonic(),
            imu_ts=snap.latest_imu.ts,
            imu_health=snap.latest_imu.health,
        )
        out = ProduceMagGuidance(inp)
        self.assertFalse(out.valid)
        self.assertEqual(out.mode, "NO_BEARING")


if __name__ == "__main__":
    unittest.main()
