"""Unit tests for motorapp: handler parsing, cache updates, state transitions, snapshot."""

import math
import time
import unittest

from Sensor_Motor import motorapp
from Sensor_Motor.motorapp import _Cache


def _reset():
    motorapp.MOTORAPP_RUNSTATUS = True
    motorapp.MOTOR_ENABLED = True
    motorapp.STATE = 0
    motorapp._PREV_STATE = -1
    motorapp._START_POINT_LOCKED = False
    motorapp._CONTROLLER = None
    motorapp._L1_STATE = None
    motorapp.PI = None
    motorapp._CACHE = _Cache()


def _gps_msg(lat=37.55, lon=126.95, course=90.0, speed=12.0, pos=True, motion=True, ts=None):
    ts = time.monotonic() if ts is None else ts
    lat_s = f"{lat}" if pos else "nan"
    lon_s = f"{lon}" if pos else "nan"
    course_s = f"{course}" if motion else "nan"
    speed_s = f"{speed}" if motion else "nan"
    motion_ts = f"{ts:.4f}" if motion else "nan"
    return f"{lat_s},{lon_s},{ts:.4f},{course_s},{speed_s},{motion_ts}"


def _imu_msg(
    roll=1.0,
    pitch=2.0,
    yaw=45.0,
    accx=0.1,
    accy=0.2,
    accz=0.3,
    magx=0.4,
    magy=0.5,
    magz=0.6,
    gyrx=0.0,
    gyry=0.0,
    gyrz=2.5,
    health=1,
    ts=None,
):
    ts = time.monotonic() if ts is None else ts
    return (
        f"{roll},{pitch},{yaw},{accx},{accy},{accz},"
        f"{magx},{magy},{magz},{gyrx},{gyry},{gyrz},{ts:.4f},0,0,{health}"
    )


def _baro_msg(alt=200.5, health=1, sink=1.2, ts=None):
    ts = time.monotonic() if ts is None else ts
    return f"{alt},{ts:.4f},{sink},{health}"


class TestHandleMec(unittest.TestCase):
    def setUp(self):
        _reset()

    def test_mec_off_disables_motor(self):
        motorapp.handle_mec("OFF")
        self.assertFalse(motorapp.MOTOR_ENABLED)

    def test_mec_on_enables_motor(self):
        motorapp.handle_mec("OFF")
        motorapp.handle_mec("ON")
        self.assertTrue(motorapp.MOTOR_ENABLED)

    def test_mec_unknown_no_crash(self):
        motorapp.handle_mec("MAYBE")


class TestHandleGps(unittest.TestCase):
    def setUp(self):
        _reset()

    def test_valid_6_fields_updates_latest_gps(self):
        motorapp.handle_gps(_gps_msg())
        gps = motorapp._CACHE.latest_gps
        self.assertAlmostEqual(gps.lat, 37.55)
        self.assertAlmostEqual(gps.lon, 126.95)
        self.assertAlmostEqual(gps.course_rad, math.radians(90.0))
        self.assertAlmostEqual(gps.speed_mps, 12.0)
        self.assertTrue(gps.pos_health)
        self.assertTrue(gps.motion_health)

    def test_valid_payload_uses_position_ts(self):
        motorapp.handle_gps(_gps_msg(ts=100.5))
        gps = motorapp._CACHE.latest_gps
        self.assertAlmostEqual(gps.pos_ts, 100.5, places=3)

    def test_too_few_fields_no_update(self):
        motorapp.handle_gps("37.55,126.95")
        self.assertIsNone(motorapp._CACHE.latest_gps.lat)

    def test_bad_float_no_update(self):
        motorapp.handle_gps("not_a_float,126.95,90.0,90.0,12.0,90.0")
        self.assertIsNone(motorapp._CACHE.latest_gps.lat)

    def test_fresh_latest_moves_to_history_on_next_gps(self):
        t0 = time.monotonic() - 0.1
        motorapp.handle_gps(_gps_msg(ts=t0))
        self.assertEqual(len(motorapp._CACHE.gps_history), 0)
        motorapp.handle_gps(_gps_msg(lat=37.55001, lon=126.95001, course=91.0, speed=12.1, ts=t0 + 0.1))
        self.assertEqual(len(motorapp._CACHE.gps_history), 1)
        self.assertAlmostEqual(motorapp._CACHE.gps_history[-1].lat, 37.55)
        self.assertTrue(motorapp._CACHE.gps_history[-1].pos_health)

    def test_invalid_position_does_not_enter_history(self):
        t0 = time.monotonic() - 0.1
        motorapp.handle_gps(_gps_msg(lat=37.55, lon=50.0, ts=t0))
        motorapp.handle_gps(_gps_msg(ts=t0 + 0.1))
        self.assertEqual(len(motorapp._CACHE.gps_history), 0)

    def test_unexpected_longitude_disables_position_and_motion(self):
        motorapp.handle_gps(_gps_msg(lat=37.55, lon=50.0))
        gps = motorapp._CACHE.latest_gps
        self.assertFalse(gps.pos_health)
        self.assertFalse(gps.motion_health)

    def test_invalid_motion_stays_out_of_latest_motion_fields(self):
        motorapp.handle_gps(_gps_msg(course=999.0))
        self.assertTrue(motorapp._CACHE.latest_gps.pos_health)
        self.assertFalse(motorapp._CACHE.latest_gps.motion_health)
        self.assertIsNone(motorapp._CACHE.latest_gps.course_rad)

    def test_motion_health_requires_position_health(self):
        motorapp.handle_gps(_gps_msg(lat=37.55, lon=50.0))
        self.assertFalse(motorapp._CACHE.latest_gps.pos_health)
        self.assertFalse(motorapp._CACHE.latest_gps.motion_health)

    def test_history_preserves_freshed_motion(self):
        t0 = time.monotonic() - 0.1
        motorapp.handle_gps(_gps_msg(ts=t0))
        motorapp.handle_gps(_gps_msg(lat=37.55001, lon=126.95001, course=91.0, speed=12.1, ts=t0 + 0.1))
        freshed = motorapp._freshed_gps_from_history(motorapp._CACHE.gps_history)
        self.assertAlmostEqual(freshed.course_rad, math.radians(90.0))
        self.assertAlmostEqual(freshed.speed_mps, 12.0)

    def test_first_gps_stays_latest_not_history(self):
        motorapp.handle_gps(_gps_msg())
        self.assertEqual(len(motorapp._CACHE.gps_history), 0)


class TestHandleGpsStartPointLocking(unittest.TestCase):
    """Tests for the start-point locking path in handle_gps (STATE >= 3)."""

    def setUp(self):
        _reset()

    def test_state_less_than_3_does_not_lock(self):
        motorapp.STATE = 2
        motorapp.handle_gps(_gps_msg())
        self.assertFalse(motorapp._START_POINT_LOCKED)
        self.assertIsNone(motorapp._CACHE.start_lat)

    def test_state3_healthy_gps_locks_start_point(self):
        motorapp.STATE = 3
        motorapp.handle_gps(_gps_msg())
        self.assertTrue(motorapp._START_POINT_LOCKED)
        self.assertAlmostEqual(motorapp._CACHE.start_lat, 37.55)
        self.assertAlmostEqual(motorapp._CACHE.start_lon, 126.95)

    def test_state3_unexpected_longitude_does_not_lock_start_point(self):
        motorapp.STATE = 3
        motorapp.handle_gps(_gps_msg(lat=37.55, lon=50.0))
        self.assertFalse(motorapp._START_POINT_LOCKED)
        self.assertIsNone(motorapp._CACHE.start_lon)


class TestHandleImu(unittest.TestCase):
    def setUp(self):
        _reset()

    def test_valid_payload(self):
        motorapp.handle_imu(_imu_msg())
        imu = motorapp._CACHE.latest_imu
        self.assertAlmostEqual(imu.gyrz_rad_s, math.radians(2.5))
        self.assertTrue(imu.health)

    def test_valid_payload_preserves_all_imu_fields(self):
        motorapp.handle_imu(
            _imu_msg(
                roll=10.0,
                pitch=-5.0,
                yaw=270.0,
                accx=1.1,
                accy=2.2,
                accz=3.3,
                magx=4.4,
                magy=5.5,
                magz=6.6,
                gyrx=7.7,
                gyry=8.8,
                gyrz=9.9,
            )
        )
        imu = motorapp._CACHE.latest_imu
        self.assertAlmostEqual(imu.roll_rad, math.radians(10.0))
        self.assertAlmostEqual(imu.pitch_rad, math.radians(-5.0))
        self.assertAlmostEqual(imu.yaw_rad, math.radians(270.0))
        self.assertAlmostEqual(imu.accx_mps2, 1.1)
        self.assertAlmostEqual(imu.accy_mps2, 2.2)
        self.assertAlmostEqual(imu.accz_mps2, 3.3)
        self.assertAlmostEqual(imu.magx_uT, 4.4)
        self.assertAlmostEqual(imu.magy_uT, 5.5)
        self.assertAlmostEqual(imu.magz_uT, 6.6)
        self.assertAlmostEqual(imu.gyrx_rad_s, math.radians(7.7))
        self.assertAlmostEqual(imu.gyry_rad_s, math.radians(8.8))
        self.assertAlmostEqual(imu.gyrz_rad_s, math.radians(9.9))

    def test_valid_payload_uses_sample_ts(self):
        motorapp.handle_imu(_imu_msg(ts=99.9))
        self.assertAlmostEqual(motorapp._CACHE.latest_imu.ts, 99.9, places=3)

    def test_bad_data_no_update(self):
        motorapp.handle_imu("bad")
        self.assertIsNone(motorapp._CACHE.latest_imu.gyrz_rad_s)

    def test_too_few_fields_no_update(self):
        motorapp.handle_imu("1.0,2.0,3.0")
        self.assertIsNone(motorapp._CACHE.latest_imu.gyrz_rad_s)

    def test_fresh_latest_moves_to_imu_history_on_next_imu(self):
        t0 = time.monotonic() - 0.1
        motorapp.handle_imu(_imu_msg(gyrz=2.5, ts=t0))
        self.assertEqual(len(motorapp._CACHE.imu_history), 0)
        motorapp.handle_imu(_imu_msg(gyrz=3.0, ts=t0 + 0.05))
        self.assertEqual(len(motorapp._CACHE.imu_history), 1)
        self.assertAlmostEqual(motorapp._CACHE.imu_history[-1].gyrz_rad_s, math.radians(2.5))
        self.assertAlmostEqual(motorapp._CACHE.imu_history[-1].yaw_rad, math.radians(45.0))

    def test_unhealthy_imu_skips_history(self):
        t0 = time.monotonic() - 0.1
        motorapp.handle_imu(_imu_msg(gyrz=2.5, health=0, ts=t0))
        motorapp.handle_imu(_imu_msg(gyrz=3.0, ts=t0 + 0.05))
        self.assertEqual(len(motorapp._CACHE.imu_history), 0)


class TestHandleBarometer(unittest.TestCase):
    def setUp(self):
        _reset()

    def test_valid_payload(self):
        motorapp.handle_barometer(_baro_msg(alt=150.0))
        self.assertAlmostEqual(motorapp._CACHE.latest_baro.alt_m, 150.0)
        self.assertTrue(motorapp._CACHE.latest_baro.health)

    def test_alt_and_health(self):
        motorapp.handle_barometer(_baro_msg(alt=200.5, health=1))
        self.assertAlmostEqual(motorapp._CACHE.latest_baro.alt_m, 200.5)
        self.assertTrue(motorapp._CACHE.latest_baro.health)

    def test_alt_health_and_ts(self):
        motorapp.handle_barometer(_baro_msg(alt=200.5, health=1, ts=77.7))
        self.assertAlmostEqual(motorapp._CACHE.latest_baro.alt_m, 200.5)
        self.assertAlmostEqual(motorapp._CACHE.latest_baro.ts, 77.7, places=3)

    def test_bad_data_no_update(self):
        motorapp.handle_barometer("NOPE")
        self.assertIsNone(motorapp._CACHE.latest_baro.alt_m)

    def test_fresh_latest_moves_to_baro_history_on_next_baro(self):
        t0 = time.monotonic() - 0.1
        motorapp.handle_barometer(_baro_msg(alt=200.5, health=1, ts=t0))
        self.assertEqual(len(motorapp._CACHE.baro_history), 0)
        motorapp.handle_barometer(_baro_msg(alt=201.0, health=1, ts=t0 + 0.1))
        self.assertEqual(len(motorapp._CACHE.baro_history), 1)
        self.assertAlmostEqual(motorapp._CACHE.baro_history[-1].alt_m, 200.5)

    def test_unhealthy_baro_skips_history(self):
        t0 = time.monotonic() - 0.1
        motorapp.handle_barometer(_baro_msg(alt=200.5, health=0, ts=t0))
        motorapp.handle_barometer(_baro_msg(alt=201.0, health=1, ts=t0 + 0.1))
        self.assertEqual(len(motorapp._CACHE.baro_history), 0)


class TestFreshedEstimates(unittest.TestCase):
    def setUp(self):
        _reset()

    def test_gps_history_regression_propagates_position_and_motion(self):
        t0 = time.monotonic() - 2.0
        origin_lat = 37.55
        origin_lon = 126.95
        lat_10m_north, lon_same = motorapp._ne_to_latlon(10.0, 0.0, origin_lat, origin_lon)
        history = [
            motorapp._GpsFromApp(
                lat=origin_lat,
                lon=origin_lon,
                pos_ts=t0,
                pos_health=1,
            ),
            motorapp._GpsFromApp(
                lat=lat_10m_north,
                lon=lon_same,
                pos_ts=t0 + 1.0,
                pos_health=1,
            ),
        ]

        freshed = motorapp._freshed_gps_from_history(
            history,
            now=t0 + 2.0,
            origin_lat=origin_lat,
            origin_lon=origin_lon,
        )
        est_n, est_e = motorapp._project_latlon_to_ne(
            freshed.lat,
            freshed.lon,
            origin_lat,
            origin_lon,
        )
        self.assertAlmostEqual(est_n, 20.0, delta=0.5)
        self.assertAlmostEqual(est_e, 0.0, delta=0.5)
        self.assertAlmostEqual(freshed.speed_mps, 10.0, delta=0.2)
        self.assertAlmostEqual(freshed.course_rad, 0.0, delta=0.05)
        self.assertTrue(freshed.pos_health)
        self.assertTrue(freshed.motion_health)

    def test_gps_history_short_hold_when_regression_unavailable(self):
        t0 = time.monotonic() - 0.5
        history = [
            motorapp._GpsFromApp(
                lat=37.55,
                lon=126.95,
                course_rad=math.radians(90.0),
                speed_mps=8.0,
                pos_ts=t0,
                motion_ts=t0,
                pos_health=1,
                motion_health=1,
            )
        ]

        freshed = motorapp._freshed_gps_from_history(history, now=t0 + 0.5)
        self.assertAlmostEqual(freshed.lat, 37.55)
        self.assertAlmostEqual(freshed.course_rad, math.radians(90.0))
        self.assertAlmostEqual(freshed.speed_mps, 8.0)

    def test_gps_history_dead_reckons_from_single_anchor_and_motion(self):
        t0 = time.monotonic() - 1.0
        origin_lat = 37.55
        origin_lon = 126.95
        history = [
            motorapp._GpsFromApp(
                lat=origin_lat,
                lon=origin_lon,
                course_rad=math.radians(90.0),
                speed_mps=8.0,
                pos_ts=t0,
                motion_ts=t0,
                pos_health=1,
                motion_health=1,
            )
        ]

        freshed = motorapp._freshed_gps_from_history(
            history,
            now=t0 + 1.0,
            origin_lat=origin_lat,
            origin_lon=origin_lon,
        )
        est_n, est_e = motorapp._project_latlon_to_ne(
            freshed.lat,
            freshed.lon,
            origin_lat,
            origin_lon,
        )
        self.assertAlmostEqual(est_n, 0.0, delta=0.5)
        self.assertAlmostEqual(est_e, 8.0, delta=0.5)
        self.assertAlmostEqual(freshed.course_rad, math.radians(90.0))

    def test_imu_history_weighted_average_within_feedback_window(self):
        t0 = time.monotonic() - 0.4
        history = [
            motorapp._ImuFromApp(gyrz_rad_s=math.radians(2.0), ts=t0 + 0.1, health=1),
            motorapp._ImuFromApp(gyrz_rad_s=math.radians(4.0), ts=t0 + 0.3, health=1),
        ]

        freshed = motorapp._freshed_imu_from_history(history, now=t0 + 0.4)
        self.assertAlmostEqual(freshed.gyrz_rad_s, math.radians(3.333333), places=5)
        self.assertTrue(freshed.health)

    def test_imu_history_stale_after_feedback_window(self):
        t0 = time.monotonic() - 1.0
        history = [motorapp._ImuFromApp(gyrz_rad_s=math.radians(2.0), ts=t0, health=1)]

        freshed = motorapp._freshed_imu_from_history(history, now=t0 + 1.0)
        self.assertIsNone(freshed.gyrz_rad_s)
        self.assertFalse(freshed.health)

    def test_baro_history_regression_propagates_altitude(self):
        t0 = time.monotonic() - 3.0
        history = [
            motorapp._BaroFromApp(alt_m=200.0, ts=t0, health=1),
            motorapp._BaroFromApp(alt_m=190.0, ts=t0 + 2.0, health=1),
        ]

        freshed = motorapp._freshed_baro_from_history(history, now=t0 + 3.0)
        self.assertAlmostEqual(freshed.sink_rate, 5.0)
        self.assertAlmostEqual(freshed.alt_m, 185.0)
        self.assertTrue(freshed.health)


class TestHandleTargetCoord(unittest.TestCase):
    def setUp(self):
        _reset()

    def test_valid_coord_updates_cache(self):
        motorapp.handle_target_coord("37.55,126.95")
        self.assertAlmostEqual(motorapp._CACHE.target_lat, 37.55)
        self.assertAlmostEqual(motorapp._CACHE.target_lon, 126.95)

    def test_out_of_range_lat_no_update(self):
        motorapp.handle_target_coord("999.0,126.95")
        self.assertIsNone(motorapp._CACHE.target_lat)

    def test_out_of_range_lon_no_update(self):
        motorapp.handle_target_coord("37.55,999.0")
        self.assertIsNone(motorapp._CACHE.target_lat)

    def test_bad_float_no_update(self):
        motorapp.handle_target_coord("not,valid")
        self.assertIsNone(motorapp._CACHE.target_lat)

    def test_wrong_field_count_no_update(self):
        motorapp.handle_target_coord("37.55")
        self.assertIsNone(motorapp._CACHE.target_lat)


class TestHandleFlightState(unittest.TestCase):
    def setUp(self):
        _reset()

    def test_state_transition(self):
        motorapp.handle_flight_state("3")
        self.assertEqual(motorapp.STATE, 3)

    def test_same_state_no_prev_update(self):
        motorapp.STATE = 2
        motorapp._PREV_STATE = 1
        motorapp.handle_flight_state("2")
        self.assertEqual(motorapp._PREV_STATE, 1)

    def test_bad_value_no_change(self):
        motorapp.STATE = 2
        motorapp.handle_flight_state("abc")
        self.assertEqual(motorapp.STATE, 2)

    def test_prev_state_recorded(self):
        motorapp.STATE = 2
        motorapp.handle_flight_state("4")
        self.assertEqual(motorapp._PREV_STATE, 2)

    def test_below_3_clears_start_point(self):
        motorapp._CACHE.start_lat = 37.55
        motorapp._CACHE.start_lon = 126.95
        motorapp._START_POINT_LOCKED = True
        motorapp.STATE = 3
        motorapp.handle_flight_state("2")
        self.assertIsNone(motorapp._CACHE.start_lat)
        self.assertFalse(motorapp._START_POINT_LOCKED)

    def test_state3_with_healthy_gps_locks_start_point(self):
        motorapp._CACHE.latest_gps.lat = 37.55
        motorapp._CACHE.latest_gps.lon = 126.95
        motorapp._CACHE.latest_gps.pos_ts = time.monotonic()
        motorapp._CACHE.latest_gps.pos_health = True
        motorapp.handle_flight_state("3")
        self.assertTrue(motorapp._START_POINT_LOCKED)
        self.assertAlmostEqual(motorapp._CACHE.start_lat, 37.55)
        self.assertAlmostEqual(motorapp._CACHE.start_lon, 126.95)

    def test_state3_without_healthy_gps_skips_start_lock(self):
        motorapp._CACHE.latest_gps.lat = None
        motorapp._CACHE.latest_gps.pos_health = False
        motorapp.handle_flight_state("3")
        self.assertFalse(motorapp._START_POINT_LOCKED)


class TestCacheSnapshot(unittest.TestCase):
    def setUp(self):
        _reset()

    def test_snapshot_copies_target(self):
        motorapp._CACHE.target_lat = 37.6
        motorapp._CACHE.target_lon = 127.0
        snap = motorapp._cache_snapshot()
        self.assertAlmostEqual(snap.target_lat, 37.6)
        self.assertAlmostEqual(snap.target_lon, 127.0)

    def test_snapshot_target_none_when_unset(self):
        snap = motorapp._cache_snapshot()
        self.assertIsNone(snap.target_lat)

    def test_snapshot_copies_start_point(self):
        motorapp._CACHE.start_lat = 37.55
        motorapp._CACHE.start_lon = 126.95
        snap = motorapp._cache_snapshot()
        self.assertAlmostEqual(snap.start_lat, 37.55)
        self.assertAlmostEqual(snap.start_lon, 126.95)

    def test_snapshot_copies_latest_gps(self):
        motorapp.handle_gps(_gps_msg())
        snap = motorapp._cache_snapshot()
        self.assertAlmostEqual(snap.latest_gps.lat, 37.55)
        self.assertAlmostEqual(snap.latest_gps.lon, 126.95)

    def test_snapshot_copies_latest_imu(self):
        motorapp.handle_imu(_imu_msg())
        snap = motorapp._cache_snapshot()
        self.assertAlmostEqual(snap.latest_imu.gyrz_rad_s, math.radians(2.5))

    def test_snapshot_copies_latest_baro(self):
        motorapp.handle_barometer(_baro_msg(alt=333.0))
        snap = motorapp._cache_snapshot()
        self.assertAlmostEqual(snap.latest_baro.alt_m, 333.0)

    def test_snapshot_is_independent_copy(self):
        motorapp._CACHE.target_lat = 10.0
        snap = motorapp._cache_snapshot()
        motorapp._CACHE.target_lat = 99.0
        self.assertAlmostEqual(snap.target_lat, 10.0)

    def test_snapshot_copies_history(self):
        t0 = time.monotonic() - 0.1
        motorapp.handle_gps(_gps_msg(ts=t0))
        motorapp.handle_gps(_gps_msg(lat=37.55001, lon=126.95001, ts=t0 + 0.1))
        snap = motorapp._cache_snapshot()
        self.assertEqual(len(snap.gps_history), 1)
        self.assertAlmostEqual(snap.gps_history[-1].lat, 37.55)


if __name__ == "__main__":
    unittest.main()
