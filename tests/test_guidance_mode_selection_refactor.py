import math
import unittest

from Sensor_Motor import guidance
from Sensor_Motor.sensor_types import _BaroFromApp, _GpsFromApp, _ImuFromApp


NOW = 100.0


def _prepare_mission():
    guidance.reset()
    origin_lat = 37.0
    origin_lon = 127.0
    target_lon = origin_lon + math.degrees(
        100.0 / (guidance.EARTH_RADIUS_M * math.cos(math.radians(origin_lat)))
    )
    guidance.set_origin_point(origin_lat, origin_lon)
    guidance.set_target_point(origin_lat, target_lon)


def _gps(pos=True, motion=True, age=0.0):
    ts = NOW - age
    return _GpsFromApp(
        lat=37.00001,
        lon=127.00001,
        course_rad=math.radians(90.0),
        speed_mps=5.0,
        pos_ts=ts,
        motion_ts=ts,
        pos_health=1 if pos else 0,
        motion_health=1 if motion else 0,
    )


def _imu(gyrz=False, yaw=False, acc=False, age=0.0):
    return _ImuFromApp(
        yaw_rad=math.radians(90.0) if yaw else None,
        gyrz_rad_s=math.radians(3.0) if gyrz else None,
        ts=NOW - age,
        lin_acc_x=0.2 if acc else None,
        lin_acc_y=0.0 if acc else None,
        lin_acc_valid=acc,
        health=1 if (gyrz or yaw or acc) else 0,
    )


def _baro(sink=False, age=0.0):
    return _BaroFromApp(
        alt_m=120.0,
        sink_rate=3.0 if sink else None,
        rx_ts=NOW - age,
        health=1 if sink else 0,
    )


def _seed_dr():
    guidance.dr_lock(
        guidance._STATE_t.dr,
        E=10.0,
        N=20.0,
        V=4.0,
        course=math.radians(80.0),
        yaw=math.radians(80.0),
        now=NOW - 0.1,
    )


class TestGuidanceModeSelectionRefactor(unittest.TestCase):
    def setUp(self):
        _prepare_mission()

    def _decide(self, gps=None, imu=None, baro=None):
        return guidance.DecideControlMode(gps, imu, baro, NOW)

    def test_case_01_gps_position_motion_gyrz_closed(self):
        self.assertEqual(
            self._decide(_gps(pos=True, motion=True), _imu(gyrz=True), _baro()),
            guidance.ControlMode.GPS_TRACKING_CLOSED,
        )

    def test_case_02_gps_position_motion_no_gyrz_open(self):
        self.assertEqual(
            self._decide(_gps(pos=True, motion=True), _imu(yaw=True), _baro()),
            guidance.ControlMode.GPS_TRACKING_OPEN,
        )

    def test_case_03_pos_only_gba_estimates_motion_closed(self):
        _seed_dr()
        self.assertEqual(
            self._decide(_gps(pos=True, motion=False), _imu(gyrz=True, acc=True), _baro(sink=True)),
            guidance.ControlMode.DR_M_GBA_CLOSED,
        )

    def test_case_04_pos_only_gb_estimates_motion_closed(self):
        _seed_dr()
        self.assertEqual(
            self._decide(_gps(pos=True, motion=False), _imu(gyrz=True), _baro(sink=True)),
            guidance.ControlMode.DR_M_GB_CLOSED,
        )

    def test_case_05_pos_only_g_estimates_motion_closed_from_last_v(self):
        _seed_dr()
        self.assertEqual(
            self._decide(_gps(pos=True, motion=False), _imu(gyrz=True), _baro()),
            guidance.ControlMode.DR_M_G_CLOSED,
        )

    def test_case_06_pos_only_yba_estimates_motion_open(self):
        self.assertEqual(
            self._decide(_gps(pos=True, motion=False), _imu(yaw=True, acc=True), _baro(sink=True)),
            guidance.ControlMode.DR_M_YBA_OPEN,
        )

    def test_case_07_pos_only_yb_estimates_motion_open(self):
        self.assertEqual(
            self._decide(_gps(pos=True, motion=False), _imu(yaw=True), _baro(sink=True)),
            guidance.ControlMode.DR_M_YB_OPEN,
        )

    def test_case_08_pos_only_y_estimates_motion_open_from_min_speed(self):
        self.assertEqual(
            self._decide(_gps(pos=True, motion=False), _imu(yaw=True), _baro()),
            guidance.ControlMode.DR_M_Y_OPEN,
        )

    def test_case_09_no_gps_gba_estimates_position_motion_closed(self):
        _seed_dr()
        self.assertEqual(
            self._decide(None, _imu(gyrz=True, acc=True), _baro(sink=True)),
            guidance.ControlMode.DR_PM_GBA_CLOSED,
        )

    def test_case_10_no_gps_gb_estimates_position_motion_closed(self):
        _seed_dr()
        self.assertEqual(
            self._decide(None, _imu(gyrz=True), _baro(sink=True)),
            guidance.ControlMode.DR_PM_GB_CLOSED,
        )

    def test_case_11_no_gps_g_estimates_position_motion_closed_from_last_v(self):
        _seed_dr()
        self.assertEqual(
            self._decide(None, _imu(gyrz=True), _baro()),
            guidance.ControlMode.DR_PM_G_CLOSED,
        )

    def test_case_12_no_gps_yba_estimates_position_motion_open(self):
        _seed_dr()
        self.assertEqual(
            self._decide(None, _imu(yaw=True, acc=True), _baro(sink=True)),
            guidance.ControlMode.DR_PM_YBA_OPEN,
        )

    def test_case_13_no_gps_yb_estimates_position_motion_open(self):
        _seed_dr()
        self.assertEqual(
            self._decide(None, _imu(yaw=True), _baro(sink=True)),
            guidance.ControlMode.DR_PM_YB_OPEN,
        )

    def test_case_14_no_gps_y_estimates_position_motion_open_from_last_v(self):
        _seed_dr()
        self.assertEqual(
            self._decide(None, _imu(yaw=True), _baro()),
            guidance.ControlMode.DR_PM_Y_OPEN,
        )

    def test_case_15_missing_origin_or_target_fails(self):
        guidance.reset()
        self.assertEqual(
            self._decide(_gps(pos=True, motion=True), _imu(gyrz=True), _baro()),
            guidance.ControlMode.FAIL,
        )

    def test_case_16_gyrz_without_course_seed_cannot_start_pos_only_dr(self):
        self.assertEqual(
            self._decide(_gps(pos=True, motion=False), _imu(gyrz=True), _baro(sink=True)),
            guidance.ControlMode.FAIL,
        )

    def test_case_17_no_gyrz_and_no_yaw_fails_even_with_dr_anchor(self):
        _seed_dr()
        self.assertEqual(
            self._decide(None, _imu(acc=True), _baro(sink=True)),
            guidance.ControlMode.FAIL,
        )

    def test_case_18_stale_raw_sensors_are_not_fresh(self):
        _seed_dr()
        self.assertEqual(
            self._decide(_gps(pos=True, motion=True, age=10.0), _imu(gyrz=True, age=10.0), _baro(sink=True, age=10.0)),
            guidance.ControlMode.FAIL,
        )


if __name__ == "__main__":
    unittest.main()
