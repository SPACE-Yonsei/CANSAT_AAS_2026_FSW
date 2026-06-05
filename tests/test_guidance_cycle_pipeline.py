import copy
import math
import unittest
from dataclasses import asdict
from unittest import mock

from Sensor_Motor import control, guidance
from Sensor_Motor.sensor_types import _BaroFromApp, _GpsFromApp, _ImuFromApp


T0 = 1000.0
T_DR_M = T0 + 5.2
T_DR_PM_1 = T0 + 10.4
T_DR_PM_2 = T0 + 10.6
ORIGIN_LAT = 37.0
ORIGIN_LON = 127.0
TARGET_E = 80.0
TARGET_N = 0.0
COURSE_EAST = math.radians(90.0)


def _latlon_from_ne(n_m, e_m):
    lat = ORIGIN_LAT + math.degrees(n_m / guidance.EARTH_RADIUS_M)
    lon = ORIGIN_LON + math.degrees(
        e_m / (guidance.EARTH_RADIUS_M * math.cos(math.radians(ORIGIN_LAT)))
    )
    return lat, lon


def _prepare_mission():
    guidance.reset()
    control.reset()
    guidance.set_origin_point(ORIGIN_LAT, ORIGIN_LON)
    target_lat, target_lon = _latlon_from_ne(TARGET_N, TARGET_E)
    guidance.set_target_point(target_lat, target_lon)


def _gps_at(now, n_m, e_m, *, pos=True, motion=True, course=COURSE_EAST, speed=4.0):
    lat, lon = _latlon_from_ne(n_m, e_m)
    return _GpsFromApp(
        lat=lat,
        lon=lon,
        course_rad=course if motion else None,
        speed_mps=speed if motion else None,
        pos_ts=now if pos else None,
        motion_ts=now if motion else None,
        pos_health=1 if pos else 0,
        motion_health=1 if motion else 0,
    )


def _imu_at(now, *, gyrz_dps=0.0, yaw=None, acc=False):
    return _ImuFromApp(
        yaw_rad=yaw,
        gyrz_rad_s=math.radians(gyrz_dps),
        ts=now,
        lin_acc_x=0.1 if acc else None,
        lin_acc_y=0.0 if acc else None,
        lin_acc_valid=acc,
        health=1,
    )


def _baro_at(now, *, sink=2.0):
    return _BaroFromApp(alt_m=120.0, sink_rate=sink, rx_ts=now, health=1)


def scenario_sensor_schedule(now):
    """Deterministic sensor schedule for the required cycle pipeline."""
    if math.isclose(now, T0):
        return (
            _gps_at(now, 0.0, 0.0, pos=True, motion=True, course=COURSE_EAST, speed=4.0),
            _imu_at(now, gyrz_dps=0.0),
            _baro_at(now),
            {"E": 0.0, "N": 0.0, "course": COURSE_EAST},
        )
    if math.isclose(now, T_DR_M):
        return (
            _gps_at(now, 1.0, 0.8, pos=True, motion=False),
            _imu_at(now, gyrz_dps=10.0),
            _baro_at(now),
            {"E": 0.8, "N": 1.0, "course": COURSE_EAST},
        )
    if math.isclose(now, T_DR_PM_1):
        return (
            None,
            _imu_at(now, gyrz_dps=10.0),
            _baro_at(now),
            {"E": 3.0, "N": 6.0, "course": COURSE_EAST},
        )
    if math.isclose(now, T_DR_PM_2):
        return (
            None,
            _imu_at(now, gyrz_dps=10.0),
            _baro_at(now),
            {"E": 3.8, "N": 8.5, "course": COURSE_EAST},
        )
    raise AssertionError(f"unexpected test time: {now}")


def _bearing_error_from_position(e_m, n_m, course, target_e, target_n):
    bearing = math.atan2(target_e - e_m, target_n - n_m)
    return _wrap_angle(bearing - course)


def _wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _dr_snapshot():
    return asdict(copy.deepcopy(guidance._STATE_t.dr))


def _values_equal_nan_ok(left, right):
    if isinstance(left, float) or isinstance(right, float):
        left_f = float(left)
        right_f = float(right)
        if math.isnan(left_f) and math.isnan(right_f):
            return True
        return math.isclose(left_f, right_f, rel_tol=0.0, abs_tol=1.0e-12)
    return left == right


class TestGuidanceCyclePipeline(unittest.TestCase):
    def setUp(self):
        _prepare_mission()

    def _run_cycle(self, now):
        gps, imu, baro, truth = scenario_sensor_schedule(now)

        mode = guidance.DecideControlMode(gps, imu, baro, now)
        dr_after_decide = _dr_snapshot()
        nav_after_decide = copy.deepcopy(guidance._STATE_t.nav)

        l1_in = guidance.ProduceL1Input(now)
        self._assert_dr_unchanged(dr_after_decide)

        l1_out = None
        if not l1_in.valid:
            ctrl_out = control.WriteNeutral(now, mode)
        else:
            l1_out = guidance.ProduceL1Output(l1_in)
            if not l1_out.valid:
                ctrl_out = control.WriteNeutral(now, mode)
            else:
                ctrl_in = control.ProduceCtrlInput(l1_out, now)
                gyrz = guidance._STATE_t.imu.gyr_z
                gyrz_meas_deg_s = math.degrees(gyrz) if math.isfinite(gyrz) else float("nan")
                ctrl_out = control.ProduceCtrlOutput(ctrl_in, gyrz_meas_deg_s, now)

        self._assert_dr_unchanged(dr_after_decide)
        flags = copy.deepcopy(guidance._STATE_t.flags)
        nav = copy.deepcopy(guidance._STATE_t.nav)
        dr = copy.deepcopy(guidance._STATE_t.dr)
        target_e = l1_in.target_E
        target_n = l1_in.target_N

        estimated_nu = getattr(l1_out, "nu", float("nan")) if l1_out else float("nan")
        truth_nu = _bearing_error_from_position(
            truth["E"], truth["N"], truth["course"], target_e, target_n
        )

        return {
            "now": now,
            "mode": mode,
            "flags": flags,
            "nav": nav,
            "dr": dr,
            "nav_after_decide": nav_after_decide,
            "l1_in": l1_in,
            "l1_out": l1_out,
            "ctrl_out": ctrl_out,
            "truth": truth,
            "estimated_nu": estimated_nu,
            "truth_nu": truth_nu,
            "stored_gps_E": guidance._STATE_t.gps.E,
            "stored_gps_N": guidance._STATE_t.gps.N,
            "stored_gps_course": guidance._STATE_t.gps.course,
        }

    def _assert_dr_unchanged(self, expected):
        actual = _dr_snapshot()
        self.assertEqual(expected.keys(), actual.keys())
        for key in expected:
            self.assertTrue(
                _values_equal_nan_ok(expected[key], actual[key]),
                f"DR field mutated after DecideControlMode: {key}",
            )

    def test_decide_calculates_nav_once_per_cycle_and_l1_uses_filled_nav(self):
        times = [T0, T_DR_M, T_DR_PM_1, T_DR_PM_2]
        rows = []
        with mock.patch.object(guidance, "FillNav", wraps=guidance.FillNav) as fill_nav:
            for index, now in enumerate(times):
                rows.append(self._run_cycle(now))
                self.assertEqual(fill_nav.call_count, index + 1)

        for row in rows:
            nav = row["nav"]
            l1_in = row["l1_in"]
            self.assertEqual(row["mode"], nav.control_mode)
            self.assertEqual(nav.timestamp, row["now"])
            self.assertTrue(nav.valid)
            self.assertTrue(l1_in.valid, l1_in.reason)
            self.assertEqual(l1_in.control_mode, nav.control_mode)
            self.assertAlmostEqual(l1_in.E, nav.E)
            self.assertAlmostEqual(l1_in.N, nav.N)
            self.assertAlmostEqual(l1_in.V, nav.V)
            self.assertAlmostEqual(l1_in.course, nav.course)
            self.assertAlmostEqual(l1_in.vE, nav.vE)
            self.assertAlmostEqual(l1_in.vN, nav.vN)

    def test_dr_m_uses_gps_position_and_dr_motion_only(self):
        self._run_cycle(T0)
        row = self._run_cycle(T_DR_M)

        self.assertEqual(row["mode"], guidance.ControlMode.DR_M_GB_CLOSED)
        self.assertTrue(row["flags"].gps_pos_fresh)
        self.assertFalse(row["flags"].gps_motion_fresh)
        self.assertAlmostEqual(row["nav"].E, row["stored_gps_E"])
        self.assertAlmostEqual(row["nav"].N, row["stored_gps_N"])
        self.assertAlmostEqual(row["dr"].current_E, row["stored_gps_E"])
        self.assertAlmostEqual(row["dr"].current_N, row["stored_gps_N"])
        self.assertGreater(abs(row["nav"].course - row["stored_gps_course"]), math.radians(0.5))

    def test_dr_pm_accumulates_position_and_motion_from_dr_current(self):
        self._run_cycle(T0)
        dr_m = self._run_cycle(T_DR_M)
        dr_pm = self._run_cycle(T_DR_PM_1)

        self.assertEqual(dr_pm["mode"], guidance.ControlMode.DR_PM_GB_CLOSED)
        self.assertFalse(dr_pm["flags"].gps_pos_fresh)
        self.assertFalse(dr_pm["flags"].gps_motion_fresh)
        self.assertAlmostEqual(dr_pm["nav"].E, dr_pm["dr"].current_E)
        self.assertAlmostEqual(dr_pm["nav"].N, dr_pm["dr"].current_N)
        self.assertAlmostEqual(dr_pm["nav"].course, dr_pm["dr"].current_course)
        self.assertAlmostEqual(dr_pm["nav"].V, dr_pm["dr"].current_V)
        self.assertNotAlmostEqual(dr_pm["nav"].E, dr_m["stored_gps_E"])
        self.assertGreater(abs(dr_pm["nav"].course - dr_m["stored_gps_course"]), math.radians(0.5))

    def test_l1_to_control_connection_remains_valid_for_gps_dr_m_and_dr_pm(self):
        rows = [self._run_cycle(t) for t in (T0, T_DR_M, T_DR_PM_1)]

        for row in rows:
            l1_in = row["l1_in"]
            l1_out = row["l1_out"]
            ctrl_out = row["ctrl_out"]
            self.assertTrue(l1_in.valid, l1_in.reason)
            self.assertIsNotNone(l1_out)
            self.assertTrue(l1_out.valid, l1_out.reason)
            self.assertEqual(l1_out.control_mode, row["mode"])
            self.assertEqual(ctrl_out.control_mode, row["mode"])
            self.assertTrue(ctrl_out.valid, ctrl_out.reason)
            self.assertGreaterEqual(ctrl_out.left_pw, control.LEFT_MIN_PULSE)
            self.assertLessEqual(ctrl_out.left_pw, control.LEFT_MAX_PULSE)
            self.assertGreaterEqual(ctrl_out.right_pw, control.RIGHT_MIN_PULSE)
            self.assertLessEqual(ctrl_out.right_pw, control.RIGHT_MAX_PULSE)

    def test_dr_drift_changes_target_direction_truth_vs_estimated(self):
        rows = [self._run_cycle(t) for t in (T0, T_DR_M, T_DR_PM_1, T_DR_PM_2)]
        initial_gap = abs(_wrap_angle(rows[1]["estimated_nu"] - rows[1]["truth_nu"]))
        final_gap = abs(_wrap_angle(rows[-1]["estimated_nu"] - rows[-1]["truth_nu"]))

        self.assertAlmostEqual(rows[1]["nav"].E, rows[1]["truth"]["E"], delta=1.0e-6)
        self.assertAlmostEqual(rows[1]["nav"].N, rows[1]["truth"]["N"], delta=1.0e-6)
        self.assertAlmostEqual(rows[-1]["l1_out"].nu, rows[-1]["estimated_nu"])
        self.assertGreater(final_gap, initial_gap + math.radians(2.0))


if __name__ == "__main__":
    unittest.main()
