import queue
import tempfile
import unittest.mock as mock
import unittest
from pathlib import Path

from flight_logic import flightlogicapp
from lib import appargs
from lib import prevstate


class TestFlightLogicApp(unittest.TestCase):
    def setUp(self):
        # Point prevstate at a fresh tmpdir per test so atomic_update writes
        # don't pollute the real lib/prevstate.json or fight other tests.
        self._tmpdir = tempfile.TemporaryDirectory()
        self._orig_state_file = prevstate._STATE_FILE
        prevstate._STATE_FILE = Path(self._tmpdir.name) / "prevstate.json"
        prevstate.reset_prevstate()

        flightlogicapp.state = 0
        flightlogicapp.max_alt = 0.0
        flightlogicapp.recent_alt = []
        flightlogicapp.cnt_ascent = 0
        flightlogicapp.cnt_apogee = 0
        flightlogicapp.cnt_release = 0
        flightlogicapp.cnt_landed = 0
        flightlogicapp.cnt_egg_drop = 0
        flightlogicapp.solenoid_count = 0
        flightlogicapp.solenoid_done = False
        flightlogicapp.sim_enable = False
        flightlogicapp.sim_active = False
        flightlogicapp.reset_release_predictor(flightlogicapp.release_predictor)
        prevstate.update_target_gps(37.56, 126.93)

    def tearDown(self):
        prevstate._STATE_FILE = self._orig_state_file
        self._tmpdir.cleanup()

    def test_launchpad_to_ascent(self):
        q = queue.Queue()
        flightlogicapp.barometer_logic(q, 210.0)
        flightlogicapp.barometer_logic(q, 220.0)
        flightlogicapp.barometer_logic(q, 230.0)
        self.assertEqual(flightlogicapp.state, 1)

    def test_max_alt_persisted_to_prevstate(self):
        q = queue.Queue()
        flightlogicapp.barometer_logic(q, 120.0)
        flightlogicapp.barometer_logic(q, 160.0)
        flightlogicapp.barometer_logic(q, 200.0)
        self.assertAlmostEqual(prevstate.PREV_MAX_ALT, 160.0)

    def test_ss_force_state(self):
        q = queue.Queue()
        flightlogicapp.handle_ss("3", q)
        self.assertEqual(flightlogicapp.state, 3)

    def test_ss_to_egg_pushes_target_to_motor(self):
        q = queue.Queue()
        prevstate.update_target_gps(37.57, 126.94)
        flightlogicapp.handle_ss("4", q)
        found = False
        while not q.empty():
            msg = q.get_nowait()
            if (
                f"|{appargs.MotorAppArg.AppID}|" in msg
                and str(appargs.FlightlogicAppArg.MID_motor_TargetCor) in msg
                and "37.57" in msg
                and "126.94" in msg
            ):
                found = True
        self.assertTrue(found)

    def test_release_blocked_without_target(self):
        q = queue.Queue()
        flightlogicapp.state = 1
        prevstate.update_target_gps(0.0, 0.0)
        flightlogicapp.handle_ss("3", q)
        self.assertEqual(flightlogicapp.state, 1)

    def test_sim_command(self):
        q = queue.Queue()
        flightlogicapp.handle_sim("ENABLE", q)
        flightlogicapp.handle_sim("ACTIVATE", q)
        self.assertTrue(flightlogicapp.sim_enable)
        self.assertTrue(flightlogicapp.sim_active)

    def test_simg_forwards_to_gps_when_active(self):
        q = queue.Queue()
        flightlogicapp.handle_sim("ENABLE", q)
        flightlogicapp.handle_sim("ACTIVATE", q)
        while not q.empty():
            q.get_nowait()
        flightlogicapp.handle_simg("37.56,126.93,90,5", q)
        msg = q.get_nowait()
        self.assertIn(f"|{appargs.GpsAppArg.AppID}|", msg)
        self.assertIn(f"|{appargs.GpsAppArg.MID_flight_gps_sim}|", msg)
        self.assertTrue(msg.endswith("|37.56,126.93,90.0,5.0,80.0"))

    def test_target_coord_validation(self):
        q = queue.Queue()
        flightlogicapp.handle_target_coord("37.56,126.93", q)
        # Should enqueue target transfer to motor
        found = False
        while not q.empty():
            msg = q.get_nowait()
            if f"|{appargs.MotorAppArg.AppID}|" in msg and str(appargs.FlightlogicAppArg.MID_motor_TargetCor) in msg:
                found = True
                break
        self.assertTrue(found)

    def test_release_sends_reason_payload(self):
        q = queue.Queue()
        flightlogicapp.state = 2
        flightlogicapp.max_alt = 1000.0
        flightlogicapp.cnt_release = 2
        with mock.patch("flight_logic.flightlogicapp.time.time", side_effect=[1.0]):
            flightlogicapp.barometer_logic(q, 849.0)
        self.assertEqual(flightlogicapp.state, 3)
        found_release_reason = False
        while not q.empty():
            msg = q.get_nowait()
            if (
                f"|{appargs.MotorAppArg.AppID}|" in msg
                and str(appargs.FlightlogicAppArg.MID_motor_burnwire) in msg
                and "TRIGGER:" in msg
            ):
                found_release_reason = True
        self.assertTrue(found_release_reason)


if __name__ == "__main__":
    unittest.main()
