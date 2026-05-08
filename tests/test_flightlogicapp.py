import queue
import unittest.mock as mock
import unittest

from flight_logic import flightlogicapp
from lib import appargs
from lib import prevstate


class TestFlightLogicApp(unittest.TestCase):
    def setUp(self):
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
        prevstate.Target_lat = 37.56
        prevstate.Target_lon = 126.93

    def test_launchpad_to_ascent(self):
        q = queue.Queue()
        flightlogicapp.barometer_logic(q, 210.0)
        flightlogicapp.barometer_logic(q, 220.0)
        flightlogicapp.barometer_logic(q, 230.0)
        self.assertEqual(flightlogicapp.state, 1)

    def test_ss_force_state(self):
        q = queue.Queue()
        flightlogicapp.handle_ss("3", q)
        self.assertEqual(flightlogicapp.state, 3)

    def test_ss_to_egg_pushes_target_to_motor(self):
        q = queue.Queue()
        prevstate.Target_lat = 37.57
        prevstate.Target_lon = 126.94
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
        prevstate.Target_lat = 0.0
        prevstate.Target_lon = 0.0
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

    def test_predictive_release_fires_above_80_percent(self):
        q = queue.Queue()
        flightlogicapp.state = 2
        flightlogicapp.max_alt = 1000.0
        with mock.patch("flight_logic.flightlogicapp.time.time", side_effect=[1.0, 2.0, 3.0, 4.0, 5.0]):
            flightlogicapp.barometer_logic(q, 860.0)
            flightlogicapp.barometer_logic(q, 850.0)
            flightlogicapp.barometer_logic(q, 830.0)
            flightlogicapp.barometer_logic(q, 820.0)
            flightlogicapp.barometer_logic(q, 810.0)
        self.assertEqual(flightlogicapp.state, 3)


if __name__ == "__main__":
    unittest.main()
