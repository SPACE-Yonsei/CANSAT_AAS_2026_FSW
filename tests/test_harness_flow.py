import queue
import tempfile
import unittest
from pathlib import Path

from comm import commapp
from flight_logic import flightlogicapp
from Sensor_Motor import motorapp
from lib import appargs, msgstructure, prevstate


class TestHarnessFlow(unittest.TestCase):
    def setUp(self):
        # Sandbox prevstate so SS/TC writes don't pollute the real
        # lib/prevstate.json or fight other tests in parallel CI.
        self._tmpdir = tempfile.TemporaryDirectory()
        self._orig_state_file = prevstate._STATE_FILE
        prevstate._STATE_FILE = Path(self._tmpdir.name) / "prevstate.json"
        prevstate.reset_prevstate()

        flightlogicapp.state = 0
        flightlogicapp.recent_alt = []
        flightlogicapp.sim_enable = False
        flightlogicapp.sim_active = False
        motorapp._CACHE = motorapp._Cache()
        motorapp._START_POINT_LOCKED = False
        motorapp.STATE = 0
        commapp.tlm_data = commapp.TelemetryData()

    def tearDown(self):
        prevstate._STATE_FILE = self._orig_state_file
        self._tmpdir.cleanup()

    def _route_once(self, q):
        packed = q.get_nowait()
        unpacked = msgstructure.unpack_msg(packed)
        if unpacked.receiver_app == appargs.FlightlogicAppArg.AppID:
            flightlogicapp.dispatch(packed, q)
        elif unpacked.receiver_app == appargs.MotorAppArg.AppID:
            motorapp.dispatch(packed)
        elif unpacked.receiver_app == appargs.CommAppArg.AppID:
            commapp.command_handler(packed)

    def _drain_until(self, q, predicate, max_steps: int = 16):
        """Route messages off ``q`` until ``predicate()`` is True or the queue drains."""
        for _ in range(max_steps):
            if predicate():
                return True
            if q.empty():
                return predicate()
            self._route_once(q)
        return predicate()

    def test_cmd_ss_routes_to_flightlogic(self):
        # SS,3 requires a valid release target; pre-seed via TC then issue SS.
        prevstate.update_target_gps(37.56, 126.93)
        q = queue.Queue()
        ok = commapp._dispatch_command("CMD,1070,SS,3", q)
        self.assertTrue(ok)
        self._route_once(q)
        self.assertEqual(flightlogicapp.state, 3)

    def test_cmd_tc_routes_to_motor_via_flightlogic(self):
        q = queue.Queue()
        ok = commapp._dispatch_command("CMD,1070,TC,37.55,126.95", q)
        self.assertTrue(ok)
        self._route_once(q)  # Comm -> FlightLogic
        self._route_once(q)  # FlightLogic -> Motor (MID_motor_TargetCor)
        self.assertAlmostEqual(motorapp._CACHE.target_lat, 37.55)
        self.assertAlmostEqual(motorapp._CACHE.target_lon, 126.95)

    def test_sim_enable_routes_comm_mode_a(self):
        """SIM ENABLE -> FlightLogic -> (GPS CLEAR + Comm MID_comm_sim "A")."""
        q = queue.Queue()
        ok = commapp._dispatch_command("CMD,1070,SIM,ENABLE", q)
        self.assertTrue(ok)
        # Comm -> FlightLogic emits two frames: GPS CLEAR + Comm "A".
        # Order is FIFO from the queue; drain both rather than asserting a
        # specific dispatch order so the test is robust to handle_sim reorders.
        self._route_once(q)  # Comm -> FlightLogic
        self._drain_until(q, lambda: commapp.tlm_data.mode == "A")
        self.assertEqual(commapp.tlm_data.mode, "A")
        self.assertTrue(flightlogicapp.sim_enable)
        self.assertFalse(flightlogicapp.sim_active)

    def test_simp_ignored_until_activate(self):
        """SIMP is dropped unless SIM ACTIVATE (sim_active); align with fsw_step1_contract."""
        q = queue.Queue()
        flightlogicapp.sim_enable = True
        flightlogicapp.sim_active = False
        flightlogicapp.recent_alt = []
        packed = msgstructure.fill_msg(
            appargs.CommAppArg.AppID,
            appargs.FlightlogicAppArg.AppID,
            appargs.CommAppArg.MID_RouteCmd_SIMP,
            "350.0",
        )
        flightlogicapp.dispatch(msgstructure.pack_msg(packed), q)
        self.assertEqual(len(flightlogicapp.recent_alt), 0)


if __name__ == "__main__":
    unittest.main()

