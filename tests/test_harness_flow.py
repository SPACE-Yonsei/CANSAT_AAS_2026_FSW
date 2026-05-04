import queue
import unittest

from comm import commapp
from flight_logic import flightlogicapp
from Sensor_Motor import motorapp
from lib import appargs, msgstructure


class TestHarnessFlow(unittest.TestCase):
    def setUp(self):
        flightlogicapp.state = 0
        flightlogicapp.recent_alt = []
        flightlogicapp.sim_enable = False
        flightlogicapp.sim_active = False
        motorapp.target.lat = 0.0
        motorapp.target.lon = 0.0
        commapp.tlm_data = commapp.TelemetryData()

    def _route_once(self, q):
        packed = q.get_nowait()
        unpacked = msgstructure.unpack_msg(packed)
        if unpacked.receiver_app == appargs.FlightlogicAppArg.AppID:
            flightlogicapp.dispatch(packed, q)
        elif unpacked.receiver_app == appargs.MotorAppArg.AppID:
            motorapp.dispatch(packed)
        elif unpacked.receiver_app == appargs.CommAppArg.AppID:
            commapp.command_handler(packed)

    def test_cmd_ss_routes_to_flightlogic(self):
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
        self._route_once(q)  # FlightLogic -> Motor
        self.assertAlmostEqual(motorapp.target.lat, 37.55)
        self.assertAlmostEqual(motorapp.target.lon, 126.95)

    def test_sim_enable_routes_comm_mode_a(self):
        """SIM ENABLE -> FlightLogic -> Comm MID_comm_sim; first CSV field is A (SIM prepare)."""
        q = queue.Queue()
        ok = commapp._dispatch_command("CMD,1070,SIM,ENABLE", q)
        self.assertTrue(ok)
        self._route_once(q)  # Comm -> FlightLogic (handle_sim + _verify_inter_app_links)
        self._route_once(q)  # Main-router analogue: first post-FL frame -> Comm (MID_comm_sim)
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
