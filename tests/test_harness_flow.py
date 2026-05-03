import queue
import unittest

from comm import commapp
from flight_logic import flightlogicapp
from Sensor_Motor import motorapp
from lib import appargs, msgstructure


class TestHarnessFlow(unittest.TestCase):
    def setUp(self):
        flightlogicapp.state = 0
        flightlogicapp.sim_enable = False
        flightlogicapp.sim_active = False
        motorapp.target = None
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


if __name__ == "__main__":
    unittest.main()
