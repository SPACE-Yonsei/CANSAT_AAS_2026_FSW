import os
import queue
import unittest

from comm import commapp
from lib import appargs, msgstructure


class DummySerial:
    def __init__(self):
        self.is_open = True
        self.writes = []

    def write(self, b: bytes):
        self.writes.append(b)


class TestCommApp(unittest.TestCase):
    def setUp(self):
        commapp.COMMAPP_RUNSTATUS = True
        commapp.TELEMETRY_ENABLE = True
        commapp._comm_serial = None
        commapp._simp_tlm_alt_hold = None
        commapp.tlm_data = commapp.TelemetryData()
        commapp.tlm_data.state = "0"
        commapp._reset_tlm_geo_dedupe()
        commapp._RBT_AUTH_TOKEN = "SECRET"
        commapp._RBT_REQUIRE_SEQ = True
        commapp._RBT_SAFE_STATES = {"0", "5"}
        commapp._RBT_LAST_SEQ = -1
        commapp._RBT_RECENT_NONCES = []
        self.reboot_called = False
        self.prev_exec = commapp._execute_reboot
        commapp._execute_reboot = self._fake_reboot

    def tearDown(self):
        commapp._execute_reboot = self.prev_exec

    def _fake_reboot(self):
        self.reboot_called = True

    def test_dispatch_route_command(self):
        q = queue.Queue()
        ok = commapp._dispatch_command("CMD,1070,MEC,ON", q)
        self.assertTrue(ok)
        msg = q.get_nowait()
        self.assertIn("|19|", msg)

    def test_dispatch_command_case_insensitive_token(self):
        q = queue.Queue()
        self.assertTrue(commapp._dispatch_command("CMD,1070,ss,3", q))
        msg = q.get_nowait()
        self.assertIn(f"|{appargs.FlightlogicAppArg.AppID}|", msg)
        self.assertIn(f"|{appargs.CommAppArg.MID_RouteCmd_SS}|", msg)
        self.assertTrue(msg.endswith("|3"))
        self.assertEqual(commapp.tlm_data.cmd_echo, "SS")

    def test_dispatch_rejects_invalid(self):
        q = queue.Queue()
        self.assertFalse(commapp._dispatch_command("BAD,LINE", q))
        self.assertFalse(commapp._dispatch_command("CMD,1070,UNKNOWN,1", q))

    def test_dispatch_simg_routes_to_flightlogic(self):
        q = queue.Queue()
        ok = commapp._dispatch_command("CMD,1070,SIMG,37.56,126.93,90,8.5", q)
        self.assertTrue(ok)
        msg = q.get_nowait()
        self.assertIn(f"|{appargs.FlightlogicAppArg.AppID}|", msg)
        self.assertIn(f"|{appargs.CommAppArg.MID_RouteCmd_SIMG}|", msg)
        self.assertTrue(msg.endswith("|37.56,126.93,90,8.5"))

    def test_dispatch_simg_rejects_bad_coords(self):
        q = queue.Queue()
        self.assertFalse(commapp._dispatch_command("CMD,1070,SIMG,0,0,90,1", q))
        self.assertTrue(q.empty())
        self.assertFalse(commapp._dispatch_command("CMD,1070,SIMG,37,126", q))
        self.assertTrue(q.empty())

    def test_rbt_requires_token_and_sequence(self):
        self.assertFalse(commapp.cmd_rbt("WRONG,1,aaa", None))
        self.assertFalse(commapp.cmd_rbt("SECRET", None))
        self.assertTrue(commapp.cmd_rbt("SECRET,1,aaa", None))
        self.assertTrue(self.reboot_called)

    def test_rbt_replay_block(self):
        self.assertTrue(commapp.cmd_rbt("SECRET,2,nonce1", None))
        self.assertFalse(commapp.cmd_rbt("SECRET,2,nonce2", None))  # seq replay
        self.assertFalse(commapp.cmd_rbt("SECRET,3,nonce1", None))  # nonce replay

    def test_rbt_state_gate(self):
        commapp.tlm_data.state = "3"
        self.assertFalse(commapp.cmd_rbt("SECRET,4,x", None))

    def test_parse_rbt_auth(self):
        self.assertEqual(commapp._parse_rbt_auth("SECRET,10,abc"), ("SECRET", 10, "abc"))
        self.assertEqual(commapp._parse_rbt_auth("SECRET:11:def"), ("SECRET", 11, "def"))
        self.assertEqual(commapp._parse_rbt_auth("SECRET"), ("SECRET", None, None))

    def test_cx_off_sends_one_final_tlm_frame(self):
        ser = DummySerial()
        commapp._comm_serial = ser
        commapp.tlm_data.packet_count = 10
        q = queue.Queue()
        self.assertTrue(commapp._dispatch_command("CMD,1070,CX,OFF", q))
        self.assertFalse(commapp.TELEMETRY_ENABLE)
        self.assertEqual(len(ser.writes), 1)
        line = ser.writes[0].decode("utf-8")
        self.assertIn(",CX,", line)
        commapp.send_tlm(ser)
        self.assertEqual(len(ser.writes), 1)

    def test_tlm_contains_distance(self):
        commapp.TELEMETRY_ENABLE = True
        ser = DummySerial()
        commapp.tlm_data.distance = 4321.0
        commapp.send_tlm(ser)
        self.assertTrue(ser.writes)
        line = ser.writes[-1].decode("utf-8")
        self.assertIn(",4321.0,", line)

    def test_tlm_omits_repeated_start_target_on_wire(self):
        """Same start/target as previous downlink frame -> empty CSV fields (GCS holds last)."""
        commapp.TELEMETRY_ENABLE = True
        ser = DummySerial()
        diag = (
            "1500,1500,12.111000,34.222000,56.333000,78.444000,"
            "37.500000,126.600000,10.0,20.0,ACTIVE"
        )
        msg = msgstructure.fill_msg(
            appargs.MotorAppArg.AppID,
            appargs.CommAppArg.AppID,
            appargs.MotorAppArg.MID_comm_motor_diag,
            diag,
        )
        commapp.command_handler(msgstructure.pack_msg(msg))
        commapp.send_tlm(ser)
        line1 = ser.writes[-1].decode("utf-8")
        self.assertIn("12.111000", line1)
        self.assertIn("56.333000", line1)
        commapp.command_handler(msgstructure.pack_msg(msg))
        commapp.send_tlm(ser)
        line2 = ser.writes[-1].decode("utf-8")
        self.assertNotIn("12.111000", line2)
        self.assertNotIn("56.333000", line2)

    def test_simp_baro_does_not_overwrite_tlm_alt_in_sim_mode(self):
        commapp.tlm_data.mode = "S"
        commapp._simp_tlm_alt_hold = 120.0
        commapp.tlm_data.altitude = 120.0
        msg = msgstructure.fill_msg(
            appargs.BarometerAppArg.AppID,
            appargs.CommAppArg.AppID,
            appargs.BarometerAppArg.MID_comm_alt,
            "1013.0,20.0,59.99",
        )
        commapp.command_handler(msgstructure.pack_msg(msg))
        self.assertEqual(commapp.tlm_data.altitude, 120.0)
        self.assertAlmostEqual(commapp.tlm_data.pressure, 1013.0)

    def test_simp_hold_cleared_when_sim_disabled(self):
        commapp._simp_tlm_alt_hold = 120.0
        msg = msgstructure.fill_msg(
            appargs.FlightlogicAppArg.AppID,
            appargs.CommAppArg.AppID,
            appargs.FlightlogicAppArg.MID_comm_sim,
            "F",
        )
        commapp.command_handler(msgstructure.pack_msg(msg))
        self.assertIsNone(commapp._simp_tlm_alt_hold)
        self.assertEqual(commapp.tlm_data.mode, "F")

    def test_tlm_multiline_for_console(self):
        line = "$1070," + ",".join(str(i) for i in range(29)) + "\n"
        pretty = commapp._tlm_multiline_for_console(line)
        self.assertIn("\n", pretty)
        self.assertIn("meta", pretty)
        self.assertIn("gps", pretty)

    def test_command_handler_drops_malformed_numeric_payload(self):
        msg = msgstructure.fill_msg(
            appargs.ImuAppArg.AppID,
            appargs.CommAppArg.AppID,
            appargs.ImuAppArg.MID_comm_euler,
            "x,2,3,4,5,6,7,8,9,10,11,12",
        )
        packed = msgstructure.pack_msg(msg)
        commapp.command_handler(packed)
        self.assertTrue(commapp.COMMAPP_RUNSTATUS)

    def test_command_handler_keeps_running_after_bad_gps_payload(self):
        msg = msgstructure.fill_msg(
            appargs.GpsAppArg.AppID,
            appargs.CommAppArg.AppID,
            appargs.GpsAppArg.MID_comm_gga,
            "120000,alt,37.0,126.0,5",
        )
        packed = msgstructure.pack_msg(msg)
        commapp.command_handler(packed)
        self.assertTrue(commapp.COMMAPP_RUNSTATUS)


if __name__ == "__main__":
    unittest.main()
