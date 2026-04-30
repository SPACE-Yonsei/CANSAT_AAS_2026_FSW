import os
import queue
import unittest

from comm import commapp


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
        commapp.tlm_data = commapp.TelemetryData()
        commapp.tlm_data.state = "0"
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

    def test_dispatch_rejects_invalid(self):
        q = queue.Queue()
        self.assertFalse(commapp._dispatch_command("BAD,LINE", q))
        self.assertFalse(commapp._dispatch_command("CMD,1070,UNKNOWN,1", q))

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

    def test_tlm_contains_distance(self):
        ser = DummySerial()
        commapp.tlm_data.distance = 4321.0
        commapp.send_tlm(ser)
        self.assertTrue(ser.writes)
        line = ser.writes[-1].decode("utf-8")
        self.assertIn(",4321.0,", line)

    def test_tlm_multiline_for_console(self):
        line = "$1070," + ",".join(str(i) for i in range(29)) + "\n"
        pretty = commapp._tlm_multiline_for_console(line)
        self.assertIn("\n", pretty)
        self.assertIn("meta", pretty)
        self.assertIn("gps", pretty)


if __name__ == "__main__":
    unittest.main()
