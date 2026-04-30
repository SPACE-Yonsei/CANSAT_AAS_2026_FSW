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

    def test_rbt_requires_token(self):
        prev = os.environ.get("RBT_AUTH_TOKEN")
        os.environ["RBT_AUTH_TOKEN"] = "SECRET"
        # module-level token is fixed at import-time; direct call should fail here
        self.assertFalse(commapp.cmd_rbt("SECRET", None))
        if prev is None:
            del os.environ["RBT_AUTH_TOKEN"]
        else:
            os.environ["RBT_AUTH_TOKEN"] = prev

    def test_tlm_contains_distance(self):
        ser = DummySerial()
        commapp.tlm_data.distance = 4321.0
        commapp.send_tlm(ser)
        self.assertTrue(ser.writes)
        line = ser.writes[-1].decode("utf-8")
        self.assertIn(",4321.0,", line)


if __name__ == "__main__":
    unittest.main()
