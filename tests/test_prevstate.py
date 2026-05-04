import os
import tempfile
import unittest
from pathlib import Path

from lib import prevstate


class TestPrevState(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.TemporaryDirectory()
        self.orig_state_file = prevstate._STATE_FILE
        prevstate._STATE_FILE = Path(self.tmpdir.name) / "prevstate.json"
        prevstate.reset_prevstate()

    def tearDown(self):
        prevstate._STATE_FILE = self.orig_state_file
        self.tmpdir.cleanup()

    def test_update_and_restore(self):
        prevstate.update_prevstate(3)
        prevstate.update_altcal(12.5)
        prevstate.update_maxalt(321.0)
        prevstate.update_target_gps(37.55, 126.94)
        prevstate.update_packet_count(99)
        prevstate.update_st_timedelta(123.4)

        prevstate.PREV_STATE = 0
        prevstate.PREV_ALT_CAL = 0.0
        prevstate.PREV_MAX_ALT = 0.0
        prevstate.Target_lat = 0.0
        prevstate.Target_lon = 0.0
        prevstate.PREV_PACKET_COUNT = 0
        prevstate.PREV_ST_TIMEDELTA = 0.0

        prevstate.init_prevstate()
        self.assertEqual(prevstate.PREV_STATE, 3)
        self.assertAlmostEqual(prevstate.PREV_ALT_CAL, 12.5)
        self.assertAlmostEqual(prevstate.PREV_MAX_ALT, 321.0)
        self.assertAlmostEqual(prevstate.Target_lat, 37.55)
        self.assertAlmostEqual(prevstate.Target_lon, 126.94)
        self.assertEqual(prevstate.PREV_PACKET_COUNT, 99)
        self.assertAlmostEqual(prevstate.PREV_ST_TIMEDELTA, 123.4)

    def test_reset_prevstate(self):
        prevstate.update_prevstate(5)
        prevstate.reset_prevstate()
        self.assertEqual(prevstate.PREV_STATE, 0)
        self.assertEqual(prevstate.PREV_PACKET_COUNT, 0)
        self.assertEqual(prevstate.Target_lat, 0.0)

    def test_runtime_overrides_from_environment(self):
        os.environ["STATE_OVERRIDE"] = "4"
        os.environ["YAW_OFFSET"] = "12.5"
        try:
            prevstate.reset_prevstate()
            prevstate.init_prevstate()
            self.assertEqual(prevstate.STATE_OVERRIDE, 4)
            self.assertEqual(prevstate.PREV_STATE, 4)
            self.assertAlmostEqual(prevstate.YAW_OFFSET, 12.5)
        finally:
            os.environ.pop("STATE_OVERRIDE", None)
            os.environ.pop("YAW_OFFSET", None)
            prevstate.refresh_runtime_overrides()


if __name__ == "__main__":
    unittest.main()
