import time
import unittest
from unittest import mock

from Sensor_Gps import gpsapp_legacy as gpsapp
from lib import appargs


class TestGpsApp(unittest.TestCase):
    def setUp(self):
        gpsapp._last_fix = None
        gpsapp._last_update_ts = 0.0
        gpsapp._last_good_position = None
        gpsapp._last_good_motion = None
        gpsapp._anchor_position = None
        gpsapp._last_epoch_gps_time = None
        gpsapp._epoch_gps_time_seen_since = 0.0
        gpsapp.GPS_HEALTH = 0
        gpsapp.POS_HEALTH = 0
        gpsapp.MOTION_HEALTH = 0
        gpsapp.LAT = 37.56
        gpsapp.LON = 126.93
        gpsapp.GPS_TIME = "000000"

    def test_valid_fix(self):
        self.assertTrue(gpsapp._is_valid_fix(37.56, 126.93, 1, 8, "A"))
        self.assertFalse(gpsapp._is_valid_fix(0.0, 0.0, 0, 0, "V"))

    def test_jump_detection(self):
        now = time.time()
        self.assertFalse(gpsapp._is_jump(37.56, 126.93, now))
        # huge jump in tiny dt
        self.assertTrue(gpsapp._is_jump(38.56, 127.93, now + 0.01))

    def test_synthetic_read_shape(self):
        data = gpsapp._synthetic_read()
        self.assertGreaterEqual(len(data), 9)
        self.assertIsInstance(data[2], float)

    def test_position_health_rejects_zero_and_bad_sats(self):
        now = time.time()
        health, _ = gpsapp._position_health(0.0, 126.93, 1, 8, "041559", 0.0, now)
        self.assertEqual(health, 0)
        health, _ = gpsapp._position_health(37.56, 126.93, 1, 2, "041559", 0.0, now)
        self.assertEqual(health, 0)

    def test_position_health_rejects_bad_time_and_stale(self):
        now = time.time()
        health, _ = gpsapp._position_health(37.56, 126.93, 1, 8, "04602.", 0.0, now)
        self.assertEqual(health, 0)
        health, _ = gpsapp._position_health(37.56, 126.93, 1, 8, "041559", 99.0, now)
        self.assertEqual(health, 0)

    def test_position_health_rejects_jump(self):
        now = time.time()
        health, _ = gpsapp._position_health(37.56, 126.93, 1, 8, "041559", 0.0, now)
        self.assertEqual(health, 1)
        gpsapp._hold_or_update_position(37.56, 126.93, health, now, "041559")
        health, _ = gpsapp._position_health(38.56, 127.93, 1, 8, "041600", 0.0, now + 0.2)
        self.assertEqual(health, 0)

    def test_motion_health_requires_rmc_and_valid_course(self):
        self.assertEqual(gpsapp._motion_health(8.0, 90.0, "V", False, 0.0, None), 0)
        self.assertEqual(gpsapp._motion_health(8.0, 361.0, "A", True, 0.0, None), 0)
        self.assertEqual(gpsapp._motion_health(8.0, 90.0, "A", True, 0.0, None), 1)

    def test_motion_health_rejects_speed_spike(self):
        gpsapp._hold_or_update_motion(5.0, 90.0, 1)
        self.assertEqual(gpsapp._motion_health(40.0, 90.0, "A", True, 0.0, None), 0)

    def test_ubx_position_health_uses_accuracy(self):
        now = time.time()
        health, _ = gpsapp._position_health(
            37.56,
            126.93,
            1,
            8,
            "041559",
            0.0,
            now,
            source="UBX_NAV_PVT",
            fix_type=3,
            h_acc=2.0,
        )
        self.assertEqual(health, 1)
        health, _ = gpsapp._position_health(
            37.56,
            126.93,
            1,
            8,
            "041600",
            0.0,
            now + 1.0,
            source="UBX_NAV_PVT",
            fix_type=3,
            h_acc=99.0,
        )
        self.assertEqual(health, 0)

    def test_ubx_motion_health_uses_accuracy(self):
        self.assertEqual(
            gpsapp._motion_health(
                8.0,
                90.0,
                "A",
                True,
                0.0,
                None,
                source="UBX_NAV_PVT",
                s_acc=0.5,
                head_acc=10.0,
            ),
            1,
        )
        self.assertEqual(
            gpsapp._motion_health(
                8.0,
                90.0,
                "A",
                True,
                0.0,
                None,
                source="UBX_NAV_PVT",
                s_acc=9.0,
                head_acc=10.0,
            ),
            0,
        )

    def test_comm_gga_payload_uses_sample_gps_time(self):
        sample = ["123456", 10.5, 37.56, 126.93, 8, 1, "A", 6.0, 90.0]
        send_calls = []

        def _capture_send(*args, **kwargs):
            send_calls.append((args, kwargs))

        def _stop_after_one_tick(_period):
            gpsapp.GPSAPP_RUNSTATUS = False

        gpsapp.GPSAPP_RUNSTATUS = True
        with mock.patch("Sensor_Gps.gpsapp_legacy._read_gps", return_value=sample), mock.patch(
            "Sensor_Gps.gpsapp_legacy._comm_tick_interval", return_value=1
        ), mock.patch("Sensor_Gps.gpsapp_legacy.msgstructure.send_msg", side_effect=_capture_send), mock.patch(
            "Sensor_Gps.gpsapp_legacy.time.sleep", side_effect=_stop_after_one_tick
        ):
            gpsapp.read_and_send_gps_data(main_queue=object())

        comm_payloads = [
            args[4]
            for args, _ in send_calls
            if len(args) >= 5 and args[3] == appargs.GpsAppArg.MID_comm_gga
        ]
        self.assertTrue(comm_payloads)
        self.assertTrue(comm_payloads[0].startswith("123456,"))
        self.assertEqual(
            gpsapp._motion_health(
                8.0,
                90.0,
                "A",
                True,
                0.0,
                None,
                source="UBX_NAV_PVT",
                s_acc=0.5,
                head_acc=120.0,
            ),
            0,
        )


if __name__ == "__main__":
    unittest.main()
