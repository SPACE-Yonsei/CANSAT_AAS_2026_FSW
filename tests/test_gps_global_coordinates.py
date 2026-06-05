import unittest

from comm import commapp


class TestGpsGlobalCoordinates(unittest.TestCase):
    def test_commapp_preserves_west_longitude_sign_on_wire(self):
        captured = []
        prev_send = commapp.uartserial.send_serial_data
        try:
            commapp.uartserial.send_serial_data = lambda _ser, line: captured.append(line) or True
            commapp.tlm_data = commapp.TelemetryData()
            commapp.tlm_data.gps_time = "12:34:56"
            commapp.tlm_data.gps_alt = 100.0
            commapp.tlm_data.gps_lat = 38.8977
            commapp.tlm_data.gps_lon = -77.0365
            commapp.tlm_data.gps_sats = 9

            commapp._send_one_tlm_frame(object())

            self.assertTrue(captured)
            parts = captured[0].rstrip("\r\n").split(",")
            self.assertEqual(parts[16], "12:34:56")
            self.assertEqual(parts[18], "38.8977")
            self.assertEqual(parts[19], "-77.0365")
            self.assertEqual(parts[20], "9")
        finally:
            commapp.uartserial.send_serial_data = prev_send


if __name__ == "__main__":
    unittest.main()
