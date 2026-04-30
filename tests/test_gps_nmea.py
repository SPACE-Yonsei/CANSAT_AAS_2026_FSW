import os
import unittest

from Sensor_Gps import gps as gps_mod


def _xor_nmea_payload(payload_after_dollar: str) -> int:
    x = 0
    for c in payload_after_dollar:
        x ^= ord(c) & 0xFF
    return x


class TestGpsNmeaChecksum(unittest.TestCase):
    def test_checksum_valid_known_sentence(self):
        body = "GNGGA,151544,3755.8370,N,12656.7170,E,1,05,1.0,126.0,M,46.9,M,,"
        cs = _xor_nmea_payload(body)
        line = f"${body}*{cs:02X}"
        self.assertTrue(gps_mod._nmea_checksum_valid(line))

    def test_checksum_rejects_tamper(self):
        body = "GNGGA,151544,3755.8370,N,12656.7170,E,1,05,1.0,126.0,M,46.9,M,,"
        cs = _xor_nmea_payload(body)
        line = f"${body}*{cs:02X}"
        bad = line.replace("3755.8370", "0755.8370")
        self.assertFalse(gps_mod._nmea_checksum_valid(bad))

    def test_ingest_ignores_bad_checksum(self):
        os.environ["GPS_NMEA_STRICT_CHECKSUM"] = "1"
        body = "GNGGA,151544,0755.8370,N,12656.7170,E,1,05,1.0,126.0,M,46.9,M,,"
        cs = _xor_nmea_payload("GNGGA,151544,3755.8370,N,12656.7170,E,1,05,1.0,126.0,M,46.9,M,,")
        line = f"${body}*{cs:02X}"
        dev: dict = {"gga": None, "rmc": None}
        gps_mod._ingest_nmea_line(dev, line)
        self.assertIsNone(dev.get("gga"))

    def test_rmc_course_clamp(self):
        parts = [
            "GNRMC",
            "151544",
            "A",
            "3755.8370",
            "N",
            "12656.7170",
            "E",
            "0.1",
            "300426.0",
            "300425",
            "",
            "",
        ]
        r = gps_mod._parse_rmc(parts)
        self.assertIsNotNone(r)
        assert r is not None
        self.assertEqual(r["course"], 0.0)


if __name__ == "__main__":
    unittest.main()
