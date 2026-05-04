import os
import struct
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

    def test_build_return_marks_missing_rmc_invalid(self):
        dev = {
            "gga": {
                "time": "151544",
                "alt": 126.0,
                "lat": 37.9306167,
                "lon": 126.9452833,
                "sats": 8,
                "fix_q": 1,
                "_seen_ts": 0.0,
            },
            "rmc": None,
        }
        row = gps_mod._gps_build_return(dev)
        self.assertIsNotNone(row)
        assert row is not None
        self.assertEqual(row[6], "V")
        self.assertEqual(row[7], 0.0)
        self.assertEqual(row[8], 0.0)
        self.assertEqual(row[9], 0)

    def test_ubx_nav_pvt_preferred_when_available(self):
        payload = bytearray(92)
        struct.pack_into("<I", payload, 0, 123456)
        struct.pack_into("<H", payload, 4, 2026)
        payload[6] = 5
        payload[7] = 4
        payload[8] = 1
        payload[9] = 2
        payload[10] = 3
        payload[11] = 0x03
        payload[20] = 3
        payload[23] = 12
        struct.pack_into("<i", payload, 24, int(126.6767983 * 1e7))
        struct.pack_into("<i", payload, 28, int(37.24895833 * 1e7))
        struct.pack_into("<i", payload, 32, 85_000)
        struct.pack_into("<i", payload, 36, 80_000)
        struct.pack_into("<I", payload, 40, 2_500)
        struct.pack_into("<I", payload, 44, 4_000)
        struct.pack_into("<i", payload, 48, 3_000)
        struct.pack_into("<i", payload, 52, 4_000)
        struct.pack_into("<i", payload, 56, 0)
        struct.pack_into("<i", payload, 60, 5_000)
        struct.pack_into("<i", payload, 64, int(53.1301 * 1e5))
        struct.pack_into("<I", payload, 68, 300)
        struct.pack_into("<I", payload, 72, int(5.0 * 1e5))

        dev = {"gga": None, "rmc": None, "pvt": None, "_ubx_tail": b""}
        gps_mod._ubx_feed_bytes(dev, gps_mod._ubx_frame(0x01, 0x07, bytes(payload)))
        row = gps_mod._gps_build_return(dev)
        self.assertIsNotNone(row)
        assert row is not None
        self.assertEqual(row[0], "010203")
        self.assertAlmostEqual(row[2], 37.2489583, places=6)
        self.assertAlmostEqual(row[3], 126.6767983, places=6)
        self.assertEqual(row[4], 12)
        self.assertEqual(row[6], "A")
        self.assertAlmostEqual(row[7], 5.0)
        self.assertEqual(row[12], "UBX_NAV_PVT")
        self.assertEqual(row[13], 3)
        self.assertAlmostEqual(row[14], 2.5)
        self.assertAlmostEqual(row[16], 0.3)


if __name__ == "__main__":
    unittest.main()
