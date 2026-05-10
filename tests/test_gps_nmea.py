import os
import struct
import time
import unittest
from contextlib import contextmanager
from unittest import mock

from Sensor_Gps import gps as gps_mod


def _xor_nmea_payload(payload_after_dollar: str) -> int:
    x = 0
    for c in payload_after_dollar:
        x ^= ord(c) & 0xFF
    return x


def _nmea_sentence(body: str) -> bytes:
    return f"${body}*{_xor_nmea_payload(body):02X}\r\n".encode("ascii")


@contextmanager
def _null_lock():
    yield


class _FakeI2CRecovery:
    def __init__(self, recovery: bytes):
        self.recovery = recovery
        self.offset = 0

    def writeto_then_readfrom(self, address, reg, buf, out_end=1, in_end=1):
        if reg in (bytes([0xFD]), bytes([0xFE])):
            raise OSError(5, "Input/output error")
        if reg == bytes([0xFF]):
            for i in range(in_end):
                idx = self.offset + i
                buf[i] = self.recovery[idx] if idx < len(self.recovery) else 0xFF
            self.offset += in_end
            return
        raise AssertionError(f"unexpected register: {reg!r}")


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
                "_seen_ts": time.time(),
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

    def test_stale_ubx_falls_back_to_fresh_gga(self):
        now = time.time()
        dev = {
            "pvt": {
                "time": "010203",
                "alt": 80.0,
                "lat": 37.0,
                "lon": 126.0,
                "num_sv": 12,
                "fix_type": 3,
                "_seen_ts": now - 10.0,
            },
            "gga": {
                "time": "151544",
                "alt": 126.0,
                "lat": 37.9306167,
                "lon": 126.9452833,
                "sats": 8,
                "fix_q": 1,
                "_seen_ts": now,
            },
            "rmc": None,
        }
        with mock.patch.dict(os.environ, {"GPS_ROW_STALE_MAX_SEC": "3"}, clear=False):
            row = gps_mod._gps_build_return(dev)
        self.assertIsNotNone(row)
        assert row is not None
        self.assertEqual(row[0], "151544")
        self.assertAlmostEqual(row[2], 37.9306167)
        self.assertEqual(row[12], "NMEA")

    def test_stale_ubx_without_gga_returns_none(self):
        dev = {
            "pvt": {
                "time": "010203",
                "alt": 80.0,
                "lat": 37.0,
                "lon": 126.0,
                "num_sv": 12,
                "fix_type": 3,
                "_seen_ts": time.time() - 10.0,
            },
            "gga": None,
            "rmc": None,
        }
        with mock.patch.dict(os.environ, {"GPS_ROW_STALE_MAX_SEC": "3"}, clear=False):
            self.assertIsNone(gps_mod._gps_build_return(dev))

    def test_nmea_tail_trim_prevents_unbounded_partial_buffer(self):
        dev = {"_nmea_tail": b"x" * 80, "gga": None, "rmc": None}
        with mock.patch.dict(os.environ, {"GPS_NMEA_TAIL_MAX_BYTES": "64"}, clear=False):
            gps_mod._nmea_feed_bytes(dev, b"y" * 16)
        self.assertEqual(dev["_nmea_tail"], b"")

    def test_i2c_available_error_can_recover_by_stream_reading_gga(self):
        body = "GNGGA,151544,3755.8370,N,12656.7170,E,1,08,1.0,126.0,M,46.9,M,,"
        sentence = _nmea_sentence(body)
        dev = {
            "kind": "i2c_ublox",
            "addr": 0x42,
            "gga": None,
            "rmc": None,
            "pvt": None,
            "_nmea_tail": b"",
            "_ubx_tail": b"",
            "_i2c_error_count": 0,
        }
        fake = _FakeI2CRecovery(sentence)
        with mock.patch("lib.i2c_bus.get_i2c", return_value=fake), mock.patch(
            "lib.i2c_bus.i2c_lock", side_effect=_null_lock
        ), mock.patch("lib.i2c_bus.reset_i2c") as reset_i2c, mock.patch.dict(
            os.environ,
            {
                "GPS_I2C_RECOVERY_READ_BYTES": str(len(sentence)),
                "GPS_ROW_STALE_MAX_SEC": "3",
            },
            clear=False,
        ):
            row = gps_mod._gps_readdata_i2c(dev)
        self.assertIsNotNone(row)
        assert row is not None
        self.assertEqual(row[12], "NMEA")
        self.assertAlmostEqual(row[2], 37.9306167, places=5)
        self.assertEqual(dev["_i2c_error_count"], 0)
        reset_i2c.assert_not_called()


if __name__ == "__main__":
    unittest.main()
