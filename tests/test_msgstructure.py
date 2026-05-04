import unittest
from multiprocessing import Queue

from lib import msgstructure


class TestMsgStructure(unittest.TestCase):
    def test_fill_msg_success(self):
        msg = msgstructure.fill_msg(12, 19, 1201901, "ON")
        self.assertIsNotNone(msg)
        self.assertEqual(msg.sender_app, 12)
        self.assertEqual(msg.receiver_app, 19)
        self.assertEqual(msg.msg_id, 1201901)
        self.assertEqual(msg.data, "ON")

    def test_fill_msg_rejects_data_delimiter(self):
        msg = msgstructure.fill_msg(12, 19, 1201901, "A|B")
        self.assertIsNone(msg)

    def test_pack_msg_success(self):
        msg = msgstructure.MsgStructure(10, 11, 1001001, "")
        packed = msgstructure.pack_msg(msg)
        self.assertEqual(packed, "10|11|1001001|")

    def test_unpack_msg_success(self):
        unpacked = msgstructure.unpack_msg("10|11|1001001|")
        self.assertNotEqual(unpacked, False)
        self.assertEqual(unpacked.sender_app, 10)
        self.assertEqual(unpacked.receiver_app, 11)
        self.assertEqual(unpacked.msg_id, 1001001)
        self.assertEqual(unpacked.data, "")

    def test_unpack_msg_rejects_bad_field_count(self):
        self.assertFalse(msgstructure.unpack_msg("10|11|1001001"))
        self.assertFalse(msgstructure.unpack_msg("10|11|1001001|x|y"))

    def test_unpack_msg_rejects_non_numeric_ids(self):
        self.assertFalse(msgstructure.unpack_msg("AA|11|1001001|x"))
        self.assertFalse(msgstructure.unpack_msg("10|BB|1001001|x"))
        self.assertFalse(msgstructure.unpack_msg("10|11|MID|x"))

    def test_send_msg_success(self):
        q = Queue()
        ok = msgstructure.send_msg(q, 13, 11, 1301101, "123.45")
        self.assertTrue(ok)
        self.assertEqual(q.get(timeout=0.2), "13|11|1301101|123.45")

    def test_send_msg_fail_on_invalid(self):
        q = Queue()
        ok = msgstructure.send_msg(q, 13, 11, 1301101, "bad|payload")
        self.assertFalse(ok)

    def test_send_msg_returns_false_when_queue_full(self):
        q = Queue(maxsize=1)
        q.put("seed")
        old_drop = msgstructure._MSG_QUEUE_DROP_WHEN_FULL
        old_timeout = msgstructure._MSG_QUEUE_PUT_TIMEOUT_SEC
        try:
            msgstructure._MSG_QUEUE_DROP_WHEN_FULL = True
            msgstructure._MSG_QUEUE_PUT_TIMEOUT_SEC = 0.0
            ok = msgstructure.send_msg(q, 13, 11, 1301101, "x")
            self.assertFalse(ok)
        finally:
            msgstructure._MSG_QUEUE_DROP_WHEN_FULL = old_drop
            msgstructure._MSG_QUEUE_PUT_TIMEOUT_SEC = old_timeout


if __name__ == "__main__":
    unittest.main()
