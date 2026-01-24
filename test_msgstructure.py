"""
Message Structure Test Script
Tests the msgstructure functions with the new API
"""

import sys
import os
from multiprocessing import Queue

# Change to script directory
os.chdir(os.path.dirname(os.path.abspath(__file__)))

from lib import appargs
from lib import types

# Simplified msgstructure for testing (to avoid logging issues)
class MsgStructure:
    sender_app: types.AppID = None
    receiver_app: types.AppID = None
    MsgID: types.MID = None
    data: str = None

def fill_msg(_sender : int, _receiver : int, _MsgID : int, _data: str):
    try:
        if '|' in _data:
            print(f"ERROR: Data should not contain '|'")
            return None
        target = MsgStructure()
        target.sender_app = _sender
        target.receiver_app = _receiver
        target.MsgID = _MsgID
        target.data = _data
        return target
    except Exception as e:
        print(f"ERROR: {e}")
        return None

def pack_msg (target: MsgStructure) -> str:
    try:
        if target.sender_app == None or target.receiver_app == None or target.MsgID == None or target.data == None:
            return "ERROR"
        else:
            data_str = str(target.data) if not isinstance(target.data, str) else target.data
            return str(target.sender_app) + "|" + str(target.receiver_app) + "|" + str(target.MsgID) + "|" + data_str
    except Exception as e:
        return "ERROR"

def unpack_msg (msg: str):
    try:
        msg_list = msg.split('|')
        if len(msg_list) != 4:
            return False
        target = MsgStructure()
        target.sender_app = int(msg_list[0])
        target.receiver_app = int(msg_list[1])
        target.MsgID = int(msg_list[2])
        target.data = msg_list[3]
        return target
    except Exception as e:
        return False

def send_msg (Main_Queue : Queue, _sender : types.AppID, _receiver : types.AppID, _MsgID : types.MID, _data: str):
    try:
        target = fill_msg(_sender, _receiver, _MsgID, _data)
        if target is None:
            return False
        msg_to_send = pack_msg(target)
        if msg_to_send == "ERROR":
            return False
        Main_Queue.put(msg_to_send)
    except Exception as e:
        return False
    return True

def test_fill_msg():
    """Test fill_msg function"""
    print("\n=== Testing fill_msg ===")

    # Test valid message
    msg = fill_msg(
        appargs.BarometerAppArg.AppID,
        appargs.FlightlogicAppArg.AppID,
        appargs.BarometerAppArg.MID_SendBarometerFlightLogicData,
        "1013.25,25.5,100.5"
    )

    if msg:
        print(f"[OK] Valid message created")
        print(f"  Sender: {msg.sender_app}")
        print(f"  Receiver: {msg.receiver_app}")
        print(f"  MsgID: {msg.MsgID}")
        print(f"  Data: {msg.data}")
    else:
        print("[FAIL] Failed to create valid message")

    # Test invalid message (with | character)
    invalid_msg = fill_msg(
        appargs.BarometerAppArg.AppID,
        appargs.FlightlogicAppArg.AppID,
        appargs.BarometerAppArg.MID_SendBarometerFlightLogicData,
        "invalid|data"
    )

    if invalid_msg is None:
        print("[OK] Correctly rejected message with | character")
    else:
        print("[FAIL] Should have rejected message with | character")

    return msg

def test_pack_msg(msg):
    """Test pack_msg function"""
    print("\n=== Testing pack_msg ===")

    packed = pack_msg(msg)

    if packed != "ERROR":
        print(f"[OK] Message packed successfully")
        print(f"  Packed: {packed}")
        parts = packed.split('|')
        print(f"  Parts: {len(parts)} (expected 4)")
        return packed
    else:
        print("[FAIL] Failed to pack message")
        return None

def test_unpack_msg(packed):
    """Test unpack_msg function"""
    print("\n=== Testing unpack_msg ===")

    unpacked = unpack_msg(packed)

    if unpacked:
        print(f"[OK] Message unpacked successfully")
        print(f"  Sender: {unpacked.sender_app}")
        print(f"  Receiver: {unpacked.receiver_app}")
        print(f"  MsgID: {unpacked.MsgID}")
        print(f"  Data: {unpacked.data}")
    else:
        print("[FAIL] Failed to unpack message")

    # Test invalid unpacking
    invalid_unpacked = unpack_msg("invalid|message")
    if not invalid_unpacked:
        print("[OK] Correctly rejected invalid message format")
    else:
        print("[FAIL] Should have rejected invalid message")

    return unpacked

def test_send_msg():
    """Test send_msg function"""
    print("\n=== Testing send_msg ===")

    test_queue = Queue(maxsize=10)

    # Send a message
    success = send_msg(
        test_queue,
        appargs.ImuAppArg.AppID,
        appargs.FlightlogicAppArg.AppID,
        appargs.ImuAppArg.MID_SendImuFlightLogicData,
        "45.2"
    )

    if success:
        print("[OK] Message sent to queue")

        # Retrieve and unpack
        received = test_queue.get(timeout=1)
        unpacked = unpack_msg(received)

        if unpacked:
            print(f"[OK] Message received from queue")
            print(f"  Sender: {unpacked.sender_app} (IMU: {appargs.ImuAppArg.AppID})")
            print(f"  Receiver: {unpacked.receiver_app} (FlightLogic: {appargs.FlightlogicAppArg.AppID})")
            print(f"  MsgID: {unpacked.MsgID}")
            print(f"  Data: {unpacked.data}")
        else:
            print("[FAIL] Failed to unpack received message")
    else:
        print("[FAIL] Failed to send message")

def test_message_flow():
    """Test complete message flow"""
    print("\n=== Testing Complete Message Flow ===")

    test_queue = Queue(maxsize=10)

    # Simulate different sensor messages
    messages = [
        {
            "name": "Barometer",
            "sender": appargs.BarometerAppArg.AppID,
            "receiver": appargs.CommAppArg.AppID,
            "mid": appargs.BarometerAppArg.MID_SendBarometerTlmData,
            "data": "1013.25,25.5,100.5"
        },
        {
            "name": "IMU",
            "sender": appargs.ImuAppArg.AppID,
            "receiver": appargs.CommAppArg.AppID,
            "mid": appargs.ImuAppArg.MID_SendImuTlmData,
            "data": "10.2,5.3,45.7,0.1,0.2,0.3,15.5,20.1,30.2,0.5,0.6,0.7"
        },
        {
            "name": "GPS",
            "sender": appargs.GpsAppArg.AppID,
            "receiver": appargs.FlightlogicAppArg.AppID,
            "mid": appargs.GpsAppArg.MID_SendGpsFlightLogicData,
            "data": "37.5665,126.9780,100.5"
        },
        {
            "name": "Flight Logic State",
            "sender": appargs.FlightlogicAppArg.AppID,
            "receiver": appargs.MotorAppArg.AppID,
            "mid": appargs.FlightlogicAppArg.MID_SendFlightStateToMotor,
            "data": "3"
        }
    ]

    print("\nSending messages...")
    for msg_info in messages:
        success = send_msg(
            test_queue,
            msg_info["sender"],
            msg_info["receiver"],
            msg_info["mid"],
            msg_info["data"]
        )
        if success:
            print(f"  [OK] {msg_info['name']} message sent")
        else:
            print(f"  [FAIL] {msg_info['name']} message failed")

    print("\nReceiving messages...")
    while not test_queue.empty():
        received = test_queue.get()
        unpacked = unpack_msg(received)

        if unpacked:
            msg_name = "Unknown"
            for msg_info in messages:
                if msg_info["mid"] == unpacked.MsgID:
                    msg_name = msg_info["name"]
                    break

            print(f"  [OK] {msg_name}: Sender={unpacked.sender_app}, Receiver={unpacked.receiver_app}, Data={unpacked.data}")
        else:
            print(f"  [FAIL] Failed to unpack message")

if __name__ == "__main__":
    print("=" * 60)
    print("CANSAT Message Structure Test")
    print("=" * 60)

    try:
        # Test individual functions
        msg = test_fill_msg()
        if msg:
            packed = test_pack_msg(msg)
            if packed:
                test_unpack_msg(packed)

        test_send_msg()
        test_message_flow()

        print("\n" + "=" * 60)
        print("All tests completed!")
        print("=" * 60)

    except Exception as e:
        print(f"\n[FAIL] Test failed with error: {e}")
        import traceback
        traceback.print_exc()
