from lib import events
from multiprocessing import Queue

class MsgStructure:
    sender_app: int = None # AppID of sender
    receiver_app: int = None # AppID of receiver
    MsgID: int = None # Message ID should be unique for identification
    data: str = None # Data

def fill_msg(_sender : int, _receiver : int, _MsgID : int, _data: str):
    try:
        if '|' in _data:
            events.LogEvent("MsgStructure", events.EventType.error, f"Data should not contain '|' since it is used to divide fields")
            return False
        target = MsgStructure()
        target.sender_app = _sender
        target.receiver_app = _receiver
        target.MsgID = _MsgID
        target.data = _data
        return target
    except Exception as e:
        events.LogEvent("MsgStructure", events.EventType.error, f"error when filling message : {e}")
        return False

def pack_msg (target: MsgStructure) -> str:
    try:
        if target.sender_app == None or target.receiver_app == None or target.MsgID == None or target.data == None:
            events.LogEvent("MsgStructure", events.EventType.error, f"error when packing message : message is not filled")
            return "ERROR"   
        else:
            # Convert data to string if it's not already (handles list, dict, etc.)
            data_str = str(target.data) if not isinstance(target.data, str) else target.data
            return str(target.sender_app) + "|" + str(target.receiver_app) + "|" + str(target.MsgID) + "|" + data_str
    except Exception as e:
        events.LogEvent("MsgStructure", events.EventType.error, f"error when packing message : {e}")
        return "ERROR"
    
def unpack_msg (target: MsgStructure, msg: str) -> bool:
    try:
        msg_list = msg.split('|')
        if len(msg_list) != 4:
            events.LogEvent("MsgStructure", events.EventType.error, f"error when unpacking message : Expected length of msg_list of 4 but {len(msg_list)}")
            return False
        else:
            target.sender_app = int(msg_list[0])
            target.receiver_app = int(msg_list[1])
            target.MsgID = int(msg_list[2])
            target.data = msg_list[3]
            return True
    except Exception as e:
        events.LogEvent("MsgStructure", events.EventType.error, f"error when unpacking message : {e}")
        return False
    
# Send message for SB Methods to route
def send_msg (Main_Queue : Queue, _sender, _receiver, _MsgID, _data: str):
    try:
        # Fill Message
        target = fill_msg(_sender, _receiver, _MsgID, _data)
        #print(f"{target.sender_app} -> {target.receiver_app} : {target.MsgID} / {target.data}")
        # Pack Message
        packed_msg = pack_msg(target)
        if packed_msg == "ERROR":
            return False
        #print(msg_to_send)
        # Put message to main app queue 
        Main_Queue.put(packed_msg)

    except Exception as e:
        events.LogEvent("MsgStructure", events.EventType.error, f"error when sending message : {e}")
        return False
    return True