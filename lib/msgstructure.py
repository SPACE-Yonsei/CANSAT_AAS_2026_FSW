"""Message envelope helpers for Queue/Pipe routing.

FSW bus message format:
    sender|receiver|MsgID|data
"""

from __future__ import annotations

from dataclasses import dataclass
import logging
import os
import queue
from multiprocessing.queues import Queue
from typing import Optional, Union


logger = logging.getLogger(__name__)

_MSG_QUEUE_PUT_TIMEOUT_SEC = float(os.environ.get("FSW_MSG_QUEUE_PUT_TIMEOUT_SEC", "0.05"))
_MSG_QUEUE_DROP_WHEN_FULL = os.environ.get("FSW_MSG_QUEUE_DROP_WHEN_FULL", "1").strip() != "0"


@dataclass
class MsgStructure:
    sender_app: int
    receiver_app: int
    msg_id: int
    data: str


def _to_int(value: Union[int, str], field_name: str) -> Optional[int]:
    try:
        return int(value)
    except (TypeError, ValueError):
        logger.error("Invalid %s: %r", field_name, value)
        return None


def fill_msg(
    sender: Union[int, str],
    receiver: Union[int, str],
    msg_id: Union[int, str],
    data: object,
) -> Optional[MsgStructure]:
    """Build a message object with basic validation."""
    sender_i = _to_int(sender, "sender")
    receiver_i = _to_int(receiver, "receiver")
    msg_i = _to_int(msg_id, "msg_id")
    data_s = "" if data is None else str(data)

    if sender_i is None or receiver_i is None or msg_i is None:
        return None

    if "|" in data_s:
        logger.error("Message data must not include '|' delimiter")
        return None

    return MsgStructure(
        sender_app=sender_i,
        receiver_app=receiver_i,
        msg_id=msg_i,
        data=data_s,
    )


def pack_msg(target: MsgStructure) -> str:
    """Serialize message object to bus string.

    Returns "ERROR" when target is invalid.
    """
    if not isinstance(target, MsgStructure):
        logger.error("pack_msg expects MsgStructure, got %r", type(target))
        return "ERROR"

    return f"{target.sender_app}|{target.receiver_app}|{target.msg_id}|{target.data}"


def unpack_msg(msg: object) -> Union[MsgStructure, bool]:
    """Deserialize bus string to message object.

    Returns False when parsing fails.
    """
    if not isinstance(msg, str):
        logger.error("unpack_msg expects str, got %r", type(msg))
        return False

    parts = msg.split("|")
    if len(parts) != 4:
        logger.error("Invalid message field count: %d", len(parts))
        return False

    unpacked = fill_msg(parts[0], parts[1], parts[2], parts[3])
    if unpacked is None:
        return False
    return unpacked


def send_msg(
    main_queue: Queue,
    sender: Union[int, str],
    receiver: Union[int, str],
    msg_id: Union[int, str],
    data: object,
) -> bool:
    """Create and enqueue a message in one step."""
    target = fill_msg(sender, receiver, msg_id, data)
    if target is None:
        return False

    packed = pack_msg(target)
    if packed == "ERROR":
        return False

    try:
        if _MSG_QUEUE_DROP_WHEN_FULL:
            main_queue.put(packed, timeout=max(0.0, _MSG_QUEUE_PUT_TIMEOUT_SEC))
        else:
            main_queue.put(packed)
    except queue.Full:
        logger.warning("Message queue full; dropping frame sender=%s receiver=%s mid=%s", sender, receiver, msg_id)
        return False
    except Exception as exc:  # pragma: no cover - queue backend specific
        logger.error("Failed to queue message: %s", exc)
        return False
    return True
