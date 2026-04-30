"""Lightweight multiprocessing-safe logging helpers."""

from __future__ import annotations

from enum import Enum
import logging
from logging.handlers import QueueHandler, QueueListener
from multiprocessing import Queue
from typing import Optional


class EventType(Enum):
    error = "error"
    warning = "warning"
    info = "info"
    debug = "debug"


_listener: Optional[QueueListener] = None
_initialized = False


def _ensure_root_logger() -> logging.Logger:
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)
    if not logger.handlers:
        handler = logging.StreamHandler()
        handler.setFormatter(
            logging.Formatter("%(asctime)s [%(levelname)s] %(name)s: %(message)s")
        )
        logger.addHandler(handler)
    return logger


def init_events_main_process() -> Queue:
    """Initialize queue logging for main process and return queue."""
    global _listener, _initialized

    log_queue: Queue = Queue()
    root = _ensure_root_logger()
    _listener = QueueListener(log_queue, *root.handlers, respect_handler_level=True)
    _listener.start()
    _initialized = True
    return log_queue


def init_events_subprocess(log_queue: Queue) -> None:
    """Attach subprocess logger to the main process queue."""
    root = logging.getLogger()
    root.setLevel(logging.INFO)
    root.handlers.clear()
    root.addHandler(QueueHandler(log_queue))


def shutdown_events() -> None:
    global _listener
    if _listener is not None:
        _listener.stop()
        _listener = None


def LogEvent(app_name: str, event_type: EventType, event_msg: str) -> None:
    """Uniform logging interface used across apps."""
    _ensure_root_logger()
    logger = logging.getLogger(app_name)
    msg = str(event_msg)

    if event_type == EventType.error:
        logger.error(msg)
    elif event_type == EventType.warning:
        logger.warning(msg)
    elif event_type == EventType.debug:
        logger.debug(msg)
    else:
        logger.info(msg)
