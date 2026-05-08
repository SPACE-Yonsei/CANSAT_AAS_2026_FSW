"""Lightweight multiprocessing-safe logging helpers."""

from __future__ import annotations

import csv
from enum import Enum
from datetime import datetime
import logging
from logging.handlers import QueueHandler, QueueListener
from multiprocessing import Queue
from pathlib import Path
from threading import Lock
from typing import Optional


class EventType(Enum):
    error = "error"
    warning = "warning"
    info = "info"
    debug = "debug"


_listener: Optional[QueueListener] = None
_initialized = False


class _CsvEventHandler(logging.Handler):
    """CSV sink for all event logs across processes."""

    def __init__(self) -> None:
        super().__init__(level=logging.DEBUG)
        self._lock = Lock()
        self._fp = None
        self._writer = None
        self._path = None
        self._open_csv()

    def _open_csv(self) -> None:
        directory = Path("eventlogs")
        directory.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._path = directory / f"events_{ts}.csv"
        self._fp = self._path.open("a", encoding="utf-8", newline="")
        self._writer = csv.writer(self._fp)
        self._writer.writerow(
            [
                "timestamp",
                "level",
                "logger",
                "process",
                "thread",
                "message",
            ]
        )
        self._fp.flush()

    def emit(self, record: logging.LogRecord) -> None:
        if self._writer is None or self._fp is None:
            return
        msg = record.getMessage()
        row = [
            datetime.fromtimestamp(record.created).isoformat(timespec="milliseconds"),
            record.levelname,
            record.name,
            record.processName,
            record.threadName,
            msg,
        ]
        with self._lock:
            self._writer.writerow(row)
            self._fp.flush()

    def close(self) -> None:
        with self._lock:
            if self._fp is not None:
                try:
                    self._fp.flush()
                    self._fp.close()
                except Exception:
                    pass
                self._fp = None
                self._writer = None
        super().close()


def _ensure_root_logger() -> logging.Logger:
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)
    if not logger.handlers:
        stream_handler = logging.StreamHandler()
        stream_handler.setLevel(logging.INFO)
        stream_handler.setFormatter(
            logging.Formatter("%(asctime)s [%(levelname)s] %(name)s: %(message)s")
        )
        csv_handler = _CsvEventHandler()
        csv_handler.setLevel(logging.DEBUG)
        logger.addHandler(stream_handler)
        logger.addHandler(csv_handler)
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
    root.setLevel(logging.DEBUG)
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
