# events.py
# Author: jeong min park
# Event logging: Python logging + QueueHandler, and app-facing API (EventType, LogEvent)

import logging
import logging.handlers
import os
from multiprocessing import Queue
from datetime import datetime

# ---------------------------------------------------------------------------
# Log directory & config
# ---------------------------------------------------------------------------
LOG_DIR = './eventlogs'
SENSOR_LOG_APPS = ['BarometerApp', 'GpsApp', 'ImuApp', 'DistanceApp', 'ElectroApp', 'CameraApp', 'MotorApp']

_log_queue: Queue = None
_queue_listener = None


class SensorLogFilter(logging.Filter):
    """특정 앱 이름만 통과시키는 필터"""
    def __init__(self, app_name: str):
        super().__init__()
        self.app_name = app_name

    def filter(self, record: logging.LogRecord) -> bool:
        return record.name == self.app_name


class ConsoleFilter(logging.Filter):
    """Console handler filter to hide WARNING logs only."""
    def filter(self, record: logging.LogRecord) -> bool:
        return record.levelno != logging.WARNING


def _setup_logging_main_process() -> Queue:
    """Main process에서 호출. 로그 큐와 리스너를 설정. Returns the log queue to pass to subprocesses."""
    global _log_queue, _queue_listener

    if not os.path.exists(LOG_DIR):
        os.makedirs(LOG_DIR)

    _log_queue = Queue(maxsize=5000)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    info_handler = logging.handlers.RotatingFileHandler(
        os.path.join(LOG_DIR, f'info_{timestamp}.log'),
        maxBytes=10*1024*1024,
        backupCount=5,
        encoding='utf-8'
    )
    info_handler.setLevel(logging.INFO)

    error_handler = logging.handlers.RotatingFileHandler(
        os.path.join(LOG_DIR, f'error_{timestamp}.log'),
        maxBytes=10*1024*1024,
        backupCount=5,
        encoding='utf-8'
    )
    error_handler.setLevel(logging.ERROR)

    debug_handler = logging.handlers.RotatingFileHandler(
        os.path.join(LOG_DIR, f'debug_{timestamp}.log'),
        maxBytes=10*1024*1024,
        backupCount=5,
        encoding='utf-8'
    )
    debug_handler.setLevel(logging.DEBUG)

    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.DEBUG)
    console_handler.addFilter(ConsoleFilter())

    formatter = logging.Formatter(
        '[%(asctime)s.%(msecs)03d] %(levelname)s | %(name)s : %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    info_handler.setFormatter(formatter)
    error_handler.setFormatter(formatter)
    debug_handler.setFormatter(formatter)
    console_handler.setFormatter(formatter)

    sensor_handlers = []
    for app_name in SENSOR_LOG_APPS:
        sensor_handler = logging.FileHandler(
            os.path.join(LOG_DIR, f'{app_name.lower()}_{timestamp}.log'),
            encoding='utf-8'
        )
        sensor_handler.setLevel(logging.DEBUG)
        sensor_handler.setFormatter(formatter)
        sensor_handler.addFilter(SensorLogFilter(app_name))
        sensor_handlers.append(sensor_handler)

    _queue_listener = logging.handlers.QueueListener(
        _log_queue,
        info_handler,
        error_handler,
        debug_handler,
        console_handler,
        *sensor_handlers,
        respect_handler_level=True
    )
    _queue_listener.start()

    return _log_queue


def _setup_logging_subprocess(log_queue: Queue):
    """Subprocess에서 호출. QueueHandler를 설정하여 메인 프로세스로 로그 전송."""
    global _log_queue
    _log_queue = log_queue

    root = logging.getLogger()
    root.setLevel(logging.DEBUG)
    root.handlers.clear()
    root.addHandler(logging.handlers.QueueHandler(log_queue))


def _get_logger(name: str) -> logging.Logger:
    return logging.getLogger(name)


def _shutdown_logging():
    """Main process에서 종료 시 호출. QueueListener 정리."""
    global _queue_listener
    if _queue_listener:
        _queue_listener.stop()
        _queue_listener = None


# ---------------------------------------------------------------------------
# App-facing API: EventType, init/shutdown, LogEvent
# ---------------------------------------------------------------------------

class EventType:
    """이벤트 타입 상수"""
    error = 0
    info = 1
    debug = 2
    warning = 3


def init_events_main_process():
    """Main process에서 호출. 로깅 시스템 초기화. Returns log_queue to pass to subprocesses."""
    log_queue = _setup_logging_main_process()
    logger = _get_logger('MAIN')
    logger.info("=== FSW Logging System Initialized ===")
    return log_queue


def init_events_subprocess(log_queue):
    """Subprocess에서 호출. 로깅 시스템 연결."""
    _setup_logging_subprocess(log_queue)


def shutdown_events():
    """Main process 종료 시 호출. 로깅 시스템 정리."""
    logger = _get_logger('MAIN')
    logger.info("=== FSW Logging System Shutdown ===")
    _shutdown_logging()


def LogEvent(app_name: str, event_type: int, event_msg: str, print_event=True):
    """
    이벤트 로깅 함수.
    Args:
        app_name: 앱 이름 (logger name으로 사용)
        event_type: EventType.error, EventType.info, EventType.debug, EventType.warning
        event_msg: 로그 메시지
        print_event: 콘솔 출력 여부 (현재는 handler 레벨에서 제어됨)
    """
    logger = _get_logger(app_name)
    if event_type == EventType.error:
        logger.error(event_msg)
    elif event_type == EventType.info:
        logger.info(event_msg)
    elif event_type == EventType.debug:
        logger.debug(event_msg)
    elif event_type == EventType.warning:
        logger.warning(event_msg)
    else:
        logger.info(event_msg)


# ---------------------------------------------------------------------------
# Optional: get_logger / logdata for code that needs direct logger access
# ---------------------------------------------------------------------------

def get_logger(name: str) -> logging.Logger:
    """Logger 인스턴스 반환 (직접 로깅이 필요한 경우)."""
    return _get_logger(name)


def logdata(target, text: str, printlogs=False):
    """Legacy. 새 코드에서는 LogEvent() 또는 get_logger()를 사용하세요."""
    _get_logger('legacy').info(text)
