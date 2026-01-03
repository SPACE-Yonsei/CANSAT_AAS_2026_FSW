# logging.py
# Author: Hyeon Lee (Refactored for multiprocessing)
# Centralized logging system using Python's logging module with QueueHandler

import logging
import logging.handlers
import os
from multiprocessing import Queue
from datetime import datetime

# Log directory
LOG_DIR = './eventlogs'

# Global log queue - will be set by main process
_log_queue: Queue = None
_queue_listener = None

def setup_logging_main_process() -> Queue:
    """
    Main process에서 호출. 로그 큐와 리스너를 설정.
    Returns the log queue to pass to subprocesses.
    """
    global _log_queue, _queue_listener
    
    # Create log directory
    if not os.path.exists(LOG_DIR):
        os.makedirs(LOG_DIR)
    
    # Create the queue
    _log_queue = Queue()
    
    # Create file handlers for different log levels
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Info handler - INFO level and above
    info_handler = logging.FileHandler(
        os.path.join(LOG_DIR, f'info_{timestamp}.log'),
        encoding='utf-8'
    )
    info_handler.setLevel(logging.INFO)
    
    # Error handler - ERROR level only
    error_handler = logging.FileHandler(
        os.path.join(LOG_DIR, f'error_{timestamp}.log'),
        encoding='utf-8'
    )
    error_handler.setLevel(logging.ERROR)
    
    # Debug handler - all levels
    debug_handler = logging.FileHandler(
        os.path.join(LOG_DIR, f'debug_{timestamp}.log'),
        encoding='utf-8'
    )
    debug_handler.setLevel(logging.DEBUG)
    
    # Console handler - DEBUG 레벨부터 모두 출력
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.DEBUG)
    
    # Formatter
    formatter = logging.Formatter(
        '[%(asctime)s.%(msecs)03d] %(levelname)s | %(name)s : %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    info_handler.setFormatter(formatter)
    error_handler.setFormatter(formatter)
    debug_handler.setFormatter(formatter)
    console_handler.setFormatter(formatter)
    
    # QueueListener - listens to queue and dispatches to handlers
    _queue_listener = logging.handlers.QueueListener(
        _log_queue,
        info_handler,
        error_handler,
        debug_handler,
        console_handler,
        respect_handler_level=True
    )
    _queue_listener.start()
    
    return _log_queue


def setup_logging_subprocess(log_queue: Queue):
    """
    Subprocess에서 호출. QueueHandler를 설정하여 메인 프로세스로 로그 전송.
    """
    global _log_queue
    _log_queue = log_queue
    
    # Get root logger
    root = logging.getLogger()
    root.setLevel(logging.DEBUG)
    
    # Remove any existing handlers
    root.handlers.clear()
    
    # Add queue handler
    queue_handler = logging.handlers.QueueHandler(log_queue)
    root.addHandler(queue_handler)


def get_logger(name: str) -> logging.Logger:
    """
    지정된 이름의 logger를 반환.
    """
    return logging.getLogger(name)


def shutdown_logging():
    """
    Main process에서 종료 시 호출. QueueListener 정리.
    """
    global _queue_listener
    if _queue_listener:
        _queue_listener.stop()
        _queue_listener = None


# Legacy support - 기존 코드 호환용
def logdata(target, text: str, printlogs=False):
    """
    Legacy function for backward compatibility.
    새 코드에서는 get_logger()를 사용하세요.
    """
    logger = get_logger('legacy')
    logger.info(text)
