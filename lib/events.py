# events.py
# Author: Hyeon Lee (Refactored for multiprocessing)
# Event logging interface using Python's logging module

import logging
from lib import logging as fsw_logging

class EventType:
    """이벤트 타입 상수"""
    error = 0
    info = 1
    debug = 2
    warning = 3


def init_events_main_process():
    """
    Main process에서 호출. 로깅 시스템 초기화.
    Returns log_queue to pass to subprocesses.
    """
    log_queue = fsw_logging.setup_logging_main_process()
    
    # Log initialization
    logger = fsw_logging.get_logger('MAIN')
    logger.info("=== FSW Logging System Initialized ===")
    
    return log_queue


def init_events_subprocess(log_queue):
    """
    Subprocess에서 호출. 로깅 시스템 연결.
    """
    fsw_logging.setup_logging_subprocess(log_queue)


def shutdown_events():
    """
    Main process 종료 시 호출. 로깅 시스템 정리.
    """
    logger = fsw_logging.get_logger('MAIN')
    logger.info("=== FSW Logging System Shutdown ===")
    fsw_logging.shutdown_logging()


def LogEvent(app_name: str, event_type: int, event_msg: str, print_event=True):
    """
    이벤트 로깅 함수.
    
    Args:
        app_name: 앱 이름 (logger name으로 사용)
        event_type: EventType.error, EventType.info, EventType.debug
        event_msg: 로그 메시지
        print_event: 콘솔 출력 여부 (현재는 handler 레벨에서 제어됨)
    """
    logger = fsw_logging.get_logger(app_name)
    
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
