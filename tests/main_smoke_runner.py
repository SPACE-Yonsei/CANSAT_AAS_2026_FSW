"""Standalone smoke runner for main orchestrator.

Runs full process startup path from `main.py`, keeps it alive briefly,
then requests graceful shutdown and exits.
"""

from __future__ import annotations

from pathlib import Path
import sys
import threading
import time

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import main as fsw_main


def _request_shutdown() -> None:
    # Flip run flag and wake queue consumer so runloop can exit cleanly.
    fsw_main.MAINAPP_RUNSTATUS = False
    fsw_main.main_queue.put("invalid-smoke-message")


def run() -> int:
    for app_id in fsw_main.app_dict:
        fsw_main.app_dict[app_id].process.start()

    monitor_thread = threading.Thread(
        target=fsw_main.process_monitor, name="ProcessMonitor", daemon=True
    )
    monitor_thread.start()

    timer = threading.Timer(3.0, _request_shutdown)
    timer.daemon = True
    timer.start()

    try:
        fsw_main.runloop(fsw_main.main_queue)
    except SystemExit:
        # terminate_FSW() calls sys.exit(); this is expected for smoke pass.
        return 0
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
