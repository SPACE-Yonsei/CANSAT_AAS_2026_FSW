"""CSV logger for IPC sensor/telemetry messages observed by main router."""

from __future__ import annotations

import csv
import re
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

from lib import appargs

_writers: Dict[int, Tuple[Any, csv.writer]] = {}
_session_dir: Optional[Path] = None

_APP_NAMES = {
    appargs.MainAppArg.AppID: appargs.MainAppArg.AppName,
    appargs.FlightlogicAppArg.AppID: appargs.FlightlogicAppArg.AppName,
    appargs.CommAppArg.AppID: appargs.CommAppArg.AppName,
    appargs.BarometerAppArg.AppID: appargs.BarometerAppArg.AppName,
    appargs.ImuAppArg.AppID: appargs.ImuAppArg.AppName,
    appargs.GpsAppArg.AppID: appargs.GpsAppArg.AppName,
    appargs.DistanceAppArg.AppID: appargs.DistanceAppArg.AppName,
    appargs.ElectroAppArg.AppID: appargs.ElectroAppArg.AppName,
    appargs.CameraAppArg.AppID: appargs.CameraAppArg.AppName,
    appargs.MotorAppArg.AppID: appargs.MotorAppArg.AppName,
}


def _name(app_id: int) -> str:
    return _APP_NAMES.get(int(app_id), f"App{app_id}")


def _safe_filename(label: str) -> str:
    """ASCII 파일명에 안전하게 줄인 뒤 .csv 접미사용 베이스 이름."""
    s = re.sub(r"[^A-Za-z0-9._-]+", "_", label.strip())
    return s or "unknown"


def _header_row() -> list[str]:
    return [
        "timestamp",
        "sender_app",
        "sender_name",
        "receiver_app",
        "receiver_name",
        "msg_id",
        "data",
    ]


def _get_writer(sender_app: int) -> csv.writer:
    global _writers, _session_dir
    sender_app = int(sender_app)
    if sender_app in _writers:
        return _writers[sender_app][1]
    if _session_dir is None:
        raise RuntimeError("sensorlog not initialized")

    base = _safe_filename(_name(sender_app))
    path = _session_dir / f"{base}.csv"
    fp = path.open("a", encoding="utf-8", newline="")
    writer = csv.writer(fp)
    writer.writerow(_header_row())
    _writers[sender_app] = (fp, writer)
    fp.flush()
    return writer


def init_sensorlog_main_process() -> None:
    """Prepare session directory; CSV files are created per sender on first message."""
    global _session_dir
    if _session_dir is not None:
        return
    root = Path("sensorlogs")
    root.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    _session_dir = root / f"run_{ts}"
    _session_dir.mkdir(parents=True, exist_ok=True)


def log_bus_message(msg) -> None:
    """Append one unpacked bus message into the CSV for msg.sender_app."""
    if _session_dir is None:
        return
    writer = _get_writer(msg.sender_app)
    writer.writerow(
        [
            datetime.now().isoformat(timespec="milliseconds"),
            int(msg.sender_app),
            _name(msg.sender_app),
            int(msg.receiver_app),
            _name(msg.receiver_app),
            int(msg.msg_id),
            str(msg.data),
        ]
    )
    fp = _writers[int(msg.sender_app)][0]
    fp.flush()


def shutdown_sensorlog() -> None:
    global _writers, _session_dir
    for fp, _ in list(_writers.values()):
        try:
            fp.flush()
            fp.close()
        except Exception:
            pass
    _writers.clear()
    _session_dir = None
