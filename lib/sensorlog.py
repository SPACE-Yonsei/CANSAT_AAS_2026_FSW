"""CSV logger for IPC sensor/telemetry messages observed by main router."""

from __future__ import annotations

import csv
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

from lib import appargs

_fp: Optional[Any] = None
_writer: Optional[csv.writer] = None

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


def init_sensorlog_main_process() -> None:
    global _fp, _writer
    if _writer is not None:
        return
    directory = Path("sensorlogs")
    directory.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = directory / f"sensor_ipc_{ts}.csv"
    _fp = path.open("a", encoding="utf-8", newline="")
    _writer = csv.writer(_fp)
    _writer.writerow(
        [
            "timestamp",
            "sender_app",
            "sender_name",
            "receiver_app",
            "receiver_name",
            "msg_id",
            "data",
        ]
    )
    _fp.flush()


def log_bus_message(msg) -> None:
    """Append one unpacked bus message into sensor IPC CSV."""
    if _writer is None or _fp is None:
        return
    _writer.writerow(
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
    _fp.flush()


def shutdown_sensorlog() -> None:
    global _fp, _writer
    if _fp is None:
        return
    try:
        _fp.flush()
        _fp.close()
    except Exception:
        pass
    _fp = None
    _writer = None
