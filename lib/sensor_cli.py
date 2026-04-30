"""Helpers for ``python3 -m Sensor_*.<driver>`` streaming CLI loops."""

from __future__ import annotations

import os


def cli_period_sec() -> float:
    """Sleep between samples; override with ``SENSOR_CLI_HZ`` (default 10 Hz)."""
    hz = float(os.environ.get("SENSOR_CLI_HZ", "10"))
    if hz <= 0:
        hz = 10.0
    return 1.0 / hz
