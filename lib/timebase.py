"""Shared monotonic time helpers for control and sensor freshness checks."""

from __future__ import annotations

import math
import time
from typing import Optional

FUTURE_TOLERANCE_S = 0.02


def now() -> float:
    """Return the monotonic time used for control, sensor age, and DR math."""
    return time.monotonic()


def wall_now() -> float:
    """Return wall-clock seconds. Use only for logs or display."""
    return time.time()


def age(now_s: float, timestamp_s: Optional[float]) -> float:
    """Return now_s - timestamp_s, or inf when either side is not finite."""
    try:
        now_f = float(now_s)
        ts_f = float(timestamp_s)
    except (TypeError, ValueError):
        return math.inf
    if not math.isfinite(now_f) or not math.isfinite(ts_f):
        return math.inf
    return now_f - ts_f


def nonnegative_age(now_s: float, timestamp_s: Optional[float]) -> float:
    """Return age clamped at zero for propagation intervals."""
    age_s = age(now_s, timestamp_s)
    if not math.isfinite(age_s):
        return math.inf
    return max(0.0, age_s)


def valid_age(
    timestamp_s: Optional[float],
    now_s: float,
    max_age_s: float,
    future_tolerance_s: float = FUTURE_TOLERANCE_S,
) -> bool:
    """Return True when timestamp_s is finite and within the allowed age window."""
    age_s = age(now_s, timestamp_s)
    if not math.isfinite(age_s):
        return False
    return -float(future_tolerance_s) <= age_s <= float(max_age_s)


def clamp_dt(
    now_s: float,
    previous_s: Optional[float],
    default_s: float,
    min_s: float,
    max_s: float,
) -> float:
    """Return a bounded positive dt for discrete control updates."""
    try:
        prev_f = float(previous_s)
    except (TypeError, ValueError):
        return float(default_s)
    if prev_f <= 0.0 or not math.isfinite(prev_f):
        return float(default_s)
    dt = nonnegative_age(now_s, prev_f)
    if not math.isfinite(dt):
        return float(default_s)
    return max(float(min_s), min(float(max_s), dt))
