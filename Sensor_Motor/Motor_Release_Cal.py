"""Predict release timing with burnwire delay compensation.

This module estimates descent rate after apogee and triggers burnwire early so
that physical separation occurs near target altitude ratio (default: 80%).
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Deque, Optional, Tuple

from lib import config


_CONFIG_TXT = Path(__file__).resolve().parents[1] / "lib" / "config.txt"


def _read_config_txt_value(*keys: str) -> Optional[float]:
    """Read the first numeric value from lib/config.txt.

    Supported format: KEY=VALUE
    Lines starting with # are ignored.
    """
    if not _CONFIG_TXT.exists():
        return None
    key_set = {k.strip().upper() for k in keys if k.strip()}
    if not key_set:
        return None
    try:
        for raw in _CONFIG_TXT.read_text(encoding="utf-8").splitlines():
            line = raw.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            k, v = line.split("=", 1)
            if k.strip().upper() not in key_set:
                continue
            return float(v.strip())
    except (OSError, ValueError):
        return None
    return None


def get_burnwire_delay_sec() -> float:
    """Get burnwire heat-up delay for release prediction.

    Priority:
      1) lib/config.txt: RELEASE_BURNWIRE_DELAY_SEC or BURNWIRE_DELAY_SEC
      2) lib/config.py: RELEASE_BURNWIRE_DELAY_SEC
      3) fallback default: 3.0
    """
    value = _read_config_txt_value("RELEASE_BURNWIRE_DELAY_SEC", "BURNWIRE_DELAY_SEC")
    if value is None:
        value = getattr(config, "RELEASE_BURNWIRE_DELAY_SEC", 3.0)
    try:
        return max(0.1, float(value))
    except (TypeError, ValueError):
        return 3.0


def get_release_target_ratio() -> float:
    value = _read_config_txt_value("RELEASE_TARGET_RATIO")
    if value is None:
        value = getattr(config, "RELEASE_TARGET_RATIO", 0.8)
    try:
        return min(0.99, max(0.5, float(value)))
    except (TypeError, ValueError):
        return 0.8


def get_release_predict_start_ratio() -> float:
    """Fraction of max_alt below which prediction sampling and force-timeout arm.

    ``lib/config.txt`` may set ``RELEASE_PREDICT_START_RATIO``; the legacy key
    ``RELEASE_FORCE_START_RATIO`` is accepted as an alias when the primary key
    is absent.
    """
    value = _read_config_txt_value("RELEASE_PREDICT_START_RATIO")
    if value is None:
        value = _read_config_txt_value("RELEASE_FORCE_START_RATIO")
    if value is None:
        value = getattr(config, "RELEASE_PREDICT_START_RATIO", 0.9)
    try:
        return min(0.99, max(0.5, float(value)))
    except (TypeError, ValueError):
        return 0.9


def get_release_hard_trigger_ratio() -> float:
    value = _read_config_txt_value(
        "RELEASE_HARD_TRIGGER_RATIO",
        "RELEASE_FORCE_TRIGGER_RATIO",
    )
    if value is None:
        value = getattr(config, "RELEASE_HARD_TRIGGER_RATIO", 0.85)
    try:
        return min(0.99, max(0.5, float(value)))
    except (TypeError, ValueError):
        return 0.85


@dataclass
class ReleasePredictorState:
    history: Deque[Tuple[float, float]] = field(default_factory=lambda: deque(maxlen=40))
    trigger_latched: bool = False
    crossed_90pct_at_s: Optional[float] = None


def reset_release_predictor(state: ReleasePredictorState) -> None:
    state.history.clear()
    state.trigger_latched = False
    state.crossed_90pct_at_s = None


def get_release_prediction_time_min_sec() -> float:
    value = _read_config_txt_value("RELEASE_PREDICT_TIME_MIN_SEC")
    if value is None:
        value = getattr(config, "RELEASE_PREDICT_TIME_MIN_SEC", 0.0)
    try:
        return max(0.0, float(value))
    except (TypeError, ValueError):
        return 0.0


def get_release_prediction_time_max_sec() -> float:
    value = _read_config_txt_value("RELEASE_PREDICT_TIME_MAX_SEC")
    if value is None:
        value = getattr(config, "RELEASE_PREDICT_TIME_MAX_SEC", 5.0)
    try:
        return max(0.1, float(value))
    except (TypeError, ValueError):
        return 5.0


def get_release_force_start_ratio() -> float:
    """Same band as ``get_release_predict_start_ratio`` (single tunable in ``config.py``)."""
    return get_release_predict_start_ratio()


def get_release_force_after_sec() -> float:
    value = _read_config_txt_value("RELEASE_FORCE_AFTER_SEC")
    if value is None:
        value = getattr(config, "RELEASE_FORCE_AFTER_SEC", 5.0)
    try:
        return max(0.1, float(value))
    except (TypeError, ValueError):
        return 5.0


@dataclass
class ReleaseDecision:
    trigger: bool
    reason: str = "NONE"
    predicted: bool = False
    time_to_target_s: Optional[float] = None


def _descent_rate_mps(history: Deque[Tuple[float, float]]) -> float:
    """Estimate positive descent rate (m/s) from history window."""
    if len(history) < 2:
        return 0.0
    t0, a0 = history[0]
    t1, a1 = history[-1]
    dt = max(1e-3, t1 - t0)
    rate = (a0 - a1) / dt
    return rate if rate > 0.0 else 0.0


def should_trigger_release(
    predictor: ReleasePredictorState,
    *,
    now_s: float,
    alt_m: float,
    max_alt_m: float,
    target_ratio: Optional[float] = None,
    predict_start_ratio: Optional[float] = None,
    hard_trigger_ratio: Optional[float] = None,
    burnwire_delay_sec: Optional[float] = None,
    force_start_ratio: Optional[float] = None,
    force_after_sec: Optional[float] = None,
    min_desc_rate_mps: float = 0.3,
    min_samples: int = 3,
) -> ReleaseDecision:
    """Return True when burnwire should be fired now.

    Logic:
      - Start prediction only after descending through max_alt*predict_start_ratio
      - Estimate descent rate from recent altitude history
      - Predict remaining time to target altitude (max_alt*target_ratio)
      - Trigger when predicted remaining time <= burnwire_delay_sec
      - Latch trigger after first True to avoid oscillation
    """
    if predictor.trigger_latched:
        return ReleaseDecision(True, reason="LATCHED", predicted=True, time_to_target_s=0.0)
    if max_alt_m <= 0.0:
        return ReleaseDecision(False)

    if target_ratio is None:
        target_ratio = get_release_target_ratio()
    if predict_start_ratio is None:
        predict_start_ratio = get_release_predict_start_ratio()
    if hard_trigger_ratio is None:
        hard_trigger_ratio = get_release_hard_trigger_ratio()
    predict_t_min = get_release_prediction_time_min_sec()
    predict_t_max = get_release_prediction_time_max_sec()
    if force_start_ratio is None:
        force_start_ratio = get_release_force_start_ratio()
    if force_after_sec is None:
        force_after_sec = get_release_force_after_sec()
    if predict_t_max < predict_t_min:
        predict_t_min, predict_t_max = predict_t_max, predict_t_min

    target_ratio = float(target_ratio)
    predict_start_ratio = float(predict_start_ratio)
    hard_trigger_ratio = float(hard_trigger_ratio)
    force_start_ratio = float(force_start_ratio)

    # Keep ordering safe: predict_start >= hard_trigger >= target.
    predict_start_ratio = max(target_ratio, predict_start_ratio)
    hard_trigger_ratio = min(predict_start_ratio, max(target_ratio, hard_trigger_ratio))

    burn_delay = get_burnwire_delay_sec() if burnwire_delay_sec is None else burnwire_delay_sec
    burn_delay = max(0.1, float(burn_delay))

    # Absolute fallback: if altitude falls below max_alt * predict band ratio,
    # force release after timeout (default 5s).
    force_start_alt_m = max_alt_m * force_start_ratio
    if alt_m <= force_start_alt_m:
        if predictor.crossed_90pct_at_s is None:
            predictor.crossed_90pct_at_s = now_s
        elif now_s - predictor.crossed_90pct_at_s >= force_after_sec:
            predictor.trigger_latched = True
            return ReleaseDecision(True, reason="FORCE_90PCT_TIMEOUT", predicted=False)
    else:
        predictor.crossed_90pct_at_s = None

    # Always keep a short history around the prediction band.
    if alt_m <= max_alt_m * predict_start_ratio:
        predictor.history.append((now_s, alt_m))
    elif predictor.history:
        predictor.history.clear()
        predictor.history.append((now_s, alt_m))

    target_alt_m = max_alt_m * target_ratio
    if alt_m <= target_alt_m:
        predictor.trigger_latched = True
        return ReleaseDecision(True, reason="TARGET_RATIO", predicted=False, time_to_target_s=0.0)

    if len(predictor.history) < min_samples:
        hard_alt_m = max_alt_m * hard_trigger_ratio
        if alt_m < hard_alt_m and alt_m <= max_alt_m * predict_start_ratio:
            predictor.trigger_latched = True
            return ReleaseDecision(True, reason="FORCE_85_NO_PREDICTION", predicted=False)
        return ReleaseDecision(False)

    rate_mps = _descent_rate_mps(predictor.history)
    if rate_mps < min_desc_rate_mps:
        hard_alt_m = max_alt_m * hard_trigger_ratio
        if alt_m < hard_alt_m and alt_m <= max_alt_m * predict_start_ratio:
            predictor.trigger_latched = True
            return ReleaseDecision(True, reason="FORCE_85_NO_PREDICTION", predicted=False)
        return ReleaseDecision(False)

    remaining_m = alt_m - target_alt_m
    time_to_target_s = remaining_m / max(rate_mps, 1e-6)
    prediction_valid = predict_t_min <= time_to_target_s <= predict_t_max
    if prediction_valid and time_to_target_s <= burn_delay:
        predictor.trigger_latched = True
        return ReleaseDecision(
            True,
            reason="PREDICTIVE",
            predicted=True,
            time_to_target_s=time_to_target_s,
        )
    if (not prediction_valid) and (alt_m < max_alt_m * hard_trigger_ratio):
        predictor.trigger_latched = True
        return ReleaseDecision(True, reason="FORCE_85_NO_PREDICTION", predicted=False)
    return ReleaseDecision(False, predicted=prediction_valid, time_to_target_s=time_to_target_s)
