"""GNSS 7 Click (u-blox) RST line — active-low pulse via GPIO (e.g. MikroE RST → Pi GPIO)."""

from __future__ import annotations

import os
import time


def send_reset_pulse() -> None:
    if os.environ.get("GNSS_RESET_ENABLE", "1").strip() == "0":
        return
    try:
        from lib import config
    except Exception:
        pin = 26
    else:
        pin = int(os.environ.get("GNSS_RESET_GPIO", str(config.GNSS_RESET_GPIO)))
    low_sec = float(os.environ.get("GNSS_RESET_LOW_SEC", "0.1"))

    try:
        import pigpio  # type: ignore
    except Exception:
        return

    pi = pigpio.pi()
    if not getattr(pi, "connected", False):
        return
    try:
        pi.set_mode(pin, pigpio.INPUT)
        pi.set_mode(pin, pigpio.OUTPUT)
        pi.write(pin, 0)
        time.sleep(max(0.01, low_sec))
        pi.set_mode(pin, pigpio.INPUT)
    finally:
        pi.stop()
    post = float(os.environ.get("GNSS_RESET_POST_SEC", "0.15"))
    if post > 0:
        time.sleep(post)
