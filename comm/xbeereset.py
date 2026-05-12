"""XBee reset pulse abstraction."""


def send_reset_pulse() -> None:
    import os
    import time

    pin = int(os.environ.get("XBEE_RESET_GPIO", "18"))
    low_sec = float(os.environ.get("XBEE_RESET_LOW_SEC", "0.1"))

    try:
        import pigpio  # type: ignore
    except Exception:
        return

    pi = pigpio.pi()
    if not getattr(pi, "connected", False):
        return
    try:
        # Floating -> low pulse -> floating (open-drain style)
        pi.set_mode(pin, pigpio.INPUT)
        pi.set_pull_up_down(pin, pigpio.PUD_UP)
        pi.set_mode(pin, pigpio.OUTPUT)
        pi.write(pin, 0)
        time.sleep(max(0.01, low_sec))
        pi.set_mode(pin, pigpio.INPUT)
        pi.set_pull_up_down(pin, pigpio.PUD_UP)
    finally:
        pi.stop()
