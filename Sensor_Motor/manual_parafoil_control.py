#!/usr/bin/env python3
"""
Quick manual controller for the parafoil motors (GPIO 12/13).

Usage:
    python Sensor_Motor/manual_parafoil_control.py

Commands at runtime:
    l <pulse>   -> set left motor pulse (1400-2400)
    r <pulse>   -> set right motor pulse (1400-2400)
    n           -> set both motors to neutral (1500)
    q           -> quit program
"""

import pigpio

LEFT_PIN = 12
RIGHT_PIN = 13

NEUTRAL = 1500
PULSE_MIN = 1400
PULSE_MAX = 2400


def clamp(value: int) -> int:
    return max(PULSE_MIN, min(PULSE_MAX, value))


def set_pulse(pi: pigpio.pi, pin: int, pulse: int) -> None:
    pi.set_servo_pulsewidth(pin, clamp(pulse))


def main() -> None:
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemon not running")

    try:
        set_pulse(pi, LEFT_PIN, NEUTRAL)
        set_pulse(pi, RIGHT_PIN, NEUTRAL)

        print("Manual parafoil motor control (GPIO 12/13).")
        print("Commands: 'l <pulse>', 'r <pulse>', 'n', 'q'")

        while True:
            raw = input("> ").strip().lower()
            if not raw:
                continue

            if raw == "q":
                break
            if raw == "n":
                set_pulse(pi, LEFT_PIN, NEUTRAL)
                set_pulse(pi, RIGHT_PIN, NEUTRAL)
                print(f"Both motors set to neutral ({NEUTRAL}µs)")
                continue

            parts = raw.split()
            if len(parts) != 2:
                print("Invalid command. Use 'l 1600', 'r 1800', 'n', or 'q'.")
                continue

            cmd, value_str = parts
            if cmd not in ("l", "r"):
                print("Command must be 'l' or 'r'.")
                continue

            try:
                pulse = int(value_str)
            except ValueError:
                print("Pulse must be an integer.")
                continue

            if cmd == "l":
                set_pulse(pi, LEFT_PIN, pulse)
                print(f"Left motor -> {clamp(pulse)}µs")
            else:
                set_pulse(pi, RIGHT_PIN, pulse)
                print(f"Right motor -> {clamp(pulse)}µs")

    finally:
        set_pulse(pi, LEFT_PIN, NEUTRAL)
        set_pulse(pi, RIGHT_PIN, NEUTRAL)
        pi.stop()


if __name__ == "__main__":
    main()

