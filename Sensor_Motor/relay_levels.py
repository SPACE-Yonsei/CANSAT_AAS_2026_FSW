"""Common logic levels for onboard relay modules (burnwire + egg use the same board type)."""

import RPi.GPIO as GPIO

# High-level trigger IN: GPIO HIGH = coil/on, GPIO LOW = idle/off
RELAY_ACTIVATE_LEVEL = GPIO.HIGH
RELAY_DEACTIVATE_LEVEL = GPIO.LOW
