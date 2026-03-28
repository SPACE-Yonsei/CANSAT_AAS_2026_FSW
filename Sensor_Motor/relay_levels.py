"""Common logic levels for onboard relay modules (burnwire + egg use the same board type)."""

import RPi.GPIO as GPIO

# Active-low IN: GPIO LOW energizes coil, HIGH = idle/off
RELAY_ACTIVATE_LEVEL = GPIO.LOW
RELAY_DEACTIVATE_LEVEL = GPIO.HIGH
