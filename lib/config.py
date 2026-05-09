"""Core runtime configuration for FSW."""

from __future__ import annotations


# GPIO map
BURNWIRE_GPIO = 5
EGG_SOLENOID_GPIO = 6
PARAFOIL_RIGHT_GPIO = 12
PARAFOIL_LEFT_GPIO = 13
# GNSS 7 Click RST (active low; pulse at GPS app startup unless GNSS_RESET_ENABLE=0)
GNSS_RESET_GPIO = 26

# Egg drop (flight state 4): rangefinder reading must be in
# [rough validity floor .. EGG_STATE_DISTANCE_TRIGGER_MM] to arm solenoid pulses.
# Typical: trigger when distance to ground <= 2500 mm with TF-Luna.
EGG_STATE_DISTANCE_TRIGGER_MM = 2500


# Relay active level (keep numeric for platform-agnostic compatibility)
RELAY_ACTIVATE_LEVEL = 1
RELAY_DEACTIVATE_LEVEL = 0


# Sensor/IO rates (Hz)
BAROMETER_RATE_HZ = 10
IMU_RATE_HZ = 10
GPS_RATE_HZ = 20
MOTOR_RATE_HZ = 20
ELECTRO_RATE_HZ = 1
DISTANCE_RATE_HZ = 10
XBEE_RATE_HZ = 10
CAMERA_FPS = 30


# Release timing tuning
RELEASE_TARGET_RATIO = 0.8  # 80% max_alt: separation altitude target
# Below this fraction of max_alt: start descent-rate history and the FORCE_90PCT_TIMEOUT timer.
RELEASE_PREDICT_START_RATIO = 0.9
RELEASE_HARD_TRIGGER_RATIO = 0.85  # hard fallback if prediction is not viable
RELEASE_BURNWIRE_DELAY_SEC = 3.0
RELEASE_PREDICT_TIME_MIN_SEC = 0.0
RELEASE_PREDICT_TIME_MAX_SEC = 5.0
RELEASE_FORCE_AFTER_SEC = 5.0  # seconds after band crossing before FORCE_90PCT_TIMEOUT