#!/usr/bin/env python3
"""
Parafoil Actuator Control Module
=================================

Converts commanded yaw rate into physical servo commands for left/right
parafoil brake toggles.

Architecture (two stages):
  1. Actuator mixer  - commanded_yaw_rate [deg/s] -> left/right arm angles [deg]
  2. Servo mapping   - arm angles [deg] -> PWM pulse widths [us]

Differential Mixer Model:
  The relationship between arm-angle difference and resulting yaw rate is
  modeled empirically as:

      yaw_rate [deg/s]  ~=  K_delta [1/s] * (left_arm_deg - right_arm_deg) [deg]

  IMPORTANT: This is an EMPIRICAL model derived from initial flight tests
  and video analysis, NOT a first-principles physical law.  The units on
  each side (deg/s vs deg) are different -- K_delta [1/s] is a system-
  identification gain that bridges the gap.

  Initial value: K_delta = 1.0  (1 deg differential ~= 1 deg/s yaw rate)

  Example:
    If commanded_yaw_rate = 20 deg/s and K_delta = 1.0:
      desired_delta_deg = 20 / 1.0 = 20 deg
      left_cmd_deg  = 60 + 20/2 = 70 deg  (more released = less braking)
      right_cmd_deg = 60 - 20/2 = 50 deg  (more pulled   = more braking)
      -> right brake engaged more -> parafoil turns right

  Future extension:
    Left/right asymmetry can be modeled by splitting K_delta into
    K_delta_left and K_delta_right if flight testing reveals significant
    asymmetry in the parafoil turning response.

Servo Convention:
  - Neutral arm position: 60 deg (both sides)
  - Arm angle range: [0, MAX_ANGLE_SCOPE] = [0, 120] deg
      0 deg   = fully pulled (maximum braking)
      60 deg  = neutral (half braking / resting position)
      120 deg = fully released (no braking)
  - Left servo:  pulse INCREASES with angle
      pulse = LEFT_ZERO + angle * PULSE_PER_DEG
  - Right servo: pulse DECREASES with angle (mirrored mounting)
      pulse = RIGHT_ZERO - angle * PULSE_PER_DEG
"""
import math
import time
import types


# =============================================================================
# Hardware Pin Configuration
# =============================================================================
PARAFOIL_LEFT_MOTOR_PIN  = 12  # GPIO 12, physical pin 32
PARAFOIL_RIGHT_MOTOR_PIN = 13  # GPIO 13, physical pin 33


# =============================================================================
# Servo Calibration Constants
# =============================================================================
PULSE_PER_DEG = 2000.0 / 180.0  # [us/deg] ~= 11.11 us per degree

# Zero-angle pulse widths (servo calibration endpoints)
LEFT_ZERO  = 600    # [us] left servo pulse at 0 deg (fully pulled)
RIGHT_ZERO = 2500   # [us] right servo pulse at 0 deg (fully pulled, mirrored)

MAX_ANGLE_SCOPE = 120  # [deg] physical arm travel limit

# Neutral position: 60 deg on both sides (symmetric half-braking)
NEUTRAL_DEG = 60.0
LEFT_NEUTRAL  = int(LEFT_ZERO  + NEUTRAL_DEG * PULSE_PER_DEG)
RIGHT_NEUTRAL = int(RIGHT_ZERO - NEUTRAL_DEG * PULSE_PER_DEG)

# Hardware pulse safety bounds [us]
PULSE_MIN = 500
PULSE_MAX = 2500


# =============================================================================
# Empirical Differential Gain
# =============================================================================
# K_delta relates arm-angle differential to yaw rate:
#   yaw_rate [deg/s]  ~=  K_delta [1/s]  *  delta_deg [deg]
# where delta_deg = left_arm_deg - right_arm_deg.
#
# Physical interpretation:
#   - yaw rate (deg/s) and arm angle (deg) are NOT the same physical quantity.
#   - K_delta is a control input-output gain from empirical testing.
#   - "1 deg of arm differential produces approximately 1 deg/s of yaw rate"
#     was observed in initial flight tests / video analysis.
#
# To tune:
#   - If the parafoil turns FASTER than expected for a given command,
#     DECREASE K_delta (less arm differential needed per deg/s).
#   - If it turns SLOWER, INCREASE K_delta.
#
# This parameter should be easily accessible for field tuning.
K_delta = 1.0  # [1/s]


# =============================================================================
# Initialization / Termination
# =============================================================================
def init_control():
    """Initialize servo hardware and set both arms to neutral (60 deg).

    Uses the pigpio daemon for hardware PWM on the Raspberry Pi.
    Both servos start at their neutral pulse width so the parafoil
    begins in a symmetric, half-braking configuration.

    Returns:
        pigpio.pi instance for subsequent servo commands.
    """
    import pigpio
    pi = pigpio.pi()
    pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, LEFT_NEUTRAL)
    pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, RIGHT_NEUTRAL)
    return pi


def terminate_parafoil_motor(pi):
    """Safely shut down servos: return to neutral, then disable PWM.

    Sequence:
      1. Set both servos to neutral (60 deg) - safe symmetric position.
      2. Brief pause (100 ms) to let servos physically reach position.
      3. Set pulse width to 0 - disables PWM signal entirely.
      4. Stop the pigpio connection.

    Args:
        pi: pigpio.pi instance (may be None if init failed)
    """
    if pi is not None:
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, LEFT_NEUTRAL)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, RIGHT_NEUTRAL)
        time.sleep(0.1)
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, 0)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, 0)
        pi.stop()


# =============================================================================
# Actuator Mixer
# =============================================================================
def actuator_mixer(commanded_yaw_rate: float) -> tuple:
    """Convert commanded yaw rate to left/right arm angles using K_delta model.

    The actuator mixer uses an empirical differential relation where arm angle
    difference is proportional to yaw rate:

        desired_delta_deg = commanded_yaw_rate / K_delta
        left_cmd_deg  = NEUTRAL + desired_delta_deg / 2
        right_cmd_deg = NEUTRAL - desired_delta_deg / 2

    The mixer is symmetric about the neutral position (60 deg):
      - Positive commanded_yaw_rate -> positive delta
        -> left arm releases (higher angle = less braking)
        -> right arm pulls (lower angle = more braking)
        -> parafoil turns RIGHT

    After clamping to [0, MAX_ANGLE_SCOPE], the actual delta may differ
    from the desired delta.  This is important for two reasons:
      1. The actual yaw rate will be less than commanded (saturation).
      2. The anti-windup in the PI controller should detect this.
    Both the desired and actual deltas are returned for logging.

    Future extension: left/right asymmetry can be handled by using
    different K_delta values per side (K_delta_left, K_delta_right).

    Args:
        commanded_yaw_rate: desired yaw rate [deg/s] from PI controller

    Returns:
        (left_cmd_deg, right_cmd_deg, actual_delta_deg, expected_yaw_rate)
        left_cmd_deg:      left arm angle after saturation [deg]
        right_cmd_deg:     right arm angle after saturation [deg]
        actual_delta_deg:  (left - right) after saturation [deg]
        expected_yaw_rate: K_delta * actual_delta_deg [deg/s]
    """
    # Desired differential from the empirical model
    desired_delta_deg = commanded_yaw_rate / K_delta

    # Symmetric mixing around neutral (60 deg)
    left_raw  = NEUTRAL_DEG + desired_delta_deg / 2.0
    right_raw = NEUTRAL_DEG - desired_delta_deg / 2.0

    # Clamp to physical arm limits [0, MAX_ANGLE_SCOPE]
    # Saturation here means the actuator cannot produce the full
    # commanded differential -- the PI anti-windup should account for this.
    left_cmd_deg  = max(0.0, min(float(MAX_ANGLE_SCOPE), left_raw))
    right_cmd_deg = max(0.0, min(float(MAX_ANGLE_SCOPE), right_raw))

    # After saturation, the actual differential may differ from desired
    actual_delta_deg = left_cmd_deg - right_cmd_deg
    expected_yaw_rate = K_delta * actual_delta_deg

    return left_cmd_deg, right_cmd_deg, actual_delta_deg, expected_yaw_rate


# =============================================================================
# Servo Mapping
# =============================================================================
def _servo_pulse_left(angle_deg: float) -> int:
    """Convert left arm angle [deg] to servo pulse width [us].

    Left servo convention:
      pulse = LEFT_ZERO + angle * PULSE_PER_DEG
      0 deg   -> LEFT_ZERO (600 us)  = fully pulled (max braking)
      60 deg  -> LEFT_NEUTRAL (~1267 us) = neutral (half braking)
      120 deg -> ~1933 us = fully released (no braking)

    Args:
        angle_deg: arm angle [deg] in [0, MAX_ANGLE_SCOPE]

    Returns:
        Pulse width [us], clamped to [PULSE_MIN, PULSE_MAX] for safety.
    """
    pulse = int(LEFT_ZERO + angle_deg * PULSE_PER_DEG)
    return max(PULSE_MIN, min(PULSE_MAX, pulse))


def _servo_pulse_right(angle_deg: float) -> int:
    """Convert right arm angle [deg] to servo pulse width [us].

    Right servo convention (mirrored mounting):
      pulse = RIGHT_ZERO - angle * PULSE_PER_DEG
      0 deg   -> RIGHT_ZERO (2500 us) = fully pulled (max braking)
      60 deg  -> RIGHT_NEUTRAL (~1833 us) = neutral (half braking)
      120 deg -> ~1167 us = fully released (no braking)

    Args:
        angle_deg: arm angle [deg] in [0, MAX_ANGLE_SCOPE]

    Returns:
        Pulse width [us], clamped to [PULSE_MIN, PULSE_MAX] for safety.
    """
    pulse = int(RIGHT_ZERO - angle_deg * PULSE_PER_DEG)
    return max(PULSE_MIN, min(PULSE_MAX, pulse))


# =============================================================================
# Apply Differential Deflection (Main Entry Point)
# =============================================================================
def apply_differential_deflection(pi, commanded_yaw_rate: float) -> types.SimpleNamespace:
    """Full actuator pipeline: mixer -> servo mapping -> hardware command.

    This is the main entry point called by motorapp each control cycle.
    It replaces the old single-sided apply_motor_deflection().

    Pipeline:
      1. actuator_mixer()  - commanded_yaw_rate -> left/right arm angles [deg]
      2. servo mapping     - arm angles -> PWM pulse widths [us]
      3. hardware write    - send pulses to GPIO via pigpio

    All intermediate values are returned for logging and post-flight analysis,
    including the actual delta after saturation and the expected yaw rate
    that the saturated command should produce.

    Args:
        pi: pigpio.pi instance (None -> skip hardware, return data only)
        commanded_yaw_rate: output from guidance PI controller [deg/s]

    Returns:
        SimpleNamespace with:
          .left_cmd_deg       [deg] left arm angle after saturation
          .right_cmd_deg      [deg] right arm angle after saturation
          .actual_delta_deg   [deg] left - right (may differ from desired)
          .expected_yaw_rate  [deg/s] K_delta * actual_delta_deg
          .left_pulse         [us] left servo pulse width
          .right_pulse        [us] right servo pulse width
    """
    # Step 1: Actuator mixer (commanded_yaw_rate -> arm angles)
    left_cmd_deg, right_cmd_deg, actual_delta_deg, expected_yaw_rate = \
        actuator_mixer(commanded_yaw_rate)

    # Step 2: Servo mapping (arm angles -> pulse widths)
    left_pulse  = _servo_pulse_left(left_cmd_deg)
    right_pulse = _servo_pulse_right(right_cmd_deg)

    # Step 3: Hardware command
    if pi is not None:
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, left_pulse)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, right_pulse)

    return types.SimpleNamespace(
        left_cmd_deg=left_cmd_deg,
        right_cmd_deg=right_cmd_deg,
        actual_delta_deg=actual_delta_deg,
        expected_yaw_rate=expected_yaw_rate,
        left_pulse=left_pulse,
        right_pulse=right_pulse
    )


def set_neutral(pi):
    """Command both servos to neutral position (60 deg).

    Used on landing, GPS invalid fallback, or when motor control is disabled.
    Both arms return to the symmetric half-braking configuration.

    Args:
        pi: pigpio.pi instance (None -> no-op)
    """
    if pi is not None:
        pi.set_servo_pulsewidth(PARAFOIL_LEFT_MOTOR_PIN, LEFT_NEUTRAL)
        pi.set_servo_pulsewidth(PARAFOIL_RIGHT_MOTOR_PIN, RIGHT_NEUTRAL)
