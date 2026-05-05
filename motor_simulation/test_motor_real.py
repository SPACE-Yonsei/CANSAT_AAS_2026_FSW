#!/usr/bin/env python3
"""
IMU yaw-driven parafoil servo test.
Reads yaw angle from BNO085, deflects servos from 60-deg neutral
proportional to heading error from the reference yaw captured at startup.
"""
import os
import sys
import time
import signal

import pigpio

FSW_ROOT = '/root/CANSAT_AAS_2026_FSW'
os.chdir(FSW_ROOT)
sys.path.insert(0, FSW_ROOT)

# Ensure config.txt exists so config.py doesn't spam on every import
_cfg = os.path.join(FSW_ROOT, 'lib', 'config.txt')
if not os.path.exists(_cfg):
    os.makedirs(os.path.dirname(_cfg), exist_ok=True)
    with open(_cfg, 'w') as _f:
        _f.write("# SELECTED=PAYLOAD\n")

from Sensor_Imu.imu import init_imu, read_sensor_data, imu_terminate

# ── Servo constants ───────────────────────────────────────────────────────────
RIGHT_PIN = 12
LEFT_PIN  = 13

PULSE_PER_DEG  = 2000.0 / 180.0

LEFT_ZERO      = 600
RIGHT_ZERO     = 2500
NEUTRAL_DEG    = 60.0                        # flight-ready neutral

LEFT_NEUTRAL   = int(LEFT_ZERO  + NEUTRAL_DEG * PULSE_PER_DEG)
RIGHT_NEUTRAL  = int(RIGHT_ZERO - NEUTRAL_DEG * PULSE_PER_DEG)
LEFT_MAX       = int(LEFT_ZERO  + 120 * PULSE_PER_DEG)
RIGHT_MIN      = int(RIGHT_ZERO - 120 * PULSE_PER_DEG)
PULSE_MIN      = 500
PULSE_MAX      = 2500

# ── Control gain ──────────────────────────────────────────────────────────────
# 1° of yaw error → YAW_GAIN μs of servo offset
# (60° yaw error fills the full 60° deflection from neutral)
YAW_GAIN = (60.0 * PULSE_PER_DEG) / 60.0    # μs / deg_error

MAX_YAW_ERROR  = 60.0   # deg — clamp input beyond this

LOOP_HZ = 20
LOOP_DT = 1.0 / LOOP_HZ


def yaw_error_wrap(current: float, reference: float) -> float:
    """Shortest angular distance, wrapped to [-180, 180]."""
    err = current - reference
    if err >  180: err -= 360
    if err < -180: err += 360
    return err


def actuator(yaw_err: float):
    """
    yaw_err > 0 (nose right of target) → pull left → right turn to correct.
    yaw_err < 0 (nose left of target)  → pull right → left turn to correct.
    Differential: LEFT increases, RIGHT decreases by same offset (or vice versa).
    """
    clamped = max(-MAX_YAW_ERROR, min(MAX_YAW_ERROR, yaw_err))
    offset  = clamped * YAW_GAIN                      # μs

    l_pw = max(PULSE_MIN, min(LEFT_MAX,  int(LEFT_NEUTRAL  + offset)))
    r_pw = max(RIGHT_MIN, min(PULSE_MAX, int(RIGHT_NEUTRAL - offset)))
    return l_pw, r_pw


def pulse_to_deg_left(pw):  return (pw - LEFT_ZERO)  / PULSE_PER_DEG
def pulse_to_deg_right(pw): return (RIGHT_ZERO - pw) / PULSE_PER_DEG


# ── Init ──────────────────────────────────────────────────────────────────────
print("Initializing pigpio...")
pi = pigpio.pi()
if not pi.connected:
    print("ERROR: pigpio connection failed — run: sudo pigpiod")
    sys.exit(1)

print("Initializing IMU (BNO085)...")
try:
    i2c, sensor = init_imu()
except RuntimeError as e:
    print(f"ERROR: IMU init failed — {e}")
    pi.stop()
    sys.exit(1)

print(f"left_neutral:  {LEFT_NEUTRAL}us  ({NEUTRAL_DEG:.0f} deg)")
print(f"right_neutral: {RIGHT_NEUTRAL}us  ({NEUTRAL_DEG:.0f} deg)")

# Capture reference yaw from first valid IMU reading
print("Capturing reference yaw...")
ref_yaw = None
for _ in range(20):
    try:
        data = read_sensor_data(sensor)
        ref_yaw = data[2]   # avg_yaw
        break
    except Exception:
        time.sleep(0.1)

if ref_yaw is None:
    print("ERROR: could not read initial yaw")
    pi.stop()
    imu_terminate(i2c)
    sys.exit(1)

print(f"Reference yaw = {ref_yaw:.2f} deg")
print("Control loop running. Press Ctrl+C to stop.\n")

# Start at neutral
pi.set_servo_pulsewidth(LEFT_PIN,  LEFT_NEUTRAL)
pi.set_servo_pulsewidth(RIGHT_PIN, RIGHT_NEUTRAL)


def shutdown(*_):
    print("\nCtrl+C — returning to neutral then off")
    pi.set_servo_pulsewidth(LEFT_PIN,  LEFT_NEUTRAL)
    pi.set_servo_pulsewidth(RIGHT_PIN, RIGHT_NEUTRAL)
    time.sleep(0.3)
    pi.set_servo_pulsewidth(LEFT_PIN,  0)
    pi.set_servo_pulsewidth(RIGHT_PIN, 0)
    pi.stop()
    imu_terminate(i2c)
    sys.exit(0)


signal.signal(signal.SIGINT,  shutdown)
signal.signal(signal.SIGTERM, shutdown)

# ── Control loop ──────────────────────────────────────────────────────────────
tick = 0
while True:
    t0 = time.monotonic()

    try:
        roll, pitch, yaw, *_ = read_sensor_data(sensor)
    except Exception as e:
        print(f"IMU read error: {e}")
        time.sleep(LOOP_DT)
        continue

    err = yaw_error_wrap(yaw, ref_yaw)
    l_pw, r_pw = actuator(err)
    pi.set_servo_pulsewidth(LEFT_PIN,  l_pw)
    pi.set_servo_pulsewidth(RIGHT_PIN, r_pw)

    tick += 1
    if tick % LOOP_HZ == 0:
        l_deg = pulse_to_deg_left(l_pw)
        r_deg = pulse_to_deg_right(r_pw)
        dir_str = "R>>" if err >  2 else "<<L" if err < -2 else "|||"
        print(
            f"[{tick // LOOP_HZ:04d}s] "
            f"yaw={yaw:>7.2f}°  ref={ref_yaw:.2f}°  err={err:>+6.2f}°  {dir_str}\n"
            f"  LEFT  : {l_pw:>4d} us  {l_deg:>6.2f} deg\n"
            f"  RIGHT : {r_pw:>4d} us  {r_deg:>6.2f} deg"
        )

    elapsed = time.monotonic() - t0
    if LOOP_DT - elapsed > 0:
        time.sleep(LOOP_DT - elapsed)
