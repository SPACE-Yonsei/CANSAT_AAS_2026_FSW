#!/usr/bin/env python3

import math
import sys
import argparse
import time
from typing import Tuple

try:
	import pigpio  # Optional, only used when --pigpio is enabled
except Exception:
	pigpio = None

try:
	import board  # Optional, only used when --bno055 is enabled
	import busio
	from adafruit_bno055 import BNO055_I2C
except Exception:
	BNO055_I2C = None
	busio = None
	board = None

def haversine_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
	"""
	Returns initial bearing from point 1 to point 2 in degrees [0, 360).
	"""
	phi1 = math.radians(lat1)
	phi2 = math.radians(lat2)
	dlambda = math.radians(lon2 - lon1)

	y = math.sin(dlambda) * math.cos(phi2)
	x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
	brng = math.degrees(math.atan2(y, x))
	return (brng + 360.0) % 360.0


def normalize_angle_deg(angle: float) -> float:
	"""
	Normalize angle to (-180, 180].
	"""
	a = (angle + 180.0) % 360.0 - 180.0
	# Map -180 to 180 for consistency
	return 180.0 if a == -180.0 else a


def steering_mix(heading_error_deg: float, base_throttle: float, max_throttle: float, k_p: float) -> Tuple[float, float]:
	"""
	Compute differential thrust for two motors based on heading error.
	- heading_error_deg: positive -> target to the right, negative -> to the left
	- base_throttle: nominal throttle (0..1)
	- max_throttle: clamp limit (0..1)
	- k_p: proportional gain [throttle per deg]
	Returns (left, right) throttle in 0..1
	"""
	diff = k_p * heading_error_deg
	left = base_throttle - diff
	right = base_throttle + diff
	left = max(0.0, min(max_throttle, left))
	right = max(0.0, min(max_throttle, right))
	return (left, right)


def throttle_to_pwm(throttle: float, min_us: int = 1000, max_us: int = 2000) -> int:
	"""
	Map 0..1 throttle to servo-style PWM microseconds.
	"""
	throttle = max(0.0, min(1.0, throttle))
	return int(min_us + (max_us - min_us) * throttle)


def servo_mix_continuous(heading_error_deg: float, neutral_us: int, max_delta_us: int, k_p_us_per_deg: float) -> Tuple[int, int]:
	"""
	Continuous-rotation servo mixing for MG92B (speed by pulse offset from neutral).
	- heading_error_deg: + -> turn right (increase right speed, decrease left)
	- neutral_us: calibrated stop pulse (typically ~1500us)
	- max_delta_us: clamp absolute delta from neutral
	- k_p_us_per_deg: proportional gain in microseconds per degree
	Returns (left_us, right_us)
	"""
	diff_us = k_p_us_per_deg * heading_error_deg
	if diff_us > max_delta_us:
		diff_us = max_delta_us
	elif diff_us < -max_delta_us:
		diff_us = -max_delta_us
	left_us = int(neutral_us - diff_us)
	right_us = int(neutral_us + diff_us)
	return (left_us, right_us)


def init_bno055() -> object:
	if BNO055_I2C is None or busio is None or board is None:
		return None
	try:
		i2c = busio.I2C(board.SCL, board.SDA)
		sensor = BNO055_I2C(i2c)
		return sensor
	except Exception:
		return None


def read_bno055_heading_deg(sensor) -> float | None:
	try:
		e = sensor.euler  # (heading, roll, pitch) in degrees; heading is yaw (0..360), may be None
		if e is None:
			return None
		heading = e[0]
		if heading is None:
			return None
		return float(heading) % 360.0
	except Exception:
		return None


def parse_float(prompt: str) -> float:
	while True:
		try:
			return float(input(prompt).strip())
		except Exception:
			print("Invalid number. Try again.")


def main() -> int:
	parser = argparse.ArgumentParser(description="Parafoil ground simulation: two-motor steering from GPS and IMU heading")
	parser.add_argument("--mode", choices=["servo", "throttle"], default="servo", help="servo: MG92B continuous rotation; throttle: 0..1 mapping (default: servo)")
	# Throttle mode options
	parser.add_argument("--base", type=float, default=0.4, help="[throttle mode] Base throttle 0..1 (default: 0.4)")
	parser.add_argument("--max", dest="max_thr", type=float, default=0.8, help="[throttle mode] Max throttle clamp 0..1 (default: 0.8)")
	parser.add_argument("--kp", type=float, default=0.005, help="[throttle mode] Heading P gain (throttle/deg), default 0.005")
	# Servo (continuous) mode options
	parser.add_argument("--neutral-us", type=int, default=1500, help="[servo mode] Neutral pulse width us (stop), default 1500")
	parser.add_argument("--delta-max-us", type=int, default=250, help="[servo mode] Max delta from neutral in us, default 250")
	parser.add_argument("--kp-us", type=float, default=2.0, help="[servo mode] Heading P gain in us/deg, default 2.0")
	parser.add_argument("--pigpio", action="store_true", help="Output PWM using pigpio (requires Raspberry Pi and pigpio)")
	parser.add_argument("--left-gpio", type=int, default=12, help="GPIO pin for left motor PWM (default: 18)")
	parser.add_argument("--right-gpio", type=int, default=13, help="GPIO pin for right motor PWM (default: 19)")
	parser.add_argument("--min-us", type=int, default=1000, help="Servo min pulse width in us (default: 1000)")
	parser.add_argument("--max-us", type=int, default=2000, help="Servo max pulse width in us (default: 2000)")
	parser.add_argument("--trim-left-us", type=int, default=0, help="[servo mode] Trim offset for left servo in us, default 0")
	parser.add_argument("--trim-right-us", type=int, default=0, help="[servo mode] Trim offset for right servo in us, default 0")
	parser.add_argument("--bno055", action="store_true", help="Read heading from BNO055 over I2C instead of manual input")
	args = parser.parse_args()

	if args.pigpio and pigpio is None:
		print("pigpio not available. Install pigpio or run without --pigpio.")
		return 1

	pi = None
	if args.pigpio:
		try:
			pi = pigpio.pi()  # gpio 18, 19. 물리적 12, 35
			if not pi.connected:
				print("Failed to connect to pigpio daemon. Is pigpiod running?")
				return 1
			pi.set_mode(args.left_gpio, pigpio.OUTPUT)
			pi.set_mode(args.right_gpio, pigpio.OUTPUT)
		except Exception as e:
			print(f"pigpio init error: {e}")
			return 1

	sensor = None
	if args.bno055:
		sensor = init_bno055()
		if sensor is None:
			print("Failed to initialize BNO055. Proceeding with manual heading input.")

	print("Parafoil Ground Simulation")
	print("Enter 'q' at any prompt to quit.")

	# Initial target set to 0,0; allow one-time override
	t_lat = 0.0
	t_lon = 0.0
	try:
		raw = input("\nTarget GPS lat,lon [default 0.0,0.0]: ").strip()
		if raw.lower() == "q":
			return 0
		if raw != "":
			t_lat_str, t_lon_str = [s.strip() for s in raw.split(",")]
			t_lat = float(t_lat_str)
			t_lon = float(t_lon_str)
	except Exception:
		print("Invalid target input. Using default 0.0,0.0.")

	# Current GPS once
	while True:
		try:
			raw = input("Current GPS lat,lon (deg): ").strip()
			if raw.lower() == "q":
				return 0
			c_lat_str, c_lon_str = [s.strip() for s in raw.split(",")]
			c_lat = float(c_lat_str)
			c_lon = float(c_lon_str)
			break
		except Exception:
			print("Invalid current GPS input. Please try again.")

	print("\nRunning... Press Ctrl+C to exit.")
	while True:
		try:
			if sensor is not None:
				imu_heading = read_bno055_heading_deg(sensor)
				if imu_heading is None:
					print("BNO055 heading invalid; skipping this cycle.")
					time.sleep(1.0)
					continue
			else:
				raw = input("IMU heading (deg, 0..360, 0=N/Up, CW): ").strip()
				if raw.lower() == "q":
					break
				imu_heading = float(raw) % 360.0

			bearing = haversine_bearing(c_lat, c_lon, t_lat, t_lon)
			err = normalize_angle_deg(bearing - imu_heading)

			if args.mode == "servo":
				left_us, right_us = servo_mix_continuous(err, args.neutral_us, args.delta_max_us, args.kp_us)
				left_us += args.trim_left_us
				right_us += args.trim_right_us
				# Clamp to allowed servo range
				left_us = max(args.min_us, min(args.max_us, left_us))
				right_us = max(args.min_us, min(args.max_us, right_us))
				print(f"Bearing: {bearing:.2f} deg | IMU: {imu_heading:.2f} deg | Error: {err:.2f} deg")
				print(f"Servo PWM us L/R: {left_us} / {right_us} (neutral {args.neutral_us}, kp_us {args.kp_us})")
			else:
				left_thr, right_thr = steering_mix(err, args.base, args.max_thr, args.kp)
				left_us = throttle_to_pwm(left_thr, args.min_us, args.max_us)
				right_us = throttle_to_pwm(right_thr, args.min_us, args.max_us)
				print(f"Bearing: {bearing:.2f} deg | IMU: {imu_heading:.2f} deg | Error: {err:.2f} deg")
				print(f"Throttle L/R: {left_thr:.3f} / {right_thr:.3f} -> PWM us L/R: {left_us} / {right_us}")

			if pi is not None:
				try:
					pi.set_servo_pulsewidth(args.left_gpio, left_us)
					pi.set_servo_pulsewidth(args.right_gpio, right_us)
				except Exception as e:
					print(f"pigpio set error: {e}")

			# 1 Hz update when using BNO055
			if sensor is not None:
				time.sleep(1.0)
		except KeyboardInterrupt:
			break

	if pi is not None:
		try:
			pi.set_servo_pulsewidth(args.left_gpio, 0)
			pi.set_servo_pulsewidth(args.right_gpio, 0)
			pi.stop()
		except Exception:
			pass

	print("Exit.")
	return 0


if __name__ == "__main__":
	sys.exit(main())


