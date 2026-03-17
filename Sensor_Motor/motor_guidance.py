#!/usr/bin/env python3
"""
Parafoil Motor Control Module
L1 Carrot Guidance + Cascaded Loop (Outer heading + Inner yaw-rate PI)
"""

import math
import time

# =============================================================================
# Guidance Constants
# =============================================================================
LAT_TO_METER = 111320.0
L_DISTANCE = 15.0        # L1 look-ahead distance (m)

# =============================================================================
# Control Gains
# =============================================================================
Kp_outer = 0.6           # heading error -> desired yaw rate
Kp_inner = 0.5           # rate error -> u
Ki_inner = 0.1           # rate error integral -> u
MAX_INTEGRAL = 30.0
DEADBAND = 8.0           # heading error deadband (degrees)

# =============================================================================
# State Variables
# =============================================================================
target_lat = 0.0
target_lon = 0.0
start_lat = 0.0
start_lon = 0.0

wind_crab_est = 0.0
pi_integral = 0.0
last_time = None

#north based angle
def _quick_angle(a: float) -> float:
    return (a + 180) % 360 - 180

# =============================================================================
# GPS Utilities
# =============================================================================
def is_gps_valid(lat: float, lon: float) -> bool:
    return not (lat == 0.0 and lon == 0.0) and abs(lat) <= 90.0 and abs(lon) <= 180.0


def calculate_distance_haversine(lat1: float, lon1: float,
                                  lat2: float, lon2: float) -> float:
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)
    a = (math.sin(d_phi / 2) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def _llh_to_ne(lat: float, lon: float) -> tuple[float, float]:
    N = (lat - start_lat) * LAT_TO_METER
    E = (lon - start_lon) * LAT_TO_METER * math.cos(math.radians(start_lat))
    return N, E


# =============================================================================
# L1 Carrot Guidance
# =============================================================================
def _carrot(my_N: float, my_E: float,
            tgt_N: float, tgt_E: float, L: float) -> tuple[float, float]:
    rope_len = math.hypot(tgt_N, tgt_E)
    if rope_len < 0.01:
        return tgt_N, tgt_E
    uN, uE = tgt_N / rope_len, tgt_E / rope_len
    s = max(my_N * uN + my_E * uE, min(0.0, rope_len))
    return (s + L) * uN, (s + L) * uE

# =============================================================================
# Target Coordinate Management
# =============================================================================
def set_start_coordinates(lat: float, lon: float):
    global start_lat, start_lon
    start_lat, start_lon = lat, lon


def set_target_coord(lat: float, lon: float):
    global target_lat, target_lon
    target_lat, target_lon = lat, lon

def draw_pattern():
    return

def guidance(pi, yaw: float, gyro_z: float,
             current_lat: float, current_lon: float,
             gps_speed: float = 0.0, gps_course: float = 0.0
             ) -> dict:
    """
    L1 Carrot Guidance + Cascaded Control

    Args:
        pi:          pigpio instance
        yaw:         current heading (degrees, 0=N, 90=E)
        gyro_z:      current yaw rate (deg/s)
        current_lat: GPS latitude
        current_lon: GPS longitude
        gps_speed:   GPS ground speed (m/s)
        gps_course:  GPS track angle (degrees)

    Returns:
        dict with control state for logging
    """
    global wind_crab_est, pi_integral, last_time

    # -- dt --
    prsnt_time = time.time()
    dt = prsnt_time - last_time if last_time else 0.1
    if dt <= 0.02 or dt > 0.5:
        dt = 0.1
    last_time = prsnt_time

    # -- GPS validity check --
    if not is_gps_valid(current_lat, current_lon) or (target_lat == 0.0 and target_lon == 0.0):  
        
        # -- [1] L1 Carrot Guidance --
        my_N, my_E = _llh_to_ne(current_lat, current_lon)
        tgt_N, tgt_E = _llh_to_ne(target_lat, target_lon)
        distance = math.hypot(tgt_N - my_N, tgt_E - my_E)

        cN, cE = _carrot(my_N, my_E, tgt_N, tgt_E, L_DISTANCE)
        desired_course = math.degrees(math.atan2(cE - my_E, cN - my_N))

        # -- [2] Wind Compensation (crab angle estimation) --
        if gps_speed > 1.0 and abs(gyro_z) < 20.0:
            current_crab = _quick_angle(gps_course - yaw)
            wind_crab_est = 0.95 * wind_crab_est + 0.05 * current_crab

        desired_heading = _quick_angle(desired_course - wind_crab_est)

        # -- [3] Outer Loop: heading error -> desired yaw rate --
        heading_error = _quick_angle(desired_heading - yaw)
        desired_yaw_rate = Kp_outer * heading_error

        # -- [4] Inner Loop: PI (yaw rate error -> u) --
        rate_error = desired_yaw_rate - gyro_z

        if abs(heading_error) <= DEADBAND:
            pi_integral = 0.0
            u = 0.0
        else:
            pi_integral = max(pi_integral + rate_error * dt, min(-MAX_INTEGRAL, MAX_INTEGRAL))
            u = Kp_inner * rate_error + Ki_inner * pi_integral
        return

    return 

def reset_control():
    global wind_crab_est, pi_integral, last_time
    wind_crab_est = 0.0
    pi_integral = 0.0
    last_time = time.time()
