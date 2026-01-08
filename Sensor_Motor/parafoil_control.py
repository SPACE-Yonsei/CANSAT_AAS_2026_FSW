#!/usr/bin/env python3
"""
파라포일 모터 제어 방향 판단 모듈

IMU와 GPS 데이터를 기반으로 파라포일 모터 제어 방향(turn angle)을 계산합니다.
"""

import math
from multiprocessing import Queue
from lib import prevstate

# Target GPS coordinates (target location)
# Initial values are loaded from prevstate
Target_lat = 0.0
Target_lon = 0.0


def init_parafoil_control():
    global Target_lat, Target_lon
    try:
        if hasattr(prevstate, 'Target_lat') and hasattr(prevstate, 'Target_lon'):
            Target_lat = prevstate.Target_lat
            Target_lon = prevstate.Target_lon
    except Exception:
        pass

# GPS validity and last valid bearing angle
GPS_VALID = False
Last_Valid_GPS_Angle = None  # Last valid GPS bearing angle (used when GPS is lost)

# Target reached radius
TARGET_REACHED_RADIUS = 50.0  # Target reached radius in meters
TARGET_REACHED = False  # Flag to track if target GPS location has been reached


def is_gps_valid(lat: float, lon: float) -> bool:
    """
    Check if GPS coordinates are valid.
    
    Args:
        lat: Latitude
        lon: Longitude
    
    Returns:
        True if GPS data is valid, False otherwise
    """
    # Check if coordinates are 0, 0 (invalid GPS reading)
    if lat == 0.0 and lon == 0.0:
        return False
    
    # Check coordinate range
    if abs(lat) > 90.0 or abs(lon) > 180.0:
        return False
    
    return True


def calculate_distance_haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Haversine 공식을 사용하여 두 GPS 좌표 간의 거리를 계산합니다.
    
    Args:
        lat1: 첫 번째 지점의 위도
        lon1: 첫 번째 지점의 경도
        lat2: 두 번째 지점의 위도
        lon2: 두 번째 지점의 경도
    
    Returns:
        거리 (미터 단위)
    """
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    # Haversine 공식
    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    distance = R * c
    return distance


def set_target_coordinates(lat: float, lon: float):
    """
    Set target GPS coordinates.
    
    Args:
        lat: Target latitude
        lon: Target longitude
    """
    global Target_lat, Target_lon
    Target_lat = lat
    Target_lon = lon
    # Also save to prevstate to persist through restarts
    try:
        prevstate.update_target_gps(lat, lon)
    except Exception:
        # prevstate module may not be initialized yet (ignore)
        pass


def get_target_coordinates():
    """
    Get currently set target GPS coordinates.
    
    Returns:
        (lat, lon) tuple
    """
    return (Target_lat, Target_lon)


def calculate_motor_control(current_yaw: float, current_lat: float, current_lon: float, gps_valid: bool = None) -> float:
    """
    Calculate motor control direction (turn angle) based on current position and attitude.
    
    Args:
        current_yaw: Current yaw angle (degrees, 0-360)
        current_lat: Current latitude
        current_lon: Current longitude
        gps_valid: GPS validity (None for auto-detection)
    
    Returns:
        turn angle (degrees, -180 ~ +180)
            - Positive: right turn needed
            - Negative: left turn needed
            - 0: straight ahead
    """
    global Target_lat, Target_lon, GPS_VALID, Last_Valid_GPS_Angle, TARGET_REACHED, TARGET_REACHED_RADIUS
    
    # Auto-detect GPS validity
    if gps_valid is None:
        gps_valid = is_gps_valid(current_lat, current_lon)
    
    GPS_VALID = gps_valid
    
    # Stop motor if target coordinates are not set
    if Target_lat == 0.0 and Target_lon == 0.0:
        return 0.0
    
    # GPS is valid: GPS-based motor control
    if GPS_VALID:
        # Calculate bearing angle to target
        gps_angle_rad = math.atan2(Target_lat - current_lat, Target_lon - current_lon)
        gps_angle_deg = math.degrees(gps_angle_rad)  # Convert radians to degrees
        
        # Normalize gps_angle to 0-360 degree range
        if gps_angle_deg < 0:
            gps_angle_deg += 360
        
        Last_Valid_GPS_Angle = gps_angle_deg
        
        # Check if target is reached
        if not TARGET_REACHED:
            distance_to_target = calculate_distance_haversine(current_lat, current_lon, Target_lat, Target_lon)
            if distance_to_target <= TARGET_REACHED_RADIUS:
                TARGET_REACHED = True
        
        # Calculate turn angle
        # current_yaw - gps_bearing: positive = right turn needed, negative = left turn needed
        angle_diff = current_yaw - gps_angle_deg
        
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360
        
        return angle_diff
    
    # GPS is invalid but previous valid GPS bearing angle exists
    elif Last_Valid_GPS_Angle is not None:
        # Maintain last valid bearing angle for motor control
        angle_diff = current_yaw - Last_Valid_GPS_Angle
        
        # Normalize angle difference to -180 ~ +180 range
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360
        
        return angle_diff
    
    # No GPS and no previous valid bearing angle: stop motor
    else:
        return 0.0


def reset_target_reached():
    """Reset target reached flag."""
    global TARGET_REACHED
    TARGET_REACHED = False


def is_target_reached() -> bool:
    """
    Check if target location has been reached.
    
    Returns:
        True if target location has been reached, False otherwise
    """
    return TARGET_REACHED


def get_last_valid_gps_angle():
    """
    Get last valid GPS bearing angle.
    
    Returns:
        Last valid GPS bearing angle (degrees, 0-360) or None
    """
    return Last_Valid_GPS_Angle


def imu_logic(Main_Queue: Queue, recent_yaw: float, appargs_module=None, msgstructure_module=None):
    """
    Process IMU data and send to motorapp.
    
    Args:
        Main_Queue: Message transmission queue
        recent_yaw: Current yaw angle
        appargs_module: appargs module (for message ID access)
        msgstructure_module: msgstructure module (for message transmission)
    """
    # Modules are required for message transmission
    if appargs_module is None or msgstructure_module is None:
        return
    
    # Send IMU data to motorapp (100Hz)
    # motorapp calculates motor control direction and controls motors
    SendImuMotorDataMsg = msgstructure_module.MsgStructure()
    msgstructure_module.send_msg(
        Main_Queue, 
        SendImuMotorDataMsg, 
        appargs_module.FlightlogicAppArg.AppID, 
        appargs_module.MotorAppArg.AppID, 
        appargs_module.FlightlogicAppArg.MID_SendImuMotorData, 
        str(recent_yaw)
    )
    
    return

