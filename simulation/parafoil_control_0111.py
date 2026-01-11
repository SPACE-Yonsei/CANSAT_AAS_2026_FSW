"""
파라포일 모터 제어 모듈
IMU/GPS 데이터 기반 turn angle 계산
"""

import math
from lib import prevstate

# =============================================================================
# 상태 변수
# =============================================================================

_target_lat = 0.0
_target_lon = 0.0
_last_error = None  # GPS 무효 시 사용할 마지막 유효 방위각


def init_parafoil_control():
    """prevstate에서 목표 좌표 로드"""
    global _target_lat, _target_lon
    try:
        _target_lat = prevstate.Target_lat
        _target_lon = prevstate.Target_lon
    except Exception:
        pass

# =============================================================================
# GPS 유틸리티
# =============================================================================

def is_gps_valid(lat: float, lon: float) -> bool:
    """GPS 좌표 유효성 검사"""
    return not (lat == 0.0 and lon == 0.0) and abs(lat) <= 90.0 and abs(lon) <= 180.0


def calculate_distance_haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Haversine 공식으로 두 좌표 간 거리 계산 (m)"""
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)
    
    a = math.sin(d_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

# =============================================================================
# 목표 좌표 관리
# =============================================================================

def set_target_coordinates(lat: float, lon: float):
    """목표 GPS 좌표 설정 (prevstate에도 저장)"""
    global _target_lat, _target_lon
    _target_lat, _target_lon = lat, lon
    try:
        prevstate.update_target_gps(lat, lon)
    except Exception:
        pass

# =============================================================================
# 모터 제어 계산
# =============================================================================

def quick_angle(angle: float) -> float:
    """각도를 -180 ~ +180 범위로 정규화"""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def calculate_error(yaw: float, lat: float, lon: float) -> float:

    global _last_error
    
    # 목표 미설정 → 직진
    if _target_lat == 0.0 and _target_lon == 0.0:
        return 0.0
    
    # GPS 유효 → 방위각 계산
    if is_gps_valid(lat, lon):
        target_angle_based_north = math.degrees(math.atan2(_target_lon - lon, _target_lat - lat))
        error=quick_angle(target_angle_based_north-yaw)
        return error
    
    # GPS 무효, 이전 방위각 없음 → 직진
    return 0.0

