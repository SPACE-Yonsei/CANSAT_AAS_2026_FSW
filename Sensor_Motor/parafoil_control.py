"""
파라포일 모터 제어 모듈
IMU/GPS 데이터 기반 파라포일 turn angle 계산
"""

import math
from lib import prevstate

# =============================================================================
# 상태 변수
# =============================================================================

_target_lat = 0.0
_target_lon = 0.0
_last_valid_angle = None
_target_reached = False

TARGET_RADIUS = 5.0  # 목표 도달 반경 (m)


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
    if lat == 0.0 and lon == 0.0:
        return False
    if abs(lat) > 90.0 or abs(lon) > 180.0:
        return False
    return True


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
    """목표 GPS 좌표 설정"""
    global _target_lat, _target_lon
    _target_lat, _target_lon = lat, lon
    try:
        prevstate.update_target_gps(lat, lon)
    except Exception:
        pass


def get_target_coordinates() -> tuple[float, float]:
    """목표 좌표 반환"""
    return _target_lat, _target_lon


# =============================================================================
# 모터 제어 계산
# =============================================================================

def _normalize_angle(angle: float) -> float:
    """각도를 -180 ~ +180 범위로 정규화"""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def calculate_motor_control(yaw: float, lat: float, lon: float) -> float:
    """
    현재 위치/자세 기반 모터 제어 각도 계산
    
    Args:
        yaw: 현재 yaw 각도 (0-360)
        lat: 현재 위도
        lon: 현재 경도
    
    Returns:
        turn angle (-180 ~ +180)
        - 양수: 우회전
        - 음수: 좌회전
        - 0: 직진
    """
    global _last_valid_angle, _target_reached
    
    # 목표 미설정
    if _target_lat == 0.0 and _target_lon == 0.0:
        return 0.0
    
    # GPS 유효
    if is_gps_valid(lat, lon):
        # 목표 방위각 계산
        bearing_rad = math.atan2(_target_lat - lat, _target_lon - lon)
        bearing_deg = math.degrees(bearing_rad)
        if bearing_deg < 0:
            bearing_deg += 360
        
        _last_valid_angle = bearing_deg
        
        return _normalize_angle(yaw - bearing_deg)
    
    # GPS 무효 but 이전 유효 각도 있음
    elif _last_valid_angle is not None:
        return _normalize_angle(yaw - _last_valid_angle)
    
    # GPS 무효, 이전 각도 없음
    return 0.0
