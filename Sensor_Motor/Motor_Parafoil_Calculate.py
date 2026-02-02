"""
파라포일 모터 제어 모듈
IMU/GPS 데이터 기반 turn angle 계산
"""

import math
from lib import prevstate

# =============================================================================
# 상태 변수
# =============================================================================

target_lat = 0.0
target_lon = 0.0
last_error = None  # GPS 무효 시 사용할 마지막 유효 방위각

def init_parafoil_control():
    global target_lat, target_lon
    try:
        target_lat = prevstate.Target_lat
        target_lon = prevstate.Target_lon
    except Exception:
        pass

# =============================================================================
# GPS 유틸리티
# =============================================================================

def is_gps_valid(lat: float, lon: float) -> bool:
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
    global target_lat, target_lon
    target_lat, target_lon = lat, lon
    try:
        prevstate.update_target_gps(lat, lon)
    except Exception:
        pass


def get_target_coordinates() -> tuple[float, float]:
    return target_lat, target_lon


# =============================================================================
# 모터 제어 계산
# =============================================================================

def quick_angle(angle: float) -> float:
    while angle >= 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

last_yaw = None
MAX_YAW_CHANGE = 25  # 한 사이클당 최대 변화량 (도)

def calculate_motor_control(yaw: float) -> float:
    global last_error, last_yaw

    if last_yaw is None:
        last_yaw = yaw
    else:
        # ★ 3. 각도 차이를 -180~180 범위로 계산
        diff = yaw - last_yaw
        
        # ★ 4. 급격한 변화 제한
        if abs(diff) > MAX_YAW_CHANGE:
            diff = MAX_YAW_CHANGE if diff > 0 else -MAX_YAW_CHANGE
        
        last_yaw = yaw

    # GPS 유효 → 방위각 계산
    if is_gps_valid(1, 1) and not (target_lat == 0.0 and target_lon == 0.0):
        target_azimuth = math.degrees(math.atan2(target_lon - 1, target_lat - 1))
        error=quick_angle(target_azimuth - yaw)
        last_error = error
        print(f"\nclaculated yaw={yaw:.1f}, azimuth={target_azimuth:.1f}, error={error:.1f}")
        return error

    # GPS 무효 + 목표 좌표 없음 → IMU 기반 기본 헤딩(0도) 유지
    if target_lat == 0.0 and target_lon == 0.0:
        return quick_angle(0.0 - yaw)
    # GPS 무효 but 이전 방위각 있음 → 유지
    if last_error is not None:
        return last_error
    
    # GPS 무효, 이전 방위각 없음 → 직진
    return 0.0
