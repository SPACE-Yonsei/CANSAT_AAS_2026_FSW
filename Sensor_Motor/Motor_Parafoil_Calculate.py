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
# ==========================================
# 전역 변수 (함수 밖에 선언하거나, 클래스 멤버변수로 사용)
# ==========================================
prev_filtered_error = 0.0  # 이전 필터링된 에러 값 저장
is_first_run = True        # 첫 실행 여부 확인

# ==========================================
# 모터 제어 함수 내부 수정
# ==========================================
def calculate_motor_control(yaw: float) -> float:
    global prev_filtered_error, is_first_run, last_error # 필요한 전역변수 호출
    
    # 1. [기존 로직] 목표 방위각 및 기본 에러 계산
    dx = target_lon - 1
    dy = target_lat - 1
    target_azimuth = 0.0
    # =============================================================================
# 모터 제어 계산
# =============================================================================

def quick_angle(angle: float) -> float:
    while angle >= 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

# 전역 변수
prev_filtered_error = 0.0
is_first_run = True
last_error = None

# 설정값
MAX_CHANGE = 15.0      # 한 루프당 최대 변화 (도)
ALPHA = 0.3            # 스무딩 팩터 (0.1=부드러움, 1.0=즉각반응)

def calculate_motor_control(yaw: float) -> float:
    global prev_filtered_error, is_first_run, last_error
    
    dx = target_lon - 1
    dy = target_lat - 1

    # GPS 무효 + 목표 좌표 없음 → 직진
    if target_lat == 0.0 and target_lon == 0.0:
        return quick_angle(0.0 - yaw)
    
    # GPS 유효 → 방위각 계산
    if is_gps_valid(1, 1):
        target_azimuth = math.degrees(math.atan2(dx, dy))
        if target_azimuth < 0:
            target_azimuth += 360
        
        # 날것의 에러 계산
        raw_error = quick_angle(target_azimuth - yaw)
        
        # ========== 필터링 시작 ==========
        
        # 첫 실행 시 초기화
        if is_first_run:
            prev_filtered_error = raw_error
            is_first_run = False
            last_error = raw_error
            print(f"yaw={yaw:.1f}, azimuth={target_azimuth:.1f}, error={raw_error:.1f} (init)")
            return raw_error
        
        # Rate Limiter: 급격한 변화 제한
        diff = raw_error - prev_filtered_error
        if diff > MAX_CHANGE:
            raw_error = prev_filtered_error + MAX_CHANGE
            print(f"⚠️ Clamped: +{MAX_CHANGE}")
        elif diff < -MAX_CHANGE:
            raw_error = prev_filtered_error - MAX_CHANGE
            print(f"⚠️ Clamped: -{MAX_CHANGE}")
        
        # Low Pass Filter: 부드럽게
        filtered_error = (ALPHA * raw_error) + ((1 - ALPHA) * prev_filtered_error)
        prev_filtered_error = filtered_error
        last_error = filtered_error
        
        print(f"yaw={yaw:.1f}, azimuth={target_azimuth:.1f}, raw={raw_error:.1f}, filtered={filtered_error:.1f}")
        return filtered_error
    
    # GPS 무효 but 이전 에러 있음 → 유지
    if last_error is not None:
        return last_error
    
    return 0.0