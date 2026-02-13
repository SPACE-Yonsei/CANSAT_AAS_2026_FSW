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
    """각도를 -180 ~ +180 범위로 정규화"""
    while angle >= 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

# 전역 변수
last_error = None

# 설정값
MAX_CHANGE = 15.0  # raw_error 급변 감지 임계값
ALPHA = 0.3        # Low Pass Filter 계수 (0.1=부드러움, 1.0=즉각반응)

def calculate_motor_control(yaw: float, current_lat, current_lon) -> float:
    # [1. 추가] 전역 변수 수정 권한 획득 (이게 없으면 저장이 안 됩니다)
    global last_error 

    # [2. 추가] 입력값 방어 (센서가 None을 줄 경우 대비)
    if yaw is None or current_lat is None or current_lon is None:
         return last_error if last_error is not None else 0.0

    # 목표 좌표 없음 → 직진
    if target_lat == 0.0 and target_lon == 0.0:
        return 0.0

    # [3. 추가] 수학 계산 중 에러(ZeroDivision 등)가 나도 멈추지 않게 try로 감쌈
    try:
        if is_gps_valid(current_lat, current_lon):
            # --- 기존 계산 로직 시작 ---
            phi1 = math.radians(current_lat)
            phi2 = math.radians(target_lat)
            d_lambda = math.radians(target_lon - current_lon)

            y = math.sin(d_lambda) * math.cos(phi2)
            x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(d_lambda)
            
            target_azimuth = math.degrees(math.atan2(y, x))
            raw_error = quick_angle(target_azimuth - yaw)
            
            last_error = raw_error
            return raw_error

    except Exception as e:
        # 에러 발생 시 로그만 찍고(선택사항) 아래 Fallback으로 넘어감
        # print(f"Calc Error: {e}") 
        pass

    # [5. 추가] GPS가 끊기거나 에러 발생 시, 기억해둔 직전 값 리턴
    if last_error is not None:
        return last_error
    
    # 아무 기록도 없으면 직진
    return 0.0
# def calculate_motor_control(yaw: float, current_lat, current_lon) -> float:
#     global prev_filtered_error, prev_raw_error, is_first_run, last_error
    
#     dx = target_lon - current_lon
#     dy = target_lat - current_lat

#     # 목표 좌표 없음 → 직진 (yaw=0 유지)
#     if target_lat == 0.0 and target_lon == 0.0:
#         return quick_angle(0.0 - yaw)
    
#     # GPS 유효 → 방위각 계산
#     if is_gps_valid(current_lat, current_lon):
#         phi1 = math.radians(current_lat)
#         phi2 = math.radians(target_lat)
#         d_lambda = math.radians(target_lon - current_lon)

#         y = math.sin(d_lambda) * math.cos(phi2)
#         x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(d_lambda)
        
#         target_azimuth = math.degrees(math.atan2(y, x))

#         raw_error = quick_angle(target_azimuth - yaw)

#         return raw_error
    
#     if last_error is not None:
#         return last_error
    
#     return 0.0