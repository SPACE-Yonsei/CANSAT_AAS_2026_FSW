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
    
    # GPS 유효성 체크 및 Azimuth 계산
    if is_gps_valid(1, 1) and not (target_lat == 0.0 and target_lon == 0.0):
        target_azimuth = math.degrees(math.atan2(dx, dy))
        if target_azimuth < 0: # 0~360 보정 (코드 스타일에 맞춤)
            target_azimuth += 360
    
    # 날것의 에러 값 계산 (튀는 값 포함됨)
    raw_error = quick_angle(target_azimuth - yaw)

    # ---------------------------------------------------------
    # ▼▼▼ [여기서부터 필터링 코드 추가] ▼▼▼
    # ---------------------------------------------------------

    # 설정값 (상황에 맞춰 조절하세요)
    MAX_CHANGE_PER_LOOP = 10.0  # 한 번에 변할 수 있는 최대 각도 (이 이상 튀면 노이즈로 간주)
    SMOOTHING_FACTOR = 0.3      # 0.1(아주 부드러움/느림) ~ 1.0(빠름/거침)

    # 첫 실행 시 초기화
    if is_first_run:
        prev_filtered_error = raw_error
        is_first_run = False
        return raw_error

    # 2. [안전장치 1] 급격한 변화 제한 (Rate Limiter)
    # "갑자기 60도가 튀었다? 말도 안 돼. 10도만 움직인 걸로 칠게."
    diff = raw_error - prev_filtered_error
    
    if diff > MAX_CHANGE_PER_LOOP:
        raw_error = prev_filtered_error + MAX_CHANGE_PER_LOOP
        print(f"⚠️ Spike ignored: clamped +{MAX_CHANGE_PER_LOOP}")
    elif diff < -MAX_CHANGE_PER_LOOP:
        raw_error = prev_filtered_error - MAX_CHANGE_PER_LOOP
        print(f"⚠️ Spike ignored: clamped -{MAX_CHANGE_PER_LOOP}")

    # 3. [안전장치 2] 부드러운 이동 (Low Pass Filter)
    # 현재 값 30%, 과거 값 70%를 섞어서 출력
    filtered_error = (SMOOTHING_FACTOR * raw_error) + ((1 - SMOOTHING_FACTOR) * prev_filtered_error)
    
    # 다음 계산을 위해 현재 값을 저장
    prev_filtered_error = filtered_error

    # 로그 출력
    print(f"Raw: {raw_error:.1f} -> Filtered: {filtered_error:.1f}")

    return filtered_error
# def calculate_motor_control(yaw: float) -> float:
#     global last_error, last_yaw

#     # if last_yaw is None:
#     #     last_yaw = yaw
#     # else:
#     #     diff = yaw - last_yaw
        
#     #     if abs(diff) > MAX_YAW_CHANGE:
#     #         diff = MAX_YAW_CHANGE if diff > 0 else -MAX_YAW_CHANGE
        
#     #     last_yaw = yaw
#     dx=target_lon - 1
#     dy=target_lat - 1

#     # GPS 유효 → 방위각 계산
#     if is_gps_valid(1, 1) and not (target_lat == 0.0 and target_lon == 0.0):
#         target_azimuth = math.degrees(math.atan2(dx, dy))
#         if target_azimuth<180:
#             target_azimuth=target_azimuth+360
#         error=quick_angle(target_azimuth - yaw)
#         last_error = error
#         print(f"\nyaw={yaw:.1f}, azimuth={target_azimuth:.1f}, error={error:.1f}")
#         return error

#     # GPS 무효 + 목표 좌표 없음 → IMU 기반 기본 헤딩(0도) 유지
#     if target_lat == 0.0 and target_lon == 0.0:
#         return quick_angle(0.0 - yaw)
#     # GPS 무효 but 이전 방위각 있음 → 유지
#     if last_error is not None:
#         return last_error
    
#     # GPS 무효, 이전 방위각 없음 → 직진
#     return 0.0
