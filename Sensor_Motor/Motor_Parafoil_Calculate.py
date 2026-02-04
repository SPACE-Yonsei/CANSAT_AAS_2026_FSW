"""
파라포일 모터 제어 모듈
IMU/GPS 데이터 기반 turn angle 계산
"""

import math
import os
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

def _nonlinear_scale(error: float) -> float:
    """
    Piecewise non-linear scaling function
    - Dead zone: |error| < DEAD_ZONE_OUTER → gentle scaling
    - Medium errors: linear region
    - Large errors: aggressive scaling
    """
    abs_error = abs(error)
    sign = 1 if error >= 0 else -1

    if abs_error < DEAD_ZONE_INNER:
        # Hard dead zone: zero output
        return 0.0
    elif abs_error < DEAD_ZONE_OUTER:
        # Soft dead zone: gentle quadratic ramp
        normalized = (abs_error - DEAD_ZONE_INNER) / (DEAD_ZONE_OUTER - DEAD_ZONE_INNER)
        return sign * DEAD_ZONE_OUTER * (normalized ** 2)
    elif abs_error < ERROR_THRESHOLD_HIGH:
        # Linear region
        return error * KP_LINEAR
    else:
        # Aggressive region: sqrt-based boost
        excess = abs_error - ERROR_THRESHOLD_HIGH
        linear_component = ERROR_THRESHOLD_HIGH * KP_LINEAR
        aggressive_component = (excess ** 0.5) * KP_AGGRESSIVE
        return sign * (linear_component + aggressive_component)

def quick_angle(angle: float) -> float:
    """각도를 -180 ~ +180 범위로 정규화"""
    while angle >= 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def calculate_motor_control_pd(yaw: float) -> float:
    """
    Non-linear PD control for parafoil steering
    Returns: error signal for motor control (-135 to +135 degrees)
    """
    global prev_filtered_error, prev_raw_error, is_first_run, last_error

    dx = target_lon - 1
    dy = target_lat - 1

    # 목표 좌표 없음 → 중립
    if target_lat == 0.0 and target_lon == 0.0:
        return 0.0

    # GPS 무효 → 마지막 에러 유지
    if not is_gps_valid(1, 1):
        return last_error if last_error is not None else 0.0

    # Target azimuth 계산
    target_azimuth = math.degrees(math.atan2(dx, dy))
    if target_azimuth < 0:
        target_azimuth += 360

    raw_error = quick_angle(target_azimuth - yaw)

    # 첫 실행: 초기화
    if is_first_run:
        prev_filtered_error = raw_error
        prev_raw_error = raw_error
        is_first_run = False
        last_error = raw_error
        return raw_error

    # Spike detection (기존 로직 유지)
    raw_diff = quick_angle(raw_error - prev_raw_error)
    if abs(raw_diff) > MAX_CHANGE:
        limited_error = prev_filtered_error + (MAX_CHANGE if raw_diff > 0 else -MAX_CHANGE)
        print(f"⚠️ Spike: raw_diff={raw_diff:.1f}, clamped")
    else:
        limited_error = raw_error

    # Non-linear scaling
    scaled_error = _nonlinear_scale(limited_error)

    # Derivative term
    derivative = quick_angle(limited_error - prev_raw_error) * KD

    # Combine P + D
    pd_output = scaled_error + derivative

    # Low Pass Filter
    filtered_error = (ALPHA * pd_output) + ((1 - ALPHA) * prev_filtered_error)

    # Clamp output
    filtered_error = max(-135, min(135, filtered_error))

    # Update state
    prev_raw_error = raw_error
    prev_filtered_error = filtered_error
    last_error = filtered_error

    print(f"////yaw={yaw:.1f}, az={target_azimuth:.1f}, raw={raw_error:.1f}, "
          f"scaled={scaled_error:.1f}, D={derivative:.1f}, out={filtered_error:.1f}")

    return filtered_error

# 전역 변수
prev_filtered_error = 0.0
prev_raw_error = 0.0
is_first_run = True
last_error = None

# 설정값 (환경변수 또는 기본값)
# Rate limiting & filtering
MAX_CHANGE = float(os.getenv("PARAFOIL_MAX_CHANGE", "15.0"))
ALPHA = float(os.getenv("PARAFOIL_ALPHA", "0.3"))

# Dead zone
DEAD_ZONE_INNER = float(os.getenv("PARAFOIL_DEAD_ZONE_INNER", "5.0"))
DEAD_ZONE_OUTER = float(os.getenv("PARAFOIL_DEAD_ZONE_OUTER", "15.0"))

# Proportional gains
KP_LINEAR = float(os.getenv("PARAFOIL_KP_LINEAR", "1.0"))
KP_AGGRESSIVE = float(os.getenv("PARAFOIL_KP_AGGRESSIVE", "8.0"))

# Thresholds
ERROR_THRESHOLD_HIGH = float(os.getenv("PARAFOIL_ERROR_THRESHOLD_HIGH", "45.0"))

# Derivative gain
KD = float(os.getenv("PARAFOIL_KD", "0.15"))

# Control mode
CONTROL_MODE = os.getenv("PARAFOIL_CONTROL_MODE", "NONLINEAR_PD")  # or "LINEAR"


def calculate_motor_control(yaw: float) -> float:
    # Mode selection: use NONLINEAR_PD or fall back to LINEAR
    if CONTROL_MODE == "NONLINEAR_PD":
        return calculate_motor_control_pd(yaw)

    # LINEAR mode (기존 로직)
    global prev_filtered_error, prev_raw_error, is_first_run, last_error

    dx = target_lon - 1
    dy = target_lat - 1

    # 목표 좌표 없음 → 직진 (yaw=0 유지)
    if target_lat == 0.0 and target_lon == 0.0:
        return quick_angle(0.0 - yaw)
    
    # GPS 유효 → 방위각 계산
    if is_gps_valid(1, 1):
        target_azimuth = math.degrees(math.atan2(dx, dy))
        if target_azimuth < 0:
            target_azimuth += 360
        
        raw_error = quick_angle(target_azimuth - yaw)
        return raw_error
        # # 첫 실행: 초기화
        # if is_first_run:
        #     prev_filtered_error = raw_error
        #     prev_raw_error = raw_error
        #     is_first_run = False
        #     last_error = raw_error
        #     print(f"yaw={yaw:.1f}, azimuth={target_azimuth:.1f}, error={raw_error:.1f} (init)")
        #     return raw_error
        
        # # raw_error 변화량 계산 (래핑 고려)
        # raw_diff = quick_angle(raw_error - prev_raw_error)
        
        # # 급변 감지 시에만 Rate Limit 적용
        # if abs(raw_diff) > MAX_CHANGE:
        #     if raw_diff > 0:
        #         limited_error = prev_filtered_error + MAX_CHANGE
        #     else:
        #         limited_error = prev_filtered_error - MAX_CHANGE
        #     print(f"⚠️ Spike: raw_diff={raw_diff:.1f}, yaw={yaw:.1f}, azimuth={target_azimuth:.1f}, error={raw_error:.1f}, clamped")
        # else:
        #     limited_error = raw_error
        
        # # Low Pass Filter
        # filtered_error = (ALPHA * limited_error) + ((1 - ALPHA) * prev_filtered_error)
        
        # # 상태 업데이트
        # prev_raw_error = raw_error
        # prev_filtered_error = filtered_error
        # last_error = filtered_error
        
        # print(f"////yaw={yaw:.1f}, azimuth={target_azimuth:.1f}, raw={raw_error:.1f}, filtered={filtered_error:.1f}")
        # return filtered_error
    
    # GPS 무효 but 이전 에러 있음 → 유지
    if last_error is not None:
        return last_error
    
    return 0.0