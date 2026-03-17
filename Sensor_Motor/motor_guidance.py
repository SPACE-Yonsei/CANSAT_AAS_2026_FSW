#!/usr/bin/env python3
"""
Parafoil Motor Control Module
L1 Carrot Guidance + Cascaded Loop (Outer heading + Inner yaw-rate PI)
"""
import math
import time
import types

cascade_pi = types.SimpleNamespace(
    Kp_outer=0.6,
    Kp_inner=1.0,
    Ki_inner=0.1,
    pi_integral=0.0,
    MAX_INTEGRAL=15.0,
    DEADBAND=5.0
)

# carrot guidance parameters
target = types.SimpleNamespace(lat=0.0, lon=0.0)
start_point = types.SimpleNamespace(lat=0.0, lon=0.0)
LAT_TO_METER = 111320.0
L_DISTANCE = 15.0
wind_effect = 0.0
last_time = None

def init_guidance():
    global wind_effect, last_time
    wind_effect = 0.0
    cascade_pi.pi_integral = 0.0
    last_time = time.time()

#north based angle
def _quick_angle(a: float) -> float:
    return (a + 180) % 360 - 180

# =============================================================================
# GPS Utilities
# =============================================================================
def is_gps_valid(lat: float, lon: float,
                 fix_quality: int = 0, sats: int = 0, rmc_status: str = "V") -> bool:
    coord_ok   = not (lat == 0.0 and lon == 0.0) and abs(lat) <= 90.0 and abs(lon) <= 180.0
    fidelity_ok = fix_quality >= 1 and sats >= 4 and rmc_status == "A"
    return coord_ok and fidelity_ok

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
    N = (lat - start_point.lat) * LAT_TO_METER
    E = (lon - start_point.lon) * LAT_TO_METER * math.cos(math.radians(start_point.lat))
    return N, E

# =============================================================================
# L1 Carrot Guidance
# =============================================================================
def _carrot(tgt_N: float, tgt_E: float) -> tuple[float, float]:
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
    start_point.lat = lat
    start_point.lon = lon

def set_target_coord(lat: float, lon: float):
    target.lat = lat
    target.lon = lon

def draw_pattern():
    return

def guidance(gps_data, imu_data, target_data) -> types.SimpleNamespace:
    """
    L1 Carrot Guidance + Cascaded Control
    """
    global wind_effect, last_time

    prsnt_time = time.time()
    dt = prsnt_time - last_time if last_time else 0.1
    if dt <= 0.02 or dt > 0.5: dt = 0.1
    last_time = prsnt_time

    # -- GPS validity check (Fault Tolerance) --
    if not is_gps_valid(gps_data.lat, gps_data.lon, gps_data.fix_quality, gps_data.sats, gps_data.rmc_status):
        return types.SimpleNamespace(u_cmd=0.0, error=0.0, state="GPS_INVALID")

    # -- [1] L1 Carrot Guidance --
    my_N, my_E = _llh_to_ne(gps_data.lat, gps_data.lon)
    tgt_N, tgt_E = _llh_to_ne(target_data.lat, target_data.lon)
    distance = math.hypot(tgt_N - my_N, tgt_E - my_E)
    if distance < 5.0: return types.SimpleNamespace(u_cmd=0.0, error=0.0, state="TARGET_REACHED")

    cN, cE = _carrot(tgt_N - my_N, tgt_E - my_E)
    desired_course = math.degrees(math.atan2(cE, cN))

    # -- [2] Wind Compensation (crab angle estimation) --
    if gps_data.speed > 1.0 and abs(imu_data.gyrz) < 20.0:
        current_crab = _quick_angle(gps_data.course - imu_data.yaw)
        wind_effect = 0.95 * wind_effect + 0.05 * current_crab

    desired_heading = _quick_angle(desired_course - wind_effect)

    # -- [3] Outer Loop: heading error -> desired yaw rate --
    heading_error = _quick_angle(desired_heading - imu_data.yaw)
    # 물리 공식 반영: V가 빠르고 L이 짧을수록 강하게 꺾음 (단위: rad/s -> deg/s)
    V = max(gps_data.speed, 1.0)
    desired_yaw_rate_rad = 2.0 * (V / L_DISTANCE) * math.sin(math.radians(heading_error))
    desired_yaw_rate = math.degrees(desired_yaw_rate_rad)

    # -- [4] Inner Loop: PI (yaw rate error -> u) --
    rate_error = desired_yaw_rate - imu_data.gyrz

    if abs(heading_error) <= cascade_pi.DEADBAND:
        cascade_pi.pi_integral = 0.0
        u = 0.0
        state_msg = "STRAIGHT"
    else:
        cascade_pi.pi_integral = max(
            min(cascade_pi.pi_integral + rate_error * dt, cascade_pi.MAX_INTEGRAL),
            -cascade_pi.MAX_INTEGRAL
        )
        u = cascade_pi.Kp_inner * rate_error + cascade_pi.Ki_inner * cascade_pi.pi_integral
        state_msg = "TURNING"

    return types.SimpleNamespace(u_cmd=u, error=heading_error, distance=distance, state=state_msg)

def reset_control():
    global wind_effect, last_time
    wind_effect = 0.0
    cascade_pi.pi_integral = 0.0
    last_time = time.time()


"""주요 변경 사항 요약:

| 항목 | 변경 전 | 변경 후 |
|------|---------|---------|
| `guidance` 시그니처 | `(pi, yaw, gyro_z, current_lat, ...)` 10개 파라미터 | `(gps_data, imu_data, target_data)` 3개 객체 |
| GPS 실패 반환 | `return` (None) | `SimpleNamespace(u_cmd=0.0, error=0.0, state="GPS_INVALID")` |
| 목표 도달 조건 | 없음 | `distance < 5.0` → `state="TARGET_REACHED"` |
| Outer Loop | `Kp_outer * heading_error` | L1 물리 공식: `2V/L * sin(heading_error)` |
| 반환값 | 없음 (누락) | `SimpleNamespace(u_cmd, error, distance, state)` |
| `set_start_coordinates` | `global start_lat, start_lon` | `start_point.lat/lon` (SimpleNamespace) |
| `_llh_to_ne` | `start_lat/start_lon` 전역 참조 | `start_point.lat/lon` |
"""