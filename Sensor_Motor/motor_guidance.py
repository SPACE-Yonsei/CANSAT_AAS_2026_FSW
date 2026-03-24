#!/usr/bin/env python3
import math
import time
import types
from datetime import datetime
from typing import Optional

_sim_log = open("0320_sim.txt", "a")

def _dbg(line: str):
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    full = f"[{ts}] {line}"
    print(full)
    _sim_log.write(full + "\n")
    _sim_log.flush()

cascade_pi = types.SimpleNamespace(
    Kp_outer      = 0.6,   # unitless
    Kp_inner      = 1.0,   # unitless
    Ki_inner      = 0.1,   # 1/s
    pi_integral   = 0.0,   # °/s·s, 적분 누적값
    MAX_INTEGRAL  = 15.0,  # °/s·s, 적분 상한
    DEADBAND      = 5.0,   # deg, heading error 허용 범위
    MAX_CMD       = 120.0, # °/s, 최대 yaw rate 명령
)

target = types.SimpleNamespace(
    lat  = None,  # Optional[float] — deg, decimal degrees
    lon  = None,  # Optional[float] — deg, decimal degrees
)
start_point = types.SimpleNamespace(
    lat  = None,  # Optional[float] — deg, decimal degrees
    lon  = None,  # Optional[float] — deg, decimal degrees
)

LAT_TO_METER: float = 111320.0  # m/deg

L_DISTANCE: float      = 25.0   # m, L1 추적 거리
L_DISTANCE_BASE: float = 25.0   # m
L_DISTANCE_HIGH: float = 40.0   # m, 고고도용
L_DISTANCE_LOW: float  = 10.0   # m, 저고도용

PATTERN_ENTRY_DIST: float = 50.0  # m — figure-8 진입 거리

ALT_HIGH: int = 300  # m
ALT_LOW: int  = 150  # m

wind_effect: Optional[float] = None  # deg, 풍향 보정값
last_time: Optional[float]   = None  # s, time.time() epoch

DEBUG_GUIDANCE: bool = True

_pattern = types.SimpleNamespace(
    lobe_sign       = 1,    # int   — +1 또는 -1
    last_switch_time = 0.0, # s, time.time() epoch
    LOBE_PERIOD     = 25.0, # s
    RADIUS          = 25.0, # m
)

# GPS 순간 이동 감지용 상태
_prev_gps = types.SimpleNamespace(
    lat         = None,   # Optional[float] — deg
    lon         = None,   # Optional[float] — deg
    time        = None,   # Optional[float] — s, epoch
    initialized = False,  # bool
)
GPS_JUMP_MAX_SPEED: float = 50.0        # m/s
GPS_STABLE_COUNT_REQUIRED: int = 2      # 샘플 수
_gps_stable_count: int = 0



def init_guidance():
    global wind_effect, last_time, L_DISTANCE, _gps_stable_count
    wind_effect = 0.0
    L_DISTANCE = L_DISTANCE_BASE
    cascade_pi.pi_integral = 0.0
    last_time = time.time()
    _pattern.lobe_sign = 1
    _pattern.last_switch_time = time.time()
    _prev_gps.lat = None
    _prev_gps.lon = None
    _prev_gps.time = None
    _prev_gps.initialized = False
    _gps_stable_count = 0

def reset_control():
    global wind_effect, last_time
    wind_effect = 0.0
    cascade_pi.pi_integral = 0.0
    last_time = time.time()


def _wrap_180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def is_gps_valid(gps_vector, gps_fidelity) -> bool:
    if (gps_vector.lat is None or gps_vector.lon is None
            or gps_fidelity.fix_quality is None
            or gps_fidelity.sats is None
            or gps_fidelity.rmc_status is None):
        return False
    coord_ok = (gps_vector.lat != 0.0
                and gps_vector.lon != 0.0
                and abs(gps_vector.lat) <= 90.0
                and abs(gps_vector.lon) <= 180.0)
    fidelity_ok = (gps_fidelity.fix_quality >= 1
                   and gps_fidelity.sats >= 4
                   and gps_fidelity.rmc_status == "A")
    return coord_ok and fidelity_ok


def is_gps_jump(lat: float, lon: float) -> bool:
    """
    Fix 직후 불안정 샘플 거부 + 비현실적 순간 이동 거부.
    True를 반환하면 이번 좌표를 사용하지 않아야 함.
    """
    global _gps_stable_count

    now = time.time()

    if not _prev_gps.initialized:
        _prev_gps.lat = lat
        _prev_gps.lon = lon
        _prev_gps.time = now
        _prev_gps.initialized = True
        _gps_stable_count = 1
        return True  # 첫 Fix는 사용하지 않음

    # Fix 직후 안정화 대기
    if _gps_stable_count < GPS_STABLE_COUNT_REQUIRED:
        _gps_stable_count += 1
        _prev_gps.lat = lat
        _prev_gps.lon = lon
        _prev_gps.time = now
        return True

    dt = now - _prev_gps.time
    if dt < 0.01:
        dt = 0.01

    dist = calculate_distance_haversine(_prev_gps.lat, _prev_gps.lon, lat, lon)
    speed = dist / dt

    _prev_gps.lat = lat
    _prev_gps.lon = lon
    _prev_gps.time = now

    if speed > GPS_JUMP_MAX_SPEED:
        _gps_stable_count = 0  # 점프 발생 → 안정화 카운터 리셋
        return True

    return False


def calculate_distance_haversine(lat1: float, lon1: float,
                                 lat2: float, lon2: float) -> float:
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)
    a = (math.sin(d_phi / 2.0) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2.0) ** 2)
    return R * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def _llh_to_en(lat: float, lon: float) -> tuple:
    if start_point.lat is None or start_point.lon is None:
        return 0.0, 0.0
    N = (lat - start_point.lat) * LAT_TO_METER
    E = (lon - start_point.lon) * LAT_TO_METER * math.cos(math.radians(start_point.lat))
    return E, N


def _carrot(my_E: float, my_N: float,
            tgt_E: float, tgt_N: float) -> tuple:
    rope_len = math.hypot(tgt_E, tgt_N)
    if rope_len < 0.01:
        return tgt_E, tgt_N

    uE = tgt_E / rope_len
    uN = tgt_N / rope_len

    s = max(0.0, min(my_E * uE + my_N * uN, rope_len))

    carrot_s = s + L_DISTANCE
    return carrot_s * uE, carrot_s * uN


def _eight(my_E: float, my_N: float,
                         tgt_E: float, tgt_N: float) -> tuple:
    now = time.time()

    if now - _pattern.last_switch_time > _pattern.LOBE_PERIOD:
        _pattern.lobe_sign *= -1
        _pattern.last_switch_time = now

    bearing_to_tgt = math.atan2(tgt_E - my_E, tgt_N - my_N)

    perp_angle = bearing_to_tgt + math.pi / 2.0

    offset_E = _pattern.RADIUS * _pattern.lobe_sign * math.sin(perp_angle)
    offset_N = _pattern.RADIUS * _pattern.lobe_sign * math.cos(perp_angle)

    return tgt_E + offset_E, tgt_N + offset_N


def set_start_coordinates(lat: float, lon: float):
    start_point.lat = lat
    start_point.lon = lon


def set_target_coord(lat: float, lon: float):
    target.lat = lat
    target.lon = lon


def _yaw_rate_pi_control(desired_yaw_rate, measured_yaw_rate, dt):
    rate_error = desired_yaw_rate - measured_yaw_rate
    max_cmd = cascade_pi.MAX_CMD

    u = cascade_pi.Kp_inner * rate_error + cascade_pi.Ki_inner * cascade_pi.pi_integral

    if abs(u) < max_cmd or (rate_error * u < 0):
        cascade_pi.pi_integral += rate_error * dt
        cascade_pi.pi_integral = max(-cascade_pi.MAX_INTEGRAL,
                                     min(cascade_pi.MAX_INTEGRAL, cascade_pi.pi_integral))
        u = cascade_pi.Kp_inner * rate_error + cascade_pi.Ki_inner * cascade_pi.pi_integral

    u_sat = max(-max_cmd, min(max_cmd, u))
    return u_sat


def guidance(imu_data, gps_vector, gps_fidelity, target_data,
             baro_m: float = 0.0, patterned: bool = False) -> types.SimpleNamespace:
    global wind_effect, last_time, L_DISTANCE

    now = time.time()
    dt = now - last_time if last_time else 0.1
    if dt <= 0.02 or dt > 0.5:
        dt = 0.1
    last_time = now

    if not is_gps_valid(gps_vector, gps_fidelity):
        if DEBUG_GUIDANCE:
            _dbg(f"[CTRL] GPS_INVALID — lat={gps_vector.lat} lon={gps_vector.lon} "
                 f"fix={gps_fidelity.fix_quality} sats={gps_fidelity.sats} rmc={gps_fidelity.rmc_status}")
        return types.SimpleNamespace(state="GPS_INVALID", distance=0.0, commanded_yaw_rate=0.0)

    if is_gps_jump(gps_vector.lat, gps_vector.lon):
        if DEBUG_GUIDANCE:
            _dbg(f"[CTRL] GPS_JUMP — lat={gps_vector.lat:.6f} lon={gps_vector.lon:.6f}")
        return types.SimpleNamespace(state="GPS_INVALID", distance=0.0, commanded_yaw_rate=0.0)
    
    if baro_m <= 0.0:
        if DEBUG_GUIDANCE:
            _dbg(f"[CTRL] BARO_INVALID — baro_m={baro_m:.1f}")
        return types.SimpleNamespace(state="BARO_INVALID", distance=0.0, commanded_yaw_rate=0.0)

    my_E, my_N = _llh_to_en(gps_vector.lat, gps_vector.lon)
    tgt_E, tgt_N = _llh_to_en(target_data.lat, target_data.lon)
    distance = math.hypot(tgt_E - my_E, tgt_N - my_N)

    if distance < 5.0:
        if DEBUG_GUIDANCE:
            _dbg(f"[CTRL] TARGET_REACHED — dist={distance:.1f}m")
        return types.SimpleNamespace(state="TARGET_REACHED", distance=distance, commanded_yaw_rate=0.0)

    if baro_m > ALT_HIGH:
        L_DISTANCE = L_DISTANCE_HIGH
    elif baro_m < ALT_LOW:
        L_DISTANCE = L_DISTANCE_LOW
    else:
        L_DISTANCE = L_DISTANCE_BASE

    if patterned and distance < PATTERN_ENTRY_DIST:
        guide_E, guide_N = _eight(my_E, my_N, tgt_E, tgt_N)
        phase = "PATTERN"
    else:
        guide_E, guide_N = _carrot(my_E, my_N, tgt_E, tgt_N)
        phase = "HOMING"

    carrot_angl_north = math.degrees(
        math.atan2(guide_E - my_E, guide_N - my_N)
    )
    wind_carrot_angl_north = _wrap_180(carrot_angl_north - wind_effect)
    angl_to_turn = _wrap_180(wind_carrot_angl_north - imu_data.yaw)

    V = max(gps_vector.speed, 1.0)
    desired_yaw_rate = math.degrees(
        2.0 * (V / L_DISTANCE) * math.sin(math.radians(angl_to_turn))
    )

    if abs(angl_to_turn) <= cascade_pi.DEADBAND:
        cascade_pi.pi_integral = 0.0
        commanded_yaw_rate = 0.0
        if phase == "HOMING":
            phase = "STRAIGHT"
        # Wind learning only when flying straight — no turn contamination
        if gps_vector.speed > 1.0:
            current_crab = _wrap_180(gps_vector.course - imu_data.yaw)
            wind_effect = 0.85 * wind_effect + 0.15 * current_crab
            wind_effect = max(-45.0, min(45.0, wind_effect))
    else:
        commanded_yaw_rate = _yaw_rate_pi_control(
            desired_yaw_rate, math.degrees(imu_data.gyrz), dt
        )
        if phase == "HOMING":
            phase = "TURNING"

    if DEBUG_GUIDANCE:
        _dbg(
            f"[CTRL] phase={phase:<8} "
            f"dist={distance:.1f}m  L={L_DISTANCE:.1f}m | "
            f"pos=({my_E:.1f},{my_N:.1f})  tgt=({tgt_E:.1f},{tgt_N:.1f})  "
            f"carrot=({guide_E:.1f},{guide_N:.1f}) | "
            f"des_crs={carrot_angl_north:.1f}°  wind={wind_effect:.1f}°  "
            f"des_hdg={wind_carrot_angl_north:.1f}°  hdg_err={angl_to_turn:.1f}° | "
            f"V={V:.1f}m/s  des_yr={desired_yaw_rate:.2f}°/s  "
            f"pi_int={cascade_pi.pi_integral:.3f}  cmd_yr={commanded_yaw_rate:.2f}°/s"
            + (f"  lobe={_pattern.lobe_sign:+d}" if patterned else "")
        )

    return types.SimpleNamespace(
        state=phase,
        distance=distance,
        commanded_yaw_rate=commanded_yaw_rate
    )
