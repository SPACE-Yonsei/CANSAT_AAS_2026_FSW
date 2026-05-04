#!/usr/bin/env python3
import math
import os
import time
import types
from datetime import datetime
from typing import Optional

_SIM_LOG_PATH = os.getenv("CANSAT_SIM_LOG", datetime.now().strftime("%m%d_sim.txt"))
_sim_log = open(_SIM_LOG_PATH, "a", encoding="utf-8")

def _dbg(line: str):
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    full = f"[{ts}] {line}"
    print(full)
    _sim_log.write(full + "\n")
    _sim_log.flush()

# ── Outer-loop 파라미터 ────────────────────────────────────────────
# tanh 포화 기반 heading error → desired_yaw_rate 변환
YR_MAX: float            = 45.0   # deg/s  outer-loop 포화 상한
                                   # actuator 선형 한계 60 deg/s의 75% - PI 과도응답 여유
CAPTURE_THRESHOLD: float = 45.0   # deg    이 이상이면 capture mode (integral freeze)
                                   # 소각 track mode와 대각도 capture mode의 경계
LANDING_YR_MAX: float    = 20.0   # deg/s  저고도 보수 클램프 상한
LANDING_ALT: float       = 20.0   # m      이 고도 이하에서 LANDING 보수 모드 적용

cascade_pi = types.SimpleNamespace(
    Kp_inner      = 1.3,   # inner-loop 비례 게인 (yaw rate error → command)
    Ki_inner      = 0.05,  # 1/s  적분 게인 (steady-state offset 제거)
    pi_integral   = 0.0,   # °/s·s  적분 누적값
    MAX_INTEGRAL  = 10.0,  # °/s·s  적분 클램프 (YR_MAX 기준으로 축소)
                            # 이유: YR_MAX=45이므로 기존 15보다 작아도 충분
    DEADBAND      = 5.0,   # deg  heading error 불감대
    MAX_CMD       = 60.0,  # °/s  actuator 유효 선형 한계 (실측 ±60 deg/s)
    last_cmd      = 0.0,   # °/s  slew limiter용 이전 명령값
    MAX_ACCEL     = 150.0, # °/s²  slew rate - 200에서 축소하여 급변 추가 억제
)

target = types.SimpleNamespace(
    lat  = None,  # Optional[float] - deg, decimal degrees
    lon  = None,  # Optional[float] - deg, decimal degrees
)
start_point = types.SimpleNamespace(
    lat  = None,  # Optional[float] - deg, decimal degrees
    lon  = None,  # Optional[float] - deg, decimal degrees
)

LAT_TO_METER: float = 111320.0  # m/deg

L_DISTANCE_BASE: float = 25.0   # m
L_DISTANCE_HIGH: float = 40.0   # m, 고고도용
L_DISTANCE_LOW: float  = 15.0   # m, 저고도용
L_DISTANCE: float      = L_DISTANCE_BASE   # m, L1 추적 거리

PATTERN_ENTRY_DIST: float = 50.0  # m - figure-8 진입 거리

ALT_HIGH: int = 300  # m
ALT_LOW: int  = 150  # m

wind_effect: Optional[float] = None  # deg, 풍향 보정값
last_time: Optional[float]   = None  # s, time.time() epoch

DEBUG_GUIDANCE: bool = True

_pattern = types.SimpleNamespace(
    lobe_sign       = 1,    # int   - +1 또는 -1
    last_switch_time = 0.0, # s, time.time() epoch
    LOBE_PERIOD     = 25.0, # s
    RADIUS          = 25.0, # m
)

# GPS 순간 이동 감지용 상태
_prev_gps = types.SimpleNamespace(
    lat         = None,   # Optional[float] - deg
    lon         = None,   # Optional[float] - deg
    time        = None,   # Optional[float] - s, epoch
    initialized = False,  # bool
)
GPS_JUMP_MAX_SPEED: float = 200.0        # m/s
GPS_STABLE_COUNT_REQUIRED: int = 2      # 샘플 수
_gps_stable_count: int = 0

TARGET_REACHED_RADIUS: float = 10.0    # m, 목표 도달 판정 반경
PATTERN_ALT_MIN: float       = 10.0   # m, 패턴 비행 진입 최소 고도
PATTERN_ALT_MAX: float       = 50.0   # m, 패턴 비행 진입 최대 고도
WIND_LEARN_MIN_SPEED: float  = 2.5    # m/s, wind 학습 최소 GPS 속도
WIND_EMA_ALPHA: float        = 0.15   # wind EMA 학습률 (0=고정, 1=즉시 반영)
WIND_MAX_DEG: float          = 45.0   # deg, wind_effect 최대 보정각
DT_MIN: float = 0.02                  # s, guidance dt 하한
DT_MAX: float = 0.5                   # s, guidance dt 상한

def init_guidance():
    global wind_effect, last_time, L_DISTANCE, _gps_stable_count
    wind_effect = 0.0
    L_DISTANCE = L_DISTANCE_BASE
    cascade_pi.pi_integral = 0.0
    cascade_pi.last_cmd = 0.0  # Slew Rate 초기화 추가
    last_time = time.time()
    _pattern.lobe_sign = 1
    _pattern.last_switch_time = time.time()
    _prev_gps.lat         = None
    _prev_gps.lon         = None
    _prev_gps.time        = None
    _prev_gps.initialized = False
    _gps_stable_count = 0
    start_point.lat = None
    start_point.lon = None

def reset_control():
    global wind_effect, last_time
    wind_effect = 0.0
    cascade_pi.pi_integral = 0.0
    cascade_pi.last_cmd = 0.0  # Slew Rate 초기화 추가
    last_time = time.time()

def _wrap_180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def reset_control() -> None:
    """Test utility: reset controller integrator and GPS gate states."""
    global _last_gps, _gps_stable_count, _integral_yr, _last_cmd_yr, _last_ctrl_ts
    _last_gps = None
    _gps_stable_count = 0
    _integral_yr = 0.0
    _last_cmd_yr = 0.0
    _last_ctrl_ts = None


def is_gps_valid(gps_vector, gps_fidelity) -> bool:
    if hasattr(gps_fidelity, "pos_health"):
        return (
            gps_vector.lat is not None
            and gps_vector.lon is not None
            and gps_vector.lat != 0.0
            and gps_vector.lon != 0.0
            and abs(gps_vector.lat) <= 90.0
            and abs(gps_vector.lon) <= 180.0
            and int(gps_fidelity.pos_health) > 0
        )
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


def _gps_velocity(gps_vector) -> float:
    return float(getattr(gps_vector, "velocity", getattr(gps_vector, "speed", 0.0)))


def _gps_direction(gps_vector) -> float:
    return float(getattr(gps_vector, "direction", getattr(gps_vector, "course", 0.0)))


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
    """위도/경도 → start_point 기준 East/North (m) 변환."""
    if start_point.lat is None or start_point.lon is None:
        return 0.0, 0.0
    N = (lat - start_point.lat) * LAT_TO_METER
    E = (lon - start_point.lon) * LAT_TO_METER * math.cos(math.radians(start_point.lat))
    return E, N


def _carrot(my_E: float, my_N: float,
            tgt_E: float, tgt_N: float) -> tuple:
    """
    L1 경로추적: start_point(원점) → target 기준선 위에서
    차량 투영점 기준 L_DISTANCE 전방에 carrot 배치.
    """
    # 기준선 방향 단위벡터 (원점 = start_point = EN 원점)
    line_len = math.hypot(tgt_E, tgt_N)
    if line_len < 0.01:
        return tgt_E, tgt_N
    uE = tgt_E / line_len
    uN = tgt_N / line_len

    # 차량의 기준선 위 투영 거리
    s = my_E * uE + my_N * uN

    # carrot = 투영점에서 L_DISTANCE 전방, [origin, target] 범위로 클램프
    # max(0.0,...): 기체가 start_point 뒤에 있을 때 carrot이 역방향으로 배치되는 것 방지
    s_carrot = max(0.0, min(s + L_DISTANCE, line_len))
    if s < 0.0:
        _dbg(f"[CARROT] vehicle behind origin: s={s:.1f}m -> clamped to {s_carrot:.1f}m")
    return s_carrot * uE, s_carrot * uN


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

def _outer_loop(angl_to_turn: float, V: float, L: float) -> float:
    """
    tanh 기반 outer-loop: heading error → desired_yaw_rate.

    설계 원칙:
      - 소각(track mode): tanh ≈ K*err → L1 공식(2V/L * err)과 동일한 감도
      - 대각도(capture mode): tanh 자연 포화 → ±YR_MAX 로 부드럽게 수렴
      - 90° 하드 스위치 없음, 전 구간 C∞ 연속
      - 출력 범위 ⊆ [−YR_MAX, +YR_MAX] ⊂ [−MAX_CMD, +MAX_CMD]

    K_outer = 2V / (L * YR_MAX)  [per degree]
      → tanh 선형 근사 기울기가 L1 소각 감도와 일치
    """
    # L1 소각 감도 매칭: YR_MAX * K = 2V/L  →  K = 2V/(L*YR_MAX)
    K = 2.0 * V / (L * YR_MAX)
    return YR_MAX * math.tanh(K * angl_to_turn)


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
    
    # ----------------------------------------------------
    # Slew Rate Limiter (변화율 제한기) 추가
    # ----------------------------------------------------
    max_delta = cascade_pi.MAX_ACCEL * dt
    cmd_delta = u_sat - cascade_pi.last_cmd

    if cmd_delta > max_delta:
        u_sat = cascade_pi.last_cmd + max_delta
    elif cmd_delta < -max_delta:
        u_sat = cascade_pi.last_cmd - max_delta

    cascade_pi.last_cmd = u_sat
    # ----------------------------------------------------

    return u_sat

_guidance_tick = 0


def guidance(imu_data, gps_vector, gps_fidelity, target,
             baro_m: float = 0.0) -> types.SimpleNamespace:
    global wind_effect, last_time, L_DISTANCE, _guidance_tick
    _guidance_tick += 1
    commanded_yaw_rate: float = 0.0  # 미초기화 참조 방지

    now = time.time()
    dt = now - last_time if last_time else 0.1
    if dt <= DT_MIN or dt > DT_MAX:
        dt = 0.1
    last_time = now

    if not is_gps_valid(gps_vector, gps_fidelity):
        if DEBUG_GUIDANCE:
            if hasattr(gps_fidelity, "pos_health"):
                gps_detail = (
                    f"pos_health={getattr(gps_fidelity, 'pos_health', 0)} "
                    f"motion_health={getattr(gps_fidelity, 'motion_health', 0)}"
                )
            else:
                gps_detail = (
                    f"fix={gps_fidelity.fix_quality} sats={gps_fidelity.sats} "
                    f"rmc={gps_fidelity.rmc_status}"
                )
            _dbg(f"[CTRL] GPS_INVALID - lat={gps_vector.lat} lon={gps_vector.lon} "
                 f"{gps_detail}")
        return types.SimpleNamespace(state="GPS_INVALID", distance=0.0, commanded_yaw_rate=0.0)

    if is_gps_jump(gps_vector.lat, gps_vector.lon):
        _dbg(f"[CTRL] GPS_JUMP - lat={gps_vector.lat:.6f} lon={gps_vector.lon:.6f} "
             f"pi_int_before={cascade_pi.pi_integral:.3f}")
        cascade_pi.pi_integral = 0.0
        return types.SimpleNamespace(state="GPS_INVALID", distance=0.0, commanded_yaw_rate=0.0)
    
    if baro_m <= 0.0:
        if DEBUG_GUIDANCE:
            _dbg(f"[CTRL] BARO_INVALID - baro_m={baro_m:.1f}")
        return types.SimpleNamespace(state="BARO_INVALID", distance=0.0, commanded_yaw_rate=0.0)

    my_E, my_N = _llh_to_en(gps_vector.lat, gps_vector.lon)
    tgt_E, tgt_N = _llh_to_en(target.lat, target.lon)

    if start_point.lat is None or start_point.lon is None:
        _dbg(f"[CTRL] START_POINT_UNSET - waiting for valid GPS fix to set origin")
        return types.SimpleNamespace(state="START_UNSET", distance=0.0, commanded_yaw_rate=0.0)

    distance = math.hypot(tgt_E - my_E, tgt_N - my_N)

    if distance < TARGET_REACHED_RADIUS:
        if DEBUG_GUIDANCE:
            _dbg(f"[CTRL] TARGET_REACHED - dist={distance:.1f}m")
        return types.SimpleNamespace(state="TARGET_REACHED", distance=distance, commanded_yaw_rate=0.0)

    if baro_m > ALT_HIGH:
        L_DISTANCE = L_DISTANCE_HIGH
    elif baro_m < ALT_LOW:
        L_DISTANCE = L_DISTANCE_LOW
    else:
        L_DISTANCE = L_DISTANCE_BASE

    # 고도에 따른 페이즈 결정:
    #   > PATTERN_ALT_MAX(50m): carrot 직선 추적
    #   PATTERN_ALT_MIN(10m) ~ PATTERN_ALT_MAX(50m): figure-8 (타겟 근처일 때)
    #   < PATTERN_ALT_MIN(10m): carrot 직선 추적 (최종 접근)
    patterned = PATTERN_ALT_MIN < baro_m < PATTERN_ALT_MAX

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

    V = max(_gps_velocity(gps_vector), 1.0)

    # ── Capture mode: 대각도 오차 시 integral freeze ──────────────────
    # |error| > CAPTURE_THRESHOLD 이면 PI 적분기를 0으로 리셋.
    # 이유: 대각도 선회 중 적분이 쌓이면 목표 통과 후 과도한 반대 제어 유발.
    if abs(angl_to_turn) > CAPTURE_THRESHOLD:
        cascade_pi.pi_integral = 0.0

    # ── Outer loop: tanh 기반 desired_yaw_rate 계산 ───────────────────
    desired_yaw_rate = _outer_loop(angl_to_turn, V, L_DISTANCE)

    # ── Inner loop: PI 제어 ───────────────────────────────────────────
    commanded_yaw_rate = _yaw_rate_pi_control(
        desired_yaw_rate, math.degrees(imu_data.gyrz), dt
    )

    # ── 저고도 보수 클램프 (LANDING 보수 모드) ────────────────────────
    # 지면 근처에서 과격한 제어로 인한 기체 불안정 방지.
    if baro_m <= LANDING_ALT:
        commanded_yaw_rate = max(-LANDING_YR_MAX,
                                 min(LANDING_YR_MAX, commanded_yaw_rate))

    if phase == "HOMING":
        if abs(angl_to_turn) <= 15.0:
            phase = "STRAIGHT"
        else:
            phase = "TURNING"

    # 바람 학습 (Wind Learning)
    if phase == "STRAIGHT" and _gps_velocity(gps_vector) > WIND_LEARN_MIN_SPEED:
        current_crab = _wrap_180(_gps_direction(gps_vector) - imu_data.yaw)
        wind_effect = (1.0 - WIND_EMA_ALPHA) * wind_effect + WIND_EMA_ALPHA * current_crab
        wind_effect = max(-WIND_MAX_DEG, min(WIND_MAX_DEG, wind_effect))

    capture = abs(angl_to_turn) > CAPTURE_THRESHOLD
    sat     = abs(commanded_yaw_rate) >= cascade_pi.MAX_CMD - 0.5
    landing_mode = baro_m <= LANDING_ALT

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
            + ("  [CAP]" if capture else "")
            + ("  [SAT]" if sat else "")
            + ("  [LND]" if landing_mode else "")
            + (f"  lobe={_pattern.lobe_sign:+d}" if patterned else "")
        )
    elif _guidance_tick % 10 == 0:
        _dbg(
            f"[CTRL/{_guidance_tick}] {phase} dist={distance:.1f}m "
            f"err={angl_to_turn:.1f}° wind={wind_effect:.1f}° "
            f"pi_int={cascade_pi.pi_integral:.3f} cmd_yr={commanded_yaw_rate:.2f}°/s"
        )

    return types.SimpleNamespace(
        state=phase,
        distance=distance,
        commanded_yaw_rate=commanded_yaw_rate
    )
