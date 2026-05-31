"""Core runtime configuration for FSW."""

from __future__ import annotations


# GPIO map
BURNWIRE_GPIO = 5
EGG_SOLENOID_GPIO = 6
PARAFOIL_RIGHT_GPIO = 12
PARAFOIL_LEFT_GPIO = 13
# GNSS 7 Click RST (active low; pulse at GPS app startup unless GNSS_RESET_ENABLE=0)
GNSS_RESET_GPIO = 26


# ── 서보 기하학 / PWM 캘리브레이션 (control.py 사용) ───────────────────────────────────
ARM_MIN_DEG         = 0.0              # 팔 최소 각도 (arm up)
ARM_MAX_DEG         = 142.0            # 팔 최대 각도
                                        # 실측: LEFT_ZERO=2480µs 기준 160°명령(702µs)에서
                                        # 서보가 물리적 180°까지 과이동.
                                        # PPD=11.11µs/deg 기준 물리적 160°에 대응하는
                                        # 모델 각도 = (2480-900)/11.11 ≈ 142°.
                                        # ARM_MAX=142 → LEFT_MIN_PULSE≈902µs → 물리적 ~160°.
NEUTRAL_ARM_DEG     = 80.0             # 팔 중립 각도
LEFT_SERVO_ZERO_US  = 2480             # 왼쪽 서보 0° PWM (µs)
RIGHT_SERVO_ZERO_US = 636              # 오른쪽 서보 0° PWM (µs)
SERVO_PULSE_PER_DEG = 2000.0 / 180.0  # µs/deg 변환 계수


# Egg drop (flight state 4): rangefinder reading must be in
# [rough validity floor .. EGG_STATE_DISTANCE_TRIGGER_MM] to arm solenoid pulses.
# Typical: trigger when distance to ground <= 2500 mm with TF-Luna.
EGG_STATE_DISTANCE_TRIGGER_MM = 2500

# Relay logic levels (numeric for platform-agnostic GPIO compatibility)
# Base polarity is active-low relay modules: ON=0, OFF=1.
RELAY_ACTIVATE_LEVEL = 0
RELAY_DEACTIVATE_LEVEL = 1

# Per-actuator relay polarity
# - Release relay is wired opposite to the base polarity.
# - Egg relay follows the base polarity.
RELEASE_RELAY_ACTIVATE_LEVEL = RELAY_DEACTIVATE_LEVEL
RELEASE_RELAY_DEACTIVATE_LEVEL = RELAY_ACTIVATE_LEVEL
EGG_RELAY_ACTIVATE_LEVEL = RELAY_DEACTIVATE_LEVEL
EGG_RELAY_DEACTIVATE_LEVEL = RELAY_ACTIVATE_LEVEL


# Sensor/IO rates (Hz)
BAROMETER_RATE_HZ = 10
IMU_RATE_HZ = 50
GPS_RATE_HZ = 10
MOTOR_RATE_HZ = 20
ELECTRO_RATE_HZ = 1
DISTANCE_RATE_HZ = 10
XBEE_RATE_HZ = 10
CAMERA_FPS = 30


# GPS sanity gates.  Defaults are for the current Korea test area; update these
# before operating at a distant site.
# ±1 km radius from expected drop zone center, converted to degrees:
#   lat: 1000 m / 111320 m/deg ≈ 0.009 deg
#   lon: 1000 m / (111320 * cos(37°)) ≈ 0.011 deg
GPS_EXPECTED_LAT_CENTER_DEG = 37.5    # 서울/연세대 테스트 기지 → 경쟁지 좌표로 변경
GPS_EXPECTED_LON_CENTER_DEG = 127.0
GPS_EXPECTED_LAT_RADIUS_DEG = 2.0     # ±220 km (한국 전역 커버)
GPS_EXPECTED_LON_RADIUS_DEG = 2.0     # ±170 km
GPS_MIN_SATS = 4
GPS_MAX_VALID_SPEED_MPS = 40.0


# Release timing tuning
RELEASE_TARGET_RATIO = 0.8  # 80% max_alt: separation altitude target
# Below this fraction of max_alt: start descent-rate history and the FORCE_90PCT_TIMEOUT timer.
RELEASE_PREDICT_START_RATIO = 0.9
RELEASE_HARD_TRIGGER_RATIO = 0.85  # hard fallback if prediction is not viable
RELEASE_BURNWIRE_DELAY_SEC = 10.0  # 기존 5초, 0519 ETD → 10초로 늘림. 5초는 너무 짧아서 패러포일이 완전히 펼치기도 전에 닫아버리는 경우가 있었음
RELEASE_PREDICT_TIME_MIN_SEC = 0.0
RELEASE_PREDICT_TIME_MAX_SEC = 5.0
RELEASE_FORCE_AFTER_SEC = 5.0  # seconds after band crossing before FORCE_90PCT_TIMEOUT


# Motor guidance/control string constants
SENSOR_QUALITY_FRESH = "FRESH"
SENSOR_QUALITY_STALE = "STALE"

CONTROL_MODE_FAIL = "FAIL"

MOTOR_REASON_INIT = "INIT"
MOTOR_REASON_IDLE = "IDLE"
MOTOR_REASON_LANDED = "LANDED"
MOTOR_REASON_GUIDANCE_INACTIVE = "GUIDANCE_INACTIVE"
MOTOR_REASON_DISABLED = "DISABLED"
MOTOR_REASON_ACTIVE = "ACTIVE"
MOTOR_REASON_DEGRADED = "DEGRADED"
MOTOR_REASON_MANUAL_PREFIX = "MANUAL_"
MOTOR_MANUAL_LEFT = "LEFT"
MOTOR_MANUAL_NEUTRAL = "NEUTRAL"
MOTOR_MANUAL_RIGHT = "RIGHT"

CTRL_FALLBACK_NONE = "NONE"
CTRL_FALLBACK_GYRO_SPIKE = "GYRO_SPIKE"

# ── 자이로 스파이크 / PID 적분 감쇠 (control.py 사용) ─────────────────────────────────
# 250→500: 자유낙하 로그에서 정상 spin이 1227 dps까지 도달 시 250으로 하면
# DETUMBLING 모드에서 PID 자체가 비활성화되어 작동 불가. spike(IMU glitch)는
# 일시적 노이즈이므로, BNO085 측정 범위(±2000 dps) 아래쪽에서 마진 두고 1500.
GYRO_SPIKE_LIMIT_DEG_S = 1500.0
INTEGRAL_DECAY_RATE    = 0.95    # 자이로 스파이크 시 적분항 사이클당 감쇠율

# ── ControlConfig 기본값 (control.py 사용) ──────────────────────────────────────────
CTRL_ANGULAR_VELOCITY_DEADBAND_DEG_S = 5.0    # FF 명령 데드밴드
CTRL_DELTA_MIN_EFFECTIVE_DEG         = 5.0    # FF 최소 유효 deflection
CTRL_EXPO                            = 1.15   # FF 엑스포 커브 지수
CTRL_ERROR_DEADBAND_DEG_S            = 2.0    # PID 에러 데드밴드
CTRL_K_I                             = 0.0    # PID 적분 게인 (PID OFF: 0, 원래 값 0.01)
CTRL_I_LIMIT_DEG                     = 15.0   # PID 적분 포화 한계

# ── 수동 조향 (motorapp.py 사용) ─────────────────────────────────────────────────────
MANUAL_STEER_DELTA_DEG = 60.0   # MTR 수동 명령 시 서보 deflection (deg)


# ── New GNC control mode string constants ────────────────────────────────────────
CONTROL_MODE_GPS_TRACKING_CLOSED = "GPS_TRACKING_CLOSED"
CONTROL_MODE_GPS_TRACKING_OPEN   = "GPS_TRACKING_OPEN"
CONTROL_MODE_DR_TRACKING_CLOSED  = "DR_TRACKING_CLOSED"
CONTROL_MODE_DR_TRACKING_OPEN    = "DR_TRACKING_OPEN"
CONTROL_MODE_DETUMBLING          = "DETUMBLING"

# Dead-reckoning method string constants
DR_METHOD_NONE                   = "NONE"
DR_METHOD_GYRO_INTEGRATION       = "GYRO_INTEGRATION"
DR_METHOD_ACC_DOUBLE_INTEGRATION = "ACC_DOUBLE_INTEGRATION"
DR_METHOD_GYRO_ACC_BLEND         = "GYRO_ACC_BLEND"

# ── L1 homing guidance tuning ────────────────────────────────────────────────────
# L_GAIN_M 12→17: 자유낙하 로그 V≈-7 m/s 기준 응답시정수 L/(2V) ≈ 0.7-1s.
# 목표 착지반경 5m 진입 시 응답 강화.
L_GAIN_M     = 17.0
V_MIN_MPS    = 0.5
V_MAX_MPS    = 15.0
V_MAX_DR_MPS = 3.0    # DR 모드 전용 속도 상한 → GPS보다 보수적으로, 포화 nu 제한

# ── nu 데드밴드: 이 각도 이내는 yaw_rate_cmd=0 → 모터 중립 ────────────────────────
# 직진 방향. |nu| < NU_DEADBAND_DEG 이면 움직임 없음.
NU_DEADBAND_DEG = 15.0

# ── Sensor freshness thresholds ──────────────────────────────────────────────────
GPS_FRESH_MAX_AGE_S         = 15.0
GPS_CONTROL_FRESH_MAX_AGE_S = GPS_FRESH_MAX_AGE_S   # backward-compat alias
IMU_FRESH_MAX_AGE_S         = 3.0   # 1.5→2.0: reinit(~2s) 완료 후 stale 전환 방지
BARO_FRESH_MAX_AGE_S        = 2.0   # 0.8→2.0: 10Hz 바로미터가 8회 miss 만에 stale → GPS 기준으로 동일
BRO_FRESH_MAX_AGE_S         = BARO_FRESH_MAX_AGE_S  # spec alias

HISTORY_WINDOW_S  = 3.0
# ── Accelerometer-aided DR ───────────────────────────────────────────────────────
USE_ACC_DOUBLE_INTEGRATION = True
# ACC_LIMIT_MPS2 1.5→2.0: 1.5이하에서 정상 비행 acc 변화와 겹쳐 acc-blend 비활성
# 2.0으로 올려 acc 보조 활성화 (LIMIT 초과는 spin/아티팩트 관점에서 적절)
ACC_LIMIT_MPS2             = 2.0
ACC_BLEND_WEIGHT           = 0.2

# ── DR confidence scaling ─────────────────────────────────────────────────────────
# DR remains available while the anchor is valid; confidence only scales L1 yaw-rate.
# Schedule: 0-2s: 1.0, 2-5s: 1.0→0.5, 5-20s: 0.5→0.0, >20s: 0.0
DR_CONF_AGE_1_S = 2.0
DR_CONF_AGE_2_S = 5.0
DR_CONF_AGE_3_S = 20.0

TARGET_RADIUS_M = 5.0

# ── Yaw rate limits per control mode (deg/s) ─────────────────────────────────────
# 35→40: 정상비행 spin 분포가 35 근처, 여유 확보를 위해 상향
GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS = 40.0
GPS_TRACKING_OPEN_YAW_RATE_LIMIT_DPS   = 25.0
DR_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS  = 20.0
# 12→15: 12는 과도하게 보수적
DR_TRACKING_OPEN_YAW_RATE_LIMIT_DPS    = 15.0
DETUMBLING_YAW_RATE_LIMIT_DPS          = 0.0
FAIL_YAW_RATE_LIMIT_DPS                = 0.0

# ── Detumbling ───────────────────────────────────────────────────────────────────
# 120→150 entry: 정상 spin이 120 이하에 수렴, 진입 한계 상향
# 40→30 exit + 0.5→1.0 hold: 한계 근처 chattering 방지 (히스테리시스 강화)
# 150→200 entry: 지상 시험 테스트에서 max 224 dps 스파이크 → 150은 너무 낮음
#   1Hz 주기로 연속 반복. 실제 파라포일 분리 spin >300 dps여서 200이 안전.
DETUMBLE_ENABLE             = True
DETUMBLE_GYRZ_THRESHOLD_DPS = 200.0
DETUMBLE_EXIT_THRESHOLD_DPS = 30.0
DETUMBLE_EXIT_HOLD_S        = 1.0

# ── Sensor sign conventions ──────────────────────────────────────────────────────
# Body→NED rotation uses ZYX Euler from BNO085 raw degree output (no re-mapping).
# GYRZ_SIGN = 1.0: raw BNO085 gyrz is already negated in handle_imu
#   (CCW→negative nav); GYRZ_SIGN applies to the already-corrected value.
# ACC_X_SIGN = 1.0: body x = forward, positive = forward acceleration.
# ACC_Y_SIGN = 1.0: body y = right, positive = rightward acceleration.
GYRZ_SIGN      = 1.0
MOTOR_CMD_SIGN = 1.0
ACC_X_SIGN     = 1.0
ACC_Y_SIGN     = 1.0

# ── Yaw-rate controller gains ─────────────────────────────────────────────────────
KFF_GPS_CLOSED = 0.0
# PID OFF 테스트: KP=0 (이전 값 0.45)
KP_GPS_CLOSED  = 0.0

KFF_DR_CLOSED  = 0.0
# PID OFF 테스트: KP=0 (이전 값 0.15)
KP_DR_CLOSED   = 0.0

KFF_GPS_OPEN = 0.10
KFF_DR_OPEN  = 0.05

# Legacy detumble PID gain. Detumbling uses fixed min/max arm angles.
KP_DETUMBLE = 0.0

KI_YAW_RATE = 0.0
KD_YAW_RATE = 0.0

# ── Motor control source mode ─────────────────────────────────────────────────────
# GPS_GUIDED  : GPS L1 가이던스 + 자이로 PID (기본값)
# GPS_ONLY    : GPS L1 가이던스, 자이로 피드백 없음 (피드포워드 전용)
# IMU_HEADING : IMU 방위각만으로 목표 헤딩 추종 (GPS 불필요)


test_MOTOR_CTRL_MODE_GPS_GUIDED  = "GPS_GUIDED"
test_MOTOR_CTRL_MODE_GPS_ONLY    = "GPS_ONLY"
test_MOTOR_CTRL_MODE_IMU_HEADING = "IMU_HEADING"

test_MOTOR_CTRL_MODE = test_MOTOR_CTRL_MODE_GPS_GUIDED  # 시작 모드

# IMU_HEADING 모드 파라미터
IMU_HEADING_TARGET_DEG    = 0.0   # 목표 방위각 (0=북쪽, GPS 없을 때 fallback)
IMU_HEADING_KP            = 2.5   # bearing 오차(deg) → angular_velocity_cmd(deg/s) P게인
IMU_HEADING_MAX_CMD_DEG_S = 20.0  # angular_velocity_cmd 상한 (deg/s)
