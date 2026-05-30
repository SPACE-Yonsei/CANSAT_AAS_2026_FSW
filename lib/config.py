"""Core runtime configuration for FSW."""

from __future__ import annotations


# GPIO map
BURNWIRE_GPIO = 5
EGG_SOLENOID_GPIO = 6
PARAFOIL_RIGHT_GPIO = 12
PARAFOIL_LEFT_GPIO = 13
# GNSS 7 Click RST (active low; pulse at GPS app startup unless GNSS_RESET_ENABLE=0)
GNSS_RESET_GPIO = 26


# ?�?� ?�보 ??기하??/ PWM 캘리브레?�션 (control.py ?�용) ?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�
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
#   lat: 1000 m / 111320 m/deg ??0.009 deg
#   lon: 1000 m / (111320 * cos(37°)) ??0.011 deg
GPS_EXPECTED_LAT_CENTER_DEG = 37.5    # ?�울/?�세?� ?�스??기�? ???�????경쟁지 좌표�?변�?
GPS_EXPECTED_LON_CENTER_DEG = 127.0
GPS_EXPECTED_LAT_RADIUS_DEG = 2.0     # ±220 km (?�국 ?�역 커버)
GPS_EXPECTED_LON_RADIUS_DEG = 2.0     # ±170 km
GPS_MIN_SATS = 4
GPS_MAX_VALID_SPEED_MPS = 40.0


# Release timing tuning
RELEASE_TARGET_RATIO = 0.8  # 80% max_alt: separation altitude target
# Below this fraction of max_alt: start descent-rate history and the FORCE_90PCT_TIMEOUT timer.
RELEASE_PREDICT_START_RATIO = 0.9
RELEASE_HARD_TRIGGER_RATIO = 0.85  # hard fallback if prediction is not viable
RELEASE_BURNWIRE_DELAY_SEC = 10.0 # 기존 5�? 0519 ETD ??10초로 ?�림. 5초는 ?�무 짧아???�하?�이 ?�전???��?기도 ?�에 ?�버리??경우가 ?�었??
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

# ?�?� ?�이�??�파?�크 / PID ?�분 감쇠 (control.py ?�용) ?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�
# 250??500: ?�유?�하 로그?�서 ?�상 spin??1227 dps까�? ?�달 ??250?�로 ?�면
# DETUMBLING 모드??PID ?�체가 비활?�화?�어 ?�동 불�?. spike(IMU glitch)??
# ?�발 ?�이즈이므�?BNO085 측정 범위(±2000 dps) ?�쪽?�서 마진 ?�고 1500.
GYRO_SPIKE_LIMIT_DEG_S = 1500.0
INTEGRAL_DECAY_RATE    = 0.95    # ?�이�??�을 ???�분???�이?�당 감쇠??

# ?�?� ControlConfig 기본�?(control.py ?�용) ?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�
CTRL_ANGULAR_VELOCITY_DEADBAND_DEG_S = 5.0    # FF 명령 ?�드밴드
CTRL_DELTA_MIN_EFFECTIVE_DEG         = 5.0    # FF 최소 ?�효 deflection
CTRL_EXPO                            = 1.15   # FF ?�스??커브 지??
CTRL_ERROR_DEADBAND_DEG_S            = 2.0    # PID ?�러 ?�드밴드
CTRL_K_I                             = 0.0    # PID ?�분 게인 (PID OFF: 0?�원�???0.01)
CTRL_I_LIMIT_DEG                     = 15.0   # PID ?�분 ?�화 ?�계

# ?�?� ?�동 조향 (motorapp.py ?�용) ?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�
MANUAL_STEER_DELTA_DEG = 60.0   # MTR ?�동 명령 ???�보 deflection (deg)


# ?�?� New GNC control mode string constants ?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�
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

# ?�?� L1 homing guidance tuning ?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�
# L_GAIN_M 12??0: ?�유?�하 로그 V??-7 m/s 기�? ?�답?�정??L/(2V) ??0.7-1s.
# ?��? ?��?반경 5m 진입 ???�답??강화.
L_GAIN_M     = 17.0
V_MIN_MPS    = 0.5
V_MAX_MPS    = 15.0
V_MAX_DR_MPS = 3.0    # DR 모드 ?�용 ?�도 ?�한 ??GPS보다 보수?�으�??�화 nu ?�제

# ?�?� nu ?�드밴드: ??각도 ?�내�?yaw_rate_cmd=0 ??모터 중립 ?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�
# ?�진??방�?. |nu| < NU_DEADBAND_DEG ?????�직임 ?�음.
NU_DEADBAND_DEG = 15.0

# ?�?� Sensor freshness thresholds ?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�
GPS_FRESH_MAX_AGE_S         = 15.0
GPS_CONTROL_FRESH_MAX_AGE_S = GPS_FRESH_MAX_AGE_S   # backward-compat alias
IMU_FRESH_MAX_AGE_S         = 3.0   # 1.5??.0: reinit(~2s) ?�료 ??stale ?�환 방�?
BARO_FRESH_MAX_AGE_S        = 2.0   # 0.8??.0: 10Hz 바로미터??8??miss 만에 stale ??GPS 기�?�??�일
BRO_FRESH_MAX_AGE_S         = BARO_FRESH_MAX_AGE_S  # spec alias

HISTORY_WINDOW_S  = 3.0
# ?�?� Accelerometer-aided DR ?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�
USE_ACC_DOUBLE_INTEGRATION = True
# ACC_LIMIT_MPS2 1.5??.0: 1.5???�상 ?�공 acc 변?�과 겹쳐 acc-blend 비활??
# 2.0?�로 ?�??acc 보조 ?�성??(LIMIT 초과??spin/?�팩???�점?�라 ?�절)
ACC_LIMIT_MPS2             = 2.0
ACC_BLEND_WEIGHT           = 0.2

# ?�?� DR confidence scaling ?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�
# DR remains available while the anchor is valid; confidence only scales L1 yaw-rate.
# Schedule: 0-2s: 1.0, 2-5s: 1.0??.5, 5-20s: 0.5??.0, >20s: 0.0
DR_CONF_AGE_1_S = 2.0
DR_CONF_AGE_2_S = 5.0
DR_CONF_AGE_3_S = 20.0

TARGET_RADIUS_M = 5.0

# ?�?� Yaw rate limits per control mode (deg/s) ?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�
# 35??0: ?�상비행 spin 분포가 35 근처??권한 ?�간 ?�향
GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS = 40.0
GPS_TRACKING_OPEN_YAW_RATE_LIMIT_DPS   = 25.0
DR_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS  = 20.0
# 12??5: 12??과도?�게 보수??
DR_TRACKING_OPEN_YAW_RATE_LIMIT_DPS    = 15.0
DETUMBLING_YAW_RATE_LIMIT_DPS          = 0.0
FAIL_YAW_RATE_LIMIT_DPS                = 0.0

# ?�?� Detumbling ?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�
# 120??50 entry: ?�상 spin??120 ?�에 ?�수, 진입 ?�계 ?�간 ?�향
# 40??0 exit + 0.5??.0 hold: ?�계 근처 chattering 방�? (?�스?�리?�스 강화)
# 150??00 entry: 지???�기 ?�스?�에??max 224 dps ?�파?�크 ??150???�무 ??��
#   1Hz 주기�??�진??반복. ?�제 ?�라?�일 분리 spin >300 dps?��?�?200???�전.
DETUMBLE_ENABLE             = True
DETUMBLE_GYRZ_THRESHOLD_DPS = 200.0
DETUMBLE_EXIT_THRESHOLD_DPS = 30.0
DETUMBLE_EXIT_HOLD_S        = 1.0

# ?�?� Sensor sign conventions ?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�
# Body?�NED rotation uses ZYX Euler from BNO085 raw degree output (no re-mapping).
# GYRZ_SIGN = 1.0: raw BNO085 gyrz is already negated in handle_imu
#   (CCW?�negative nav); GYRZ_SIGN applies to the already-corrected value.
# ACC_X_SIGN = 1.0: body x = forward, positive = forward acceleration.
# ACC_Y_SIGN = 1.0: body y = right, positive = rightward acceleration.
GYRZ_SIGN      = 1.0
MOTOR_CMD_SIGN = 1.0
ACC_X_SIGN     = 1.0
ACC_Y_SIGN     = 1.0

# ?�?� Yaw-rate controller gains ?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�
KFF_GPS_CLOSED = 0.0
# PID OFF ?�스?? KP=0 (?�복 ??0.45)
KP_GPS_CLOSED  = 0.0

KFF_DR_CLOSED  = 0.0
# PID OFF ?�스?? KP=0 (?�복 ??0.15)
KP_DR_CLOSED   = 0.0

KFF_GPS_OPEN = 0.10
KFF_DR_OPEN  = 0.05

# Legacy detumble PID gain. Detumbling uses fixed min/max arm angles.
KP_DETUMBLE = 0.0

KI_YAW_RATE = 0.0
KD_YAW_RATE = 0.0

# ?�?� Motor control source mode ?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�?�
# GPS_GUIDED  : GPS L1 가?�던??+ ?�이�?PID (기본�?
# GPS_ONLY    : GPS L1 가?�던?? ?�이�??�드�??�음 (?�드?�워???�용)
# IMU_HEADING : IMU ?�력�?방위각만?�로 목표 ?�딩 추종 (GPS 불필??


MOTOR_CTRL_MODE_GPS_GUIDED  = "GPS_GUIDED"
MOTOR_CTRL_MODE_GPS_ONLY    = "GPS_ONLY"
MOTOR_CTRL_MODE_IMU_HEADING = "IMU_HEADING"

# IMU_HEADING 모드 파라미터
IMU_HEADING_TARGET_DEG    = 0.0   # 목표 방위각 (0=북쪽, GPS 없을 때 fallback)
IMU_HEADING_KP            = 1.5   # bearing 오차(deg) → angular_velocity_cmd(deg/s) P게인
IMU_HEADING_MAX_CMD_DEG_S = 20.0  # angular_velocity_cmd 상한 (deg/s)
