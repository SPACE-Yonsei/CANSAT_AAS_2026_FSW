"""Core runtime configuration for FSW."""

from __future__ import annotations


# GPIO map
BURNWIRE_GPIO = 5
EGG_SOLENOID_GPIO = 6
PARAFOIL_RIGHT_GPIO = 12
PARAFOIL_LEFT_GPIO = 13
# GNSS 7 Click RST (active low; pulse at GPS app startup unless GNSS_RESET_ENABLE=0)
GNSS_RESET_GPIO = 26


# Servo arm geometry / PWM calibration (used by control.py)
ARM_MIN_DEG         = 0.0              # 팔 최소 각도 (arm up)
ARM_MAX_DEG         = 160.0            # 팔 최대 각도
                             
NEUTRAL_ARM_DEG     = 80.0             # 팔 중립 각도
LEFT_SERVO_ZERO_US  = 2480             # 왼쪽 서보 0° PWM (µs)
RIGHT_SERVO_ZERO_US = 636              # 오른쪽 서보 0° PWM (µs)
SERVO_PULSE_PER_DEG = 2000.0 / 180.0  # µs/deg 변환 계수

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


# GPS sanity gates.  No geographic expected-area box: any valid GPS fix is
# accepted so the system works at any site (Korea, US, anywhere).
GPS_MIN_SATS = 4
GPS_MAX_VALID_SPEED_MPS = 40.0

#must be deleted
# Release timing tuning
RELEASE_TARGET_RATIO = 0.8  # 80% max_alt: separation altitude target
# Below this fraction of max_alt: start descent-rate history and the FORCE_90PCT_TIMEOUT timer.
RELEASE_PREDICT_START_RATIO = 0.9
RELEASE_HARD_TRIGGER_RATIO = 0.85  # hard fallback if prediction is not viable
RELEASE_BURNWIRE_DELAY_SEC = 10.0 # 기존 5초, 0519 ETD 때 10초로 늘림. 5초는 너무 짧아서 낙하산이 완전히 펴지기 전에 타버리는 경우가 있었음.
RELEASE_PREDICT_TIME_MIN_SEC = 0.0
RELEASE_PREDICT_TIME_MAX_SEC = 5.0
RELEASE_FORCE_AFTER_SEC = 5.0  # seconds after band crossing before FORCE_90PCT_TIMEOUT

MOTOR_MANUAL_LEFT = "LEFT"
MOTOR_MANUAL_NEUTRAL = "NEUTRAL"
MOTOR_MANUAL_RIGHT = "RIGHT"

# Gyro spike / PID integral decay (used by control.py)
# 250→1500: 자유낙하 로그에서 정상 spin이 1227 dps까지 도달했음.
# Keep this above normal spin rates so only IMU glitches are rejected.
# spike(IMU glitch)는 단발 노이즈이므로 BNO085 측정 범위(±2000 dps) 안쪽에 마진을 두고 1500 사용.
GYRO_SPIKE_LIMIT_DEG_S = 1500.0
INTEGRAL_DECAY_RATE    = 0.95    # 자이로 값을 못 받을 때 적분항 사이클당 감쇠율

# ControlConfig defaults (used by control.py)
CTRL_ANGULAR_VELOCITY_DEADBAND_DEG_S = 5.0    # FF 명령 데드밴드
CTRL_DELTA_MIN_EFFECTIVE_DEG         = 5.0    # FF 최소 유효 deflection
CTRL_EXPO                            = 1.5    # 큰 오차에서 응답 가파르게
CTRL_ERROR_DEADBAND_DEG_S            = 5.0    # PID 에러 데드밴드
CTRL_K_I                             = 0.0    # PID 적분 게인 (PID OFF: 0, 권장 시작값: 0.01)
CTRL_I_LIMIT_DEG                     = 15.0   # PID 적분 포화 한계

# ── DR 전용 제어 권한 (DR = FF-only 조향; rate-damping PID 제거) ───────────────
# 배경: DR에서는 V가 바닥(~0.5 m/s)이라 L1 yaw_rate_cmd가 작고(≤~5 dps), GPS FF
# 데드밴드(5)에 걸려 FF가 죽는다. 그 결과 남은 rate-damping PID가 heading FF를
# 덮어써 측정 gyro만 죽이려다 nu와 반대로 조향한다(실측 부호 반대 ~29%).
# 따라서 DR은 기본적으로 PID를 끄고 FF(heading 기반)만 사용한다.
DR_PID_ENABLED = False                          # 롤백 필요 시 True (DR PID 복귀)
# DR FF 명령 데드밴드(deg/s). GPS(5.0)와 분리해 작은 yaw_rate_cmd도 FF로 살린다.
DR_CTRL_ANGULAR_VELOCITY_DEADBAND_DEG_S = 1.0
# DR FF expo 정규화 분모(deg/s) = "감도" 노브. clamp limit과 분리한다.
# mode yaw-rate limit(50 등)으로 정규화하면 DR cmd(≤~5)가 곡선 floor(~5°)에 깔려
# nu에 비례하지 않는다. 더 작은 기준으로 정규화해 nu 비례 응답을 살린다(낮을수록 민감).
# 20260606 raw_motor.csv 분석: DR 구간 |nu_clamped| 평균 69.6°(중앙값 90° 포화)인데
# delta_ff는 평균 12.9°에 그쳐 nu가 안 줄었다(부호는 정확, saturated 0). 실측 DR
# yaw_rate_cmd(~3 dps)에 정규화 기준을 맞춰 nu 비례 응답을 ~2배로 키운다(20→10).
DR_FF_REF_DPS = 6
# DR FF 출력 cap(deg). 일반 천장(±160)보다 작게 둬 감도 상향이 full hard-over/나선
# (spiral)로 가지 않게 한다.
DR_FF_DELTA_LIMIT_DEG = 60.0

# Manual steering (used by motorapp.py)
MANUAL_STEER_DELTA_DEG = 60.0   # MTR 수동 명령 시 서보 deflection (deg)


# L1 homing guidance tuning
# L_GAIN_M=12: 자유낙하 로그 V≈-7 m/s 기준 응답 시정수 L/(2V) ≈ 0.7-1s.
# 목표 반경 5m 진입 시 응답을 강화.
# 로그 기반 재튜닝 후보: 15~18 m (현재 12는 응답이 다소 공격적일 수 있음, 비행 로그로 확정)
L_GAIN_M     = 8.0    # 20260606: 12→8 (감도 상향). nu 응답 시정수 L/(2V)를 단축해 같은 nu에서 yaw_rate_cmd ↑.
V_MIN_MPS    = 0.5
V_MAX_MPS    = 15.0
# 20260531: baro_sink 초기 5.04→3.0 캡으로 L1 출력 약화. 7.0으로 확대했으나
# baro sink spike/EMA 필터 도입(아래 DR_BARO_SINK_* 참고)에 맞춰 보수적으로 6.5 시작.
V_MAX_DR_MPS = 6.5

# L1 조향식 yaw_rate_cmd = 2·V/L·sin(nu)는 V에 비례한다. DR 속도 추정이
# baro sink≈0(벤치/완만한 강하)에서 V_MIN(0.5)으로 붕괴하면 nu가 커도 명령이
# 사실상 0이 된다(로그 run_20260606_171741: DR med|nu|=66°인데 med|cmd|=2.8dps).
# 조향식에만 적용하는 속도 하한 — 검증 게이트(nav.V≥V_MIN)와 DR 위치 적분에는
# 미적용 — 으로 저속 추정에서도 실제 선회를 명령한다.
# 0.5=기존 동작(무효과), 권장 4.0, 보수적 3.0, 공격적 5~6. 실비행 V≈6~8에선 거의 안 묶임.
L1_STEER_V_FLOOR_MPS = 4.0

# nu deadband: 작은 각도 오차에서는 yaw_rate_cmd=0 및 모터 중립 유지.
# 미세 진동 방지. |nu| < NU_DEADBAND_DEG이면 움직임 없음.
# 20260606: 5→3 (감도 상향). GPS_CLOSED nu가 대부분 <5°라 조향 FF가 83% 죽던 문제 완화.
NU_DEADBAND_DEG = 3.0

# Sensor freshness thresholds
GPS_FRESH_MAX_AGE_S         = 5.0
IMU_FRESH_MAX_AGE_S         = 3.0   # 1.5→3.0: reinit(~2s) 완료 전 stale 전환 방지
BARO_FRESH_MAX_AGE_S        = 2.0   # 0.8→2.0: 10Hz 바로미터가 8회 miss만으로 stale 처리되던 문제 완화

# IMU 가속도 입력 모드 (motorapp._compute_linear_acc가 참조).
#   "RAW"    : ax/ay/az가 중력 포함 raw 가속도 → roll/pitch 기반 중력 제거 수행.
#   "LINEAR" : ax/ay/az가 이미 중력 제거된 linear acceleration → 그대로 사용(이중 제거 방지).
# 현재 IMU app(imuapp.py)은 bno.acceleration(raw)을 송신하므로 기본값 "RAW"가 맞다.
# BNO085 linear_acceleration으로 송신부를 바꾸면 "LINEAR"로 변경.
IMU_ACCEL_INPUT_MODE = "RAW"
# sample_ts(=IMU monotonic) 기반 노후 샘플 reject 사용 여부.
#   True  : rx_ts - sample_ts > LIN_ACC_SAMPLE_MAX_AGE_S 면 lin_acc invalid.
#   False : sample_age 기반 reject 끄고 rx_ts freshness만 사용(두 timebase가 다를 때).
# 현재 sample_ts/rx_ts 모두 time.monotonic()이라 같은 timebase → True 적합.
LIN_ACC_USE_SAMPLE_AGE_GATE = True

# Accelerometer-aided DR.
# 내부 로직은 USE_ACC_BLEND_CORRECTION만 사용한다 (acc는 weak blend, double
# integration 아님).
USE_ACC_BLEND_CORRECTION   = False
# 20260531 실측: lin_acc XY mag mean=1.32, 누적 velocity error 최대 3.6 m/s
# 1.5로 축소하여 오염 샘플 비율 감소
ACC_LIMIT_MPS2             = 1.5
ACC_BLEND_WEIGHT           = 0.15
LIN_ACC_XY_MAX_MPS2        = ACC_LIMIT_MPS2
# |gyrz| > ACC_GYRZ_REJECT_DPS 시 acc 데이터 거부 (BNO085 acc/Euler 비동기 방지)
# 실측: |gyrz|>80 dps 구간에서 lin_acc mag 3.1~19.1 m/s² 이상값 집중
ACC_GYRZ_REJECT_DPS        = 80.0
# 1.0→2.2: 닭 2차 실측 glide ratio (gps_speed 4.97 / baro sink 2.2 ≈ 2.26).
# gain=1.0(수평=수직 가정)은 DR 수평속도를 ~절반으로 과소추정 → L1 yaw_rate_cmd(∝V)가
# 둔해짐. 실측 활공비로 보정. (gps_speed 표본 제한적 → bench 재확인 권장)
DR_SINK_TO_HSPEED_GAIN     = 2.2

# DR confidence scaling
# DR remains available while the anchor is valid; confidence only scales L1 yaw-rate.
# 실측(20260531) State3+4 비행시간 52.7s. AGE_3=20s는 비행 중반에 confidence=0 소진.
# Schedule: 0-5s: 1.0, 5-20s: 1.0→0.5, 20-60s: 0.5→0.0, >60s: 0.0
DR_CONF_AGE_1_S = 5.0
DR_CONF_AGE_2_S = 30.0
DR_CONF_AGE_3_S = 60.0

# ── DR safety guards (명시적 상수: getattr fallback이 inf로 꺼지지 않도록 보장) ──
# DR_PM 한 cycle 위치 적분이 이 거리를 넘으면 비정상으로 보고 update reject.
DR_MAX_POSITION_JUMP_M          = 3.0
# DR anchor가 이보다 오래되면 FillNav가 DR_TIMEOUT으로 FAIL.
DR_MAX_AGE_S                    = 45.0
# DR confidence가 이보다 낮으면 ProduceL1Output이 LOW_DR_CONFIDENCE로 invalid.
DR_MIN_CONFIDENCE_FOR_CONTROL   = 0.20
# gyro yaw-rate가 이보다 크면 closed-loop gyro feedback을 쓰지 않고 OPEN/FAIL 경로로.
DR_MAX_YAW_RATE_DPS_FOR_CONTROL = 120.0
# yaw-gyro blend 시 두 코스 추정 차이가 이 각도를 넘으면 blend 거부(gyro-only fallback).
YAW_GYRO_BLEND_MAX_DEG          = 45.0

# ── baro sink 기반 DR speed 필터링 ───────────────────────────────────────────
# |raw_sink| > DR_BARO_SINK_MAX_MPS 이면 spike로 거부(이번 cycle baro_sink_fresh=False).
DR_BARO_SINK_MAX_MPS = 6.0
# sink rate EMA 시정수(s). alpha = dt / (tau + dt).
DR_SINK_EMA_TAU_S    = 0.7
# sink_rate 부호 규약: True면 "하강 시 양수"(현 baro app 가정), DR speed에 그대로 사용.
# False면 "하강 시 음수" → DR speed에 -sink_rate 사용.
BARO_SINK_POSITIVE_DOWN = True

# ── Origin lock policy ───────────────────────────────────────────────────────
# candidate 없음. STATE >= 3(DESCENT 이상)에서 처음 들어오는 유효 GPS 좌표를
# 그대로 origin으로 lock한다(motorapp.handle_gps). 별도 튜닝 상수 없음.

# Yaw rate limits per control mode (deg/s)
# 35→60: 정상비행 spin 분포가 35 dps 근처까지 올라와 제어 여유를 확보하기 위해 상향
GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS = 60.0
GPS_TRACKING_OPEN_YAW_RATE_LIMIT_DPS   = 35.0

# Per-mode yaw-rate limits for the GBA/GB/G · YBA/YB/Y source taxonomy (deg/s).
# DR_M_* (position from GPS, motion estimated) is trusted slightly more than the
# corresponding DR_PM_* (position dead-reckoned). Gyro(G) sources allow higher
# rates than yaw(Y) sources; baro(B) and acc(A) richness adds margin.
DR_M_GBA_YAW_RATE_LIMIT_DPS            = 50.0
DR_M_GB_YAW_RATE_LIMIT_DPS             = 45.0
DR_M_G_YAW_RATE_LIMIT_DPS              = 35.0
DR_M_YBA_YAW_RATE_LIMIT_DPS            = 25.0
DR_M_YB_YAW_RATE_LIMIT_DPS             = 20.0
DR_M_Y_YAW_RATE_LIMIT_DPS             = 15.0
DR_PM_GBA_YAW_RATE_LIMIT_DPS           = 45.0
DR_PM_GB_YAW_RATE_LIMIT_DPS            = 40.0
DR_PM_G_YAW_RATE_LIMIT_DPS             = 30.0
DR_PM_YBA_YAW_RATE_LIMIT_DPS           = 20.0
DR_PM_YB_YAW_RATE_LIMIT_DPS            = 15.0
DR_PM_Y_YAW_RATE_LIMIT_DPS             = 10.0

# Sensor sign conventions
# Body→NED rotation uses ZYX Euler from BNO085 raw degree output (no re-mapping).
# GYRZ_SIGN = 1.0: raw BNO085 gyrz is already negated in handle_imu
#   (CCW→negative nav); GYRZ_SIGN applies to the already-corrected value.
# ACC_X_SIGN = 1.0: body x = forward, positive = forward acceleration.
# ACC_Y_SIGN = 1.0: body y = right, positive = rightward acceleration.
GYRZ_SIGN      = 1.0
ACC_X_SIGN     = 1.0
ACC_Y_SIGN     = 1.0

# Yaw-rate controller gains.
# Based on first-flight log fit (K=0.63, tau=0.5), conservatively reduced
# for second/third-flight failure cases and lower-confidence DR modes.
KP_GPS_CLOSED   = 0.53
KI_GPS_CLOSED   = 1.06
KD_GPS_CLOSED   = 0.0

KP_DR_M_CLOSED  = 0.40
KI_DR_M_CLOSED  = 0.25
KD_DR_M_CLOSED  = 0.0

KP_DR_PM_CLOSED = 0.30
KI_DR_PM_CLOSED = 0.2
KD_DR_PM_CLOSED = 0.0

# Feedforward scale per mode. GPS=1.0; DR rides the FF curve at reduced
# authority since its rate command is less trustworthy.
DR_M_FF_SCALE    = 0.8
DR_PM_FF_SCALE   = 0.6

KD_YAW_RATE = 0.0

# Motor control source preference (operator-selectable tag, CMC 명령으로 변경).
# 최종 control mode는 항상 guidance.DecideControlMode가 결정한다. 이 값은 더 이상
# motorapp에서 별도 제어 분기를 만들지 않으며, 로깅/소스 선호 표식으로만 쓰인다.
# IMU_HEADING은 더 이상 독립 출력 모드가 아니다. 선택돼도 guidance 파이프라인이
# 그대로 돌고, yaw만 fresh한 구간은 DR_M_Y*/DR_PM_Y*, 아니면 FAIL로 매핑된다.
# (MOTOR_CTRL_MODE_IMU_HEADING 상수는 commapp CMC 매핑이 참조하므로 유지한다.)
MOTOR_CTRL_MODE_GPS_GUIDED  = "GPS_GUIDED"
MOTOR_CTRL_MODE_GPS_ONLY    = "GPS_ONLY"
MOTOR_CTRL_MODE_IMU_HEADING = "IMU_HEADING"
