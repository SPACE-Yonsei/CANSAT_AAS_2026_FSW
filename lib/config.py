"""Core runtime configuration for FSW."""

from __future__ import annotations


# GPIO map
BURNWIRE_GPIO = 20
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
# 250으로 두면 DETUMBLING 모드와 PID 전체가 비활성화되어 제동이 불가함.
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

# Manual steering (used by motorapp.py)
MANUAL_STEER_DELTA_DEG = 60.0   # MTR 수동 명령 시 서보 deflection (deg)


# L1 homing guidance tuning
# L_GAIN_M=12: 자유낙하 로그 V≈-7 m/s 기준 응답 시정수 L/(2V) ≈ 0.7-1s.
# 목표 반경 5m 진입 시 응답을 강화.
L_GAIN_M     = 12.0
V_MIN_MPS    = 0.5
V_MAX_MPS    = 15.0
V_MAX_DR_MPS = 7.0    # 20260531: baro_sink 초기 5.04→3.0 캡으로 L1 출력 약화. 7.0으로 확대

# nu deadband: 작은 각도 오차에서는 yaw_rate_cmd=0 및 모터 중립 유지.
# 미세 진동 방지. |nu| < NU_DEADBAND_DEG이면 움직임 없음.
NU_DEADBAND_DEG = 5.0

# Sensor freshness thresholds
GPS_FRESH_MAX_AGE_S         = 5.0
IMU_FRESH_MAX_AGE_S         = 3.0   # 1.5→3.0: reinit(~2s) 완료 전 stale 전환 방지
BARO_FRESH_MAX_AGE_S        = 2.0   # 0.8→2.0: 10Hz 바로미터가 8회 miss만으로 stale 처리되던 문제 완화

# Accelerometer-aided DR.
# 내부 로직은 USE_ACC_BLEND_CORRECTION만 사용한다 (acc는 weak blend, double
# integration 아님).
USE_ACC_BLEND_CORRECTION   = True
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

# ── Candidate origin policy ──────────────────────────────────────────────────
# State 1~2에서도 GPS position을 candidate origin으로 저장해두고, State 3 진입 시
# candidate age가 MAX_AGE 이하이면 origin으로 lock한다. State 3 이후 첫 GPS가
# 늦게 들어와도 late lock을 허용하여 DR anchor 생성 지연을 줄인다.
CANDIDATE_ORIGIN_MAX_AGE_S = 30.0
ALLOW_LATE_ORIGIN_LOCK     = True

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

# Detumbling
# 실측(20260531) State3+4 |gyr_z| p99=141 dps, max=159 dps.
# 200 dps 임계값은 실제 비행에서 미도달 → DETUMBLE 미진입.
# 100 dps(p95 근방)로 낮춰 실제 spin 구간에서 진입 가능하게 수정.
# EXIT 30→20: 출구 히스테리시스 확대로 chattering 방지.
DETUMBLE_ENABLE             = True
# 닭 1·2차 실측: |gyrz| p50≈57~61, p90≈153~162, max≈217~224 dps.
# 진입 120은 정상 spin(p50~60, p90~155) 한복판이라 1차에서 디텀블 747회 채터링.
# 진입 120→150(≈p90, 진짜 텀블만), 이탈 100→80, hold 0.05→0.3s로 히스테리시스 확대.
DETUMBLE_GYRZ_THRESHOLD_DPS = 120.0
DETUMBLE_EXIT_THRESHOLD_DPS = 80.0
DETUMBLE_EXIT_HOLD_S        = 0.3

# Sensor sign conventions
# Body→NED rotation uses ZYX Euler from BNO085 raw degree output (no re-mapping).
# GYRZ_SIGN = 1.0: raw BNO085 gyrz is already negated in handle_imu
#   (CCW→negative nav); GYRZ_SIGN applies to the already-corrected value.
# ACC_X_SIGN = 1.0: body x = forward, positive = forward acceleration.
# ACC_Y_SIGN = 1.0: body y = right, positive = rightward acceleration.
GYRZ_SIGN      = 1.0
ACC_X_SIGN     = 1.0
ACC_Y_SIGN     = 1.0

# Yaw-rate controller gains
# 20260531 실측 angular_velocity_err 평균 -38.85 dps → 폐루프 보정 활성
KP_GPS_CLOSED  = 0.35

# Per-mode PID gains for the new ControlMode taxonomy (yaw-rate loop, deg/s).
# GPS uses the existing KP_GPS_CLOSED. DR_M (position from GPS) is slightly
# weaker; DR_PM (position dead-reckoned) is the most conservative.
# KI/KD default to 0.0 (P-only loop) — kept explicit so they can be tuned per mode.
KI_GPS_CLOSED   = 0.0
KD_GPS_CLOSED   = 0.0
KP_DR_M_CLOSED  = 0.25
KI_DR_M_CLOSED  = 0.0
KD_DR_M_CLOSED  = 0.0
KP_DR_PM_CLOSED = 0.18
KI_DR_PM_CLOSED = 0.0
KD_DR_PM_CLOSED = 0.0

# Feedforward scale per mode. GPS=1.0; DR rides the FF curve at reduced
# authority since its rate command is less trustworthy.
DR_M_FF_SCALE    = 0.8
DR_PM_FF_SCALE   = 0.6

# Proportional gain for DETUMBLING: delta_arm_deg = -KP_DETUMBLE * omega_z_dps
# 0.0 = legacy bang-bang (max deflection). >0 = proportional braking.
KP_DETUMBLE = 0.8

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