# factors.md — Motor / Guidance / Control 핵심 정리

> 본 문서는 CanSat AAS 2026 FSW의 모터·가이던스·제어 서브시스템 전반에 대한
> **최종 점검 결과**, **실행 흐름**, **구조 인덱스**, **지상 모터 테스트 가이드**,
> **실비행 튜닝 가이드**를 한 곳에 정리한다.

---

## §1. 최종 점검 결과 (전체)

| 항목 | 결과 |
|---|---|
| **py_compile** — main, flightlogic, gpsapp, imuapp, baroapp, motorapp, guidance, control, config, prevstate, appargs, msgstructure | **12/12 OK** |
| **import** — Sensor_Motor.{motorapp, guidance, control} | OK |
| **충돌 마커** (`<<<<<<<` / `=======` / `>>>>>>>`) | 0건 |
| **레거시 잔재** (`FillFresh`/`NOMINAL_*`/`DEGRADED_*`/`lat_acc_cmd`/`nu1`/`carrot_proj`/`SENSOR_QUALITY_`/`_START_POINT_LOCKED`/`_wrap180`/legacy MID 등) | 0건 |
| **회귀 테스트** — 7개 파일 | **205/205 OK** |
| **IPC MID 라우팅 일관성** | OK (아래 §2.2 표 참조) |

### §1.1 모듈 간 연결 확인

```
                            ┌─────────────────────────┐
                            │       main.py           │
                            │   Queue + Pipe Pool     │
                            └────┬───────────────┬────┘
                                 │ Pipe          │ Pipe
                                 ↓               ↓
   ┌──────────────┐   send_msg   ┌─────────┐   ┌──────────────────┐
   │  gpsapp      │─────────────▶│         │   │ flightlogicapp   │
   │  (10 Hz)     │ MID_motor_gps│ main.py │   │ - state machine  │
   └──────────────┘              │ queue   │   │ - release ctrl   │
   ┌──────────────┐              │   ↓     │   └────────┬─────────┘
   │  imuapp      │─────────────▶│ routes  │            │ MID_motor_state
   │  (50 Hz)     │ MID_motor_imu│   to    │            │ MID_motor_TargetCor
   └──────────────┘              │ child   │            │ MID_motor_burnwire
   ┌──────────────┐              │ pipe    │            │ MID_motor_EggDrop
   │  baroapp     │─────────────▶│         │            │
   │  (10 Hz)     │ MID_motor_alt│         │            │
   └──────────────┘              └────┬────┘            ↓
                                      │ Pipe(child)     │
                                      ↓                 ↓
                              ┌───────────────────────────────────┐
                              │    Sensor_Motor/motorapp.py       │
                              │  ┌─────────────────────────────┐  │
                              │  │  dispatch() → handle_*()    │  │
                              │  └────────┬────────────────────┘  │
                              │           ↓                       │
                              │  ┌─────────────────────────────┐  │
                              │  │  _CACHE (snapshot)          │  │
                              │  └────────┬────────────────────┘  │
                              │           ↓                       │
                              │  ┌─────────────────────────────┐  │
                              │  │  ctrl_parafoil (20 Hz loop) │  │
                              │  │  → guidance.decidefresh     │  │
                              │  │  → guidance.produceL1input  │  │
                              │  │  → guidance.produceL1output │  │
                              │  │  → control.ProduceCtrlInput │  │
                              │  │  → control.ProduceCtrlOutput│  │
                              │  │  → control.ProducePulse(PI) │  │
                              │  │  → sensorlog + _send_diag   │  │
                              │  └─────────────────────────────┘  │
                              └───────────────────────────────────┘
                                          ↓ servo PWM
                                  ┌───────────────────┐
                                  │  pigpio → 서보×2  │
                                  └───────────────────┘
```

### §1.2 데이터 경로 무결성

| 경로 | 검증 |
|---|---|
| `gpsapp → motorapp.handle_gps` | 6필드 `lat,lon,pos_ts,course_deg,spd_mps,motion_ts` 일치 (handle_gps의 `len(fields) != 6` 가드와 송신 포맷 일치) |
| `imuapp → motorapp.handle_imu` | 16필드 (15필드 + HEALTH) 일치 (handle_imu의 `len(fields) < 15` 가드, `len(fields) >= 16` health 읽기 모두 정상) |
| `baroapp → motorapp.handle_barometer` | 3필드 `alt,sample_ts,sink_rate` 일치 |
| `flightlogic → motorapp.handle_flight_state` | 정수 state code 단일 필드 |
| `flightlogic → motorapp.handle_target_coord` | `lat,lon` 2필드 + (0,0) 거부 일관성 |
| `motorapp → commapp` (`_send_diag`) | 29필드 페이로드 (`test_motor_integrity` 스키마 검증) |

---

## §2. ctrl_parafoil 실행 조건

### §2.1 게이트 체인 (순서대로 평가)

```
매 사이클 (20 Hz, period=0.05s):
  cycle_start = time.monotonic()
  now = timebase.now()

  ┌─ snapshot ─┐
  │ motor_enabled, state, manual_mode, snap = _cache_snapshot()
  │            │  ← _UPDATE_LOCK 하에서 atomic snapshot
  └────────────┘

  Gate 1: motor 활성 + state 충족
  ─────────────────────────────────────────────────────
  if not motor_enabled or state < 3:
      WriteZero(PI)           ← 서보 중립 PWM
      _sleep_for_period(); continue

  Gate 2: 착륙 상태
  ─────────────────────────────────────────────────────
  if state == 5:
      WriteOff(PI)            ← PWM 단절 (pulse_width=0)
      _sleep_for_period(); continue

  Gate 3: 수동 조향 우선
  ─────────────────────────────────────────────────────
  if manual_mode != MOTOR_MANUAL_NEUTRAL:
      cmd = _manual_steer_command(now, manual_mode)
      ProducePulse + sensorlog + _send_diag
      _sleep_for_period(); continue

  Gate 4: 가이던스 + 컨트롤 파이프라인
  ─────────────────────────────────────────────────────
  fresh    = decidefresh(gps, imu, baro, state, now)
  l1_input = produceL1input(fresh, gps, imu, state, release_state, now)
  if not _ORIGIN_SAVED and _sync_origin_to_prevstate(): _ORIGIN_SAVED = True
  g_out    = produceL1output(l1_input)
  if g_out.control_valid:
      measured_dps = _measured_yaw_rate_dps(g_out, fresh, imu)
      cmd = ProduceCtrlOutput(_CONTROLLER, ProduceCtrlInput(g_out, now), measured_dps, now)
  else:
      cmd = WriteNeutral(now, g_out.reason or MOTOR_REASON_GUIDANCE_INACTIVE)
  ProducePulse(PI, cmd) + sensorlog + _send_diag
  _sleep_for_period()
```

### §2.2 실행 조건 매트릭스

| 사전조건 | 필수 값 | 충족 시 진행 |
|---|---|---|
| `MOTOR_ENABLED` | `True` (CMD `MEC ON` 또는 prevstate `PREV_MOTOR_ENABLED=1`) | Gate 1 통과 |
| `STATE` | `≥3` AND `≠5` | Gate 1, 2 통과 |
| `MANUAL_STEER_MODE` | `NEUTRAL` | Gate 3 통과 → 자율 가이던스 |
| `_GUIDANCE_STATE.origin_ready` | True (첫 fresh GPS로 자동 lock 또는 prevstate 복원) | `FAIL_NO_ORIGIN` 회피 |
| `_GUIDANCE_STATE.target_ready` | True (handle_target_coord 또는 prevstate 복원 후 자동 투영) | `FAIL_NO_TARGET` 회피 |
| **GPS fresh** (point + velocity) | age ≤ `GPS_FRESH_MAX_AGE_S=2.0s` | `GPS_TRACKING_*` 모드 |
| **GPS stale + DR 가능** | dr_start_* 채워짐 + IMU yaw OR gyrz fresh + confidence > 0 | `DR_TRACKING_*` 모드 |
| **gyrz_fresh** | imu_age ≤ `IMU_FRESH_MAX_AGE_S=1.5s` AND gyrz 값 존재 | `*_CLOSED` 분기 |
| **gyrz spin > 임계** | `\|gyrz\| ≥ DETUMBLE_GYRZ_THRESHOLD_DPS=150` | `DETUMBLING` 모드 |

### §2.3 게이트 통과 후 컨트롤러 동작 매트릭스

| guidance.ControlMode | `pid_enabled` | yaw_rate_limit | measured_dps | → control.mode |
|---|---|---:|---|---|
| `GPS_TRACKING_CLOSED` | True | 40 dps | 유효 | `CLOSED_LOOP` |
| `GPS_TRACKING_OPEN` | True | 25 dps | NaN | `FEEDFORWARD_ONLY` |
| `DR_TRACKING_CLOSED` | True | 20 dps | 유효 | `CLOSED_LOOP` |
| `DR_TRACKING_OPEN` | True | 15 dps | NaN | `FEEDFORWARD_ONLY` |
| `DETUMBLING` | True (`kp_override=KP_DETUMBLE`) | 0 dps | 유효 | `CLOSED_LOOP` (cmd=0 + 측정 yaw 역방향 제동) |
| `FAIL` | False (`control_valid=False`) | — | — | `NEUTRAL` (WriteNeutral) |

---

## §3. 구조/변수/함수 인덱스 (실행 순서·전달 순서)

### §3.1 motorapp.py — IPC 수신 + 캐시 + 컨트롤 루프

#### 모듈 레벨 글로벌

| 이름 | 타입 | 역할 | 초기값 |
|---|---|---|---|
| `_CACHE` | `_Cache` | 모든 센서 최신 샘플 + target/start 저장소 | 빈 `_Cache()` |
| `_GUIDANCE_STATE` | `guidance.GuidanceState` | guidance 누적 상태 (history, DR anchor, nav) | 빈 GuidanceState |
| `_CONTROLLER` | `control.Ctrler` | PID 상태 + 이전 arm 각도 | `None` until init() |
| `STATE` | `int` | 현재 비행 state (0..5) | 0 |
| `MOTOR_ENABLED` | `bool` | 모터 활성 플래그 | True |
| `MANUAL_STEER_MODE` | `str` | NEUTRAL/LEFT/RIGHT | NEUTRAL |
| `RELEASE_ACTION_ENABLED` / `EGG_ACTION_ENABLED` | `bool` | actuator 권한 | True |
| `_ORIGIN_SAVED` | `bool` | origin → prevstate 1회 동기화 플래그 | False |
| `PI` | `pigpio.pi` or None | 서보 PWM 핸들 | None until init() |
| `_UPDATE_LOCK` / `_CTRL_LOCK` | `threading.Lock` | 캐시/컨트롤러 동시접근 직렬화 | locked-on-demand |

#### 데이터클래스

| 클래스 | 필드 (요약) |
|---|---|
| `_GpsFromApp` | lat, lon, course_rad, speed_mps, pos_ts, motion_ts, rx_ts, pos_health, motion_health |
| `_ImuFromApp` | roll/pitch/yaw_rad, accx/y/z_mps2, magx/y/z_uT, gyrx/y/z_rad_s, ts, rx_ts, freefall, tumble, lin_acc_x/y/z, lin_acc_valid |
| `_BaroFromApp` | alt_m, sink_rate, ts, rx_ts |
| `_Cache` | latest_gps/imu/baro, target_lat/lon, start_lat/lon |

#### 함수 (실행 순서)

| # | 함수 | 호출자 | 역할 |
|---:|---|---|---|
| 1 | `motorapp_main(main_queue, main_pipe)` | main.py launcher | 진입점. init → ctrl_thread spawn → dispatch loop |
| 2 | `init()` | motorapp_main | prevstate 로드, target/origin 복원, `_CONTROLLER` 생성, `PI` 초기화, Motor_Release/Egg init |
| 3 | `dispatch(msg)` (loop) | motorapp_main | IPC msg → mid 분기 → handle_* |
| 3a | `handle_gps(data)` | dispatch | 6필드 파싱 → `_CACHE.latest_gps` |
| 3b | `handle_imu(data)` | dispatch | 16필드 파싱 + `_compute_linear_acc` → `_CACHE.latest_imu` (HEALTH=0이면 미갱신) |
| 3c | `handle_barometer(data)` | dispatch | 3필드 파싱 → `_CACHE.latest_baro` |
| 3d | `handle_target_coord(data)` | dispatch | (0,0) 거부 → `_CACHE.target_*` + `_GUIDANCE_STATE.target_*` |
| 3e | `handle_flight_state(data)` | dispatch | `STATE` 갱신, state<3 시 reset_guidance + controller_reset |
| 3f | `handle_mec(data)` | dispatch | `MOTOR_ENABLED` ON/OFF |
| 3g | `handle_mtr(data)` | dispatch | `MANUAL_STEER_MODE` NEUTRAL/LEFT/RIGHT |
| 3h | `handle_fac(data)` | dispatch | RELEASE/EGG actuator enable ON/OFF |
| 3i | `handle_cmc(data)` | dispatch | (no-op + log) IPC 호환성 유지 |
| 3j | `handle_release(data)` | dispatch | Motor_Release.activate_burnwire 스레드 spawn |
| 3k | `handle_egg_drop()` | dispatch | Motor_Egg.activate_solenoid 스레드 spawn |
| 4 | `_cache_snapshot()` | ctrl_parafoil | `_UPDATE_LOCK` 하 deep-copy `_Cache` |
| 5 | `ctrl_parafoil(main_queue)` | thread | 20 Hz 루프 (period=0.05s, rate-compensated) |
| 5a | `_ctrl_cycle(main_queue, now)` | ctrl_parafoil | 한 사이클 본체 (테스트용 추출) |
| 5b | `_manual_steer_command(now, mode)` | _ctrl_cycle | manual_steer 모드 fixed delta_arm |
| 5c | `_measured_yaw_rate_dps(g_out, fresh, imu)` | _ctrl_cycle | gyrz_rad_s × GYRZ_SIGN → deg/s (조건 미충족 시 NaN) |
| 5d | `_sync_origin_to_prevstate()` | _ctrl_cycle | origin → `_CACHE.start_*` + `prevstate.update_start_point` (1회) |
| 5e | `_send_diag(main_queue, cmd, g_out, diag_state, snap)` | _ctrl_cycle | 29필드 페이로드 → commapp |
| 5f | `_fmt_num(value, digits)` | _send_diag | float 또는 'nan' |
| 5g | `_sleep_for_period(cycle_start, period)` | ctrl_parafoil | `time.sleep(max(0, period - elapsed))` |

### §3.2 guidance.py — L1 target-fixed homing 파이프라인

#### Enums / 상수

| 이름 | 값 |
|---|---|
| `ControlMode` | GPS_TRACKING_CLOSED, GPS_TRACKING_OPEN, DR_TRACKING_CLOSED, DR_TRACKING_OPEN, DETUMBLING, FAIL |
| `DRMethod` | NONE, GYRO_INTEGRATION, ACC_DOUBLE_INTEGRATION, GYRO_ACC_BLEND |
| `EARTH_RADIUS_M` | 6_371_000.0 |

#### 데이터클래스

| 클래스 | 핵심 필드 | 메서드 |
|---|---|---|
| `GpsSample` | lat, lon, point_E/N, pos_ts, pos_valid, course_rad, speed_mps, motion_ts, motion_valid | `timestamp` (pos_ts), `has_valid_position/point/velocity/nav` |
| `ImuSample` | roll/pitch/yaw, acc_x/y/z, gyr_x/y/z, mag_x/y/z, timestamp, lin_acc_x/y/z, gyrz_valid, yaw_valid, acc_valid, lin_acc_valid, freefall, tumble | — |
| `BarometerSample` | altitude, timestamp, pressure, valid | — |
| `FreshResult` | point_fresh, velocity_fresh, imu_fresh, imu_gyrz_fresh, imu_yaw_fresh, imu_acc_fresh, imu_linear_acc_fresh, barometer_fresh + age_s | `gyrz_fresh/gyrz_age_s/baro_fresh` 별칭 |
| `GuidanceState` | origin_lat/lon/ready, target_lat/lon/E/N/ready, gps/imu/baro_history, nav_E/N/course/V/vE/vN/confidence/dr_age/control_mode/dr_method, dr_start_E/N/V/course/time/vE/vN, yaw_at_dropout, gyro_integral_since_dropout, last_dr_update_time, detumble_exit_start | — |
| `L1Input` | valid, reason, control_mode, dr_method, confidence, E/N/vE/vN/V/course, origin_E/N, target_E/N/lat/lon, point_age, velocity_age, imu_age, barometer_age, dr_age | — |
| `L1Output` | timestamp, control_valid, nominal, reason, control_mode, dr_method, confidence, origin_E/N, target_E/N, target_bearing, nu, distance_to_target, yaw_rate_cmd, yaw_rate_limit_dps, ground_speed_mps, crossTrack, alongTrack, pos_E/N, carrot_E/N, current_heading_rad, pid_enabled, kp_override | — |

#### 함수 (실행/호출 순서)

| # | 함수 | 호출자 | 역할 |
|---:|---|---|---|
| **유틸** | | | |
| u1 | `latlon_to_ne(lat, lon, olat, olon) → (N, E)` | convert_latlon_to_local_en | 등각 평면 근사 변환 |
| u2 | `ne_to_latlon(N, E, olat, olon) → (lat, lon)` | (외부, 텔레메트리) | 역변환 |
| u3 | `convert_latlon_to_local_en(lat, lon, olat, olon) → (E, N)` | decidefresh, convert_target_to_local_en_if_possible | **E first** 규약 |
| u4 | `wrap_pi(angle_rad)` | produceL1output, _estimate_dr_course | [-π, π) wrap |
| u5 | `clamp(x, lo, hi)` | saturated_sin, produceL1output, DR update | 일반 클램프 |
| u6 | `saturated_sin(nu)` | produceL1output | **sin(clamp(nu, ±π/2))** — 입력 제한 sat |
| u7 | `compute_dr_confidence(dr_age)` | produceL1input | 3-segment 선형: 1.0 → 0.5 → 0.0 |
| u8 | `choose_yaw_rate_limit(control_mode, dr_method)` | produceL1output | 모드별 한계 (rad/s) |
| **내부 헬퍼** | | | |
| h1 | `_prune_history(history, oldest_ts)` | decidefresh | window 밖 샘플 제거 |
| h2 | `_latest(history)` | decidefresh, _should_detumble, produceL1input | 마지막 샘플 또는 None |
| h3 | `_ok(v)` | 전반 | math.isfinite 안전 래퍼 |
| h4 | `_last_valid_position(state)` | produceL1input | gps_history 역순 valid pos |
| h5 | `_last_valid_point(state)` | produceL1input | EN 유효 sample |
| h6 | `_last_valid_velocity(state)` | decidefresh, produceL1input | motion 유효 sample |
| h7 | `_last_valid_gps_sample(state)` | produceL1input GPS_TRACKING | 위치+속도 모두 유효 sample |
| h8 | `_can_dead_reckon(state, fresh)` | produceL1input DR 분기 | DR 가능 여부 |
| h9 | `_circular_mean(a, b)` | _estimate_dr_course | atan2 기반 각도 평균 |
| h10 | `_estimate_dr_course(state, li, fresh)` | _update_state_from_dead_reckoning | yaw_delta + gyro_integral 블렌드 |
| h11 | `_update_state_from_dead_reckoning(state, fresh, now)` | produceL1input DR 분기 | DR 위치/속도/dr_method 갱신 |
| h12 | `_should_detumble(state, fresh, now)` | produceL1input | 임계+히스테리시스 판정 |
| h13 | `_fail_l1input(reason, state, fresh)` | produceL1input 모든 FAIL 분기 | FAIL L1Input 빌더 |
| **퍼블릭 파이프라인** | | | |
| p1 | `decidefresh(gps, imu, baro, state, now)` | ctrl_parafoil | GpsSample/ImuSample/BarometerSample append → prune → FreshResult |
| p2 | `produceL1input(fresh, gps, imu, state, release_state, now)` | ctrl_parafoil | gate (state<3, origin, target, detumble) → GPS_TRACKING / DR / FAIL 분기 → L1Input 반환. **side effects: state.origin/target/nav_*/dr_start_* 갱신** |
| p3 | `produceL1output(l1input)` | ctrl_parafoil | 순수 계산: target_bearing = wrap_pi(atan2(dE, dN)); nu = wrap_pi(bearing-course); yaw_rate_cmd = 2V/L · saturated_sin(nu) · confidence → clamp → L1Output |
| p4 | `convert_target_to_local_en_if_possible(state)` | motorapp.init, produceL1input | origin 확보 후 target_lat/lon → target_E/N |
| p5 | `reset_guidance_state_for_flight(state)` | motorapp.handle_flight_state (state<3) | nav/DR/origin/target_E/N/ready/histories 모두 리셋 (target_lat/lon만 보존) |

### §3.3 control.py — yaw-rate controller + 모터 믹서

#### 모듈 레벨 상수 (config 또는 하드코딩)

| 이름 | 값/출처 | 역할 |
|---|---|---|
| `CTRL_MODE_NEUTRAL/CLOSED_LOOP/FEEDFORWARD_ONLY/GUIDANCE_TIMEOUT` | str | 컨트롤러 출력 mode 라벨 |
| `CTRL_FALLBACK_NONE/GUIDANCE_TIMEOUT/GUIDANCE_ATTENUATED/GYRO_SPIKE` | str | fallback 라벨 |
| `PARAFOIL_LEFT/RIGHT_MOTOR_PIN` | config GPIO | 서보 핀 |
| `ARM_MIN_DEG / NEUTRAL_ARM_DEG / ARM_MAX_DEG` | config (0/80/160) | arm 각도 규약 |
| `DELTA_ARM_MAX_DEG` | 160 (= 2 × min(80-0, 160-80)) | 하드웨어 차이 한계 |
| `LEFT_ZERO / RIGHT_ZERO / PULSE_PER_DEG` | config 캘리브레이션 | µs 변환 |
| `LEFT_NEUTRAL / RIGHT_NEUTRAL / LEFT_MIN/MAX_PULSE / RIGHT_MIN/MAX_PULSE` | 파생값 | clamp 한계 |
| `GUIDANCE_TIMEOUT_ATTENUATE_S / GUIDANCE_TIMEOUT_FAIL_S` | config (0.5/1.5) | 명령 age 기준 |
| `GYRO_SPIKE_LIMIT_DEG_S` | config (1500) | 스파이크 거부 임계 |
| `INTEGRAL_DECAY_RATE` | config (0.95) | gyro 없을 때 적분 감쇠 |

#### 데이터클래스

| 클래스 | 필드 |
|---|---|
| `ControlConfig` | ANGULAR_VELOCITY_CMD_MAX_DEG_S (40), ANGULAR_VELOCITY_DEADBAND_DEG_S (5), **DELTA_FF_MAX_DEG (160)**, DELTA_MIN_EFFECTIVE_DEG (5), EXPO (1.15), ERROR_DEADBAND_DEG_S (2), K_P (0.30), K_I (0.01), K_D (0), I_LIMIT_DEG (15), **DELTA_PID_MAX_DEG (30)**, **DELTA_TOTAL_MAX_DEG (160)**, MAX_ARM_RATE_DEG_S (60) |
| `_PIDState` | integral_deg, prev_error_deg, prev_time |
| `Ctrler` | config (ControlConfig), pid (_PIDState), prev_left_angle_deg, prev_right_angle_deg |
| `CtrlInput` | angular_velocity_cmd_deg_s, ground_speed_mps, valid, timestamp, pid_enabled, control_mode, dr_method, kp_override |
| `CtrlOutput` | timestamp, left_pw, right_pw, left_angle_deg, right_angle_deg, delta_arm_deg, delta_ff_deg, delta_pid_deg, angular_velocity_cmd_deg_s, angular_velocity_meas_deg_s, angular_velocity_error_deg_s, motor_cmd, saturated, sensor_valid, valid, mode, fallback_mode, guidance_command_age_s |

#### 함수 (실행/호출 순서)

| # | 함수 | 호출자 | 역할 |
|---:|---|---|---|
| **팩토리/리셋** | | | |
| f1 | `MakeCtrler(cfg=None) → Ctrler` | motorapp.init | Ctrler 생성 |
| f2 | `controller_reset(ctl)` | motorapp.handle_flight_state (state<3) | PID 및 이전 arm 각도 초기화 |
| f3 | `WriteNeutral(now, mode) → CtrlOutput` | _ctrl_cycle FAIL/manual 경로 | 중립 PWM 명령 객체 |
| **변환** | | | |
| f4 | `ProduceCtrlInput(g_out, now) → CtrlInput` | _ctrl_cycle | L1Output → CtrlInput (rad/s → deg/s) |
| f5 | `angular_velocity_to_delta_ff(cmd_dps, cfg) → delta_ff_deg` | ProduceCtrlOutput | expo curve + deadband |
| f6 | `ConnectRoMo(delta_arm_deg) → (left_pw, right_pw, left_ang, right_ang, delta)` | ProduceCtrlOutput, _manual_steer_command | 차이값 → 좌우 arm + µs PWM |
| **메인 step** | | | |
| f7 | `ProduceCtrlOutput(ctl, cmd, gyrz_meas_dps, now) → CtrlOutput` | _ctrl_cycle | (1) cmd.valid/NaN guard, (2) timeout, (3) clamp, (4) FF, (5) gyro spike check, (6) PID (gyrz 유효 시) or FF only, (7) sum & clamp ±DELTA_TOTAL_MAX, (8) ConnectRoMo, (9) slew limit, (10) 상태 업데이트 |
| **GPIO** | | | |
| g1 | `init_control() → pi or None` | motorapp.init | pigpio 연결 + 서보 zero |
| g2 | `WriteZero(pi)` | ctrl_parafoil Gate1 | 양팔 0° (arm up) |
| g3 | `WriteOff(pi)` | ctrl_parafoil Gate2 | PWM cut (pulse=0) |
| g4 | `Set180(pi)` | (external use) | 양팔 180° |
| g5 | `ProducePulse(pi, cmd)` | _ctrl_cycle | cmd.left_pw / right_pw 적용 |

### §3.4 데이터 흐름 요약 (1 cycle, GPS_TRACKING_CLOSED)

```
_CACHE                      _GUIDANCE_STATE                      _CONTROLLER
─────                       ───────────────                      ───────────
latest_gps    ─snapshot─┐
latest_imu    ─snapshot─┤
latest_baro   ─snapshot─┤
target_lat/lon          ↓
start_lat/lon       ┌────────┐
                    │ snap   │
                    └────┬───┘
                         │
                         ↓
                  decidefresh ──── append ──→ gps/imu/baro_history
                         │       ←── return ── FreshResult
                         ↓
                  produceL1input ──→ updates: nav_*, dr_start_*, origin, target
                         │       ←── return ── L1Input
                         ↓
                  produceL1output (pure)
                         │
                         ↓
                       L1Output (yaw_rate_cmd, control_valid, pid_enabled, ...)
                         │
                         ↓
                  ProduceCtrlInput → CtrlInput (deg/s, pid_enabled)
                         │
                         ↓
                  ProduceCtrlOutput ── reads/updates ──→ ctl.pid.{integral, prev_error, prev_time}
                                                       ctl.prev_left/right_angle
                         │
                         ↓
                       CtrlOutput (left_pw, right_pw, delta_arm, mode, ...)
                         │
                         ↓
                  ProducePulse(PI, cmd) → 서보 PWM
                  sensorlog.log_motor_ctrl(cmd)
                  _send_diag(...) → commapp → ground
```

---

## §4. 지상 모터 동작 확인 — prevstate 시나리오별 가이드

### §4.1 prevstate JSON 위치

`lib/prevstate.py`의 `_STATE_FILE` (기본: `prevstate.json`, 보통 작업 디렉터리에 위치).
FSW 부팅 시 `prevstate.init_prevstate()`가 디스크에서 로딩.

### §4.2 모드별 prevstate 템플릿

> ⚠️ 지상에서 실행할 때는 GPS 신호가 없어도 prevstate에 `PREV_START_LAT/LON/LOCKED=1`을 설정해서 origin을 강제로 lock할 수 있다. 단, `PREV_START_LAT/LON`은 실제 발사장 근처 좌표여야 의미가 있다.

#### Mode A. GPS_TRACKING_CLOSED 확인 (정상 추적, PID 작동)

```json
{
  "PREV_STATE": 4,
  "PREV_MOTOR_ENABLED": 1,
  "PREV_TARGET_LAT": 37.561000,
  "PREV_TARGET_LON": 126.950000,
  "PREV_START_LAT":  37.551000,
  "PREV_START_LON":  126.950000,
  "PREV_START_LOCKED": 1,
  "PREV_BEARING": null,
  "PREV_ALT_CAL": 0, "PREV_MAX_ALT": 0,
  "PREV_PACKET_COUNT": 0, "PREV_ST_TIMEDELTA": 0.0, "PREV_YAW_OFFSET": 0.0,
  "PREV_SOLENOID_COUNT": 0, "PREV_SOLENOID_DONE": 0
}
```

**지상 입력 시퀀스 (CLI 또는 시뮬레이션):**
1. FSW 부팅 → `init()`이 origin=start_lat/lon, target=target_lat/lon, MOTOR_ENABLED=True 복원
2. `handle_flight_state("4")` IPC 또는 직접 호출 → STATE=4
3. **GPS 메시지 주입 (10 Hz)** — start 위치 근처에서 시작:
   ```
   handle_gps("37.551500,126.950000,{ts},0.0,5.0,{ts}")    # 0°N 방향, 5 m/s
   ```
4. **IMU 메시지 주입 (50 Hz, gyrz 작은 값)** — 정상 비행 가정:
   ```
   handle_imu("0,0,0,0,0,-9.81,0,0,0,0,0,2.0,{ts},0,0,1")
   ```
5. 첫 사이클 후 모터 명령 관측:
   - 기대: target이 북쪽이고 course 북쪽이므로 nu≈0 → 작은 right (target_lat가 미세 동쪽) 또는 0
   - 모터: `left_angle≈80°, right_angle≈80°` (직진)
6. GPS 좌표를 살짝 동쪽으로 옮겨가며 nu를 만들면 → 모터가 RIGHT (delta_arm>0)
7. **확인 채널**: `_send_diag` 페이로드 또는 `sensorlog.log_motor_ctrl(cmd)` 출력에서 `mode=CLOSED_LOOP`, `delta_ff_deg>0`, `delta_pid_deg≠0` 확인

#### Mode B. GPS_TRACKING_OPEN 확인 (gyrz stale, FF only)

위 Mode A에서 **IMU 송신만 중단** (1.5초 이상 미수신). 그러면:
- `fresh.imu_gyrz_fresh = False`
- guidance: `GPS_TRACKING_OPEN`
- control: `pid_enabled=True`지만 `measured_dps=NaN` → 내부에서 `sensor_valid=False` → `mode=FEEDFORWARD_ONLY`
- 모터: FF 단독으로 같은 방향 명령 (PID trim 없음)

**확인**: `_send_diag`에서 `mode=FEEDFORWARD_ONLY`, `delta_pid_deg=0`, `sensor_valid=0`.

#### Mode C. DR_TRACKING_CLOSED 확인 (GPS dropout, IMU 정상)

1. Mode A처럼 시작
2. GPS_TRACKING_CLOSED 진입 확인 (mode 로그)
3. **GPS 송신 중단**, IMU 계속 (특히 gyrz)
4. 2초 후 (`GPS_FRESH_MAX_AGE_S=2.0`) `fresh.point_fresh=False` → DR 분기
5. `produceL1input` → `_update_state_from_dead_reckoning`이 nav_E/N 적분
6. `mode = DR_TRACKING_CLOSED`, `confidence < 1.0` (decay), `dr_method = GYRO_INTEGRATION`
7. 모터: yaw_rate_limit가 20 dps로 줄어 권한 제한
8. **확인**: diag `mode`, `confidence`, `dr_method` 값 추적

#### Mode D. DR_TRACKING_OPEN 확인 (GPS+gyrz dropout, yaw만)

- C의 절차에서 IMU의 **gyrz_deg_s만 NaN/매우 늦은 ts** 주입, yaw_rad는 fresh 유지
- 현재 imuapp 구현상 gyrz만 따로 stale 만들기 어려움 → 시뮬레이션에서 직접 `FreshResult(imu_gyrz_fresh=False, imu_yaw_fresh=True)` 주입하는 방식이 실용적
- 모터: FF only, 권한 15 dps

#### Mode E. DETUMBLING 확인 (강한 회전 시 역방향 제동)

1. Mode A처럼 시작 (motor enabled, state 4)
2. **IMU gyrz를 큰 값으로 주입** — 200 dps 이상 (deg/s, imuapp이 음수화하므로 IMU raw는 -200 dps 등):
   ```
   handle_imu("0,0,0,0,0,-9.81,0,0,0,0,0,200.0,{ts},0,1,1")   # tumble=1 옵션
   ```
3. `_should_detumble` 통과 → `mode = DETUMBLING`
4. `produceL1output`: `yaw_rate_cmd=0`, `pid_enabled=True`, `kp_override=KP_DETUMBLE=0.20`
5. control: `error = 0 - 200 = -200`, `pid = 0.20 × -200 = -40°` (clamped to ±DELTA_PID_MAX=30°)
6. **확인**: diag `mode=CLOSED_LOOP`, `delta_pid_deg=-30°` (역방향 제동), `delta_ff_deg=0`
7. gyrz를 임계 아래로 (30 dps 이하) 1초 유지 → 정상 모드 복귀

#### Mode F. FAIL 확인 (target 미설정 또는 GPS+IMU 모두 stale)

`PREV_TARGET_LAT=0, PREV_TARGET_LON=0` 으로 prevstate 작성 → init에서 target 거부 → first cycle에서 `FAIL_NO_TARGET` → control: `mode=NEUTRAL`, `left_pw=LEFT_NEUTRAL`, `right_pw=RIGHT_NEUTRAL`.

#### Mode G. Manual steer (LEFT/RIGHT)

```
handle_mtr("RIGHT")    # 또는 LEFT, NEUTRAL
```
- Gate 3 통과 → `_manual_steer_command` → delta = ±MANUAL_STEER_DELTA_DEG (=60°)
- 모터: `delta_arm_deg = +60`, `left=50°, right=110°` (RIGHT 케이스)
- diag `mode=MANUAL_RIGHT`

### §4.3 지상 테스트 권장 도구

| 도구 | 사용처 |
|---|---|
| `tests/test_motor_simulation.py::TestMultiCycleSimulation` | 모드 전환 시퀀스 자동 검증 (회귀용) |
| `motorapp._ctrl_cycle(None, now)` 직접 호출 | 하드웨어 없이 한 사이클씩 step-by-step 확인 |
| pigpio + 실서보 + prevstate JSON | 풀-스택 dry run (PWM 실측) |
| `_send_diag` 페이로드 (commapp → ground) | 실시간 텔레메트리 (29필드) |
| `sensorlog.log_motor_ctrl(cmd)` | 로컬 CSV 로그 |

### §4.4 검증 체크리스트 (모드별)

지상 테스트 시 각 모드에서 아래를 확인:

```
[ ] STATE = 4 (release deployed) 진입 확인
[ ] MOTOR_ENABLED = True
[ ] origin_ready = True (start_lat/lon lock 후)
[ ] target_ready = True (target 투영 후)
[ ] guidance.ControlMode 라벨 일치
[ ] control.mode 라벨 일치
[ ] delta_arm_deg 부호가 nu 부호와 일치
[ ] left_angle + right_angle = 160° (선형 합 유지)
[ ] PWM이 LEFT/RIGHT_MIN/MAX_PULSE 범위 내
[ ] slew rate < MAX_ARM_RATE_DEG_S × dt
[ ] guidance_command_age_s < 0.5s (정상)
```

---

## §5. 실비행 계수 튜닝 가이드

### §5.1 튜닝 우선순위 (영향력 큰 순서)

1. **L1 게인 `L_GAIN_M`** — 회두 응답 시정수 결정. 가장 먼저 조정.
2. **yaw_rate_limit 모드별** — 권한 한계. 너무 좁으면 saturation 빈발, 너무 넓으면 oscillation.
3. **PID `K_P` (KP_GPS_CLOSED)** — closed-loop trim 강도.
4. **DETUMBLE 임계 + hold** — spin 안정성. 자유낙하 데이터로 결정.
5. **SPEED_DECAY_TAU_S** — DR 정확도. 실측 활공 속도 곡선 필요.
6. **DELTA_ARM_MAX / DELTA_TOTAL_MAX** — 하드웨어 한계 검토.
7. **ACC_LIMIT_MPS2** — acc-blend 활성화 여부.

### §5.2 비행 데이터 수집 → 튜닝 사이클

```
1. 비행 전: 현재 파라미터로 짧은 테스트 비행 (가능하면 풍선 부양)
2. 비행 중 텔레메트리 + 로컬 로그 확보 (Motor.csv + IMU.csv + GPS.csv)
3. 분석:
   - L1 nu vs time 그래프 → 평균 nu (수렴 시간)
   - delta_arm vs time → saturation 비율
   - yaw_rate_cmd vs measured → tracking error
   - mode 전환 횟수 (chattering 여부)
4. 파라미터 조정 (한 번에 1-2개)
5. 재시뮬레이션 (test_motor_simulation 으로 동작 확인)
6. 다음 비행
```

### §5.3 파라미터별 튜닝 룰

#### L_GAIN_M (현재 10.0)
- **너무 작다 (≤6)**: 빠른 회두 → oscillation, overshoot
- **너무 크다 (≥15)**: 느린 회두 → 큰 target 오차
- **튜닝 신호**: target 도달 후 ±5m 안에서 oscillate → L_GAIN을 1.5배
- **공식 기반**: 응답시정수 τ ≈ L / (2V). V=8m/s에서 L=10이면 τ=0.625s (적정)

#### yaw_rate_limit (모드별)
- **GPS_CLOSED (현재 40 dps)**:
  - 텔레메트리에서 `yaw_rate_cmd` 분포 측정
  - 99-percentile이 한계의 80% 이내면 적정
  - 한계에 자주 닿으면 (+5 dps)씩 상향, oscillation 발생 시 (-5 dps) 후퇴
- **DR_*_OPEN (현재 15 dps)**:
  - DR 정확도 낮을수록 보수적
  - 분석 도구: GPS 복귀 시 DR-실위치 거리. 10m 이상이면 한계 축소

#### KP_GPS_CLOSED (현재 0.30)
- **너무 작다**: error 누적 → 평균 nu 크고 도달 늦음
- **너무 크다**: oscillation, saturation, motor 진동
- **튜닝 신호**: yaw_rate_error 추세선이 천천히 0으로 수렴하면 (+0.05), oscillate하면 (-0.05)
- **임계값**: K_P > 0.5는 거의 항상 oscillation. 0.20-0.40 사이 권장.

#### KP_DETUMBLE (현재 0.20)
- 자유낙하 spin 1227 dps 데이터 기준 산정
- 더 큰 spin 발생 시 (-0.05), 제동 안 될 때 (+0.05)
- DELTA_PID_MAX (30°) clamp 때문에 K_P × spin > 150 이상은 saturated

#### DETUMBLE 임계 (entry 150, exit 30, hold 1.0s)
- **entry 150 dps**: 정상 spin (≤50 dps) 대비 3배 마진
- 비행 데이터에서 false trigger 비율 측정 → 5% 이상이면 (+30)
- **exit 30 + hold 1.0s**: 임계 근처 chattering 방지
- chattering 발생 시 hold (+0.5s) 또는 exit threshold (-10)

#### GYRO_SPIKE_LIMIT_DEG_S (현재 1500)
- BNO085 max range ±2000 dps의 75%
- spike 거부율이 1% 이상이면 너무 보수적 (상향)
- 실제 IMU glitch (단발 노이즈)는 매우 드물어 1500 충분

#### SPEED_DECAY_TAU_S (현재 6)
- DR 비행 시 GPS 복귀 시점의 추정-실측 차이로 검증
- 추정 속도 > 실측 → tau 단축
- 추정 속도 < 실측 → tau 연장
- 5-12s 범위에서 조정

#### ACC_LIMIT_MPS2 (현재 2.0)
- 정상 활공 acc 분포 (linear, gravity 제거) 측정
- 95-percentile이 한계의 50% 이하면 안전
- 임팩트/spin 시점 acc는 5+ m/s²이므로 한계 2.0은 적절히 차단

#### DELTA_TOTAL_MAX_DEG (현재 160)
- 하드웨어 ARM_MAX 한계와 동일하게 유지
- 한계 도달 빈도 (`saturated=1` 비율) 1% 이상이면 yaw_rate_cmd 한계 또는 K_P 검토

### §5.4 튜닝 안티패턴 (피해야 할 변경)

| 안티패턴 | 이유 |
|---|---|
| 여러 파라미터 동시 변경 | 인과관계 추적 불가 |
| 시뮬레이션 통과 후 실비행 미검증 | 모델 불일치 누적 |
| 한 번의 비행으로 결론 | 풍, GPS 노이즈 등 외란 변동 큼 — 3회 이상 평균 |
| `K_I` 과다 (0.05 이상) | 적분 windup → overshoot |
| `L_GAIN` 비대칭 (좌우 회두 다르게) | 기체 dynamics 비대칭 = 모델 결함, 게인 트릭으로 보정 위험 |
| DETUMBLE 임계 너무 낮춤 | 정상 spin이 detumble 진입 → 가이던스 중단 |
| `MAX_ARM_RATE_DEG_S` 너무 큼 (≥120) | slew 한계 무의미 → 서보 부하 |
| `GYRO_SPIKE_LIMIT` 낮춤 (< 800) | 정상 spin 차단 → CLOSED_LOOP 미작동 |

### §5.5 실비행 후 점검 항목

매 비행 후 로그에서 다음을 확인:

```
1. mode 전환 시퀀스: 0 → 1 → 2 → 3 → 4 → 5 정상 진행?
2. GPS dropout 발생 시 DR 사용률 (DR_TRACKING_* 비율)
3. DETUMBLING 진입 횟수 + 평균 hold 시간
4. saturated=True 사이클 비율
5. yaw_rate_error RMS
6. 최종 도달 거리 (TARGET_RADIUS_M 대비)
7. 모터 명령 vs 측정 yaw rate 시간 그래프 (PID tracking)
8. left_pw / right_pw min/max — 서보 한계 도달 여부
```

### §5.6 권장 파라미터 변경 단위

| 파라미터 | 1회 조정 권장 단위 |
|---|---|
| `L_GAIN_M` | ±1.0 |
| `yaw_rate_limit` | ±5 dps |
| `K_P` | ±0.05 |
| `K_I` | ±0.005 |
| `K_D` | ±0.01 (도입 시 매우 작게 시작) |
| `KP_DETUMBLE` | ±0.05 |
| `DETUMBLE_*_THRESHOLD` | ±20 dps |
| `DETUMBLE_EXIT_HOLD_S` | ±0.5 s |
| `SPEED_DECAY_TAU_S` | ±2 s |
| `ACC_LIMIT_MPS2` | ±0.5 m/s² |

---

## §6. 빠른 참조

### §6.1 핵심 상수 (현재 값)

| 항목 | 값 |
|---|---|
| **L1**: `L_GAIN_M` | 10.0 m |
| **L1**: `V_MIN/MAX_MPS` | 0.5 / 15.0 m/s |
| **L1**: `saturated_sin` | sin(clamp(nu, ±π/2)) |
| **L1**: `TARGET_RADIUS_M` | 5.0 m |
| **Fresh**: `GPS/IMU/BARO_FRESH_MAX_AGE_S` | 2.0 / 1.5 / 2.0 s |
| **DR**: `SPEED_DECAY_TAU_S`, `DR_CONF_AGE_{1,2,3}_S` | 6.0 / 2,5,8 s |
| **DR**: `ACC_LIMIT_MPS2`, `ACC_BLEND_WEIGHT` | 2.0 / 0.2 |
| **Yaw limits dps**: GPS_CLOSED/OPEN, DR_CLOSED/OPEN | 40 / 25 / 20 / 15 |
| **Detumble**: entry/exit/hold | 150 dps / 30 dps / 1.0 s |
| **Spike**: `GYRO_SPIKE_LIMIT_DEG_S` | 1500 dps |
| **Gains**: K_P / K_I / K_D / KP_DETUMBLE | 0.30 / 0.01 / 0 / 0.20 |
| **Auth**: DELTA_FF / PID / TOTAL / ARM_MAX | 160 / 30 / 160 / 160 (deg) |
| **Mech**: ARM_MIN/NEUTRAL/MAX | 0 / 80 / 160 deg |
| **PWM cal**: LEFT_ZERO/RIGHT_ZERO/PULSE_PER_DEG | 2480 / 636 / 11.11 µs/deg |
| **Loop**: MOTOR_RATE_HZ | 20 Hz |
| **Timeout**: ATTENUATE/FAIL | 0.5 / 1.5 s |

### §6.2 IPC 메시지 ID (motor inbound)

| MID 이름 | 값 | 송신자 → 수신자 | 페이로드 |
|---|---|---|---|
| `GpsAppArg.MID_motor_gps` | 1501901 | gpsapp → motorapp | lat,lon,pos_ts,course_deg,spd_mps,motion_ts |
| `ImuAppArg.MID_motor_imu` | 1401901 | imuapp → motorapp | roll,pitch,yaw,ax,ay,az,mx,my,mz,gx,gy,gz_dps,ts,freefall,tumble,health |
| `BarometerAppArg.MID_motor_alt` | 1301901 | baroapp → motorapp | alt_m,sample_ts,sink_rate |
| `FlightlogicAppArg.MID_motor_state` | 1101902 | flightlogic → motorapp | state(int) |
| `FlightlogicAppArg.MID_motor_TargetCor` | 1101901 | flightlogic → motorapp | lat,lon |
| `FlightlogicAppArg.MID_motor_burnwire` | 1101903 | flightlogic → motorapp | TRIGGER:reason |
| `FlightlogicAppArg.MID_motor_EggDrop` | 1101904 | flightlogic → motorapp | (any) |
| `CommAppArg.MID_RouteCmd_MEC/MTR/FAC/CMC` | — | commapp → motorapp | ON/OFF / LEFT/RIGHT/NEUTRAL / ON/OFF, ALL/REL/EGG / mode |
| `MotorAppArg.MID_comm_motor_diag` | (outbound) | motorapp → commapp | 29필드 diag payload |
| `MainAppArg.MID_TerminateProcess` | 1001001 | main → all | (any) |

### §6.3 검증된 테스트 (205건)

| 파일 | 테스트 수 |
|---|---|
| `test_motor_simulation.py` | 14 (prevstate-driven sim) |
| `test_motor_integrity.py` | 53 (헬퍼/edge case/데이터 흐름) |
| `test_motor_control.py` | 34 (control 단위) |
| `test_motor_guidance_homing.py` | 46 (guidance 파이프라인) |
| `test_motorapp.py` | 39 (핸들러/캐시) |
| `test_motor_ipc_harness.py` | 17 (IPC 디스패치) |
| `test_motorapp_start_lock.py` | 2 (origin lock) |

---

문서 끝.
