# motorapp / guidance 변수·함수 레퍼런스 + 전 경우의 수 수치 예시

> 기준 코드: 2026-05-29  
> 변경 이력:
> - 2026-05-29: 섹션 8 추가 — 비행 중 모드 전이 시나리오 (Origin 획득, GPS dropout→DR, GPS 복구, pos-only fallback, STATE 전환 리셋, 전이 흐름도)
> - 2026-05-29: 섹션 7 추가 — 센서 값 경우의 수 전체 열거 (GPS/IMU/Baro 4레이어, ControlMode 매트릭스, DR 앵커 초기화, Timeout 수치 일람)
> - 2026-05-29: CASE 5 오류 수정 — `_should_detumble`은 `DecideControlMode` 이전 호출, `ProduceDetumbleOutput`은 중립이 아닌 최대 반대편향 출력
> - 이전: UpdateAnchors→UpdateRaws 이름 변경 / NavState 중첩 구조 / IMU_FRESH_MAX_AGE_S=3.0 / GPS_STALE_TIMEOUT_SEC=15.0 / DR 속도감쇄 제거 / NMEA 캐시 레이어 제거

---

## 1. 전역 변수 목록

### motorapp.py

| 변수 | 타입 | 초기값 | 의미 |
|------|------|--------|------|
| `STATE` | int | 0 | 비행 상태 (3=낙하중, 5=착지) |
| `MOTOR_ENABLED` | bool | False | 서보 활성 여부 (MEC ON/OFF) |
| `MOTORAPP_RUNSTATUS` | bool | True | 앱 루프 실행 여부 |
| `RELEASE_ACTION_ENABLED` | bool | True | 번와이어 허용 |
| `EGG_ACTION_ENABLED` | bool | True | 솔레노이드 허용 |
| `PI` | pigpio handle | None | 하드웨어 GPIO 핸들 |
| `_ORIGIN_SAVED` | bool | False | origin→prevstate 동기화 완료 여부 |
| `_CACHE_t` | `_Cache` | 빈 구조체 | 최신 raw 센서 스냅샷 |
| `_UPDATE_LOCK` | Lock | — | `_CACHE_t` 보호 |
| `_CTRLER_t` | `Ctrler\|None` | None | PID 컨트롤러 상태 |
| `_CTRL_LOCK` | Lock | — | `_CTRLER_t.reset` 동시성 |
| `_PREV_STATE` | int | 0 | 직전 비행 상태 |

---

### guidance.py

| 변수 | 타입 | 의미 |
|------|------|------|
| `_MISSION_t` | `MissionFrame` | origin/target 좌표 (비행 1회 설정) |
| `_STATE_t` | `GuidanceState` | 항법 추정값·앵커·DR 전체 |

#### MissionFrame 필드

| 필드 | 초기값 | 의미 |
|------|--------|------|
| `origin_lat/lon` | nan | 출발 GPS 좌표 |
| `origin_ready` | False | origin 확보 여부 |
| `target_E/N` | nan | origin 기준 목표 투영 좌표 (m) |
| `target_ready` | False | 투영 완료 여부 |
| `_target_lat/lon` | nan | 투영 전 raw 목표 좌표 |
| `_raw_lat/lon` | nan | 마지막 GPS 수신값 (origin 후보) |

#### GuidanceState 필드

| 필드 | 초기값 | 의미 |
|------|--------|------|
| `gps` | `GpsAnchor()` | 마지막 신선 GPS 앵커 |
| `imu` | `ImuAnchor()` | 마지막 신선 IMU 앵커 |
| `baro` | `BaroAnchor()` | 마지막 신선 기압계 앵커 |
| `nav` | `NavState()` | 현재 항법 추정값 서브구조 |
| `dr` | `DRState()` | GPS dropout 이후 적분 상태 |
| `detumble_exit_start` | nan | DETUMBLING 탈출 히스테리시스 타이머 |

#### NavState 필드 (`_STATE_t.nav.*`)

| 필드 | 초기값 | 의미 |
|------|--------|------|
| `nav.E` | nan | 현재 위치 E (m, origin 기준) |
| `nav.N` | nan | 현재 위치 N (m, origin 기준) |
| `nav.course` | nan | 현재 진행방향 (rad) |
| `nav.V` | nan | 현재 속도 (m/s) |
| `nav.confidence` | 0.0 | 항법 신뢰도 [0, 1] |
| `nav.control_mode` | FAIL | 현재 제어 모드 (ControlMode enum) |

#### DRState 필드 (`_STATE_t.dr.*`)

| 필드 | 초기값 | 의미 |
|------|--------|------|
| `anchor_E/N` | nan | GPS 마지막 위치 (앵커, 이후 불변) |
| `anchor_V` | nan | 앵커 시점 속도 (m/s), **감쇄 없이 유지** |
| `anchor_course` | nan | 앵커 시점 진행방향 (rad, 불변) |
| `anchor_time` | nan | 앵커 잠금 시각 (monotonic, 불변) |
| `yaw_at_anchor` | nan | 앵커 잠금 시점 IMU yaw (rad) |
| `gyro_integral` | 0.0 | 앵커 이후 누적 yaw 변화 (rad) |
| `last_step_time` | nan | 직전 DR 스텝 시각 |
| `method` | NONE | DRMethod enum |
| `confidence` | 0.0 | DR 신뢰도 [0,1] (L1 cmd 스케일링용) |

---

## 2. 함수 목록

### motorapp.py

| 함수 | 입력 | 출력 | 역할 |
|------|------|------|------|
| `_cache_snapshot()` | — | `_Cache` | `_CACHE_t` 깊은 복사 |
| `_compute_linear_acc(roll,pitch,ax,ay,az)` | deg, m/s² | (lax,lay,laz) | body-frame 가속도에서 중력 제거 |
| `handle_gps(data)` | CSV str | — | GPS 페이로드 파싱 → `_CACHE_t.latest_gps` 갱신 |
| `handle_imu(data)` | CSV str | — | IMU 페이로드 파싱 → `_CACHE_t.latest_imu` 갱신 |
| `handle_barometer(data)` | CSV str | — | 기압계 페이로드 파싱 → `_CACHE_t.latest_baro` 갱신 |
| `handle_target_coord(data)` | CSV str | — | 타겟 좌표 유효성 검사 → `guidance.set_target()` |
| `handle_flight_state(data)` | CSV str | — | STATE 갱신, state<3 시 guidance/PID 리셋 |
| `handle_mec(data)` | "ON"/"OFF" | — | `MOTOR_ENABLED` 토글 + prevstate 저장 |
| `handle_fac(data)` | CSV str | — | 액추에이터 허용 플래그 제어 |
| `_sync_origin_to_prevstate()` | — | bool | origin 확정 시 prevstate에 1회 저장 |
| `_ctrl_cycle(main_queue, now)` | float | `CtrlOutput\|None` | 한 사이클 전체 제어 계산 |
| `ctrl_parafoil(main_queue)` | — | — | 20Hz 제어 루프 스레드 |
| `dispatch(msg)` | str | — | MID별 핸들러 라우팅 |
| `init()` | — | — | prevstate 복원 + 컨트롤러 초기화 |
| `motorapp_main(queue, pipe)` | — | — | 진입점: init + 스레드 + dispatch 루프 |

### guidance.py

| 함수 | 입력 | 출력 | 역할 |
|------|------|------|------|
| `latlon_to_ne(lat,lon,orig_lat,orig_lon)` | deg | (N,E) m | GPS→로컬 NE 변환 |
| `dr_is_valid(dr)` | `DRState` | bool | 앵커 5개 모두 finite 확인 |
| `dr_lock(dr,E,N,V,course,yaw,now)` | — | — | GPS 신선 시 DR 앵커 잠금·적분 리셋 |
| `dr_reset(dr)` | — | — | 비행 리셋 시 DR 전체 초기화 |
| `dr_estimate_course(dr,imu)` | — | float rad | gyro 적분+yaw delta 블렌드로 heading 추정 |
| `_compute_dr_confidence(age)` | float s | float [0,1] | DR 신뢰도 계산 |
| `UpdateRaws(gps,imu,baro,now)` | raw 센서 | — | duck-type 센서 → 앵커 갱신 (매 사이클) |
| `DecideControlMode(now)` | float | `ControlMode` | 신선도 판단 → 6개 모드 중 결정 |
| `_update_state_from_dead_reckoning(now)` | float | — | DR 적분 수행, `nav.*` 갱신 |
| `ProduceL1Input(now)` | float | `L1Input` | origin 획득 + nav 갱신 + L1Input 생성 |
| `ProduceL1Output(l1in)` | `L1Input` | `L1Output` | L1 공식으로 yaw_rate_cmd 계산 (pure) |
| `set_target(lat,lon)` | deg | — | 타겟 설정, origin 있으면 즉시 투영 |
| `reset()` | — | — | origin/nav/DR 초기화 (target lat/lon 보존) |

### control.py

| 함수 | 입력 | 출력 | 역할 |
|------|------|------|------|
| `ProduceCtrlInput(l1_output, now)` | `L1Output`, float | `CtrlInput` | rad/s → deg/s 단위 변환 |
| `angular_velocity_to_delta_ff(cmd_dps, cfg)` | dps, cfg | float deg | Expo 형상 FF arm 편향각 계산 |
| `ConnectRoMo(delta_arm_deg)` | deg | (lpw,rpw,la,ra,da) | 차동 편향각 → arm 각도 + PWM |
| `ProduceCtrlOutput(ctl, cmd, gyrz, now)` | — | `CtrlOutput` | FF + PID + slew-rate → PWM 출력 |
| `ProduceDetumbleOutput(now)` | float | `CtrlOutput` | DETUMBLING: 중립 서보 반환 |
| `MoveServo(pi, ctrl_out)` | — | — | pigpio로 실제 PWM 출력 |

---

## 3. 공통 전제 및 설정값

```
비행 상태: STATE=3, MOTOR_ENABLED=True
origin:    (37.4490°N, 127.1230°E)  →  지역 좌표 (E=0, N=0)
target:    (37.4600°N, 127.1234°E)  →  투영 결과 target_E=+33m, target_N=+1223m

config.py 주요 값:
  L_GAIN_M = 17.0           GPS_FRESH_MAX_AGE_S = 15.0
  V_MIN_MPS = 0.5           IMU_FRESH_MAX_AGE_S = 3.0
  V_MAX_MPS = 15.0          GPS_STALE_TIMEOUT_SEC(gpsapp) = 15.0
  V_MAX_DR_MPS = 3.0
  NU_DEADBAND_DEG = 15.0
  TARGET_RADIUS_M = 5.0
  DETUMBLE_GYRZ_THRESHOLD_DPS = 200.0
  DETUMBLE_EXIT_THRESHOLD_DPS = 30.0
  DETUMBLE_BRAKE_DELTA_DEG = 80.0

  ControlConfig 기본값:
    ANGULAR_VELOCITY_CMD_MAX_DEG_S = 40.0  (GPS_TRACKING_CLOSED 한계)
    ANGULAR_VELOCITY_DEADBAND_DEG_S = 5.0
    DELTA_FF_MAX_DEG = 160.0
    DELTA_MIN_EFFECTIVE_DEG = 5.0
    EXPO = 1.15
    ERROR_DEADBAND_DEG_S = 2.0
    K_P = 0.0  (PID OFF 테스트중)
    K_I = 0.0,  K_D = 0.0
    MAX_ARM_RATE_DEG_S = 200.0

서보 중립:
  left_neutral  = 2480 - 80*11.11 = 1591 µs
  right_neutral = 636  + 80*11.11 = 1525 µs
```

---

## 4. 경우의 수별 함수 호출 체인

---

### CASE 1 — GPS_TRACKING_CLOSED (nu < 데드밴드, 직진)

#### 좌표 평면 (origin 기준 NE 좌표, 축척 압축)

```
N(m)
 ↑
1223 ┤                    ★ target (33, 1223)
     │
     │           ↗ course=10°
  56 ┤       ✈ vehicle (17.7, 55.6)
     │
   0 ┼───────────────────────── E(m)
     0        17.7   33

 nu = target_bearing(0.75°) - course(10°) = -9.24°
 → |nu| < 15° 데드밴드 → 조향 없음 (직진)
```

#### 입력 센서 데이터

```
dispatch("GPS|37.4495,127.1232,1,1000.000,10.0,4.0,1,1000.000")
dispatch("IMU|0,0,45,0,0,9.81,0,0,5,1,1000.010")
  gyrz_raw=+5°/s (BNO085 기준 CCW)
```

#### ① handle_gps(data) → _CACHE_t.latest_gps

```
파라미터 파싱:
  lat=37.4495, lon=127.1232, pos_health=1, pos_ts=1000.000
  course_deg=10.0, speed_mps=4.0, motion_health=1, motion_ts=1000.000

→ _GpsFromApp(
    lat=37.4495,            lon=127.1232,
    pos_ts=1000.000,        pos_health=1,
    course_rad=0.1745 rad,  speed_mps=4.0,
    motion_ts=1000.000,     motion_health=1,
    rx_ts=monotonic()
  )
→ _CACHE_t.latest_gps = 위 객체
```

#### ② handle_imu(data) → _CACHE_t.latest_imu

```
파라미터 파싱:
  roll=0°, pitch=0°, yaw=45°, ax=ay=0, az=9.81, gyrz_raw=+5°/s, health=1, ts=1000.010

_compute_linear_acc(0°, 0°, 0, 0, 9.81):
  g_x = -sin(0)*9.81 = 0,  g_y = cos(0)*sin(0)*9.81 = 0
  lin_ax=0, lin_ay=0

gyrz_rad_s = radians(-5.0) = -0.0873 rad/s  ← Z-up CCW → nav CW 부호 반전

→ _ImuFromApp(
    yaw_rad=0.7854 rad,      gyrz_rad_s=-0.0873 rad/s,
    lin_acc_x=0, lin_acc_y=0, lin_acc_valid=True,
    health=1,                ts=1000.010,  rx_ts=monotonic()
  )
→ _CACHE_t.latest_imu = 위 객체
```

#### ③ _ctrl_cycle(now=1000.020)

```
with _UPDATE_LOCK:
    motor_enabled = True,  state = 3
    snap_t = _cache_snapshot()   ← _CACHE_t 깊은 복사

state=3 → 활성 분기 진입
_CTRLER_t 이미 초기화됨
```

#### ④ UpdateRaws(snap_t.latest_gps, snap_t.latest_imu, snap_t.latest_baro, 1000.020)

```
[GPS]  pos_health=1 → lat,lon,pos_ts 처리
  latlon_to_ne(37.4495, 127.1232, 37.4490, 127.1230):
    dLat=radians(0.0005)=8.727e-6 rad, N=8.727e-6*6371000 = 55.6 m
    dLon=radians(0.0002)=3.491e-6 rad, E=3.491e-6*6371000*cos(37.449°) = 17.7 m

갱신 결과:
  _STATE_t.gps.E=17.7,  _STATE_t.gps.N=55.6
  _STATE_t.gps.pos_ts=1000.000,  _STATE_t.gps.pos_valid=True
  _MISSION_t._raw_lat=37.4495,  _MISSION_t._raw_lon=127.1232

  motion_health=1 →
  _STATE_t.gps.V=4.0,  _STATE_t.gps.course=0.1745 rad
  _STATE_t.gps.motion_ts=1000.000,  _STATE_t.gps.motion_valid=True

[IMU]  health=1, ts=1000.010 →
  _STATE_t.imu.ts=1000.010
  _STATE_t.imu.yaw=0.7854 rad,   _STATE_t.imu.yaw_valid=True
  _STATE_t.imu.gyr_z=-0.0873 rad/s,  _STATE_t.imu.gyrz_valid=True
  _STATE_t.imu.lin_acc_x=0,  _STATE_t.imu.lin_acc_y=0
  _STATE_t.imu.lin_acc_valid=True
```

#### ⑤ DecideControlMode(now=1000.020)

```
신선도 판단:
  pos_fresh:  pos_valid=True,  age=0.020s ≤ 15.0  → True
  vel_fresh:  motion_valid=True,  age=0.020s ≤ 15.0  → True
  imu_fresh:  ts=1000.010,  age=0.010s ≤ 3.0  → True
  gyrz_fresh: imu_fresh=True, gyrz_valid=True  → True

DETUMBLING: gyrz_dps=abs(degrees(-0.0873))=5.0 < 200.0  → 진입 안 함
GPS_TRACKING_CLOSED: pos+vel+gyrz 모두 fresh  → ✅

반환: ControlMode.GPS_TRACKING_CLOSED
부수효과: _STATE_t.nav.control_mode = GPS_TRACKING_CLOSED
```

#### ⑥ ProduceL1Input(now=1000.020)

```
origin_ready=True, target_ready=True
mode = GPS_TRACKING_CLOSED  ← 모드 분기

[nav 갱신] GPS → nav 직접 할당:
  _STATE_t.nav.E       = _STATE_t.gps.E      = 17.7 m
  _STATE_t.nav.N       = _STATE_t.gps.N      = 55.6 m
  _STATE_t.nav.course  = _STATE_t.gps.course = 0.1745 rad
  _STATE_t.nav.V       = _STATE_t.gps.V      = 4.0 m/s
  _STATE_t.nav.confidence = 1.0

[DR 앵커 갱신]
  imu_yaw = _STATE_t.imu.yaw = 0.7854 rad (yaw_valid=True)
  dr_lock(dr, E=17.7, N=55.6, V=4.0, course=0.1745, yaw=0.7854, now=1000.020)
    dr.anchor_E=17.7,  dr.anchor_N=55.6
    dr.anchor_V=4.0,   dr.anchor_course=0.1745
    dr.anchor_time=1000.020,  dr.yaw_at_anchor=0.7854
    dr.gyro_integral=0.0  (리셋)

반환: L1Input(
  valid=True,    reason="GPS_TRACKING",
  control_mode=GPS_TRACKING_CLOSED,
  confidence=1.0,
  E=17.7,        N=55.6,
  V=4.0,         course=0.1745 rad,
  target_E=33.0, target_N=1223.0
)
```

#### ⑦ ProduceL1Output(l1in)

```
입력: E=17.7, N=55.6, V=4.0, course=0.1745, target_E=33, target_N=1223

dE = 33.0 - 17.7 = 15.3 m
dN = 1223.0 - 55.6 = 1167.4 m
dist = hypot(15.3, 1167.4) = 1167.5 m  > 5.0 → 타겟 미도달

target_bearing = atan2(15.3, 1167.4) = 0.01311 rad (0.75°)
nu = wrap_pi(0.01311 - 0.1745) = -0.1614 rad (-9.24°)

|nu| = 9.24° < NU_DEADBAND(15°)  → sin_nu_eff = 0.0

V_eff = clamp(4.0, 0.5, 15.0) = 4.0 m/s
yaw_rate_cmd = 2.0 * 4.0 / 17.0 * 0.0 * 1.0 (confidence) = 0.0 rad/s
lim = radians(40.0) = 0.698 rad/s → clamp → 0.0 rad/s

반환: L1Output(
  control_valid=True,    nominal=True,
  reason="GPS_TRACKING", control_mode=GPS_TRACKING_CLOSED,
  yaw_rate_cmd=0.0 rad/s,
  yaw_rate_limit_dps=40.0,
  nu=-0.1614 rad(-9.24°),
  target_bearing=0.01311 rad,
  distance_to_target=1167.5 m,
  ground_speed_mps=4.0,
  pid_enabled=True
)
```

#### ⑧ ProduceCtrlInput(l1out, now=1000.020)

```
입력: l1out.yaw_rate_cmd=0.0 rad/s, control_valid=True, pid_enabled=True

반환: CtrlInput(
  angular_velocity_cmd_deg_s = degrees(0.0) = 0.0 dps,
  ground_speed_mps = 4.0,
  valid = True,
  pid_enabled = True,
  control_mode = GPS_TRACKING_CLOSED
)
```

#### ⑨ ProduceCtrlOutput(_CTRLER_t, ctrl_in, gyrz_meas=-5.0 dps, now)

```
입력:
  angular_velocity_cmd_deg_s = 0.0 dps
  angular_velocity_meas_deg_s = degrees(-0.0873) = -5.0 dps  ← snap_t.latest_imu.gyrz_rad_s 변환

① FF 계산:
  clamped = clamp(0.0, -40, +40) = 0.0
  |0.0| < DEADBAND(5.0)  → delta_ff = 0.0°

② 스파이크 검사:
  |gyrz| = 5.0 < 1500  → sensor_valid = True

③ PID (pid_active=True, K_P=0.0):
  error = 0.0 - (-5.0) = 5.0 dps
  |error| = 5.0 > ERROR_DEADBAND(2.0) → error = 5.0
  delta_pid = K_P(0.0) * 5.0 = 0.0°  ← PID OFF 상태

④ 합산:
  delta_sum = delta_ff(0.0) + delta_pid(0.0) = 0.0°

⑤ ConnectRoMo(0.0°):
  left_angle  = 80 - 0/2 = 80°
  right_angle = 80 + 0/2 = 80°
  left_pw  = int(2480 - 80*11.11) = 1591 µs
  right_pw = int(636  + 80*11.11) = 1525 µs

반환: CtrlOutput(
  mode=CLOSED_LOOP,
  delta_ff=0.0°,  delta_pid=0.0°,  delta_arm=0.0°,
  left_angle=80°,    right_angle=80°,
  left_pw=1591µs,    right_pw=1525µs,  ← 중립
  sensor_valid=True, valid=True
)
```

#### ⑩ MoveServo(PI, ctrl_out) → 하드웨어 출력

```
pigpio.set_servo_pulsewidth(LEFT_PIN,  1591)
pigpio.set_servo_pulsewidth(RIGHT_PIN, 1525)
→ 양쪽 arm = 80° (중립) → 패러포일 직진
```

---

### CASE 1b — GPS_TRACKING_CLOSED (nu > 데드밴드, 왼쪽 선회)

#### 좌표 평면

```
N(m)
 ↑
1223 ┤                    ★ target (33, 1223)
     │                   /
     │                  / target_bearing=0.75°
     │                 /
     │                / ← nu=-89.25°
     │               ↗ (목표 방향)
  56 ┤        →→→ ✈ vehicle (17.7, 55.6)
     │            course=90° (정동향)
   0 ┼───────────────────────── E(m)
     0         17.7   33

 nu = 0.75° - 90° = -89.25°  → 목표는 기수 왼쪽 89°
 → 왼쪽 선회 명령 (yaw_rate_cmd < 0)
```

#### 변경된 입력

```
handle_imu("0,0,0,0,0,9.81,0,0,5,1,1000.010")  ← course=90° 시나리오

_STATE_t.nav.course = radians(90°) = 1.5708 rad
(GPS motion_health=1, course_deg=90.0으로 수신된 상태)
```

#### ⑦ ProduceL1Output — 변경 부분

```
E=17.7, N=55.6, V=4.0, course=1.5708 rad (90°)
target_E=33, target_N=1223

target_bearing = atan2(15.3, 1167.4) = 0.01311 rad (0.75°)
nu = wrap_pi(0.01311 - 1.5708) = -1.5577 rad (-89.25°)

|nu| = 89.25° > 15° → 데드밴드 통과
clamp(-1.5577, -π/2, π/2) = -1.5577  (±π/2=±1.5708 이내)
sin_nu_eff = sin(-1.5577) ≈ -1.000

V_eff = clamp(4.0, 0.5, 15.0) = 4.0
yaw_rate_cmd = 2.0 * 4.0 / 17.0 * (-1.000) * 1.0 = -0.4706 rad/s
lim = radians(40.0) = 0.6981
clamp(-0.4706, -0.6981, +0.6981) = -0.4706 rad/s = -26.96 dps

반환: L1Output(
  yaw_rate_cmd=-0.4706 rad/s,
  yaw_rate_limit_dps=40.0,
  nu=-1.5577 rad (-89.25°),
  pid_enabled=True
)
```

#### ⑧ ProduceCtrlInput

```
angular_velocity_cmd_deg_s = degrees(-0.4706) = -26.96 dps
```

#### ⑨ ProduceCtrlOutput(gyrz_meas=-5.0 dps)

```
① FF 계산:
  clamped = -26.96 dps
  |clamped| = 26.96 > 5.0 (DEADBAND) → 통과
  x = 26.96 / 40.0 = 0.674
  x^1.15 = 0.674^1.15 = exp(1.15*ln(0.674)) = exp(1.15*(-0.394)) = exp(-0.453) = 0.636
  delta = 5.0 + (160.0 - 5.0) * 0.636 = 5.0 + 98.6 = 103.6°
  delta_ff = copysign(103.6, -26.96) = -103.6°

② 스파이크 검사: sensor_valid=True

③ PID (K_P=0):
  error = -26.96 - (-5.0) = -21.96 dps
  |error| > 2.0 → error = -21.96
  delta_pid = 0.0 * (-21.96) = 0.0°

④ 합산:
  delta_sum = -103.6 + 0.0 = -103.6°
  clamp(-103.6, -160, 160) = -103.6°

⑤ Slew-rate 제한 (dt=0.05s, 첫 사이클):
  max_step = 200.0 * 0.05 = 10.0°/사이클
  left_des  = clamp(80 + 51.8, 0, 160) = 131.8°
  right_des = clamp(80 - 51.8, 0, 160) = 28.2°
  left_angle  = clamp(131.8, 80-10, 80+10) = 90°  ← slew 제한
  right_angle = clamp(28.2,  70,    90)    = 70°  ← slew 제한

⑥ PWM:
  left_pw  = int(2480 - 90*11.11) = 1480 µs
  right_pw = int(636  + 70*11.11) = 1414 µs

[정상 상태 수렴 후 — 수 사이클 뒤]:
  left_angle  → 131.8°,  right_angle → 28.2°
  left_pw  → int(2480 - 131.8*11.11) = 1016 µs
  right_pw → int(636  + 28.2*11.11)  = 949 µs

반환: CtrlOutput(
  mode=CLOSED_LOOP,
  delta_ff=-103.6°,  delta_pid=0.0°,  delta_arm=-103.6°,
  [첫 사이클]  left_angle=90°, right_angle=70°,
               left_pw=1480µs, right_pw=1414µs
  [수렴 후]    left_angle=131.8°, right_angle=28.2°,
               left_pw=1016µs, right_pw=949µs
)
```

#### ⑩ 물리적 해석

```
delta_arm < 0 → 왼쪽 회전 명령:
  left arm 각도  ↑ (더 내려감 = 왼쪽 줄 잡아당김)
  right arm 각도 ↓ (올라감  = 오른쪽 줄 풀림)
  → 패러포일 왼쪽 후방 당김 → 왼쪽으로 회전
```

---

### CASE 2 — GPS_TRACKING_OPEN

#### 좌표 평면

```
N(m)
 ↑
1223 ┤                    ★ target (33, 1223)
     │
     │           ↗ course=10°
  56 ┤       ✈ vehicle (17.7, 55.6)
     │
   0 ┼───────────────────────── E(m)
     0        17.7   33

 기하는 CASE 1b와 동일. 차이: IMU gyrz 없음 → PID 닫힌루프 불가
```

#### 변경된 입력

```
handle_imu("0,0,45,0,0,9.81,0,0,0,0,1000.010")
  health=0  → _CACHE_t.latest_imu.ts=1000.010, health=0
```

#### ⑤ DecideControlMode — 변경 부분

```
imu_fresh: health=0 → _STATE_t.imu.ts 미갱신(nan) → imu_fresh=False
gyrz_fresh=False, yaw_fresh=False

GPS_TRACKING_CLOSED: gyrz_fresh=False → ❌
GPS_TRACKING_OPEN:   pos+vel fresh, gyrz 불필요 → ✅

→ ControlMode.GPS_TRACKING_OPEN
```

#### ⑦ ProduceL1Output — 변경 부분

```
yaw_rate_limit_dps=25.0 (OPEN 한계)
lim = radians(25.0) = 0.4363 rad/s
clamp(-0.4706, -0.4363, +0.4363) = -0.4363 rad/s (한계 적용)
→ yaw_rate_cmd=-0.4363 rad/s = -24.99 dps
```

#### ⑨ ProduceCtrlOutput — 변경 부분

```
gyrz_meas = nan (health=0 → gyrz_rad_s 없음) or 0.0
sensor_valid = False  (not finite)

pid_active = False  ← gyrz 없음
mode = FEEDFORWARD_ONLY

delta_ff = angular_velocity_to_delta_ff(-24.99, cfg):
  x = 24.99/40.0 = 0.625
  0.625^1.15 = exp(1.15*ln(0.625)) = exp(1.15*(-0.470)) = exp(-0.541) = 0.582
  delta = 5.0 + 155.0 * 0.582 = 5.0 + 90.2 = 95.2°
  delta_ff = -95.2°

delta_sum = -95.2°  (pid 없음)
→ 서보 동작 CASE 1b와 유사하나 PID 없고, 한계가 25dps로 낮아 명령이 약간 작음
```

---

### CASE 3 — DR_TRACKING_CLOSED

#### 좌표 평면

```
N(m)
 ↑
1223 ┤                    ★ target (33, 1223)
     │                   ↗ 목표 방향
     │                  /
     │                 / target_bearing=-0.36°
     │                /
     │               / ← nu=-27.4°
     │           ↗ course=27.1°
 122 ┤     ✈ vehicle (40, 122)  ← 18초 DR 후 추정 위치
     │
   0 ┼───────────────────────── E(m)
     0    17.7 33 40

 GPS dropout 18초 경과. anchor=(17.7, 55.6) 기준으로 DR 적분.
 vehicle이 target의 동쪽으로 약간 오버슛.
```

#### 입력 상태

```
GPS: 마지막 수신 pos_ts=985.0, motion_ts=985.0  (age=18s > 15s → stale)
DR: anchor=(17.7, 55.6), anchor_V=4.0, anchor_course=0.1745, anchor_time=985.0
    gyro_integral=0.3 rad (18s 누적)
IMU: gyrz_rad_s=-0.0873 rad/s, ts=1002.990

handle_imu("0,0,62,0,0,9.81,0,0,5,1,1002.990")
  yaw=62°=1.082 rad, gyrz=-0.0873 rad/s, ts=1002.990
```

#### ⑤ DecideControlMode(now=1003.000)

```
pos_fresh:  age=18s > 15.0  → False
vel_fresh:  age=18s > 15.0  → False
imu_fresh:  age=0.01s ≤ 3.0  → True
gyrz_fresh: True

GPS 모드들: pos_fresh=False → ❌
DR_TRACKING_CLOSED: dr_is_valid=True, gyrz_fresh=True → ✅
  (confidence 체크 없음 — 모드 진입은 dr_is_valid만으로)

→ ControlMode.DR_TRACKING_CLOSED
```

#### ⑥ ProduceL1Input(now=1003.000) → _update_state_from_dead_reckoning

```
DR 분기 진입 → _update_state_from_dead_reckoning(1003.000) 호출

[dt 계산]
  dr.last_step_time = 1002.990 (이전 스텝)
  dt = 1003.000 - 1002.990 = 0.010s
  clamp(0.010, 0, 0.5) = 0.010s

[① gyro 적분]
  dr.gyro_integral += (-0.0873) * GYRZ_SIGN(1.0) * 0.010
  = 0.3 + (-0.000873) = 0.2991 rad

[② heading 추정: dr_estimate_course(dr, imu)]
  base = dr.anchor_course = 0.1745 rad
  course_gyro = wrap_pi(0.1745 + 0.2991) = 0.4736 rad (27.1°)
  course_yaw  = wrap_pi(0.1745 + wrap_pi(1.082 - 0.7854))
              = wrap_pi(0.1745 + 0.2966) = 0.4711 rad (27.0°)
  |yaw-gyro| = 0.0025 < π/4 → circular_mean
  course_est ≈ 0.4724 rad (27.1°)

[③ 속도 유지 — 감쇄 없음]
  V_dr = dr.anchor_V = 4.0 m/s

[④ EN 속도]
  vE = 4.0 * sin(0.4724) = 4.0 * 0.455 = 1.820 m/s
  vN = 4.0 * cos(0.4724) = 4.0 * 0.891 = 3.562 m/s

[⑤ 가속도계 보정] (ACC_LIMIT=2.0, lin_acc_x=lin_acc_y=0)
  hypot(0,0) = 0.0 ≤ 2.0 → 보정 적용
  aE=0, aN=0 → vE,vN 변화 없음

[⑥ 위치 갱신]
  _STATE_t.nav.E  += 1.820 * 0.010 = +0.0182
  _STATE_t.nav.N  += 3.562 * 0.010 = +0.0356
  (18s 누적 → 추정 위치: nav.E≈40.0m, nav.N≈122.0m)
  _STATE_t.nav.V      = 4.0
  _STATE_t.nav.course = 0.4724 rad
  dr.last_step_time   = 1003.000

[⑦ 신뢰도]
  age = 1003.0 - 985.0 = 18.0s
  _compute_dr_confidence(18.0):
    a2=5 < 18 ≤ a3=20
    conf = 0.5*(1-(18-5)/(20-5)) = 0.5*(1-0.867) = 0.067
  dr.confidence = 0.067,  _STATE_t.nav.confidence = 0.067

반환: L1Input(
  valid=True,    reason="DR_TRACKING",
  control_mode=DR_TRACKING_CLOSED,
  dr_method=GYRO_ACC_BLEND,
  confidence=0.067,
  E=40.0,        N=122.0,
  V=4.0,         course=0.4724 rad,
  target_E=33.0, target_N=1223.0
)
```

#### ⑦ ProduceL1Output(l1in)

```
dE = 33.0 - 40.0 = -7.0 m
dN = 1223.0 - 122.0 = 1101.0 m
dist = hypot(-7.0, 1101.0) = 1101.0 m

target_bearing = atan2(-7.0, 1101.0) = -0.00636 rad (-0.36°)
nu = wrap_pi(-0.00636 - 0.4724) = -0.4788 rad (-27.4°)

|nu| = 27.4° > 15° → 데드밴드 통과
sin_nu_eff = sin(-0.4788) = -0.461

V_eff = clamp(4.0, 0.5, V_MAX_DR=3.0) = 3.0 m/s  ← V_MAX_DR 상한
yaw_rate_cmd = 2.0 * 3.0 / 17.0 * (-0.461) = -0.1627 rad/s
             *= confidence(0.067) = -0.01090 rad/s
lim = radians(20.0) = 0.349 rad/s
clamp(-0.01090, -0.349, +0.349) = -0.01090 rad/s = -0.625 dps

반환: L1Output(
  control_valid=True,
  yaw_rate_cmd=-0.01090 rad/s (-0.625 dps),
  yaw_rate_limit_dps=20.0,
  nu=-0.4788 rad(-27.4°),
  confidence=0.067,
  pid_enabled=True
)
```

#### ⑧ ProduceCtrlInput

```
angular_velocity_cmd_deg_s = degrees(-0.01090) = -0.625 dps
```

#### ⑨ ProduceCtrlOutput(gyrz_meas=-5.0 dps)

```
① FF 계산:
  |clamped| = 0.625 < DEADBAND(5.0)  → delta_ff = 0.0°

② sensor_valid=True

③ PID (K_P=0):
  error = -0.625 - (-5.0) = 4.375 dps  (> ERROR_DEADBAND)
  delta_pid = 0.0 * 4.375 = 0.0°

④ delta_sum = 0.0°  → 중립 서보

결과: mode=CLOSED_LOOP
  left_pw=1591µs, right_pw=1525µs  ← 중립

해석: 신뢰도 6.7%로 yaw_rate_cmd가 0.625dps까지 억제됨
     → FF 데드밴드(5dps) 이내라 실제 조향 없음
     → 수렴을 기다리며 직진 유지 (GPS 복구 대기)
```

---

### CASE 4 — DR_TRACKING_OPEN

#### 좌표 평면

```
CASE 3과 동일 기하. 차이: gyrz 없어 gyro_integral 누적 불가
  course_est = yaw 단독 = 0.4711 rad
```

#### 변경된 입력

```
handle_imu: gyrz=nan (gyrz_valid=False), yaw=62°=1.082 rad (yaw_valid=True)
```

#### ⑤ DecideControlMode — 변경 부분

```
gyrz_fresh=False  → DR_TRACKING_CLOSED ❌
yaw_fresh=True    → DR_TRACKING_OPEN ✅
```

#### dr_estimate_course (gyro 없음)

```
course_gyro = None (gyrz_valid=False)
course_yaw  = wrap_pi(0.1745 + wrap_pi(1.082 - 0.7854)) = 0.4711 rad (27.0°)
→ yaw만 사용: course_est = 0.4711 rad
```

#### ⑦⑨ ProduceL1Output / ProduceCtrlOutput — 변경 부분

```
yaw_rate_limit_dps = 15.0  (OPEN, CLOSED=20.0보다 낮음)
  lim = radians(15.0) = 0.2618 rad/s

yaw_rate_cmd ≈ -0.01090 * (0.067/1.0) ... 거의 동일
→ 여전히 FF 데드밴드 이내 → delta_ff=0 → 중립 서보

mode = FEEDFORWARD_ONLY  ← gyrz 없으므로
```

---

### CASE 5 — DETUMBLING

> **주의**: `_should_detumble()`은 `DecideControlMode()` **이전**에 호출된다.
> True이면 L1 파이프라인 전체를 건너뛰고 즉시 반환.

#### 좌표 평면

```
N(m)
 ↑
     ┤
     │       ↻↻↻  vehicle (17.7, 55.6) CW 고속 스핀
  56 ┤       ✈ gyrz=+250 dps (오른쪽 회전)
     │
   0 ┼───────────────── E(m)

 _should_detumble() → True (|250| ≥ DETUMBLE_GYRZ_THRESHOLD=200)
 DecideControlMode(), ProduceL1Input/Output 호출 없음
```

#### 입력

```
handle_imu("0,0,45,0,0,9.81,0,0,250,1,1000.010")
  gyrz_deg_s=+250 (BNO085, CCW=양수 규약)
  → gyrz_rad_s = radians(-250) = -4.363 rad/s  ← 부호 반전 (handle_imu)
  → _STATE_t.imu.gyr_z = -4.363 rad/s  (UpdateRaws)
```

#### ③ _ctrl_cycle — _should_detumble() 분기

```
_fresh_gyrz_dps(snap_t, now):
  imu.health=1, ts=1000.010, age=0.01s ≤ 3.0s → 유효
  gyrz_rad_s=-4.363 → degrees(-4.363)=-250.0 dps
  반환: -250.0 dps

_should_detumble():
  gyrz_dps = -250.0,  abs(-250.0) = 250.0 ≥ 200.0
  _DETUMBLE_ACTIVE = True
  → True 반환

  → _ctrl_cycle 내부:
    guidance._STATE_t.nav.control_mode = ControlMode.DETUMBLING
    gz_meas = _fresh_gyrz_dps() = -250.0 dps
    ← DecideControlMode/ProduceL1Input/L1Output 호출 없음
```

#### ④ ProduceDetumbleOutput(now, gz_meas=-250.0 dps)

```
DELTA_ARM_MAX_DEG = 2.0 * min(80-0, 142-80) = 2.0 * 62 = 124°

① 스파이크 검사:
  gyro_finite = True
  |250| > GYRO_SPIKE_LIMIT(1500)? → False → sensor_valid=True

② 중립 조건:
  sensor_valid=True, |250| > CTRL_ERROR_DEADBAND(2.0) → 중립 건너뜀

③ 반대부호 최대 편향:
  delta = -copysign(124, -250.0) = +124°
  (gyrz < 0 = CCW 스핀 → delta > 0 = 오른쪽 회전 명령으로 상쇄)

④ ConnectRoMo(+124°):
  left_angle  = clamp(80 - 124/2,  0, 142) = clamp(80-62, 0, 142)  = 18°
  right_angle = clamp(80 + 124/2,  0, 142) = clamp(80+62, 0, 142)  = 142°
  left_pw  = int(2480 - 18*11.11)  = int(2480-200) = 2280 µs
  right_pw = int(636  + 142*11.11) = int(636+1578) = 2214 µs

반환: CtrlOutput(
  mode=DETUMBLING,
  delta_arm=+124°,
  left_angle=18°,     right_angle=142°,
  left_pw=2280µs,     right_pw=2214µs,   ← 최대 CW 편향
  sensor_valid=True,  valid=True
)

물리 해석:
  delta > 0 → left arm 위로(줄 풀림), right arm 아래로(줄 당김)
  → 패러포일 오른쪽 후방 당김 → CW(오른쪽) 회전 유발
  → 현재 CCW 스핀(-250dps)을 상쇄 ✓
```

#### CASE 5b — DETUMBLING 중립 (gyro=nan 또는 spike)

```
gz_meas = nan  (imu stale 또는 _fresh_gyrz_dps=None)
또는
|gz_meas| > 1500 dps (spike)

→ sensor_valid = False
→ 중립 출력:
  delta=0°, left_angle=80°, right_angle=80°
  left_pw=1591µs, right_pw=1525µs
  valid=True
```

#### DETUMBLING 탈출 타이머

```
매 사이클 _should_detumble():

[탈출 조건 진입] abs(gyrz_dps) ≤ 30.0 dps:
  최초 진입: _DETUMBLE_EXIT_START = now → return True (아직 유지)
  경과 < 1.0s: return True (유지)
  경과 ≥ 1.0s: _DETUMBLE_ACTIVE=False, _DETUMBLE_EXIT_START=nan → return False

[탈출 후] DecideControlMode() 재개 → GPS/DR/FAIL 중 결정
[스핀 재발] abs(gyrz_dps) ≥ 200 → 재진입
[히스테리시스] 탈출 구간(30dps)에서 재진입 구간(200dps) 사이 170dps 여유 → chattering 방지
```

---

### CASE 6 — FAIL

#### 좌표 평면

```
N(m)
 ↑
1223 ┤                    ★ target (33, 1223)
     │
     │           ? course=unknown
  56 ┤       ✈ vehicle (위치 불명)
     │         GPS>15s stale, DR anchor=nan, IMU>3s stale
   0 ┼───────────────────────── E(m)

 모든 센서 stale → 제어 불능 → 서보 OFF
```

#### 상태

```
GPS: pos_ts=900.0, age=103s > 15s → stale
DR:  anchor_time=nan  (또는 dr_is_valid=False)
IMU: ts=900.0, age=103s > 3.0s → imu_fresh=False
```

#### ⑤ DecideControlMode

```
pos_fresh=False,  vel_fresh=False
imu_fresh=False   (age=103s > 3.0s)
gyrz_fresh=False, yaw_fresh=False

모든 모드: ❌
→ ControlMode.FAIL
→ _STATE_t.nav.control_mode = FAIL
```

#### ⑥ _ctrl_cycle FAIL 분기

```
DETUMBLING: gyrz_fresh=False → 건너뜀
GPS/DR 모드들: 해당 없음

→ FAIL 분기:
if PI is not None:
    control.WriteOff(PI)
        → set_servo_pulsewidth(LEFT_PIN,  0)
        → set_servo_pulsewidth(RIGHT_PIN, 0)
return None
```

---

## 5. GPS freshness 레이어 (3단계)

| 레이어 | 파일 | 임계값 | 역할 |
|--------|------|--------|------|
| ~~NMEA 캐시~~ | ~~gps.py~~ | ~~2.0s~~ | **제거됨** — last-known 무조건 반환 |
| GPS 앱 차단 | gpsapp.py | **15.0s** | STALE 판정 시 motorapp에 GPS 전송 중단 |
| guidance freshness | config.py | **15.0s** | pos_fresh/vel_fresh 판단 → ControlMode 결정 |

---

## 6. 전체 분기 요약표

| 모드 | GPS pos | GPS vel | gyrz | yaw | DR앵커 | 서보 결과 |
|------|---------|---------|------|-----|--------|-----------|
| **GPS_TRACKING_CLOSED** | ✅ | ✅ | ✅ | — | 갱신 | L1+FF±PID, lim=40dps |
| **GPS_TRACKING_OPEN** | ✅ | ✅ | ❌ | — | 갱신 | L1+FF only, lim=25dps |
| **DR_TRACKING_CLOSED** | ❌ | ❌ | ✅ | — | ✅ | DR L1+FF±PID, cmd×conf, lim=20dps |
| **DR_TRACKING_OPEN** | ❌ | ❌ | ❌ | ✅ | ✅ | DR L1+FF only, cmd×conf, lim=15dps |
| **DETUMBLING** | — | — | ≥200dps | — | — | 중립 (80°/80°) |
| **FAIL** | ❌ | ❌ | ❌ | ❌ | ❌ | WriteOff (0µs) |

> **현재 PID 상태**: K_P=K_I=K_D=0.0 (PID OFF 테스트 중)  
> CLOSED 모드도 실질적으로 FF 전용. K_P 원복값: GPS=0.45, DR=0.15
>
> **DR 모드 진입**: confidence 체크 없음. `dr_is_valid()` + gyrz/yaw fresh만으로 진입.  
> confidence는 `ProduceL1Output`에서 yaw_rate_cmd에 곱해지는 스케일 팩터.  
> confidence=0 → cmd=0 되지만 모드는 DR_TRACKING_* 유지 (FAIL 아님).

---

## 7. 센서 값 경우의 수 전체 열거

> 각 레이어는 독립적으로 분기를 가짐.  
> 표기: ✅=조건 충족, ❌=조건 미충족, —=해당 없음.

---

### 7-A. GPS 경우의 수

#### 레이어 1: gpsapp.py (하드웨어 → IPC 메시지)

```
판단 함수:
  _eval_pos_fidelity()   → pos_health 결정
  _eval_motion_fidelity() → motion_health 결정
  _valid_age()            → age 검사 (age ∈ [-0.02s, GPS_STALE_TIMEOUT_SEC=15s])
```

**pos_health 결정 트리** (8개 게이트, 순서대로)

| 케이스 | 조건 | pos_health | motorapp 수신 lat/lon |
|--------|------|------------|----------------------|
| G-P1 | 정상: age≤15s, fix≥1, sats≥4, HDOP≤3.0, no-jump | 1 | 실제 좌표 |
| G-P2 | GPS 하드웨어 전혀 없음 (`gps_instance=None`) | — | 전송 없음 |
| G-P3 | gpsapp stale: age>15s | 0 | `nan` |
| G-P4 | fix_quality=0 (위성 없음) | 0 | `nan` |
| G-P5 | sats < 4 (`GPS_MIN_SATS`) | 0 | `nan` |
| G-P6 | HDOP > 3.0 | 0 | `nan` |
| G-P7 | jump rate > 30 m/s | 0 | `nan` |
| G-P8 | 좌표 (0,0) sentinel | 0 | `nan` |
| G-P9 | ~~기대 영역 외~~ (제거됨: 지리적 박스 게이트 없음, 전 세계 좌표 허용) | — | — |
| G-P10 | SIM_GPS_ACTIVE=True | 1 (강제) | 시뮬 좌표 |

**motion_health 결정 조건** (모두 AND)

| 케이스 | 조건 | motion_health |
|--------|------|---------------|
| G-M1 | pos_health=True AND rmc='A' AND 0.3≤speed≤40 AND 0≤course<360 | 1 |
| G-M2 | pos_health=False | 0 |
| G-M3 | rmc_status ≠ 'A' (정지 또는 신호 없음) | 0 |
| G-M4 | speed < 0.3 m/s (GPS_MIN_MOTION_MPS) | 0 |
| G-M5 | speed > 40 m/s (GPS_MAX_VALID_SPEED_MPS) | 0 |
| G-M6 | age > 15s | 0 |

#### 레이어 2: motorapp.py handle_gps() → `_GpsFromApp`

| pos_health | motion_health | _GpsFromApp 결과 |
|-----------|---------------|-----------------|
| 1 | 1 | lat/lon/pos_ts/course_rad/speed_mps/motion_ts 모두 유효 |
| 1 | 0 | lat/lon/pos_ts 유효, course_rad/speed_mps/motion_ts = None |
| 0 | 0 | 모든 필드 None |
| 0 | 1 | 불가 (pos_health=False → motion_health 강제 False) |

#### 레이어 3: guidance.py UpdateRaws() → `GpsAnchor`

| `_GpsFromApp` 상태 | GpsAnchor 변화 |
|-------------------|---------------|
| pos_health=1, lat/lon/ts valid, origin_ready=True | `gps.E/N` 갱신, `gps.pos_ts` 갱신, `gps.pos_valid=True` |
| pos_health=1, lat/lon/ts valid, **origin_ready=False** | `_raw_lat/lon` 갱신만, `gps.E/N` 미갱신 |
| pos_health=0 (전부 None) | GpsAnchor 전체 **미갱신** (이전 값 유지, pos_ts 도 갱신 안 됨) |
| motion_health=1 | `gps.V/course/motion_ts` 갱신, `gps.motion_valid=True` |
| motion_health=0 | gps.V/course/motion_ts 미갱신 (이전 값 유지) |

> `gps.pos_valid` 는 한 번 True 가 되면 절대 False 로 되돌아가지 않음.  
> freshness 는 `now - gps.pos_ts` 로만 판단한다.

#### 레이어 4: guidance.py DecideControlMode() — GPS freshness 판단

| gps.pos_valid | now - pos_ts | pos_fresh |
|---------------|-------------|-----------|
| False (한 번도 못받음) | — | False |
| True | ≤ 15.0s | True |
| True | > 15.0s | **False** (stale) |

| gps.motion_valid | now - motion_ts | vel_fresh |
|-----------------|----------------|-----------|
| False | — | False |
| True | ≤ 15.0s | True |
| True | > 15.0s | **False** |

#### 레이어 5: guidance.py ProduceL1Input() — origin 획득 및 DR 앵커 갱신

| 조건 | 결과 |
|------|------|
| origin_ready=False, pos_fresh=True, raw_lat/lon valid | origin 확정, gps.E/N=0.0 기록, target 재투영 |
| origin_ready=False, pos_fresh=False | `L1Input(valid=False, reason="NO_ORIGIN")` |
| origin_ready=True, target_ready=False | `L1Input(valid=False, reason="NO_TARGET")` |
| mode=GPS_TRACKING_*, gps.E/N/V/course NaN | `L1Input(valid=False, reason="GPS_NAN")` |
| mode=GPS_TRACKING_*, 모든 값 finite | `dr_lock()` 실행, L1Input(valid=True) |

---

### 7-B. IMU 경우의 수

#### 레이어 1: imuapp.py (하드웨어 → IPC 메시지)

| 케이스 | 조건 | HEALTH | payload |
|--------|------|--------|---------|
| I-1 | 정상 read, age≤1s (send 스레드 판단) | 1 | roll,pitch,yaw,acc,gyr,1,sample_mono_ts |
| I-2 | send 스레드: now-_last_sample_ts > **1.0s** | 0 | nan,nan,...,0,sample_mono_ts |
| I-3 | read 스레드: now-_last_sample_ts > **2.0s** 且 cooldown 지남 | 0 + reinit | BNO085 RST 펄스 후 재초기화 |
| I-4 | 연속 read 실패 **5회** | 0 + reinit | 재초기화 |
| I-5 | 시작 후 **15s** 미경과 | 1 (health는 정상) | yaw=raw (offset 미적용) |
| I-6 | 시작 후 15s 경과 (최초 1회) | 1 | yaw=0 영점 기록 후 offset 적용 |

#### 레이어 2: motorapp.py handle_imu() → `_ImuFromApp`

| health | 결과 |
|--------|------|
| 1 | roll/pitch/yaw/acc/gyr 모두 저장, `gyrz_rad_s = radians(-gyrz_deg_s)` (부호 반전), lin_acc 계산 |
| 0 | `_ImuFromApp(ts=sample_ts, rx_ts=..., health=0)` — 나머지 전부 None |

**부호 변환 상세:**

```
gyrz_deg_s (BNO085 원시, CCW=양수)
  → handle_imu: gyrz_rad_s = radians(-gyrz_deg_s)   ← Z-up→nav 반전
  → guidance: gyr_z *= GYRZ_SIGN(1.0)               ← 추가 반전 없음
  → _should_detumble: gyrz_dps = degrees(gyrz_rad_s) ← 복원: -원시값
    ∴ BNO085 +250dps(CCW) → _should_detumble 수신 -250dps
```

#### 레이어 3: guidance.py UpdateRaws() → `ImuAnchor`

| health | ts valid | 결과 |
|--------|----------|------|
| 1 | valid | `imu.ts`, `imu.yaw`, `imu.gyr_z`, `imu.lin_acc_x/y` 갱신 |
| 0 | — | ImuAnchor **전체 미갱신** (이전 값 + 이전 ts 유지) |
| 1 | yaw=None | `imu.yaw` 미갱신, `imu.yaw_valid=False` 유지 |
| 1 | gyrz=None | `imu.gyr_z` 미갱신, `imu.gyrz_valid=False` 유지 |
| 1 | lin_acc_valid=False | `imu.lin_acc_x/y` 미갱신, `imu.lin_acc_valid=False` |

#### 레이어 4: motorapp.py `_fresh_gyrz_dps()` — detumble용 별도 신선도 판정

| 조건 | 반환 |
|------|------|
| health=1, ts finite, gyrz finite, age ≤ **3.0s** | `degrees(gyrz_rad_s)` (dps) |
| health=0 | `None` → `_should_detumble` False |
| age > 3.0s | `None` → `_should_detumble` False |
| gyrz=nan (gyrz_valid=False) | `None` |

#### 레이어 5: guidance.py DecideControlMode() — IMU freshness 판단

| imu.ts | age | gyrz_valid | yaw_valid | 결과 |
|--------|-----|-----------|-----------|------|
| nan | — | — | — | imu_fresh=False, gyrz_fresh=False, yaw_fresh=False |
| finite | ≤ **3.0s** | True | True | imu_fresh=True, gyrz_fresh=True, yaw_fresh=True |
| finite | ≤ 3.0s | False | True | gyrz_fresh=False, yaw_fresh=True → DR_OPEN 가능 |
| finite | ≤ 3.0s | True | False | gyrz_fresh=True, yaw_fresh=False → CLOSED 가능 |
| finite | > 3.0s | any | any | imu_fresh=False, 모두 False |

#### gyrz 값에 따른 동작 분기 (레이어 통합)

| |gyrz_dps| 범위 | _should_detumble | ProduceDetumbleOutput | ProduceCtrlOutput |
|---|---------|-----------------|----------------------|-------------------|
| 0 | 0~2.0 | False | 중립 (deadband) | 중립 (deadband) |
| 2.0~5.0 | False | — | FF deadband 이내 → 중립 |
| 5.0~200 | False | — | FF 동작 (expo 곡선) |
| ≥200 | **True** (DETUMBLING) | 최대 반대편향 | — |
| >1500 | True (spike) | 중립 (spike rejection) | 중립 (spike rejection) |
| nan/None | False | 중립 | 중립 (sensor_valid=False) |

---

### 7-C. Barometer 경우의 수

#### 레이어 1: barometerapp.py (하드웨어 → IPC)

| 케이스 | 조건 | BAROMETER_HEALTH | payload |
|--------|------|-----------------|---------|
| B-1 | 정상 read, age≤1s | 1 | alt_m, sink_rate, 1 |
| B-2 | send 스레드: age > **1.0s** | 0 | nan, nan, 0 |
| B-3 | BMP init 실패 (`_baro_hw=False`) | 0 | 0.0, nan, 0 |
| B-4 | BMP read 예외 | 0 | 0.0, nan, 0 |
| B-5 | sink_rate 스파이크 (|raw| > 30 m/s) | 1 | alt 정상, sink_rate=nan |
| B-6 | CAL 명령 수신 | 1 | `BAROMETER_OFFSET` 갱신 후 정상 alt |

#### 레이어 2: motorapp.py handle_barometer() → `_BaroFromApp`

| health | 결과 |
|--------|------|
| 1, alt/sink finite | `_BaroFromApp(alt_m=..., sink_rate=..., health=1)` |
| 1, alt=nan | `_BaroFromApp(alt_m=None, sink_rate=None, health=1)` |
| 0 | `_BaroFromApp(alt_m=None, sink_rate=None, health=0)` |

#### 레이어 3: guidance.py UpdateRaws() → `BaroAnchor`

| health | alt valid | 결과 |
|--------|----------|------|
| 1 | valid | `baro.alt_m`, `baro.ts`, `baro.valid=True` 갱신 |
| 0 | — | BaroAnchor 미갱신 |

> **현재 guidance.py는 baro를 ControlMode 결정이나 L1 계산에 사용하지 않음.**  
> `baro.alt_m`, `baro.sink_rate`는 저장만 됨 (미래 고도 기반 로직용 예약).

---

### 7-D. ControlMode 결정 전체 매트릭스

> 우선순위: DETUMBLING > GPS_TRACKING_CLOSED > GPS_TRACKING_OPEN > DR_TRACKING_CLOSED > DR_TRACKING_OPEN > FAIL

| | pos_fresh | vel_fresh | gyrz_fresh | yaw_fresh | dr_valid | \|gyrz\|≥200 | ControlMode | 서보 |
|---|-----------|-----------|-----------|-----------|---------|-------------|------------|------|
| **D-1** | ✅ | ✅ | ✅ | — | — | ❌ | GPS_CLOSED | L1+FF+PID, lim=40dps |
| **D-2** | ✅ | ✅ | ❌ | — | — | ❌ | GPS_OPEN | L1+FF only, lim=25dps |
| **D-3** | ❌ | — | ✅ | — | ✅ | ❌ | DR_CLOSED | DR L1+FF+PID×conf, lim=20dps |
| **D-4** | ❌ | — | ❌ | ✅ | ✅ | ❌ | DR_OPEN | DR L1+FF×conf, lim=15dps |
| **D-5** | any | any | any | any | any | ✅ | DETUMBLING | 최대반대편향 (또는 중립 if spike/nan) |
| **D-6** | ❌ | — | ❌ | ❌ | ✅ | ❌ | FAIL | WriteOff |
| **D-7** | ❌ | — | any | any | ❌ | ❌ | FAIL | WriteOff |
| **D-8** | ✅ | ❌ | any | any | ❌ | ❌ | FAIL | WriteOff |
| **D-9** | ✅ | ❌ | any | any | ✅ | ❌ | DR_CLOSED/OPEN | DR_is_valid이면 DR 진입 |

> **D-9 설명**: pos_fresh=True 이지만 vel_fresh=False 인 경우 (위성 느림).
> GPS_TRACKING 요건(pos+vel 모두) 미달 → DR 경로로 fallback.
> 단, DR 앵커가 없으면 FAIL.

**추가 모드 전환 불가 조건 (override):**

| 조건 | 결과 |
|------|------|
| `MOTOR_ENABLED=False` | WriteZero (모든 모드 무관) |
| `STATE < 3` | WriteZero (비행 전) |
| `STATE == 5` | WriteOff (착지) |
| `_CTRLER_t is None` | `MakeCtrler()` 후 정상 진행 |
| `PI is None` (pigpio 없음) | `MoveServo` no-op (로그만) |

---

### 7-E. DR 앵커 초기화 경우의 수

`ProduceL1Input()` 내부, GPS_TRACKING 분기 이외 구간에서 실행.

| 조건 | 결과 |
|------|------|
| dr_is_valid=True (이미 있음) | 앵커 초기화 건너뜀 |
| dr_is_valid=False, pos_fresh=True, imu_fresh=True, yaw_valid=True | `dr_lock(E,N,0.0,yaw,imu_yaw,now)` — 속도=0으로 정적 초기화 |
| dr_is_valid=False, pos_fresh=True, imu_fresh=True, gyrz_valid=True (yaw 없음) | `anchor_course=dr.anchor_course` (이전 알던 방향) 사용 |
| dr_is_valid=False, pos_fresh=True, imu_fresh=False | `L1Input(valid=False, reason="NO_HEADING_SOURCE")` |
| dr_is_valid=False, pos_fresh=False | 초기화 불가 (위치 기준점 없음) |

---

### 7-F. DR 신뢰도 구간별 동작

| 앵커 나이 (age = now - anchor_time) | confidence | yaw_rate_cmd 효과 |
|-------------------------------------|-----------|-------------------|
| 0 ~ 2.0s | 1.0 | 원본 명령 100% |
| 2.0 ~ 5.0s | 1.0 → 0.5 (선형) | 명령 100%→50% 감쇄 |
| 5.0 ~ 20.0s | 0.5 → 0.0 (선형) | 명령 50%→0% 감쇄 |
| > 20.0s | 0.0 | 명령 완전 제거 (모드는 DR 유지) |

> confidence=0 이어도 ControlMode는 DR_TRACKING_* 유지.  
> WriteOff 되지 않음 — FF 데드밴드(5dps) 이내면 중립 서보.

---

### 7-G. Timeout 조건 전체 수치 일람

| # | 파일 | 상수 | 값 | 조건 | 효과 |
|---|------|------|----|------|------|
| 1 | gpsapp | `GPS_STALE_TIMEOUT_SEC` | **15s** | age > 15s | GPS 변수 None 리셋, pos_health=0 전송 |
| 2 | imuapp | `IMU_STALE_TIMEOUT_SEC` | **1s** | age > 1s (send) | HEALTH=0, nan payload |
| 3 | imuapp | `IMU_STALE_REINIT_SEC` | **2s** | age > 2s (read) | BNO085 RST+reinit |
| 4 | imuapp | `IMU_REINIT_COOLDOWN_SEC` | **5s** | 재reinit 방지 쿨다운 | reinit 스킵 |
| 5 | imuapp | `IMU_MAX_CONSECUTIVE_ERRORS` | **5회** | 연속 read 실패 횟수 | reinit |
| 6 | imuapp | `_STARTUP_YAW_ZERO_DELAY_S` | **15s** | 시작 후 경과 시간 | yaw 영점 보류 |
| 7 | barometerapp | `BAROMETER_STALE_TIMEOUT_SEC` | **1s** | age > 1s | HEALTH=0 |
| 8 | motorapp | `IMU_FRESH_MAX_AGE_S` | **3s** | age > 3s | `_fresh_gyrz_dps`=None |
| 9 | motorapp | `DETUMBLE_EXIT_HOLD_S` | **1s** | 히스테리시스 유지 | Detumble 탈출 지연 |
| 10 | guidance | `GPS_FRESH_MAX_AGE_S` | **15s** | age > 15s | pos_fresh/vel_fresh=False |
| 11 | guidance | `IMU_FRESH_MAX_AGE_S` | **3s** | age > 3s | imu_fresh=False |
| 12 | guidance | `DR_CONF_AGE_1/2/3_S` | **2/5/20s** | 구간별 나이 | confidence 단계적 감쇠 |
| 13 | control | `_clamp_dt` lo/hi | **0.01/0.2s** | dt 범위 이탈 | default 0.1s 대체 |
| 14 | motorapp | `ctrl_thread.join` | **1s** | 종료 시 대기 | 스레드 강제 종료 |
| 15 | imuapp | `t1/t2.join` | **2s** | 종료 시 대기 | 스레드 강제 종료 |

---

## 8. 비행 중 모드 전이 시나리오

> 정적 상태(단일 ControlMode) 이외에 **시간에 따라 모드가 바뀌는** 전이 과정을 다룬다.

---

### 8-A. Origin 최초 획득 (비행 시작 직후)

**전제**: 전원 켜고 낙하 전, GPS가 처음으로 신호를 잡는 순간.

```
[T=0s] imuapp/gpsapp 시작. STATE=0 or 1.

[T=0 ~ T=first_GPS_lock]
  UpdateRaws(): pos_health=0 → gps.E/N 미갱신, gps.pos_valid=False
  ProduceL1Input(): origin_ready=False, pos_fresh=False
    → L1Input(valid=False, reason="NO_ORIGIN")
  → _ctrl_cycle: STATE<3 → WriteZero (비행 전이라 서보 무관)

[T=first_GPS_lock] GPS pos_health=1 최초 수신
  UpdateRaws():
    pos_health=1, origin_ready=False
    → mi._raw_lat = lat,  mi._raw_lon = lon    ← raw만 저장
    → gps.E/N 미투영 (origin 없음)
    gps.pos_ts = ts,  gps.pos_valid = True

  ProduceL1Input() 내부 origin 획득 분기:
    origin_ready=False, pos_fresh=True, _ok(_raw_lat)=True
    → mi.origin_lat = _raw_lat,  mi.origin_lon = _raw_lon
    → mi.origin_ready = True
    → gps.E = 0.0,  gps.N = 0.0    ← 첫 수신 위치가 origin (E=0, N=0)
    target_ready=False이면 아직 대기
    _ok(mi._target_lat) → True이면 즉시 투영:
      latlon_to_ne(target_lat, target_lon, origin_lat, origin_lon)
      → mi.target_E, mi.target_N,  mi.target_ready = True

  _sync_origin_to_prevstate() (motorapp):
    origin_ready=True → prevstate.update_start_point(lat, lon, True)  ← 1회만

[T=first_GPS_lock + 다음 사이클]
  UpdateRaws(): pos_health=1, origin_ready=True
    → latlon_to_ne() 투영 → gps.E/N 정상 갱신 시작
  DecideControlMode() → GPS_TRACKING_* 진입 가능
```

**핵심**: origin 확정 전 사이클은 모두 `reason="NO_ORIGIN"`. origin이 확정되는 그 사이클에 E=0, N=0이 기록되고 다음 사이클부터 실제 좌표가 들어온다.

---

### 8-B. GPS_TRACKING → DR_TRACKING 전환 (GPS dropout)

**전제**: GPS_TRACKING_CLOSED로 정상 비행 중 GPS 신호 소실.

```
타임라인:
  T=1000.0s  마지막 GPS pos/vel 수신
  T=1000.0~1015.0s  GPS 없음 (gpsapp: health=0 전송)
  T=1015.0s  pos_ts age = 15.0s → stale 경계

[T=1000.0~1014.99s]  GPS stale 미달 (age < 15s)
  UpdateRaws(): pos_health=0 → gps.E/N/V/course 미갱신 (1000.0s 값 유지)
  DecideControlMode():
    now - pos_ts = 14.99s < 15.0s → pos_fresh=True  ← 아직 GPS_TRACKING 유지
    GPS_TRACKING_CLOSED 또는 OPEN 유지
  ProduceL1Input():
    GPS_TRACKING 분기 → dr_lock() 매 사이클 호출
    dr.anchor_E/N/V/course = 1000.0s 시점 GPS 값 (매 사이클 덮어씀)
    dr.gyro_integral = 0.0  (매 사이클 리셋)

[T=1015.001s]  age = 15.001s → stale 초과 첫 사이클
  DecideControlMode():
    pos_fresh=False (15.001 > 15.0)
    vel_fresh=False
    gyrz_fresh=True (IMU 정상)
    dr_is_valid=True  ← 마지막 GPS_TRACKING 사이클에서 dr_lock됨
    → DR_TRACKING_CLOSED  ← 전환 발생

  ProduceL1Input():
    DR_TRACKING 분기 → _update_state_from_dead_reckoning()
    dt = 1015.001 - dr.anchor_time(1014.99) = 0.011s  ← 첫 DR 스텝
    gyro_integral += gyr_z * 0.011
    course_est = dr_estimate_course()
    nav.E += V * sin(course_est) * 0.011
    nav.N += V * cos(course_est) * 0.011
    confidence = _compute_dr_confidence(age=0.011s) = 1.0

변수 상태 비교:
  직전 사이클 (GPS):  nav.E = gps.E (정확)  confidence=1.0
  전환 사이클 (DR):   nav.E ≈ gps.E + ε    confidence=1.0 (아직 2s 이내)
  ← E/N 점프 없음. 앵커가 마지막 GPS값이므로 연속.
```

---

### 8-C. DR_TRACKING → GPS_TRACKING 복구 (GPS 재획득)

**전제**: DR_TRACKING_CLOSED 중 GPS 신호 복구.

```
타임라인:
  T=1015s  DR_TRACKING 전환 (앵커=마지막 GPS)
  T=1035s  DR로 20초 경과. confidence=0.0 (20s 이상)
  T=1036s  GPS pos/vel 재수신

[T=1036.0s] GPS 재획득 첫 사이클
  UpdateRaws():
    pos_health=1 → gps.E/N 갱신 (실제 현재 위치)
    motion_health=1 → gps.V/course 갱신
    gps.pos_ts = 1036.0,  gps.motion_ts = 1036.0

  DecideControlMode():
    pos_fresh=True (age=0s)
    vel_fresh=True
    gyrz_fresh=True
    → GPS_TRACKING_CLOSED  ← 복구

  ProduceL1Input():
    GPS_TRACKING 분기:
    nav.E = gps.E  ← DR 추정값 → GPS 실측값으로 즉시 덮어씀
    nav.N = gps.N  ← 점프 가능성 있음 (DR 오차 누적량에 따라)
    nav.V = gps.V
    nav.course = gps.course
    nav.confidence = 1.0

    dr_lock(dr, gps.E, gps.N, gps.V, gps.course, imu_yaw, 1036.0)
    dr.gyro_integral = 0.0  ← 적분 리셋
    dr.anchor_time   = 1036.0

주의 — nav 점프:
  DR 20초 누적 오차가 있으면 gps.E vs nav.E 사이 수십m 차이 가능.
  덮어쓰는 순간 L1 nu가 급변 → 조향 명령 급변 → slew-rate가 서보 속도 제한.
  max_step = MAX_ARM_RATE(200dps) × dt(0.05s) = 10°/사이클로 완충.
```

---

### 8-D. GPS pos만 있고 vel 없는 상태 (D-9 수치 예시)

**전제**: GPS 위성 충분하나 RMC 수신 늦음. pos_ts 신선, motion_ts stale.

```
상태:
  gps.pos_ts   = 1000.0s  (age=0.01s → pos_fresh=True)
  gps.motion_ts = 980.0s  (age=20.0s → vel_fresh=False)
  imu.gyrz_valid = True,  gyrz_fresh = True

DecideControlMode():
  pos_fresh=True,  vel_fresh=False
  → GPS_TRACKING_* 불가 (vel 필요)
  dr_is_valid=True (이전 GPS_TRACKING 사이클에서 앵커 있음)
  gyrz_fresh=True
  → DR_TRACKING_CLOSED

ProduceL1Input():
  GPS_TRACKING 분기 건너뜀 (mode≠GPS_TRACKING)
  pos_fresh=True, dr_is_valid=True → 앵커 초기화 건너뜀 (이미 있음)
  DR_TRACKING 분기:
    _update_state_from_dead_reckoning()
    ← GPS 위치는 있지만 사용 안 함; DR 앵커 기준 적분

실제 동작:
  GPS 위치(gps.E/N)는 신선하게 갱신되고 있지만 L1에는 사용 안 됨.
  DR 앵커(anchor_time=마지막 GPS_TRACKING 사이클)부터 적분으로만 항법.
  RMC 복구되면 다음 사이클에 vel_fresh=True → GPS_TRACKING 복귀.
```

---

### 8-E. STATE 전환에 따른 리셋 동작

`handle_flight_state(new_state)` 호출 시 전체 변화.

```
[new_state == STATE]  변화 없음 → return

[new_state < 3]  비행 전 상태로 복귀
  prevstate.clear_start_point()
  guidance.reset():
    _MISSION_t = MissionFrame()  ← origin/target/raw 전체 소실
    _STATE_t   = GuidanceState() ← GPS/IMU/baro 앵커, DR 전체 소실
    _MISSION_t._target_lat/lon = 이전 target 좌표 보존
  control.controller_reset(_CTRLER_t):
    pid.integral=0, pid.prev_error=0, pid.prev_time=0
    prev_left/right_angle = 80°
  _ORIGIN_SAVED = False   ← origin 재동기화 허용
  _DETUMBLE_ACTIVE = False
  _DETUMBLE_EXIT_START = nan

  결과:
    다음 사이클: origin_ready=False → NO_ORIGIN → WriteZero
    GPS 재수신 때까지 origin 재획득 대기

[new_state = 3 or 4]  낙하/강하 중
  새 상태만 저장. 리셋 없음.
  (STATE<3에서 3으로 갈 때도 리셋 없음 — 조건은 new_state<3)

[new_state = 5]  착지
  리셋 없음. _ctrl_cycle에서:
    state==5 → WriteOff → return None
  서보 PWM=0 유지. 다른 동작 없음.
```

**STATE 전환 요약표:**

| 전환 | guidance.reset() | controller_reset() | 서보 |
|------|-----------------|-------------------|------|
| any→0,1,2 | ✅ (origin 소실) | ✅ | WriteZero |
| any→3,4 | ❌ | ❌ | 정상 제어 |
| any→5 | ❌ | ❌ | WriteOff |

---

### 8-F. 모드 전이 전체 흐름도

```
전원 ON
  │
  ├─[STATE<3]─────────────────────────────────────────────────────┐
  │  WriteZero                                                     │
  │  GPS 수신 시작 → origin 획득 대기 (8-A)                        │
  │                                                                │
  └─[STATE=3 진입]                                                 │
       │                                                           │
       ├─[origin 없음] → NO_ORIGIN → WriteZero                    │
       │                                                           │
       ├─[origin 있음, GPS 신선]                                   │
       │    GPS_TRACKING_CLOSED/OPEN                               │
       │    dr_lock() 매 사이클 갱신                               │
       │         │                                                 │
       │         ├─[GPS age > 15s] → DR_TRACKING_CLOSED/OPEN (8-B)│
       │         │    DR 적분. confidence 감쇠.                    │
       │         │         │                                       │
       │         │         ├─[GPS 복구] → GPS_TRACKING 복귀 (8-C) │
       │         │         │    nav 점프 가능. slew로 완충.        │
       │         │         │                                       │
       │         │         └─[IMU stale] → DR_OPEN or FAIL        │
       │         │                                                 │
       │         └─[vel stale만] → DR_TRACKING (8-D)              │
       │              pos 있어도 DR 앵커 기준 항법                 │
       │                                                           │
       ├─[|gyrz| ≥ 200dps] → DETUMBLING (CASE 5)                  │
       │    최대 반대편향. GPS/DR 파이프라인 건너뜀.               │
       │    탈출 후 GPS/DR/FAIL 재판정.                            │
       │                                                           │
       ├─[모든 센서 stale] → FAIL → WriteOff (CASE 6)             │
       │                                                           │
       └─[STATE=5] → WriteOff 유지                                 │
                                                      ←───────────┘
                                              STATE<3 시 guidance.reset()
```
