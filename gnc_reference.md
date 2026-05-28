# motorapp / guidance 변수·함수 레퍼런스 + 전 경우의 수 수치 예시

> 기준 코드: 2026-05-29  
> 변경 이력: UpdateAnchors→UpdateRaws 이름 변경 / NavState 중첩 구조 / IMU_FRESH_MAX_AGE_S=3.0 / GPS_STALE_TIMEOUT_SEC=15.0 / DR 속도감쇄 제거 / NMEA 캐시 레이어 제거

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

#### 좌표 평면

```
N(m)
 ↑
     ┤
     │       ↻↻↻  vehicle (17.7, 55.6) 고속 스핀 중
  56 ┤       ✈ gyrz=250 dps
     │
   0 ┼───────────────── E(m)

 gyrz ≥ 200 dps → L1 계산 전에 DETUMBLING 진입
 목표 추종 없음 → 중립 서보로 스핀 감쇄 기다림
```

#### 입력

```
handle_imu("0,0,45,0,0,9.81,0,0,250,1,1000.010")
  gyrz_deg_s=250 → gyrz_rad_s=radians(-250)=-4.363 rad/s
  → _STATE_t.imu.gyr_z=-4.363 rad/s
```

#### ⑤ DecideControlMode

```
gyrz_fresh=True
gyrz_dps = abs(degrees(-4.363)) = 250.0 ≥ 200.0  → DETUMBLING 진입
nav.control_mode = DETUMBLING

반환: ControlMode.DETUMBLING
```

#### ⑥ _ctrl_cycle DETUMBLING 분기

```
mode == DETUMBLING → GPS/DR 파이프라인 건너뜀

ctrl_out_t = control.ProduceDetumbleOutput(now)
  → WriteNeutral(now, mode=CONTROL_MODE_DETUMBLING)
  → CtrlOutput(
      mode=DETUMBLING,
      left_angle=80°,    right_angle=80°,
      left_pw=1591µs,    right_pw=1525µs,  ← 중립
      valid=False
    )

control.MoveServo(PI, ctrl_out_t)
  → left_pw=1591, right_pw=1525
return ctrl_out_t  (ProduceL1Input/L1Output 호출 없음)
```

#### DETUMBLING 탈출

```
매 사이클:
  gyrz_dps < 30.0 → detumble_exit_start 타이머 시작
  → 1.0초 유지 시 탈출, nav.control_mode 초기화 후 GPS/DR 체크로 복귀
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
