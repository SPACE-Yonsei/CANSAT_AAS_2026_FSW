# motorapp / guidance 변수·함수 레퍼런스 + 전 경우의 수 수치 예시

> 기준 코드: 2026-05-29  
> 변경 이력: NavState 중첩 구조 반영 / IMU_FRESH_MAX_AGE_S 3.0 / GPS_STALE_TIMEOUT_SEC 15.0 / DR 속도감쇄 제거 / NMEA 캐시 레이어 제거

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
| `nav` | `NavState()` | 현재 항법 추정값 서브구조 (**아래 참조**) |
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
| `anchor_E/N` | nan | GPS 마지막 위치 (앵커) |
| `anchor_V` | nan | 앵커 시점 속도 (m/s), DR 중 **감쇄 없이 유지** |
| `anchor_course` | nan | 앵커 시점 진행방향 (rad) |
| `anchor_time` | nan | 앵커 잠금 시각 (monotonic) |
| `yaw_at_anchor` | nan | 앵커 잠금 시점 IMU yaw (rad) |
| `gyro_integral` | 0.0 | 앵커 이후 누적 yaw 변화 (rad) |
| `last_step_time` | nan | 직전 DR 스텝 시각 |
| `method` | NONE | DRMethod enum |
| `confidence` | 0.0 | DR 신뢰도 [0, 1] (L1 cmd 스케일링용) |

---

## 2. 함수 목록

### motorapp.py

| 함수 | 입력 | 출력 | 역할 |
|------|------|------|------|
| `_cache_snapshot()` | — | `_Cache` | `_CACHE_t` 깊은 복사 (락 내부에서 호출) |
| `_compute_linear_acc(roll,pitch,ax,ay,az)` | deg, m/s² | (lax,lay,laz) | body-frame 가속도에서 중력 제거 |
| `handle_gps(data)` | CSV str | — | GPS 페이로드 파싱 → `_CACHE_t.latest_gps` 갱신 |
| `handle_imu(data)` | CSV str | — | IMU 페이로드 파싱 → `_CACHE_t.latest_imu` 갱신 |
| `handle_barometer(data)` | CSV str | — | 기압계 페이로드 파싱 → `_CACHE_t.latest_baro` 갱신 |
| `handle_target_coord(data)` | CSV str | — | 타겟 좌표 유효성 검사 → `guidance.set_target()` |
| `handle_flight_state(data)` | CSV str | — | STATE 갱신, state<3 시 guidance/PID 리셋 |
| `handle_release(data)` | str | — | 번와이어 스레드 실행 |
| `handle_egg_drop()` | — | — | 솔레노이드 스레드 실행 |
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
| `_compute_dr_confidence(age)` | float s | float [0,1] | DR 신뢰도 계산 (0~2s:1.0, 2~5s:0.5, 5~20s:0.0) |
| `UpdateAnchors(gps,imu,baro,now)` | raw 센서 | — | duck-type 센서 → 앵커 갱신 (매 사이클) |
| `DecideControlMode(now)` | float | `ControlMode` | 신선도 판단 → 6개 모드 중 결정 |
| `_update_state_from_dead_reckoning(now)` | float | — | DR 적분 수행, `nav.*` 갱신 |
| `ProduceL1Input(now)` | float | `L1Input` | origin 획득 + nav 갱신 + L1Input 생성 |
| `ProduceL1Output(l1in)` | `L1Input` | `L1Output` | L1 공식으로 yaw_rate_cmd 계산 (pure) |
| `set_target(lat,lon)` | deg | — | 타겟 설정, origin 있으면 즉시 투영 |
| `reset()` | — | — | origin/nav/DR 초기화 (target lat/lon 보존) |

---

## 3. 경우의 수별 수치 예시

**공통 설정값** (config.py):
```
L_GAIN_M = 17.0,  V_MIN=0.5, V_MAX=15.0, V_MAX_DR=3.0
NU_DEADBAND_DEG = 15.0
GPS_FRESH_MAX_AGE_S = 15.0,  IMU_FRESH_MAX_AGE_S = 3.0   ← 1.5→3.0
GPS_STALE_TIMEOUT_SEC(gpsapp) = 15.0                      ← 5.0→15.0
NMEA_CACHE_MAX_AGE_SEC: 제거됨 (gps.py 드라이버는 last-known 무조건 반환)
NEUTRAL_ARM_DEG = 80°,  PULSE_PER_DEG = 11.11 µs/°
LEFT_ZERO=2480µs, RIGHT_ZERO=636µs
DETUMBLE_GYRZ_THRESHOLD_DPS = 200.0,  DETUMBLE_EXIT_THRESHOLD_DPS = 30.0
```

**비행 상태 전제**: `STATE=3`, `MOTOR_ENABLED=True`, `origin=(37.4490, 127.1230)`, `target=(37.4600, 127.1234)` → `target_E=+33m, target_N=+1223m`

---

### CASE 1 — GPS_TRACKING_CLOSED

**조건**: GPS pos+vel 신선 ✅, IMU gyrz 신선 ✅

#### 초기 센서 데이터 (dispatch 수신)

```
handle_gps("37.4495,127.1232,1,1000.000,10.0,4.0,1,1000.000")
  lat=37.4495, lon=127.1232, pos_health=1, pos_ts=1000.000
  course_deg=10.0, speed_mps=4.0, motion_health=1, motion_ts=1000.000

handle_imu("0,0,45,0,0,9.81,0,0,5,1,1000.010")
  roll=0°, pitch=0°, yaw=45°, az=9.81m/s², gyrz=5°/s, health=1, sample_ts=1000.010
```

#### handle_gps() 내부 처리

```
course_rad = radians(10.0) = 0.1745 rad
_CACHE_t.latest_gps:
  lat=37.4495, lon=127.1232
  pos_ts=1000.000, pos_health=1
  course_rad=0.1745 rad, speed_mps=4.0
  motion_ts=1000.000, motion_health=1
```

#### handle_imu() 내부 처리

```
gyrz_deg_s = 5.0  →  gyrz_rad_s = radians(-5.0) = -0.0873 rad/s  ← 부호 반전(nav 규약)
lin_acc: _compute_linear_acc(0°, 0°, 0, 0, 9.81)
  g_x = -sin(0)*9.81 = 0,  g_y = cos(0)*sin(0)*9.81 = 0,  g_z = cos(0)*cos(0)*9.81 = 9.81
  lin_ax=0-0=0, lin_ay=0-0=0, lin_az=9.81-9.81=0

_CACHE_t.latest_imu:
  yaw_rad=radians(45)=0.7854 rad
  gyrz_rad_s=-0.0873 rad/s
  lin_acc_x=0, lin_acc_y=0, lin_acc_valid=True, health=1, ts=1000.010
```

#### _ctrl_cycle(now=1000.020)

```python
# 게이트: MOTOR_ENABLED=True, STATE=3 → 통과
guidance.UpdateAnchors(snap_t.latest_gps, snap_t.latest_imu, snap_t.latest_baro, 1000.020)
```

#### UpdateAnchors()

```
[GPS]
  pos_health=1, lat=37.4495, lon=127.1232, pos_ts=1000.000
  origin_ready=True → latlon_to_ne(37.4495, 127.1232, 37.4490, 127.1230)
    dLat = radians(0.0005) = 8.727e-6 rad
    dLon = radians(0.0002) = 3.491e-6 rad
    N = 8.727e-6 * 6371000 = 55.6 m
    E = 3.491e-6 * 6371000 * cos(radians(37.449)) ≈ 17.7 m
  _STATE_t.gps.E=17.7, _STATE_t.gps.N=55.6
  _STATE_t.gps.pos_ts=1000.000, pos_valid=True
  _MISSION_t._raw_lat=37.4495, _MISSION_t._raw_lon=127.1232

  motion_health=1 → _STATE_t.gps.V=4.0, _STATE_t.gps.course=0.1745 rad
  _STATE_t.gps.motion_ts=1000.000, motion_valid=True

[IMU]
  health=1, ts=1000.010
  _STATE_t.imu.ts=1000.010
  _STATE_t.imu.yaw=0.7854 rad, yaw_valid=True
  _STATE_t.imu.gyr_z=-0.0873 rad/s, gyrz_valid=True
  _STATE_t.imu.lin_acc_x=0, lin_acc_y=0, lin_acc_valid=True
```

#### DecideControlMode(now=1000.020)

```
pos_fresh:  pos_valid=True, 1000.020-1000.000=0.02s ≤ 15.0  → True
vel_fresh:  motion_valid=True, 0.02s ≤ 15.0                  → True
imu_fresh:  ts=1000.010, 1000.020-1000.010=0.01s ≤ 3.0       → True
gyrz_fresh: imu_fresh=True, gyrz_valid=True                   → True

DETUMBLING 체크:
  gyrz_dps = abs(degrees(-0.0873)) = 5.0 dps  < 200.0  → 진입 안 함

GPS_TRACKING_CLOSED 체크:
  pos_fresh=True, vel_fresh=True, gyrz_fresh=True  → ✅

→ st_t.nav.control_mode = ControlMode.GPS_TRACKING_CLOSED
```

#### ProduceL1Input(now=1000.020)

```
origin_ready=True, target_ready=True
mode = GPS_TRACKING_CLOSED

_STATE_t.nav.E       = gps.E       = 17.7 m
_STATE_t.nav.N       = gps.N       = 55.6 m
_STATE_t.nav.course  = gps.course  = 0.1745 rad (10°)
_STATE_t.nav.V       = gps.V       = 4.0 m/s
_STATE_t.nav.confidence = 1.0

dr_lock(dr, E=17.7, N=55.6, V=4.0, course=0.1745, yaw=0.7854, now=1000.020)
  anchor_E=17.7, anchor_N=55.6, anchor_V=4.0, anchor_course=0.1745
  yaw_at_anchor=0.7854, gyro_integral=0.0, anchor_time=1000.020

L1Input(
  valid=True, reason="GPS_TRACKING",
  control_mode=GPS_TRACKING_CLOSED, confidence=1.0,
  E=17.7, N=55.6, V=4.0, course=0.1745,
  target_E=33.0, target_N=1223.0
)
```

#### ProduceL1Output(l1in)

```
dE = 33.0 - 17.7 = 15.3 m
dN = 1223.0 - 55.6 = 1167.4 m
dist = hypot(15.3, 1167.4) = 1167.5 m  > TARGET_RADIUS_M(5.0) → 계속

target_bearing = atan2(15.3, 1167.4) = 0.01311 rad (0.75°)
nu = wrap_pi(0.01311 - 0.1745) = -0.1614 rad (-9.24°)

|nu| = 9.24° < NU_DEADBAND_DEG(15°)  → sin_nu_eff = 0.0  (데드밴드 내)

V_eff = clamp(4.0, 0.5, 15.0) = 4.0
yaw_rate_cmd = 2.0 * 4.0 / 17.0 * 0.0 = 0.0 rad/s
yaw_rate_cmd *= confidence(1.0) = 0.0
lim = radians(40.0) = 0.698 rad/s  (GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS)

L1Output(
  control_valid=True, nominal=True, reason="GPS_TRACKING",
  yaw_rate_cmd=0.0 rad/s, yaw_rate_limit_dps=40.0,
  nu=-0.1614 rad(-9.24°), target_bearing=0.01311 rad,
  distance_to_target=1167.5 m, pid_enabled=True
)
```

→ `ProduceCtrlInput`:  `angular_velocity_cmd_deg_s = degrees(0.0) = 0.0 dps`  
→ `ProduceCtrlOutput`:  `delta_ff=0, delta_pid=0, delta_arm=0°`  
→ `ConnectRoMo(0°)`: `left=80°, right=80°, left_pw=1591µs, right_pw=1525µs` ← **중립 서보**

---

### CASE 1b — GPS_TRACKING_CLOSED (nu > 데드밴드, 우선회)

```
현재 위치: E=17.7m, N=55.6m, course=90°(정동향)
목표:      E=33m, N=1223m

target_bearing = atan2(15.3, 1167.4) = 0.75°
nu = wrap_pi(0.75° - 90°) = -89.25° = -1.558 rad

|nu|=89.25° > 15° → 데드밴드 통과
clamp(-1.558, -π/2, π/2) = -1.558 (범위 내)
sin_nu_eff = sin(-1.558) ≈ -1.000

V_eff = clamp(4.0, 0.5, 15.0) = 4.0
yaw_rate_cmd = 2.0 * 4.0 / 17.0 * (-1.000) = -0.471 rad/s
             *= 1.0  →  -0.471 rad/s  (-27.0°/s)
clamp(-0.471, -0.698, +0.698) = -0.471 rad/s
```

`ProduceCtrlOutput`(KP=0, pid_active=False 경우):
```
delta_ff = angular_velocity_to_delta_ff(-27.0, cfg)
  x = 27.0 / 45.0 = 0.60
  delta ≈ -18.0° (예시)

ConnectRoMo(-18.0°):
  left_angle  = 80 - (-18)/2 = 89°
  right_angle = 80 + (-18)/2 = 71°
  left_pw  = 2480 - 89*11.11 = 1491µs
  right_pw = 636  + 71*11.11 = 1425µs
  → 왼쪽 arm 내려가고 오른쪽 arm 올라감 → 왼쪽 선회
```

---

### CASE 2 — GPS_TRACKING_OPEN

**조건**: GPS pos+vel 신선 ✅, IMU gyrz **없음** ❌

```
handle_gps("37.4495,127.1232,1,1000.000,10.0,4.0,1,1000.000")  ← 동일

handle_imu("0,0,45,0,0,9.81,0,0,0,0,1000.010")
  health=0  → ts/rx_ts만 갱신, gyrz_valid=False, yaw_valid=False
```

#### DecideControlMode

```
pos_fresh=True, vel_fresh=True
imu_fresh: health=0 → _STATE_t.imu.ts 미갱신(nan) → imu_fresh=False
gyrz_fresh=False, yaw_fresh=False

DETUMBLING: gyrz_fresh=False → 체크 불가(건너뜀)
GPS_TRACKING_CLOSED: gyrz_fresh=False → ❌
GPS_TRACKING_OPEN:   pos_fresh=True, vel_fresh=True → ✅

→ st_t.nav.control_mode = ControlMode.GPS_TRACKING_OPEN
```

#### ProduceL1Output

```
동일 L1Input(E=17.7, N=55.6, V=4.0, course=0.1745)
yaw_rate_limit_dps = 25.0  ← CLOSED(40.0)보다 낮음
radians(25) = 0.436 rad/s
clamp(-0.471, -0.436, +0.436) = -0.436 rad/s  (한계 적용)
pid_enabled=True (L1Output 플래그)

→ ProduceCtrlOutput: sensor_valid=False(gyrz 없음) → pid_active=False
  mode = FEEDFORWARD_ONLY  ← 자이로 없으므로 FF만
```

---

### CASE 3 — DR_TRACKING_CLOSED

**조건**: GPS dropout (15초 초과), DR 앵커 유효, gyrz 신선

```
# GPS 마지막 수신: pos_ts=985.0  (now=1003.0 → age=18s > 15s)
# DR 앵커: anchor_E=17.7, anchor_N=55.6, anchor_V=4.0, anchor_course=0.1745 rad
#          anchor_time=985.0, gyro_integral(누적)=0.3 rad (18s간 적분)
# IMU: gyrz_rad_s=-0.0873, ts=1002.990

handle_imu("0,0,62,0,0,9.81,0,0,5,1,1002.990")
  yaw=62°=1.082rad, gyrz=-0.0873rad/s, ts=1002.990
```

#### DecideControlMode(now=1003.000)

```
pos_fresh:  pos_ts=985.0, age=18.0s > 15.0  → False
vel_fresh:  motion_ts=985.0, age=18.0s > 15.0 → False
imu_fresh:  ts=1002.990, age=0.01s ≤ 3.0      → True
gyrz_fresh: True

DETUMBLING: 5dps < 200 → 건너뜀
GPS_TRACKING_CLOSED: pos_fresh=False → ❌
GPS_TRACKING_OPEN:   vel_fresh=False → ❌
DR_TRACKING_CLOSED:
  dr_is_valid=True, gyrz_fresh=True → ✅
  (confidence 체크 없음 — DR 모드 유지, confidence는 L1 cmd만 스케일링)

→ st_t.nav.control_mode = ControlMode.DR_TRACKING_CLOSED
```

#### ProduceL1Input → _update_state_from_dead_reckoning(now=1003.000)

```
dt = 1003.0 - 1002.990 = 0.01s  (last_step_time 기준)

① gyro 적분:
  dr.gyro_integral += (-0.0873) * GYRZ_SIGN(1.0) * 0.01
  = 기존 0.3 + (-0.000873) = 0.2991 rad

② heading 추정: dr_estimate_course(dr, imu)
  base = anchor_course = 0.1745 rad (10°)
  course_gyro = wrap_pi(0.1745 + 0.2991) = 0.4736 rad (27.1°)
  course_yaw  = wrap_pi(0.1745 + wrap_pi(1.082 - 0.7854))
              = wrap_pi(0.1745 + 0.2966) = 0.4711 rad (27.0°)
  |wrap_pi(0.4711 - 0.4736)| = 0.0025 < π/4 → circular_mean
  course_est ≈ 0.4724 rad (27.1°)

③ 속도 유지 (감쇄 없음 — anchor_V 그대로):
  V_dr = dr.anchor_V = 4.0 m/s   ← exp(-age) 감쇄 없음

④ EN 속도:
  vE = 4.0 * sin(0.4724) = 4.0 * 0.4549 = 1.820 m/s
  vN = 4.0 * cos(0.4724) = 4.0 * 0.8905 = 3.562 m/s

⑥ 위치 갱신:
  nav.E += 1.820 * 0.01 ≈ 이전E + 0.0182
  nav.N += 3.562 * 0.01 ≈ 이전N + 0.0356
  (18초간 누적 예: nav_E≈50.0m, nav_N≈119.0m)

nav.V = 4.0 m/s,  nav.course = 0.4724 rad

⑦ 신뢰도:
  age = 1003.0 - 985.0 = 18.0s
  _compute_dr_confidence(18.0):
    a1=2, a2=5, a3=20
    a2 < 18 ≤ a3 → conf = 0.5*(1 - (18-5)/(20-5)) = 0.5*(1-0.867) = 0.067
  dr.confidence = 0.067,  nav.confidence = 0.067
```

#### ProduceL1Output (DR_TRACKING_CLOSED)

```
V_eff = clamp(4.0, 0.5, V_MAX_DR(3.0)) = 3.0  ← V_MAX_DR 상한 적용

dE = 33.0 - 50.0 = -17.0 m  (18초 DR 후 E가 목표 지나침, 예시)
dN = 1223.0 - 119.0 = 1104.0 m
dist = hypot(-17.0, 1104.0) = 1104.1 m

target_bearing = atan2(-17.0, 1104.0) = -0.0154 rad (-0.88°)
nu = wrap_pi(-0.0154 - 0.4724) = -0.4878 rad (-27.9°)

|nu|=27.9° > 15° → sin_nu_eff = sin(-0.4878) = -0.468

yaw_rate_cmd = 2.0 * 3.0 / 17.0 * (-0.468) = -0.1650 rad/s
yaw_rate_cmd *= confidence(0.067) = -0.01106 rad/s  (-0.63°/s)

lim = radians(20.0) = 0.349 rad/s
clamp(-0.01106, -0.349, 0.349) = -0.01106 rad/s (-0.63°/s)

L1Output(
  yaw_rate_cmd=-0.01106 rad/s,  yaw_rate_limit_dps=20.0,
  nu=-0.4878 rad(-27.9°),  confidence=0.067,
  pid_enabled=True
)
→ 신뢰도 6.7%로 명령 대부분 억제. delta_ff ≈ 매우 작은 값 → 서보 거의 중립
```

---

### CASE 4 — DR_TRACKING_OPEN

**조건**: GPS dropout, DR 앵커 유효, gyrz **없음**, yaw 신선

```
handle_imu("0,0,62,0,0,9.81,nan,nan,nan,1,1002.990")
  gyrz_valid=False, yaw_rad=1.082, yaw_valid=True
```

#### DecideControlMode

```
gyrz_fresh=False, yaw_fresh=True
DR_TRACKING_CLOSED: gyrz_fresh=False → ❌
DR_TRACKING_OPEN:   dr_is_valid=True, yaw_fresh=True → ✅

→ st_t.nav.control_mode = ControlMode.DR_TRACKING_OPEN
```

#### dr_estimate_course (gyro 없음)

```
course_gyro = None  (gyrz_valid=False)
course_yaw  = wrap_pi(0.1745 + wrap_pi(1.082 - 0.7854)) = 0.4711 rad (27.0°)
→ yaw만 사용: course_est = 0.4711 rad

V_eff = clamp(4.0, 0.5, 3.0) = 3.0  (V_MAX_DR)
yaw_rate_limit = radians(15.0) = 0.2618 rad/s  ← DR_CLOSED(20dps)보다 낮음
```

---

### CASE 5 — DETUMBLING

**조건**: gyrz ≥ 200 dps

```
handle_imu("0,0,45,0,0,9.81,0,0,250,1,1000.010")
  gyrz_deg_s=250 → gyrz_rad_s=radians(-250)=-4.363 rad/s
```

#### DecideControlMode

```
gyrz_fresh=True
DETUMBLING 체크:
  gyrz_dps = abs(degrees(-4.363)) = 250.0 dps  ≥ 200.0  → ✅
  currently = (nav.control_mode != DETUMBLING → 첫 진입)
  → nav.control_mode = DETUMBLING, return DETUMBLING
```

#### _ctrl_cycle에서

```python
if mode == guidance.ControlMode.DETUMBLING:
    ctrl_out_t = control.ProduceDetumbleOutput(now)
    # gyrz_meas = degrees(-4.363) = -250 dps
    # delta_sum = -copysign(DETUMBLE_BRAKE_DELTA_DEG(80), -250) = +80°
    # → 반대 방향으로 제동 브레이크
    control.MoveServo(PI, ctrl_out_t)
    return ctrl_out_t
```

**DETUMBLING 탈출 조건**:
```
gyrz_dps ≤ 30.0 → detumble_exit_start 타이머 시작
→ 1.0초(DETUMBLE_EXIT_HOLD_S) 유지 → DETUMBLING 탈출, 아래 GPS/DR 체크로 진행
```

---

### CASE 6 — FAIL

**조건**: GPS dropout > 15s, DR 앵커 없음(또는 적분 불가), IMU stale

```
# GPS: pos_ts=900.0  (now=1003.0 → age=103s > 15s)
# DR:  anchor_time=nan  (또는 age=103s → confidence=0.0이지만 모드 결정에는 무관)
# IMU: ts=900.0  (age=103s > 3.0s) → imu_fresh=False
```

#### DecideControlMode

```
pos_fresh=False, vel_fresh=False
imu_fresh:  age=103s > 3.0  → False
gyrz_fresh=False, yaw_fresh=False

DETUMBLING: gyrz_fresh=False → 건너뜀
GPS_TRACKING_CLOSED: False
GPS_TRACKING_OPEN:   False
DR_TRACKING_CLOSED:  gyrz_fresh=False → ❌  (or dr_is_valid=False)
DR_TRACKING_OPEN:    yaw_fresh=False  → ❌

→ st_t.nav.control_mode = ControlMode.FAIL
```

#### _ctrl_cycle에서

```python
# DETUMBLING, GPS/DR 모두 해당 없음 → FAIL 분기
if PI is not None:
    control.WriteOff(PI)   # 서보 신호 OFF (0µs)
return None
```

---

## 4. GPS freshness 레이어 (3단계)

| 레이어 | 파일 | 임계값 | 역할 |
|--------|------|--------|------|
| ~~NMEA 캐시~~ | ~~gps.py~~ | ~~2.0s~~ | **제거됨** — 드라이버는 last-known 무조건 반환 |
| GPS 앱 차단 | gpsapp.py | **15.0s** | STALE 판정 시 motorapp에 GPS 전송 중단 |
| guidance freshness | config.py | **15.0s** | pos_fresh / vel_fresh 판단 → ControlMode 결정 |

두 레이어가 동일 임계값(15s)으로 정렬됨 → GPS 차단 타이밍과 guidance 신선도 판단 일치.

---

## 5. 전체 분기 요약표

| 모드 | GPS pos | GPS vel | gyrz | yaw | DR앵커 | 결과 명령 |
|------|---------|---------|------|-----|--------|-----------|
| **GPS_TRACKING_CLOSED** | ✅ fresh | ✅ fresh | ✅ fresh | — | 갱신됨 | L1 + PID 가능 |
| **GPS_TRACKING_OPEN** | ✅ fresh | ✅ fresh | ❌ | — | 갱신됨 | L1 + FF만 |
| **DR_TRACKING_CLOSED** | ❌ stale | ❌ stale | ✅ fresh | — | ✅ valid | DR L1 + PID, cmd×conf |
| **DR_TRACKING_OPEN** | ❌ stale | ❌ stale | ❌ | ✅ fresh | ✅ valid | DR L1 + FF만, cmd×conf |
| **DETUMBLING** | — | — | ≥200dps | — | — | 브레이크 고정 편향(±80°) |
| **FAIL** | ❌ | ❌ | ❌ | ❌ | ❌ or no-anchor | WriteOff (신호 없음) |

> **DR 모드 진입 조건 주의**: `confidence > 0` 체크 없음.  
> `dr_is_valid()` + gyrz/yaw 신선만으로 진입. confidence는 `ProduceL1Output`에서 yaw_rate_cmd에만 곱해짐.  
> → DR age가 20s를 넘어 confidence=0이 되어도 모드는 DR_TRACKING_*로 유지되고 cmd=0이 됨 (FAIL 진입은 IMU stale에 의해서만 발생).
