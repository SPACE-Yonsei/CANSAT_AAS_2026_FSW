# `motorapp.ctrl_paragldr()` 정보 이동 정리

이 문서는 `Sensor_Motor/motorapp.py`의 `ctrl_paragldr()`가 실행될 때, 센서/명령 데이터가 어떤 함수와 변수를 거쳐 최종 모터 pulse로 변환되는지 순서대로 추적하기 위한 문서이다.

대상 파일:

- `Sensor_Motor/motorapp.py`
- `Sensor_Motor/motor_guidance.py`
- `Sensor_Motor/motor_control.py`

---

## 1. 전체 흐름 요약

```text
main_pipe 수신 메시지
  -> motorapp.dispatch()
  -> handle_gps() / handle_imu() / handle_barometer()
     / handle_target_coord() / handle_flight_state()
  -> motorapp 전역 상태 갱신
  -> ctrl_paragldr() 제어 루프
  -> _snapshot_sensors()
  -> _check_fdir()
  -> motor_guidance.guidance()
  -> motor_control.control()
  -> motor_control.actuator_mixer()
  -> pi.set_servo_pulsewidth()
```

핵심은 `ctrl_paragldr()`가 직접 센서 메시지를 읽는 것이 아니라, 각 `handle_*()` 함수들이 갱신해 둔 전역 상태를 `_snapshot_sensors()`로 복사해서 사용한다는 점이다.

---

## 2. `ctrl_paragldr()` 이전: 입력 메시지가 전역 변수로 저장되는 과정

### 2.1 메시지 수신과 분배

`motorapp_main(main_pipe)`는 main process에서 들어오는 packed message를 계속 받는다.

```text
main_pipe.recv()
  -> msgstructure.unpack_msg()
  -> dispatch(unpacked_msg)
```

`dispatch()`는 `MsgID`를 보고 `MSG_HANDLERS`에서 대응되는 handler를 찾는다.

주요 handler:

| MsgID 출처 | handler | 저장/동작 |
|---|---|---|
| GPS app | `handle_gps()` | `GpsVector`, `GpsFidelity` 갱신 |
| IMU app | `handle_imu()` | `altitude.yaw`, `altitude.gyrz`, `altitude.healthy` 갱신 |
| Barometer app | `handle_barometer()` | `baro_m` 갱신 |
| Flight logic app | `handle_target_coord()` | `target.lat`, `target.lon` 갱신 |
| Flight logic app | `handle_flight_state()` | `state` 갱신, state 3 진입 시 start point 설정 |
| Comm app | `handle_mec()` | `motor_enabled` 갱신 |

---

## 3. 전역 상태 변수의 의미

`ctrl_paragldr()`가 직접 참조하는 상태는 아래 전역 변수에 축적된다.

### 3.1 비행/제어 상태

| 변수 | 타입/단위 | 의미 |
|---|---:|---|
| `running` | bool | 제어 thread 루프 유지 여부 |
| `state` | int | flight state. `state >= 3`부터 parafoil 제어 후보 |
| `motor_enabled` | bool | MEC 명령에 따른 모터 enable 상태 |
| `pi` | pigpio object | 실제 GPIO servo pulse 출력 객체 |
| `logger` | `MotorLogger` | guidance/control 로그 기록 |
| `_start_point_locked` | bool | state 3 기준 start point가 확정되었는지 |

### 3.2 목표 지점

```python
target = types.SimpleNamespace(
    lat=None,
    lon=None,
)
```

- `handle_target_coord(data)`에서 `lat,lon` 문자열을 받아 갱신한다.
- 동시에 `motor_guidance.set_target_coord(new_lat, new_lon)`도 호출한다.
- `_check_fdir()`에서 target이 없으면 `"No target coordinates received"`로 failsafe 처리된다.

### 3.3 IMU 상태

```python
altitude = types.SimpleNamespace(
    yaw=None,      # deg, 0-360
    gyrz=None,     # rad/s
    healthy=False,
)
```

- `handle_imu(data)`가 `yaw,gyrz,health` 3개 필드를 파싱한다.
- `yaw`는 현재 heading이다.
- `gyrz`는 yaw rate 측정값이며 `motor_guidance.guidance()` 안에서 `deg/s`로 변환된다.
- `healthy=False`이면 FDIR에서 즉시 reject된다.

### 3.4 Barometer 상태

```python
baro_m = None
```

- `handle_barometer(data)`가 첫 번째 필드를 float으로 읽어 저장한다.
- `motor_guidance.guidance(..., baro_m=snap.baro_m)`로 전달된다.
- guidance 내부에서 altitude에 따라 L1/carrot 거리와 landing 제한이 달라진다.

### 3.5 GPS 위치/속도 상태

```python
GpsVector = types.SimpleNamespace(
    lat=None,
    lon=None,
    speed=None,
    course=None,
)
```

| 필드 | 단위 | 사용처 |
|---|---:|---|
| `lat` | deg | 현재 위치, start/target 대비 EN 변환 |
| `lon` | deg | 현재 위치, start/target 대비 EN 변환 |
| `speed` | m/s | outer-loop yaw-rate 계산의 `V` |
| `course` | deg | wind learning에서 crab angle 계산 |

### 3.6 GPS 품질 상태

```python
GpsFidelity = types.SimpleNamespace(
    rmc_status=None,
    fix_quality=None,
    sats=None,
    jump_rejected=True,
)
```

| 필드 | 의미 |
|---|---|
| `rmc_status` | `"A"`이면 active, `"V"`이면 void |
| `fix_quality` | `0` no fix, `1` GPS, `2` DGPS 등 |
| `sats` | 위성 수 |
| `jump_rejected` | `motor_guidance.is_gps_jump()`가 이번 GPS를 reject했는지 |

---

## 4. GPS handler에서 start point가 정해지는 과정

`handle_gps(data)`는 다음 순서로 동작한다.

```text
GPS data string
  -> lat, lon, speed, course, fix, sats, rmc 파싱
  -> NaN/Inf 검사
  -> motor_guidance.is_gps_jump(new_lat, new_lon)
  -> update_lock 안에서 GpsVector/GpsFidelity 갱신
  -> state == 3 이고 start_point 미확정이고 GPS valid이면
       motor_guidance.set_start_coordinates()
```

즉, state 3 진입 이후 첫 valid GPS가 start point가 된다.  
단, `handle_flight_state()`에서도 state 3으로 바뀌는 순간 이미 valid GPS가 있으면 즉시 start point를 설정한다.

---

## 5. `ctrl_paragldr()` 루프 구조

`ctrl_paragldr()`는 `init()`에서 thread로 시작된다.

```python
threads["ControlLog_Thread"] = threading.Thread(
    target=ctrl_paragldr,
    name="ControlLog_Thread",
    daemon=True
)
```

루프 주기는 `CONTROL_LOG_INTERVAL = 0.1`초다.

전체 구조:

```text
while running:
  snap = _snapshot_sensors()

  if snap.state >= 3 and snap.motor_enabled:
    if snap.state == 5:
      motor_control.set_motors_off(pi)
      sleep(0.1)
      continue

    reason = _check_fdir(snap)

    if reason is not None:
      log failsafe
      motor_control.set_neutral(pi)
    else:
      logger.guidance_in(...)
      result = motor_guidance.guidance(...)
      motor_result = motor_control.control(pi, result.commanded_yaw_rate)
      logger.motor_out(motor_result)
      logger.control(result, motor_result)

  heartbeat log every 10 ticks
  sleep(0.1)
```

---

## 6. `_snapshot_sensors()`가 하는 일

`_snapshot_sensors()`는 여러 handler가 갱신 중인 전역 값을 `update_lock` 안에서 한 번에 복사한다.

결과는 `snap`이라는 `SimpleNamespace`다.

```text
snap.state
snap.motor_enabled
snap.baro_m

snap.gps.lat
snap.gps.lon
snap.gps.speed
snap.gps.course

snap.gps_fidelity.rmc_status
snap.gps_fidelity.fix_quality
snap.gps_fidelity.sats
snap.gps_fidelity.jump_rejected

snap.imu.yaw
snap.imu.gyrz
snap.imu.healthy

snap.target.lat
snap.target.lon
```

이 snapshot은 이후 FDIR, guidance, logging에 같은 기준 상태로 전달된다.

---

## 7. `_check_fdir(snap)` 검사 순서

`ctrl_paragldr()`는 guidance를 호출하기 전에 `_check_fdir(snap)`로 제어 가능 여부를 판단한다.

반환값:

- `None`: 정상, guidance/control 진행
- 문자열: failsafe 사유, neutral 출력

검사 순서:

| 단계 | 조건 | 실패 시 결과 |
|---|---|---|
| FDIR-0 | GPS/IMU/BARO 값이 `None`, NaN, Inf인지 | `"No data received: ..."` |
| FDIR-1 | `snap.imu.healthy`가 True인지 | `"IMU stale (sensor-reported)"` |
| FDIR-2 | `motor_guidance.is_gps_valid()` 통과 여부 | `"GPS invalid ..."` |
| FDIR-2b | `snap.gps_fidelity.jump_rejected`가 False인지 | `"GPS jump rejected ..."` |
| FDIR-3 | `abs(gyrz) <= 100 deg/s`인지 | `"|gyrz|=... > threshold"` |
| FDIR-4 | `baro_m > 0`인지 | `"Baro altitude invalid ..."` |
| FDIR-5 | target 좌표가 존재하는지 | `"No target coordinates received"` |

FDIR 실패 시 정보 이동:

```text
reason != None
  -> failsafe log 출력
  -> motor_control.set_neutral(pi)
  -> LEFT_NEUTRAL / RIGHT_NEUTRAL pulse 출력
  -> guidance/control은 호출되지 않음
```

---

## 8. FDIR 통과 후 `motor_guidance.guidance()` 입력

FDIR을 통과하면 다음 값들이 guidance로 들어간다.

```python
result = motor_guidance.guidance(
    snap.imu,
    snap.gps,
    snap.gps_fidelity,
    snap.target,
    baro_m=snap.baro_m,
)
```

입력 변수 매핑:

| guidance 인자 | 실제 전달값 | 주요 필드 |
|---|---|---|
| `imu_data` | `snap.imu` | `yaw`, `gyrz`, `healthy` |
| `gps_vector` | `snap.gps` | `lat`, `lon`, `speed`, `course` |
| `gps_fidelity` | `snap.gps_fidelity` | 현재 코드에서는 guidance 내부에서 직접 사용 거의 없음 |
| `target` | `snap.target` | `lat`, `lon` |
| `baro_m` | `snap.baro_m` | altitude-dependent guidance |

---

## 9. `motor_guidance.guidance()` 내부 정보 이동

### 9.1 시간 간격 `dt` 계산

```text
now = time.time()
dt = now - last_time
dt가 너무 작거나 크면 0.1로 보정
last_time = now
```

`dt`는 inner-loop PI 제어와 slew limiter에 사용된다.

### 9.2 위경도에서 EN 좌표로 변환

```python
my_E, my_N = _llh_to_en(gps_vector.lat, gps_vector.lon)
tgt_E, tgt_N = _llh_to_en(target.lat, target.lon)
```

기준은 `start_point.lat/lon`이다.

```text
N = (lat - start_lat) * LAT_TO_METER
E = (lon - start_lon) * LAT_TO_METER * cos(start_lat)
```

결과:

- `my_E`, `my_N`: start point 기준 현재 위치
- `tgt_E`, `tgt_N`: start point 기준 목표 위치

### 9.3 start point 확인

```text
start_point가 없으면
  -> state="START_UNSET"
  -> commanded_yaw_rate=0
```

이 경우 motor output은 0 yaw-rate 기준으로 계산된다.

### 9.4 목표 거리 계산

```python
distance = hypot(tgt_E - my_E, tgt_N - my_N)
```

`distance < TARGET_REACHED_RADIUS`이면:

```text
state="TARGET_REACHED"
commanded_yaw_rate=0
```

현재 상수:

```text
TARGET_REACHED_RADIUS = 10 m
```

### 9.5 고도에 따른 L1/carrot 거리 선택

```text
baro_m > 300     -> L_DISTANCE = 40 m
baro_m < 150     -> L_DISTANCE = 15 m
else             -> L_DISTANCE = 25 m
```

관련 상수:

| 상수 | 값 | 의미 |
|---|---:|---|
| `ALT_HIGH` | `300` | 고고도 기준 |
| `ALT_LOW` | `150` | 저고도 기준 |
| `L_DISTANCE_HIGH` | `40 m` | 고고도 carrot 거리 |
| `L_DISTANCE_BASE` | `25 m` | 기본 carrot 거리 |
| `L_DISTANCE_LOW` | `15 m` | 저고도 carrot 거리 |

### 9.6 유도점 선택: carrot 또는 figure-8

```text
if 10 m < baro_m < 50 m and distance < 20 m:
  guide_E, guide_N = _eight(...)
  phase = "PATTERN"
else:
  guide_E, guide_N = _carrot(...)
  phase = "HOMING"
```

대부분의 접근 구간은 `_carrot()`을 사용한다.

`_carrot()`의 개념:

```text
start_point -> target 직선 경로 위에 현재 위치를 투영
투영점보다 L_DISTANCE 앞쪽의 점을 carrot point로 설정
```

### 9.7 현재 위치에서 유도점을 바라보는 heading 계산

```python
carrot_angl_north = degrees(atan2(guide_E - my_E, guide_N - my_N))
```

여기서 `atan2(E, N)`을 쓰므로 North 기준 heading이다.

### 9.8 바람 보정 후 heading error 계산

```python
wind_carrot_angl_north = _wrap_180(carrot_angl_north - wind_effect)
angl_to_turn = _wrap_180(wind_carrot_angl_north - imu_data.yaw)
```

의미:

```text
angl_to_turn
  = desired_heading - current_yaw
```

`verify_motor_fsw.py`의 trace CSV에서는 이 값이 `heading_error_deg`로 저장된다.

### 9.9 속도 `V` 설정

```python
V = max(gps_vector.speed, 1.0)
```

GPS speed가 너무 작아도 최소 1 m/s를 사용한다.

### 9.10 Capture mode: 큰 heading error에서 integral reset

```python
if abs(angl_to_turn) > CAPTURE_THRESHOLD:
    cascade_pi.pi_integral = 0.0
```

현재 상수:

```text
CAPTURE_THRESHOLD = 45 deg
```

큰 각도 회전 중에는 PI 적분항이 쌓이지 않도록 reset한다.

### 9.11 Outer-loop: heading error에서 desired yaw-rate 생성

```python
desired_yaw_rate = _outer_loop(angl_to_turn, V, L_DISTANCE)
```

내부 수식:

```text
K = 2 * V / (L * YR_MAX)
desired_yaw_rate = YR_MAX * tanh(K * angl_to_turn)
```

관련 상수:

```text
YR_MAX = 45 deg/s
```

출력 의미:

```text
heading error를 줄이기 위해 필요한 목표 yaw-rate
```

### 9.12 Inner-loop PI: desired yaw-rate에서 commanded yaw-rate 생성

```python
commanded_yaw_rate = _yaw_rate_pi_control(
    desired_yaw_rate,
    degrees(imu_data.gyrz),
    dt,
)
```

입력:

| 변수 | 의미 |
|---|---|
| `desired_yaw_rate` | outer-loop가 요구한 yaw-rate, deg/s |
| `degrees(imu_data.gyrz)` | 실제 측정 yaw-rate, deg/s |
| `dt` | 제어 주기 |

내부 계산:

```text
rate_error = desired_yaw_rate - measured_yaw_rate
u = Kp_inner * rate_error + Ki_inner * pi_integral
u_sat = clamp(u, -MAX_CMD, +MAX_CMD)
u_sat에 slew-rate 제한 적용
```

관련 상수:

| 변수 | 값 | 의미 |
|---|---:|---|
| `Kp_inner` | `1.3` | yaw-rate error 비례 게인 |
| `Ki_inner` | `0.05` | 적분 게인 |
| `MAX_INTEGRAL` | `10.0` | 적분항 제한 |
| `MAX_CMD` | `60 deg/s` | actuator yaw-rate 명령 제한 |
| `MAX_ACCEL` | `150 deg/s^2` | 명령 변화율 제한 |

### 9.13 저고도 landing clamp

```python
if baro_m <= LANDING_ALT:
    commanded_yaw_rate = clamp(commanded_yaw_rate, -20, +20)
```

관련 상수:

```text
LANDING_ALT = 20 m
LANDING_YR_MAX = 20 deg/s
```

### 9.14 phase 결정

```text
phase == "HOMING"이고 abs(angl_to_turn) <= 15 deg -> "STRAIGHT"
phase == "HOMING"이고 abs(angl_to_turn) > 15 deg  -> "TURNING"
```

`guidance()` 반환값:

```python
types.SimpleNamespace(
    state=phase,
    distance=distance,
    commanded_yaw_rate=commanded_yaw_rate,
)
```

---

## 10. `motor_control.control()` 내부 정보 이동

`ctrl_paragldr()`는 guidance 결과 중 `commanded_yaw_rate`만 motor control에 넘긴다.

```python
motor_result = motor_control.control(pi, result.commanded_yaw_rate)
```

### 10.1 `control()`의 역할

```text
commanded_yaw_rate
  -> actuator_mixer()
  -> left/right pulse 계산
  -> pi.set_servo_pulsewidth()
  -> motor_result 반환
```

### 10.2 `actuator_mixer(commanded_yaw_rate)`

핵심 수식:

```python
pulse_offset = commanded_yaw_rate / 2.0 * K_pulse

left_raw_pw  = LEFT_NEUTRAL  + pulse_offset
right_raw_pw = RIGHT_NEUTRAL + pulse_offset

left_pulse  = clamp(left_raw_pw, PULSE_MIN, LEFT_MAX_PULSE)
right_pulse = clamp(right_raw_pw, RIGHT_MIN_PULSE, PULSE_MAX)
```

상수:

| 변수 | 값 | 의미 |
|---|---:|---|
| `PULSE_PER_DEG` | `2000 / 180 = 11.111... us/deg` | servo 변환 계수 |
| `K_pulse` | `PULSE_PER_DEG` | yaw-rate 명령을 pulse offset으로 변환 |
| `LEFT_ZERO` | `600 us` | left servo 0 deg 기준 |
| `RIGHT_ZERO` | `2500 us` | right servo 0 deg 기준 |
| `NEUTRAL_DEG` | `60 deg` | neutral arm angle |
| `LEFT_NEUTRAL` | `1266 us` | left neutral pulse |
| `RIGHT_NEUTRAL` | `1833 us` | right neutral pulse |
| `MAX_ANGLE_SCOPE` | `120 deg` | servo 각도 제한 |
| `PULSE_MIN` | `500 us` | pulse lower bound |
| `PULSE_MAX` | `2500 us` | pulse upper bound |

### 10.3 pulse에서 arm angle 산출

```python
left_cmd_deg  = (left_pulse  - LEFT_ZERO)  / PULSE_PER_DEG
right_cmd_deg = (RIGHT_ZERO  - right_pulse) / PULSE_PER_DEG
```

주의할 점:

- left는 pulse가 증가하면 angle이 증가한다.
- right는 `RIGHT_ZERO - right_pulse`이므로 pulse가 증가하면 angle이 감소한다.
- 같은 `pulse_offset`을 양쪽에 더하지만, left/right angle은 반대 방향 효과를 갖는다.

### 10.4 motor result 반환값

```python
types.SimpleNamespace(
    left_cmd_deg=left_cmd_deg,
    right_cmd_deg=right_cmd_deg,
    actual_delta_deg=actual_delta_deg,
    expected_yaw_rate=expected_yaw_rate,
    left_pulse=left_pulse,
    right_pulse=right_pulse,
)
```

| 필드 | 의미 |
|---|---|
| `left_cmd_deg` | left motor arm command angle |
| `right_cmd_deg` | right motor arm command angle |
| `actual_delta_deg` | `right_cmd_deg - left_cmd_deg` |
| `expected_yaw_rate` | pulse offset으로부터 역산한 yaw-rate |
| `left_pulse` | left servo PWM pulse |
| `right_pulse` | right servo PWM pulse |

---

## 11. 정상 제어 시 정보 이동 한 줄 요약

```text
GpsVector.lat/lon + target.lat/lon + start_point
  -> _llh_to_en()
  -> my_E,my_N,tgt_E,tgt_N
  -> _carrot() or _eight()
  -> guide_E,guide_N
  -> carrot heading
  -> wind 보정
  -> angl_to_turn
  -> _outer_loop()
  -> desired_yaw_rate
  -> _yaw_rate_pi_control()
  -> commanded_yaw_rate
  -> motor_control.control()
  -> actuator_mixer()
  -> left_pulse/right_pulse
  -> pi.set_servo_pulsewidth()
```

---

## 12. Failsafe 시 정보 이동 한 줄 요약

```text
snap
  -> _check_fdir()
  -> reason 문자열 생성
  -> log("Failsafe: ...")
  -> motor_control.set_neutral(pi)
  -> LEFT_NEUTRAL / RIGHT_NEUTRAL 출력
```

FDIR 실패 시 `motor_guidance.guidance()`와 `motor_control.control()`은 호출되지 않는다.

---

## 13. `ctrl_paragldr()`를 디버깅할 때 봐야 할 핵심 변수

### 13.1 입력 상태

```text
snap.state
snap.motor_enabled
snap.baro_m
snap.gps.lat
snap.gps.lon
snap.gps.speed
snap.gps.course
snap.gps_fidelity.fix_quality
snap.gps_fidelity.sats
snap.gps_fidelity.rmc_status
snap.gps_fidelity.jump_rejected
snap.imu.yaw
snap.imu.gyrz
snap.imu.healthy
snap.target.lat
snap.target.lon
```

### 13.2 FDIR 결과

```text
reason = _check_fdir(snap)
```

- `reason is None`: 제어 가능
- `reason != None`: failsafe, neutral

### 13.3 Guidance 내부 주요 값

현재 `guidance()` 반환값에는 일부만 담기므로, 상세 분석 시에는 debug log 또는 별도 trace가 필요하다.

핵심 내부 값:

```text
my_E, my_N
tgt_E, tgt_N
distance
L_DISTANCE
guide_E, guide_N
carrot_angl_north
wind_effect
wind_carrot_angl_north
angl_to_turn
V
desired_yaw_rate
cascade_pi.pi_integral
commanded_yaw_rate
phase
```

### 13.4 Motor control 주요 값

```text
commanded_yaw_rate
pulse_offset
left_raw_pw
right_raw_pw
left_pulse
right_pulse
left_cmd_deg
right_cmd_deg
actual_delta_deg
expected_yaw_rate
```

---

## 14. 로그에서 확인 가능한 출력

`DEBUG_GUIDANCE = True`이면 정상 제어 시 다음 로그가 남는다.

### 14.1 guidance input

`logger.guidance_in(...)`:

```text
state, baro
yaw, gyrz
gps lat/lon/speed/course
fix/sats/rmc
target lat/lon
```

### 14.2 guidance debug

`motor_guidance.guidance()` 내부 debug:

```text
phase
dist
L
pos=(my_E,my_N)
tgt=(tgt_E,tgt_N)
carrot=(guide_E,guide_N)
des_crs
wind
des_hdg
hdg_err
V
des_yr
pi_int
cmd_yr
[CAP], [SAT], [LND]
```

### 14.3 motor output

`logger.motor_out(motor_result)`:

```text
left angle / pulse
right angle / pulse
actual_delta_deg
expected_yaw_rate
```

---

## 15. 주의할 점

1. `target`은 `motorapp.target`과 `motor_guidance.target`에 모두 저장된다.
   - FDIR은 `snap.target`을 본다.
   - guidance는 인자로 받은 `snap.target`을 사용한다.
   - `motor_guidance.set_target_coord()`는 별도 전역 target도 갱신하지만, 현재 `guidance()`에서는 인자 `target`이 직접 쓰인다.

2. `start_point`는 `motor_guidance` 모듈 내부 전역 상태다.
   - `motorapp`이 state 3 진입 또는 첫 valid GPS에서 `set_start_coordinates()`를 호출해 설정한다.

3. `gyrz` 단위는 `motorapp`에서는 rad/s, `guidance` 내부 PI에서는 deg/s다.
   - `_check_fdir()`도 rad/s 기준으로 `math.radians(100.0)`와 비교한다.

4. `heading error`의 실제 변수명은 `angl_to_turn`이다.
   - trace CSV에서는 이해를 위해 `heading_error_deg`라고 썼다.

5. `commanded_yaw_rate`가 바로 pulse가 되는 것은 아니다.
   - `commanded_yaw_rate / 2 * K_pulse`가 pulse offset이 된다.

6. state 5에서는 guidance/control을 거치지 않고 `set_motors_off()`로 servo 신호를 0으로 끊는다.

---

## 16. 한눈에 보는 정상 제어 데이터 체인

```text
GPS(lat/lon/speed/course)
IMU(yaw/gyrz/healthy)
BARO(baro_m)
TARGET(lat/lon)
STATE(state)
MEC(motor_enabled)
        |
        v
_snapshot_sensors()
        |
        v
_check_fdir()
        |
        | pass
        v
motor_guidance.guidance()
        |
        | commanded_yaw_rate
        v
motor_control.control()
        |
        | left_pulse, right_pulse
        v
pigpio servo output
```

