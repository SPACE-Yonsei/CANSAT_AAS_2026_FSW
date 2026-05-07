# CANSAT AAS 2026 FSW — Claude 작업 지침

## 프로젝트 목적
1 kg급 CanSat 패러포일 자율 귀환 비행 소프트웨어.
사출 직후 GPS 유도를 통해 직선 경로를 추종하며 목표 지점에 착륙한다.

---

## 현재 작업 단계 (Phase 1 진행 중)

**5단계 ArduPilot L1 통합 계획:**

| Phase | 내용 | 상태 |
|-------|------|------|
| 1 | `motor_guidance.py` outer loop → ArduPilot L1 `2V²/L·sin(η)` 교체 + `motorapp.py` GPS velocity 활용 강화 | **진행 중** |
| 2 | `motor_control.py` K_pulse 재보정, 비대칭 mixer 검토 | 대기 |
| 3 | 센서 미수신 시 fallback 처리 (GPS dead-reckoning, IMU/baro hold) | 대기 |
| 4 | CANSAT_SIM_LOG playback + Monte Carlo wind 시뮬레이션 | 대기 |
| 5 | 낙하 테스트 후 Kp/Ki/L_DISTANCE 튜닝, 동역학 모델링 | 대기 |

**Phase 1 핵심 변경:**
- `_outer_loop()` 제거 → eta 기반 `accel_lat = 2V²/L·sin(η)` → `desired_yaw_rate = accel_lat/V`
- `bearing_to_carrot`(haversine) - `gps_track_deg`(GPS direction) = eta
- wind_effect는 eta 계산 전 gps_track에서 차감 (desired_heading 보정 방식 폐기)
- V < 0.5 m/s 구간에서 `desired_yaw_rate = 0`

---

## 아키텍처 요약

```
main.py (Process 오케스트레이터, Queue 라우팅)
  ├── Sensor_Motor/motorapp.py       ← 10Hz 제어 루프
  │     ├── motor_guidance.py        ← L1 경로 추종 + cascaded PI
  │     └── motor_control.py         ← servo PWM mixer (pigpio)
  ├── Sensor_Gps/gpsapp.py           ← lat,lon,direction,velocity,pos_health,motion_health
  ├── Sensor_IMU/imuapp.py           ← yaw_deg, gyrz_deg_s, imu_health
  ├── Sensor_Barometer/barometerapp.py ← altitude_m
  ├── flight_logic/flightlogicapp.py ← state machine (0-5), target 좌표 송신
  └── comm/commapp.py                ← Xbee 텔레메트리, MEC 명령
```

**Flight State:**
- 0 LAUNCH_PAD / 1 ASCENT / 2 APOGEE → 모터 neutral 대기
- 3 RELEASE → start point lock, 유도 시작
- 4 EGG → 유도 계속
- 5 LANDED → 모터 off

---

## 핵심 파일

| 파일 | 역할 |
|------|------|
| `Sensor_Motor/motor_guidance.py` | L1 경로 추종, cascaded PI, wind learning, GPS/baro 검증 |
| `Sensor_Motor/motorapp.py` | 메시지 핸들러, 10Hz 루프, start point lock |
| `Sensor_Motor/motor_control.py` | servo PWM, differential mixer, pigpio |
| `lib/appargs.py` | MID/AppID 전체 정의 — **수정 시 /cmd-add-mid 사용** |
| `lib/msgstructure.py` | IPC 메시지 pack/unpack |
| `lib/config.py` | GPIO 핀, 센서 rate, relay 파라미터 |
| `tests/test_motor_guidance.py` | guidance 단위 테스트 (12개 케이스) |

---

## 좌표 / 부호 규약 (불변)

- 지역 좌표계: N=North, E=East. bearing: 0°=North, +90°=East
- `gps_vector.direction`: GPS ground-track (속도 방향), deg
- `imu_data.yaw`: IMU 자력계 yaw, deg
- `imu_data.gyrz`: yaw rate, **deg/s** (rad/s 아님)
- `commanded_yaw_rate > 0` = **좌회전**
- `LEFT_SIGN = +1`, `RIGHT_SIGN = -1` (differential brake)
- `motor_guidance.guidance()` 는 actuator를 모른다 — `commanded_yaw_rate` 만 반환

---

## IPC 계약 (동결)

```
GPS → Motor:   lat,lon,direction,velocity,pos_health,motion_health   (6 fields)
IMU → Motor:   yaw_deg,gyrz_deg_s,imu_health                        (3 fields)
Baro → Motor:  altitude_m                                             (1 field)
FL → Motor:    target_lat,target_lon                                  (2 fields)
Motor → Comm:  left_pulse,right_pulse,start_lat,start_lon,target_lat,target_lon,
               carrot_lat,carrot_lon,current_heading,desired_heading,state
```

IPC 필드 추가/변경은 반드시 `lib/appargs.py` + 양쪽 앱 핸들러 동시 수정.
새 MID 등록 시 `/cmd-add-mid` 슬래시 커맨드 사용.

---

## 개발 규칙

1. **10Hz 루프에 blocking 금지** — GPIO, file I/O, sleep, heavy math 없음
2. **Lock 순서** — 센서 읽기는 반드시 `_UPDATE_LOCK` 내부에서
3. **FDIR 유지** — 예외 발생 시 `set_neutral()` 호출 후 루프 계속
4. **단위 명시** — 라디안과 degree 혼용 금지. 변수명에 `_deg`, `_rad`, `_m`, `_ms` suffix 권장
5. **테스트 먼저** — guidance 변경 후 반드시 `pytest tests/test_motor_guidance.py -v` 통과
6. **pigpio 미사용 환경** — `motor_control.init_control()`은 테스트에서 mock 처리됨

---

## 테스트 실행

```bash
# motor 관련 전체
.venv/Scripts/python.exe -m pytest tests/test_motor_guidance.py tests/test_motorapp.py tests/test_motor_control.py -v

# 전체 스위트
.venv/Scripts/python.exe -m pytest tests/ -v

# 특정 케이스
.venv/Scripts/python.exe -m pytest tests/test_motor_guidance.py::test_gps_jump_rejected -v
```

디버그 출력:
```bash
set CANSAT_DEBUG_GUIDANCE=1   # guidance tick 상세 출력
set CANSAT_DEBUG_CONTROL=1    # servo command 로그
```

---

## 슬래시 커맨드

| 커맨드 | 용도 |
|--------|------|
| `/cmd-add-mid` | appargs.py에 새 MID 등록 |
| `/cmd-check-flight-state` | 상태 전이 검증 |
| `/cmd-check-tlm-schema` | 텔레메트리 필드 스키마 확인 |
| `/cmd-new-app` | 새 센서 앱 boilerplate 생성 |
| `/cmd-review-ipc` | IPC 정적 감사 |

## 에이전트

| 에이전트 파일 | 용도 |
|--------------|------|
| `agent-ardupilot-integration` | ArduPilot L1/parafoil 알고리즘 포팅 |
| `agent-control-reviewer` | L1/PI/mixer 코드 리뷰 (OK/WARN/FAIL) |
| `agent-ipc-auditor` | MID 불일치, 라우팅 오류 감지 |
| `agent-codebase-explorer` | 심볼 위치, 의존성 탐색 (read-only) |

---

## ArduPilot 참조 (Phase 1)

로컬 클론: `C:\workspace\ardupilot\`

| 파일 | 참조 항목 |
|------|----------|
| `libraries/AP_L1_Control/AP_L1_Control.cpp` | `update_waypoint()`, Nu=Nu1+Nu2, sine_Nu1 clamp ±0.7071, `_prevent_indecision()` |
| `ArduPlane/commands_logic.cpp` | `verify_nav_wp()`, acceptance radius, finish-line |
| `ArduPlane/commands.cpp` | `set_next_WP()`, behind-A / past-B 처리 |

**Phase 1 핵심 공식:**
```python
# ArduPilot AP_L1_Control::update_waypoint() 동치
bearing_to_carrot = haversine_bearing(my_lat, my_lon, carrot_lat, carrot_lon)
eta = _wrap_180(bearing_to_carrot - gps_track_deg)      # velocity vector 기준 각도
eta = max(-90.0, min(90.0, eta))                         # ±90° 클램프
accel_lat = 2.0 * V**2 / L_DISTANCE * math.sin(math.radians(eta))
desired_yaw_rate = math.degrees(accel_lat / max(V, 0.5))
desired_yaw_rate = max(-YR_MAX, min(YR_MAX, desired_yaw_rate))
```


# CANSAT AAS 2026 FSW 기능 정리 (코드 기준)

생성: 2026-05-05  
기준: 현재 저장소 코드 (`main.py`, `lib/*`, `comm/*`, `Sensor_*/*app.py`)

---

## 0) 핵심 동작 요약 (미션 필수)

아래 6가지는 캔위성 운용 핵심이다.

1. `main.py`가 멀티프로세스 앱을 실행/감시/재시작한다.
2. 모든 앱 간 메시지는 `main_queue`를 거쳐 Main이 Pipe로 라우팅한다.
3. `flight_logic/flightlogicapp.py`가 상태 전이(0~5)를 결정한다.
4. 센서 앱이 주기적으로 메시지를 송신한다(FlightLogic/Motor/Comm).
5. `Sensor_Motor/motorapp.py`가 센서 수신 + FDIR + guidance/control을 수행한다.
6. `comm/commapp.py`가 지상국 명령 수신 및 텔레메트리 송신을 담당한다.

---

## 1) 메시지 계약

### `lib/appargs.py`
- AppID:
  - Main=10, FlightLogic=11, Comm=12, Barometer=13, IMU=14, GPS=15, Distance=16, Electro=17, Camera=18, Motor=19
- MID:
  - `MainAppArg.MID_TerminateProcess = 1001001`
  - 센서/명령 라우팅 MID는 각 `*AppArg` 클래스에 정의

### `lib/msgstructure.py`
- 메시지 버스 포맷: `sender|receiver|msg_id|data`
- `fill_msg()`: 타입/구분자 검증 후 `MsgStructure` 생성
- `pack_msg()`: 구조체를 버스 문자열로 직렬화
- `unpack_msg()`: 버스 문자열을 구조체로 역직렬화
- `send_msg()`: `fill -> pack -> queue.put` 원스텝 전송

---

## 2) Main 오케스트레이터 (`main.py`)

- 각 앱 Process 및 Pipe 생성 후 `app_dict`로 관리
- `runloop(main_queue)`:
  - Queue에서 메시지 수신
  - `unpack_msg()`로 `receiver_app` 확인
  - 해당 앱 Pipe로 전달
- `process_monitor()`:
  - 3초 주기로 프로세스 생존 확인
  - 죽은 앱은 `restart_app()`로 재기동 시도
- `terminate_FSW()`:
  - 전체 앱에 `MID_TerminateProcess` 전송
  - join timeout 후 필요 시 terminate/kill
  - `prevstate.reset_prevstate()`, `events.shutdown_events()` 수행

---

## 3) 설정/영속성/로깅

### `lib/config.py`
- GPIO 매핑, 센서 주기(Hz), 릴레이 active/deactive 레벨 정의
- 런타임 오버라이드:
  - `STATE_OVERRIDE` (env)
  - `YAW_OFFSET` (env)

### `lib/prevstate.py`
- 파일: `lib/prevstate.json`
- 영속 필드:
  - `PREV_STATE`, `PREV_ALT_CAL`, `PREV_MAX_ALT`
  - `Target_lat`, `Target_lon`
  - `PREV_PACKET_COUNT`, `PREV_ST_TIMEDELTA`
- 함수:
  - `init_prevstate()`, `reset_prevstate()`
  - `update_prevstate()`, `update_altcal()`, `update_maxalt()`
  - `update_target_gps()`, `update_packet_count()`, `update_st_timedelta()`

### `lib/events.py`
- 메인: `init_events_main_process()`가 `QueueListener` 시작
- 서브프로세스: `init_events_subprocess(log_queue)`가 `QueueHandler` 부착
- 통합 로그 API:
  - `LogEvent(app_name, event_type, event_msg)`
  - `event_type`: `error`, `warning`, `info`, `debug`

---

## 4) Comm 앱 (`comm/commapp.py`, `comm/uartserial.py`)

### 주요 역할
- UART로 명령 수신
- 명령 검증/분기 후 타 앱으로 라우팅 메시지 송신
- 1Hz 텔레메트리 CSV 송신

### 주요 함수
- `command_handler(recv_msg)`: 센서/상태 메시지로 `tlm_data` 갱신
  - `MID_comm_sim`: `data`를 `,`로 split한 **첫 필드**를 텔레메트리 `mode`에 반영. 계약상 `mode_char in {F, S, A}` (`fsw_step1_contract.md` 참고).
- `send_tlm(serial_instance)`: 텔레메트리 직렬화 및 전송
- `read_cmd(main_queue, serial_instance)`: 지상국 명령 파싱
- 명령 핸들러:
  - `cmd_cx`, `cmd_st`, `cmd_sim`, `cmd_simp`, `cmd_cal`
  - `cmd_mec`, `cmd_ss`, `cmd_rbt`, `cmd_cam`, `cmd_tc`

### `comm/uartserial.py`
- 실장치 Serial + 테스트용 `DummySerial` 지원
- `init_serial()`, `send_serial_data()`, `receive_serial_data()`, `terminate_serial()`

---

## 5) Flight Logic (`flight_logic/flightlogicapp.py`)

### 상태 머신
- 상태: 0(LAUNCH_PAD), 1(ASCENT), 2(APOGEE), 3(RELEASE), 4(EGG), 5(LANDED)

### 입력 처리
- `handle_barometer()`: 실비행 모드에서 고도 입력 처리
- `handle_simp()`: 시뮬레이션 활성 시 고도 입력 처리
- `handle_distance()`: EGG 상태 거리 기반 솔레노이드 트리거
- `handle_ss()`: 상태 강제 전이
- `handle_target_coord()`: 목표 좌표 검증/저장 후 Motor 전달

### 핵심 로직
- `barometer_logic()`:
  - 최근 고도 윈도우로 `max_alt` 추적
  - 상태별 카운터 기반 전이 판단
- `solenoid_logic()`:
  - 거리 기준으로 `MID_motor_EggDrop` 송신
- `send_current_state_thread()`:
  - 1Hz로 `MID_comm_state` 송신

### SIM 모드
- `handle_sim()`:
  - `ENABLE` -> `sim_enable=True`, `sim_active=False` 후 `_verify_inter_app_links()` 호출. Comm에는 `MID_comm_sim` payload `A,LINKCHK`(첫 필드 `A` = SIM 준비 모드) 등 비액츄에이터 점검용 프레임을 송신.
  - `ACTIVATE` -> `sim_active=True`일 때 Comm에 `MID_comm_sim` `"S"` (SIMP로 `barometer_logic` 주입)
  - `DISABLE` -> 플래그 해제 후 Comm에 `MID_comm_sim` `"F"`
- `handle_simp()`: **`sim_active`가 아니면** 고도 주입 없음 (ENABLE만 한 상태 A에서는 SIMP 무시)

---

## 6) Motor 앱 (`Sensor_Motor/*`)

### `motorapp.py`
- Pipe 수신 `dispatch()`로 MID별 핸들러 호출
- 수신 데이터:
  - GPS (`MID_motor_gps`)
  - IMU (`MID_motor_imu`)
  - Barometer (`MID_motor_alt`)
  - Target (`MID_motor_TargetCor`)
  - Flight state/명령 (`MID_motor_state`, `MID_motor_burnwire`, `MID_motor_EggDrop`, `MID_RouteCmd_MEC`)
- `ctrl_paragldr()` 제어 루프:
  - 상태/모터 enable 확인
  - `_check_fdir()` 이상 시 neutral
  - 정상 시 guidance -> control

### `motor_guidance.py`
- L1 계열 유도 + yaw-rate PI 제어
- GPS 무결성/점프 검증
- 고도에 따른 L-distance 및 yaw-rate 제한

### `motor_control.py`
- servo mixer + PWM 출력
- `init_control()`, `control()`, `set_neutral()`, `set_motors_off()`

### `Motor_Release.py`, `Motor_Egg.py`
- 번와이어/솔레노이드 제어 GPIO 동작

---

## 7) 센서 앱 (송신 경로 중심)

### Barometer (`Sensor_Barometer/barometerapp.py`)
- 읽기 스레드 + 송신 스레드 분리
- 송신:
  - -> FlightLogic: `MID_flight_alt`
  - -> Motor: `MID_motor_alt`
  - -> Comm: `MID_comm_alt` (주기 축약 송신)
- 하드웨어 실패 시 synthetic 경로로 폴백 가능

### IMU (`Sensor_Imu/imuapp.py`)
- 읽기 고주기, 송신 저주기 분리
- stale timeout 기반 `HEALTH` 관리
- 송신:
  - -> Motor: `MID_motor_imu` (`yaw,gyrz,HEALTH`)
  - -> Comm: `MID_comm_euler` (full telemetry)

### GPS (`Sensor_Gps/gpsapp.py`)
- stale timeout + `GPS_HEALTH` 관리
- 모터용 속도는 `GPS_MAX_SPEED_FOR_MOTOR = 15.0`으로 게이트
- 송신:
  - -> Motor: `MID_motor_gps`
  - -> Comm: `MID_comm_gga` (주기 축약 송신)

### Distance (`Sensor_Distance/distanceapp.py`)
- 송신:
  - -> FlightLogic: `MID_flight_dis`
  - -> Comm: `MID_comm_dis`

### Electro (`Sensor_Electro/electroapp.py`)
- 송신:
  - -> Comm: `MID_comm_volt`

### Camera (`Sensor_Camera/cameraapp.py`)
- 명령 수신 기반 녹화 ON/OFF
- `MID_cam_activate`, `MID_RouteCmd_CAM` 처리

---

## 8) 런타임/배포

### `startup.sh`
- 필요 시 `pigpiod` 기동
- `.venv` 활성화
- `python3 main.py` 실행

### `cansat-fsw.service`
- `ExecStart=/opt/cansat-fsw/startup.sh`
- `Restart=on-failure`

---

## 9) 현황 메모 (혼동 방지)

아래 항목은 현재 저장소에서 미구현/부재/단순화 상태다.

- `comm/xbeereset.py`: `send_reset_pulse()`는 현재 스텁
- `Sensor_Imu/Calibrator.py`: 저장소에 없음
- `Sensor_Motor/motor_logger.py`: 저장소에 없음
- `Sensor_Motor/set_motor_angle.py`: 저장소에 없음
- `reset_control()` 함수: 현재 코드베이스에 없음



