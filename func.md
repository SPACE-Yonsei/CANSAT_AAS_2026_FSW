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

---

## 10) 변경 원칙

문서 갱신 시 아래를 우선한다.

1. 핵심 운용 경로(Queue/Pipe, 상태머신, 센서 송신, 모터 제어, Comm I/F) 우선 보전
2. 드라이버 세부는 파일 헤더/함수명 기준으로만 기술
3. 문서에 없는 기능을 "있다"라고 쓰지 않고, 미구현은 현황으로 분리 표기

