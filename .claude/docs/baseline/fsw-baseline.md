# CANSAT AAS 2026 FSW 통합 기준서 (Claude + func 융합본)

생성일: 2026-04-30  
목적: 기존 FSW를 기반으로 새 FSW를 재구축할 때, 기능 누락 없이 핵심을 유지하고 구조적으로 개선하기 위한 단일 기준 문서

---

## 0) 최우선 원칙 (절대 누락 금지)

1. 메시지 중심 구조 유지: `Queue(상향) + Pipe(하향)` 버스 아키텍처는 반드시 유지.
2. 앱 간 계약 유지: `AppID`, `MID`, payload 포맷 계약을 먼저 고정하고 구현.
3. 상태기계 보존: FlightLogic 상태 전이 규칙(0~5)과 전이 조건, 강제 상태 명령(SS) 유지.
4. 안전 우선: FDIR, failsafe(모터 neutral/off), 재시작/종료 로직 보존.
5. 영속성 유지: `prevstate` 기반 상태/보정값/타깃/카운터 복원 기능 유지.
6. 검증 가능성 강화: 센서 미연결 상태에서도 앱 관계/라우팅 검증 가능한 harness 우선 구성.

---

## 1) 현재 FSW 핵심 구조 요약

### 1.1 코어 실행 모델
- `main.py`가 각 앱 프로세스 생성/시작/감시/재시작 담당.
- 상향 메시지: 각 앱 -> `Main_Queue`.
- 하향 메시지: 메인 라우터 -> 앱별 `Pipe`.
- 메시지 포맷: `sender|receiver|MsgID|data`.

### 1.2 핵심 라이브러리 역할
- `lib/appargs.py`: AppID/MID 정의(앱 간 계약의 단일 소스).
- `lib/msgstructure.py`: `fill_msg`, `pack_msg`, `unpack_msg`, `send_msg`.
- `lib/events.py`: 멀티프로세스 안전 로깅(`QueueListener/QueueHandler`).
- `lib/config.py`: GPIO, 센서 주기, 릴레이 레벨, 동작 파라미터.
- `lib/prevstate.py`: 상태/보정값/타깃좌표/패킷카운트/시간 오프셋 영속화.

### 1.3 주요 앱 구성
- 통신: `comm/commapp.py`, `comm/uartserial.py`, `comm/xbeereset.py`
- 비행로직: `flight_logic/flightlogicapp.py`
- 센서: Barometer/IMU/GPS/Distance/Electro
- 액추에이터: Motor(Release/Egg/Guidance/Control)
- 카메라: `Sensor_Camera/cameraapp.py`, `picam.py`

---

## 2) 기능 보존 체크리스트 (재구축 시 기준)

아래 항목은 새 FSW에서 "동일 기능 또는 개선 기능"으로 반드시 구현해야 한다.

### 2.1 메인 오케스트레이터 (`main.py`)
- `run_app(AppID)`: 앱 프로세스 시작
- `runloop(Main_Queue)`: 메시지 수신 후 receiver Pipe로 라우팅
- `checkrunstatus()`: 죽은 프로세스 감지
- `restart_app(appID)`: 프로세스 재생성/재시작
- `process_monitor()`: 주기 감시(약 3초)
- `terminate_FSW()`: 전체 앱 종료 시퀀스 + 강제 종료 + 로그 shutdown + 상태 초기화

### 2.2 메시지/계약 계층
- `AppID`, `MID` 체계 유지 (호환 또는 마이그레이션 레이어 제공)
- `send_msg` 단일 인터페이스 유지
- `data` 필드의 파싱 실패, 포맷 오류 방어 로직 유지

### 2.3 통신 앱 (`commapp`)
- 명령 파싱 정규식 분기:
  - 로컬: `CX`, `ST`, `RBT`
  - 라우팅: `SIM`, `SIMP`, `CAL`, `MEC`, `SS`, `CAM`, `TC`
- 텔레메트리 1Hz 송신
- `tlm_data` 집계 업데이트(MID 기반)
- `cmd_echo`, mission time 보정(`set_timedelta`) 기능

### 2.4 FlightLogic 상태기계
- 상태 정의 유지:
  - 0 LAUNCH_PAD
  - 1 ASCENT
  - 2 APOGEE
  - 3 RELEASE
  - 4 EGG
  - 5 LANDED
- 전이 조건 유지(고도/카운터 기반)
- 강제 상태 전이(`SS`) 지원
- SIM 모드(`ENABLE/ACTIVATE/DISABLE`) 및 SIMP 처리 유지
- 타깃좌표 수신/검증/전달(`TC`)
- 상태 1Hz 브로드캐스트(`MID_comm_state`)

### 2.5 MotorApp + Guidance + Control
- 입력 핸들러 유지: GPS/IMU/BARO/TARGET/STATE/MEC/RELEASE/EGG
- 제어 루프(약 10Hz) 유지
- FDIR 체크:
  - 센서 NaN/Inf/None
  - IMU health
  - GPS stale/유효성/jump
  - 과도 회전
  - 고도/타깃 이상
- 비정상 시 failsafe(`set_neutral`)
- 착륙 상태 시 모터 off(`set_motors_off`)
- Guidance 핵심:
  - L1 carrot 추종
  - figure-8 패턴
  - 고도별 L-distance/phase 전환
  - yaw-rate PI + anti-windup + slew-rate

### 2.6 센서 앱 공통
- 각 앱 독립 프로세스 + 읽기/송신 스레드 구조 유지
- I2C lock(`fcntl`, `/tmp/i2c-1.lock`) 기반 경합 완화 유지 또는 개선
- 오류 복구:
  - IMU 연속 실패 시 재초기화
  - 각 센서 read 실패 시 재시도/대기
- 송신 주기 유지:
  - Barometer: read 10Hz, Comm 1Hz
  - IMU: read 100Hz, send 10Hz/Comm 1Hz
  - GPS: read 10Hz, Motor event성/Comm 1Hz
  - Distance: read 10Hz, send 5Hz
  - Electro: read/send 1Hz

### 2.7 Camera 앱
- CAM ON/OFF 명령 수신
- FlightLogic activate 신호 수신(`MID_cam_activate`)
- 녹화 스레드 반복 동작(세그먼트 기록)

### 2.8 영속성/운용
- `prevstate` 읽기/쓰기/복원
- `STATE_OVERRIDE` 지원
- `YAW_OFFSET` 반영
- startup/systemd 기반 자동 실행 유지

---

## 3) 누락 위험 포인트 (특히 주의)

1. **재시작 인자 시그니처**  
   앱별 launcher 인자 형태가 다를 수 있으므로, restart 템플릿을 App별로 분리.

2. **명령 보안 (`RBT`)**  
   인증 없는 재부팅은 위험. 토큰/시퀀스/상태 게이트 도입 필수.

3. **Telemetry 스키마 일관성**  
   필드 순서/콤마 처리/파서 호환성 자동 테스트 필요.

4. **Distance TLM 누락 방지**  
   현재 요구사항상 distance는 반드시 TLM에 포함되도록 명세 반영.

5. **GPS 정밀도**  
   lat/lon 출력 자리수(최소 5~6자리) 확보.

6. **센서 없는 상태 검증성**  
   Mock sensor/harness로 앱 간 메시지 계약 검증 가능해야 함.

---

## 4) 새 FSW 권장 아키텍처 (현행 기반 개선안)

### 4.1 유지할 것
- 멀티프로세스 분리(앱 격리)
- 메시지 버스 중심 라우팅
- 상태기계 중심 제어
- FDIR + failsafe 전략

### 4.2 개선할 것
- 공통 인터페이스:
  - App lifecycle 표준화 (`init/start/handle_msg/shutdown/health`)
  - launcher 시그니처 통일
- 메시지 계약:
  - MID 및 payload schema를 코드+문서 동시 생성
  - 유효성 검사 공통 유틸
- 테스트:
  - 메시지 계약 테스트
  - 상태 전이 단위테스트
  - 센서 mock 통합테스트
- 운영:
  - 헬스 리포트(heartbeat, queue backlog, loop latency)
  - watchdog + graceful degradation

---

## 5) 단계별 입력 프롬프트 (중요도 순, 순차 입력용)

아래 프롬프트를 1번부터 순서대로 입력하면, 한번에 몰아치지 않고 핵심부터 안전하게 새 FSW를 구축할 수 있다.

### Prompt 1 - 시스템 계약 고정 (최우선)
```text
우리는 CANSAT FSW를 새로 재구축한다.
기존 구조의 핵심(멀티프로세스, Queue+Pipe 라우팅, AppID/MID 계약, 상태기계)은 유지하고 코드 품질을 개선한다.
먼저 전체 앱 목록, AppID, MID, 메시지 payload 스키마를 표로 정의하고 "절대 호환 규칙"을 명시해줘.
누락 검증 체크리스트도 함께 만들어줘.
```

### Prompt 2 - 코어 런타임 뼈대 생성
```text
계약 문서를 기준으로 main orchestrator 골격을 작성해줘.
필수 함수: run_app, runloop, process_monitor, checkrunstatus, restart_app, terminate_FSW.
특히 restart_app은 앱별 launcher 시그니처 차이를 안전하게 처리하도록 설계해줘.
```

### Prompt 3 - 메시지 라이브러리 구현
```text
msgstructure 계층을 구현해줘.
fill_msg/pack_msg/unpack_msg/send_msg를 만들고, sender|receiver|MsgID|data 포맷을 엄격 검증해줘.
파싱 실패/필드 오류/금지문자('|') 처리 정책을 테스트 코드와 함께 작성해줘.
```

### Prompt 4 - 상태/설정/영속성 계층
```text
config + prevstate 모듈을 구현해줘.
STATE_OVERRIDE, YAW_OFFSET, ALT_CAL, MAX_ALT, TARGET_COORD, PACKET_COUNT, ST_TIMEDELTA를 저장/복원해야 한다.
재부팅 후 복원 시나리오 테스트도 같이 작성해줘.
```

### Prompt 5 - CommApp(명령 파서 + TLM 집계) 구현
```text
commapp를 구현해줘.
명령어 CX/ST/SIM/SIMP/CAL/MEC/SS/RBT/CAM/TC를 지원하고,
로컬 처리와 라우팅 처리를 분리해줘.
TLM 1Hz 송신을 구현하고 distance 필드를 반드시 포함해줘.
보안상 RBT는 인증 게이트를 넣어줘.
```

### Prompt 6 - FlightLogic 상태기계 구현
```text
flightlogicapp를 구현해줘.
상태 0~5(LAUNCH_PAD/ASCENT/APOGEE/RELEASE/EGG/LANDED)와 기존 전이 조건을 유지해줘.
SIM, SIMP, SS, TC 처리와 상태 1Hz 브로드캐스트를 포함해줘.
상태 전이 단위테스트를 반드시 작성해줘.
```

### Prompt 7 - MotorApp + FDIR + Guidance 구현
```text
motorapp/motor_guidance/motor_control을 구현해줘.
입력 핸들러(GPS/IMU/BARO/TARGET/STATE/MEC/RELEASE/EGG), 10Hz 제어루프, FDIR, failsafe를 포함해줘.
guidance는 L1 carrot + figure-8 + yaw rate PI(anti-windup/slew-rate) 구조를 유지해줘.
```

### Prompt 8 - 센서 앱들 구현
```text
barometer/imu/gps/distance/electro 앱을 공통 템플릿 기반으로 구현해줘.
각 앱은 read thread + send thread + command handler 구조를 따르고,
I2C lock, 재시도, health/stale 플래그를 포함해줘.
기존 송신 주기(10Hz/5Hz/1Hz)를 유지해줘.
```

### Prompt 9 - Camera 앱 구현
```text
cameraapp/picam 모듈을 구현해줘.
CAM ON/OFF와 MID_cam_activate를 모두 지원하고,
세그먼트 녹화 반복 스레드 구조를 유지해줘.
카메라 미연결 시 graceful degrade 하도록 처리해줘.
```

### Prompt 10 - 센서 없는 통합 검증 하네스
```text
실센서 없이 앱 간 연결을 검증할 수 있는 harness를 만들어줘.
mock sensor publisher, mock ground command sender, message recorder를 포함하고
명령->라우팅->상태전이->TLM 경로를 자동 검증하는 통합 테스트를 작성해줘.
```

### Prompt 11 - 운영/배포 안정화
```text
startup/systemd 스크립트를 정리해줘.
환경 경로 불일치 문제를 없애고, 로그 경로/권한/재시작 정책을 통일해줘.
운영 점검 체크리스트(부팅, 복구, 종료, 재시작)를 작성해줘.
```

### Prompt 12 - 최종 누락 검수
```text
아래 기능 보존 체크리스트 기준으로 구현 결과를 감사(audit)해줘.
누락/부분구현/위험요소를 항목별로 표시하고, 수정 패치를 제시해줘.
특히 상태기계, FDIR, 메시지 계약, prevstate 복원, distance TLM 포함 여부를 우선 검증해줘.
```

---

## 6) 최종 인수 조건 (Done Definition)

아래를 모두 만족하면 "기능 누락 없는 개선 FSW"로 판정한다.

1. 명령-라우팅-실행-텔레메트리 전체 체인이 동작한다.
2. 상태기계(0~5) 전이와 SS 강제 전이가 재현된다.
3. Motor 제어 루프가 FDIR 조건에서 neutral/off failsafe로 전환된다.
4. prevstate 기반 복원(상태/보정값/타깃/카운터/시간오프셋)이 동작한다.
5. 실센서 미연결 환경에서 harness 테스트가 통과한다.
6. telemetry 스키마가 문서와 1:1 일치하며 distance가 포함된다.
7. 프로세스 비정상 종료 시 재시작/종료 시퀀스가 안정 동작한다.

---

## 7) 참고: 유지 대상 함수군 인덱스

아래 함수군은 재구축 시 이름이 바뀌어도 기능은 반드시 유지해야 한다.

- Main: `run_app`, `runloop`, `process_monitor`, `checkrunstatus`, `restart_app`, `terminate_FSW`
- Msg: `fill_msg`, `pack_msg`, `unpack_msg`, `send_msg`
- Events: `init_events_main_process`, `init_events_subprocess`, `shutdown_events`, `LogEvent`
- Prevstate: `init_prevstate`, `update_prevstate`, `update_altcal`, `update_maxalt`, `update_target_gps`, `update_packet_count`, `update_st_timedelta`, `reset_prevstate`
- Comm: `read_cmd`, `send_tlm`, `command_handler`, `cmd_*`
- FlightLogic: `dispatch`, `barometer_logic`, `solenoid_logic`, `to_*`, `handle_*`
- Motor: `dispatch`, `handle_*`, `_check_fdir`, `ctrl_paragldr`
- Guidance: `is_gps_valid`, `is_gps_jump`, `guidance`, `_outer_loop`, `_yaw_rate_pi_control`
- Control: `init_control`, `control`, `set_neutral`, `set_motors_off`
- Sensor apps: 각 `*_init`, `read_*`, `send_*`, `command_handler`, `*_main`, `*_terminate`

---

## 8) 실행 진행 현황 (claude 브랜치)

- [x] Prompt 1: 시스템 계약 고정 (`../contracts/ipc-contract-freeze.md`)
- [x] Prompt 2: main 재시작 시그니처 안정화 (`main.py`)
- [x] Prompt 3: 메시지 계층 구현 + 테스트 (`lib/msgstructure.py`, `tests/test_msgstructure.py`)
- [x] Prompt 4: appargs/events/config/prevstate 구현 + 테스트
- [x] Prompt 5: CommApp 명령/라우팅/TLM(distance 포함) 구현 + 테스트
- [x] Prompt 6: FlightLogic 상태기계 구현 + 테스트
- [x] Prompt 7: MotorApp/Guidance/Control/FDIR 베이스 구현 + 테스트
- [x] Prompt 8: 센서 앱(BARO/IMU/GPS/DIST/ELECTRO) 베이스 구현
- [x] Prompt 9: Camera 앱/드라이버 베이스 구현
- [x] Prompt 10: 센서 없는 하네스 흐름 테스트 구현
- [ ] Prompt 11: 운영/배포 스크립트 정리 (다음 단계)
- [ ] Prompt 12: 최종 누락 감사 및 보완 (다음 단계)

현재 테스트 상태:
- `python -m unittest ...` 24 tests passed
- 핵심 모듈 `py_compile` 통과

---

이 문서를 기준으로 새 FSW를 단계적으로 구축하면, 핵심 기능 누락 없이 구조 개선과 검증성 향상을 동시에 달성할 수 있다.
