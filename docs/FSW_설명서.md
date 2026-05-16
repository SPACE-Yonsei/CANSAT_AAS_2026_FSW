# CANSAT FSW 설명서

이 문서는 `CANSAT_AAS_2026_FSW-4`의 비행소프트웨어(FSW) 구조와 운용 방법을 빠르게 이해하기 위한 기술 설명서입니다.

## 1. 시스템 개요

- 언어/런타임: Python + `multiprocessing` 기반 멀티프로세스 구조
- 중심 라우터: `main.py`
- 통신 방식:
  - 프로세스 -> Main: 공용 `Queue`
  - Main -> 프로세스: 앱별 `Pipe`
- IPC 메시지 포맷: `sender|receiver|MsgID|data`
- 핵심 구성 앱:
  - `Comm`(지상국 명령/텔레메트리)
  - `FlightLogic`(상태머신/미션 이벤트)
  - `Motor`(유도/제어/액추에이터)
  - `Barometer`, `IMU`, `GPS`, `Distance`, `Electro`, `Camera`

## 2. 프로세스 아키텍처

### 2.1 부팅 순서

1. `main.py`에서 이벤트 로깅/센서 로깅 초기화
2. `prevstate` 로드(재부팅 복구용)
3. 앱별 프로세스 생성 및 시작
4. 백그라운드 모니터 스레드가 앱 생존 감시(기본 3초 주기)
5. Main runloop가 Queue 메시지를 수신해 대상 앱 Pipe로 라우팅

### 2.2 장애 복구

- 앱 프로세스가 비정상 종료되면 Main이 자동 재시작을 시도
- 재시작 시 새 Pipe/Process를 재생성해 `app_dict`를 갱신
- 종료(`Ctrl+C`/Terminate MID) 시 모든 앱에 종료 메시지 전파 후 join/강제종료 수행

## 3. 앱 역할 요약

### Main (`main.py`)

- 앱 프로세스 생명주기 관리
- IPC 라우팅 허브
- 프로세스 죽음 감시 및 재시작

### Comm (`comm/commapp.py`)

- `CMD,1070,...` uplink 명령 파싱/검증/라우팅
- 1 Hz 텔레메트리 송신
- 각 센서/모터/비행상태 데이터를 통합해 표준 TLM CSV 라인 생성

### FlightLogic (`flight_logic/flightlogicapp.py`)

- 비행 상태머신(0~5) 운영
- 상태 전이에 따라 Motor/Camera 등으로 이벤트 MID 전송
- SIM 모드 ENABLE/ACTIVATE/DISABLE 처리

### Motor (`Sensor_Motor/motorapp.py`)

- GPS/IMU/Baro 입력 캐시 관리
- L1 기반 경로유도 + yaw-rate PI 제어
- 서보 출력/번와이어/에그드롭 액추에이터 실행
- 상태/진단 정보를 Comm으로 전송

### Sensor Apps

- Barometer/IMU/GPS/Distance/Electro/Camera 각각 독립 프로세스
- 주기 샘플링 후 필요한 앱(FlightLogic, Motor, Comm)으로 MID 전송

## 4. IPC 계약(AppID/MID)

정의 파일: `lib/appargs.py`

- AppID
  - `10`: Main
  - `11`: FlightLogic
  - `12`: Comm
  - `13`: Barometer
  - `14`: IMU
  - `15`: GPS
  - `16`: Distance
  - `17`: Electro
  - `18`: Camera
  - `19`: Motor

주요 MID 예시:

- Main -> 전체 종료: `MID_TerminateProcess`
- FlightLogic -> Motor: 상태/타겟/번와이어/에그드롭
- Comm -> FlightLogic: `SIM/SIMP/SIMG/SS/TC`
- Comm -> Motor: `MEC/FAC/MTR`
- Comm -> Camera: `CAM`

## 5. 지상국 명령 체계(Comm)

입력 포맷:

- `CMD,1070,<CMD>,<OPTION>`

주요 명령:

- `CX,ON|OFF`: 텔레메트리 송신 on/off
- `ST,HH:MM:SS` 또는 `ST,GPS`: 시간 기준 설정
- `SIM,ENABLE|ACTIVATE|DISABLE`: 시뮬레이션 모드 제어
- `SIMP,<alt_m>`: 시뮬레이션 고도 주입
- `SIMG,<lat>,<lon>,<course_deg>,<speed_mps>[,<alt_m>]`: 시뮬 GPS 주입
- `CAL,<...>`: 고도 보정 관련 명령을 Barometer로 전달
- `MEC,ON|OFF`: 모터 제어 활성화 게이트
- `FAC,ON|OFF` 또는 `FAC,REL|EGG,ON|OFF`: 액추에이터 강제동작 게이트
- `MTR,LEFT|NEUTRAL|RIGHT`(또는 `L/N/R`): 수동 조향 모드
- `SS,0~5`: 상태 강제 전이
- `TC,<lat>,<lon>`: 목표 좌표 설정
- `CAM,ON|OFF`: 카메라 녹화 제어
- `RBT,<auth>`: 인증 기반 재부팅
- `XRST,NOW|1|ON`: XBee reset pulse

## 6. 비행 상태머신

상태 정의:

- `0` Launch Pad
- `1` Ascent
- `2` Apogee
- `3` Release
- `4` Egg
- `5` Landed

핵심 전이 개요:

- `0 -> 1`: 고도 상승 조건 연속 만족
- `1/2 -> 3`: release predictor 조건 만족 시 번와이어 트리거
- `3 -> 4`: 저고도 조건 충족
- `4 -> 5`: 착륙 판정 조건 장시간 충족
- `SS` 명령으로 각 상태 강제 전이 가능

보조 동작:

- `1` 진입 시 Camera 활성화 메시지 전송
- `3` 진입 시 Motor에 타겟 좌표/번와이어 이벤트 전송
- `4`에서 거리센서 또는 저고도 fallback으로 에그드롭 트리거

## 7. Motor 유도/제어 파이프라인

1. 센서 MID 수신 -> 내부 캐시 갱신
2. `guidance.ProduceL1Input/Output`으로 유도명령 계산
3. `control.ProduceCtrlOutput`으로 yaw-rate PI 제어
4. `ProducePulse`로 좌우 서보 펄스 출력
5. 진단 payload를 Comm에 송신(MID `MID_comm_motor_diag`)

안전/운용 포인트:

- `MOTOR_ENABLED=False` 또는 상태 `<3`이면 중립 출력
- `MTR` 수동모드가 활성화되면 자동유도 대신 수동 조향 우선
- `FAC`로 release/egg 액션 게이트를 분리 관리 가능

## 8. 텔레메트리 포맷

Comm은 1Hz로 `$1070,...` CSV 라인을 송신합니다.

- 메타: team id, 시각, 패킷카운트, mode, state
- 센서: 고도/온도/기압, 전압/전류/전력, IMU, GPS, 거리
- 제어: 필터자세, 시작/목표/carrot 좌표, 헤딩, 서보 PWM, guidance 상태
- 마지막 명령 에코: `cmd_echo`

참고:

- 동일한 start/target 좌표는 연속 프레임에서 빈 필드로 압축 전송됩니다.
- UART 송신 실패 시 콘솔/이벤트 로그 경고가 남습니다.

## 9. 상태 영속화(`prevstate`)

파일: `lib/prevstate.json`

저장되는 핵심 값:

- 최근 flight state / max altitude
- target/start 좌표 및 잠금 여부
- packet count, ST time delta
- 모터 enable 상태, solenoid 카운트

특징:

- 다중 프로세스 동시 접근을 위한 파일락 사용(Windows/POSIX 분기)
- read-merge-write 원자 업데이트 방식
- 재부팅 후 미션 상태 복구에 사용

## 10. 로그 구조

- 이벤트 로그: `eventlogs/events_*.csv`
- IPC/센서 로그 세션: `sensorlogs/run_*/`
  - 앱별 버스 로그 CSV
  - raw 센서 CSV(`raw_barometer.csv`, `raw_imu.csv`, ...)
- 모터 제어 디버그 로그: `motorlogs/motor_control_*.csv`

## 11. 실행/테스트 빠른 가이드

실행:

```bash
python main.py
```

주요 테스트:

```bash
python -m unittest
python -m unittest tests/test_main_smoke.py
python -m unittest tests/test_commapp.py
```

### 11.1 테스트 실행 기본 패턴

- 전체 테스트:

```bash
python -m unittest
```

- 단일 파일:

```bash
python -m unittest tests/test_commapp.py
```

- 단일 클래스:

```bash
python -m unittest tests.test_commapp.TestCommApp
```

- 단일 테스트 함수:

```bash
python -m unittest tests.test_commapp.TestCommApp.test_dispatch_route_command
```

### 11.2 test별 실행 커맨드

아래는 `tests/` 폴더의 주요 `test_*.py`를 개별 실행하는 예시입니다.

```bash
python -m unittest tests/test_main_smoke.py
python -m unittest tests/test_msgstructure.py
python -m unittest tests/test_prevstate.py
python -m unittest tests/test_commapp.py
python -m unittest tests/test_flightlogicapp.py
python -m unittest tests/test_barometerapp.py
python -m unittest tests/test_imuapp.py
python -m unittest tests/test_gpsapp.py
python -m unittest tests/test_gps_nmea.py
python -m unittest tests/test_distanceapp.py
python -m unittest tests/test_electroapp.py
python -m unittest tests/test_cameraapp.py
python -m unittest tests/test_motorapp.py
python -m unittest tests/test_motorapp_start_lock.py
python -m unittest tests/test_motor_control.py
python -m unittest tests/test_motor_guidance.py
python -m unittest tests/test_motor_guidance_safety.py
python -m unittest tests/test_motor_release_cal.py
python -m unittest tests/test_motor_actuators.py
python -m unittest tests/test_motor_ipc_harness.py
python -m unittest tests/test_harness_flow.py
python -m unittest tests/test_sensor_cli.py
python -m unittest tests/test_scenario_runner.py
```

### 11.3 목적별 추천 실행 순서

- IPC/기본 안정성 빠른 확인:
  - `tests/test_msgstructure.py`
  - `tests/test_prevstate.py`
  - `tests/test_commapp.py`
  - `tests/test_main_smoke.py`
- 비행로직/센서 경로 점검:
  - `tests/test_flightlogicapp.py`
  - `tests/test_barometerapp.py`
  - `tests/test_imuapp.py`
  - `tests/test_gpsapp.py`
  - `tests/test_distanceapp.py`
  - `tests/test_electroapp.py`
- 모터 유도/제어/액추에이터 점검:
  - `tests/test_motorapp.py`
  - `tests/test_motor_control.py`
  - `tests/test_motor_guidance.py`
  - `tests/test_motor_guidance_safety.py`
  - `tests/test_motor_release_cal.py`
  - `tests/test_motor_actuators.py`

### 11.4 실패 시 디버깅 팁

- verbosity를 올려 상세 로그 확인:

```bash
python -m unittest -v tests/test_commapp.py
```

- 최근 변경 모듈부터 파일 단위로 재실행 후, 마지막에 전체 테스트를 수행하는 것을 권장합니다.

## 12. 운영 시 권장 체크리스트

- UART 장치/baud가 XBee 설정과 일치하는지 확인
- I2C 버스 번호(`FSW_I2C_BUS`) 및 센서 주소 환경변수 점검
- 비행 전 `TC`로 target 좌표 설정(Release 안전 정책)
- `RBT_AUTH_TOKEN` 등 재부팅 인증 환경변수 관리
- 로그 디렉터리(`eventlogs`, `sensorlogs`, `motorlogs`) 저장공간 확인

---

관련 소스:

- `main.py`
- `lib/appargs.py`
- `lib/msgstructure.py`
- `comm/commapp.py`
- `flight_logic/flightlogicapp.py`
- `Sensor_Motor/motorapp.py`
- `lib/prevstate.py`
- `lib/events.py`
- `lib/sensorlog.py`
