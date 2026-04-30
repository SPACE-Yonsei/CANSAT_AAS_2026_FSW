# FSW Step 1 - System Contract Freeze

목표: 새 FSW 구현 전에 앱/메시지 계약을 고정하여 기능 누락과 인터페이스 붕괴를 방지한다.

## 1) App Catalog (고정 대상)

- Main: 라우팅, 프로세스 관리, 종료/재시작
- FlightLogic: 상태기계, 상태전이, 비행 이벤트 발행
- Comm: 지상국 명령 파싱, 텔레메트리 집계/송신
- Barometer: 고도/온도/압력 수집, 보정, 전송
- IMU: 자세/가속도/자기장/각속도 수집 및 필터링
- GPS: 위치/속도/코스/고정상태 수집
- Distance: 근접 거리 측정 (TF-Luna)
- Electro: 전압/전류/전력 수집 (INA228)
- Camera: 녹화 제어
- Motor: 유도/제어/액추에이터 실행, FDIR/failsafe

## 2) AppID Freeze

아래 값은 기존 운용과 호환 기준으로 유지한다.

- `Main=10`
- `FlightLogic=11`
- `Comm=12`
- `Barometer=13`
- `IMU=14`
- `GPS=15`
- `Distance=16`
- `Electro=17`
- `Camera=18`
- `Motor=19`

## 3) MID Freeze (핵심 계약)

### 3.1 Global/System
- `MID_TerminateProcess`: Main -> all apps, 프로세스 종료 신호

### 3.2 Route Command (Comm -> target app)
- `MID_RouteCmd_SIM`: Comm -> FlightLogic (`SIM ENABLE/ACTIVATE/DISABLE`)
- `MID_RouteCmd_SIMP`: Comm -> FlightLogic (SIMP 기반 고도 입력)
- `MID_RouteCmd_CAL`: Comm -> Barometer or FlightLogic (보정)
- `MID_RouteCmd_MEC`: Comm -> Motor (`MEC ON/OFF`)
- `MID_RouteCmd_SS`: Comm -> FlightLogic (`SS <state>`)
- `MID_RouteCmd_CAM`: Comm -> Camera (`CAM ON/OFF`)
- `MID_RouteCmd_TC`: Comm -> FlightLogic (`TC <lat>,<lon>`)

### 3.3 Sensor/State Telemetry to Comm
- `MID_comm_alt`: Barometer -> Comm (`pressure,temp,alt`)
- `MID_comm_euler`: IMU -> Comm (`filtered/acc/mag/gyro`)
- `MID_comm_gga`: GPS -> Comm (`time,alt,lat,lon,sats`)
- `MID_comm_dis`: Distance -> Comm (`distance_mm`)
- `MID_comm_volt`: Electro -> Comm (`volt,current,power`)
- `MID_comm_state`: FlightLogic -> Comm (`state`)
- `MID_comm_sim`: FlightLogic -> Comm (`mode`)

### 3.4 Flight/Motor Control Plane
- `MID_flight_alt`: Barometer -> FlightLogic (`alt`)
- `MID_flight_dis`: Distance -> FlightLogic (`distance_mm`)
- `MID_motor_alt`: Barometer -> Motor (`alt`)
- `MID_motor_imu`: IMU -> Motor (`yaw,gyrz,health`)
- `MID_motor_gps`: GPS -> Motor (`lat,lon,speed,course,fix,sats,rmc,health`)
- `MID_motor_state`: FlightLogic -> Motor (`state`)
- `MID_motor_burnwire`: FlightLogic -> Motor (release trigger)
- `MID_motor_EggDrop`: FlightLogic -> Motor (egg drop trigger)
- `MID_motor_PullArms`: FlightLogic -> Motor (arm command)
- `MID_motor_TargetCor`: FlightLogic -> Motor (`target_lat,target_lon`)
- `MID_cam_activate`: FlightLogic -> Camera (record activate)

## 4) Payload Schema Freeze (최소 필드 계약)

모든 메시지는 공통 포맷 문자열을 따른다.

- Bus envelope: `sender|receiver|MsgID|data`
- 금지: `data` 내부에 `|`
- 파싱 실패 시 정책: drop + error log

`data` 필드 최소 스키마:

- `MID_comm_alt`: `"{pressure},{temperature},{altitude}"`
- `MID_comm_euler`: `"{f_roll},{f_pitch},{f_yaw},{accx},{accy},{accz},{magx},{magy},{magz},{gyrx},{gyry},{gyrz}"`
- `MID_comm_gga`: `"{gps_time},{gps_alt},{gps_lat},{gps_lon},{gps_sats}"`
- `MID_comm_dis`: `"{distance_mm}"`
- `MID_comm_volt`: `"{voltage},{current},{power}"`
- `MID_comm_state`: `"{state_int}"`
- `MID_comm_sim`: `"{mode_char}"` where `mode_char in {F,S,A}`
- `MID_flight_alt`, `MID_motor_alt`: `"{altitude_m}"`
- `MID_flight_dis`: `"{distance_mm}"`
- `MID_motor_imu`: `"{yaw_deg},{gyrz_dps},{health_flag}"`
- `MID_motor_gps`: `"{lat},{lon},{speed_ms},{course_deg},{fix_quality},{sats},{rmc_status},{health_flag}"`
- `MID_motor_state`: `"{state_int}"`
- `MID_motor_TargetCor`: `"{target_lat},{target_lon}"`
- Route command payload:
  - SIM: `"ENABLE" | "ACTIVATE" | "DISABLE"`
  - SIMP: `"{pressure_like_value_or_alt}"`
  - CAL: `"CAL"` or `"CAL,{offset}"`
  - MEC: `"ON" | "OFF"`
  - SS: `"{0..5}"`
  - CAM: `"ON" | "OFF"`
  - TC: `"{lat},{lon}"`

## 5) Absolute Compatibility Rules (절대 호환 규칙)

1. AppID는 변경 금지. 변경 필요 시 alias 라우터를 제공한 뒤 점진 마이그레이션.
2. 기존 MID는 삭제 금지. 대체 MID 추가 시 최소 1 릴리즈 동안 동시 지원.
3. `sender|receiver|MsgID|data` 포맷은 유지.
4. 상태값 enum(0~5) 의미는 유지.
5. Comm 명령어 문법(CX/ST/SIM/SIMP/CAL/MEC/SS/RBT/CAM/TC)은 유지.
6. FDIR에서 failsafe(`neutral`/`off`) 동작은 완화 금지.
7. `prevstate` 핵심 항목(state/altcal/maxalt/target/packet_count/st_timedelta)은 보존.
8. Telemetry에 `distance` 필드는 반드시 포함.

## 6) Missing-Feature Verification Checklist

- [ ] 모든 앱 프로세스가 Main에 등록되고 launch 가능하다.
- [ ] 각 앱이 `MID_TerminateProcess`를 수신하면 정상 종료한다.
- [ ] Comm 명령어 10종이 모두 파싱/분기된다.
- [ ] RouteCmd 7종이 올바른 수신 앱으로 라우팅된다.
- [ ] FlightLogic 상태 0~5 전이가 재현된다.
- [ ] Motor 제어루프가 state>=3에서만 동작한다.
- [ ] FDIR 이상 시 neutral/off failsafe가 동작한다.
- [ ] 센서앱 송신 주기가 기존(10Hz/5Hz/1Hz) 정책을 만족한다.
- [ ] `MID_comm_dis`가 TLM 송신 문자열에 포함된다.
- [ ] prevstate 복원(재시작/재부팅) 시 핵심 값이 유지된다.
- [ ] restart_app가 앱별 launcher 시그니처 차이를 안전하게 처리한다.
- [ ] 센서 미연결 상태에서도 harness로 라우팅/상태전이 검증이 가능하다.

## 7) Step 1 Exit Criteria

아래 4개를 만족하면 Step 1 완료:

- AppID/MID/스키마/호환규칙 문서가 고정됨
- 구현자가 이 문서를 기준으로 코딩 가능
- 누락 체크리스트가 테스트 항목으로 전환 가능
- 다음 단계(main orchestrator 골격 작성) 입력 준비 완료

