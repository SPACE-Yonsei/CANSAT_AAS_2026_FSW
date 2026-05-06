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
