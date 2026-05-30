# GCS SIM 모드 검증 절차 — GPS 신선도 · 모터 Fallback

> **대상 버전:** mag 브랜치 (2026-05-31 기준)
> **목적:** GCS GPS 신선도 조절 기능(주입/Null)을 이용해
> motorapp의 fallback 전이와 모터 출력을 벤치에서 검증한다.

---

## 1. 사전 지식

### 1.1 GPS 신선도 조작 버튼 위치

GCS Command 패널 하단 `GPS 신선도:` 행:

| 버튼 | 전송 커맨드 | FSW 내부 효과 |
|------|------------|--------------|
| **주입 (fresh)** | `SIMGR,{SIMGR 입력란 값}` | gpsapp `SIM_GPS_ACTIVE=True`, `pos_health=1` |
| **Null (stale)** | `SIMGN` | gpsapp `pos_health=0` 강제 → motorapp GPS 즉시 무효 |

SIMGR 입력란 기본값 `0,0,0,5,100` 형식: **`dE_m, dN_m, course_deg, speed_m/s, alt_m`**
(타겟 좌표 기준 상대 오프셋. `0,0,0,5,100` = 타겟 위 0m, 고도 100m, 동쪽으로 5m/s)

> **주의 — TLM 상의 GPS 표시:** SIMGN 후에도 TLM의 `gps_lat/lon`은 마지막 값이
> 동결 표시된다. `gps_time`도 계속 갱신되므로 TLM만 보면 GPS가 살아있는 것처럼
> 보인다. 실제 pos_health=0 여부는 GCS `GPS: ○STALE` 레이블 및
> `fallback(local)` 행의 `L:` 경과 시간으로 판단해야 한다.

### 1.2 ControlMode 결정 기준 (guidance.py `DecideControlMode`)

| 모드 | 조건 |
|------|------|
| `GPS_TRACKING_CLOSED` | GPS 위치·속도 신선 + IMU gyrz 신선 |
| `GPS_TRACKING_OPEN` | GPS 위치·속도 신선, gyrz 없음 |
| `DR_TRACKING_CLOSED` | DR 앵커 유효 + gyrz 신선 |
| `DR_TRACKING_OPEN` | DR 앵커 유효 + IMU yaw 신선 |
| `FAIL` | 위 모두 해당 없음 → 모터 WriteOff |

### 1.3 GCS 로컬 Fallback 레벨 (ground_station.py `GuidanceFallbackEstimator`)

| 레벨 | 이름 | 주요 이유 |
|------|------|----------|
| L0 | `NORMAL_L1_PID` | 모든 센서 current |
| L1 | `DEGRADED_L1_PID` | BMP stale / Location stale / CS stale |
| L2 | `L1_FF_ONLY` | gyrz 없음 |
| L3 | `TARGET_BEARING_HOLD` | course/speed 불확실 |
| L4 | `YAW_DAMPING_ONLY` | GPS 없음, gyro만 |
| L5 | `SAFE_GLIDE_NEUTRAL` | 신뢰 센서 없음 |

신선도 임계값 (`ground_station.py:147`):
- Location current: **0.35s**, stale: **1.0s**
- Course/Speed current: **0.50s**, stale: **1.0s**
- GyroZ current: **0.35s**
- BMP Alt current: **0.60s**

복구 히스테리시스: 신선 상태 **3.0초** 유지 후 상위 레벨로 복구.

---

## 2. SIM 모드 진입 절차 (공통 선행 단계)

> **이 단계를 건너뛰면 guidance_state, motor_enabled 등 TLM 뒷 컬럼이
> 모두 공란으로 출력된다.** 특히 MEC,ON이 필수.

```
1. SIM,ENABLE          → mode: F→A
2. SIM,ACTIVATE        → mode: A→S
3. SIMP,120            → altitude: 120.00
4. TC,37.56,126.94     → 타겟 좌표 설정
5. SS,3                → state: 0→3 (RELEASE)
6. SIMGR,0,0,0,5,100   → GPS 주입 (타겟 위치, course=0, 5m/s, 100m)
   또는 GCS "주입 (fresh)" 버튼 클릭
```

> **MEC,ON은 통상 불필요.** `prevstate.py`의 `PREV_MOTOR_ENABLED` 기본값이 `1`이므로
> motorapp이 기동 시 `MOTOR_ENABLED=True`로 자동 시작된다. 직전 세션에서
> `MEC,OFF`를 전송했다면 prevstate에 0이 저장되어 있으므로 이 경우에만 `MEC,ON` 필요.

진입 완료 확인:
- TLM: `mode=S, state=3, altitude=120.00, motor_enabled=1`
- `guidance(fs):` → `GPS_TRACKING_CLOSED` 또는 `GPS_TRACKING_OPEN`
- `fallback(local):` → `L0 NORMAL_L1_PID`
- 좌/우 펄스 바: 1500 근방 (중립) 또는 타겟 방향 편향값

---

## 3. TC-1 — GPS → Stale 전이 (Null 버튼)

**목적:** SIMGN 후 fallback 레벨 상승 및 모터 출력 변화 확인

### 3.1 GPS fresh 기준값 수집

1. SIMGR 입력란에 `0,0,0,5,100` 입력 후 **주입 (fresh)** 클릭
   - `GPS: ●FRESH` 표시
   - `fallback(local): L0`, `guidance(fs): GPS_TRACKING_CLOSED`
   - 좌/우 펄스 기록 (baseline)

### 3.2 Null → Stale 전이

2. **Null (stale)** 클릭 → `GPS: ○STALE`

| 경과 시간 | GCS `fallback(local)` | `guidance(fs)` (TLM) | 좌/우 펄스 |
|----------|----------------------|----------------------|-----------|
| 즉시 (~첫 TLM) | **L4 (NAV_UNAVAILABLE_GYRO_ONLY)** | `GPS_TRACKING_*` → `DR_TRACKING_*` | 소폭 변화 가능 |
| DR 앵커 소진 후 | **L5 (SAFE_GLIDE_NEUTRAL)** | `DR_TRACKING_*` → `FAIL` | **양쪽 0 또는 1500** |

> **버그 수정 (2026-05-31):** SIMGN 후 TLM의 동결된 GPS lat/lon이 매 패킷마다
> `_last_valid_location_ts`를 리셋해서 `location_s`가 0으로 유지되는 버그가 있었음.
> 이로 인해 L1 전이가 일어나지 않고 L0 유지 또는 L3 직행 현상 발생.
>
> **수정 후:** `SIMGN` → `_gps_null_mode=True` → TLM GPS로 인한 타임스탬프 리셋 차단
> → `location_s=inf`, `course_speed_s=inf` → 첫 TLM에서 **즉시 L4** 진입.
> `주입 (fresh)` 버튼 → `reactivate_gps()` → `_gps_null_mode=False` → 다음 TLM부터 추적 재개.
>
> **실측 관찰 (2026-05-31 1차):** SIMGN ~12초 후 IMU filtered_yaw가 ~360° → ~326°로
> 전환 (GPS course → DR+IMU heading 전환 시점). 2차 테스트에서 물리 이동 포함.

### 3.3 합격 기준

- [ ] SIMGN 후 **첫 TLM 수신 시** `fallback(local)` → **L4 이상**
- [ ] `guidance(fs)` TLM이 `GPS_TRACKING_*` → `DR_*` → `FAIL` 순서로 전환
- [ ] `FAIL` 전환 후 좌/우 펄스 = 1500(중립) 또는 0(WriteOff)
- [ ] GCS `GPS: ○STALE` 레이블 즉시 전환

---

## 4. TC-2 — Stale → Fresh 복구 (히스테리시스)

**목적:** fresh 재주입 시 3초 복구 타이머 작동 확인

TC-1 완료(stale 상태)에서 이어서 진행.

1. **주입 (fresh)** 클릭 → `GPS: ●FRESH`
2. 즉시 확인: fallback 레벨 **변화 없음** (히스테리시스 대기 중)
3. 3.0초 경과 후: `fallback(local)` → **L0 NORMAL_L1_PID**
4. `guidance(fs)` → `GPS_TRACKING_CLOSED`

### 합격 기준

- [ ] fresh 재주입 후 **3초 미만 시점에서 L0 복구 없음**
- [ ] 3초 이상 안정 유지 후 L0 복구
- [ ] 복구 후 모터 펄스가 타겟 방향으로 수렴

---

## 5. TC-3 — 빠른 주입/Null 반복 (히스테리시스 스트레스)

**목적:** 3초 미만의 빠른 상태 반전에서 오복구 없는 것 확인

```
[주입] → 1초 대기 → [Null] → 1초 대기
[주입] → 1초 대기 → [Null] → 1초 대기
[주입] → 4초 대기 (복구 대기)
```

### 합격 기준

- [ ] 주입 후 1초 반복 구간에서 fallback L0 복구 **없음**
- [ ] 마지막 주입 후 4초 대기 시 L0 복구
- [ ] DR 앵커가 살아있는 동안 `DR_TRACKING_*` 모드 유지 (FAIL 조기 전환 금지)

---

## 6. TC-4 — IMU_HEADING 모드에서의 Stale

**목적:** GPS 없을 때 IMU_HEADING 모드가 heading 유지 fallback으로 작동하는지 확인

1. SIM 진입 완료 후 `Ctrl mode` 버튼 클릭 → `IMU_HEADING` 선택 (`CMC,IMU_HEADING` TX)
2. **주입 (fresh)** 클릭 → GPS+타겟 유효 시 bearing 계산, 유효하지 않으면 heading 유지
3. **Null (stale)** 클릭

### 기대 동작

- IMU_HEADING 모드는 GPS stale 후에도 IMU yaw 기반 heading으로 PID 유지
- 좌/우 펄스 non-zero 유지 (GPS_GUIDED와 달리 DR 앵커 불필요)
- `guidance(fs)` TLM → `FAIL` 전환 없이 IMU_HEADING 유지

### 합격 기준

- [ ] SIMGN 후 좌/우 펄스 0이 되지 않음 (GPS_GUIDED 와 구별)
- [ ] filtered_yaw 연속 변화 (PID가 동작 중)

---

## 7. 관찰 체크리스트

```
□  mode=S, state=3 확인 후 MEC,ON 전송
□  GPS: ●FRESH 확인 (주입 버튼 또는 수동 SIMGR)
□  fallback(local) L0, guidance(fs) GPS_TRACKING_CLOSED 기준값 확인
□  Null 클릭 → 1.0s 이내 fallback L 상승 확인
□  12~15s 후 guidance(fs) → FAIL 전환, 펄스 변화 확인
□  주입 재클릭 → 3s 후 L0 복구 확인
□  TC-3 빠른 반복 시 조기 복구 없음 확인
```

---

## 8. 알려진 동작 특이사항

| 현상 | 원인 | 확인 방법 |
|------|------|----------|
| SIMGN 후 TLM gps_lat/lon 동결 표시 | gpsapp이 pos_health=0으로 보내지만 lat/lon 값 자체는 유지 | GCS `GPS: ○STALE` 레이블 및 `L:` 경과 시간 확인 |
| gps_time이 SIMGN 후에도 계속 갱신 | gpsapp의 GPS 메시지 타임스탬프는 계속 발행 | 신선도는 pos_health=0 여부로만 판단 |
| filtered_yaw가 SIMGN 후 ~12초 뒤 급변 | guidance가 GPS course → DR+IMU heading으로 전환되는 시점 | 정상 동작. 예상 전환 시간: DR anchor time-out 이후 |
| MEC,ON 없어도 motorapp이 MOTOR_ENABLED=True로 기동 | prevstate `PREV_MOTOR_ENABLED` 기본값=1, 직전 세션 MEC,OFF 시엔 0 복원 | TLM `motor_enabled` 확인; 0이면 MEC,ON 전송 |
| guidance_state, motor_enabled TLM 공란 | SS,3 이전이거나 GPS origin 미확정 시 ctrl_cycle FAIL 조기 반환 | SS,3 + SIMGR 주입 후 확인 |
