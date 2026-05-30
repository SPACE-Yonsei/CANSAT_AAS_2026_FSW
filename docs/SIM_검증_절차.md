# GCS SIM 모드 검증 절차 — GPS 신선도 · Fallback

> **대상 버전:** mag 브랜치 (2026-05-31)
> **목적:** GCS GPS 신선도 조절(주입/Null)로 motorapp fallback 전이를 벤치에서 검증.

---

## 1. 사전 지식

### 1.1 GPS 신선도 조작

GCS Command 패널 `GPS 신선도:` 행:

| 버튼 / 명령창 입력 | FSW 효과 | GCS 효과 |
|-------------------|---------|---------|
| **주입 (fresh)** 버튼 또는 명령창 `SIMGR,...` | gpsapp `pos_health=1`, motion 유효 | `GPS: ●FRESH`, `_gps_null_mode=False`, CS 3s 선행 부스트 |
| **Null (stale)** 버튼 또는 명령창 `SIMGN` | gpsapp `pos_health=0` | `GPS: ○STALE`, `_gps_null_mode=True`, location/CS = ∞ |

SIMGR 입력란 형식: `dE_m, dN_m, course_deg, speed_m/s, alt_m` (타겟 기준 상대 오프셋)
- `0,0,0,5,100` = 타겟 정상, course 0°, 5 m/s, 고도 100 m

> **TLM 주의:** SIMGN 후에도 TLM `gps_lat/lon`은 마지막 값으로 동결 표시,
> `gps_time`도 계속 갱신된다. 신선도는 `GPS: ○STALE` 레이블과
> `fallback(local)` 행의 나이 숫자(`L: CS: G: A:`)로만 판단한다.

### 1.2 GCS 로컬 Fallback 레벨

| 레벨 | 이름 | 전환 조건 |
|------|------|----------|
| L0 | `NORMAL_L1_PID` | location·CS·gyroZ·Alt 모두 current |
| L1 | `DEGRADED_L1_PID` | location stale 또는 CS stale (1개 이상) |
| L2 | `L1_FF_ONLY` | gyroZ 없음 |
| L3 | `TARGET_BEARING_HOLD` | location fresh + CS stale |
| L4 | `YAW_DAMPING_ONLY` | GPS 없음 (location ≥ 1s), gyroZ 유효 |
| L5 | `SAFE_GLIDE_NEUTRAL` | 신뢰 센서 없음 |

신선도 임계값:
| 센서 | current (≤) | stale (>) |
|------|------------|-----------|
| Location (GPS 위치) | 0.35 s | 1.0 s |
| Course/Speed (GPS 속도) | 0.50 s | 1.0 s |
| GyroZ (IMU) | 0.35 s | — |
| BMP Alt | 0.60 s | — |

**히스테리시스:** 악화(downgrade) 즉각 / 복구(upgrade)는 3.0 s 연속 안정 필요.

### 1.3 벤치 환경 제약

| 항목 | 벤치 (Pi 없음) | 실 하드웨어 |
|------|---------------|------------|
| Motor pulse TLM | 항상 `0,0` (pigpio 미초기화) | 실제 servo pulse |
| guidance_state TLM | 공란 (motor output 없음) | 실제 모드 |
| fallback(local) GCS 레이블 | **검증 가능** | 검증 가능 |

벤치에서는 **`fallback(local)`** 레이블만 관찰 기준으로 삼는다.

### 1.4 SIMGR 주입 후 복구 동작

SIMGR은 FSW에 `motion_health=1` (course/speed 유효) 데이터를 제공하므로, GCS 추정기는
주입 즉시 CS를 fresh로 간주한다 (`cs_ts = now + 3.0 s`). 이후 타임라인:

| 경과 시간 | cs_s | raw 레벨 | level (히스테리시스 고려) |
|----------|------|---------|------------------------|
| 0 s | 0 | L0 | 복구 후보 시작 |
| 3.0 s | 0 | L0 | **L0 복구 완료** |
| 3.5 s | 0.5 | L1 | L0 → L1 즉각 하강 |
| 4.5 s | 1.5 | L3 | L1 → L3 즉각 하강 |

---

## 2. SIM 모드 진입 (공통 선행 단계)

```
1. SIM,ENABLE          → mode: F→A
2. SIM,ACTIVATE        → mode: A→S
3. SIMP,120            → altitude: 120.00  (fallback 테스트 시 생략 가능)
4. TC,37.57,126.94     → 타겟 설정 (또는 원하는 좌표)
5. SS,3                → state: 0→3
6. SIMGR,0,0,0,5,100   → GPS 주입 / 또는 "주입 (fresh)" 버튼
```

완료 확인:
- TLM: `mode=S`, `state=3`
- `GPS: ●FRESH`
- `fallback(local): L0 NORMAL_L1_PID [L:0.0 CS:0.0 G:0.0 A:0.0s]`

> MEC,ON은 통상 불필요. `prevstate.PREV_MOTOR_ENABLED` 기본값=1이므로
> motorapp 기동 시 자동 MOTOR_ENABLED=True. 직전 세션 MEC,OFF 이력이 있으면 필요.

---

## 3. TC-1 — GPS Null → 즉각 L4 전이

**검증:** SIMGN 후 fallback(local)이 첫 TLM에서 L4로 전환되는지 확인.

### 절차

1. SIM 진입 완료 → `fallback(local): L0` 기준값 확인
2. **Null (stale)** 클릭 (또는 명령창 `SIMGN`)

### 기대 동작

| 타이밍 | `fallback(local)` | `GPS: ` 레이블 |
|--------|------------------|--------------|
| 버튼 클릭 즉시 | L0 유지 (아직 TLM 미도착) | `GPS: ○STALE` |
| 첫 TLM 수신 (~1 s) | **L4 NAV_UNAVAILABLE_GYRO_ONLY** | `GPS: ○STALE` |
| `[L:-- CS:-- G:0.x A:0.xs]` | location·CS age = ∞ | — |

### 합격 기준

- [ ] **Null 버튼 클릭 직후** `GPS: ○STALE` 레이블 즉시 전환
- [ ] **첫 TLM 수신 시** `fallback(local)` → L4 이상
- [ ] `L: --` `CS: --` (∞) 표시 (동결 GPS로 인한 false-fresh 없음)

---

## 4. TC-2 — SIMGR 재주입 → 3초 후 L0 복구

**검증:** fresh 재주입 후 히스테리시스 3 s 타이머가 정상 작동하는지 확인.

TC-1 완료 후 (L4 상태) 이어서 진행.

### 절차

1. **주입 (fresh)** 클릭 (또는 명령창 `SIMGR,0,0,0,5,100`)
2. GCS 레이블 관찰: `GPS: ●FRESH`
3. 타이머 시작 — 3 s 대기

### 기대 동작

| 경과 | `fallback(local)` |
|------|------------------|
| 0 s | L4 유지 (히스테리시스 대기) |
| ~3 s | **L0 NORMAL_L1_PID** |
| ~3.5 s | L1 DEGRADED (CS 자연 소멸) |

### 합격 기준

- [ ] 주입 후 **3 s 미만** 시점에서 L0 복구 없음
- [ ] 주입 후 **3 s 경과** 시 L0 전환
- [ ] 3.5 s 이후 L1 → L3 자연 하강 (CS 소멸 정상)

---

## 5. TC-3 — 빠른 주입/Null 반복 (히스테리시스 스트레스)

**검증:** 1 s 간격 빠른 반복에서 L0 오복구 없음, 마지막 주입 후 3 s에 복구.

### 절차 (TC-1 상태에서 시작)

```
주입 → 1 s 대기 → Null → 1 s 대기
주입 → 1 s 대기 → Null → 1 s 대기
주입 → 3 s 대기   ← 복구 대기
```

### 기대 동작

- 1 s 간격 구간: `주입 → L4` 진입 → `Null → L4 유지` (L0 복구 안 됨)
  - 주입 직후 L0 raw 발생하지만 Null이 3 s 내에 들어오므로 복구 미완성
- 마지막 주입 후 3 s: **L0 복구**

### 합격 기준

- [ ] 1 s 반복 구간 중 L0 전환 **없음**
- [ ] 마지막 주입 후 정확히 **3 s ± 1 TLM 주기**에 L0 전환
- [ ] L0 직후 ~0.5 s 내 L1 하강 (CS 소멸 — 정상 동작)

---

## 6. TC-4 — IMU_HEADING 모드에서 Null

**검증:** GPS stale 후에도 IMU heading 유지, fallback이 L4로 전환.

### 절차

1. SIM 진입 완료
2. `Ctrl mode` 버튼 → `IMU_HEADING` (또는 명령창 `CMC,IMU_HEADING`)
3. **주입 (fresh)** → `GPS: ●FRESH`, `fallback(local): L0`
4. `filtered_yaw` 기준값 기록 (GCS heading 표시 또는 TLM 필드)
5. **Null (stale)** 클릭

### 기대 동작

| 항목 | 기대값 |
|------|--------|
| `fallback(local)` | L4 전환 (L0→L4, TC-1과 동일) |
| `filtered_yaw` | Null 전후 동일하게 유지 (IMU 기반) |
| 실 하드웨어 motor pulse | 0이 되지 않음 (IMU_HEADING은 GPS 없어도 동작) |

> **벤치 한계:** Pi 없으면 pulse = 0이므로 `filtered_yaw`의 연속성으로만 판단.

### 합격 기준

- [ ] Null 후 `fallback(local)` → L4 (TC-1과 동일)
- [ ] `filtered_yaw` 연속 변화 없음 (heading 유지)
- [ ] (실 하드웨어) GPS_GUIDED의 L4 후 WriteOff(0)와 달리 pulse non-zero

---

## 7. 관찰 체크리스트

```
□  mode=S, state=3, GPS: ●FRESH 확인
□  fallback(local) L0 기준값 확인
□  TC-1: Null → 첫 TLM 내 L4 전환 확인
□  TC-2: 주입 후 3 s에 L0 복구 확인
□  TC-3: 1 s 반복 중 L0 오복구 없음, 마지막 주입 3 s 후 복구
□  TC-4: IMU_HEADING + Null → L4, filtered_yaw 유지
```

---

## 8. 알려진 특이사항

| 현상 | 원인 | 확인 방법 |
|------|------|----------|
| SIMGN 후 TLM `gps_lat/lon` 동결 표시 | gpsapp이 last fix 유지하며 `pos_health=0`만 변경 | `GPS: ○STALE` 레이블 및 `L: --` 확인 |
| `gps_time` SIMGN 후에도 계속 갱신 | gpsapp GPS 타임스탬프는 pos_health 무관하게 발행 | 정상 동작 |
| 벤치에서 motor pulse 항상 `0,0` | pigpio 미초기화 (PI=None) → WriteOff/WriteZero 호출 skip | 실 Pi에서 재검증 필요 |
| `guidance_state` TLM 항상 공란 | motor output 없는 상태에서 commapp이 필드 미발행 | 실 Pi 또는 상태 확인 |
| L0 복구 후 즉시 L1으로 하강 | CS 부스트 만료 (3.5 s 후) — 정상 동작 | SIMGR 재주입으로 L0 유지 |
| prevstate에 MEC,OFF 이력 있으면 motor 비활성 | `PREV_MOTOR_ENABLED=0` 복원 | TLM `motor_enabled` 확인, `MEC,ON` 전송 |
