# CANSAT_AAS_2026_Flight_Logic
Flight Logic App for CANSAT AAS 2026

## FLIGHTLOGICAPP 용어 정리

### 상태 (State) 정의

#### 1. LAUNCH_PAD (상태 0)
- **설명**: 로켓에 탑재된 상태, 땅에서부터 상공 150m까지
- **전환 조건**: 고도가 150m를 초과하면 ASCENT 상태로 전환
- **동작**:
  - 모든 센서 데이터 수집
  - 최대 고도(MAX_ALT) 초기화
  - 파라포일 모터 비활성화

#### 2. ASCENT (상태 1)
- **설명**: 상승 단계, 상공 150m부터 최고고도까지
- **전환 조건**:
  - 최고고도 근처(MAX_ALT - 0.25m 이내)에서 APOGEE 상태로 전환
  - 최고고도에서 20m 이상 하강 시 DESCENT 상태로 전환
- **동작**:
  - 최대 고도(MAX_ALT) 추적 및 업데이트
  - 카메라 활성화
  - 파라포일 모터 비활성화

#### 3. APOGEE (상태 2)
- **설명**: 최고고도 도달 단계
- **전환 조건**: 최고고도에서 20m 이상 하강 시 DESCENT 상태로 전환
- **동작**:
  - 파라포일 알고리즘 시작 (GPS 기반 목표 지점 계산)
  - **파라포일 모터는 작동하지 않음** (모터 제어 비활성화)
  - 최대 고도 유지 및 모니터링

#### 4. DESCENT (상태 3)
- **설명**: 하강 단계, 최고고도부터 최고고도의 80%까지
- **전환 조건**: 고도가 최고고도의 80% 이하로 하강 시 PROBE_RELEASE 상태로 전환
- **동작**:
  - 파라포일 모터 작동 시작 (GPS 기반 목표 지점으로 유도)
  - 카메라 활성화
  - GPS 기반 모터 제어 활성화

#### 5. PROBE_RELEASE (상태 4)
- **설명**: Container와 Payload 분리, 파라포일로 하강
- **전환 조건**: 고도가 15m 이하로 하강 시 LANDED 상태로 전환
- **동작**:
  - **번와이어(Burn Wire) 작동**: Container와 Payload 분리
  - **파라포일 모터 작동**: GPS 기반 목표 지점으로 유도
  - **솔레노이드 안전 작동**: 고도 3~4m에서 솔레노이드 5~6번 반복 작동 (안전을 위해)
  - **Egg 분리**: 목표 GPS 지점 도달 후 고도 2m에서 egg 분리 (egg motor 작동)
  - 카메라 활성화

#### 6. LANDED (상태 5)
- **설명**: 착륙 완료
- **전환 조건**: 최종 상태 (수동 전환 또는 고도 15m 이하에서 자동 전환)
- **동작**:
  - **모든 모터 종료**: 파라포일 모터 정지
  - 상태 저장 및 로깅

### 주요 동작 요약

#### 파라포일 모터 작동 시점
- **비활성화**: LAUNCH_PAD, ASCENT, APOGEE 상태
- **활성화**: DESCENT, PROBE_RELEASE 상태
  - GPS 기반 목표 지점으로 유도
  - IMU 데이터(100Hz)로 실시간 제어

#### 번와이어(Burn Wire) 작동
- **시점**: PROBE_RELEASE 상태 전환 시 즉시 작동
- **목적**: Container와 Payload 분리

#### 솔레노이드 작동
- **시점**: PROBE_RELEASE 상태에서 고도 3~4m
- **횟수**: 5~6번 반복 작동
- **목적**: 안전을 위한 사전 작동

#### Egg 분리
- **조건**: 
  1. 목표 GPS 지점 도달 (반경 50m 이내)
  2. 고도 2m 이하
- **동작**: Egg motor 작동하여 egg 분리

### 상태 전환 다이어그램

```
LAUNCH_PAD (0)
    ↓ (고도 > 150m)
ASCENT (1)
    ↓ (최고고도 도달)
APOGEE (2) [파라포일 알고리즘 시작, 모터 작동 안 함]
    ↓ (고도 하강)
DESCENT (3) [파라포일 모터 작동 시작]
    ↓ (고도 ≤ 최고고도 × 80%)
PROBE_RELEASE (4) [번와이어 작동, 파라포일 모터 작동]
    ├─ (고도 3~4m) → 솔레노이드 5~6번 작동
    └─ (목표 도달 + 고도 ≤ 2m) → Egg 분리
    ↓ (고도 ≤ 15m)
LANDED (5) [모든 모터 종료]
```

### 고도 기준점

- **150m**: LAUNCH_PAD → ASCENT 전환
- **최고고도**: APOGEE 상태 도달
- **최고고도 × 80%**: DESCENT → PROBE_RELEASE 전환
- **3~4m**: 솔레노이드 안전 작동 구간
- **2m**: Egg 분리 고도
- **15m**: PROBE_RELEASE → LANDED 전환

### 메시지 ID (Message ID)

- `MID_PayloadReleaseMotorActivate` (1403): 번와이어 작동
- `MID_SendPayloadMotorRatation` (1409): 파라포일 모터 제어
- `MID_SolenoidActivate` (1411): 솔레노이드 작동
- `MID_PayloadEggMotorActivate` (1410): Egg 분리 모터 작동
- `MID_PayloadMotorStop` (1412): 모든 모터 정지
