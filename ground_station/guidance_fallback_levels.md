# Guidance Fallback Levels (Ground Station Local Estimator)

이 문서는 `ground_station.py`에 구현된 로컬 센서 신뢰도 기반 fallback 레벨 판정 기준을 정리한다.  
비행 SW(`guidance_state`)와 별개로 지상국에서 독립 추정하여 모니터링 목적으로 표시한다.

## 1) 입력 센서와 신선도 기준

- Location (`gps_lat`, `gps_lon`, `gps_sats`)
  - current: `age <= 0.35s`
  - stale 허용: `age <= 1.00s`
  - 품질: `gps_sats >= 4` (값이 비어있으면 품질 체크 생략)
- Course/Speed (GPS 연속점으로 추정)
  - current: `age <= 0.50s`
  - stale 허용: `age <= 1.00s`
  - 추정 유효 조건:
    - `0.2s <= dt <= 2.5s`
    - `1m <= dist <= 120m`
    - `0.8 <= speed <= 35 m/s`
- GyroZ (`gyro_yaw`)
  - current: `age <= 0.35s`
  - 물리 plausibility: `|gyro_yaw| <= 1500 deg/s`
- BMP Altitude (`altitude_m`)
  - current: `age <= 0.60s`
  - 물리 plausibility: `-1500m <= altitude <= 60000m`

## 2) 레벨 정의

- Level 0: `NORMAL_L1_PID`
  - Location/Course/Speed/GyroZ/BMP가 모두 current
- Level 1: `DEGRADED_L1_PID`
  - GyroZ current 유지
  - Location 또는 Course/Speed 또는 BMP 중 일부 stale(1초 이내)
  - 추정값 허용 구간에서 약화 제어
- Level 2: `L1_FF_ONLY`
  - Location/Course/Speed는 stale 허용 구간 내 유지
  - GyroZ current 없음
  - PID off, feed-forward only 가정
- Level 3: `TARGET_BEARING_HOLD`
  - Location current
  - Course/Speed 신뢰 불가
  - 목표점 bearing 약반영 가정
- Level 4: `YAW_DAMPING_ONLY`
  - Navigation 불가(Location stale 초과)
  - GyroZ current만 신뢰 가능
- Level 5: `SAFE_GLIDE_NEUTRAL`
  - 위치/방향/속도/회전율 핵심 상태 신뢰 불가

## 3) 전이 규칙 (히스테리시스)

- 강등(더 큰 레벨 번호): 즉시 전이
- 복귀(더 작은 레벨 번호): 아래 조건 동시 만족 시 전이
  - 복귀 후보 레벨이 `3.0s` 이상 연속 유지
  - 현재 레벨 체류시간이 `1.0s` 이상

목적: 경계 구간에서 level chattering 방지.

## 4) 지상국 UI 표시

- `guidance(fs): ...`
  - 비행 SW 텔레메트리 `guidance_state` 원본
- `fallback(local): ...`
  - 지상국 로컬 추정 레벨 (레벨, reason code, 센서 age)

예시:

`fallback(local): L1 DEGRADED_L1_PID (COURSE_SPEED_STALE_EST) [L:0.2 CS:0.7 G:0.1 A:0.2s]`

## 5) 참고

- 본 로직은 안전 모니터링 및 운용 판단 보조용이다.
- 실제 제어기 적용 시에는 비행 SW 내부 상태/제어 제한값과 동일 기준으로 맞추는 것이 바람직하다.
