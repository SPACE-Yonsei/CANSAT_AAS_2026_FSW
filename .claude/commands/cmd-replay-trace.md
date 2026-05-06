---
description: 낙하 테스트 CSV 로그를 replay_motor_trace.py로 재현하고 경로 추종 지표를 분석한다 (Phase 5)
allowed-tools: Bash Read
argument-hint: "<csv_path> (예: logs/drop_test_01.csv)"
---

낙하 테스트 로그를 리플레이하고 제어 성능을 분석하라. 인자: `$ARGUMENTS`

## 실행

```bash
PYTHONIOENCODING=utf-8 .venv/Scripts/python.exe tests/replay_motor_trace.py $ARGUMENTS 2>&1
```

인자가 없으면 사용법을 출력하고 중단하라.

## 핵심 지표 추출

리플레이 완료 후 `replay_result.csv`와 `replay_segment_summary.md`를 읽어 다음을 표로 정리하라:

| 지표 | 값 | 기준 | 판정 |
|------|-----|------|------|
| 최종 목표 거리 (final_distance) | ? m | ≤ 30 m | PASS/FAIL |
| Cross-track RMS | ? m | ≤ 15 m | PASS/FAIL |
| Cross-track 최대값 | ? m | ≤ 50 m | PASS/FAIL |
| Along-track 단조 증가율 | ? % | ≥ 80% | PASS/FAIL |
| Yaw-rate 부호변화/초 | ? 회/s | ≤ 1.0 | PASS/FAIL |
| 서보 포화율 | ? % | ≤ 30% | PASS/FAIL |
| FDIR dwell time | ? s | ≤ 5 s | PASS/FAIL |

## 세그먼트 분석

`replay_segment_summary.md`의 각 세그먼트(INIT_INVALID_GPS, EARLY_DESCENT, STABLE_GLIDE, HIGH_YAW_RATE, LOW_ALTITUDE, SENSOR_FAULT)에 대해:
- 지속 시간과 비율
- 해당 구간 평균 cross-track error
- 이상 거동 구간 식별

## 튜닝 제안

FAIL 항목이 있으면 다음과 연결하여 파라미터 조정 방향을 제시하라:
- `final_distance > 30m` → `L_DISTANCE`, `Kp_inner` 조정
- `cross-track RMS 과대` → `YR_MAX`, `Ki_inner` 조정  
- `yaw-rate 진동` → `DEADBAND`, `MAX_ACCEL` 조정
- `FDIR dwell 과대` → GPS health 임계값, stale timeout 조정
