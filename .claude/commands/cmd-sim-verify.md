---
description: sim_verify.py를 실행하고 4개 검증 질문의 핵심 지표를 추출한다 (Phase 4)
allowed-tools: Bash Read
---

`sim_verify.py`를 실행하고 결과를 분석하라. 인자: `$ARGUMENTS` (없으면 기본값 사용)

## 실행

```bash
PYTHONIOENCODING=utf-8 .venv/Scripts/python.exe sim_verify.py $ARGUMENTS 2>&1
```

## 핵심 지표 추출

실행 후 다음 4개 질문에 대한 결과를 표로 정리하라:

| 질문 | 지표 | 기준값 | 실측값 | 판정 |
|------|------|--------|--------|------|
| Q1: Monte Carlo 수렴 | 목표 반경 내 착지율 | ≥ 80% | ? | PASS/FAIL |
| Q2: FDIR 게이트 | fault → neutral 전환 건수 | 100% | ? | PASS/FAIL |
| Q3: Jitter | 정상 비행 중 yaw_rate 부호변화/초 | ≤ 0.5회/s | ? | PASS/FAIL |
| Q4: 180° deadlock | ±180° 초기오차 탈출 시간 | ≤ 10s | ? | PASS/FAIL |

## Phase별 해석 기준

**Phase 1 완료 직후 (L1 교체 후 첫 실행):**
- Q1 수렴율이 tanh 기준 대비 +5%p 이상이면 개선 확인
- Q4 탈출 시간이 줄어들면 `_prevent_indecision` 효과 확인

**Phase 4 (시뮬레이터 고도화 후):**
- 바람 perturbation ±5 m/s 하에서 Q1 ≥ 70% 유지
- Monte Carlo n ≥ 200으로 통계적 유의성 확보

실패 항목은 `motor_guidance.py` 관련 파라미터와 연결하여 튜닝 제안을 포함하라.
