---
description: motor 관련 테스트 4종을 일괄 실행하고 결과를 요약한다
allowed-tools: Bash
---

motor 관련 테스트를 실행하라.

## 실행 순서

### 1. 문법 검사 (빠른 사전 확인)
```bash
.venv/Scripts/python.exe -m py_compile Sensor_Motor/motor_guidance.py Sensor_Motor/motorapp.py Sensor_Motor/motor_control.py && echo "SYNTAX OK"
```

### 2. 단위 테스트 4종 실행
```bash
.venv/Scripts/python.exe -m pytest \
  tests/test_motor_guidance.py \
  tests/test_motor_guidance_safety.py \
  tests/test_motorapp.py \
  tests/test_motor_control.py \
  tests/test_motor_actuators.py \
  tests/test_motor_ipc_harness.py \
  -v --tb=short 2>&1
```

## 결과 요약 형식

실행 후 다음 형식으로 요약하라:

```
PASSED: N / FAILED: N / ERROR: N

실패 항목:
  - test_xxx: <실패 원인 한 줄>

커버리지 공백:
  - eta/sin(η) 계산 검증: [있음/없음]
  - V=0 corner case: [있음/없음]
  - wind crab 분리 검증: [있음/없음]
```

실패가 있으면 가장 가능성 높은 원인과 수정 위치(파일:줄)를 제시하라.
