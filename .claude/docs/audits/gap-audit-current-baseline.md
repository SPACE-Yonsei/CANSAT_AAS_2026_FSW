# FSW Step 12 - Gap Audit (Current Baseline)

기준 문서: `../baseline/fsw-baseline.md`, `../contracts/ipc-contract-freeze.md`

## 1) 핵심 판정 요약

- 메시지 계약 계층: 구현됨 (`lib/msgstructure.py`) / 테스트 통과
- 상태기계/FDIR 체인: 베이스 구현됨 (`flight_logic`, `Sensor_Motor`) / 테스트 통과
- prevstate 복원: 구현됨 (`lib/prevstate.py`) / 테스트 통과
- distance TLM 포함: 구현됨 (`comm/commapp.py`) / 테스트 통과
- 센서 없는 하네스: 구현됨 (`tests/test_harness_flow.py`) / 테스트 통과

## 2) 누락/부분구현/위험요소

### A. 부분구현
1. 하드웨어 드라이버 계층
   - 현재 센서/카메라/모터 드라이버는 synthetic/stub 중심.
   - 실제 I2C/UART/GPIO/picamera2 연동 코드로 교체 필요.

2. Guidance 알고리즘
   - 현재는 L1-like 단순화 버전.
   - figure-8, anti-windup, slew-rate, wind compensation 고도화 필요.

3. Comm 보안
   - `RBT`는 토큰 게이트 추가됨.
   - 향후 nonce/시퀀스/유효시간 검증으로 강화 필요.

### B. 위험요소
1. 실제 센서 주기/지터 검증 미완
   - 합성 데이터 기준 통과이므로 하드웨어-in-loop 시험 필요.
2. 프로세스 재시작 회복성 장기 시험 미완
   - soak test(수시간)로 queue backlog/leak 확인 필요.

## 3) 즉시 보완 권고 (우선순위)

1. IMU/BARO/GPS/Distance/Electro 실제 드라이버 교체
2. Motor guidance 고급 로직 이식
3. 하드웨어 없는 CI 테스트 + 하드웨어 통합 테스트 분리
4. systemd 실제 배포 경로로 실기기 검증
5. TLM 스키마 고정 스냅샷 테스트 추가

## 4) 현 시점 테스트 증거

- `python -m unittest ...` -> 24 tests passed
- `python -m py_compile ...` -> core modules pass

## 5) 결론

현재 브랜치 결과물은 "기능 누락 방지용 실행형 베이스라인"으로 유효하다.  
다음 단계는 하드웨어 실연동 및 제어 알고리즘 정밀 이식이다.
