# Ground Station GUI

XBee로 들어오는 CANSAT TLM을 수신/표시/CSV 저장하고, 명령을 송신하는 단일 파일 GUI입니다.

## 실행

```powershell
pip install pyserial
python ground_station/ground_station.py
```

`tkinter`는 Windows 표준 Python에 기본 포함이라 별도 설치가 필요 없습니다.

## 사용

1. 상단의 `Port`에서 **수신측 XBee 어댑터 COM 포트** 선택 (FSW가 송신에 쓰는 COM은 여기 쓰지 마세요)
2. `Baud`는 FSW 환경변수 `UART_BAUD` 와 동일하게 (기본 9600)
3. `Connect` 버튼 클릭
4. 좌측 패널에 라이브 텔레메트리, 우측 콘솔에 raw RX가 출력됩니다.
5. `CSV log` 체크 시 `ground_station/logs/gs_YYYYMMDD_HHMMSS.csv` 로 저장됩니다.

## 명령 송신

`CMD,1070,<CMD>,<option>` 형식으로 전송합니다 (FSW의 `comm/commapp.py` 가 받는 포맷).

프리셋 드롭다운에서 선택하거나, `Body` 입력란에 직접 입력 후 `Send` 또는 Enter.

지원 명령 예시:
- `CX,ON` / `CX,OFF`
- `ST,GPS` 또는 `ST,HH:MM:SS`
- `SIM,ENABLE|ACTIVATE|DISABLE`
- `SIMP,<pressure>`
- `CAL,<value>`
- `MEC,ON|OFF`
- `SS,0..5`
- `CAM,ON|OFF`
- `TC,<lat>,<lon>`
- `XRST,NOW`

## TLM 포맷

`comm/commapp.py:send_tlm` 가 보내는 30 필드 CSV (`$1070,...`)를 그대로 파싱합니다. 필드가 부족하면 raw 콘솔에는 보이되 GUI 값은 갱신되지 않고 `err` 카운터가 올라갑니다.

## Scenario player (closed-loop SIM 자동 시연)

`SIMG`/`SIMP`를 자동 송신하여 캔위성 비행을 GCS 지도에 애니메이션으로 띄울 수 있습니다. closed-loop이기 때문에 FSW가 텔레메트리로 보내는 `left_pulse_us`/`right_pulse_us`를 다시 읽어 가상 캔위성의 heading을 업데이트하므로, 강풍/낙하속도 시나리오에서 **유도 응답이 실제로 경로에 반영**됩니다.

### GCS GUI 에서

1. 평소처럼 Connect 후 우측 하단의 **Scenario player** 패널에서 preset 선택 (`calm`, `west8`, `gust12`, `fast_descent`, `slow_descent`, `anti_parallel`, `long_range`)
2. 필요하면 wind speed / wind dir / descent / airspeed / turn 90deg distance 입력란을 채워 preset 값 일부만 override (빈칸이면 preset 값 사용)
3. **Play** 클릭 — 자동으로 `CX,ON → SIM,ENABLE → SIM,ACTIVATE → TC → SIMG → SIMP → SS,3`을 보낸 뒤 1Hz로 SIMG/SIMP를 갱신
4. 끝나면 자동으로 `SS,5 → SIM,DISABLE`. 중간에 멈추려면 **Stop**

`Mission flow mode`를 켜면 target 반경에 먼저 들어가도 즉시 종료하지 않고 계속 하강하므로, SIM 모드에서 `RELEASE -> EGG -> LANDED` 흐름 확인이 쉬워집니다.

### CLI 로 (GCS 끄고)

```powershell
python ground_station/scenario_player.py --list                 # 사용 가능한 preset 보기
python ground_station/scenario_player.py --port COM5 --scenario west8
python ground_station/scenario_player.py --port COM5 --scenario gust12
```

빌트인 preset 요약:

| preset          | 풍속              | 낙하   | 비고                          |
|-----------------|------------------|--------|------------------------------|
| `calm`          | 0 m/s            | 5 m/s  | 350 m NE 목표, 200 m 시작 고도 |
| `west8`         | 8 m/s W          | 5 m/s  | 일정 강풍 측풍                 |
| `gust12`        | 12 m/s W ± 3 m/s | 5 m/s  | 4 s 주기 sin gust + 강풍       |
| `fast_descent`  | 0                | 8 m/s  | 짧은 비행                     |
| `slow_descent`  | 0                | 3 m/s  | 긴 글라이드                    |
| `anti_parallel` | 2 m/s            | 4 m/s  | 180° 헤딩 오류 회복 테스트      |
| `long_range`    | 2 m/s            | 5 m/s  | 1 km 목표, 600 m 시작 고도     |

### 물리 모델 (단순화)

- 헤딩 적분: `yaw_rate ≈ pulse_to_yaw_gain × delta_arm_deg`, 여기서 `delta_arm_deg = (left_pw + right_pw − 3100) / (2000/180)` — `Sensor_Motor/motor_control.py` 의 mixer 와 동일 식
- 헤딩 적분: `yaw_rate ≈ gain × delta_arm_deg`, 여기서 `delta_arm_deg = (left_pw + right_pw − 3000) / (2000/180)` — `Sensor_Motor/motor_control.py` 중립값(1511/1489) 기준
- 기본 `gain`은 실측값(최대 조향에서 약 30m 진행 시 90도 회전)과 기준 속도로 자동 환산
- 지면속도: airspeed 벡터(heading 방향) + 풍속 벡터 (`wind_dir_met` = 풍원 방향, 기상학적 관습)
- gust: `wind_speed + gust_amp × sin(2π t / period)`
- 고도: 선택적 jitter 포함 일정 sink rate
- 수치 적분: tick 내부 substep 적분으로 통신 주기가 커도 경로 점프를 완화
- 종료 조건: `alt ≤ 0` (착륙) / 목표 반경 도달 / `timeout_s` 초과

### 안전

- Stop / Disconnect / 창 닫기 모두에서 시나리오를 정리. Disconnect 시점에 시나리오가 아직 실행 중이면 send_teardown 없이 즉시 정리 (포트가 이미 닫혔을 수 있음).
- 매 SIMG는 `(0,0)`을 방지하기 위해 클램프 후 송신.
