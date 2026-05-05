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
