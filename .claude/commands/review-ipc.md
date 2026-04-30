---
description: FSW 전체의 Queue/Pipe IPC 메시지 라우팅 정합성을 검토한다
allowed-tools: Read Grep Glob
---

FSW의 메시지 라우팅 정합성을 검토하라.

## 검토 항목

### 1. MID 등록 vs 실제 사용 대조
- `lib/appargs.py`의 모든 MID를 목록화
- `send_msg(` 호출 시 사용된 MID와 대조
- 등록됐지만 사용 안 되는 MID 목록 출력
- 사용되지만 등록 안 된 MID (하드코딩) 목록 출력

### 2. 수신자 등록 확인
- `main.py`의 라우팅 딕셔너리에서 각 AppID가 Pipe를 갖는지 확인
- MID의 ReceiverAppID가 실제 라우팅에 포함됐는지 확인

### 3. 단방향 메시지 추적
각 MID에 대해 다음을 표로 출력:
```
MID | 발신 앱 | 수신 앱 | send_msg 파일:줄 | dispatch 파일:줄 | 상태
```
상태: OK / 미수신(dispatch 없음) / 미발신(send_msg 없음) / 라우팅누락

### 4. 라우팅 경로 검증
- `main.py`에서 `receiver_app` 기준 Pipe 분기 로직 확인
- 누락된 AppID Pipe 출력

## 출력 형식
문제 없는 항목은 생략하고 이상 항목만 심각도(높음/중간/낮음)와 함께 출력.
