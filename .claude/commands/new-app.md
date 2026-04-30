---
description: 새 FSW 센서/액추에이터 앱 모듈 보일러플레이트 생성
allowed-tools: Read Write Edit Bash
argument-hint: "<ModuleName> (예: Magnetometer)"
---

`$ARGUMENTS` 이름으로 새 FSW 앱 모듈을 생성하라.

## 생성할 파일 구조
```
Sensor_$ARGUMENTS/
├── <name>app.py   # 앱 메인 (dispatch, read loop, send_msg)
├── <name>.py      # 하드웨어 드라이버
└── README.md
```

## 반드시 따를 규칙

### 1. lib/appargs.py 에 클래스 추가
- 기존 AppID 최댓값 + 1 을 새 AppID로 사용 (현재 최대: MotorAppArg.AppID = 19)
- MID 패턴: `<AppID>01<DestAppID>` (예: AppID=20, Comm행 MID = 2001201)
- 추가할 MID: `MID_comm_<name>` (Comm 앱으로 텔레메트리)

### 2. app.py 패턴 (기존 앱과 동일한 구조 유지)
```python
def read_<name>_data(driver, shared): ...
def send_<name>_data(shared, main_queue, log_queue): ...
def <name>app_main(pipe, main_queue, log_queue): ...
```

### 3. send_msg 호출 형식
```python
from lib.msgstructure import send_msg, fill_msg
msg = fill_msg(AppArg.AppID, CommAppArg.AppID, AppArg.MID_comm_<name>, data)
send_msg(main_queue, msg)
```

### 4. I2C lock 사용
```python
import fcntl
LOCK_FILE = "/tmp/i2c-1.lock"
with open(LOCK_FILE, "w") as lf:
    fcntl.flock(lf, fcntl.LOCK_EX | fcntl.LOCK_NB)
    # I2C 읽기
    fcntl.flock(lf, fcntl.LOCK_UN)
```

### 5. main.py 에 프로세스 등록 안내
생성 후 main.py에 추가할 내용을 사용자에게 알려라:
- import 구문
- Process 생성 및 launcher 등록
