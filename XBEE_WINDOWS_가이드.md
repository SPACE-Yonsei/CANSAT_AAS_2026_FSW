# XBee Windows 통신 및 Serial Studio 설정 가이드

## 📋 목차
1. [개요](#개요)
2. [Raspberry Pi 실행 방법](#raspberry-pi-실행-방법)
3. [필수 준비물](#필수-준비물)
4. [XBee 페어링 (X-CTU 사용)](#xbee-페어링-x-ctu-사용)
5. [1단계: 하드웨어 연결](#1단계-하드웨어-연결)
6. [2단계: 소프트웨어 설치](#2단계-소프트웨어-설치)
7. [3단계: XBee 포트 확인](#3단계-xbee-포트-확인)
8. [4단계: Python 스크립트 실행](#4단계-python-스크립트-실행)
9. [5단계: Serial Studio 설정](#5단계-serial-studio-설정)
10. [6단계: 통신 테스트](#6단계-통신-테스트)
11. [명령어 사용법](#명령어-사용법)
12. [문제 해결](#문제-해결)

---

## 개요

이 가이드는 Windows PC에서 XBee를 통해 페이로드(Payload)로부터 센서 데이터를 수신하고, Serial Studio로 시각화하는 전체 과정을 설명합니다.

### 통신 구조
```
┌─────────────┐         ┌──────────────┐         ┌──────────────┐
│   Payload   │ ──XBee──│  Ground.py   │ ──TCP──│ Serial Studio│
│  (Raspberry │         │  (Windows)   │         │  (Windows)   │
│     Pi)     │         │              │         │              │
└─────────────┘         └──────────────┘         └──────────────┘
```

**동작 원리:**
1. Payload(Raspberry Pi)에서 XBee로 센서 데이터 전송
2. Windows의 `Ground.py`가 XBee로부터 데이터 수신
3. `Ground.py`가 TCP 서버로 데이터를 Serial Studio에 전달
4. Serial Studio에서 데이터 시각화

---

## Raspberry Pi 실행 방법

### 실행 파일
Raspberry Pi(Payload)에서는 **`main.py`** 파일을 실행합니다.

### 실행 방법

#### 방법 1: 직접 실행 (수동)
```bash
cd /home/pi/CANSAT_AAS_2026_FSW
source venv/bin/activate  # 가상환경 활성화
python3 main.py
```

#### 방법 2: startup.sh 스크립트 사용
```bash
cd /home/pi/CANSAT_AAS_2026_FSW
bash startup.sh
```

#### 방법 3: systemd 서비스로 자동 실행 (부팅 시 자동 시작)
```bash
# 서비스 활성화
sudo systemctl enable cansat-fsw.service
sudo systemctl start cansat-fsw.service

# 서비스 상태 확인
sudo systemctl status cansat-fsw.service

# 서비스 중지
sudo systemctl stop cansat-fsw.service

# 서비스 비활성화 (부팅 시 자동 시작 안 함)
sudo systemctl disable cansat-fsw.service
```

### 실행 전 확인사항

1. **설정 파일 확인**
   - `lib/config.txt` 파일이 존재하는지 확인
   - 설정 파일이 없으면 오류가 발생합니다:
     ```
     Config file does not exist: lib/config.txt, Configure the config file to run FSW!
     ```

2. **가상환경 활성화**
   - 프로젝트는 가상환경(`venv`)을 사용합니다
   - 실행 전에 가상환경을 활성화해야 합니다

3. **필수 서비스 실행**
   - `pigpiod` 서비스가 실행 중이어야 합니다:
     ```bash
     sudo systemctl status pigpiod
     ```

### 실행 중인 앱들
`main.py`는 다음 앱들을 멀티프로세스로 실행합니다:
- **commapp**: XBee 통신 (텔레메트리 전송 및 명령 수신)
- **barometerapp**: 기압계 센서
- **gpsapp**: GPS 센서
- **imuapp**: IMU 센서
- **electroapp**: 전압/전류 센서
- **cameraapp**: 카메라
- **motorapp**: 모터 제어
- **distanceapp**: 거리 센서 (TF-Luna)
- **flightlogicapp**: 비행 로직 및 상태 관리
- **hkapp**: 하우스키핑

### 종료 방법
- **Ctrl + C**: 정상 종료 (모든 프로세스가 정리됩니다)
- **강제 종료**: `Ctrl + C`를 두 번 누르면 강제 종료됩니다

### 로그 확인
실행 중인 로그는 콘솔에 표시되며, systemd 서비스로 실행하는 경우:
```bash
sudo journalctl -u cansat-fsw.service -f
```

---

## 필수 준비물

### 하드웨어
- ✅ XBee 모듈 (Coordinator 모드 - 지상국용)
- ✅ XBee USB 어댑터 (XBee Explorer 등)
- ✅ USB 케이블
- ✅ Windows PC

### 소프트웨어
- ✅ Python 3.7 이상
- ✅ pyserial 라이브러리
- ✅ Serial Studio (https://serial-studio.github.io/)
- ✅ X-CTU (XBee 설정 도구, https://www.digi.com/support/product-detection-tools/xctu)

---

## XBee 페어링 (X-CTU 사용)

### X-CTU 설치
1. Digi 공식 사이트에서 X-CTU 다운로드: https://www.digi.com/support/product-detection-tools/xctu
2. 설치 파일 실행하여 설치

### XBee 페어링 절차

#### 1단계: 첫 번째 XBee 설정 (Payload용 - Router 모드)

1. **XBee를 USB 어댑터에 연결하고 PC에 연결**
2. **X-CTU 실행**
3. **포트 선택**
   - 상단에서 XBee가 연결된 COM 포트 선택
   - Baud Rate: `9600` 선택
   - **"Test/Query" 버튼 클릭**하여 연결 확인
   - 연결 성공 시 "OK" 메시지 표시
4. **"Modem Configuration" 탭 클릭**
5. **"Read" 버튼 클릭**하여 현재 설정 읽기
6. **중요 설정 변경:**
   - **CE (Coordinator Enable)**: `0` (Router 모드)
   - **ID (PAN ID)**: 원하는 값 입력 (예: `2026` 또는 `3139`)
   - **DL (Destination Address Low)**: 나중에 설정 (Coordinator의 MY 주소)
   - **MY (16-bit Source Address)**: 고유 주소 (예: `1`)
   - **BD (Interface Data Rate)**: `3` (9600 baud)
7. **"Write" 버튼 클릭**하여 설정 저장
8. **SH (Serial Number High)와 SL (Serial Number Low) 기록** (다른 XBee 설정에 필요)

#### 2단계: 두 번째 XBee 설정 (Ground Station용 - Coordinator 모드)

1. **다른 XBee를 USB 어댑터에 연결하고 PC에 연결**
2. **X-CTU에서 새로운 포트 선택**
3. **"Test/Query" 버튼 클릭**하여 연결 확인
4. **"Read" 버튼 클릭**
5. **중요 설정 변경:**
   - **CE (Coordinator Enable)**: `1` (Coordinator 모드)
   - **ID (PAN ID)**: 첫 번째 XBee와 **동일한 값** (예: `2026`)
   - **DL (Destination Address Low)**: 첫 번째 XBee의 MY 주소 (예: `1`)
   - **MY (16-bit Source Address)**: 고유 주소 (예: `2`)
   - **BD (Interface Data Rate)**: `3` (9600 baud)
6. **"Write" 버튼 클릭**하여 설정 저장

#### 3단계: Router XBee의 DL 설정 업데이트

1. **첫 번째 XBee (Router)로 다시 전환**
2. **"Read" 버튼 클릭**
3. **DL (Destination Address Low)**: Coordinator의 MY 주소 (예: `2`)로 변경
4. **"Write" 버튼 클릭**하여 설정 저장

#### 4단계: 페어링 확인

1. **두 XBee 모두 연결 상태 유지**
2. **X-CTU의 "Terminal" 탭에서 테스트:**
   - 한쪽 XBee에서 "Hello" 입력 후 전송
   - 다른 쪽 XBee에서 수신되는지 확인
3. **LED 확인:**
   - 데이터 전송 시 XBee LED가 깜빡여야 함

### XBee 설정 요약

| 설정 항목 | Payload (Router) | Ground Station (Coordinator) |
|-----------|------------------|------------------------------|
| **CE** | `0` (Router) | `1` (Coordinator) |
| **ID (PAN ID)** | 동일한 값 (예: `2026`) | 동일한 값 (예: `2026`) |
| **MY** | `1` | `2` |
| **DL** | `2` (Coordinator의 MY) | `1` (Router의 MY) |
| **BD** | `3` (9600) | `3` (9600) |

### 주의사항
- ⚠️ **PAN ID는 반드시 동일해야 함** (가장 중요!)
- ⚠️ **DL (Destination Address)는 상대방의 MY 주소**
- ⚠️ 설정 변경 후 **Write 버튼을 반드시 클릭**해야 저장됨
- ⚠️ XBee를 재부팅하면 설정이 적용됨
- ⚠️ 두 XBee의 BD (Baud Rate)가 동일해야 함

### 대안: 자동 페어링 (AT 모드)
일부 XBee 모듈은 자동으로 네트워크를 형성할 수 있습니다:
- Coordinator: CE = 1, ID = 동일
- Router: CE = 0, ID = 동일
- DL을 설정하지 않으면 브로드캐스트로 통신 (모든 XBee가 수신)

### 문제 해결
- **연결이 안 될 때**: Test/Query 버튼으로 연결 확인
- **설정이 저장되지 않을 때**: Write 버튼 클릭 후 XBee 재부팅
- **통신이 안 될 때**: PAN ID가 동일한지 확인

---

## 1단계: 하드웨어 연결

### XBee 모듈 연결
1. **XBee 모듈을 USB 어댑터에 장착**
   - XBee의 핀을 어댑터에 정확히 맞춰 삽입
   - ⚠️ 방향 확인 필수 (잘못된 방향은 손상 위험)

2. **USB 케이블로 PC에 연결**
   - USB 어댑터를 USB 케이블로 PC에 연결
   - Windows가 드라이버를 자동 설치하는지 확인

3. **XBee 설정 확인**
   - Coordinator 모드: 지상국(Ground Station)용
   - Router 모드: 페이로드(Payload)용
   - 두 XBee가 같은 PAN ID를 사용해야 함

---

## 2단계: 소프트웨어 설치

### Python 설치 확인
```powershell
python --version
```
Python 3.7 이상이 필요합니다. 없으면 [python.org](https://www.python.org/)에서 다운로드

### pyserial 설치
```powershell
pip install pyserial
```

### Serial Studio 설치
1. https://serial-studio.github.io/ 접속
2. Windows 버전 다운로드
3. 설치 파일 실행하여 설치

---

## 3단계: XBee 포트 확인

### 방법 1: Windows 장치 관리자
1. `Win + X` 키 누르기
2. "장치 관리자" 선택
3. "포트(COM 및 LPT)" 섹션 확장
4. "USB Serial Port (COMx)" 또는 "Silicon Labs CP210x USB to UART Bridge (COMx)" 확인
5. **COM 번호 기록** (예: COM3, COM6, COM7)

### 방법 2: Python으로 확인
```powershell
python -c "import serial.tools.list_ports; ports = serial.tools.list_ports.comports(); [print(f'{i}: {p.device}') for i, p in enumerate(ports)]"
```

---

## 4단계: Python 스크립트 실행

### 스크립트 위치
```
Sensor_Gcs/
  └── scripts/
      ├── Ground.py              # 메인 스크립트
      └── Command_function.py    # 명령어 함수
```

### 실행 방법

1. **PowerShell 또는 명령 프롬프트 열기**

2. **프로젝트 디렉토리로 이동**
```powershell
cd "C:\Users\bagj6\OneDrive - 충남중학교\연세대학교\06_동아리\스페이스Y\2026 AAS CANSAT\CANSAT_AAS_2026_FSW-2\Sensor_Gcs\scripts"
```

3. **Ground.py 실행**
```powershell
python Ground.py
```

4. **포트 선택**
   - 사용 가능한 포트 목록이 표시됩니다:
   ```
   사용 가능한 포트 목록:
     0: COM6
     1: COM5
   
   포트 번호를 선택하세요 (0-1): 
   ```
   - XBee가 연결된 포트의 **인덱스 번호**를 입력합니다
   - 예: COM6을 선택하려면 `0` 입력, COM5를 선택하려면 `1` 입력
   - ⚠️ 잘못된 번호를 입력하면 다시 입력하라는 메시지가 표시됩니다

5. **TCP 서버 시작 확인**
   - 다음 메시지가 표시되면 성공:
   ```
   선택된 포트: COM6
   
   TCP server started at 127.0.0.1:12345. Waiting for Serial Studio...
   Command List: ['CX', 'ST', 'SIM', 'SIMP', 'CAL', 'MEC']
   Enter command to send to payload:
   ```

### ⚠️ 주의사항
- `Ground.py`는 Serial Studio가 연결될 때까지 대기합니다
- Serial Studio를 먼저 실행하고 연결해야 합니다 (다음 단계 참고)

---

## 5단계: Serial Studio 설정

### 1. Serial Studio 실행
- Serial Studio를 실행합니다

### 2. Setup 탭 열기
- 상단 툴바에서 **"Setup"** 버튼 클릭 (톱니바퀴 아이콘)
- 오른쪽에 Setup 패널이 표시됩니다

### 3. I/O Interface 설정 변경
1. **DEVICE SETUP** 섹션에서 **"I/O Interface:"** 드롭다운 메뉴 클릭
2. 기본값은 **"Serial Port"**로 되어 있습니다
3. **"Network"** 선택

### 4. Network 설정
I/O Interface를 Network로 변경하면 새로운 설정 옵션이 나타납니다:

1. **Protocol**: 드롭다운에서 **"TCP Client"** 선택
2. **Host**: `127.0.0.1` 입력 (또는 `localhost`)
3. **Port**: `12345` 입력
4. **Connect** 버튼 클릭 (상단 툴바 오른쪽에 있는 플러그 아이콘)

### 5. 연결 확인
- Serial Studio가 연결되면 `Ground.py` 콘솔에 다음 메시지가 표시됩니다:
  ```
  Serial Studio connected from ('127.0.0.1', xxxxx)
  ```
- Serial Studio의 Console 탭에서 연결 상태를 확인할 수 있습니다

### 6. 데이터 포맷 설정

#### Quick Plot 모드 사용 (CSV 데이터)
1. **FRAME PARSING** 섹션에서 **"Quick Plot (Comma Separated Values)"** 라디오 버튼 선택
   - 이 모드는 CSV 형식의 데이터를 자동으로 파싱합니다
2. 데이터가 수신되면 자동으로 그래프가 표시됩니다

#### JSON 프로젝트 파일 사용 (고급)
프로젝트에 JSON 설정 파일이 있다면:
1. **"Parse via JSON Project File"** 라디오 버튼 선택
2. **"Change Project File"** 버튼 클릭
3. `Sensor_Gcs/2025 AAS CANSAT.json` 파일 선택
4. 이 파일에는 데이터 필드 정의와 위젯 설정이 포함되어 있습니다

### 7. 데이터 확인

#### 예시 데이터 형식
페이로드에서 전송되는 텔레메트리 데이터는 다음과 같은 형식입니다:
```
1070,12:34:56,1234,F,LAUNCH_PAD,123.45,25.3,101.32,3.7,0.5,1.85,...
```

**필드 설명:**
- `1070`: Team ID
- `12:34:56`: Mission Time
- `1234`: Packet Count
- `F`: Mode (F=Flight, S=Simulation)
- `LAUNCH_PAD`: State
- `123.45`: Altitude (m)
- `25.3`: Temperature (°C)
- `101.32`: Pressure (kPa)
- `3.7`: Voltage (V)
- `0.5`: Current (A)
- `1.85`: Power (W)
- ... (기타 센서 데이터)

#### 데이터 시각화
- **Dashboard 탭**: 상단 툴바에서 "Dashboard" 버튼 클릭
- **Widgets 탭**: 개별 위젯으로 데이터 확인
- 데이터가 수신되면 자동으로 그래프가 업데이트됩니다

### ⚠️ 중요 사항
- **연결 순서**: `Ground.py`를 먼저 실행한 후 Serial Studio에서 연결해야 합니다
- **포트 번호**: 반드시 `12345`로 설정
- **Host**: `127.0.0.1` 또는 `localhost` 사용
- I/O Interface를 **"Network"**로 변경하지 않으면 TCP 연결이 불가능합니다

---

## 6단계: 통신 테스트

### 체크리스트
- [ ] XBee가 USB 어댑터에 올바르게 연결됨
- [ ] Windows에서 COM 포트 인식 확인
- [ ] `Ground.py` 스크립트 실행
- [ ] 올바른 COM 포트 선택
- [ ] TCP 서버 시작 메시지 확인
- [ ] Serial Studio 실행
- [ ] Serial Studio에서 TCP Client로 연결
- [ ] Host: `127.0.0.1`, Port: `12345` 설정
- [ ] Connect 버튼 클릭
- [ ] "Serial Studio connected" 메시지 확인
- [ ] Serial Studio에서 데이터 수신 확인

### 데이터 수신 확인
- Serial Studio의 대시보드에서 실시간 데이터가 표시되는지 확인
- 그래프, 게이지 등 위젯이 업데이트되는지 확인
- `Ground.py` 콘솔에서 에러 메시지가 없는지 확인

---

## 명령어 사용법

`Ground.py`를 실행하면 명령어를 입력할 수 있습니다. 사용 가능한 명령어:

### CX - 텔레메트리 ON/OFF
```
Enter command to send to payload: CX
Input the ON/OFF : ON
```
- `ON`: 텔레메트리 전송 시작
- `OFF`: 텔레메트리 전송 중지

### ST - 시간 설정
```
Enter command to send to payload: ST
UTC : put the time that you want (hh:mm:ss)
GPS : put just 'GPS'
Input How to set the time : 12:00:00
```
- 시간 형식: `HH:MM:SS` (예: `12:00:00`)
- 또는 `GPS` 입력 시 GPS 시간 사용

### SIM - 시뮬레이션 모드 제어
```
Enter command to send to payload: SIM
Input the ENABLE(E), ACTIVATE(A), or DISABLE(D) : E
```
- `ENABLE` 또는 `E`: 시뮬레이션 모드 활성화
- `ACTIVATE` 또는 `A`: 시뮬레이션 데이터 전송 시작 (CSV 파일 필요)
- `DISABLE` 또는 `D`: 시뮬레이션 모드 비활성화

### CAL - 고도 보정
```
Enter command to send to payload: CAL
```
- 현재 고도를 0m로 보정

### MEC - 메커니즘 제어
```
Enter command to send to payload: MEC
Input Device (MOTOR, CAMERA) : MOTOR
Input the ON/OFF : ON
```
- `MOTOR`: 모터 제어
- `CAMERA`: 카메라 제어
- `ON`/`OFF`: 작동/정지

---

## 문제 해결

### ❌ 문제 1: COM 포트를 찾을 수 없음

**증상:**
```
SerialException: could not open port 'COM6': [Error 2] The system cannot find the file specified.
```

**해결 방법:**
1. 장치 관리자에서 COM 포트 확인
2. 다른 프로그램이 포트를 사용 중인지 확인
3. USB 케이블 재연결
4. XBee 어댑터 재연결
5. PC 재부팅

---

### ❌ 문제 2: TCP 연결 실패

**증상:** Serial Studio에서 연결 실패

**해결 방법:**
1. `Ground.py`가 실행 중인지 확인
2. 포트 번호 확인 (12345)
3. 다른 프로그램이 12345 포트를 사용 중인지 확인:
   ```powershell
   netstat -ano | findstr :12345
   ```
4. Windows 방화벽 설정 확인
5. `Ground.py`를 재시작

---

### ❌ 문제 3: 데이터가 수신되지 않음 (XBee 페어링 문제)

**증상:** Serial Studio에 데이터가 표시되지 않음, XBee LED가 깜빡이지 않음

**원인:** 두 XBee 모듈이 페어링되지 않았거나 설정이 일치하지 않음

**해결 방법: X-CTU로 XBee 페어링**

👉 **상세한 페어링 절차는 위의 [XBee 페어링 (X-CTU 사용)](#xbee-페어링-x-ctu-사용) 섹션을 참고하세요.**

**빠른 체크리스트:**
1. 두 XBee의 **PAN ID가 동일한지** 확인 (가장 중요!)
2. Coordinator: CE = 1, Router: CE = 0
3. DL은 상대방의 MY 주소로 설정
4. X-CTU의 Terminal 탭에서 통신 테스트

**4. Payload 확인**
   - Payload에서 데이터를 전송 중인지 확인
   - Payload의 XBee LED가 깜빡이는지 확인 (데이터 전송 시)

**5. Ground.py 콘솔 확인**
   - 에러 메시지가 있는지 확인
   - `[Sensor Thread] Error:` 메시지 확인

**6. 연결 순서 확인**
   - 올바른 순서: `Ground.py` 실행 → Serial Studio 연결

---

### ❌ 문제 4: 데이터 형식 오류

**증상:** Serial Studio에서 데이터 파싱 실패

**해결 방법:**
1. 데이터 형식 확인 (CSV 형식인지)
2. Serial Studio의 데이터 포맷 설정 확인
3. 필드 구분자 확인 (쉼표 `,`)
4. 첫 번째 줄이 헤더인지 확인

---

### ❌ 문제 5: 명령어가 전송되지 않음

**증상:** 명령어를 입력했지만 Payload에서 반응 없음

**해결 방법:**
1. XBee 연결 확인
2. 명령어 형식 확인 (예: `CMD,3139,CX,ON`)
3. Team ID 확인 (기본값: 3139)
4. Payload의 명령어 수신 로그 확인

---

### ❌ 문제 6: 포트 선택 오류 (IndexError)

**증상:**
```
IndexError: list index out of range
```

**원인:** 포트 목록에 없는 인덱스 번호를 입력한 경우

**해결 방법:**
1. 포트 목록을 다시 확인합니다
2. **인덱스는 0부터 시작**합니다
   - 예: 포트 목록이 `['COM6', 'COM5']`인 경우
   - COM6을 선택하려면: `0` 입력
   - COM5를 선택하려면: `1` 입력
   - ❌ `5`나 `6`을 입력하면 안 됩니다!
3. 수정된 `Ground.py`는 잘못된 입력 시 다시 입력하도록 안내합니다

---

### ❌ 문제 7: Serial Studio에서 연결 실패

**증상:** Serial Studio에서 Connect 버튼을 눌러도 연결되지 않음

**해결 방법:**
1. **I/O Interface 확인**
   - Setup 탭에서 **"I/O Interface"**가 **"Network"**로 설정되어 있는지 확인
   - ❌ "Serial Port"로 되어 있으면 TCP 연결이 불가능합니다

2. **Ground.py 실행 확인**
   - `Ground.py`가 실행 중이고 "Waiting for Serial Studio..." 메시지가 표시되는지 확인
   - `Ground.py`를 먼저 실행한 후 Serial Studio에서 연결해야 합니다

3. **Host와 Port 확인**
   - Host: `127.0.0.1` 또는 `localhost`
   - Port: `12345` (정확히 일치해야 함)

4. **방화벽 확인**
   - Windows 방화벽이 12345 포트를 차단하지 않는지 확인
   - 로컬호스트(127.0.0.1)는 일반적으로 문제없지만, 필요시 방화벽 예외 추가

5. **포트 사용 중 확인**
   - 다른 프로그램이 12345 포트를 사용 중인지 확인:
     ```powershell
     netstat -ano | findstr :12345
     ```

---

### ❌ 문제 8: Serial Studio에서 데이터가 표시되지 않음

**증상:** 연결은 되었지만 데이터가 보이지 않음

**해결 방법:**
1. **Frame Parsing 설정 확인**
   - Setup 탭의 **"FRAME PARSING"** 섹션 확인
   - CSV 데이터인 경우: **"Quick Plot (Comma Separated Values)"** 선택
   - JSON 프로젝트 파일 사용 시: **"Parse via JSON Project File"** 선택

2. **Ground.py 콘솔 확인**
   - XBee로부터 데이터를 수신하고 있는지 확인
   - 에러 메시지가 있는지 확인

3. **Dashboard 탭 확인**
   - 상단 툴바에서 **"Dashboard"** 버튼 클릭
   - 데이터가 Dashboard에 표시되는지 확인

4. **Console 탭 확인**
   - 상단 툴바에서 **"Console"** 버튼 클릭
   - 원시 데이터가 표시되는지 확인

---

## 추가 팁

### 로그 파일 저장
데이터를 파일로 저장하려면 `Ground.py`의 `sensor_data_receiver` 함수를 수정:

```python
def sensor_data_receiver(xbee_serial, client_socket):
    log_file = open('xbee_log.csv', 'a', encoding='utf-8')
    while True:
        try:
            if xbee_serial.in_waiting:
                data = xbee_serial.readline().decode('utf-8').strip()
                log_file.write(data + '\n')
                log_file.flush()  # 즉시 파일에 쓰기
                client_socket.sendall((data + '\n').encode('utf-8'))
        except Exception as e:
            print(f"[Sensor Thread] Error: {e}")
            break
    log_file.close()
```

### 다른 포트 번호 사용
`Ground.py`에서 TCP 포트를 변경하려면:
```python
TCP_PORT = 12345  # 원하는 포트 번호로 변경
```

### 다른 보드레이트 사용
XBee 기본 보드레이트는 9600입니다. 변경하려면:
```python
XBEE_BAUDRATE = 9600  # 115200, 57600 등으로 변경 가능
```

---

## 요약

### 전체 프로세스 요약

#### Raspberry Pi (Payload) 측
1. **프로젝트 디렉토리로 이동**
   ```bash
   cd /home/pi/CANSAT_AAS_2026_FSW
   ```

2. **가상환경 활성화 및 실행**
   ```bash
   source venv/bin/activate
   python3 main.py
   ```
   또는
   ```bash
   bash startup.sh
   ```

3. **확인사항**
   - `lib/config.txt` 파일 존재 확인
   - `pigpiod` 서비스 실행 중 확인
   - XBee가 `/dev/serial0`에 연결되어 있는지 확인

#### Windows (Ground Station) 측

1. **하드웨어 연결**
   - XBee를 USB 어댑터에 연결
   - PC에 USB 케이블로 연결

2. **포트 확인**
   - 장치 관리자에서 COM 포트 확인

3. **스크립트 실행**
   ```powershell
   cd Sensor_Gcs\scripts
   python Ground.py
   ```
   - 포트 번호 선택

4. **Serial Studio 설정 및 연결**
   - Serial Studio 실행
   - Setup 탭 열기
   - I/O Interface: **"Network"** 선택
   - Protocol: **"TCP Client"** 선택
   - Host: `127.0.0.1`, Port: `12345`
   - Connect 버튼 클릭
   - Frame Parsing: **"Quick Plot (Comma Separated Values)"** 선택

5. **데이터 확인**
   - Serial Studio의 Dashboard에서 실시간 데이터 확인
   - 데이터가 자동으로 그래프로 표시됩니다

---

## 참고 자료

### 프로젝트 파일
- `Sensor_Gcs/scripts/Ground.py` - Windows용 GCS 스크립트
- `Sensor_Gcs/scripts/Command_function.py` - 명령어 함수 정의
- `comm/commapp.py` - Payload의 통신 앱 (Raspberry Pi)

### 유용한 링크
- Serial Studio: https://serial-studio.github.io/
- pyserial 문서: https://pyserial.readthedocs.io/
- XBee 설정: Digi XBee 공식 문서

---

**이제 Windows에서 XBee 통신을 받고 Serial Studio로 시각화할 수 있습니다!** 🚀
