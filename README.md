# CANSAT AAS 2026 FSW

Python multiprocessing flight software for Yonsei Space-Y CANSAT.

## Documentation

- Korean FSW architecture/operation guide: `docs/FSW_설명서.md`

## Current Architecture

- `main.py`: process orchestration, queue/pipe routing, restart/terminate handling
- IPC bus: `sender|receiver|MsgID|data`
- Core apps: Comm, FlightLogic, Motor, Barometer, IMU, GPS, Distance, Electro, Camera
- Unit + smoke tests are in `tests/`

## Environment Setup (Raspberry Pi)

### 1) Base packages

```bash
sudo apt update
sudo apt install -y git python3 python3-venv python3-dev
```

### 2) Python venv

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
```

### 3) Sensor/IO dependencies

```bash
pip install adafruit-circuitpython-bmp3xx
pip install adafruit-circuitpython-gps
pip install adafruit-circuitpython-bno08x
pip install adafruit-circuitpython-ina228
pip install adafruit-extended-bus
pip install pigpio
```

### 4) Camera dependencies

```bash
sudo apt install -y python3-picamera2 libcamera-apps libcamera-tools
```

## Camera Configuration (Important)

The current implementation uses `Sensor_Camera/picam.py` and `Sensor_Camera/cameraapp.py`.

### Hardware config example (Pi Cam v3 / IMX708)

Edit `/boot/firmware/config.txt` (path may differ by OS image):

```ini
camera_auto_detect=0
[all]
dtoverlay=imx708
```

Then reboot.

### Runtime camera behavior

- Camera init tries `picamera2` first.
- **Recording starts automatically** when the Camera app starts (i.e. when you run `python3 main.py`). No uplink command is required for the first segments.
- Recording config (current code):
  - video format: `RGB888`
  - resolution: `640x480`
  - segmented recording via `cameraapp` (`SEGMENT_SEC=7.0`)
  - each segment is raw **H.264** elementary stream (`.h264`) via `FileOutput` (no MP4 mux per segment)
- Output directory: `PICAM_Video/` (under the FSW working directory)
- Output filename format: `P_MMDD_HHMMSS_microsec.h264` (or placeholder `.txt` if the backend fails). Remux locally if needed: `ffmpeg -i seg.h264 -c copy seg.mp4`
- **강제 종료 시 영상 보존**:
  - 이미 완료된 세그먼트(7초 단위)는 안전하게 저장됨
  - `SIGTERM` (일반 종료): `finally` 블록에서 `cam.close()` 호출 → 현재 세그먼트 저장 가능
  - `SIGKILL` (강제 종료): 현재 녹화 중인 세그먼트만 손실 (최대 7초)
  - 불완전한 `.h264` 파일도 ffmpeg으로 복구 가능: `ffmpeg -i incomplete.h264 -c copy recovered.mp4`
- If camera backend is unavailable:
  - system falls back gracefully
  - placeholder segment files are created so pipeline/test does not break

### Camera command path

- `CMD,1070,CAM,OFF` -> stop recording (after boot auto-start)
- `CMD,1070,CAM,ON` -> start or resume recording (if you stopped with `CAM,OFF`)
- FlightLogic entering ASCENT (state 1) sends `MID_cam_activate` to the Camera app as an additional start trigger (harmless if already recording)

## Security Notes (Comm)

`RBT` command now supports token + anti-replay checks.

- supported auth payload formats:
  - `token`
  - `token,seq`
  - `token,seq,nonce`
  - `token:seq:nonce`
- env vars:
  - `RBT_AUTH_TOKEN` (required)
  - `RBT_REQUIRE_SEQ` (`1` default)
  - `RBT_SAFE_STATES` (`0,5` default)

Example:

```bash
export RBT_AUTH_TOKEN="YOUR_SECRET"
export RBT_REQUIRE_SEQ=1
export RBT_SAFE_STATES="0,5"
```

## Run

### Local run

```bash
source .venv/bin/activate
python main.py
```

### systemd run

Use included scripts:

```bash
chmod +x startup.sh setup_systemd_service.sh
./setup_systemd_service.sh
s
sudo systemctl status cansat-fsw.service
```

## 발사 전 절차 (Pre-Launch Checklist)

### XBee 사전 설정 (XCTU)
- PAN ID = `1070`
- 브로드캐스트 금지: GCS XBee MAC 주소를 페이로드 XBee의 DH/DL에 설정 (유니캐스트)

### 발사대 설치 후 순서

```
① 캔샛 전원 ON
   └─ 카메라 녹화 자동 시작 (FSW 부팅과 동시)

② CMD,1070,CX,ON          — 텔레메트리 활성화, 패킷 흐름 확인

③ CMD,1070,ST,GPS          — GPS로 UTC 시간 동기화
   └─ GPS 수신 불가 시: CMD,1070,ST,HH:MM:SS

④ CMD,1070,TC,<lat>,<lon>  — 대회장 타겟 좌표 입력 (prevstate에 저장됨)

⑤ CMD,1070,CAL             — ★ 발사 직전 마지막 명령
   └─ 고도 → 0m 영점 (발사대 기준)
   └─ PACKET_COUNT → 0 리셋
   └─ prevstate 동기화 (비행 중 리셋 복구 기준점 확정)

⑥ 발사
```

> **주의:** CAL 이후 추가 명령을 보내면 패킷 카운트가 올라간 채로 비행이 시작됩니다.

---

## 커맨드 목록 (Command Reference)

모든 명령 형식: `CMD,1070,<CMD>,<OPTION>` (팀 ID = 1070)

### 필수 명령 (AAS 스펙 3.1.2)

| 커맨드 | 형식 | 기능 | CMD_ECHO 예시 |
|---|---|---|---|
| **CX** | `CX,ON\|OFF` | 텔레메트리 ON/OFF | `CXON` / `CXOFF` |
| **ST** | `ST,GPS\|HH:MM:SS` | 미션 시간 설정 | `STGPS` / `ST13:35:59` |
| **SIM** | `SIM,ENABLE\|ACTIVATE\|DISABLE` | 시뮬레이션 모드 제어 | `SIMENABLE` 등 |
| **SIMP** | `SIMP,<Pa>` | 시뮬 기압 주입 (해수면 절대 기압, SIM ACTIVATE 이후에만 유효) | `SIMP101325` |
| **CAL** | `CAL` | 고도 영점 + PACKET_COUNT 0 리셋 | `CAL` |
| **MEC** | `MEC,<DEVICE>,ON\|OFF` | 메커니즘 작동 | `MECMOTORON` |

**MEC DEVICE 정의:**

| DEVICE | 대상 |
|---|---|
| `MOTOR` | 모터(서보) 활성/비활성 |
| `RELEASE` | 릴리즈 릴레이 |
| `EGG` | 에그 솔레노이드 |

### 팀 선택 명령 (Optional)

| 커맨드 | 형식 | 기능 |
|---|---|---|
| **TC** | `TC,<lat>,<lon>` | 타겟 GPS 좌표 설정 (prevstate 저장, 리셋 복구) |
| **SS** | `SS,0~6` | 비행 상태 강제 점프 (0=LAUNCH_PAD … 6=LANDED) |
| **FAC** | `FAC,REL\|EGG,ON\|OFF` | 릴리즈/에그 직접 제어 |
| **MTR** | `MTR,LEFT\|NEUTRAL\|RIGHT` | 서보 수동 조향 |
| **CMC** | `CMC,GPS_GUIDED\|GPS_ONLY\|IMU_HEADING` | 제어 모드 전환 |
| **CAM** | `CAM,ON\|OFF` | 카메라 수동 제어 |
| **RBT** | `RBT,<token>[,<seq>[,<nonce>]]` | 인증된 시스템 리부팅 |
| **XRST** | `XRST,NOW` | XBee 하드웨어 리셋 |
| **SIMG** | `SIMG,<lat>,<lon>,<course°>,<speed_m/s>[,<alt_m>]` | SIM GPS 주입 |
| **SIMGR** | `SIMGR,<east_m>,<north_m>,<course°>,<speed>[,<alt>]` | SIM GPS 타겟 기준 상대좌표 주입 |
| **SIMGN** | `SIMGN` | SIM GPS 강제 stale |

### CMD_ECHO 규칙

CMD_ECHO = 커맨드명 + 옵션을 **콤마 없이** 붙여쓴 값. TLM 패킷이 CSV이므로 콤마 금지.

| 수신 | CMD_ECHO |
|---|---|
| `CMD,1070,CX,ON` | `CXON` |
| `CMD,1070,SIMP,101325` | `SIMP101325` |
| `CMD,1070,MEC,MOTOR,ON` | `MECMOTORON` |
| `CMD,1070,CAL` | `CAL` |
| `CMD,1070,ST,13:35:59` | `ST13:35:59` |

---

## 시뮬레이션 모드 운용

```
① CMD,1070,CX,ON
② CMD,1070,ST,GPS
③ CMD,1070,TC,<lat>,<lon>
④ CMD,1070,CAL              ← 반드시 CAL 먼저 (절대고도 → 상대고도 변환 기준)
⑤ CMD,1070,SIM,ENABLE
⑥ CMD,1070,SIM,ACTIVATE
⑦ Serial Studio 또는 GCS에서 SIMP 파일 1Hz 재생
⑧ (완료 후) CMD,1070,SIM,DISABLE
```

> **주의:** CAL 없이 SIMP를 사용하면 SIMP 값(해수면 절대기압)이 발사대 오프셋 보정 없이 그대로 고도로 해석됩니다. 해발 500m 대회장에서 CAL 미실행 시 SIMP 최초값이 ~500m로 해석되어 즉시 ASCENT 상태로 진입합니다.

---

## Test

### Full test suite

```bash
python -m unittest
```

### Main process smoke test

```bash
python -m unittest tests/test_main_smoke.py
```

### Sensor driver CLI (repo root, stream until Ctrl+C)

Default rate is **10 Hz**; override with `SENSOR_CLI_HZ` (e.g. `5` for 5 Hz). Folder name is **`Sensor_Imu`** (capital **I**), not `Sensor_IMU`. **IMU CLI** prints a line immediately, then may sit **15–60 s** during BNO08x init (I2C lock if `main.py` or another sensor CLI is running).

```bash
export FSW_I2C_BUS=1   # on Pi if needed
export SENSOR_CLI_HZ=10

python3 -m Sensor_Barometer.barometer
python3 -m Sensor_Imu.imu
python3 -m Sensor_Electro.electro
python3 -m Sensor_Distance.distance
# GPS: I2C u-blox DDC only
python3 -m Sensor_Gps.gps
```

## Optional Permissions (non-root user)

```bash
sudo usermod -aG i2c $USER
sudo usermod -aG gpio $USER
sudo usermod -aG video $USER
```

## Notes

- Current app layer is operational and tested.
- Some low-level hardware driver modules still include shim/fallback paths for non-hardware development.
- See `.claude/docs/baseline/fsw-baseline.md` and `.claude/docs/audits/gap-audit-current-baseline.md` for phased implementation/audit status.

## Troubleshooting

### 1) `git pull` fails due to `lib/prevstate.json`

`lib/prevstate.json` is a runtime state file and should not be version-controlled.
If your older branch still tracks it, run:

```bash
git rm --cached lib/prevstate.json
echo "lib/prevstate.json" >> .gitignore
git add .gitignore
```

Then commit once and retry pull/merge.

### 2) Ctrl+C shutdown prints many child tracebacks

If you still see repeated `KeyboardInterrupt` stack traces in subprocesses,
you are likely running older app modules. Update to the latest app code,
which includes graceful `pipe.poll()` shutdown guards in all app runloops.

### 3) `receive_serial_data failed: 'NoneType' object cannot be interpreted as an integer`

This is a serial backend edge case seen during shutdown on some environments.
Latest `comm/uartserial.py` treats this transient case as "no data" to reduce log noise.

### 4) TLM is too noisy/quiet in console

- Default is now console ON (`FSW_LOG_TLM=1` behavior).
- To silence local TLM prints while keeping UART transmit, run:

```bash
FSW_LOG_TLM=0 python3 main.py
```

### 5) Real sensors vs synthetic data

- **Barometer**: **BMP3xx only** (e.g. **BMP390**) via `Sensor_Barometer/barometer.py`, I2C `BARO_I2C_ADDR` (default `0x77`), package `adafruit-circuitpython-bmp3xx`. On failure the app logs and falls back to synthetic data.
- **IMU**: uses `Sensor_Imu/imu.py` (BNO08x, default `0x4A`) when `adafruit-circuitpython-bno08x` works.
- **GPS**: `Sensor_Gps/gps.py` uses **u-blox DDC I2C** (NMEA/UBX over I2C), e.g. **MikroE GNSS 7 Click (NEO-M9N)** at **`0x42`**. Flight **UART** is reserved for **XBee/comm**; GNSS is on the same I2C bus as baro / IMU / etc. (`FSW_I2C_BUS`, `GPS_I2C_ADDR`).
  - **INA228** is **not** at `0x42`; keep `ELECTRO_I2C_ADDR=0x40` unless your shunt board uses another address.
- **IMU**: Adafruit BNO08x low-level **packet debug prints** are silenced by default. To turn them back on for driver bring-up: `BNO08X_DEBUG=1 python3 main.py`.
- **Distance**: **Benewake TF-Luna I2C** via `Sensor_Distance/distance.py` (default address **`0x10`**). Extra pip package not required. Env: `DISTANCE_I2C_ADDR`, `DISTANCE_TFL_TRIGGER` (default `1`; set `0` for continuous mode per Benewake docs).
- **Power**: **INA228** only via `Sensor_Electro/electro.py`, `ELECTRO_I2C_ADDR` (default `0x40`), package `adafruit-circuitpython-ina228`.
- Env hints: `BARO_I2C_ADDR`, `BARO_INIT_RETRIES`, `ELECTRO_I2C_ADDR`, `IMU_I2C_ADDR`, `IMU_ENABLE_FEATURE_ATTEMPTS`, `IMU_BOOT_DRAIN_SEC` (default **0**; optional short drain), `IMU_BOOT_DRAIN_MAX_CALLS`, `IMU_BOOT_DRAIN_MAX_PACKETS`, `IMU_OPEN_DRAIN_CYCLES`, `IMU_OPEN_DRAIN_MAX_PACKETS`, `IMU_INIT_ROUNDS`, `IMU_POST_OPEN_DELAY_SEC`, `IMU_INIT_PROGRESS` (`1` = print each init step; `Sensor_Imu.imu` CLI sets this), `GPS_I2C_ADDR`, `GPS_I2C_READ_CHUNK` (default 32, u-blox FIFO burst), `GPS_NMEA_STRICT_CHECKSUM` (default on), `GPS_I2C_INIT_RETRIES`, `GPS_I2C_INIT_DELAY_SEC`, `GPS_USE_UART`, `GPS_DEVICE`, `GPS_BAUD`, `GPS_DEBUG_RAW`, `DISTANCE_I2C_ADDR`, `DISTANCE_TFL_TRIGGER`.
- **`FSW_I2C_BUS`**: if `sudo i2cdetect -y 1` shows your sensors but Python reports `No I2C device at address`, Blinka may be using a different bus than `i2c-1`. Run `pip install adafruit-extended-bus` and e.g. `export FSW_I2C_BUS=1` before `main.py`.
- **I2C multiprocessing**: baro / power / IMU / distance each run in a separate process; by default Linux uses `flock` on `FSW_I2C_LOCK_FILE` (default `/tmp/fsw_i2c.lock`) so SMBus transactions do not interleave. Set `FSW_I2C_FLOCK=0` only if you know you do not need it.
- **IMU init**: if logs show `Was not able to enable feature` / feature `1` (accelerometer), try `IMU_POST_OPEN_DELAY_SEC=0.5`, confirm `IMU_I2C_ADDR` (`0x4A` vs `0x4B`), wiring, and `i2cdetect`. `BNO08X_DEBUG=1` enables verbose driver output. If **rotation_vector** (`5`) fails, FSW retries with **game_rotation_vector** (no magnetometer fusion) and uses `game_quaternion` for Euler. Raise `IMU_ENABLE_FEATURE_ATTEMPTS` if needed.
- **IMU init hangs** inside Adafruit `read_bytes` / `_read_header`: keep **`IMU_BOOT_DRAIN_SEC=0`** (default); long packet drains can block on I2C. For **CLI-only** debugging try `FSW_I2C_FLOCK=0`. Use **`python3 -m Sensor_Imu.imu`** with **`IMU_INIT_PROGRESS=1`** to see which enable step stalls. Prefer **100 kHz** I2C (`dtparam=i2c_arm_baudrate=100000`) if the bus is marginal.

### DietPi / Pi quick fixes (from common log lines)

| Log | Action |
|-----|--------|
| `No module named 'picamera2'` in venv | `picamera2`는 apt 패키지라 venv에 직접 설치 불가. venv를 `--system-site-packages`로 재생성: `python3 -m venv --system-site-packages venv` |
| `No module named 'adafruit_ina228'` | In the **same venv** you use for `python3 main.py`: `pip install adafruit-circuitpython-ina228` |
| `No I2C device at address: 0x40` but `i2cdetect` has no `40` | **INA228 not on the bus** (or wrong address). Your scan’s `42` is GNSS, not shunt monitor — wire INA228 or set `ELECTRO_I2C_ADDR` if the board uses a different chip address. |
| IMU `No I2C device at address: 0x4a` though `i2cdetect` shows `4a` | Try `IMU_I2C_ADDR=0x4B`, `BNO08X_DEBUG=1`, RESET/PS0 wiring, and **100 kHz** (`dtparam=i2c_arm_baudrate=100000`). Chip at `4a` must be **BNO08x** (library probes hard). |
| IMU init **hangs** (Ctrl+C in `read_bytes` / `_read_header`) | Default **`IMU_BOOT_DRAIN_SEC=0`**. Do not raise `IMU_OPEN_DRAIN_*` aggressively. `FSW_I2C_FLOCK=0` for single CLI. Shorter wiring / stronger pull-ups. |
| GPS I2C `[Errno 5]` / **`[Errno 121]`** while streaming | u-blox FIFO must be drained with **burst read** (write `0xFF`, read up to **32** bytes per transaction), not one SMBus read per byte. FSW does this by default; tune `GPS_I2C_READ_CHUNK` if needed. |
| GPS lat/lon “teleport” / wrong `crs` for one line | Usually **corrupted NMEA** on the wire; FSW **drops lines with bad XOR checksum** (`GPS_NMEA_STRICT_CHECKSUM=1` default). RMC **course** is clamped to **0–360°** (junk → `0`). |
| GPS probe flaky at boot | `GPS_I2C_INIT_RETRIES` / `GPS_I2C_INIT_DELAY_SEC`; **0xFD** and **0xFE** are read as **separate** 1-byte reads. |
| Distance synthetic / init errors | FSW uses **TF-Luna I2C** at `0x10` by default; check wiring, `FSW_I2C_BUS`, and `i2cdetect`. |
| `No I2C device at address: 0x77` but `i2cdetect -y 1` shows `77` | Force the same bus Python uses: `pip install adafruit-extended-bus` then `export FSW_I2C_BUS=1`. If SDO=GND use `BARO_I2C_ADDR=0x76`. |
| `i2cdetect` shows `42` | **u-blox GNSS** on I2C — default; set `GPS_I2C_ADDR` if not `0x42`. |
| GPS `No such file /dev/ttyUSB0` | GPS is I2C-only in this FSW. Do not wire GNSS to the flight UART; reserve UART for XBee/comm. |
| Baro TLM looks like ~1013 hPa and temp toggling ±0.2 °C | That is **synthetic** fallback after BMP init/read failure — fix I2C address/hardware first |
| Power TLM stepping 7.35–7.38 V in a 4-step pattern | **Synthetic** electro — usually missing `ina228` pip package or INA init error |

### 6) XBee “not connected” / ground station sees nothing

- **One UART, one peripheral**: the Pi’s primary UART is for XBee/comm; use **I2C GNSS** for GPS.
- **Baud match**: set XCTU **Interface Data Rate** to the same value as FSW `UART_BAUD` (default `9600`). Example: `UART_BAUD=115200 python3 main.py`.
- **Wiring**: XBee DIN → Pi TX, DOUT → Pi RX, common GND; logic is 3.3 V.
- **Sanity check**: loop back or use another PC serial monitor at the same baud to confirm bytes leave the Pi when TLM logging is on.


# 전체 로그 통채로로
scp -r root@192.168.96.168:/root/CANSAT_AAS_2026_FSW/eventlogs .
scp -r root@192.168.96.168:/root/CANSAT_AAS_2026_FSW/sensorlogs .

# csv만
scp "root@192.168.0.42:/root/CANSAT_AAS_2026_FSW/logs/*.csv" "$env:USERPROFILE\Desktop\fsw_logs\"

# 최신 파일 1개
ssh root@192.168.1.100 "ls -t /root/CANSAT_AAS_2026_FSW/logs/*.csv | head -1"
# 위 출력 경로를 그대로 scp:
scp "root@192.168.0.42:/root/CANSAT_AAS_2026_FSW/logs/<위에서_나온_파일>.csv" .

# 영상
scp "root@192.168.96.168:/root/CANSAT_AAS_2026_FSW/PICAM_Video/*.h264" .