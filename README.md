# CANSAT AAS 2026 FSW

Python multiprocessing flight software for Yonsei Space-Y CANSAT.

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
pip install adafruit-circuitpython-ina22x
pip install pigpio
```

### 4) Camera dependencies

```bash
sudo apt install -y python3-picamera2 libcamera-apps libcamera-tools
```

## Camera Configuration (Important)

The current implementation uses `Sensor_Camera/picam.py` and `Sensor_Camera/cameraapp.py`.

### Hardware config example (Pi Cam v3 / IMX708)

Edit `/boot/firmware/config.txt`:

```ini
camera_auto_detect=0
[all]
dtoverlay=imx708
```

Then reboot.

### Runtime camera behavior

- Camera init tries `picamera2` first.
- Recording config (current code):
  - video format: `RGB888`
  - resolution: `640x480`
  - segmented recording via `cameraapp` (`SEGMENT_SEC=1.0`)
- Output directory: `PICAM_Video/`
- Output filename format: `P_MMDD_HHMMSS_microsec.*`
- If camera backend is unavailable:
  - system falls back gracefully
  - placeholder segment files are created so pipeline/test does not break

### Camera command path

- `CMD,1070,CAM,ON` -> start recording
- `CMD,1070,CAM,OFF` -> stop recording
- FlightLogic release flow can trigger `MID_cam_activate` to force start

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
sudo systemctl start cansat-fsw.service
sudo systemctl status cansat-fsw.service
```

## Test

### Full test suite

```bash
python -m unittest
```

### Main process smoke test

```bash
python -m unittest tests/test_main_smoke.py
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
- See `Cluad.md` and `fsw_step12_audit.md` for phased implementation/audit status.

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
you are likely running older app modules. Update to latest `claude` branch code,
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

- **Barometer**: uses `Sensor_Barometer/barometer.py` (BMP3xx on I2C, default `0x77`) when Blinka + `adafruit-circuitpython-bmp3xx` work; otherwise the app falls back to synthetic altitude/pressure.
- **IMU**: uses `Sensor_Imu/imu.py` (BNO08x, default `0x4A`) when `adafruit-circuitpython-bno08x` works.
- **GPS**: uses `Sensor_Gps/gps.py` on a **separate** UART from the radio. Set `GPS_DEVICE` (e.g. `/dev/ttyUSB0`) and `GPS_BAUD` (often `9600` or `38400`). If no GPS serial is available, `gpsapp` keeps its synthetic track.
- **Distance**: still uses the built-in synthetic rangefinder in `distanceapp.py` unless you add a real sensor module and wire it there.
- Env hints: `BARO_I2C_ADDR`, `IMU_I2C_ADDR`, `GPS_DEVICE`, `GPS_BAUD`.

### 6) XBee “not connected” / ground station sees nothing

- **One UART, one peripheral**: if the Pi’s primary UART (`/dev/serial0`) is wired to the XBee, a UART GPS cannot share that same port; use USB GPS (`/dev/ttyUSB0`) or a second UART.
- **Baud match**: set XCTU **Interface Data Rate** to the same value as FSW `UART_BAUD` (default `9600`). Example: `UART_BAUD=115200 python3 main.py`.
- **Wiring**: XBee DIN → Pi TX, DOUT → Pi RX, common GND; logic is 3.3 V.
- **Sanity check**: loop back or use another PC serial monitor at the same baud to confirm bytes leave the Pi when TLM logging is on.
