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
