#!/usr/bin/env bash
# DietPi initial install script for CANSAT_AAS_2026_FSW (mag branch)
# Run as root: sudo bash initial_install.sh
# Installs system packages, Python venv, and systemd service.

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="/root/venv"
SERVICE_SRC="${REPO_DIR}/cansat-fsw.service"
SERVICE_DST="/etc/systemd/system/cansat-fsw.service"
CONFIG_TXT="/boot/firmware/config.txt"
# Older DietPi images use /boot/config.txt
if [[ ! -f "${CONFIG_TXT}" ]]; then
    CONFIG_TXT="/boot/config.txt"
fi

echo "=============================="
echo " CANSAT FSW - DietPi Installer"
echo "=============================="
echo " Repo dir : ${REPO_DIR}"
echo " Venv dir : ${VENV_DIR}"
echo " Boot cfg : ${CONFIG_TXT}"
echo ""

# ---------------------------------------------------------------------------
# 1. System packages
# ---------------------------------------------------------------------------
echo "[1/6] Installing system packages..."
apt-get update -qq

# Core packages — must succeed
apt-get install -y \
    python3 python3-pip python3-venv python3-dev \
    git i2c-tools \
    libgpiod-dev \
    libopenblas-dev \
    build-essential \
    wget unzip

# picamera2 — available on Bookworm; skip gracefully on older images
apt-get install -y python3-picamera2 python3-libcamera 2>/dev/null \
    || echo "    [WARN] python3-picamera2 not found in apt — camera will use fallback"

# libgpiod runtime lib — package name differs by Debian version
apt-get install -y libgpiod2 2>/dev/null \
    || apt-get install -y libgpiod3 2>/dev/null \
    || echo "    [WARN] libgpiod runtime lib not found — GPIO may be limited"

# pigpio — not in Debian Bookworm repos; build from source if needed
if ! apt-get install -y pigpio 2>/dev/null; then
    echo "    pigpio not in apt — building from source (joan2937/pigpio)..."
    rm -rf /tmp/pigpio-master /tmp/pigpio.zip
    cd /tmp
    wget -q https://github.com/joan2937/pigpio/archive/master.zip -O pigpio.zip
    unzip -q pigpio.zip
    cd pigpio-master
    make -j$(nproc)
    # Python setup.py fails on Python 3.12+ (distutils removed); daemon binary is
    # installed before that step, and the Python client is handled by pip below.
    make install || true
    [[ -f /usr/local/bin/pigpiod ]] || install -m 0755 pigpiod /usr/local/bin/
    ldconfig
    # Source build has no systemd unit — create one
    if [[ ! -f /lib/systemd/system/pigpiod.service ]] && \
       [[ ! -f /usr/lib/systemd/system/pigpiod.service ]]; then
        cat > /etc/systemd/system/pigpiod.service <<'EOF'
[Unit]
Description=Pigpio GPIO daemon
After=network.target

[Service]
Type=forking
ExecStart=/usr/local/bin/pigpiod
Restart=on-failure

[Install]
WantedBy=multi-user.target
EOF
    fi
    cd "${REPO_DIR}"
    echo "    pigpio built and installed from source"
fi

# ---------------------------------------------------------------------------
# 2. /boot config.txt — I2C, UART, Camera, pigpio
# ---------------------------------------------------------------------------
echo "[2/6] Configuring ${CONFIG_TXT}..."

_ensure_entry() {
    local key="$1" val="$2"
    if grep -qE "^${key}=" "${CONFIG_TXT}" 2>/dev/null; then
        sed -i "s|^${key}=.*|${key}=${val}|" "${CONFIG_TXT}"
    else
        echo "${key}=${val}" >> "${CONFIG_TXT}"
    fi
}

_ensure_entry "dtparam=i2c_arm" "on"
_ensure_entry "dtparam=i2c1" "on"

# i2c-dev exposes /dev/i2c-N; not auto-loaded on DietPi
grep -qx "i2c-dev" /etc/modules || echo "i2c-dev" >> /etc/modules
_ensure_entry "enable_uart" "1"
_ensure_entry "camera_auto_detect" "1"

# I2C baudrate: 400kHz (fast-mode) for INA228/BNO08x/etc.
if ! grep -q "i2c_arm_baudrate" "${CONFIG_TXT}"; then
    echo "dtparam=i2c_arm_baudrate=400000" >> "${CONFIG_TXT}"
fi

# Disable Bluetooth so PL011 (ttyAMA0) is freed for XBee / GNSS / TF-Luna.
# Without this, enable_uart=1 only exposes the mini-UART (ttyS0) which is
# clock-coupled to the CPU and produces baud errors at higher speeds.
if ! grep -q "disable-bt" "${CONFIG_TXT}"; then
    echo "dtoverlay=disable-bt" >> "${CONFIG_TXT}"
fi

echo "    done (reboot required for hardware changes to take effect)"

# ---------------------------------------------------------------------------
# 3. Python venv + pip packages
# ---------------------------------------------------------------------------
echo "[3/6] Creating venv at ${VENV_DIR}..."
python3 -m venv --system-site-packages "${VENV_DIR}"
# --system-site-packages lets the venv use apt-installed picamera2/libcamera

echo "[3/6] Installing pip packages..."
"${VENV_DIR}/bin/pip" install --upgrade pip --quiet
"${VENV_DIR}/bin/pip" install \
    smbus2 \
    RPi.GPIO \
    pigpio \
    pyserial \
    adafruit-blinka \
    adafruit-circuitpython-bmp3xx \
    adafruit-circuitpython-bno08x \
    adafruit-circuitpython-ina228 \
    adafruit-extended-bus

# ---------------------------------------------------------------------------
# 4. pigpiod systemd enable
# ---------------------------------------------------------------------------
echo "[4/6] Enabling pigpiod service..."
systemctl enable pigpiod
systemctl start pigpiod || true

# ---------------------------------------------------------------------------
# 5. FSW systemd service
# ---------------------------------------------------------------------------
echo "[5/6] Installing cansat-fsw.service..."

if [[ ! -f "${SERVICE_SRC}" ]]; then
    echo "    [WARN] ${SERVICE_SRC} not found, generating default service file"
    cat > "${SERVICE_SRC}" <<EOF
[Unit]
Description=CANSAT Flight Software
After=network.target pigpiod.service
Requires=pigpiod.service

[Service]
Type=simple
User=root
WorkingDirectory=${REPO_DIR}
Environment="PATH=${VENV_DIR}/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
Environment="FSW_VENV_DIR=${VENV_DIR}"
ExecStart=/bin/bash ${REPO_DIR}/startup.sh
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF
fi

cp "${SERVICE_SRC}" "${SERVICE_DST}"
# Patch paths into the installed copy
sed -i "s|^WorkingDirectory=.*|WorkingDirectory=${REPO_DIR}|" "${SERVICE_DST}"
sed -i "s|^ExecStart=.*|ExecStart=/bin/bash ${REPO_DIR}/startup.sh|" "${SERVICE_DST}"

DROP_IN="/etc/systemd/system/cansat-fsw.service.d"
mkdir -p "${DROP_IN}"
cat > "${DROP_IN}/fsw-venv.conf" <<EOF
[Service]
Environment=FSW_VENV_DIR=${VENV_DIR}
Environment=PATH=${VENV_DIR}/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
EOF

chmod +x "${REPO_DIR}/startup.sh"
systemctl daemon-reload
systemctl enable cansat-fsw.service

# ---------------------------------------------------------------------------
# 6. Verify
# ---------------------------------------------------------------------------
echo "[6/6] Verifying venv imports..."
"${VENV_DIR}/bin/python3" - <<'PYCHECK'
import importlib, sys
PKGS = [
    ("smbus2",                  "smbus2"),
    ("RPi.GPIO",                "RPi.GPIO"),
    ("pigpio",                  "pigpio"),
    ("serial",                  "pyserial"),
    ("board",                   "adafruit-blinka"),
    ("adafruit_bmp3xx",         "adafruit-circuitpython-bmp3xx"),
    ("adafruit_bno08x",         "adafruit-circuitpython-bno08x"),
    ("adafruit_ina228",         "adafruit-circuitpython-ina228"),
    ("adafruit_extended_bus",   "adafruit-extended-bus"),
]
ok = True
for mod, pkg in PKGS:
    try:
        importlib.import_module(mod)
        print(f"  [OK]   {mod}")
    except ImportError as e:
        print(f"  [FAIL] {mod}  ({pkg}): {e}", file=sys.stderr)
        ok = False
sys.exit(0 if ok else 1)
PYCHECK

echo ""
echo "=============================="
echo " Install complete."
echo " Reboot to apply hardware config (I2C, camera, UART)."
echo ""
echo " After reboot:"
echo "   sudo systemctl start cansat-fsw.service"
echo "   journalctl -fu cansat-fsw.service"
echo "=============================="
