#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_FILE="${ROOT_DIR}/cansat-fsw.service"

if [[ ! -f "${SERVICE_FILE}" ]]; then
  echo "service file not found: ${SERVICE_FILE}"
  exit 1
fi

sudo cp "${SERVICE_FILE}" /etc/systemd/system/cansat-fsw.service
sudo sed -i "s|^WorkingDirectory=.*|WorkingDirectory=${ROOT_DIR}|g" /etc/systemd/system/cansat-fsw.service
sudo sed -i "s|^ExecStart=.*|ExecStart=${ROOT_DIR}/startup.sh|g" /etc/systemd/system/cansat-fsw.service

# systemd status=217/USER if User= does not exist on the host (e.g. DietPi has no "pi").
# Override: FSW_SYSTEMD_USER=myuser bash setup_systemd_service.sh
TARGET_USER="${FSW_SYSTEMD_USER:-}"
if [[ -z "${TARGET_USER}" ]]; then
  if [[ -n "${SUDO_USER:-}" ]]; then
    TARGET_USER="${SUDO_USER}"
  else
    TARGET_USER="$(stat -c '%U' "${ROOT_DIR}" 2>/dev/null || echo root)"
  fi
fi
if ! id -u "${TARGET_USER}" >/dev/null 2>&1; then
  echo "error: systemd User '${TARGET_USER}' does not exist (set FSW_SYSTEMD_USER)"
  exit 1
fi
sudo sed -i "s|^User=.*|User=${TARGET_USER}|g" /etc/systemd/system/cansat-fsw.service

sudo chmod +x "${ROOT_DIR}/startup.sh"

# Optional: point systemd at a venv outside the repo (e.g. /root/venv).
#   FSW_VENV_DIR=/root/venv bash setup_systemd_service.sh
DROP_IN="/etc/systemd/system/cansat-fsw.service.d"
if [[ -n "${FSW_VENV_DIR:-}" ]]; then
  sudo mkdir -p "${DROP_IN}"
  # Prepend venv bin to PATH so bare `python3` / multiprocessing `sys.executable`
  # resolve to the venv (some boards lack bin/python3 symlink; startup.sh also picks explicitly).
  {
    printf '%s\n' '[Service]'
    printf '%s\n' "Environment=FSW_VENV_DIR=${FSW_VENV_DIR}"
    printf '%s\n' "Environment=PATH=${FSW_VENV_DIR}/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin"
  } | sudo tee "${DROP_IN}/fsw-venv.conf" >/dev/null
  echo "wrote ${DROP_IN}/fsw-venv.conf (FSW_VENV_DIR=${FSW_VENV_DIR} + PATH prepend)"
fi

sudo systemctl daemon-reload
sudo systemctl enable cansat-fsw.service
echo "installed cansat-fsw.service ($(grep -E '^User=' /etc/systemd/system/cansat-fsw.service || echo 'User=(unset, runs as root)'))"
echo "start with: sudo systemctl start cansat-fsw.service"
