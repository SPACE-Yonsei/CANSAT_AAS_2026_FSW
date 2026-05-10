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

sudo systemctl daemon-reload
sudo systemctl enable cansat-fsw.service
echo "installed cansat-fsw.service"
echo "start with: sudo systemctl start cansat-fsw.service"
