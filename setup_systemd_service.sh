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
sudo chmod +x "${ROOT_DIR}/startup.sh"

sudo systemctl daemon-reload
sudo systemctl enable cansat-fsw.service
echo "installed cansat-fsw.service"
echo "start with: sudo systemctl start cansat-fsw.service"
