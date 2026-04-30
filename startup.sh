#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${ROOT_DIR}/.venv"

if command -v pigpiod >/dev/null 2>&1; then
  if ! pgrep -x pigpiod >/dev/null 2>&1; then
    pigpiod || true
  fi
fi

if [[ -d "${VENV_DIR}" ]]; then
  # shellcheck disable=SC1091
  source "${VENV_DIR}/bin/activate"
fi

cd "${ROOT_DIR}"
exec python3 main.py
