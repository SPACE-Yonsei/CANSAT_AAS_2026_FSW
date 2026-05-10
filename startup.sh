#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Prefer explicit path, then .venv, then venv (many boards use `python3 -m venv venv`).
VENV_DIR="${FSW_VENV_DIR:-}"
if [[ -z "${VENV_DIR}" || ! -d "${VENV_DIR}" ]]; then
  if [[ -d "${ROOT_DIR}/.venv" ]]; then
    VENV_DIR="${ROOT_DIR}/.venv"
  elif [[ -d "${ROOT_DIR}/venv" ]]; then
    VENV_DIR="${ROOT_DIR}/venv"
  else
    VENV_DIR=""
  fi
fi

if command -v pigpiod >/dev/null 2>&1; then
  if ! pgrep -x pigpiod >/dev/null 2>&1; then
    pigpiod || true
  fi
fi

if [[ -n "${VENV_DIR}" ]]; then
  # shellcheck disable=SC1091
  source "${VENV_DIR}/bin/activate"
fi

export FSW_I2C_BUS="${FSW_I2C_BUS:-1}"
export GPS_I2C_ADDR="${GPS_I2C_ADDR:-0x42}"

cd "${ROOT_DIR}"
exec python3 main.py
