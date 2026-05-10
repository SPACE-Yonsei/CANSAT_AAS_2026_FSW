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

PYTHON_BIN="python3"
if [[ -n "${VENV_DIR}" && -x "${VENV_DIR}/bin/python3" ]]; then
  PYTHON_BIN="${VENV_DIR}/bin/python3"
elif [[ -n "${VENV_DIR}" ]]; then
  echo "startup.sh: venv at ${VENV_DIR} has no bin/python3; using PATH python3" >&2
fi

export FSW_I2C_BUS="${FSW_I2C_BUS:-1}"
export GPS_I2C_ADDR="${GPS_I2C_ADDR:-0x42}"

cd "${ROOT_DIR}"
# Use venv interpreter directly so systemd (non-interactive) always picks pip packages.
exec "${PYTHON_BIN}" main.py
