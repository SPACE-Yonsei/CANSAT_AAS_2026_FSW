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

# Pick the venv interpreter (some trees only have bin/python or bin/python3.13).
PYTHON_BIN="python3"
if [[ -n "${VENV_DIR}" ]]; then
  _picked=""
  for _cand in python3 python3.13 python; do
    if [[ -x "${VENV_DIR}/bin/${_cand}" ]]; then
      _picked="${VENV_DIR}/bin/${_cand}"
      break
    fi
  done
  if [[ -n "${_picked}" ]]; then
    PYTHON_BIN="${_picked}"
    echo "startup.sh: using venv python ${PYTHON_BIN}" >&2
  else
    echo "startup.sh: venv at ${VENV_DIR} has no bin/python3|python3.13|python; using PATH python3" >&2
  fi
fi

export FSW_I2C_BUS="${FSW_I2C_BUS:-1}"
export GPS_I2C_ADDR="${GPS_I2C_ADDR:-0x42}"

cd "${ROOT_DIR}"
# Use venv interpreter directly so systemd (non-interactive) always picks pip packages.
exec "${PYTHON_BIN}" main.py
