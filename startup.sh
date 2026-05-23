#!/bin/bash
# This is the startup script
# This script should be executed on startup

# Resolve script location so it works regardless of user (pi, dietpi, etc.)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python_path="${python_path:-${SCRIPT_DIR}}"

# Venv: prefer env var override, then common locations relative to home
HOME_DIR="$(eval echo ~"${SUDO_USER:-${USER}}")"
if [[ -n "${FSW_VENV_DIR:-}" ]]; then
    venv_path="${FSW_VENV_DIR}/bin"
elif [[ -d "${HOME_DIR}/env/bin" ]]; then
    venv_path="${HOME_DIR}/env/bin"
elif [[ -d "${SCRIPT_DIR}/venv/bin" ]]; then
    venv_path="${SCRIPT_DIR}/venv/bin"
else
    venv_path=""
fi

echo "Path > ${python_path}"
if [[ -n "${venv_path}" ]]; then
    echo "venv > ${venv_path}/activate"
    . "${venv_path}/activate"
else
    echo "venv > (none found, using system python)"
fi

cd "${python_path}"
export FSW_I2C_BUS=1
exec python3 main.py
