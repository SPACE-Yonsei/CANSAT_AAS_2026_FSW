#!/usr/bin/env bash
set -euo pipefail

# Default: kill only CANSAT-FSW related Python processes.
# Use --all to kill every python/python3 process.
# Use --pattern "<regex>" to override matching rule.

MODE="default"
PATTERN="CANSAT_AAS_2026_FSW|main.py|ground_station.py|Sensor_|commapp.py"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --all)
            MODE="all"
            shift
            ;;
        --pattern)
            if [[ $# -lt 2 ]]; then
                echo "[ERR] --pattern requires a value"
                exit 1
            fi
            MODE="pattern"
            PATTERN="$2"
            shift 2
            ;;
        -h|--help)
            cat <<'EOF'
Usage:
  ./killpython.sh
  ./killpython.sh --all
  ./killpython.sh --pattern "<regex>"

Options:
  --all             Kill all python/python3 processes.
  --pattern <regex> Kill processes matching regex via pgrep -f.
EOF
            exit 0
            ;;
        *)
            echo "[ERR] Unknown option: $1"
            exit 1
            ;;
    esac
done

if [[ "$MODE" == "all" ]]; then
    PIDS="$(pgrep -f '(^|/)(python|python3)([[:space:]]|$)' || true)"
else
    PIDS="$(pgrep -f "$PATTERN" || true)"
fi

if [[ -z "${PIDS}" ]]; then
    echo "[INFO] No matching Python process found."
    exit 0
fi

echo "[INFO] Target PIDs: ${PIDS}"
echo "${PIDS}" | xargs -r kill -TERM

sleep 1

REMAINING="$(echo "${PIDS}" | xargs -r ps -o pid= -p 2>/dev/null | tr -d ' ' || true)"
if [[ -n "${REMAINING}" ]]; then
    echo "[WARN] Force killing remaining PIDs: ${REMAINING}"
    echo "${REMAINING}" | xargs -r kill -KILL
fi

echo "[INFO] Done."
