import json, sys, re

payload = json.load(sys.stdin)
cmd = payload.get("tool_input", {}).get("command", "")

DANGEROUS = [
    r"rm\s+-rf?\s+/",
    r"rm\s+-rf?\s+\.",
    r"rm\s+.*\.(py|sh|md|json)\b",
    r">\s*(main\.py|lib/|Sensor_|comm/|flight_logic/)",
    r"git\s+reset\s+--hard",
    r"git\s+push\s+--force",
]

for pattern in DANGEROUS:
    if re.search(pattern, cmd):
        print(f"[HOOK] 위험 명령 차단: {cmd[:80]}", file=sys.stderr)
        sys.exit(2)

sys.exit(0)
