<<<<<<< HEAD
﻿import json, sys, re
=======
"""
PreToolUse hook - Bash 명령 위험도 검사.
stdin: {"tool_name": "Bash", "tool_input": {"command": "..."}}
exit 2 → Claude에게 에러 전달 후 실행 블로킹
exit 0 → 통과
"""
import json, sys, re
>>>>>>> 8a94f4a (skill, agent, hanes)

payload = json.load(sys.stdin)
cmd = payload.get("tool_input", {}).get("command", "")

<<<<<<< HEAD
DANGEROUS = [
    r"rm\s+-rf?\s+/",
    r"rm\s+-rf?\s+\.",
    r"rm\s+.*\.(py|sh|md|json)\b",
    r">\s*(main\.py|lib/|Sensor_|comm/|flight_logic/)",
    r"git\s+reset\s+--hard",
    r"git\s+push\s+--force",
=======
# 소스 디렉터리에 대한 rm -rf 차단
DANGEROUS = [
    r"rm\s+-rf?\s+/",                          # rm -rf /
    r"rm\s+-rf?\s+\.",                          # rm -rf .
    r"rm\s+.*\.(py|sh|md|json)\b",             # .py/.sh 파일 직접 삭제
    r">\s*(main\.py|lib/|Sensor_|comm/|flight_logic/)",  # 소스 파일 덮어쓰기
    r"git\s+reset\s+--hard",                   # hard reset
    r"git\s+push\s+--force",                   # force push
>>>>>>> 8a94f4a (skill, agent, hanes)
]

for pattern in DANGEROUS:
    if re.search(pattern, cmd):
        print(f"[HOOK] 위험 명령 차단: {cmd[:80]}", file=sys.stderr)
        sys.exit(2)

sys.exit(0)
