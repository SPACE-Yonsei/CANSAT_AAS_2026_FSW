<<<<<<< HEAD
﻿import json, sys, ast, os
=======
"""
PostToolUse hook - Write/Edit 후 Python 파일 문법 검사.
stdin: {"tool_name": "Write", "tool_input": {"file_path": "..."}, "tool_result": ...}
exit 2 → 문법 오류 시 Claude에게 전달
exit 0 → 통과 또는 비Python 파일
"""
import json, sys, ast, os
>>>>>>> 8a94f4a (skill, agent, hanes)

payload = json.load(sys.stdin)
file_path = payload.get("tool_input", {}).get("file_path", "")

if not file_path.endswith(".py"):
    sys.exit(0)

if not os.path.isfile(file_path):
    sys.exit(0)

try:
    with open(file_path, "r", encoding="utf-8") as f:
        source = f.read()
    ast.parse(source)
    sys.exit(0)
except SyntaxError as e:
    print(f"[HOOK] 문법 오류 in {file_path}: {e}", file=sys.stderr)
    sys.exit(2)
