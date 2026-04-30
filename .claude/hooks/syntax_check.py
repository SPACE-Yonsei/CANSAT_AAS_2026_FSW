import json, sys, ast, os

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
