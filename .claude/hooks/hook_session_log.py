import subprocess, datetime, os, json, sys

log_dir = "logs"
os.makedirs(log_dir, exist_ok=True)

# staged + unstaged 변경 모두 수집
staged = subprocess.run(
    ["git", "diff", "--name-only", "--cached"],
    capture_output=True, text=True
).stdout.splitlines()

unstaged = subprocess.run(
    ["git", "diff", "--name-only"],
    capture_output=True, text=True
).stdout.splitlines()

untracked = subprocess.run(
    ["git", "ls-files", "--others", "--exclude-standard"],
    capture_output=True, text=True
).stdout.splitlines()

# 중복 제거 후 .py 필터
all_changed = sorted(set(staged + unstaged))
py_changed = [f for f in all_changed if f.endswith(".py")]
new_files = [f for f in untracked if f.endswith(".py")]

timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
log_path = os.path.join(log_dir, "session_log.txt")

# 훅 payload에서 세션 정보 추출 (있을 경우)
try:
    payload = json.load(sys.stdin) if not sys.stdin.isatty() else {}
except Exception:
    payload = {}

with open(log_path, "a", encoding="utf-8") as f:
    f.write(f"\n[{timestamp}] 세션 종료\n")
    if py_changed:
        f.write(f"  수정됨 ({len(py_changed)}개):\n")
        for path in py_changed:
            tag = "[staged]" if path in staged else "[unstaged]"
            f.write(f"    {tag} {path}\n")
    if new_files:
        f.write(f"  신규 미추적 ({len(new_files)}개):\n")
        for path in new_files:
            f.write(f"    [untracked] {path}\n")
    if not py_changed and not new_files:
        f.write("  변경 없음\n")
