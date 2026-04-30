import subprocess, datetime, os

log_dir = "logs"
os.makedirs(log_dir, exist_ok=True)

result = subprocess.run(
    ["git", "diff", "--name-only", "HEAD"],
    capture_output=True, text=True
)
changed = [f for f in result.stdout.splitlines() if f.endswith(".py")]

timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
log_path = os.path.join(log_dir, "session_log.txt")

with open(log_path, "a", encoding="utf-8") as f:
    f.write(f"\n[{timestamp}] 세션 종료 - 변경된 파일 ({len(changed)}개)\n")
    for path in changed:
        f.write(f"  {path}\n")
