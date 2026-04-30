import subprocess
import sys
import unittest
from pathlib import Path


class TestMainSmoke(unittest.TestCase):
    def test_main_startup_and_graceful_shutdown(self):
        repo_root = Path(__file__).resolve().parents[1]
        runner = repo_root / "tests" / "main_smoke_runner.py"

        proc = subprocess.run(
            [sys.executable, str(runner)],
            cwd=str(repo_root),
            text=True,
            capture_output=True,
            timeout=20,
        )

        self.assertEqual(
            proc.returncode,
            0,
            msg=f"main smoke failed.\nstdout:\n{proc.stdout}\nstderr:\n{proc.stderr}",
        )


if __name__ == "__main__":
    unittest.main()
