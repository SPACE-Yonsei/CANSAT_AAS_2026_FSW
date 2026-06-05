from __future__ import annotations

import sys
from pathlib import Path


TESTS = Path(__file__).resolve().parent
if str(TESTS) not in sys.path:
    sys.path.insert(0, str(TESTS))

from test_flight_log_replay_sil_current import *  # noqa: F401,F403,E402
