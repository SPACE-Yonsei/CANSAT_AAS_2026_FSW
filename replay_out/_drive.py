import os, sys
sys.stdout.reconfigure(encoding="utf-8")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import replay_guidance_modes as R

BASE = r"C:\Users\ms kang\Desktop\\" + "닭"  # 닭
DIRS = ["1차_origin_miss", "2차", "3차"]  # 1차 2차 3차
TAG = {"1차_origin_miss": "1cha", "2차": "2cha", "3차": "3cha"}

for d in DIRS:
    p = os.path.join(BASE, d, "raw_motor_part.xlsx")
    tag = TAG[d]
    print("########", tag, os.path.exists(p))
    sys.argv = ["x", p, "--out", f"replay_out/result_{tag}.csv",
                "--summary", f"replay_out/summary_{tag}.md"]
    R.main()
    print()
