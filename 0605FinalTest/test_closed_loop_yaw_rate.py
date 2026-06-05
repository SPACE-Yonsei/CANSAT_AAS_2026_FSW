from collections import Counter

from tools import sim_closed_loop_yaw_rate as sim


def test_closed_loop_yaw_rate_acceptance(tmp_path):
    rows = sim.run_all_cases()
    out_path = tmp_path / "closed_loop_yaw_rate_results.csv"
    sim.write_csv(rows, out_path)
    summary = sim.summarize(rows)

    for mode, counts in summary.items():
        print(f"{mode}: {counts}")
    fail_reasons = Counter(
        reason
        for row in rows
        for reason in str(row["fail_reason"]).split("|")
        if reason
    )
    print(f"fail_reasons={dict(fail_reasons)}")
    print(f"gps_faster_than_dr_pm={sim.gps_faster_than_dr_pm(rows)}")

    assert out_path.exists()
    assert all(not row["diverged"] for row in rows)
    all_pass = all(row["pass"] for row in rows)
    gps_fast = sim.gps_faster_than_dr_pm(rows)
    assert all_pass and gps_fast, (
        f"all_pass={all_pass}, gps_faster_than_dr_pm={gps_fast}, "
        f"summary={summary}, fail_reasons={dict(fail_reasons)}"
    )
