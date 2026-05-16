"""Live-plot Motor FSW replay control outputs.

Default input:
  C:\\Users\\ms kang\\Desktop\\0405_droptest\\happened\\replay_result.csv

Typical usage:
  python tests\\plot_replay_motor_control.py
  python tests\\plot_replay_motor_control.py --speed 5
  python tests\\plot_replay_motor_control.py --save-png
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path


DEFAULT_INPUT = Path(r"C:\Users\ms kang\Desktop\0405_droptest\happened\replay_result.csv")
DEFAULT_PNG = Path(r"C:\Users\ms kang\Desktop\0405_droptest\happened\motor_control_replay_plot.png")


def _float(row: dict, name: str, default: float = math.nan) -> float:
    raw = row.get(name, "")
    if raw is None or raw == "":
        return default
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def _bool(row: dict, name: str) -> bool:
    return str(row.get(name, "")).strip().lower() in {"1", "true", "yes", "y"}


def _load_rows(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"empty CSV: {path}")
    return rows


def _series(rows: list[dict], name: str) -> list[float]:
    return [_float(row, name) for row in rows]


def _safe_ylim(ax, values: list[float], pad_ratio: float = 0.12) -> None:
    clean = [v for v in values if math.isfinite(v)]
    if not clean:
        return
    lo = min(clean)
    hi = max(clean)
    if lo == hi:
        lo -= 1.0
        hi += 1.0
    pad = max(1e-6, (hi - lo) * pad_ratio)
    ax.set_ylim(lo - pad, hi + pad)


def _prepare_axes(rows: list[dict]):
    import matplotlib.pyplot as plt

    t = _series(rows, "sim_t")
    if not any(math.isfinite(v) for v in t):
        t = _series(rows, "tick")

    desired = _series(rows, "current_desired_angular_velocity_deg_s")
    commanded = _series(rows, "current_commanded_angular_velocity_deg_s")
    measured = _series(rows, "gyrz_deg_s")
    left_pulse = _series(rows, "current_left_pulse_us")
    right_pulse = _series(rows, "current_right_pulse_us")
    distance = _series(rows, "current_distance_m")
    crosstrack = _series(rows, "current_crosstrack_error_m")
    baro = _series(rows, "baro_m")
    fdir_bad_t = [t[i] for i, row in enumerate(rows) if not _bool(row, "current_fdir_pass")]
    fdir_bad_y = [0.0 for _ in fdir_bad_t]

    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(13, 9), constrained_layout=True)
    fig.suptitle("Motor FSW replay: guidance/control live view")

    ax = axes[0]
    desired_line, = ax.plot([], [], label="desired yaw rate (deg/s)", color="#2878b5", linewidth=1.8)
    commanded_line, = ax.plot([], [], label="commanded yaw rate (deg/s)", color="#d04a02", linewidth=1.8)
    measured_line, = ax.plot([], [], label="measured gyrz (deg/s)", color="#6a4c93", linewidth=1.0, alpha=0.8)
    ax.axhline(45.0, color="0.5", linestyle="--", linewidth=0.8)
    ax.axhline(-45.0, color="0.5", linestyle="--", linewidth=0.8)
    ax.set_ylabel("yaw rate")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right", ncols=3, fontsize=8)
    _safe_ylim(ax, desired + commanded + measured)

    ax = axes[1]
    left_line, = ax.plot([], [], label="left pulse (us)", color="#00876c", linewidth=1.8)
    right_line, = ax.plot([], [], label="right pulse (us)", color="#b02a30", linewidth=1.8)
    ax.axhline(1500.0, color="0.35", linestyle="--", linewidth=0.8, label="neutral")
    ax.axhline(600.0, color="0.6", linestyle=":", linewidth=0.8)
    ax.axhline(2500.0, color="0.6", linestyle=":", linewidth=0.8)
    ax.set_ylabel("servo pulse")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right", ncols=3, fontsize=8)
    ax.set_ylim(500, 2600)

    ax = axes[2]
    distance_line, = ax.plot([], [], label="distance to target (m)", color="#466f9d", linewidth=1.8)
    crosstrack_line, = ax.plot([], [], label="cross-track error (m)", color="#f0a202", linewidth=1.5)
    ax.set_ylabel("guidance")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right", ncols=2, fontsize=8)
    _safe_ylim(ax, distance + crosstrack)

    ax = axes[3]
    baro_line, = ax.plot([], [], label="baro altitude (m)", color="#3b7d23", linewidth=1.8)
    fdir_scatter = ax.scatter(fdir_bad_t, fdir_bad_y, label="FDIR blocked", color="#c1121f", s=12, alpha=0.5)
    ax.set_ylabel("alt / FDIR")
    ax.set_xlabel("replay time (s)")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right", ncols=2, fontsize=8)
    _safe_ylim(ax, baro + [0.0])

    cursors = [a.axvline(t[0], color="black", linewidth=1.0, alpha=0.55) for a in axes]
    status = fig.text(0.01, 0.01, "", fontsize=9)

    lines = {
        "desired": desired_line,
        "commanded": commanded_line,
        "measured": measured_line,
        "left": left_line,
        "right": right_line,
        "distance": distance_line,
        "crosstrack": crosstrack_line,
        "baro": baro_line,
        "fdir": fdir_scatter,
    }
    data = {
        "t": t,
        "desired": desired,
        "commanded": commanded,
        "measured": measured,
        "left": left_pulse,
        "right": right_pulse,
        "distance": distance,
        "crosstrack": crosstrack,
        "baro": baro,
        "cursors": cursors,
        "status": status,
    }
    return fig, axes, lines, data


def _draw_until(idx: int, rows: list[dict], lines: dict, data: dict) -> None:
    t = data["t"][: idx + 1]
    for name in ("desired", "commanded", "measured", "left", "right", "distance", "crosstrack", "baro"):
        lines[name].set_data(t, data[name][: idx + 1])
    cur_t = data["t"][idx]
    for cursor in data["cursors"]:
        cursor.set_xdata([cur_t, cur_t])
    row = rows[idx]
    data["status"].set_text(
        "t={:.2f}s | segment={} | phase={} | action={} | fdir={}".format(
            cur_t,
            row.get("segment", ""),
            row.get("current_phase", ""),
            row.get("current_safety_action", ""),
            row.get("current_fdir_reason", "") or "PASS",
        )
    )


def plot_static(rows: list[dict], output: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, _axes, lines, data = _prepare_axes(rows)
    _draw_until(len(rows) - 1, rows, lines, data)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160)
    plt.close(fig)
    print(f"saved={output}")


def plot_live(rows: list[dict], speed: float) -> None:
    import matplotlib.pyplot as plt

    fig, _axes, lines, data = _prepare_axes(rows)
    t = data["t"]
    plt.show(block=False)
    for idx in range(len(rows)):
        _draw_until(idx, rows, lines, data)
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        if idx + 1 < len(rows):
            dt = max(0.0, t[idx + 1] - t[idx])
            time.sleep(dt / max(speed, 1e-6))
    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--speed", type=float, default=10.0, help="replay speed multiplier for live plot")
    parser.add_argument("--save-png", type=Path, nargs="?", const=DEFAULT_PNG, default=None)
    parser.add_argument("--no-show", action="store_true", help="only validate input; do not open a live window")
    args = parser.parse_args()

    rows = _load_rows(args.input)
    if args.save_png is not None:
        plot_static(rows, args.save_png)
    if not args.no_show and args.save_png is None:
        plot_live(rows, args.speed)


if __name__ == "__main__":
    main()
