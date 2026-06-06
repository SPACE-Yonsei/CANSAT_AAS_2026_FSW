"""Validate a recorded raw_motor.csv against the DR<=GPS steering invariant.

Drop-test acceptance helper. For every valid DR control row it:
  1. checks steering DIRECTION (sign(delta_arm) vs sign(nu_clamped)),
  2. recomputes delta_ff under the CURRENT (config) FF-shaping convention from the
     logged L1 command (yaw_rate_cmd_dps) — which is independent of FF shaping —
     and verifies it never exceeds GPS_TRACKING_CLOSED FF at the same command,
  3. reports steering-authority health (cap engagement, deadband loss, V-floor
     reliance) so over/under-tuning is visible.

Usage:
    python -m tools.analyze_motor_run [path\\to\\raw_motor.csv]

Exit code is non-zero if any DR row would exceed GPS_TRACKING_CLOSED (invariant
break) or if steering direction disagrees with the heading error.
"""
from __future__ import annotations

import sys

import numpy as np
import pandas as pd

from lib import config
from Sensor_Motor import control

DEFAULT = r"C:\Users\ms kang\Downloads\run_20260606_203310\raw_motor.csv"

GPS_REF = config.GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS
GPS_DB = config.CTRL_ANGULAR_VELOCITY_DEADBAND_DEG_S
CAP = config.DR_FF_DELTA_LIMIT_DEG


def _ff_scale(mode: str) -> float:
    if mode.startswith("DR_PM_"):
        return config.DR_PM_FF_SCALE
    if mode.startswith("DR_M_"):
        return config.DR_M_FF_SCALE
    return 1.0


def _num(df, col):
    return pd.to_numeric(df[col], errors="coerce")


def main(path: str) -> int:
    df = pd.read_csv(path, low_memory=False)
    df["mode"] = df["control_mode"].astype(str).str.replace("ControlMode.", "", regex=False)
    valid = df[df["output_valid"] == 1].copy()
    dr = valid[valid["mode"].str.startswith("DR_")].copy()
    gps = valid[valid["mode"].str.startswith("GPS_TRACKING")].copy()

    print(f"file: {path}")
    print(f"rows={len(df)}  valid_ctrl={len(valid)}  GPS={len(gps)}  DR={len(dr)}")
    print("mode counts:\n" + df["mode"].value_counts().to_string())

    rc = 0
    if not len(dr):
        print("\n(no DR rows — nothing to validate)")
        return rc

    # 1) direction
    nu = _num(dr, "nu_clamped_deg")
    da = _num(dr, "delta_arm_deg")
    steer = nu.notna() & da.notna() & (nu.abs() > config.NU_DEADBAND_DEG) & (da.abs() > 1e-6)
    agree = float((np.sign(nu[steer]) == np.sign(da[steer])).mean() * 100) if steer.any() else 100.0
    print(f"\n[direction] sign(delta_arm)==sign(nu): {agree:.1f}%  (n={int(steer.sum())})")
    if agree < 99.9:
        print("  !! DIRECTION INVARIANT BROKEN")
        rc = 1

    # 2) strength: recompute delta_ff under CURRENT convention from logged cmd
    cmd = _num(dr, "yaw_rate_cmd_dps").fillna(0.0).to_numpy()
    lim = _num(dr, "yaw_rate_limit_dps").fillna(50.0).to_numpy()
    sc = dr["mode"].map(_ff_scale).to_numpy()
    new_ff, gps_ff = [], []
    for c, l, s in zip(cmd, lim, sc):
        f = control.angular_velocity_to_delta_ff(c, l, deadband_dps=GPS_DB, ref_dps=GPS_REF) * s
        new_ff.append(max(-CAP, min(CAP, f)))
        gps_ff.append(control.angular_velocity_to_delta_ff(c, GPS_REF, deadband_dps=GPS_DB, ref_dps=GPS_REF))
    new_ff = np.array(new_ff)
    gps_ff = np.array(gps_ff)
    viol = int((np.abs(new_ff) > np.abs(gps_ff) + 1e-6).sum())
    print(f"\n[strength] DR delta_ff recomputed under current config (ref={GPS_REF}, db={GPS_DB}):")
    print(f"  |cmd| dps    mean={np.abs(cmd).mean():.1f} med={np.median(np.abs(cmd)):.1f} "
          f"p90={np.quantile(np.abs(cmd),0.9):.1f} max={np.abs(cmd).max():.1f}")
    print(f"  |delta_ff|   mean={np.abs(new_ff).mean():.1f} med={np.median(np.abs(new_ff)):.1f} "
          f"max={np.abs(new_ff).max():.1f}")
    print(f"  DR>GPS_CLOSED violations: {viol}/{len(dr)}  (want 0)")
    if viol:
        rc = 1

    # 3) authority health
    capped = float((np.abs(new_ff) >= CAP - 1e-6).mean() * 100)
    dead = float(((np.abs(cmd) >= 1.0) & (np.abs(cmd) < GPS_DB)).mean() * 100)
    nonzero = float((np.abs(new_ff) > 1e-6).mean() * 100)
    v = _num(dr, "nav_V")
    below_floor = float((v < config.L1_STEER_V_FLOOR_MPS).mean() * 100)
    print(f"\n[authority] cap({CAP}) engaged: {capped:.1f}% (high% = bang-bang)")
    print(f"  nonzero-FF rows: {nonzero:.1f}%   deadband-killed (1<=|cmd|<{GPS_DB}): {dead:.1f}%")
    print(f"  DR nav_V med={v.median():.2f}  below V_FLOOR({config.L1_STEER_V_FLOOR_MPS}): {below_floor:.1f}%"
          f"   <- steering leans on V_FLOOR when high")

    print(f"\nRESULT: {'PASS' if rc == 0 else 'FAIL'} (DR<=GPS invariant + direction)")
    return rc


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else DEFAULT))
