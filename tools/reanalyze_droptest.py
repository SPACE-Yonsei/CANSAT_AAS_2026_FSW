"""
Drop-test re-analysis with GPS outlier rejection.

Usage:
    python tools/reanalyze_droptest.py \
        --baro  path/to/barometer.txt \
        --gps   path/to/gps.csv \
        --imu   path/to/imu.txt

Expected file formats (same as 2026-04-05 drop test):
  barometer.txt  tab-sep: timestamp  alt_m  temp_c  press_hpa  alt_raw_m  alt_cal_m
  gps.csv        comma:   timestamp, lat, lon, speed_ms, course_deg, sats
  imu.txt        tab-sep: timestamp  roll pitch yaw  gyrx gyry gyrz  ax ay az  mx my mz
"""

import argparse
import math
import sys
from pathlib import Path

# ── Constants ────────────────────────────────────────────────────────────────
KOREA_LAT = (33.0, 39.0)
KOREA_LON = (124.0, 132.0)
GPS_JUMP_MAX_SPEED = 30.0    # m/s — tighter than motorapp (200) for post-analysis
GPS_SPEED_MAX      = 15.0    # m/s — parafoil physical ceiling
MIN_SATS           = 4
GLIDE_ALT_MIN      = 100.0   # m  — altitude band for clean glide analysis
GLIDE_ALT_MAX      = 350.0   # m
WIND_LEARN_MIN_V   = 2.0     # m/s — minimum GPS speed for wind learning
EMA_ALPHA          = 0.1


def load_baro(path: Path) -> list[dict]:
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split('\t') if '\t' in line else line.split()
            if len(parts) < 2:
                continue
            try:
                rows.append({'ts': parts[0], 'alt': float(parts[1])})
            except ValueError:
                continue
    return rows


def load_gps(path: Path) -> list[dict]:
    rows = []
    prev_lat = prev_lon = prev_t = None
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split(',')
            if len(parts) < 5:
                continue
            try:
                ts   = parts[0].strip()
                lat  = float(parts[1])
                lon  = float(parts[2])
                spd  = float(parts[3])
                crs  = float(parts[4])
                sats = int(parts[5]) if len(parts) > 5 else 0
            except ValueError:
                continue

            # validity checks
            if lat == 0.0 or lon == 0.0:
                continue
            if not (KOREA_LAT[0] <= lat <= KOREA_LAT[1]):
                continue
            if not (KOREA_LON[0] <= lon <= KOREA_LON[1]):
                continue
            if sats < MIN_SATS:
                continue

            # jump check using implied speed between consecutive fixes
            if prev_lat is not None:
                dist = _haversine(prev_lat, prev_lon, lat, lon)
                # use 1 s as nominal dt when timestamp parsing is unavailable
                if dist > GPS_JUMP_MAX_SPEED * 2.0:  # allow 2 s for max-speed jump
                    prev_lat, prev_lon = lat, lon
                    continue

            # speed plausibility
            if spd > GPS_SPEED_MAX:
                spd = GPS_SPEED_MAX

            rows.append({'ts': ts, 'lat': lat, 'lon': lon, 'spd': spd, 'crs': crs, 'sats': sats})
            prev_lat, prev_lon = lat, lon
    return rows


def load_imu(path: Path) -> list[dict]:
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split('\t') if '\t' in line else line.split()
            if len(parts) < 7:
                continue
            try:
                rows.append({
                    'ts':   parts[0],
                    'yaw':  float(parts[3]),
                    'gyrz': float(parts[6]),
                })
            except (ValueError, IndexError):
                continue
    return rows


def _haversine(lat1, lon1, lat2, lon2) -> float:
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))


def _wrap_180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def analyze(baro_rows, gps_rows, imu_rows):
    print("\n" + "="*60)
    print("  DROP TEST RE-ANALYSIS (GPS outlier-rejected)")
    print("="*60)

    # ── Sink rate from barometer ──────────────────────────────────
    alts = [r['alt'] for r in baro_rows]
    # find indices where alt is in GLIDE band
    glide_alts = [(i, a) for i, a in enumerate(alts) if GLIDE_ALT_MIN < a < GLIDE_ALT_MAX]
    v_sinks = []
    for i in range(1, len(glide_alts)):
        i1, a1 = glide_alts[i-1]
        i2, a2 = glide_alts[i]
        if i2 - i1 > 20:  # gap too large
            continue
        dt = (i2 - i1) * 0.1  # assume 10 Hz baro
        if dt > 0:
            vs = (a1 - a2) / dt
            if 0.5 < vs < 10.0:
                v_sinks.append(vs)

    if v_sinks:
        v_sink_mean = sum(v_sinks) / len(v_sinks)
        v_sink_std  = math.sqrt(sum((v - v_sink_mean)**2 for v in v_sinks) / len(v_sinks))
        print(f"\n[SINK RATE]")
        print(f"  V_sink = {v_sink_mean:.2f} ± {v_sink_std:.2f} m/s  (n={len(v_sinks)} windows)")
    else:
        v_sink_mean = None
        print("\n[SINK RATE] insufficient barometer data in glide band")

    # ── Horizontal speed from outlier-rejected GPS ────────────────
    print(f"\n[GPS QUALITY]  valid rows after outlier rejection: {len(gps_rows)}")
    if gps_rows:
        speeds = [r['spd'] for r in gps_rows]
        spd_mean = sum(speeds) / len(speeds)
        spd_sorted = sorted(speeds)
        spd_median = spd_sorted[len(spd_sorted)//2]
        spd_std = math.sqrt(sum((s - spd_mean)**2 for s in speeds) / len(speeds))
        print(f"\n[HORIZONTAL SPEED]")
        print(f"  mean   = {spd_mean:.2f} m/s")
        print(f"  median = {spd_median:.2f} m/s  ← use this for L1 stability condition")
        print(f"  std    = {spd_std:.2f} m/s")
        print(f"  L1 stability threshold: L1 > 2 × {spd_median:.1f} × 2.0 = {2*spd_median*2:.1f} m")
        v_h = spd_median
    else:
        v_h = None
        print("\n[HORIZONTAL SPEED] no valid GPS rows")

    # ── Glide ratio ───────────────────────────────────────────────
    if v_h is not None and v_sink_mean is not None and v_sink_mean > 0:
        glide = v_h / v_sink_mean
        print(f"\n[GLIDE RATIO]")
        print(f"  G = V_h / V_sink = {v_h:.2f} / {v_sink_mean:.2f} = {glide:.2f}")
        print(f"  (REF-5 reference: ~2.8 for small canopy)")
    else:
        glide = None
        print("\n[GLIDE RATIO] insufficient data")

    # ── Wind estimation (crab angle) ─────────────────────────────
    # Match GPS course timestamps with nearest IMU yaw timestamps
    if gps_rows and imu_rows:
        imu_by_ts = {r['ts'][:8]: r for r in imu_rows}  # key by HH:MM:SS

        crab_angles = []
        for g in gps_rows:
            if g['spd'] < WIND_LEARN_MIN_V:
                continue
            ts_key = g['ts'][:8] if len(g['ts']) >= 8 else g['ts']
            imu = imu_by_ts.get(ts_key)
            if imu is None:
                continue
            crab = _wrap_180(g['crs'] - imu['yaw'])
            crab_angles.append(crab)

        if crab_angles:
            # EMA wind estimate
            wind = 0.0
            for c in crab_angles:
                wind = (1 - EMA_ALPHA) * wind + EMA_ALPHA * c

            crab_mean = sum(crab_angles) / len(crab_angles)
            crab_std  = math.sqrt(sum((c - crab_mean)**2 for c in crab_angles) / len(crab_angles))
            print(f"\n[WIND ESTIMATION]")
            print(f"  EMA wind_effect = {wind:.1f}° (crab angle)")
            print(f"  crab mean = {crab_mean:.1f}° ± {crab_std:.1f}°  (n={len(crab_angles)})")
            if v_h is not None:
                w_spd = abs(v_h * math.sin(math.radians(crab_mean)))
                print(f"  Estimated wind speed ≈ {w_spd:.1f} m/s  (V_h × sin(crab))")
                if v_h > 0:
                    print(f"  V_wind/V_air ratio ≈ {w_spd/v_h:.2f}  (>0.7 = high-wind mode needed)")
        else:
            print("\n[WIND ESTIMATION] no GPS+IMU overlap with speed > threshold")

    # ── Natural yaw tendency ──────────────────────────────────────
    if imu_rows:
        gyrz_vals = [math.degrees(r['gyrz']) if abs(r['gyrz']) < math.pi else r['gyrz']
                     for r in imu_rows]
        gyrz_filt = [g for g in gyrz_vals if abs(g) < 30.0]
        if gyrz_filt:
            gyrz_mean = sum(gyrz_filt) / len(gyrz_filt)
            gyrz_std  = math.sqrt(sum((g - gyrz_mean)**2 for g in gyrz_filt) / len(gyrz_filt))
            print(f"\n[NATURAL YAW TENDENCY]")
            print(f"  mean gyrz = {gyrz_mean:.2f} deg/s ± {gyrz_std:.2f}")
            if abs(gyrz_mean) > 0.5:
                direction = "CW" if gyrz_mean > 0 else "CCW"
                print(f"  → Consistent {direction} tendency — consider adding trim offset")

    # ── Summary for code tuning ───────────────────────────────────
    print(f"\n{'='*60}")
    print("  PARAMETER RECOMMENDATIONS FOR CODE")
    print(f"{'='*60}")
    if v_sink_mean:
        print(f"  V_sink  = {v_sink_mean:.2f} m/s  → LANDING_ALT={v_sink_mean*6:.0f}m gives ~6s window")
    if v_h:
        l1_min = 2 * v_h * 2.0
        l1_rec = max(30.0, math.ceil(l1_min / 5) * 5)
        print(f"  V_h     = {v_h:.2f} m/s  → L_DISTANCE_BASE ≥ {l1_min:.0f}m → recommend {l1_rec:.0f}m")
    if glide:
        print(f"  G       = {glide:.2f}    → cross-range reach = G × release_alt")


def main():
    parser = argparse.ArgumentParser(description="Drop-test re-analysis with GPS outlier rejection")
    parser.add_argument('--baro', required=True, help='Path to barometer.txt')
    parser.add_argument('--gps',  required=True, help='Path to gps.csv')
    parser.add_argument('--imu',  required=True, help='Path to imu.txt')
    args = parser.parse_args()

    baro_rows = load_baro(Path(args.baro))
    gps_rows  = load_gps(Path(args.gps))
    imu_rows  = load_imu(Path(args.imu))

    print(f"Loaded: baro={len(baro_rows)} imu={len(imu_rows)} gps(clean)={len(gps_rows)}")
    analyze(baro_rows, gps_rows, imu_rows)


if __name__ == '__main__':
    main()
