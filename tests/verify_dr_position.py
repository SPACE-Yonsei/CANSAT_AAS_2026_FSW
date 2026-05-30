#!/usr/bin/env python3
"""DR 위치 누적 검증: 1사이클 큰 dt vs 320사이클 작은 dt 비교.

핵심 인사이트:
  gps_sample=drop을 해도 마지막 GPS 타임스탬프가 GPS_FRESH_MAX_AGE_S(15s) 동안 유효하다.
  그 구간 동안 nav 위치는 마지막 GPS 위치에 고정(frozen)된다.
  DR이 실제로 위치를 적분하는 건 GPS 신선도가 만료된 이후부터다.

3단계 구조:
  Phase 1. GPS 앵커 잠금 (1사이클)
  Phase 2. GPS 신선도 소진 (300사이클 × 0.05s = 15s, nav 위치 고정)
  Phase 3. 순수 DR 적분 (320사이클 × 0.05s = 16s) ← 여기서만 위치 누적됨
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.interactive_ctrl_parafoil_sim import (
    DEFAULT_INPUTS, SAMPLE_DROP, SAMPLE_SEND, Simulator,
)
from Sensor_Motor import guidance
from lib import config

ORIGIN_LAT = float(config.GPS_EXPECTED_LAT_CENTER_DEG)
ORIGIN_LON = float(config.GPS_EXPECTED_LON_CENTER_DEG)
TARGET_E   = 0.0
TARGET_N   = 900.0

DT          = 1.0 / float(config.MOTOR_RATE_HZ)   # 0.05s
GPS_STALE_S = float(config.GPS_FRESH_MAX_AGE_S)    # 15s
DR_RUN_S    = GPS_STALE_S + 1.0                    # 16s (GPS_FRESH_MAX_AGE_S + 1)

CYCLES_STALE = round(GPS_STALE_S / DT)            # 300 사이클 (GPS 신선도 소진)
CYCLES_DR    = round(DR_RUN_S / DT)               # 320 사이클 (순수 DR 구간)

SPEED_MPS    = 8.0
EXPECTED_DR_N = SPEED_MPS * DR_RUN_S              # 128m (이상적 DR 직진)


def make_sim() -> Simulator:
    inputs = dict(DEFAULT_INPUTS)
    inputs["speed_mps"] = SPEED_MPS
    inputs["course_deg"] = 0.0
    inputs["yaw_deg"] = 0.0
    inputs["gyrz_nav_deg_s"] = 0.0
    return Simulator(ORIGIN_LAT, ORIGIN_LON, TARGET_E, TARGET_N, inputs)


def run_cycles(sim: Simulator, n: int, gps_sample: str,
               imu_health: bool = True) -> None:
    for _ in range(n):
        sim.run_cycle({
            "dt_s": DT,
            "gps_sample": gps_sample,
            "gps_pos_ok": True, "gps_motion_ok": True,
            "imu_sample": SAMPLE_SEND, "imu_health": imu_health,
            "gyrz_nav_deg_s": 0.0,
        })


def run() -> None:
    print("=" * 62)
    print("DR 위치 누적 검증 (3단계 프로토콜)")
    print(f"속도={SPEED_MPS}m/s  DT={DT}s  GPS_FRESH={GPS_STALE_S}s")
    print(f"Phase2 (GPS 신선도 소진): {CYCLES_STALE}사이클 × {DT}s = {CYCLES_STALE*DT:.0f}s")
    print(f"Phase3 (순수 DR): {CYCLES_DR}사이클 × {DT}s = {CYCLES_DR*DT:.0f}s")
    print(f"이상적 DR 예상 N: {EXPECTED_DR_N:.2f}m")
    print("=" * 62)

    sim = make_sim()

    # ── Phase 1: GPS 앵커 잠금 (1사이클) ──────────────────────────────────────
    sim.run_cycle({
        "dt_s": DT,
        "gps_sample": SAMPLE_SEND, "gps_pos_ok": True, "gps_motion_ok": True,
        "imu_sample": SAMPLE_SEND, "imu_health": True,
    })
    st = guidance._STATE_t
    dr = st.dr
    print(f"\n[Phase1] GPS 앵커 잠금")
    print(f"  mode={st.nav.control_mode.value}  anchor(E={dr.anchor_E:.2f}, N={dr.anchor_N:.2f})"
          f"  V={dr.anchor_V:.1f}m/s  course={math.degrees(dr.anchor_course):.1f}deg")

    # ── Phase 2: GPS 신선도 소진 (nav 위치 고정 구간) ─────────────────────────
    n_samples: list[float] = []
    modes: set[str] = set()
    for i in range(CYCLES_STALE):
        sim.run_cycle({
            "dt_s": DT,
            "gps_sample": SAMPLE_DROP,
            "imu_sample": SAMPLE_SEND, "imu_health": True,
            "gyrz_nav_deg_s": 0.0,
        })
        st2 = guidance._STATE_t
        modes.add(st2.nav.control_mode.value)
        n_samples.append(st2.nav.N)

    n_at_stale = guidance._STATE_t.nav.N
    mode_str = "/".join(sorted(modes))
    print(f"\n[Phase2] GPS 신선도 소진 ({CYCLES_STALE}사이클)")
    print(f"  활성 모드: {mode_str}")
    print(f"  nav.N 범위: {min(n_samples):.2f}~{max(n_samples):.2f}m  "
          f"(고정={abs(max(n_samples)-min(n_samples))<0.01})")
    print(f"  Phase2 종료 시점 nav.N={n_at_stale:.2f}m  "
          f"conf={guidance._STATE_t.nav.confidence:.2f}")

    # ── Phase 3: 순수 DR 적분 (실비행 재현) ──────────────────────────────────
    n_start = guidance._STATE_t.nav.N
    conf_start = guidance._STATE_t.nav.confidence
    dr_ns: list[float] = []
    dr_confs: list[float] = []
    dr_modes: set[str] = set()

    for i in range(CYCLES_DR):
        sim.run_cycle({
            "dt_s": DT,
            "gps_sample": SAMPLE_DROP,
            "imu_sample": SAMPLE_SEND, "imu_health": True,
            "gyrz_nav_deg_s": 0.0,
        })
        st3 = guidance._STATE_t
        dr_ns.append(st3.nav.N)
        dr_confs.append(st3.nav.confidence)
        dr_modes.add(st3.nav.control_mode.value)

    n_end   = guidance._STATE_t.nav.N
    delta_n = n_end - n_start
    conf_end = guidance._STATE_t.nav.confidence
    error   = abs(delta_n - EXPECTED_DR_N)
    pct     = delta_n / EXPECTED_DR_N * 100 if EXPECTED_DR_N else float("nan")
    sim.restore()

    print(f"\n[Phase3] 순수 DR 적분 ({CYCLES_DR}사이클 = {CYCLES_DR*DT:.0f}s)")
    print(f"  활성 모드: {'/'.join(sorted(dr_modes))}")
    print(f"  conf: {conf_start:.2f} → {conf_end:.2f}")
    print(f"  nav.N: {n_start:.2f}m → {n_end:.2f}m  (ΔN={delta_n:.2f}m)")
    print(f"  이상적 예측: {EXPECTED_DR_N:.2f}m")
    print(f"  오차: {error:.2f}m  ({100-pct:.1f}% 부족)" if pct < 100
          else f"  오차: {error:.2f}m  ({pct-100:.1f}% 초과)")

    print()
    print("=" * 62)
    print("결론")
    print("=" * 62)
    if error < 2.0:
        print(f"[OK] 20Hz 다중 사이클 DR 누적 정확도 확인: dN={delta_n:.2f}m ~ {EXPECTED_DR_N:.0f}m")
    else:
        print(f"[!!] dN={delta_n:.2f}m vs 이상값={EXPECTED_DR_N:.0f}m -- 원인 분석 필요")

    print()
    print("★ Phase2 핵심 동작 (GPS freshness 소진 구간):")
    print(f"  GPS가 끊겨도 {GPS_STALE_S:.0f}s 동안 nav 위치가 마지막 GPS fix에 고정.")
    print(f"  이 구간에서 CanSat은 실제로 약 {SPEED_MPS*GPS_STALE_S:.0f}m 이동하지만")
    print(f"  nav 추정값은 {min(n_samples):.1f}m~{max(n_samples):.1f}m에 머뭄.")
    print(f"  GPS freshness 만료 후 DR이 마지막 anchor에서 적분을 시작하므로")
    print(f"  실제 위치 대비 최대 ~{SPEED_MPS*GPS_STALE_S:.0f}m 오차 발생 가능.")


if __name__ == "__main__":
    run()
