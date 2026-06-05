"""state 0 -> 5 를 2분(120초, 1 Hz) 동안 걷는 SIMP 기압 파일 생성 + FSW 상태머신 검증.

두 가지 포맷을 출력한다:
  1) sim_pressure_0to5_2min.csv             - 커스텀 GCS 재생기용 (압력 Pa 값만)
  2) sim_pressure_0to5_2min_serialstudio.txt - Serial Studio/범용 송신용 (완성 명령 프레임)

가정: SIM 시작 전 CAL(지상)로 PREV_ALT_CAL ~= 0 -> alt_relative == ISA(p_pa).
"""

# --- ISA 역변환: 상대고도(m) -> 절대기압(Pa) (commapp._pressure_pa_to_alt_m 의 역) ---
def alt_to_pa(alt_m: float) -> float:
    return 101325.0 * (1.0 - alt_m * 2.25577e-5) ** 5.25588


# --- 고도 프로파일 (120 샘플 = 120초 @1Hz) ---
def altitude_at(t: int) -> float:
    if t <= 4:                      # 0..4  : 지상 (LAUNCH_PAD, 여기서 CAL)
        return 0.0
    if t <= 35:                     # 5..35 : 상승 0 -> 500 m (~16 m/s)
        return (t - 5) / 30.0 * 500.0
    if t <= 52:                     # 36..52: 아포지 부근 완만 하강 500 -> 415 (~5 m/s)
        return 500.0 - (t - 36) * 5.0
    if t <= 95:                     # 53..95: 본 하강 415 -> ~40 m (~8.7 m/s)
        return 415.0 - (t - 52) * 8.7
    return max(12.0, 40.0 - (t - 95) * 1.0)   # 96..119: 40 -> 16 m 부근 유지 (state 5 체류)


ALTS = [altitude_at(t) for t in range(120)]


# --- FSW 상태머신 충실 재현 (flightlogicapp.barometer_logic + release TARGET_RATIO 0.8) ---
def simulate(alts):
    state = 0
    max_alt = 0.0
    recent = []
    baseline = None
    cnt_asc = cnt_apo = cnt_rel = cnt_egg = 0
    cal_done = True                 # CAL 이미 수행했다고 가정
    log = []
    for t, alt in enumerate(alts):
        recent.append(alt)
        recent = recent[-3:]
        if len(recent) >= 2:
            sw = sorted(recent, reverse=True)
            filtered = sw[1]
            max_alt = max(max_alt, filtered)
        else:
            filtered = alt
            max_alt = max(max_alt, alt)

        def rel_cond(a):
            return max_alt > 0 and a <= max_alt * 0.8   # TARGET_RATIO 하드 트리거

        if state == 0:
            if cal_done:
                if baseline is None:
                    baseline = alt
                cnt_asc = cnt_asc + 1 if (alt - baseline) > 100 else 0
                if cnt_asc >= 3:
                    cnt_asc = cnt_apo = cnt_rel = cnt_egg = 0
                    state = 1
        elif state == 1:
            rc = rel_cond(filtered)
            apo = max_alt > 0 and (max_alt * 0.8 < alt < max_alt - 0.25)
            cnt_rel = cnt_rel + 1 if rc else 0
            cnt_apo = cnt_apo + 1 if apo else 0
            if cnt_rel >= 3:
                cnt_asc = cnt_apo = cnt_rel = cnt_egg = 0; state = 4
            elif cnt_apo >= 2:
                cnt_asc = cnt_apo = cnt_rel = cnt_egg = 0; state = 2
        elif state == 2:
            cnt_apo += 1
            if cnt_apo >= 15:
                cnt_asc = cnt_apo = cnt_rel = cnt_egg = 0; state = 3
        elif state == 3:
            rc = rel_cond(filtered)
            cnt_rel = cnt_rel + 1 if rc else 0
            if cnt_rel >= 3:
                cnt_asc = cnt_apo = cnt_rel = cnt_egg = 0; state = 4
        elif state == 4:
            cnt_rel = cnt_rel + 1 if alt <= 50 else 0
            if cnt_rel >= 3:
                cnt_asc = cnt_apo = cnt_rel = cnt_egg = 0; state = 5
        log.append((t, alt, max_alt, state))
    return log


def main():
    log = simulate(ALTS)
    print("=== state transition check ===")
    prev = None
    for t, alt, mx, st in log:
        if st != prev:
            print(f"  t={t:3d}s  alt={alt:6.1f}m  max={mx:6.1f}m  -> state {st}")
            prev = st
    reached = sorted({st for *_, st in log})
    print(f"reached states: {reached}, final state={log[-1][3]}")
    assert reached[:6] == [0, 1, 2, 3, 4, 5], f"0~5 transition failed: {reached}"
    print("OK: state 0->1->2->3->4->5 all passed\n")

    import os
    here = os.path.dirname(__file__)

    # 1) 커스텀 GCS 재생기용: 압력값(Pa)만 줄당 1개 (GCS가 SIMP로 감쌈)
    out_gcs = os.path.join(here, "sim_pressure_0to5_2min.csv")
    with open(out_gcs, "w", encoding="utf-8") as f:
        f.write("# CANSAT SIM 기압 프로파일: state 0->5, 120s @1Hz\n")
        f.write("# GCS 'SIM 기압 파일 열기'로 로드 후 재생. 값=절대기압(Pa).\n")
        f.write("# 사용 전: CX,ON -> SIM,ENABLE -> SIM,ACTIVATE -> CAL (지상)\n")
        f.write("# pressure_pa\n")
        for alt in ALTS:
            f.write(f"{alt_to_pa(alt):.0f}\n")
    print(f"GCS file written: {out_gcs} ({len(ALTS)} lines)")

    # 2) Serial Studio / 범용 송신용: 완성된 명령 프레임을 줄당 1개 (주석 없음, 1Hz로 전송)
    out_ss = os.path.join(here, "sim_pressure_0to5_2min_serialstudio.txt")
    with open(out_ss, "w", encoding="utf-8") as f:
        for alt in ALTS:
            f.write(f"CMD,1070,SIMP,{alt_to_pa(alt):.0f}\n")
    print(f"Serial Studio file written: {out_ss} ({len(ALTS)} lines)")


if __name__ == "__main__":
    main()
