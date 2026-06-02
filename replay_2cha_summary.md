# Guidance Replay Summary

- Input rows: 2267  (state 3+4: 945 rows)
- **State 3+4 duration**: 47.4 s
- Origin lock: t=17321.128  source=LATE_GPS
- First DR mode (replay): t=17321.128

## 1. Mode duration — BEFORE (logged) vs AFTER (replay), state 3+4

| Category | BEFORE s (%) | AFTER s (%) |
|---|---|---|
| FAIL | 11.2 (23.6%) | 9.6 (20.3%) |
| GPS_TRACKING | 9.8 (20.7%) | 2.2 (4.6%) |
| DR_M | 0.0 (0.0%) | 6.5 (13.7%) |
| DR_PM | 0.0 (0.0%) | 18.6 (39.2%) |
| DETUMBLING | 0.9 (1.8%) | 10.5 (22.2%) |
| DR_(legacy, before only) | 25.6 (53.9%) | — |

## 2. AFTER mode breakdown (state 3+4)

| Mode | duration s | % |
|---|---|---|
| DR_PM_GBA_CLOSED | 18.6 | 39.2% |
| DETUMBLING | 10.5 | 22.2% |
| FAIL | 9.6 | 20.3% |
| DR_M_GBA_CLOSED | 6.5 | 13.7% |
| GPS_TRACKING_CLOSED | 2.2 | 4.6% |

## 3. FAIL / invalid reason duration (state 3+4)

| reason | duration s | % |
|---|---|---|
| DETUMBLING | 10.5 | 22.2% |
| NO_ORIGIN | 9.6 | 20.3% |
| NAV_INVALID | 0.8 | 1.6% |

## 4. L1 valid ratio (state 3+4)

- L1Input valid: 26.5 s (55.9%)
- L1Output valid: 26.5 s (55.9%)

## 5. Saturation (state 3+4)

- yaw_rate_cmd saturated: 0.0%
- servo command saturated: 0.0%

## 6. Notes / remaining issues

- FAIL duration: BEFORE 11.2s → AFTER 9.6s (Δ -1.6s)
- DR (DR_M+DR_PM) duration AFTER: 25.1s (52.9%)
- See replay_guidance_result.csv for per-cycle detail.
