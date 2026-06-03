# Guidance Replay Summary

- Input rows: 2267  (state 3+4: 945 rows)
- **State 3+4 duration**: 47.4 s
- Origin lock: t=17321.128  source=LATE_GPS
- First DR mode (replay): t=17321.128

## 1. Mode duration — BEFORE (logged) vs AFTER (replay), state 3+4

| Category | BEFORE s (%) | AFTER s (%) |
|---|---|---|
| FAIL | 11.2 (23.6%) | 10.3 (21.7%) |
| GPS_TRACKING | 9.8 (20.7%) | 2.8 (5.9%) |
| DR_M | 0.0 (0.0%) | 6.9 (14.5%) |
| DR_PM | 0.0 (0.0%) | 22.2 (46.8%) |
| DETUMBLING | 0.9 (1.8%) | 5.3 (11.1%) |
| DR_(legacy, before only) | 25.6 (53.9%) | — |

## 2. AFTER mode breakdown (state 3+4)

| Mode | duration s | % |
|---|---|---|
| DR_PM_GBA_CLOSED | 18.6 | 39.2% |
| FAIL | 10.3 | 21.7% |
| DR_M_GBA_CLOSED | 6.5 | 13.7% |
| DETUMBLING | 5.3 | 11.1% |
| DR_PM_YBA_OPEN | 3.6 | 7.6% |
| GPS_TRACKING_CLOSED | 2.2 | 4.6% |
| GPS_TRACKING_OPEN | 0.7 | 1.4% |
| DR_M_YBA_OPEN | 0.4 | 0.7% |

## 3. FAIL / invalid reason duration (state 3+4)

| reason | duration s | % |
|---|---|---|
| NO_ORIGIN | 10.3 | 21.7% |
| DETUMBLING | 5.3 | 11.1% |
| NAV_INVALID | 0.5 | 1.1% |

## 4. L1 valid ratio (state 3+4)

- L1Input valid: 31.4 s (66.1%)
- L1Output valid: 31.4 s (66.1%)

## 5. Saturation (state 3+4)

- yaw_rate_cmd saturated: 0.0%
- servo command saturated: 0.0%

## 6. Notes / remaining issues

- FAIL duration: BEFORE 11.2s → AFTER 10.3s (Δ -0.9s)
- DR (DR_M+DR_PM) duration AFTER: 29.1s (61.3%)
- See replay_guidance_result.csv for per-cycle detail.
