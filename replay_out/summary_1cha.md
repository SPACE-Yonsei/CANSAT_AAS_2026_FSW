# Guidance Replay Summary

- Input rows: 2804  (state 3+4: 993 rows)
- **State 3+4 duration**: 49.8 s
- Origin lock: t=2818.58  source=LATE_GPS
- First DR mode (replay): t=2818.881

## 1. Mode duration — BEFORE (logged) vs AFTER (replay), state 3+4

| Category | BEFORE s (%) | AFTER s (%) |
|---|---|---|
| FAIL | 12.3 (24.8%) | 15.5 (31.2%) |
| GPS_TRACKING | 0.0 (0.0%) | 0.0 (0.0%) |
| DR_M | 0.0 (0.0%) | 4.1 (8.3%) |
| DR_PM | 0.0 (0.0%) | 23.6 (47.3%) |
| DETUMBLING | 37.4 (75.2%) | 6.6 (13.2%) |
| DR_(legacy, before only) | 0.0 (0.0%) | — |

## 2. AFTER mode breakdown (state 3+4)

| Mode | duration s | % |
|---|---|---|
| DR_PM_GBA_CLOSED | 21.0 | 42.1% |
| FAIL | 15.5 | 31.2% |
| DETUMBLING | 6.6 | 13.2% |
| DR_M_GBA_CLOSED | 3.5 | 7.1% |
| DR_PM_YBA_OPEN | 2.6 | 5.2% |
| DR_M_YBA_OPEN | 0.6 | 1.2% |

## 3. FAIL / invalid reason duration (state 3+4)

| reason | duration s | % |
|---|---|---|
| NO_ORIGIN | 15.5 | 31.2% |
| DETUMBLING | 6.6 | 13.2% |

## 4. L1 valid ratio (state 3+4)

- L1Input valid: 27.7 s (55.6%)
- L1Output valid: 27.7 s (55.6%)

## 5. Saturation (state 3+4)

- yaw_rate_cmd saturated: 0.0%
- servo command saturated: 0.0%

## 6. Notes / remaining issues

- FAIL duration: BEFORE 12.3s → AFTER 15.5s (Δ +3.2s)
- DR (DR_M+DR_PM) duration AFTER: 27.7s (55.6%)
- See replay_guidance_result.csv for per-cycle detail.
