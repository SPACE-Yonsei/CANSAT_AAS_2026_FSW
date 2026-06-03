# Guidance Replay Summary

- Input rows: 7426  (state 3+4: 4747 rows)
- **State 3+4 duration**: 239.1 s
- Origin lock: t=2072.227  source=LATE_GPS
- First DR mode (replay): t=2072.227

## 1. Mode duration — BEFORE (logged) vs AFTER (replay), state 3+4

| Category | BEFORE s (%) | AFTER s (%) |
|---|---|---|
| FAIL | 224.7 (94.0%) | 224.7 (94.0%) |
| GPS_TRACKING | 4.6 (1.9%) | 4.6 (1.9%) |
| DR_M | 0.0 (0.0%) | 1.0 (0.4%) |
| DR_PM | 0.0 (0.0%) | 8.8 (3.7%) |
| DETUMBLING | 0.0 (0.0%) | 0.0 (0.0%) |
| DR_(legacy, before only) | 9.8 (4.1%) | — |

## 2. AFTER mode breakdown (state 3+4)

| Mode | duration s | % |
|---|---|---|
| FAIL | 224.7 | 94.0% |
| DR_PM_GBA_CLOSED | 8.8 | 3.7% |
| GPS_TRACKING_OPEN | 3.7 | 1.6% |
| DR_M_GBA_CLOSED | 1.0 | 0.4% |
| GPS_TRACKING_CLOSED | 0.9 | 0.4% |

## 3. FAIL / invalid reason duration (state 3+4)

| reason | duration s | % |
|---|---|---|
| NO_ORIGIN | 210.1 | 87.9% |
| NO_GUIDANCE_SOURCE | 14.7 | 6.1% |
| NAV_INVALID | 0.2 | 0.1% |

## 4. L1 valid ratio (state 3+4)

- L1Input valid: 14.1 s (5.9%)
- L1Output valid: 14.1 s (5.9%)

## 5. Saturation (state 3+4)

- yaw_rate_cmd saturated: 0.0%
- servo command saturated: 0.0%

## 6. Notes / remaining issues

- FAIL duration: BEFORE 224.7s → AFTER 224.7s (Δ +0.0s)
- DR (DR_M+DR_PM) duration AFTER: 9.8s (4.1%)
- See replay_guidance_result.csv for per-cycle detail.
