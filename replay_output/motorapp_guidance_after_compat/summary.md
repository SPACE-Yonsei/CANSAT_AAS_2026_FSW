# Motorapp Guidance/Control Replay

## Replay Summary
| case | active ticks | valid control | origin | target | top mode | top reason/fail | |delta| p95/max |
|---|---:|---:|---|---|---|---|---:|
| 0405_no_target | 2344 | 0.0% | (37.250686, 126.674964) | None | [('DEGRADED_FEEDFORWARD', 855), ('FAIL', 522)] | [('NO_POSITION', 1954), ('NO_MOTION', 362)] / [('NO_POSITION', 1954), ('NO_MOTION', 362)] | nan/nan |
| 0405_final_target | 2344 | 82.8% | (37.250686, 126.674964) | (37.254138, 126.680626) | [('DEGRADED_FEEDFORWARD', 855), ('FAIL', 522)] | [('DEGRADED_FEEDFORWARD', 855), ('DEGRADED_CLOSED_LOOP', 446)] / [('NONE', 1822), ('NO_MOTION', 362)] | 77.59/128.17 |
| 0510_no_target | 11318 | 0.0% | (37.52476983333334, 126.61987766666668) | None | [('FAIL', 11318)] | [('DR_TIMEOUT', 6821), ('NO_MOTION', 4387)] / [('DR_TIMEOUT', 6821), ('NO_MOTION', 4387)] | nan/nan |
| 0510_logged_target | 11318 | 38.2% | (37.52476983333334, 126.61987766666668) | (37.522207, 126.618779) | [('FAIL', 11318)] | [('DR_TIMEOUT', 6821), ('NO_MOTION', 4387)] / [('DR_TIMEOUT', 6821), ('NO_MOTION', 4387)] | 65.00/65.00 |
| 0510_final_target | 11318 | 38.2% | (37.52476983333334, 126.61987766666668) | (37.52544483333334, 126.6194795) | [('FAIL', 11318)] | [('DR_TIMEOUT', 6821), ('NO_MOTION', 4387)] / [('DR_TIMEOUT', 6821), ('NO_MOTION', 4387)] | 65.00/65.00 |

## Freshness Observations
| log | sensor | valid samples | mean gap | p95 gap | max gap | fresh age | gaps over fresh |
|---|---|---:|---:|---:|---:|---:|---:|
| 0405 | gps_pos | 120 | 1.187 | 3.005 | 5.010 | 1.500 | 8 |
| 0405 | gps_motion | 28 | 5.010 | 12.424 | 17.038 | 1.500 | 27 |
| 0405 | imu | 143 | 1.002 | 1.003 | 1.005 | 0.300 | 142 |
| 0405 | baro | 143 | 1.002 | 1.003 | 1.005 | 0.750 | 142 |
| 0510 | gps_pos | 4245 | 0.133 | 0.072 | 350.985 | 1.500 | 1 |
| 0510 | gps_motion | 0 | nan | nan | nan | 1.500 | 0 |
| 0510 | imu | 11199 | 0.050 | 0.095 | 0.102 | 0.300 | 0 |
| 0510 | baro | 9200 | 0.061 | 0.096 | 55.924 | 0.750 | 13 |

## Notes
- Cases ending in no_target replay the logs without injecting target coordinates.
- Cases ending in final_target inject the final valid GPS coordinate as a synthetic target so the current L1/control path can be exercised.
- 0405 Communication telemetry is only 1 Hz, so it is stricter than the production 10 Hz GPS/baro and 10 Hz IMU sender behavior.
- 0510 logs use older Motor message formats; this replay adapts those fields to the current data classes.
