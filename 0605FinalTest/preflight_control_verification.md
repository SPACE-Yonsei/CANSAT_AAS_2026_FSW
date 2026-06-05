# Preflight Control Verification

## Passed SIL checks

These tests use the production `Sensor_Motor.guidance`, `Sensor_Motor.control`,
and `Sensor_Motor.motorapp` modules directly.

- `0605FinalTest/test_control_mode_matrix.py`: exhaustive fresh/stale/invalid mode
  reachability matrix.
- `0605FinalTest/test_l1_geometry_matrix.py`: L1 `nu` sign, deadband, yaw-rate limit,
  and speed guard/clamp behavior.
- `0605FinalTest/test_control_command_grid.py`: yaw-rate command grid, FF/PID sign,
  per-mode gain selection, servo angle direction, and FAIL neutral output.
- `0605FinalTest/test_freshness_boundaries.py`: GPS, IMU, and barometer freshness
  threshold boundaries at `threshold +/- 0.01 s`.
- `0605FinalTest/test_motorapp_payload_integration.py`: real motorapp payload parsing,
  IMU `gyrz` sign inversion, linear acceleration validity, and FakePi servo
  output direction.
- `0605FinalTest/test_flight_log_replay.py`: replays an available real motor control
  log through motorapp handlers and the production guidance/control cycle.

## Replay SIL

Tool:

```bash
python 0605FinalTest/replay_flight_log.py
```

Default replay input:

- `motorlogs/motor_control_20260604_161105.csv`

The repository currently does not contain a complete separate
`raw_gps/raw_imu/raw_barometer` set. The replay tool therefore supports the
embedded sensor columns in `motor_control_*.csv` and converts them back to the
current `motorapp.handle_gps`, `handle_imu`, and `handle_barometer` payload
formats. Hardware output is blocked with `FakePi`; motor raw logging is patched
inside tests.

Current default replay summary:

- cycles: 1
- state 4, distance <= 50 m, L1/control-valid cycles: 1
- mode distribution: `GPS_TRACKING_CLOSED: 1`
- saturation ratio: 0.0
- gyro rejected ratio: 0.0
- baro sink spike ratio: 0.0

## Closed-loop yaw-rate plant SIL

Tool:

```bash
python 0605FinalTest/sim_closed_loop_yaw_rate.py
```

Plant:

```text
tau * w_dot + w = K * delta_arm_deg + disturbance
```

Matrix:

- `K`: 0.32, 0.63, 1.0
- `tau`: 0.3, 0.5, 0.66 s
- disturbance: -15, 0, +15 dps
- command: -30, -15, +15, +30 dps
- mode: `GPS_TRACKING_CLOSED`, `DR_M_G_CLOSED`, `DR_PM_G_CLOSED`
- `dt=0.05 s`, `duration=10 s`

Current production gains do not pass the full acceptance matrix:

- total cases: 324
- `GPS_TRACKING_CLOSED`: 90 pass, 18 fail
- `DR_M_G_CLOSED`: 42 pass, 66 fail
- `DR_PM_G_CLOSED`: 48 pass, 60 fail
- diverged cases: 0
- failure reason: `FINAL_ERROR` in 144 cases
- `GPS_TRACKING_CLOSED` is not always faster than `DR_PM_G_CLOSED` under the
  current combined FF-scale and per-mode yaw-rate-limit behavior.

This is a no-go item for blind flight tuning. It does not imply the code is
crashing; it means the current gains/authority model do not meet the requested
tracking margin across the conservative plant/disturbance envelope.

## Go/no-go criteria

Go for bench/HIL only when:

- All mode matrix, L1 geometry, command grid, freshness, and payload integration
  tests pass.
- Real-log replay completes without exceptions.
- At least one state 4, distance <= 50 m control-valid replay cycle exists.
- Replay saturation ratio is below 30%.
- Replay yaw-rate command and arm deflection limits are never violated.
- Closed-loop plant simulation has zero divergence.

Go for flight only after bench/HIL also confirms:

- Servo PWM polarity and arm direction match the installed hardware.
- Positive `delta_arm_deg` turns the payload right in a suspended or low-risk
  tether test.
- IMU raw `gyrz` sign after `motorapp.handle_imu` matches the nav convention.
- Barometer sink sign and spike threshold match descent data.
- Closed-loop plant final-error failures are resolved or explicitly accepted by
  the team with a reduced flight envelope.

## Remaining bench/HIL items

1. Run `servo_hw_check.py` or an equivalent low-power servo bench check for
   GPIO 12/right and GPIO 13/left.
2. Command fixed positive and negative `delta_arm_deg` and record actual line
   pull direction.
3. Rotate the IMU by hand and verify raw `gyrz` sign, handler inversion, and
   control feedback sign.
4. Replay a longer real drop log containing synchronized GPS, IMU, barometer,
   and motor cycles.
5. Identify `K` and `tau` from bench yaw-rate response, then rerun
   `0605FinalTest/sim_closed_loop_yaw_rate.py`.
6. Retune PID/FF only after the measured plant envelope is known.
