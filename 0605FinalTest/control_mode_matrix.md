# Control Mode Matrix SIL

## How to run

From the repository root:

```bash
python 0605FinalTest/sim_control_modes.py
pytest -q 0605FinalTest/test_control_mode_matrix.py
pytest -q 0605FinalTest/test_motorapp_integration.py
```

`0605FinalTest/sim_control_modes.py` writes
`0605FinalTest/control_mode_matrix.csv` and prints mode counts, unreachable
modes, and FAIL reason counts.

## CSV columns

Case inputs:

- `origin_ready`, `target_ready`
- `gps_pos`, `gps_motion`
- `imu_gyrz`, `imu_yaw`
- `baro_sink`, `acc`
- `dr_ready`, `dr_old`
- `gyro_spike`, `baro_spike`

Pipeline outputs:

- `mode`, `fail_reason`
- `gps_pos_fresh`, `gps_motion_fresh`
- `imu_gyrz_fresh`, `imu_yaw_fresh`
- `baro_sink_fresh`, `acc_fresh`
- `dr_anchor_valid`, `dr_current_valid`, `nav_valid`
- `l1_in_valid`, `l1_in_reason`
- `l1_out_valid`, `l1_out_reason`
- `yaw_rate_cmd_dps`, `yaw_rate_limit_dps`
- `ctrl_valid`, `ctrl_reason`
- `delta_arm_deg`, `delta_ff_deg`, `delta_pid_deg`
- `kp_used`, `ff_scale`
- `left_pw`, `right_pw`
- `saturated`, `gyro_rejected`
- `baro_sink_spike`, `speed_clamped`, `dr_speed_source`
- `pid_enabled`

## Mode reachability summary

The current matrix reaches every `guidance.ControlMode` enum value.

- `GPS_TRACKING_CLOSED`: origin and target are set, GPS position and motion are
  fresh, and gyro-z is fresh and below the DR control yaw-rate guard.
- `GPS_TRACKING_OPEN`: origin and target are set, GPS position and motion are
  fresh, but gyro-z is missing or rejected by the DR control yaw-rate guard.
- `DR_M_GBA_CLOSED`: GPS position is fresh, GPS motion is stale, DR current is
  valid, gyro-z, barometer sink, and acceleration are fresh.
- `DR_M_GB_CLOSED`: same as `DR_M_GBA_CLOSED`, but acceleration is not fresh.
- `DR_M_G_CLOSED`: same GPS/DR/gyro path, with speed taken from last DR speed
  because barometer sink is stale or spike rejected.
- `DR_M_YBA_OPEN`: GPS position is fresh, GPS motion is stale, yaw, barometer
  sink, and acceleration are fresh, and gyro closed-loop feedback is unavailable.
- `DR_M_YB_OPEN`: same as `DR_M_YBA_OPEN`, but acceleration is not fresh.
- `DR_M_Y_OPEN`: same yaw path, with speed taken from last DR speed.
- `DR_PM_*`: same source suffix rules as `DR_M_*`, but GPS position is stale
  and both position and motion come from DR current propagation.
- `FAIL`: origin/target are missing, no guidance source is available, or the DR
  anchor is older than `config.DR_MAX_AGE_S`.

## FAIL reasons

The current exhaustive run reports these FAIL reasons:

- `NO_ORIGIN`: origin is not set.
- `NO_TARGET`: origin is set but target is not set.
- `NO_COURSE_SOURCE`: GPS position is fresh enough to seed DR, but no usable
  course source exists.
- `NO_GUIDANCE_SOURCE`: origin and target are set, but neither GPS tracking nor
  a valid DR fallback source can support guidance.
- `DR_TIMEOUT`: DR current exists, but the anchor age exceeds
  `config.DR_MAX_AGE_S`.

## Production import boundary

The SIL harness imports and calls the production modules directly:

- `lib.config`
- `Sensor_Motor.guidance`
- `Sensor_Motor.control`
- `Sensor_Motor.motorapp` for the integration test

The harness does not copy `DecideControlMode`,
`ProduceL1Input`, `ProduceL1Output`, `ProduceCtrlInput`, or
`ProduceCtrlOutput`. Test cases call these functions through qualified module
names after `importlib.reload()`.

## Hardware isolation

The matrix tests do not initialize pigpio and do not write servo hardware. The
motorapp integration test installs a `FakePi` object that records
`set_servo_pulsewidth()` calls. `sensorlog.log_motor_raw()` is monkeypatched to
avoid file I/O.

## Before flight

Verify these configuration items with flight data or bench tests:

- `KP_GPS_CLOSED`, `KI_GPS_CLOSED`, `KD_GPS_CLOSED`
- `KP_DR_M_CLOSED`, `KI_DR_M_CLOSED`, `KD_DR_M_CLOSED`
- `KP_DR_PM_CLOSED`, `KI_DR_PM_CLOSED`, `KD_DR_PM_CLOSED`
- `DR_MAX_AGE_S`
- `DR_MAX_YAW_RATE_DPS_FOR_CONTROL`
- `DR_BARO_SINK_MAX_MPS`
- Per-mode yaw-rate limits
- `GYRZ_SIGN`
- `DR_M_FF_SCALE`, `DR_PM_FF_SCALE`
- Servo zero pulse, neutral angle, min/max angle, and pulse-per-degree values
