# Manual sensor-cycle simulator

`tools/manual_sensor_cycle_sim.py` is a Software-In-the-Loop (SIL) harness for
the parafoil guidance/control pipeline. You feed it **sensor** inputs and it
runs the *production* `Sensor_Motor.guidance` + `Sensor_Motor.control` code
exactly once per cycle, so you can see — before flight — which control mode the
pipeline selects, whether it keeps pointing at the target, and how dead
reckoning (DR) behaves as GPS degrades.

---

## 1. Purpose

Answer these questions from a chosen sequence of sensor inputs:

1. Which control mode is auto-selected for a given sensor combination?
2. With healthy GPS, does the command point at the target?
3. When GPS *motion* drops, does it switch to `DR_M_*` and keep pointing?
4. When GPS *position* also drops, does it switch to `DR_PM_*` and accumulate DR?
5. As DR accumulates, how do `target_distance`, `nu`, `yaw_rate_cmd`, and the
   motor arm differential evolve?
6. Does the motor arm steer the payload toward the target?
7. How do cycles 1, 2, 3, and 10 differ?

The operator never selects a mode. The mode is an **output** of
`guidance.DecideControlMode`, derived purely from sensor freshness.

---

## 2. Two input layers: premise vs per-cycle

**A. Premise / simulation setup** (constant for the run, set from the CLI):
origin lat/lon, target E/N, initial payload E/N, initial course/speed/yaw/gyro,
initial baro alt/sink, `dt`, cycle count, sample cycles, output path, preset.

**B. Per-cycle sensor inputs** (editable every cycle): the health state
(`on`/`off`/`stale`) and values for GPS position, GPS motion, IMU yaw, IMU gyro,
IMU acceleration, and barometer. In interactive mode a blank line keeps the
previous cycle's inputs.

| Layer | Examples | When set |
| --- | --- | --- |
| A — premise | origin, target, dt, cycles | once (CLI / preset) |
| B — per cycle | `gps_pos`, `gps_motion`, `yaw`, `gyrz`, `baro`, `acc`, values | every cycle |

---

## 3. Why mode is never an input

`guidance.DecideControlMode(gps, imu, baro, now)` runs
`UpdateRaw → ComputeFreshFlags → FillNav` and **FillNav both fills the nav state
and picks the mode**. If a test injected the mode directly it would no longer be
testing the production selection logic — the exact thing we want to verify.
So this harness:

- calls `DecideControlMode` **exactly once** per cycle (the only mode source);
- never assigns `guidance._STATE_t.nav.control_mode`;
- never calls `SelectControlMode` or `FillNav` directly;
- never calls `DecideControlMode` twice in one cycle;
- never copies guidance/control internals into the harness.

---

## 4. One cycle pipeline

Each cycle runs, in order:

1. Build `gps`, `imu`, `baro` mocks (`SimpleNamespace`) from the cycle input.
2. `mode = guidance.DecideControlMode(gps, imu, baro, now)` — **once**.
3. Record `dr.current_E/N` **before** `ProduceL1Input`.
4. `l1_in = guidance.ProduceL1Input(now)`.
5. Record `dr.current_E/N` **after** `ProduceL1Input` (must be unchanged).
6. Produce output:
   - `if not l1_in.valid`: `l1_out = None`; `ctrl_out = control.WriteNeutral(now, mode)`.
   - else `l1_out = guidance.ProduceL1Output(l1_in)`:
     - `if not l1_out.valid`: `ctrl_out = control.WriteNeutral(now, mode)`.
     - else `ctrl_in = control.ProduceCtrlInput(l1_out, now)`;
       `gyrz_meas = cycle gyrz_dps if gyrz fresh & finite else 0.0`;
       `ctrl_out = control.ProduceCtrlOutput(ctrl_in, gyrz_meas, now)`.
7. Append a row to the trace CSV.
8. If the cycle is a sample cycle, print the key results.
9. `now += dt`; next cycle.

At the start of every run the harness does
`importlib.reload(guidance)`, `importlib.reload(control)`, `control.reset()` so
module-level state never leaks between runs. `now` starts at `1000.0`.

---

## 5. Interactive use

```
python tools/manual_sensor_cycle_sim.py --interactive --cycles 10 --sample-cycles 1,2,3,10
```

Each cycle prompts for `key=value` edits. Blank line keeps the previous cycle.

```
cycle 1 input: gps_pos=on gps_motion=on yaw=on gyrz=on baro=on acc=on speed_mps=4
cycle 2 input:                 (Enter → keep previous)
cycle 3 input: gps_motion=off
cycle 6 input: gps_pos=off gps_motion=off
```

Parser rules: multiple `key=value` per line; state keys take `on`/`off`/`stale`;
numeric keys are parsed as float; an **unknown key is an error** (the cycle is
re-prompted, not silently ignored).

Recognised keys: state — `gps_pos gps_motion yaw gyrz baro acc`; numeric —
`payload_E payload_N course_deg speed_mps yaw_deg gyrz_dps baro_alt baro_sink
acc_x acc_y target_E target_N`; misc — `state motor_enabled`.

---

## 6. Preset use

```
python tools/manual_sensor_cycle_sim.py --preset GPS_CLOSED_RIGHT --cycles 10 --sample-cycles 1,2,3,10
python tools/manual_sensor_cycle_sim.py --list-presets
```

| Preset | Expectation |
| --- | --- |
| `GPS_CLOSED_RIGHT` | `GPS_TRACKING_CLOSED`, nu>0, yaw_rate>0, arm RIGHT |
| `GPS_CLOSED_LEFT` | `GPS_TRACKING_CLOSED`, nu<0, yaw_rate<0, arm LEFT |
| `GPS_OPEN_RIGHT` | `GPS_TRACKING_OPEN`, pid disabled, arm RIGHT |
| `GPS_DROPOUT_SEQUENCE_RIGHT` | c1–2 CLOSED, c3–5 `DR_M_GBA_CLOSED`, c6–10 `DR_PM_GBA_CLOSED` |
| `DR_M_GBA_RIGHT` | warmup → `DR_M_GBA_CLOSED` (pos from GPS) |
| `DR_M_G_RIGHT` | `DR_M_G_CLOSED`, `dr_speed_source=SPEED_LASTV` |
| `DR_M_YBA_RIGHT` | `DR_M_YBA_OPEN`, pid disabled |
| `DR_PM_GBA_RIGHT` | `DR_PM_GBA_CLOSED`, DR accumulates |
| `DR_PM_G_RIGHT` | `DR_PM_G_CLOSED`, `SPEED_LASTV` |
| `DR_PM_YBA_RIGHT` | `DR_PM_YBA_OPEN`, pid disabled |
| `DR_TIMEOUT_RIGHT` | DR ages past `DR_MAX_AGE_S` → `FAIL`/`DR_TIMEOUT`; arm 0 after |
| `BARO_SPIKE_RIGHT` | cycle 5 `baro_sink_spike=True`, falls back to `SPEED_LASTV`, no blow-up |
| `GYRO_SPIKE_RIGHT` | cycle 5 → `*_OPEN`, spike not used in CLOSED PID |

Presets that need a locked anchor run a silent **warmup** of GPS-closed cycles
first (40 by default), then the main, recorded sequence. `DR_TIMEOUT_RIGHT`
overrides `dt` to 6.0 s so the anchor ages past 45 s within 10 cycles.

Precedence for `dt`/`cycles`: explicit CLI flag > preset value > module default.

---

## 7. Scripted CSV use

Author a CSV with a `cycle` column and any per-cycle keys; replay it:

```
python tools/manual_sensor_cycle_sim.py --script sim_input.csv --cycles 10 --sample-cycles 1,2,3,10
```

`sim_input.csv` (shipped example):

```
cycle,gps_pos,gps_motion,yaw,gyrz,baro,acc,payload_E,payload_N,course_deg,speed_mps,yaw_deg,gyrz_dps,baro_sink
1,on,on,on,on,on,on,0,0,0,4,0,0,2
2,on,on,on,on,on,on,0,0,0,4,0,0,2
3,on,off,on,on,on,on,0,0,0,4,0,0,2
...
6,off,off,on,on,on,on,0,0,0,4,0,0,2
...
10,off,off,on,on,on,on,0,0,0,4,0,0,2
```

Blank cells keep the previous cycle's value. Unknown columns are an error.

---

## 8. Reproducing a GPS dropout

The transition you want to see is **GPS_TRACKING → DR_M → DR_PM**:

- All sensors `on` → `GPS_TRACKING_CLOSED` (or `_OPEN` if gyro is off).
- `gps_motion=off` (position still on) → `DR_M_*` (P from GPS, motion estimated).
- `gps_pos=off` and `gps_motion=off` → `DR_PM_*` (true dead reckoning).

Use the `GPS_DROPOUT_SEQUENCE_RIGHT` preset or `sim_input.csv`.

> **off vs stale, and compressed time.** guidance freshness is *age-based* on the
> last accepted sample. A real `health=0` dropout leaves the previous (recent)
> timestamp in place, so a channel keeps reading *fresh* for up to its full
> age window (GPS 5 s, IMU 3 s, baro 2 s) of *simulated* time. Because this
> harness compresses time (`dt` defaults to 0.05 s), an instantaneous dropout
> would not register inside a 10-cycle run. So both `off` and `stale` present an
> **aged** sample (health=1, finite value, timestamp older than the window),
> which makes the dropout register the same cycle you ask for it. guidance makes
> no decision that distinguishes a powered-off sensor from a stale one, so the
> resulting control mode is identical; the literal `on`/`off`/`stale` you typed
> is still recorded in the trace.

---

## 9. Reading the output

Per-cycle terminal block (sample cycles):

```
[cycle 1]
mode = GPS_TRACKING_CLOSED
target_distance_m = 82.46
nu_deg = 75.96
yaw_rate_cmd_dps = 37.06
right_minus_left_angle_deg = 12.00
delta_arm_deg = 103.80
dr_age_s = 0.00
dr_confidence = 1.00
target_side = RIGHT
cmd_side = RIGHT
arm_side = RIGHT
judgement = TARGET RIGHT -> CMD RIGHT -> ARM RIGHT: OK
```

Key fields:

- `mode` / `fail_reason` — the selected control mode (or FAIL reason).
- `target_distance_m`, `nu_deg`, `target_bearing_deg` — L1 geometry.
- `yaw_rate_cmd_dps` vs `yaw_rate_limit_dps` — command and its per-mode cap.
- `delta_arm_deg` — commanded differential; `right_minus_left_angle_deg` is the
  slew-rate-limited actual differential applied to the servos.
- `dr_age_s`, `dr_confidence`, `dr_method`, `dr_speed_source` — DR health.
- `baro_sink_spike`, `speed_clamped`, `saturated` — safety-guard flags.

The full set of CSV columns is documented at the top of the trace file header.

---

## 10. Judging target pointing

Three "sides" are computed each cycle:

- `target_side`: `nu_deg > +NU_DEADBAND_DEG` → RIGHT, `< −deadband` → LEFT, else CENTER.
- `cmd_side`: sign of `yaw_rate_cmd_dps`.
- `arm_side`: sign of `right_minus_left_angle_deg` (right arm down = right turn).

`target_pointing_ok` is True when:

- **valid** L1/control: `target_side == cmd_side == arm_side`; or
- **FAIL/invalid**: `cmd_side == CENTER and arm_side == CENTER` (no motion).

So a healthy cycle steers the same way the target sits, and a failed cycle is
provably neutral.

---

## 11. DR_M vs DR_PM

- **DR_M_*** — GPS *position* is still fresh; only *motion* (course/speed) is
  estimated from gyro/yaw + baro/acc. `nav.E/N` come straight from GPS; the
  position is **never integrated**. `dr.current_E/N` track GPS, so they do not
  drift.
- **DR_PM_*** — GPS *position* is stale too: true dead reckoning. Both position
  and motion are propagated from `dr.current` using estimated velocity, so
  `dr.current_E/N` **accumulate** every cycle. Confidence decays with anchor age
  and scales the yaw-rate command; past `DR_MAX_AGE_S` the mode goes
  `FAIL`/`DR_TIMEOUT`.

The third letters encode the sources: `G`=gyro, `Y`=yaw, `B`=baro, `A`=acc.
Gyro sources give `*_CLOSED` (PID active); yaw-only sources give `*_OPEN`
(feed-forward only).

---

## 12. Why ProduceL1Input must not re-integrate DR

`DecideControlMode` already advanced `dr.current` for this cycle inside
`FillNav`. `ProduceL1Input` is only a *read/convert* step that packages the
already-filled `NavState` into an `L1Input`. If it also stepped `dr.current`,
DR would be integrated **twice per cycle**, doubling the drift rate and
corrupting every downstream distance/bearing. The harness records
`dr.current_E/N` immediately before and after `ProduceL1Input` and exposes
`dr_double_integrated_by_l1input`; it must always be `False` (asserted in the
test suite).

---

## 13. Pre-flight acceptance criteria

Before flight, a run should satisfy:

1. The mode progression matches the intended sensor sequence
   (`GPS_TRACKING_* → DR_M_* → DR_PM_*`).
2. For every valid cycle, `target_side == cmd_side == arm_side`.
3. `abs(yaw_rate_cmd_dps) <= yaw_rate_limit_dps` for every valid cycle.
4. `abs(delta_arm_deg) <= DELTA_ARM_MAX_DEG` for every cycle.
5. `yaw_rate_cmd_dps` and `right_minus_left_angle_deg` never have opposite signs.
6. `dr_double_integrated_by_l1input` is `False` everywhere.
7. DR accumulates in `DR_PM_*` and times out into `FAIL`/`DR_TIMEOUT`, after
   which the arms are neutral.
8. Baro and gyro spikes degrade gracefully (no NaN, no closed-loop use of the
   spike).

Run the regression suite with:

```
python -m pytest -q tests/test_manual_sensor_cycle_sim.py
```

## Run examples

```
python tools/manual_sensor_cycle_sim.py --preset GPS_CLOSED_RIGHT --cycles 10 --sample-cycles 1,2,3,10
python tools/manual_sensor_cycle_sim.py --preset GPS_CLOSED_LEFT --cycles 10 --sample-cycles 1,2,3,10
python tools/manual_sensor_cycle_sim.py --preset GPS_DROPOUT_SEQUENCE_RIGHT --cycles 10 --sample-cycles 1,2,3,10
python tools/manual_sensor_cycle_sim.py --preset DR_PM_GBA_RIGHT --cycles 10 --sample-cycles 1,2,3,10
python tools/manual_sensor_cycle_sim.py --interactive --cycles 10 --sample-cycles 1,2,3,10
python tools/manual_sensor_cycle_sim.py --script sim_input.csv --cycles 10 --sample-cycles 1,2,3,10
```

The trace is written to `manual_sensor_cycle_trace.csv` (repo root) by default;
override with `--output`.
