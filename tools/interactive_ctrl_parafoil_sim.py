#!/usr/bin/env python3
"""Interactive one-cycle simulator for motorapp/guidance/control.

The simulator feeds human-friendly local E/N meter inputs into the real
motorapp handlers, then calls motorapp._ctrl_cycle() exactly once per cycle.
Servo writes are captured by FakePi, so no hardware is required.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from Sensor_Motor import control, guidance, motorapp  # noqa: E402
from lib import config, prevstate  # noqa: E402


EARTH_RADIUS_M = guidance.EARTH_RADIUS_M

SAMPLE_SEND = "send"
SAMPLE_DROP = "drop"
SAMPLE_CHOICES = {SAMPLE_SEND, SAMPLE_DROP}

DEFAULT_INPUTS: dict[str, Any] = {
    "dt_s": 1.0 / float(config.MOTOR_RATE_HZ),
    "state": 3,
    "motor_enabled": True,
    "e_m": 0.0,
    "n_m": 0.0,
    "course_deg": 0.0,
    "speed_mps": 8.0,
    "roll_deg": 0.0,
    "pitch_deg": 0.0,
    "yaw_deg": 0.0,
    # Navigation/control sign: right turn positive. The raw IMU payload is
    # negated before handle_imu because motorapp.handle_imu flips raw BNO z.
    "gyrz_nav_deg_s": 0.0,
    "accx_mps2": 0.0,
    "accy_mps2": 0.0,
    "accz_mps2": 9.81,
    "alt_m": 100.0,
    "sink_rate_mps": 3.0,
    "gps_sample": SAMPLE_SEND,
    "gps_pos_ok": True,
    "gps_motion_ok": True,
    "imu_sample": SAMPLE_SEND,
    "imu_health": True,
    "baro_sample": SAMPLE_SEND,
    "baro_health": True,
    "detumble_enable": bool(config.DETUMBLE_ENABLE),
}

EDIT_FIELD_ORDER = [
    "dt_s",
    "state",
    "motor_enabled",
    "e_m",
    "n_m",
    "course_deg",
    "speed_mps",
    "yaw_deg",
    "gyrz_nav_deg_s",
    "gps_sample",
    "gps_pos_ok",
    "gps_motion_ok",
    "imu_sample",
    "imu_health",
    "baro_sample",
    "baro_health",
    "alt_m",
    "sink_rate_mps",
    "detumble_enable",
]

BOOL_FIELDS = {
    "motor_enabled",
    "gps_pos_ok",
    "gps_motion_ok",
    "imu_health",
    "baro_health",
    "detumble_enable",
}
INT_FIELDS = {"state"}
SAMPLE_FIELDS = {"gps_sample", "imu_sample", "baro_sample"}


@dataclass(frozen=True)
class Case:
    label: str
    note: str = ""
    values: dict[str, Any] = field(default_factory=dict)


class FakePi:
    """Small pigpio-compatible sink that records PWM writes."""

    connected = 1

    def __init__(self) -> None:
        self.pulses: dict[int, int] = {}
        self.history: list[tuple[float, int, int]] = []

    def set_servo_pulsewidth(self, pin: int, width: int) -> None:
        width_i = int(width)
        self.pulses[int(pin)] = width_i
        self.history.append((time.monotonic(), int(pin), width_i))

    def pulse(self, pin: int) -> int | None:
        return self.pulses.get(int(pin))


def case(label: str, note: str = "", **values: Any) -> Case:
    return Case(label=label, note=note, values=values)


PRESETS: dict[str, list[Case]] = {
    "all_cases": [
        case(
            "idle_state_before_release",
            "STATE<3 blocks control and writes zero-angle pulses.",
            state=2,
            gps_sample=SAMPLE_SEND,
            imu_sample=SAMPLE_SEND,
            baro_sample=SAMPLE_SEND,
        ),
        case(
            "active_no_origin_no_sensor",
            "STATE=3, motor ON, no sensor sample: FAIL path writes PWM off.",
            reset_before=True,
            state=3,
            gps_sample=SAMPLE_DROP,
            imu_sample=SAMPLE_DROP,
            baro_sample=SAMPLE_DROP,
        ),
        case(
            "gps_tracking_closed_origin_lock",
            "Fresh GPS position+motion and fresh gyro: origin locks, GPS_TRACKING_CLOSED.",
            e_m=0.0,
            n_m=0.0,
            course_deg=0.0,
            yaw_deg=0.0,
            gyrz_nav_deg_s=0.0,
            gps_sample=SAMPLE_SEND,
            gps_pos_ok=True,
            gps_motion_ok=True,
            imu_sample=SAMPLE_SEND,
            imu_health=True,
        ),
        case(
            "gps_tracking_closed_right_command",
            "Target is east/right of current bearing enough to command a right turn.",
            e_m=-220.0,
            n_m=120.0,
            course_deg=0.0,
            yaw_deg=0.0,
            gyrz_nav_deg_s=0.0,
        ),
        case(
            "detumbling_entry",
            "Fresh gyro above DETUMBLE_GYRZ_THRESHOLD_DPS.",
            e_m=-210.0,
            n_m=160.0,
            course_deg=5.0,
            yaw_deg=5.0,
            gyrz_nav_deg_s=250.0,
        ),
        case(
            "detumbling_exit_hold_start",
            "Gyro falls below exit threshold; current code starts hold then falls through.",
            gyrz_nav_deg_s=10.0,
        ),
        case(
            "detumbling_exit_after_hold",
            "After DETUMBLE_EXIT_HOLD_S, normal GPS tracking resumes.",
            dt_s=float(config.DETUMBLE_EXIT_HOLD_S) + 0.20,
            gyrz_nav_deg_s=10.0,
        ),
        case(
            "dr_closed_after_gps_dropout",
            "GPS sample is absent past freshness limit, IMU gyro remains fresh.",
            dt_s=float(config.GPS_FRESH_MAX_AGE_S) + 1.0,
            gps_sample=SAMPLE_DROP,
            imu_sample=SAMPLE_SEND,
            imu_health=True,
            gyrz_nav_deg_s=5.0,
        ),
        case(
            "dr_closed_gps_pos_only",
            "GPS position arrives but motion is invalid; existing DR anchor is used.",
            dt_s=0.20,
            gps_sample=SAMPLE_SEND,
            gps_pos_ok=True,
            gps_motion_ok=False,
            imu_sample=SAMPLE_SEND,
            imu_health=True,
            gyrz_nav_deg_s=4.0,
            e_m=-180.0,
            n_m=190.0,
        ),
        case(
            "fail_after_dr_when_imu_stale",
            "GPS still absent and IMU is not refreshed past IMU freshness age.",
            dt_s=float(config.IMU_FRESH_MAX_AGE_S) + 0.50,
            gps_sample=SAMPLE_DROP,
            imu_sample=SAMPLE_DROP,
            baro_sample=SAMPLE_DROP,
        ),
        case(
            "target_reached",
            "Fresh GPS inside TARGET_RADIUS_M should command zero yaw rate.",
            reset_before=True,
            e_m=0.0,
            n_m=float(config.TARGET_RADIUS_M) * 0.5 + 900.0,
            course_deg=0.0,
            speed_mps=6.0,
            yaw_deg=0.0,
            gyrz_nav_deg_s=0.0,
            gps_sample=SAMPLE_SEND,
            gps_pos_ok=True,
            gps_motion_ok=True,
            imu_sample=SAMPLE_SEND,
            imu_health=True,
        ),
        case(
            "motor_disabled",
            "MEC OFF blocks control and writes zero-angle pulses.",
            motor_enabled=False,
            state=3,
            gps_sample=SAMPLE_SEND,
            imu_sample=SAMPLE_SEND,
            baro_sample=SAMPLE_SEND,
        ),
        case(
            "landed_state",
            "STATE=5 writes PWM off.",
            motor_enabled=True,
            state=5,
        ),
        case(
            "gps_open_clean_yaw_only_anchor",
            "Clean reset: GPS pos+motion fresh, yaw fresh, gyro missing -> GPS_TRACKING_OPEN.",
            reset_before=True,
            state=3,
            motor_enabled=True,
            e_m=0.0,
            n_m=0.0,
            course_deg=0.0,
            speed_mps=8.0,
            yaw_deg=0.0,
            gyrz_nav_deg_s=math.nan,
            gps_sample=SAMPLE_SEND,
            gps_pos_ok=True,
            gps_motion_ok=True,
            imu_sample=SAMPLE_SEND,
            imu_health=True,
        ),
        case(
            "dr_open_after_gps_dropout_yaw_only",
            "GPS stale, yaw fresh, gyro missing from a clean anchor -> DR_TRACKING_OPEN.",
            dt_s=float(config.GPS_FRESH_MAX_AGE_S) + 1.0,
            gps_sample=SAMPLE_DROP,
            imu_sample=SAMPLE_SEND,
            imu_health=True,
            yaw_deg=15.0,
            gyrz_nav_deg_s=math.nan,
        ),
        case(
            "gyro_spike_control_fallback",
            "DETUMBLE disabled only for this diagnostic: gyro spike should hit control fallback.",
            reset_before=True,
            detumble_enable=False,
            state=3,
            motor_enabled=True,
            e_m=-220.0,
            n_m=120.0,
            course_deg=0.0,
            speed_mps=8.0,
            yaw_deg=0.0,
            gyrz_nav_deg_s=1600.0,
            gps_sample=SAMPLE_SEND,
            gps_pos_ok=True,
            gps_motion_ok=True,
            imu_sample=SAMPLE_SEND,
            imu_health=True,
        ),
    ],
    "dr_focus": [
        case(
            "anchor_closed",
            "Fresh GPS+gyro creates the DR anchor.",
            reset_before=True,
            state=3,
            motor_enabled=True,
            gps_sample=SAMPLE_SEND,
            gps_pos_ok=True,
            gps_motion_ok=True,
            imu_sample=SAMPLE_SEND,
            imu_health=True,
            e_m=0.0,
            n_m=0.0,
            course_deg=0.0,
            yaw_deg=0.0,
            gyrz_nav_deg_s=0.0,
        ),
        case(
            "dr_closed_short_dropout",
            "GPS dropout with fresh gyro.",
            dt_s=float(config.GPS_FRESH_MAX_AGE_S) + 1.0,
            gps_sample=SAMPLE_DROP,
            imu_sample=SAMPLE_SEND,
            gyrz_nav_deg_s=3.0,
        ),
        case(
            "dr_closed_confidence_decay",
            "More dropout time lowers DR confidence.",
            dt_s=4.0,
            gps_sample=SAMPLE_DROP,
            imu_sample=SAMPLE_SEND,
            gyrz_nav_deg_s=3.0,
        ),
        case(
            "dr_fail_no_imu",
            "DR anchor exists, but no fresh IMU yaw/gyro remains.",
            dt_s=float(config.IMU_FRESH_MAX_AGE_S) + 0.5,
            gps_sample=SAMPLE_DROP,
            imu_sample=SAMPLE_DROP,
        ),
    ],
}


class Simulator:
    def __init__(self, origin_lat: float, origin_lon: float, target_e: float,
                 target_n: float, inputs: dict[str, Any]) -> None:
        self.origin_lat = float(origin_lat)
        self.origin_lon = float(origin_lon)
        self.target_e = float(target_e)
        self.target_n = float(target_n)
        self.inputs = dict(inputs)
        self.now = time.monotonic()
        self.fake_pi = FakePi()
        self.original_detumble_enable = bool(config.DETUMBLE_ENABLE)
        self._patch_prevstate()
        self.reset_runtime()

    def _patch_prevstate(self) -> None:
        # Avoid mutating lib/prevstate.json during exploratory simulations.
        prevstate.update_start_point = lambda *args, **kwargs: None
        prevstate.clear_start_point = lambda *args, **kwargs: None
        prevstate.update_motor_enabled = lambda *args, **kwargs: None

    def reset_runtime(self) -> None:
        guidance.reset()
        motorapp._CACHE_t = motorapp._Cache()
        motorapp._CTRLER_t = control.MakeCtrler()
        motorapp._ORIGIN_SAVED = True
        motorapp._PREV_STATE = 0
        motorapp.STATE = int(self.inputs["state"])
        motorapp.MOTOR_ENABLED = bool(self.inputs["motor_enabled"])
        motorapp.MOTORAPP_RUNSTATUS = True
        motorapp.PI = self.fake_pi
        self.fake_pi.pulses.clear()
        self.fake_pi.history.clear()
        self._set_target_from_local_ne()

    def restore(self) -> None:
        config.DETUMBLE_ENABLE = self.original_detumble_enable

    def _set_target_from_local_ne(self) -> None:
        lat, lon = ne_to_latlon(
            self.target_n, self.target_e, self.origin_lat, self.origin_lon
        )
        motorapp.handle_target_coord(f"{lat:.9f},{lon:.9f}")

    def run_cycle(self, values: dict[str, Any]) -> control.CtrlOutput | None:
        self.inputs.update(values)
        config.DETUMBLE_ENABLE = bool(self.inputs["detumble_enable"])
        dt_s = max(0.0, float(self.inputs["dt_s"]))
        self.now += dt_s

        motorapp.handle_mec("ON" if self.inputs["motor_enabled"] else "OFF")
        motorapp.handle_flight_state(str(int(self.inputs["state"])))
        self._send_gps()
        self._send_imu()
        self._send_baro()
        return motorapp._ctrl_cycle(None, self.now)

    def _send_gps(self) -> None:
        if self.inputs["gps_sample"] == SAMPLE_DROP:
            return
        lat, lon = ne_to_latlon(
            float(self.inputs["n_m"]),
            float(self.inputs["e_m"]),
            self.origin_lat,
            self.origin_lon,
        )
        pos_ok = 1 if self.inputs["gps_pos_ok"] else 0
        motion_ok = 1 if self.inputs["gps_motion_ok"] else 0
        payload = ",".join(
            [
                f"{lat:.9f}",
                f"{lon:.9f}",
                str(pos_ok),
                f"{self.now:.6f}",
                f"{float(self.inputs['course_deg']):.6f}",
                f"{float(self.inputs['speed_mps']):.6f}",
                str(motion_ok),
                f"{self.now:.6f}",
            ]
        )
        motorapp.handle_gps(payload)

    def _send_imu(self) -> None:
        if self.inputs["imu_sample"] == SAMPLE_DROP:
            return
        health = 1 if self.inputs["imu_health"] else 0
        gyrz_nav = float(self.inputs["gyrz_nav_deg_s"])
        raw_gyrz = -gyrz_nav if math.isfinite(gyrz_nav) else math.nan
        payload = ",".join(
            [
                fmt_float(self.inputs["roll_deg"]),
                fmt_float(self.inputs["pitch_deg"]),
                fmt_float(self.inputs["yaw_deg"]),
                fmt_float(self.inputs["accx_mps2"]),
                fmt_float(self.inputs["accy_mps2"]),
                fmt_float(self.inputs["accz_mps2"]),
                "0.0",
                "0.0",
                fmt_float(raw_gyrz),
                str(health),
                f"{self.now:.6f}",
            ]
        )
        motorapp.handle_imu(payload)

    def _send_baro(self) -> None:
        if self.inputs["baro_sample"] == SAMPLE_DROP:
            return
        health = 1 if self.inputs["baro_health"] else 0
        payload = (
            f"{float(self.inputs['alt_m']):.6f},"
            f"{float(self.inputs['sink_rate_mps']):.6f},"
            f"{health}"
        )
        motorapp.handle_barometer(payload)


def ne_to_latlon(n_m: float, e_m: float, origin_lat: float,
                 origin_lon: float) -> tuple[float, float]:
    lat = origin_lat + math.degrees(n_m / EARTH_RADIUS_M)
    cos_lat = math.cos(math.radians(origin_lat))
    if abs(cos_lat) < 1.0e-9:
        raise ValueError("origin latitude is too close to the pole")
    lon = origin_lon + math.degrees(e_m / (EARTH_RADIUS_M * cos_lat))
    return lat, lon


def fmt_float(value: Any) -> str:
    v = float(value)
    return "nan" if not math.isfinite(v) else f"{v:.6f}"


def fmt_value(value: Any) -> str:
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, float) and not math.isfinite(value):
        return "nan"
    return str(value)


def parse_bool(raw: str) -> bool:
    token = raw.strip().lower()
    if token in {"1", "true", "t", "yes", "y", "on"}:
        return True
    if token in {"0", "false", "f", "no", "n", "off"}:
        return False
    raise ValueError(f"expected boolean, got {raw!r}")


def parse_value(key: str, raw: str) -> Any:
    raw = raw.strip()
    if key in BOOL_FIELDS:
        return parse_bool(raw)
    if key in INT_FIELDS:
        return int(raw)
    if key in SAMPLE_FIELDS:
        token = raw.lower()
        if token in {"s", "send", "ok", "fresh"}:
            return SAMPLE_SEND
        if token in {"d", "drop", "none", "missing", "stale"}:
            return SAMPLE_DROP
        raise ValueError(f"{key} must be send or drop")
    if raw.lower() in {"nan", "none", "missing"}:
        return math.nan
    return float(raw)


def apply_inline_overrides(values: dict[str, Any], text: str) -> dict[str, Any]:
    out = dict(values)
    cleaned = text.replace(",", " ")
    for item in cleaned.split():
        if "=" not in item:
            raise ValueError(f"override must be key=value, got {item!r}")
        key, raw = item.split("=", 1)
        key = key.strip()
        if key not in DEFAULT_INPUTS:
            raise ValueError(f"unknown key {key!r}")
        out[key] = parse_value(key, raw)
    return out


def prompt_edit(values: dict[str, Any]) -> dict[str, Any]:
    edited = dict(values)
    print("Edit fields. Blank keeps the shown value. Use nan for missing gyro.")
    for key in EDIT_FIELD_ORDER:
        raw = input(f"  {key} [{fmt_value(edited[key])}]: ").strip()
        if not raw:
            continue
        edited[key] = parse_value(key, raw)
    return edited


def print_case_input(label: str, note: str, values: dict[str, Any]) -> None:
    print("\n" + "=" * 78)
    print(f"case: {label}")
    if note:
        print(f"note: {note}")
    compact = [
        f"dt={fmt_value(values['dt_s'])}",
        f"state={values['state']}",
        f"motor={'ON' if values['motor_enabled'] else 'OFF'}",
        f"E={fmt_value(values['e_m'])}",
        f"N={fmt_value(values['n_m'])}",
        f"course={fmt_value(values['course_deg'])}deg",
        f"V={fmt_value(values['speed_mps'])}m/s",
        f"yaw={fmt_value(values['yaw_deg'])}deg",
        f"gyrz_nav={fmt_value(values['gyrz_nav_deg_s'])}deg/s",
    ]
    sensors = [
        f"gps={values['gps_sample']} pos={int(values['gps_pos_ok'])} mot={int(values['gps_motion_ok'])}",
        f"imu={values['imu_sample']} health={int(values['imu_health'])}",
        f"baro={values['baro_sample']} health={int(values['baro_health'])}",
        f"detumble={int(values['detumble_enable'])}",
    ]
    print("input: " + " | ".join(compact))
    print("sensor: " + " | ".join(sensors))


def print_cycle_output(sim: Simulator, ctrl_out: control.CtrlOutput | None) -> None:
    st = guidance._STATE_t
    mi = guidance._MISSION_t
    mode = st.nav.control_mode.value if hasattr(st.nav.control_mode, "value") else str(st.nav.control_mode)
    dr_method = st.dr.method.value if hasattr(st.dr.method, "value") else str(st.dr.method)
    print("-" * 78)
    print(
        "guidance: "
        f"mode={mode} dr={dr_method} conf={safe_num(st.nav.confidence):.2f} "
        f"origin={int(mi.origin_ready)} target={int(mi.target_ready)}"
    )
    print(
        "nav: "
        f"E={safe_num(st.nav.E):.2f} N={safe_num(st.nav.N):.2f} "
        f"course={safe_deg(st.nav.course):.2f}deg V={safe_num(st.nav.V):.2f}m/s "
        f"target_E={safe_num(mi.target_E):.2f} target_N={safe_num(mi.target_N):.2f}"
    )
    if ctrl_out is None:
        print("control: no CtrlOutput returned (blocked, landed, disabled, or FAIL/off path)")
    else:
        print(
            "control: "
            f"valid={int(ctrl_out.valid)} mode={ctrl_out.mode} "
            f"fallback={ctrl_out.fallback_mode} sensor={int(ctrl_out.sensor_valid)} "
            f"sat={int(ctrl_out.saturated)} "
            f"cmd={ctrl_out.angular_velocity_cmd_deg_s:.2f}deg/s "
            f"meas={safe_num(ctrl_out.angular_velocity_meas_deg_s):.2f}deg/s "
            f"delta={ctrl_out.delta_arm_deg:.2f}deg "
            f"ff={ctrl_out.delta_ff_deg:.2f} pid={ctrl_out.delta_pid_deg:.2f}"
        )
    left_pw = sim.fake_pi.pulse(control.PARAFOIL_LEFT_MOTOR_PIN)
    right_pw = sim.fake_pi.pulse(control.PARAFOIL_RIGHT_MOTOR_PIN)
    print(
        "servo: "
        f"L={format_servo('left', left_pw)} | R={format_servo('right', right_pw)}"
    )


def safe_num(value: Any) -> float:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return math.nan
    return v if math.isfinite(v) else math.nan


def safe_deg(rad: Any) -> float:
    v = safe_num(rad)
    return math.degrees(v) if math.isfinite(v) else math.nan


def format_servo(side: str, pulse: int | None) -> str:
    if pulse is None:
        return "not-written"
    if pulse == 0:
        return "OFF(pw=0)"
    if side == "left":
        angle = (control.LEFT_ZERO - pulse) / control.PULSE_PER_DEG
    else:
        angle = (pulse - control.RIGHT_ZERO) / control.PULSE_PER_DEG
    return f"{angle:.2f}deg pw={pulse}"


def list_cases() -> None:
    for preset, cases in PRESETS.items():
        print(f"\n[{preset}]")
        for i, c in enumerate(cases, 1):
            print(f"{i:02d}. {c.label}: {c.note}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Step through one ctrl_parafoil cycle at a time."
    )
    parser.add_argument("--preset", choices=sorted(PRESETS), default="all_cases")
    parser.add_argument("--manual", action="store_true",
                        help="ignore preset list and prompt every field each cycle")
    parser.add_argument("--auto", action="store_true",
                        help="run the selected preset without pausing")
    parser.add_argument("--list-cases", action="store_true",
                        help="print preset cycle input sets and exit")
    parser.add_argument("--origin-lat", type=float,
                        default=float(config.GPS_EXPECTED_LAT_CENTER_DEG))
    parser.add_argument("--origin-lon", type=float,
                        default=float(config.GPS_EXPECTED_LON_CENTER_DEG))
    parser.add_argument("--target-e", type=float, default=0.0,
                        help="target east coordinate in meters from origin")
    parser.add_argument("--target-n", type=float, default=900.0,
                        help="target north coordinate in meters from origin")
    return parser


def run_preset(sim: Simulator, cases: list[Case], auto: bool) -> None:
    current = dict(sim.inputs)
    for idx, c in enumerate(cases, 1):
        if c.values.get("reset_before"):
            current = dict(DEFAULT_INPUTS)
            current.update(
                {
                    "state": c.values.get("state", 3),
                    "motor_enabled": c.values.get("motor_enabled", True),
                }
            )
            sim.inputs = dict(current)
            sim.reset_runtime()

        cycle_values = dict(current)
        for key, value in c.values.items():
            if key != "reset_before":
                cycle_values[key] = value

        while True:
            print_case_input(f"{idx:02d}/{len(cases)} {c.label}", c.note, cycle_values)
            if auto:
                choice = ""
            else:
                choice = input(
                    "Enter=run, edit=field prompts, key=value ...=override, "
                    "skip, quit: "
                ).strip()
            if choice.lower() in {"q", "quit", "exit"}:
                return
            if choice.lower() in {"s", "skip"}:
                break
            if choice.lower() in {"e", "edit"}:
                cycle_values = prompt_edit(cycle_values)
                continue
            if choice:
                try:
                    cycle_values = apply_inline_overrides(cycle_values, choice)
                except ValueError as exc:
                    print(f"input error: {exc}")
                    continue
            ctrl_out = sim.run_cycle(cycle_values)
            print_cycle_output(sim, ctrl_out)
            current = dict(sim.inputs)
            break


def run_manual(sim: Simulator) -> None:
    current = dict(sim.inputs)
    idx = 1
    while True:
        print_case_input(f"manual {idx}", "Prompted values feed one _ctrl_cycle call.", current)
        choice = input("Enter=run current, edit, key=value ...=override, reset, quit: ").strip()
        if choice.lower() in {"q", "quit", "exit"}:
            return
        if choice.lower() in {"r", "reset"}:
            current = dict(DEFAULT_INPUTS)
            sim.inputs = dict(current)
            sim.reset_runtime()
            continue
        if choice.lower() in {"e", "edit"}:
            current = prompt_edit(current)
            continue
        if choice:
            try:
                current = apply_inline_overrides(current, choice)
            except ValueError as exc:
                print(f"input error: {exc}")
                continue
        ctrl_out = sim.run_cycle(current)
        print_cycle_output(sim, ctrl_out)
        current = dict(sim.inputs)
        idx += 1


def main() -> int:
    args = build_parser().parse_args()
    if args.list_cases:
        list_cases()
        return 0

    inputs = dict(DEFAULT_INPUTS)
    sim = Simulator(
        origin_lat=args.origin_lat,
        origin_lon=args.origin_lon,
        target_e=args.target_e,
        target_n=args.target_n,
        inputs=inputs,
    )

    print(
        "Interactive parafoil control simulator\n"
        f"origin lat/lon=({args.origin_lat:.9f}, {args.origin_lon:.9f}), "
        f"target E/N=({args.target_e:.2f}, {args.target_n:.2f}) m\n"
        "Angles are degrees. course/yaw use North=0, East=+90. "
        "gyrz_nav is the controller sign: right turn positive.\n"
        "Sample fields: send=deliver a handler payload, drop=no new sensor sample."
    )

    try:
        if args.manual:
            run_manual(sim)
        else:
            run_preset(sim, PRESETS[args.preset], args.auto)
    finally:
        sim.restore()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
