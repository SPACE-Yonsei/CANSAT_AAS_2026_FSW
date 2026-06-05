from __future__ import annotations

import csv
import importlib
import math
import sys
from pathlib import Path
from typing import Any
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from Sensor_Motor import control, guidance, motorapp  # noqa: E402
from lib import config  # noqa: E402


NOW = 1000.0
ORIGIN_LAT = 37.0
ORIGIN_LON = 127.0
TARGET_E_M = 100.0
CSV_PATH = ROOT / "motorapp_payload_integration_current.csv"


class FakePi:
    def __init__(self):
        self.calls: list[tuple[int, int]] = []

    def set_servo_pulsewidth(self, pin, pulsewidth):
        self.calls.append((pin, pulsewidth))


def reload_stack():
    global control, guidance, motorapp
    guidance = importlib.reload(guidance)
    control = importlib.reload(control)
    motorapp = importlib.reload(motorapp)
    control.reset()


def latlon_from_ne(north_m: float, east_m: float) -> tuple[float, float]:
    lat = ORIGIN_LAT + math.degrees(north_m / guidance.EARTH_RADIUS_M)
    lon = ORIGIN_LON + math.degrees(
        east_m / (guidance.EARTH_RADIUS_M * math.cos(math.radians(ORIGIN_LAT)))
    )
    return lat, lon


def configure_motorapp(*, target_ready: bool) -> FakePi:
    fake_pi = FakePi()
    motorapp.STATE = 4
    motorapp.MOTOR_ENABLED = True
    motorapp.PI = fake_pi
    motorapp._STEER_MODE = ""
    motorapp.MOTOR_CTRL_MODE = config.MOTOR_CTRL_MODE_GPS_GUIDED
    motorapp._ORIGIN_LOCKED = True
    guidance.set_origin_point(ORIGIN_LAT, ORIGIN_LON)
    if target_ready:
        target_lat, target_lon = latlon_from_ne(0.0, TARGET_E_M)
        guidance.set_target_point(target_lat, target_lon)
    return fake_pi


def push_payloads(*, course_deg: float = 0.0, raw_gyrz_deg_s: float = 12.0, sink_rate: float = 3.0):
    motorapp.handle_gps(
        f"{ORIGIN_LAT},{ORIGIN_LON},1,{NOW},{course_deg},{5.0},1,{NOW}"
    )
    motorapp.handle_imu(
        f"0.0,0.0,{course_deg},0.0,0.0,9.81,0.0,0.0,{raw_gyrz_deg_s},1,{NOW},0.0"
    )
    motorapp.handle_barometer(f"120.0,{sink_rate},1")


def _last_two_pwm(fake_pi: FakePi):
    return fake_pi.calls[-2:] if len(fake_pi.calls) >= 2 else []


def run_cycle_case(case: str, *, target_ready: bool) -> dict[str, Any]:
    reload_stack()
    fake_pi = configure_motorapp(target_ready=target_ready)
    log_calls: list[tuple[tuple[Any, ...], dict[str, Any]]] = []
    l1_inputs: list[Any] = []
    l1_outputs: list[Any] = []

    orig_l1input = motorapp.guidance.ProduceL1Input
    orig_l1output = motorapp.guidance.ProduceL1Output

    def capture_log(*args, **kwargs):
        log_calls.append((args, kwargs))

    def capture_l1input(now):
        value = orig_l1input(now)
        l1_inputs.append(value)
        return value

    def capture_l1output(l1_in):
        value = orig_l1output(l1_in)
        l1_outputs.append(value)
        return value

    with (
        mock.patch.object(motorapp.time, "monotonic", return_value=NOW),
        mock.patch.object(motorapp.sensorlog, "log_motor_raw", side_effect=capture_log) as log_mock,
        mock.patch.object(motorapp.prevstate, "update_start_point", return_value=None),
        mock.patch.object(motorapp, "_publish_motor_diag", return_value=None),
        mock.patch.object(motorapp.guidance, "DecideControlMode", wraps=motorapp.guidance.DecideControlMode) as decide_mock,
        mock.patch.object(motorapp.guidance, "ProduceL1Input", side_effect=capture_l1input) as l1in_mock,
        mock.patch.object(motorapp.guidance, "ProduceL1Output", side_effect=capture_l1output) as l1out_mock,
        mock.patch.object(motorapp.control, "ProduceCtrlInput", wraps=motorapp.control.ProduceCtrlInput) as ctrl_in_mock,
        mock.patch.object(motorapp.control, "ProduceCtrlOutput", wraps=motorapp.control.ProduceCtrlOutput) as ctrl_out_mock,
    ):
        push_payloads(course_deg=0.0, raw_gyrz_deg_s=12.0, sink_rate=3.0)
        cached_gps = motorapp._CACHE_t.latest_gps
        cached_imu = motorapp._CACHE_t.latest_imu
        cached_baro = motorapp._CACHE_t.latest_baro
        out = motorapp._ctrl_cycle(None, NOW)

    l1_in = l1_inputs[-1] if l1_inputs else None
    l1_out = l1_outputs[-1] if l1_outputs else None
    mode = guidance._STATE_t.nav.control_mode

    return {
        "case": case,
        "out": out,
        "mode": mode,
        "fake_pi": fake_pi,
        "log_calls": log_calls,
        "log_call_count": log_mock.call_count,
        "decide_call_count": decide_mock.call_count,
        "l1input_call_count": l1in_mock.call_count,
        "l1output_call_count": l1out_mock.call_count,
        "ctrlinput_call_count": ctrl_in_mock.call_count,
        "ctrloutput_call_count": ctrl_out_mock.call_count,
        "l1_in": l1_in,
        "l1_out": l1_out,
        "cached_gps": cached_gps,
        "cached_imu": cached_imu,
        "cached_baro": cached_baro,
        "last_pwm": _last_two_pwm(fake_pi),
        "guidance": guidance,
        "control": control,
        "motorapp": motorapp,
    }


def run_valid_case() -> dict[str, Any]:
    return run_cycle_case("valid_guidance_control", target_ready=True)


def run_invalid_neutral_case() -> dict[str, Any]:
    return run_cycle_case("invalid_no_target_neutral", target_ready=False)


def run_all_cases() -> list[dict[str, Any]]:
    return [run_valid_case(), run_invalid_neutral_case()]


def write_csv(results: list[dict[str, Any]], path: Path = CSV_PATH) -> None:
    fieldnames = [
        "case",
        "mode",
        "out_valid",
        "out_reason",
        "decide_call_count",
        "l1input_call_count",
        "l1output_call_count",
        "ctrlinput_call_count",
        "ctrloutput_call_count",
        "log_call_count",
        "cached_gps_pos_health",
        "cached_gps_motion_health",
        "cached_imu_gyrz_dps",
        "cached_baro_sink_rate",
        "delta_arm_deg",
        "left_pw",
        "right_pw",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        for result in results:
            out = result["out"]
            writer.writerow({
                "case": result["case"],
                "mode": result["mode"].value,
                "out_valid": getattr(out, "valid", False),
                "out_reason": getattr(out, "reason", ""),
                "decide_call_count": result["decide_call_count"],
                "l1input_call_count": result["l1input_call_count"],
                "l1output_call_count": result["l1output_call_count"],
                "ctrlinput_call_count": result["ctrlinput_call_count"],
                "ctrloutput_call_count": result["ctrloutput_call_count"],
                "log_call_count": result["log_call_count"],
                "cached_gps_pos_health": result["cached_gps"].pos_health,
                "cached_gps_motion_health": result["cached_gps"].motion_health,
                "cached_imu_gyrz_dps": math.degrees(result["cached_imu"].gyrz_rad_s)
                if result["cached_imu"].gyrz_rad_s is not None else float("nan"),
                "cached_baro_sink_rate": result["cached_baro"].sink_rate,
                "delta_arm_deg": getattr(out, "delta_arm_deg", 0.0),
                "left_pw": getattr(out, "left_pw", 0),
                "right_pw": getattr(out, "right_pw", 0),
            })


def main() -> int:
    results = run_all_cases()
    write_csv(results)
    for result in results:
        out = result["out"]
        print(
            f"{result['case']}: mode={result['mode'].value} "
            f"out_valid={getattr(out, 'valid', False)} reason={getattr(out, 'reason', '')} "
            f"decide_calls={result['decide_call_count']} log_calls={result['log_call_count']}"
        )
    print(f"csv: {CSV_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
