"""CLI front-end for ``scenario_runner.ScenarioRunner``.

Drives the FSW through a SIM-mode scenario from the terminal. Useful when the
GCS GUI is not running, or for batch / scripted comparisons. The GCS GUI must
*not* be connected to the same COM port at the same time.

Usage::

    python ground_station/scenario_player.py --port COM5 --scenario calm
    python ground_station/scenario_player.py --port COM5 --scenario gust12 --baud 9600

    # List built-in presets:
    python ground_station/scenario_player.py --list

The trail can still be visualised in real time: open the GCS GUI **after** the
CLI run finishes and inspect the saved telemetry CSV, or run the GCS on a
second XBee receiver if you have one.
"""
from __future__ import annotations

import argparse
import io
import queue
import sys
import threading
import time
from datetime import datetime
from typing import Optional

# Korean Windows defaults to cp949 — force UTF-8 so descriptions / headings
# (which may contain em-dashes etc.) print without UnicodeEncodeError.
try:
    sys.stdout = io.TextIOWrapper(
        sys.stdout.buffer, encoding="utf-8", errors="replace"
    )
    sys.stderr = io.TextIOWrapper(
        sys.stderr.buffer, encoding="utf-8", errors="replace"
    )
except (AttributeError, ValueError):
    pass

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("pyserial is required: pip install pyserial", file=sys.stderr)
    sys.exit(1)

import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from scenario_runner import (  # noqa: E402  (sys.path tweak above)
    PRESETS,
    ScenarioConfig,
    ScenarioRunner,
    get_preset,
    list_preset_names,
)


TEAM_ID = "1070"


# Match TLM_FIELDS order from ground_station.py so left/right pulse parse the
# same way the GUI parses them (only the slots we need).
_TLM_FIELDS = [
    "team_id", "time", "packet_count", "mode", "state",
    "altitude_m", "temperature_c", "pressure_hpa",
    "voltage_v", "current_a", "power_w",
    "gyro_roll", "gyro_pitch", "gyro_yaw",
    "acc_roll", "acc_pitch", "acc_yaw",
    "mag_roll", "mag_pitch", "mag_yaw",
    "gps_time", "gps_alt", "gps_lat", "gps_lon", "gps_sats",
    "distance_cm", "cmd_echo",
    "filtered_roll", "filtered_pitch", "filtered_yaw",
    "start_lat", "start_lon",
    "target_lat", "target_lon",
    "carrot_lat", "carrot_lon",
    "current_heading_deg", "desired_heading_deg",
    "left_pulse_us", "right_pulse_us", "guidance_state",
]
_LEGACY_TLM_FIELDS = 30


class _SerialReader(threading.Thread):
    """Background reader that pushes complete lines into a queue."""

    def __init__(self, ser: serial.Serial, q: "queue.Queue[str]") -> None:
        super().__init__(daemon=True)
        self._ser = ser
        self._q = q
        self._stop = threading.Event()

    def stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(256)
            except Exception as exc:
                self._q.put(f"[serial-error] {exc}")
                break
            if not chunk:
                continue
            buf.extend(chunk)
            while True:
                nl = buf.find(b"\n")
                if nl < 0:
                    break
                raw = bytes(buf[:nl])
                del buf[: nl + 1]
                line = raw.decode("utf-8", errors="ignore").strip("\r\n").strip()
                if line:
                    self._q.put(line)


def _parse_tlm(line: str) -> Optional[dict]:
    if not line.startswith(f"${TEAM_ID},"):
        return None
    body = line[1:]
    parts = body.split(",")
    if len(parts) < _LEGACY_TLM_FIELDS:
        return None
    if len(parts) < len(_TLM_FIELDS):
        parts.extend([""] * (len(_TLM_FIELDS) - len(parts)))
    return {key: parts[i].strip() for i, key in enumerate(_TLM_FIELDS)}


# --------------------------------------------------------------- CLI driver

class _Driver:
    def __init__(self, port: str, baud: int, verbose: bool = False) -> None:
        self._ser = serial.Serial(port, baud, timeout=0.2)
        self._q: "queue.Queue[str]" = queue.Queue()
        self._reader = _SerialReader(self._ser, self._q)
        self._reader.start()
        self._latest_tlm: Optional[dict] = None
        self._verbose = verbose

    def close(self) -> None:
        self._reader.stop()
        try:
            self._ser.close()
        except Exception:
            pass

    def drain(self, timeout_s: float = 0.0) -> None:
        """Pull recent lines from the serial queue and update latest TLM dict."""
        deadline = time.monotonic() + max(0.0, timeout_s)
        while True:
            try:
                line = self._q.get_nowait()
            except queue.Empty:
                if time.monotonic() >= deadline:
                    return
                time.sleep(0.005)
                continue
            parsed = _parse_tlm(line)
            if parsed is not None:
                self._latest_tlm = parsed
            elif self._verbose:
                print(f"  rx: {line}")

    def get_tlm(self) -> Optional[dict]:
        self.drain()
        return self._latest_tlm

    def send_body(self, body: str) -> bool:
        line = f"CMD,{TEAM_ID},{body}\n"
        try:
            self._ser.write(line.encode("utf-8"))
            print(f"  tx: {body}")
            return True
        except Exception as exc:
            print(f"  tx-error: {exc}", file=sys.stderr)
            return False


def _print_presets() -> None:
    print("Built-in scenarios:\n")
    width = max(len(n) for n in list_preset_names())
    for name in list_preset_names():
        cfg = PRESETS[name]
        print(f"  {name:<{width}}  {cfg.description}")
        print(f"  {'':<{width}}    "
              f"alt0={cfg.start_alt_m:.0f}m  "
              f"airspeed={cfg.airspeed_ms:.1f}m/s  "
              f"descent={cfg.descent_rate_ms:.1f}m/s  "
              f"wind={cfg.wind_speed_ms:.1f}@{cfg.wind_dir_met_deg:.0f}deg")
    print()


def _build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="scenario_player",
        description="Closed-loop SIM scenario player for CANSAT GCS.",
    )
    p.add_argument("--port", help="Serial port (e.g. COM5 or /dev/ttyUSB0).")
    p.add_argument("--baud", type=int, default=9600,
                   help="Baud rate (default 9600 = FSW UART_BAUD default; match XCTU Interface Data Rate).")
    p.add_argument("--scenario", choices=list_preset_names(),
                   help="Preset name; required unless --list.")
    p.add_argument("--list", action="store_true",
                   help="List available scenarios and exit.")
    p.add_argument("--list-ports", action="store_true",
                   help="List visible COM ports and exit.")
    p.add_argument("--seed", type=int, default=0,
                   help="RNG seed for descent jitter / gust phase.")
    p.add_argument("--tick", type=float,
                   help="Override scenario tick period (s).")
    p.add_argument("--no-release", action="store_true",
                   help="Skip SS,3 in setup (e.g. when state is already RELEASE).")
    p.add_argument("--keep-sim-on", action="store_true",
                   help="Do not send SIM,DISABLE on completion.")
    p.add_argument("--verbose", action="store_true",
                   help="Print all received serial lines (not only TLM).")
    return p


def _override_config(cfg: ScenarioConfig, args: argparse.Namespace) -> ScenarioConfig:
    if args.tick is not None and args.tick > 0:
        cfg.tick_period_s = float(args.tick)
    if args.no_release:
        cfg.use_release_state = False
    if args.keep_sim_on:
        cfg.auto_disable_sim = False
    return cfg


def main(argv: Optional[list[str]] = None) -> int:
    args = _build_argparser().parse_args(argv)

    if args.list:
        _print_presets()
        return 0
    if args.list_ports:
        for p in sorted({p.device for p in list_ports.comports()}):
            print(p)
        return 0
    if not args.scenario or not args.port:
        print("ERROR: --port and --scenario are required (or use --list).", file=sys.stderr)
        return 2

    cfg = _override_config(get_preset(args.scenario), args)

    print(f"=== scenario: {cfg.name} === {datetime.now().isoformat(timespec='seconds')}")
    print(f"  {cfg.description}")
    print(f"  start  : ({cfg.start_lat:.5f}, {cfg.start_lon:.5f})  alt={cfg.start_alt_m:.0f}m")
    print(f"  target : ({cfg.target_lat:.5f}, {cfg.target_lon:.5f})")
    print(f"  wind   : {cfg.wind_speed_ms:.1f}m/s from {cfg.wind_dir_met_deg:.0f}deg  "
          f"gust=+/-{cfg.gust_amp_ms:.1f}m/s @{cfg.gust_period_s:.1f}s")
    print(f"  descent: {cfg.descent_rate_ms:.1f}m/s (+/-{cfg.descent_jitter_ms:.1f})  "
          f"airspeed={cfg.airspeed_ms:.1f}m/s")
    print(f"  tick   : {cfg.tick_period_s:.2f}s   timeout={cfg.timeout_s:.0f}s")
    print()

    try:
        drv = _Driver(args.port, args.baud, verbose=args.verbose)
    except Exception as exc:
        print(f"ERROR opening {args.port} @ {args.baud}: {exc}", file=sys.stderr)
        return 3

    print(f"connected: {args.port} @ {args.baud}\n")

    runner = ScenarioRunner(
        send_cb=drv.send_body,
        get_tlm_cb=drv.get_tlm,
        log_cb=print,
        config=cfg,
        rng_seed=args.seed,
    )

    try:
        runner.start()
        # Setup already waits ``setup_inter_cmd_delay_s`` between UART cmds; add a
        # short extra beat before SIMG ticks (avoid stacking multipliers on slow links).
        time.sleep(max(0.5, min(4.0, cfg.setup_inter_cmd_delay_s * 0.35)))

        next_tick = time.monotonic()
        while not runner.state.finished:
            now = time.monotonic()
            if now < next_tick:
                drv.drain(timeout_s=next_tick - now)
                continue
            still_running = runner.tick()
            s = runner.state
            print(f"  t={s.elapsed_s:5.1f}s  "
                  f"alt={s.alt_m:6.1f}m  "
                  f"hdg={s.heading_deg:5.1f}deg  "
                  f"course={s.course_deg:5.1f}deg  "
                  f"yaw_rate={s.yaw_rate_deg_s:+5.1f}deg/s  "
                  f"d_target={s.distance_to_target_m:6.1f}m  "
                  f"gs={s.ground_speed_ms:4.1f}m/s")
            if not still_running:
                break
            next_tick += cfg.tick_period_s
    except KeyboardInterrupt:
        print("\n  interrupted by user — sending teardown")
    finally:
        runner.stop(reason="cli-exit")
        drv.close()

    print()
    s = runner.state
    print(f"=== done ===  reason={s.finish_reason}  ticks={s.tick_count}  "
          f"elapsed={s.elapsed_s:.1f}s  final_d={s.distance_to_target_m:.1f}m  "
          f"alt={s.alt_m:.1f}m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
