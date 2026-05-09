"""CANSAT AAS 2026 - Ground Station GUI.

Receives telemetry from XBee (USB serial), parses the CANSAT TLM CSV emitted by
`comm/commapp.py:send_tlm`, displays it live, logs to CSV, and sends commands as
`CMD,1070,<CMD>,<body>` (see `comm/commapp.py:_dispatch_command`).

SIM mode (bench / map rehearsal) — send in order:
  1. CMD,1070,SIM,ENABLE     — prepare (TLM mode column A)
  2. CMD,1070,SIM,ACTIVATE  — SIM on (TLM S); required before SIMP/SIMG
  3. CMD,1070,SIMP,<alt_m>  — simulated baro altitude for flight logic / state machine
  4. CMD,1070,SIMG,lat,lon,course_deg,speed_m_s[,alt_m] — simulated GPS fix (course: ground track deg, speed: m/s)
  5. CMD,1070,TC,lat,lon    — release target (required: SS,3 is blocked if prevstate target is unset)
  6. CMD,1070,SS,3          — jump to RELEASE (commands are case-insensitive: ss,3 works)
  7. CMD,1070,SIM,DISABLE   — exit SIM (TLM F); GPS returns to hardware path

Run:
    python ground_station/ground_station.py

Map: wheel = zoom; Fit restores auto-bounds. SIMG from this UI clears the blue trail.
Target (0,0) is ignored for map centering / marker.

Standalone, no other repo modules required (only `pyserial` + tkinter).
"""

from __future__ import annotations

import csv
import math
import os
import queue
import sys
import threading
import time
import tkinter as tk
from datetime import datetime
from tkinter import ttk, messagebox

try:
    import serial
    from serial.tools import list_ports
except Exception as exc:
    print("pyserial is required. Install with: pip install pyserial")
    print("Import error:", exc)
    sys.exit(1)

# Scenario player lives next to this file; importable when run as a script
# (`python ground_station/ground_station.py`) by re-using the script's dir on
# sys.path. PyInstaller bundles already place this module alongside.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from scenario_runner import (  # type: ignore[import-not-found]
        PRESETS as _SCENARIO_PRESETS,
        ScenarioConfig as _ScenarioConfig,
        ScenarioRunner as _ScenarioRunner,
        get_preset as _scenario_get_preset,
        list_preset_names as _scenario_list_presets,
    )
    _SCENARIO_AVAILABLE = True
except Exception as _scenario_import_exc:  # pragma: no cover  (defensive)
    _SCENARIO_PRESETS = {}
    _ScenarioConfig = None  # type: ignore[assignment]
    _ScenarioRunner = None  # type: ignore[assignment]
    _scenario_get_preset = None  # type: ignore[assignment]
    _scenario_list_presets = lambda: []  # type: ignore[assignment]
    _SCENARIO_AVAILABLE = False


TEAM_ID = "1070"

TLM_FIELDS = [
    "team_id", "time", "packet_count", "mode", "state",
    "altitude_m", "temperature_c", "pressure_hpa",
    "voltage_v", "current_a", "power_w",
    "gyro_roll", "gyro_pitch", "gyro_yaw",
    "acc_roll", "acc_pitch", "acc_yaw",
    "mag_roll", "mag_pitch", "mag_yaw",
    "gps_time", "gps_alt", "gps_lat", "gps_lon", "gps_sats",
    "distance_mm", "cmd_echo",
    "filtered_roll", "filtered_pitch", "filtered_yaw",
    "start_lat", "start_lon",
    "target_lat", "target_lon",
    "carrot_lat", "carrot_lon",
    "current_heading_deg", "desired_heading_deg",
    "left_pulse_us", "right_pulse_us", "guidance_state",
]

LEGACY_TLM_FIELDS = 30

# Fixed character widths for telemetry value columns — avoids layout jump when cmd_echo / GPS strings change.
_TLM_VALUE_WIDTH_DEFAULT = 11
_TLM_VALUE_WIDTH: dict[str, int] = {
    "cmd_echo": 36,
    "gps_lat": 12,
    "gps_lon": 12,
    "gps_time": 10,
    "time": 10,
    "packet_count": 7,
    "team_id": 6,
    "mode": 4,
    "state": 4,
    "altitude_m": 10,
    "temperature_c": 8,
    "pressure_hpa": 10,
    "voltage_v": 8,
    "current_a": 8,
    "power_w": 8,
    "distance_mm": 10,
    "gps_alt": 8,
    "gps_sats": 4,
    "filtered_roll": 9,
    "filtered_pitch": 9,
    "filtered_yaw": 9,
    "gyro_roll": 9,
    "gyro_pitch": 9,
    "gyro_yaw": 9,
    "acc_roll": 9,
    "acc_pitch": 9,
    "acc_yaw": 9,
    "mag_roll": 9,
    "mag_pitch": 9,
    "mag_yaw": 9,
}

# Serial RX: process at most this many lines per Tk tick so bursts (USB backlog) do not freeze the UI for seconds.
_RX_MAX_LINES_PER_TICK = 24
_RX_POLL_IDLE_MS = 22
_RX_POLL_BACKLOG_MS = 1
# Map trail: cap vertices sent to Canvas (full history kept in memory for bounds)
_MAP_TRACK_DRAW_MAX = 450
# Map view: ref = **target** when telemetry has it (else start, else data centroid).
# Half-extent uses max |E|/|N| from ref to all track + landmark points, × margin, capped symmetrically
# so operator↔vehicle separation and full path stay on-screen (legacy ±1km/±500m clipped far fixes).
_MAP_REF_MAX_HALF_M = 50_000.0
_MAP_VIEW_MARGIN = 1.14
_MAP_VIEW_MIN_HALF_M = 2.5
# When start & target are both known, pad view so neither sits on the plot edge.
_MAP_START_TARGET_PAD_FACTOR = 1.22
# Ignore GPS→GPS trail segments longer than this (SIM jump / multimodal bug) to avoid one long blue chord.
_MAP_TRAIL_MAX_SEGMENT_M = 25_000.0
# User zoom: <1 zooms in (smaller half-extent), >1 zooms out. Clamped in handlers.
_MAP_ZOOM_MIN_SCALE = 0.35
_MAP_ZOOM_MAX_SCALE = 5.0
_MAP_ZOOM_STEP = 1.18
# Target at (0,0) is treated as unset — do not center map on null island.
_MAP_NULL_LAT_TOL = 1.0e-4
_MAP_NULL_LON_TOL = 1.0e-4

# Map styling (dark plot, readable axes)
_MAP_BG = "#0b1220"
_MAP_PLOT_FILL = "#111827"
_MAP_GRID = "#334155"
_MAP_BORDER = "#64748b"
_MAP_TICK = "#cbd5e1"
_MAP_AXIS_LABEL = "#94a3b8"
_MAP_FONT_SMALL = ("Segoe UI", 8)
_MAP_FONT_TICK = ("Consolas", 9)
_MAP_FONT_AXIS = ("Segoe UI", 9, "bold")
_MAP_FONT_LEGEND = ("Consolas", 8)


def _decimate_trail(
    trail: list[tuple[float, float]], max_points: int
) -> list[tuple[float, float]]:
    """Evenly sample along the path, always keeping first and last points.

    Strided ``trail[::step]`` often drops the newest fix and creates uneven gaps
    that look like a broken or 'teleporting' blue path.
    """
    n = len(trail)
    if n <= max_points or max_points < 2:
        return list(trail)
    out: list[tuple[float, float]] = []
    last_i = -1
    denom = max_points - 1
    for j in range(max_points):
        i = int(round(j * (n - 1) / denom))
        i = max(0, min(n - 1, i))
        if i != last_i:
            out.append(trail[i])
            last_i = i
    if out[-1] != trail[-1]:
        out.append(trail[-1])
    return out


def _map_geo_decimals(span_m: float) -> int:
    """Decimal places for tick labels from approximate plot span (meters)."""
    if span_m > 50_000:
        return 2
    if span_m > 5_000:
        return 3
    if span_m > 500:
        return 4
    if span_m > 50:
        return 5
    return 6


def _valid_gps_latlon(lat: float, lon: float) -> bool:
    return (
        math.isfinite(lat)
        and math.isfinite(lon)
        and -90.0 <= lat <= 90.0
        and -180.0 <= lon <= 180.0
    )


def _is_meaningful_target_latlon(lat: float, lon: float) -> bool:
    """False for (0,0) placeholder — map ref should fall back to start/centroid."""
    if not _valid_gps_latlon(lat, lon):
        return False
    return abs(lat) > _MAP_NULL_LAT_TOL or abs(lon) > _MAP_NULL_LON_TOL


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6371000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = (
        math.sin(dphi / 2.0) ** 2
        + math.cos(p1) * math.cos(p2) * math.sin(dlmb / 2.0) ** 2
    )
    return 2.0 * r * math.asin(min(1.0, math.sqrt(a)))


COMMAND_PRESETS = [
    ("CX,ON",   "Telemetry ON"),
    ("CX,OFF",  "Telemetry OFF"),
    ("ST,GPS",  "Sync time to GPS"),
    ("SIM,ENABLE",   "SIM mode enable"),
    ("SIM,ACTIVATE", "SIM mode activate"),
    ("SIM,DISABLE",  "SIM mode disable"),
    ("SIMP,120",     "SIM baro altitude (m)"),
    ("SIMG,37.56,126.93,90,8.5", "SIM GPS lat,lon,course°,speed_m/s"),
    ("SIMG,37.56,126.93,90,8.5,100", "SIM GPS + alt_m"),
    ("TC,37.57,126.94", "Target lat,lon (release)"),
    ("CAL,",         "Calibrate barometer (zero-set)"),
    ("MEC,ON",  "Mechanism ON"),
    ("MEC,OFF", "Mechanism OFF"),
    ("CAM,ON",  "Camera ON"),
    ("CAM,OFF", "Camera OFF"),
    ("SS,0", "Set state 0"),
    ("SS,1", "Set state 1"),
    ("SS,2", "Set state 2"),
    ("SS,3", "Set state 3"),
    ("SS,4", "Set state 4"),
    ("SS,5", "Set state 5"),
    ("XRST,NOW", "XBee reset pulse"),
]


class SerialWorker(threading.Thread):
    """Read lines from the serial port and forward them via a queue."""

    def __init__(self, ser: serial.Serial, rx_queue: "queue.Queue[str]"):
        super().__init__(daemon=True)
        self._ser = ser
        self._rx = rx_queue
        self._stop = threading.Event()

    def stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(256)
            except Exception as exc:
                self._rx.put(f"[serial-error] {exc}")
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
                    self._rx.put(line)


class GroundStation(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("CANSAT AAS 2026 - Ground Station")
        self.geometry("1200x780")
        self.minsize(1100, 700)

        self._ser: serial.Serial | None = None
        self._worker: SerialWorker | None = None
        self._rx_queue: "queue.Queue[str]" = queue.Queue()
        self._csv_file = None
        self._csv_writer = None
        self._packet_count = 0
        self._bad_packet_count = 0
        self._last_packet_ts: float | None = None
        self._tlm_vars: dict[str, tk.StringVar] = {}
        self._track_points: list[tuple[float, float]] = []
        self._map_points: dict[str, tuple[float, float]] = {}
        self._held_start_latlon: tuple[float, float] | None = None
        self._held_target_latlon: tuple[float, float] | None = None
        # 1.0 = auto fit; scale < 1 → zoom in, > 1 → zoom out (applied to map half-extents).
        self._map_user_scale: float = 1.0
        self._current_heading_deg = math.nan
        self._desired_heading_deg = math.nan
        self._left_pulse_us = 0
        self._right_pulse_us = 0
        # Avoid blocking the Tk mainloop: CSV flush / map redraw / console scroll are debounced.
        self._csv_flush_after_id: str | None = None
        self._map_redraw_after_id: str | None = None
        self._map_dirty: bool = False
        self._console_scroll_after_id: str | None = None
        # Latest parsed TLM dict (also fed to the Scenario panel as a feedback
        # source). Updated each time a frame survives _parse_tlm.
        self._latest_tlm: dict[str, str] | None = None
        # Scenario player state (None when no scenario active).
        self._scenario_runner: "_ScenarioRunner | None" = None
        self._scenario_after_id: str | None = None
        self._scenario_setup_cmds: list[str] | None = None
        self._scenario_setup_ix: int = 0
        # Async teardown pump (avoids blocking the Tk loop while spacing
        # SS,5 / SIM,DISABLE on slow XBee links).
        self._scenario_teardown_cmds: list[str] | None = None
        self._scenario_teardown_ix: int = 0
        self._scenario_status_var = tk.StringVar(value="idle")

        self._build_ui()
        self._refresh_ports()
        self.after(_RX_POLL_IDLE_MS, self._drain_rx)
        self.after(500, self._update_status)

    # --------------------------------------------------------------- UI ---
    def _build_ui(self) -> None:
        style = ttk.Style(self)
        try:
            style.theme_use("vista")
        except tk.TclError:
            pass
        style.configure("Stat.TLabel", font=("Consolas", 11))
        style.configure("StatHdr.TLabel", font=("Segoe UI", 10, "bold"))
        style.configure("Big.TLabel", font=("Consolas", 16, "bold"))

        self._build_top_bar()
        body = ttk.Frame(self)
        body.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)
        # uniform: left/right panes keep a stable 1:1 split when the window is resized
        body.columnconfigure(0, weight=1, uniform="gs_body")
        body.columnconfigure(1, weight=1, uniform="gs_body")
        body.rowconfigure(0, weight=1)

        left = ttk.Frame(body)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 4))
        right = ttk.Frame(body)
        right.grid(row=0, column=1, sticky="nsew", padx=(4, 0))
        right.rowconfigure(0, weight=1)
        right.rowconfigure(1, weight=1)
        right.columnconfigure(0, weight=1)

        self._build_telemetry_panel(left)
        self._build_map_and_motor(right)
        self._build_console_and_command(right, row_offset=1)
        self._build_scenario_panel(right, row=3)
        self._build_status_bar()

    def _build_top_bar(self) -> None:
        bar = ttk.Frame(self)
        bar.pack(fill=tk.X, padx=8, pady=8)

        ttk.Label(bar, text="Port:").pack(side=tk.LEFT)
        self._port_var = tk.StringVar()
        self._port_combo = ttk.Combobox(
            bar, textvariable=self._port_var, width=12, state="readonly"
        )
        self._port_combo.pack(side=tk.LEFT, padx=4)

        ttk.Button(bar, text="Refresh", command=self._refresh_ports).pack(side=tk.LEFT)

        ttk.Label(bar, text="  Baud:").pack(side=tk.LEFT)
        # Default 9600: matches FSW UART_BAUD default (see README). Use 38400 if
        # FSW sets UART_BAUD=38400 and XCTU Interface Data Rate matches (BD=5).
        # 9600 can saturate the XBee RX buffer at ~430B telemetry frames
        # and cause bursty / merged-line arrival (see comm/uartserial.py).
        self._baud_var = tk.StringVar(value="38400")
        baud = ttk.Combobox(
            bar, textvariable=self._baud_var, width=8, state="readonly",
            values=("9600", "19200", "38400", "57600", "115200"),
        )
        baud.pack(side=tk.LEFT, padx=4)

        self._connect_btn = ttk.Button(bar, text="Connect", command=self._toggle_connect)
        self._connect_btn.pack(side=tk.LEFT, padx=8)

        self._log_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(bar, text="CSV log", variable=self._log_var).pack(side=tk.LEFT, padx=8)

        self._autoscroll_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            bar, text="Auto-scroll console", variable=self._autoscroll_var
        ).pack(side=tk.LEFT, padx=8)

        ttk.Button(bar, text="Clear console", command=self._clear_console).pack(side=tk.LEFT)

    def _build_telemetry_panel(self, parent: ttk.Frame) -> None:
        wrap = ttk.LabelFrame(parent, text="Telemetry")
        wrap.pack(fill=tk.BOTH, expand=True)

        groups: list[tuple[str, list[tuple[str, str]]]] = [
            ("Header", [
                ("team_id", "Team ID"),
                ("time", "Time"),
                ("packet_count", "Packet"),
                ("mode", "Mode"),
                ("state", "State"),
            ]),
            ("Barometer", [
                ("altitude_m", "Altitude (m)"),
                ("temperature_c", "Temp (°C)"),
                ("pressure_hpa", "Pressure (hPa)"),
            ]),
            ("Power", [
                ("voltage_v", "Voltage (V)"),
                ("current_a", "Current (A)"),
                ("power_w", "Power (W)"),
            ]),
            ("Gyro (deg/s)", [
                ("gyro_roll", "roll"),
                ("gyro_pitch", "pitch"),
                ("gyro_yaw", "yaw"),
            ]),
            ("Accel (g)", [
                ("acc_roll", "roll"),
                ("acc_pitch", "pitch"),
                ("acc_yaw", "yaw"),
            ]),
            ("Mag (uT)", [
                ("mag_roll", "roll"),
                ("mag_pitch", "pitch"),
                ("mag_yaw", "yaw"),
            ]),
            ("GPS", [
                ("gps_time", "Time"),
                ("gps_alt", "Alt (m)"),
                ("gps_lat", "Lat"),
                ("gps_lon", "Lon"),
                ("gps_sats", "Sats"),
            ]),
            ("Distance / Echo", [
                ("distance_mm", "Distance (mm)"),
                ("cmd_echo", "Cmd echo"),
            ]),
            ("Filtered (deg)", [
                ("filtered_roll", "roll"),
                ("filtered_pitch", "pitch"),
                ("filtered_yaw", "yaw"),
            ]),
        ]

        for col in range(2):
            wrap.columnconfigure(col, weight=1, uniform="tlm_cols")

        for idx, (title, fields) in enumerate(groups):
            row, col = divmod(idx, 2)
            box = ttk.LabelFrame(wrap, text=title)
            box.grid(row=row, column=col, sticky="nsew", padx=6, pady=4)
            box.columnconfigure(1, weight=1)
            for i, (key, label) in enumerate(fields):
                ttk.Label(box, text=label, style="StatHdr.TLabel").grid(
                    row=i, column=0, sticky="w", padx=6, pady=2
                )
                var = tk.StringVar(value="—")
                self._tlm_vars[key] = var
                vw = _TLM_VALUE_WIDTH.get(key, _TLM_VALUE_WIDTH_DEFAULT)
                ttk.Label(
                    box,
                    textvariable=var,
                    style="Stat.TLabel",
                    width=vw,
                    anchor="e",
                ).grid(row=i, column=1, sticky="e", padx=6, pady=2)

        for r in range((len(groups) + 1) // 2):
            wrap.rowconfigure(r, weight=1)

    def _build_map_and_motor(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Guidance Map / Motor")
        frame.grid(row=0, column=0, sticky="nsew")
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=0)
        frame.rowconfigure(1, weight=1)
        frame.rowconfigure(2, weight=0)

        map_tool = ttk.Frame(frame)
        map_tool.grid(row=0, column=0, sticky="ew", padx=6, pady=(6, 0))
        ttk.Button(map_tool, text="Zoom −", width=8, command=self._map_zoom_out).pack(
            side=tk.LEFT, padx=(0, 4)
        )
        ttk.Button(map_tool, text="Zoom +", width=8, command=self._map_zoom_in).pack(
            side=tk.LEFT, padx=(0, 4)
        )
        ttk.Button(map_tool, text="Fit", width=7, command=self._map_zoom_reset).pack(
            side=tk.LEFT, padx=(0, 4)
        )
        ttk.Button(map_tool, text="Clear trail", width=11, command=self._clear_gps_trail).pack(
            side=tk.LEFT, padx=(8, 0)
        )

        self._map_canvas = tk.Canvas(
            frame,
            height=300,
            background=_MAP_BG,
            highlightthickness=1,
            highlightbackground="#2d3748",
        )
        self._map_canvas.grid(row=1, column=0, sticky="nsew", padx=6, pady=6)
        self._map_canvas.bind("<Configure>", lambda _e: self._request_map_redraw())
        self._map_canvas.bind("<MouseWheel>", self._on_map_mousewheel)

        bars = ttk.Frame(frame)
        bars.grid(row=2, column=0, sticky="ew", padx=6, pady=(0, 6))
        bars.columnconfigure(1, weight=1)
        bars.columnconfigure(3, weight=1)

        ttk.Label(bars, text="Left pulse").grid(row=0, column=0, sticky="w")
        self._left_bar = ttk.Progressbar(bars, orient="horizontal", mode="determinate", maximum=1000)
        self._left_bar.grid(row=0, column=1, sticky="ew", padx=(6, 10))
        self._left_pulse_var = tk.StringVar(value="0 us")
        ttk.Label(bars, textvariable=self._left_pulse_var, width=10).grid(row=0, column=2, sticky="e")

        ttk.Label(bars, text="Right pulse").grid(row=1, column=0, sticky="w")
        self._right_bar = ttk.Progressbar(bars, orient="horizontal", mode="determinate", maximum=1000)
        self._right_bar.grid(row=1, column=1, sticky="ew", padx=(6, 10))
        self._right_pulse_var = tk.StringVar(value="0 us")
        ttk.Label(bars, textvariable=self._right_pulse_var, width=10).grid(row=1, column=2, sticky="e")

        self._heading_var = tk.StringVar(value="heading: -- / desired hdg: --")
        ttk.Label(
            bars,
            textvariable=self._heading_var,
            width=46,
            anchor="w",
        ).grid(row=0, column=3, rowspan=2, sticky="w")
        self._guidance_var = tk.StringVar(value="guidance: --")
        ttk.Label(
            bars,
            textvariable=self._guidance_var,
            width=96,
            anchor="w",
        ).grid(row=2, column=0, columnspan=4, sticky="w", pady=(4, 0))

    def _build_console_and_command(self, parent: ttk.Frame, row_offset: int = 0) -> None:
        parent.rowconfigure(row_offset, weight=1)
        parent.rowconfigure(row_offset + 1, weight=0)
        parent.columnconfigure(0, weight=1)

        console_box = ttk.LabelFrame(parent, text="Raw RX (XBee)")
        console_box.grid(row=row_offset, column=0, sticky="nsew", pady=(8, 0))
        console_box.rowconfigure(0, weight=1)
        console_box.columnconfigure(0, weight=1)

        self._console = tk.Text(
            console_box, wrap="none", height=18, font=("Consolas", 9),
            background="#0e1116", foreground="#d6deeb", insertbackground="#d6deeb",
        )
        self._console.grid(row=0, column=0, sticky="nsew")
        yscroll = ttk.Scrollbar(console_box, orient="vertical", command=self._console.yview)
        yscroll.grid(row=0, column=1, sticky="ns")
        self._console.configure(yscrollcommand=yscroll.set)
        self._console.tag_configure("ok", foreground="#9ece6a")
        self._console.tag_configure("warn", foreground="#e0af68")
        self._console.tag_configure("err", foreground="#f7768e")
        self._console.tag_configure("tx", foreground="#7aa2f7")

        cmd_box = ttk.LabelFrame(parent, text="Command (CMD,1070,...)")
        cmd_box.grid(row=row_offset + 1, column=0, sticky="ew", pady=(8, 0))
        cmd_box.columnconfigure(1, weight=1)

        ttk.Label(cmd_box, text="Preset:").grid(row=0, column=0, padx=6, pady=4, sticky="w")
        self._preset_var = tk.StringVar()
        preset = ttk.Combobox(
            cmd_box, textvariable=self._preset_var, state="readonly",
            values=[f"{cmd:<12}  {desc}" for cmd, desc in COMMAND_PRESETS],
            width=40,
        )
        preset.grid(row=0, column=1, padx=6, pady=4, sticky="ew")
        preset.bind("<<ComboboxSelected>>", self._on_preset_selected)

        ttk.Label(cmd_box, text="Body:").grid(row=1, column=0, padx=6, pady=4, sticky="w")
        self._cmd_var = tk.StringVar(value="CX,ON")
        entry = ttk.Entry(cmd_box, textvariable=self._cmd_var)
        entry.grid(row=1, column=1, padx=6, pady=4, sticky="ew")
        entry.bind("<Return>", lambda _e: self._send_command())

        ttk.Button(cmd_box, text="Send", command=self._send_command).grid(
            row=1, column=2, padx=6, pady=4
        )

        force_box = ttk.Frame(cmd_box)
        force_box.grid(row=2, column=0, columnspan=3, sticky="ew", padx=6, pady=(2, 6))
        ttk.Label(force_box, text="Force action:").pack(side=tk.LEFT)
        ttk.Button(
            force_box,
            text="Force RELEASE motor",
            command=self._send_force_release,
        ).pack(side=tk.LEFT, padx=(8, 6))
        ttk.Button(
            force_box,
            text="Force EGG motor",
            command=self._send_force_egg,
        ).pack(side=tk.LEFT, padx=6)

    def _build_scenario_panel(self, parent: ttk.Frame, row: int) -> None:
        """Closed-loop scenario player panel — preset + overrides + controls.

        Shows a brief description, lets the operator override the most common
        environmental knobs, and animates the result on the existing map by
        sending SIMG / SIMP at the scenario tick rate.
        """
        parent.rowconfigure(row, weight=0)

        if not _SCENARIO_AVAILABLE:
            box = ttk.LabelFrame(parent, text="Scenario player (unavailable)")
            box.grid(row=row, column=0, sticky="ew", pady=(8, 0))
            ttk.Label(
                box,
                text="scenario_runner.py 가 import되지 않아 비활성. "
                     "ground_station/scenario_runner.py 가 같은 폴더에 있는지 확인하세요.",
                foreground="#f87171",
                wraplength=500,
            ).pack(padx=8, pady=6, anchor="w")
            return

        box = ttk.LabelFrame(parent, text="Scenario player (closed-loop SIM)")
        box.grid(row=row, column=0, sticky="ew", pady=(8, 0))
        for c in range(6):
            box.columnconfigure(c, weight=0)
        box.columnconfigure(1, weight=1)

        preset_names = _scenario_list_presets()
        default_name = preset_names[0] if preset_names else ""

        ttk.Label(box, text="Preset:").grid(row=0, column=0, padx=6, pady=4, sticky="w")
        self._scenario_preset_var = tk.StringVar(value=default_name)
        preset_combo = ttk.Combobox(
            box, textvariable=self._scenario_preset_var,
            state="readonly", values=preset_names, width=22,
        )
        preset_combo.grid(row=0, column=1, padx=6, pady=4, sticky="ew")
        preset_combo.bind("<<ComboboxSelected>>", self._on_scenario_preset_change)

        self._scenario_play_btn = ttk.Button(
            box, text="Play", width=8, command=self._on_scenario_play
        )
        self._scenario_play_btn.grid(row=0, column=2, padx=(8, 4), pady=4)
        self._scenario_stop_btn = ttk.Button(
            box, text="Stop", width=8, state="disabled",
            command=self._on_scenario_stop,
        )
        self._scenario_stop_btn.grid(row=0, column=3, padx=4, pady=4)

        # Description / status spans the row.
        self._scenario_desc_var = tk.StringVar(
            value=_SCENARIO_PRESETS[default_name].description if default_name else ""
        )
        ttk.Label(
            box, textvariable=self._scenario_desc_var,
            foreground="#94a3b8", wraplength=560,
        ).grid(row=1, column=0, columnspan=4, sticky="w", padx=6, pady=(0, 4))

        # Optional overrides — blank means "use preset value".
        ttk.Label(box, text="Wind speed (m/s):").grid(row=2, column=0, sticky="w", padx=6, pady=2)
        self._scenario_wind_speed_var = tk.StringVar()
        ttk.Entry(box, textvariable=self._scenario_wind_speed_var, width=8).grid(
            row=2, column=1, sticky="w", padx=6
        )
        ttk.Label(box, text="Wind dir FROM (deg):").grid(row=2, column=2, sticky="e", padx=6)
        self._scenario_wind_dir_var = tk.StringVar()
        ttk.Entry(box, textvariable=self._scenario_wind_dir_var, width=8).grid(
            row=2, column=3, sticky="w", padx=6
        )

        ttk.Label(box, text="Descent (m/s):").grid(row=3, column=0, sticky="w", padx=6, pady=2)
        self._scenario_descent_var = tk.StringVar()
        ttk.Entry(box, textvariable=self._scenario_descent_var, width=8).grid(
            row=3, column=1, sticky="w", padx=6
        )
        ttk.Label(box, text="Airspeed (m/s):").grid(row=3, column=2, sticky="e", padx=6)
        self._scenario_airspeed_var = tk.StringVar()
        ttk.Entry(box, textvariable=self._scenario_airspeed_var, width=8).grid(
            row=3, column=3, sticky="w", padx=6
        )

        ttk.Label(box, textvariable=self._scenario_status_var,
                  font=("Consolas", 9), foreground="#bae6fd").grid(
            row=4, column=0, columnspan=4, sticky="w", padx=6, pady=(4, 6)
        )

    def _on_scenario_preset_change(self, _event=None) -> None:
        if not _SCENARIO_AVAILABLE:
            return
        name = self._scenario_preset_var.get()
        cfg = _SCENARIO_PRESETS.get(name)
        if cfg is None:
            return
        self._scenario_desc_var.set(cfg.description)

    def _scenario_override_config(self, cfg) -> None:
        """Apply non-empty Entry overrides onto a preset config (in-place)."""
        def _maybe_float(var: tk.StringVar) -> float | None:
            s = var.get().strip()
            if s == "":
                return None
            try:
                v = float(s)
            except ValueError:
                return None
            if not math.isfinite(v):
                return None
            return v

        v = _maybe_float(self._scenario_wind_speed_var)
        if v is not None and v >= 0.0:
            cfg.wind_speed_ms = v
        v = _maybe_float(self._scenario_wind_dir_var)
        if v is not None:
            cfg.wind_dir_met_deg = v % 360.0
        v = _maybe_float(self._scenario_descent_var)
        if v is not None and v > 0.0:
            cfg.descent_rate_ms = v
        v = _maybe_float(self._scenario_airspeed_var)
        if v is not None and v > 0.0:
            cfg.airspeed_ms = v

    def _on_scenario_play(self) -> None:
        if not _SCENARIO_AVAILABLE:
            return
        if self._ser is None:
            messagebox.showwarning("Not connected", "먼저 포트에 연결하세요.")
            return
        if self._scenario_runner is not None:
            return  # already running
        name = self._scenario_preset_var.get()
        try:
            cfg = _scenario_get_preset(name)
        except KeyError:
            messagebox.showerror("Scenario", f"unknown preset: {name}")
            return
        # Each play uses a fresh dataclass copy so overrides don't pollute the
        # global preset table across runs.
        from copy import deepcopy
        cfg = deepcopy(cfg)
        self._scenario_override_config(cfg)

        # Map view: clear leftover trail so the new run starts clean.
        self._clear_gps_trail()

        runner = _ScenarioRunner(
            send_cb=self._send_body,
            get_tlm_cb=self._get_latest_tlm,
            log_cb=lambda msg: self._append_console(msg, "ok"),
            config=cfg,
        )
        self._scenario_runner = runner
        self._scenario_play_btn.configure(state="disabled")
        self._scenario_stop_btn.configure(state="normal")
        self._scenario_status_var.set(f"starting: {cfg.name}")
        try:
            self._scenario_setup_cmds = runner.begin_async_setup()
        except RuntimeError as exc:
            self._append_console(f"[scenario] setup error: {exc}", "err")
            self._scenario_runner = None
            self._scenario_play_btn.configure(state="normal")
            self._scenario_stop_btn.configure(state="disabled")
            return
        self._scenario_setup_ix = 0
        self._pump_scenario_setup()

    def _pump_scenario_setup(self) -> None:
        runner = self._scenario_runner
        cmds = self._scenario_setup_cmds
        if runner is None or cmds is None:
            return
        ix = self._scenario_setup_ix
        if ix >= len(cmds):
            runner.complete_async_setup()
            self._scenario_setup_cmds = None
            self._scenario_setup_ix = 0
            cfg = runner.config
            first_tick_ms = max(
                800, min(5000, int(cfg.setup_inter_cmd_delay_s * 600))
            )
            self._scenario_after_id = self.after(first_tick_ms, self._scenario_tick)
            return
        self._send_body(cmds[ix])
        self._scenario_setup_ix = ix + 1
        delay_ms = max(0, int(runner.config.setup_inter_cmd_delay_s * 1000))
        self._scenario_after_id = self.after(delay_ms, self._pump_scenario_setup)

    def _scenario_tick(self) -> None:
        self._scenario_after_id = None
        runner = self._scenario_runner
        if runner is None:
            return
        cfg = runner.config
        spacing_ms = max(0, int(cfg.simg_simp_spacing_s * 1000))
        gap_after_simp_ms = max(
            0, int((cfg.tick_period_s - cfg.simg_simp_spacing_s) * 1000)
        )
        try:
            if runner.awaiting_simp():
                still_running = runner.tick_send_simp()
                delay_ms = gap_after_simp_ms if still_running else None
            else:
                still_running = runner.tick_integrate_and_simg()
                delay_ms = spacing_ms if still_running else None
        except Exception as exc:
            self._append_console(f"[scenario] tick error: {exc}", "err")
            still_running = False
            delay_ms = None
        s = runner.state
        self._scenario_status_var.set(
            f"{runner.config.name}  t={s.elapsed_s:5.1f}s  "
            f"alt={s.alt_m:6.1f}m  d={s.distance_to_target_m:6.1f}m  "
            f"hdg={s.heading_deg:5.1f}deg  yr={s.yaw_rate_deg_s:+5.1f}deg/s"
        )
        if still_running and delay_ms is not None:
            self._scenario_after_id = self.after(delay_ms, self._scenario_tick)
        elif not still_running:
            self._finalise_scenario(send_teardown=True, reason=s.finish_reason or "complete")

    def _on_scenario_stop(self) -> None:
        if self._scenario_runner is None:
            return
        self._finalise_scenario(send_teardown=True, reason="user-stop")

    def _finalise_scenario(self, *, send_teardown: bool, reason: str) -> None:
        runner = self._scenario_runner
        if runner is None:
            return
        if self._scenario_after_id is not None:
            try:
                self.after_cancel(self._scenario_after_id)
            except tk.TclError:
                pass
            self._scenario_after_id = None
        self._scenario_setup_cmds = None
        self._scenario_setup_ix = 0
        # Mark the runner stopped without sending teardown synchronously —
        # SS,5 / SIM,DISABLE pacing happens in _pump_scenario_teardown so the
        # Tk loop stays responsive on slow UART links.
        try:
            runner.stop(send_teardown=False, reason=reason)
        except Exception as exc:
            self._append_console(f"[scenario] stop error: {exc}", "err")

        teardown_cmds = (
            runner.teardown_command_sequence(reason) if send_teardown else []
        )
        self._scenario_teardown_cmds = teardown_cmds
        self._scenario_teardown_ix = 0
        s = runner.state
        if teardown_cmds:
            self._scenario_status_var.set(
                f"stopping: {runner.config.name}  reason={s.finish_reason}"
            )
            self._scenario_play_btn.configure(state="disabled")
            self._scenario_stop_btn.configure(state="disabled")
            self._pump_scenario_teardown()
        else:
            self._scenario_status_var.set(
                f"done: {runner.config.name}  reason={s.finish_reason}  "
                f"final_d={s.distance_to_target_m:.1f}m  alt={s.alt_m:.1f}m"
            )
            self._scenario_runner = None
            self._scenario_teardown_cmds = None
            self._scenario_play_btn.configure(state="normal")
            self._scenario_stop_btn.configure(state="disabled")

    def _pump_scenario_teardown(self) -> None:
        """Send one teardown command, wait ``teardown_inter_cmd_delay_s``, repeat."""
        self._scenario_after_id = None
        runner = self._scenario_runner
        cmds = self._scenario_teardown_cmds
        if runner is None or cmds is None:
            return
        ix = self._scenario_teardown_ix
        if ix >= len(cmds):
            s = runner.state
            self._scenario_status_var.set(
                f"done: {runner.config.name}  reason={s.finish_reason}  "
                f"final_d={s.distance_to_target_m:.1f}m  alt={s.alt_m:.1f}m"
            )
            self._scenario_runner = None
            self._scenario_teardown_cmds = None
            self._scenario_teardown_ix = 0
            self._scenario_play_btn.configure(state="normal")
            self._scenario_stop_btn.configure(state="disabled")
            return
        self._send_body(cmds[ix])
        self._scenario_teardown_ix = ix + 1
        delay_ms = max(0, int(runner.config.teardown_inter_cmd_delay_s * 1000))
        self._scenario_after_id = self.after(delay_ms, self._pump_scenario_teardown)

    def _build_status_bar(self) -> None:
        bar = ttk.Frame(self)
        bar.pack(fill=tk.X, padx=8, pady=(0, 6))
        self._status_var = tk.StringVar(value="Disconnected")
        ttk.Label(bar, textvariable=self._status_var, width=36, anchor="w").pack(
            side=tk.LEFT
        )
        self._rate_var = tk.StringVar(value="rx: 0 / err: 0 / rate: -- Hz")
        ttk.Label(bar, textvariable=self._rate_var, width=38, anchor="e").pack(
            side=tk.RIGHT
        )

    # ---------------------------------------------------------- actions ---
    def _refresh_ports(self) -> None:
        ports = sorted({p.device for p in list_ports.comports()})
        self._port_combo["values"] = ports
        if ports and not self._port_var.get():
            self._port_var.set(ports[0])

    def _toggle_connect(self) -> None:
        if self._ser is None:
            self._connect()
        else:
            self._disconnect()

    def _connect(self) -> None:
        port = self._port_var.get().strip()
        if not port:
            messagebox.showerror("Port required", "선택된 COM 포트가 없습니다.")
            return
        try:
            baud = int(self._baud_var.get())
        except ValueError:
            baud = 9600
        try:
            ser = serial.Serial(port, baud, timeout=0.2)
        except Exception as exc:
            messagebox.showerror("Open failed", f"{port} @ {baud}\n{exc}")
            return

        self._ser = ser
        self._worker = SerialWorker(ser, self._rx_queue)
        self._worker.start()
        if self._log_var.get():
            self._open_csv()
        self._connect_btn.configure(text="Disconnect")
        self._status_var.set(f"Connected {port} @ {baud}")
        self._append_console(f"[connect] {port} @ {baud}", "ok")

    def _disconnect(self) -> None:
        # Stop a running scenario first so it doesn't try to write to a closed
        # serial port. send_teardown=False because the port may already be
        # gone — best-effort cleanup only.
        if self._scenario_runner is not None:
            self._finalise_scenario(send_teardown=False, reason="serial-disconnect")
        if self._map_redraw_after_id is not None:
            try:
                self.after_cancel(self._map_redraw_after_id)
            except tk.TclError:
                pass
            self._map_redraw_after_id = None
        self._map_dirty = False
        if self._console_scroll_after_id is not None:
            try:
                self.after_cancel(self._console_scroll_after_id)
            except tk.TclError:
                pass
            self._console_scroll_after_id = None
        if self._worker is not None:
            self._worker.stop()
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        self._worker = None
        self._close_csv()
        self._connect_btn.configure(text="Connect")
        self._status_var.set("Disconnected")
        self._append_console("[disconnect]", "warn")

    def _open_csv(self) -> None:
        try:
            log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
            os.makedirs(log_dir, exist_ok=True)
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = os.path.join(log_dir, f"gs_{ts}.csv")
            self._csv_file = open(path, "w", newline="", encoding="utf-8")
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow(["host_time"] + TLM_FIELDS + ["raw"])
            self._csv_file.flush()
            self._append_console(f"[csv] writing -> {path}", "ok")
        except Exception as exc:
            self._append_console(f"[csv-error] {exc}", "err")
            self._csv_writer = None
            self._csv_file = None

    def _close_csv(self) -> None:
        if self._csv_flush_after_id is not None:
            try:
                self.after_cancel(self._csv_flush_after_id)
            except tk.TclError:
                pass
            self._csv_flush_after_id = None
        if self._csv_file is not None:
            try:
                self._csv_file.flush()
            except Exception:
                pass
            try:
                self._csv_file.close()
            except Exception:
                pass
        self._csv_file = None
        self._csv_writer = None

    def _on_preset_selected(self, _event=None) -> None:
        sel = self._preset_var.get().split("  ", 1)[0].strip()
        if sel:
            self._cmd_var.set(sel)

    def _send_command(self) -> None:
        if self._ser is None:
            messagebox.showwarning("Not connected", "먼저 포트에 연결하세요.")
            return
        body = self._cmd_var.get().strip()
        if not body:
            return
        if not self._send_body(body):
            return
        ubody = body.strip().upper().replace(" ", "")
        # Manual SIMG entry = new leg, so reset the blue trail. Scenario-mode
        # SIMGs go through _send_body without this reset and accumulate.
        if ubody.startswith("SIMG,"):
            self._clear_gps_trail()

    def _send_force_release(self) -> None:
        """Force release actuator path via state jump command."""
        if self._ser is None:
            messagebox.showwarning("Not connected", "먼저 포트에 연결하세요.")
            return
        ok = messagebox.askyesno(
            "Force RELEASE",
            "강제 RELEASE(SS,3)를 전송합니다.\n"
            "주의: target(TC) 미설정 시 FlightLogic에서 차단됩니다.\n"
            "계속할까요?",
        )
        if not ok:
            return
        self._send_body("SS,3")

    def _send_force_egg(self) -> None:
        """Force egg-drop actuator path via state jump command."""
        if self._ser is None:
            messagebox.showwarning("Not connected", "먼저 포트에 연결하세요.")
            return
        ok = messagebox.askyesno(
            "Force EGG",
            "강제 EGG(SS,4)를 전송합니다.\n"
            "주의: 즉시 에그 솔레노이드 트리거 조건으로 진입할 수 있습니다.\n"
            "계속할까요?",
        )
        if not ok:
            return
        self._send_body("SS,4")

    def _send_body(self, body: str) -> bool:
        """Low-level CMD send used by both manual entry and scenario player.

        Does NOT reset the GPS trail; the caller decides (the manual UI does).
        """
        if self._ser is None:
            return False
        body = (body or "").strip()
        if not body:
            return False
        # Reject pre-prefixed commands the operator likely pasted in by accident.
        upper = body.upper()
        if upper.startswith("CMD,") or body.startswith("$"):
            self._append_console(
                f"[TX-error] body must not start with 'CMD,' or '$' — got {body!r}",
                "err",
            )
            return False
        line = f"CMD,{TEAM_ID},{body}\n"
        try:
            self._ser.write(line.encode("utf-8"))
            self._append_console(f"[TX] {line.strip()}", "tx")
            return True
        except Exception as exc:
            self._append_console(f"[TX-error] {exc}", "err")
            return False

    def _get_latest_tlm(self) -> dict | None:
        """Hook exposed to the Scenario runner for closed-loop feedback."""
        return self._latest_tlm

    def _clear_gps_trail(self) -> None:
        """Drop path history (e.g. new SIMG leg or operator reset)."""
        self._track_points.clear()
        self._request_map_redraw()

    def _map_zoom_in(self) -> None:
        self._map_user_scale = max(
            _MAP_ZOOM_MIN_SCALE, self._map_user_scale / _MAP_ZOOM_STEP
        )
        self._request_map_redraw()

    def _map_zoom_out(self) -> None:
        self._map_user_scale = min(
            _MAP_ZOOM_MAX_SCALE, self._map_user_scale * _MAP_ZOOM_STEP
        )
        self._request_map_redraw()

    def _map_zoom_reset(self) -> None:
        self._map_user_scale = 1.0
        self._request_map_redraw()

    def _on_map_mousewheel(self, event: tk.Event) -> None:
        if getattr(event, "delta", 0) > 0:
            self._map_zoom_in()
        else:
            self._map_zoom_out()

    def _parse_optional_float(self, value: str) -> float | None:
        try:
            v = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(v):
            return None
        return v

    def _ingest_tlm_track(self, parsed: dict[str, str]) -> None:
        """Append GPS to trail — call for every telemetry row so the path stays correct when UI is coalesced."""
        cur_lat = self._parse_optional_float(parsed.get("gps_lat", ""))
        cur_lon = self._parse_optional_float(parsed.get("gps_lon", ""))
        if cur_lat is None or cur_lon is None:
            return
        # Drop (0,0) placeholder so the trail does not pin a phantom point on null island
        # while waiting for the first real GPS fix.
        if not _is_meaningful_target_latlon(cur_lat, cur_lon):
            return
        if self._track_points:
            la, lo = self._track_points[-1]
            if _haversine_m(la, lo, cur_lat, cur_lon) > _MAP_TRAIL_MAX_SEGMENT_M:
                return
        self._track_points.append((cur_lat, cur_lon))
        if len(self._track_points) > 500:
            self._track_points = self._track_points[-500:]

    def _apply_telemetry_ui(self, parsed: dict[str, str]) -> None:
        """Refresh telemetry labels, motor bars, map layers from one frame (latest in a batch)."""
        # Cache so the Scenario runner can read closed-loop feedback (left/right
        # pulse, etc.) without re-parsing the raw line.
        self._latest_tlm = parsed
        self._apply_to_ui(parsed)
        cur_lat = self._parse_optional_float(parsed.get("gps_lat", ""))
        cur_lon = self._parse_optional_float(parsed.get("gps_lon", ""))
        start_lat = self._parse_optional_float(parsed.get("start_lat", ""))
        start_lon = self._parse_optional_float(parsed.get("start_lon", ""))
        target_lat = self._parse_optional_float(parsed.get("target_lat", ""))
        target_lon = self._parse_optional_float(parsed.get("target_lon", ""))
        carrot_lat = self._parse_optional_float(parsed.get("carrot_lat", ""))
        carrot_lon = self._parse_optional_float(parsed.get("carrot_lon", ""))
        cur_hdg = self._parse_optional_float(parsed.get("current_heading_deg", ""))
        des_hdg = self._parse_optional_float(parsed.get("desired_heading_deg", ""))

        if start_lat is not None and start_lon is not None:
            self._held_start_latlon = (start_lat, start_lon)
        if (
            target_lat is not None
            and target_lon is not None
            and _is_meaningful_target_latlon(target_lat, target_lon)
        ):
            self._held_target_latlon = (target_lat, target_lon)

        self._map_points = {}
        if self._held_start_latlon is not None:
            self._map_points["start"] = self._held_start_latlon
        if self._held_target_latlon is not None:
            tg = self._held_target_latlon
            if _is_meaningful_target_latlon(tg[0], tg[1]):
                self._map_points["target"] = tg
        if carrot_lat is not None and carrot_lon is not None:
            self._map_points["carrot"] = (carrot_lat, carrot_lon)
        if (
            cur_lat is not None
            and cur_lon is not None
            and _valid_gps_latlon(cur_lat, cur_lon)
        ):
            self._map_points["current"] = (cur_lat, cur_lon)

        self._current_heading_deg = cur_hdg if cur_hdg is not None else math.nan
        self._desired_heading_deg = des_hdg if des_hdg is not None else math.nan

        left_pulse = self._parse_optional_float(parsed.get("left_pulse_us", ""))
        right_pulse = self._parse_optional_float(parsed.get("right_pulse_us", ""))
        self._left_pulse_us = int(left_pulse) if left_pulse is not None else 0
        self._right_pulse_us = int(right_pulse) if right_pulse is not None else 0
        self._left_bar["value"] = max(0, min(1000, self._left_pulse_us - 1000))
        self._right_bar["value"] = max(0, min(1000, self._right_pulse_us - 1000))
        self._left_pulse_var.set(f"{self._left_pulse_us} us")
        self._right_pulse_var.set(f"{self._right_pulse_us} us")
        ch = "--" if not math.isfinite(self._current_heading_deg) else f"{self._current_heading_deg:.1f}deg"
        dh = "--" if not math.isfinite(self._desired_heading_deg) else f"{self._desired_heading_deg:.1f}deg"
        self._heading_var.set(f"heading: {ch} / desired hdg: {dh}")
        gstate = parsed.get("guidance_state", "").strip() or "--"
        self._guidance_var.set(f"guidance: {gstate}")
        self._request_map_redraw()

    def _request_map_redraw(self) -> None:
        self._map_dirty = True
        if self._map_redraw_after_id is not None:
            return
        self._map_redraw_after_id = self.after(48, self._flush_map_redraw)

    def _flush_map_redraw(self) -> None:
        self._map_redraw_after_id = None
        if not self._map_dirty:
            return
        self._map_dirty = False
        self._draw_map()

    def _draw_map(self) -> None:
        c = self._map_canvas
        c.delete("all")
        w = max(10, c.winfo_width())
        h = max(10, c.winfo_height())

        margin_l, margin_r = 56, 10
        margin_t, margin_b = 14, 44

        all_pts = list(self._track_points) + list(self._map_points.values())
        if not all_pts:
            c.create_text(
                w / 2,
                h / 2,
                text="Waiting for GPS / guidance map telemetry…",
                fill="#94a3b8",
                font=_MAP_FONT_AXIS,
            )
            return

        lats = [p[0] for p in all_pts]
        lons = [p[1] for p in all_pts]
        lat_min, lat_max = min(lats), max(lats)
        lon_min, lon_max = min(lons), max(lons)

        ref_lat: float
        ref_lon: float
        tg = self._map_points.get("target")
        if tg is not None and _is_meaningful_target_latlon(tg[0], tg[1]):
            ref_lat, ref_lon = tg[0], tg[1]
        elif "start" in self._map_points:
            ref_lat, ref_lon = self._map_points["start"]
        else:
            ref_lat = (lat_min + lat_max) / 2.0
            ref_lon = (lon_min + lon_max) / 2.0

        meter_per_lon = 111320.0 * math.cos(math.radians(ref_lat))
        meter_per_lon = meter_per_lon if abs(meter_per_lon) > 1e-6 else 1.0

        half_e_data = 0.0
        half_n_data = 0.0
        for la, lo in all_pts:
            east = (lo - ref_lon) * meter_per_lon
            north = (la - ref_lat) * 111320.0
            half_e_data = max(half_e_data, abs(east))
            half_n_data = max(half_n_data, abs(north))

        st_pair = self._map_points.get("start")
        tg_pair = self._map_points.get("target")
        if (
            st_pair is not None
            and tg_pair is not None
            and _is_meaningful_target_latlon(tg_pair[0], tg_pair[1])
        ):
            dlat_m = abs(st_pair[0] - tg_pair[0]) * 111320.0
            dlon_m = abs(st_pair[1] - tg_pair[1]) * meter_per_lon
            sep_m = math.hypot(dlat_m, dlon_m)
            pad_m = sep_m * _MAP_START_TARGET_PAD_FACTOR / 2.0
            half_e_data = max(half_e_data, pad_m)
            half_n_data = max(half_n_data, pad_m)

        base_e = max(half_e_data * _MAP_VIEW_MARGIN, _MAP_VIEW_MIN_HALF_M)
        base_n = max(half_n_data * _MAP_VIEW_MARGIN, _MAP_VIEW_MIN_HALF_M)
        half_e = min(_MAP_REF_MAX_HALF_M, base_e * self._map_user_scale)
        half_n = min(_MAP_REF_MAX_HALF_M, base_n * self._map_user_scale)
        dec = _map_geo_decimals(max(2.0 * half_e, 2.0 * half_n, 5.0))

        pl = float(margin_l)
        pr = float(w - margin_r)
        pt = float(margin_t)
        pb = float(h - margin_b)
        pw = max(20.0, pr - pl)
        ph = max(20.0, pb - pt)

        def project(lat: float, lon: float) -> tuple[float, float]:
            east = (lon - ref_lon) * meter_per_lon
            north = (lat - ref_lat) * 111320.0
            x = pl + pw / 2.0 + (east / half_e) * (pw / 2.0)
            y = pt + ph / 2.0 - (north / half_n) * (ph / 2.0)
            return x, y

        def inv_lon(px: float) -> float:
            east = ((px - pl - pw / 2.0) / (pw / 2.0)) * half_e
            return ref_lon + east / meter_per_lon

        def inv_lat(py: float) -> float:
            north = ((pt + ph / 2.0 - py) / (ph / 2.0)) * half_n
            return ref_lat + north / 111320.0

        # Plot background + border
        c.create_rectangle(pl, pt, pr, pb, fill=_MAP_PLOT_FILL, outline=_MAP_BORDER, width=2)

        # Grid (geographic — constant lat / constant lon lines through plot)
        n_grid = 5
        for i in range(1, n_grid):
            gx = pl + (pw * i / n_grid)
            c.create_line(gx, pt, gx, pb, fill=_MAP_GRID, width=1, dash=(3, 5))
        for i in range(1, n_grid):
            gy = pt + (ph * i / n_grid)
            c.create_line(pl, gy, pr, gy, fill=_MAP_GRID, width=1, dash=(3, 5))

        # Axis ticks: longitude along bottom, latitude along left
        n_ticks = 5
        for i in range(n_ticks + 1):
            t = i / n_ticks
            px = pl + pw * t
            lon_v = inv_lon(px)
            c.create_line(px, pb, px, pb + 5, fill=_MAP_TICK, width=2)
            c.create_text(
                px,
                pb + 8,
                text=f"{lon_v:.{dec}f}°",
                fill=_MAP_TICK,
                font=_MAP_FONT_TICK,
                anchor="n",
            )

        for i in range(n_ticks + 1):
            t = i / n_ticks
            py = pb - ph * t
            lat_v = inv_lat(py)
            c.create_line(pl - 5, py, pl, py, fill=_MAP_TICK, width=2)
            c.create_text(
                pl - 8,
                py,
                text=f"{lat_v:.{dec}f}°",
                fill=_MAP_TICK,
                font=_MAP_FONT_TICK,
                anchor="e",
            )

        c.create_text(
            (pl + pr) / 2.0,
            h - 6,
            text="Longitude (°)  —  east →",
            fill=_MAP_AXIS_LABEL,
            font=_MAP_FONT_AXIS,
            anchor="s",
        )
        c.create_text(
            8,
            (pt + pb) / 2.0,
            text="Latitude (°)",
            fill=_MAP_AXIS_LABEL,
            font=_MAP_FONT_AXIS,
            anchor="center",
            angle=90,
        )

        c.create_text(
            pr - 4,
            pt + 4,
            text="N",
            anchor="ne",
            fill="#e2e8f0",
            font=("Segoe UI", 11, "bold"),
        )

        if "target" in self._map_points and _is_meaningful_target_latlon(
            self._map_points["target"][0], self._map_points["target"][1]
        ):
            _ref_lbl = "target"
        elif "start" in self._map_points:
            _ref_lbl = "start"
        else:
            _ref_lbl = "centroid"
        scale_txt = (
            f"ref {_ref_lbl}  E±{half_e:.0f}m  N±{half_n:.0f}m"
            f"  scale×{self._map_user_scale:.2f}"
            f"  (max ±{int(_MAP_REF_MAX_HALF_M / 1000)}km)"
        )
        c.create_text(pl + 4, pt + 4, text=scale_txt, anchor="nw", fill=_MAP_AXIS_LABEL, font=_MAP_FONT_SMALL)

        if len(self._track_points) >= 2:
            pts: list[float] = []
            trail = _decimate_trail(self._track_points, _MAP_TRACK_DRAW_MAX)
            prev_xy: tuple[float, float] | None = None
            for lat, lon in trail:
                x, y = project(lat, lon)
                if prev_xy is not None:
                    if abs(x - prev_xy[0]) < 0.35 and abs(y - prev_xy[1]) < 0.35:
                        continue
                pts.extend([x, y])
                prev_xy = (x, y)
            if len(pts) >= 4:
                c.create_line(
                    *pts,
                    fill="#38bdf8",
                    width=3,
                    smooth=False,
                    capstyle=tk.ROUND,
                    joinstyle=tk.ROUND,
                )

        colors = {
            "start": "#34d399",
            "target": "#f87171",
            "carrot": "#fbbf24",
            "current": "#38bdf8",
        }
        for name in ("start", "target", "carrot", "current"):
            if name not in self._map_points:
                continue
            lat_p, lon_p = self._map_points[name]
            x, y = project(lat_p, lon_p)
            r = 7 if name == "current" else 5
            c.create_oval(
                x - r, y - r, x + r, y + r,
                fill=colors[name], outline="#f8fafc", width=2,
            )
            label = f"{name}\n{lat_p:.{dec}f}°, {lon_p:.{dec}f}°"
            c.create_text(
                x + r + 6, y - 10, text=label, fill="#f1f5f9",
                font=_MAP_FONT_LEGEND, anchor="nw",
            )

        if "current" in self._map_points:
            cx, cy = project(*self._map_points["current"])
            for deg, color in (
                (self._current_heading_deg, "#38bdf8"),
                (self._desired_heading_deg, "#fbbf24"),
            ):
                if not math.isfinite(deg):
                    continue
                rad = math.radians(deg)
                dx = math.sin(rad) * 36.0
                dy = -math.cos(rad) * 36.0
                c.create_line(cx, cy, cx + dx, cy + dy, fill=color, width=3, arrow=tk.LAST)
            c.create_text(
                cx, cy + 22,
                text=(
                    f"ψ {self._current_heading_deg:.0f}°"
                    if math.isfinite(self._current_heading_deg)
                    else "ψ —"
                ),
                fill="#bae6fd",
                font=_MAP_FONT_SMALL,
                anchor="n",
            )

    # ------------------------------------------------------ rx pipeline ---
    def _drain_rx(self) -> None:
        n = 0
        last_tlm: dict[str, str] | None = None
        last_tlm_line: str | None = None
        last_host_ts = ""
        tlm_in_tick = 0
        try:
            while n < _RX_MAX_LINES_PER_TICK:
                line = self._rx_queue.get_nowait()
                n += 1
                host_ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                parsed = self._parse_tlm(line)
                if parsed is not None:
                    self._packet_count += 1
                    self._last_packet_ts = time.time()
                    self._ingest_tlm_track(parsed)
                    self._write_csv_row(parsed, line)
                    tlm_in_tick += 1
                    last_tlm = parsed
                    last_tlm_line = line
                    last_host_ts = host_ts
                else:
                    self._bad_packet_count += 1
                    tag = "warn"
                    if line.startswith("[serial-error]"):
                        tag = "err"
                    self._append_console(f"{host_ts}  {line}", tag)
        except queue.Empty:
            pass

        if last_tlm is not None:
            self._apply_telemetry_ui(last_tlm)
            if tlm_in_tick > 1:
                self._append_console(f"{last_host_ts}  (×{tlm_in_tick}) {last_tlm_line}", "ok")
            else:
                self._append_console(f"{last_host_ts}  {last_tlm_line}", "ok")

        delay = _RX_POLL_BACKLOG_MS if n >= _RX_MAX_LINES_PER_TICK else _RX_POLL_IDLE_MS
        self.after(delay, self._drain_rx)

    def _parse_tlm(self, line: str) -> dict[str, str] | None:
        if not line.startswith(f"${TEAM_ID},"):
            return None
        body = line[1:]
        parts = body.split(",")
        if len(parts) < LEGACY_TLM_FIELDS:
            return None
        if len(parts) < len(TLM_FIELDS):
            parts.extend([""] * (len(TLM_FIELDS) - len(parts)))
        out = {key: parts[i].strip() for i, key in enumerate(TLM_FIELDS)}
        out["team_id"] = TEAM_ID
        return out

    def _apply_to_ui(self, parsed: dict[str, str]) -> None:
        for key, var in self._tlm_vars.items():
            if key in ("gps_lat", "gps_lon"):
                continue
            val = parsed.get(key, "")
            var.set(val if val != "" else "—")
        la_s = parsed.get("gps_lat", "").strip()
        lo_s = parsed.get("gps_lon", "").strip()
        gla = self._parse_optional_float(la_s)
        glo = self._parse_optional_float(lo_s)
        if gla is not None and glo is not None and _valid_gps_latlon(gla, glo):
            self._tlm_vars["gps_lat"].set(la_s)
            self._tlm_vars["gps_lon"].set(lo_s)
        elif la_s == "" and lo_s == "":
            pass
        # Malformed / out-of-range GPS: keep previous display to avoid one-frame garbage.

    def _write_csv_row(self, parsed: dict[str, str], raw: str) -> None:
        if self._csv_writer is None:
            return
        try:
            row = [datetime.now().isoformat(timespec="milliseconds")]
            row.extend(parsed.get(key, "") for key in TLM_FIELDS)
            row.append(raw)
            self._csv_writer.writerow(row)
            assert self._csv_file is not None
            if self._csv_flush_after_id is None:
                self._csv_flush_after_id = self.after(200, self._debounced_csv_flush)
        except Exception as exc:
            self._append_console(f"[csv-error] {exc}", "err")

    def _debounced_csv_flush(self) -> None:
        self._csv_flush_after_id = None
        if self._csv_file is None:
            return
        try:
            self._csv_file.flush()
        except Exception as exc:
            self._append_console(f"[csv-error] flush {exc}", "err")

    # ------------------------------------------------------ console UI ---
    def _append_console(self, text: str, tag: str = "") -> None:
        self._console.insert(tk.END, text + "\n", tag)
        if int(self._console.index("end-1c").split(".")[0]) > 4000:
            self._console.delete("1.0", "1500.0")
        if self._autoscroll_var.get():
            if self._console_scroll_after_id is None:
                self._console_scroll_after_id = self.after(40, self._debounced_console_scroll)

    def _debounced_console_scroll(self) -> None:
        self._console_scroll_after_id = None
        try:
            self._console.see(tk.END)
        except tk.TclError:
            pass

    def _clear_console(self) -> None:
        self._console.delete("1.0", tk.END)

    # ------------------------------------------------------- status bar ---
    def _update_status(self) -> None:
        if self._ser is not None and self._last_packet_ts is not None:
            age = time.time() - self._last_packet_ts
            self._rate_var.set(
                f"rx: {self._packet_count} / err: {self._bad_packet_count} / "
                f"last: {age:.1f}s ago"
            )
        else:
            self._rate_var.set(
                f"rx: {self._packet_count} / err: {self._bad_packet_count}"
            )
        self.after(500, self._update_status)

    def destroy(self) -> None:  # type: ignore[override]
        if self._map_redraw_after_id is not None:
            try:
                self.after_cancel(self._map_redraw_after_id)
            except tk.TclError:
                pass
            self._map_redraw_after_id = None
        if self._console_scroll_after_id is not None:
            try:
                self.after_cancel(self._console_scroll_after_id)
            except tk.TclError:
                pass
            self._console_scroll_after_id = None
        self._disconnect()
        super().destroy()


def main() -> None:
    app = GroundStation()
    app.mainloop()


if __name__ == "__main__":
    main()
