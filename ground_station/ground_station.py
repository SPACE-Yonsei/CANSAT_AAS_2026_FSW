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


TEAM_ID = "1070"

TLM_FIELDS = [
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

LEGACY_TLM_FIELDS = 30

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
        self._csv_writer: csv.writer | None = None
        self._packet_count = 0
        self._bad_packet_count = 0
        self._last_packet_ts: float | None = None
        self._tlm_vars: dict[str, tk.StringVar] = {}
        self._track_points: list[tuple[float, float]] = []
        self._map_points: dict[str, tuple[float, float]] = {}
        self._current_heading_deg = math.nan
        self._desired_heading_deg = math.nan
        self._left_pulse_us = 0
        self._right_pulse_us = 0
        # Avoid blocking the Tk mainloop: CSV flush / map redraw / console scroll are debounced.
        self._csv_flush_after_id: str | None = None
        self._map_redraw_after_id: str | None = None
        self._map_dirty: bool = False
        self._console_scroll_after_id: str | None = None

        self._build_ui()
        self._refresh_ports()
        self.after(20, self._drain_rx)
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
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=1)
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
        self._baud_var = tk.StringVar(value="9600")
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
                ("distance_cm", "Distance (cm)"),
                ("cmd_echo", "Cmd echo"),
            ]),
            ("Filtered (deg)", [
                ("filtered_roll", "roll"),
                ("filtered_pitch", "pitch"),
                ("filtered_yaw", "yaw"),
            ]),
        ]

        for col in range(2):
            wrap.columnconfigure(col, weight=1)

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
                ttk.Label(box, textvariable=var, style="Stat.TLabel").grid(
                    row=i, column=1, sticky="e", padx=6, pady=2
                )

        for r in range((len(groups) + 1) // 2):
            wrap.rowconfigure(r, weight=1)

    def _build_map_and_motor(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Guidance Map / Motor")
        frame.grid(row=0, column=0, sticky="nsew")
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)

        self._map_canvas = tk.Canvas(
            frame,
            height=300,
            background=_MAP_BG,
            highlightthickness=1,
            highlightbackground="#2d3748",
        )
        self._map_canvas.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)
        self._map_canvas.bind("<Configure>", lambda _e: self._request_map_redraw())

        bars = ttk.Frame(frame)
        bars.grid(row=1, column=0, sticky="ew", padx=6, pady=(0, 6))
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

        self._heading_var = tk.StringVar(value="heading: -- / target: --")
        ttk.Label(bars, textvariable=self._heading_var).grid(row=0, column=3, rowspan=2, sticky="w")
        self._guidance_var = tk.StringVar(value="guidance: --")
        ttk.Label(bars, textvariable=self._guidance_var).grid(row=2, column=0, columnspan=4, sticky="w", pady=(4, 0))

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

    def _build_status_bar(self) -> None:
        bar = ttk.Frame(self)
        bar.pack(fill=tk.X, padx=8, pady=(0, 6))
        self._status_var = tk.StringVar(value="Disconnected")
        ttk.Label(bar, textvariable=self._status_var).pack(side=tk.LEFT)
        self._rate_var = tk.StringVar(value="rx: 0 / err: 0 / rate: -- Hz")
        ttk.Label(bar, textvariable=self._rate_var).pack(side=tk.RIGHT)

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
        line = f"CMD,{TEAM_ID},{body}\n"
        try:
            self._ser.write(line.encode("utf-8"))
            self._append_console(f"[TX] {line.strip()}", "tx")
        except Exception as exc:
            self._append_console(f"[TX-error] {exc}", "err")

    def _parse_optional_float(self, value: str) -> float | None:
        try:
            v = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(v):
            return None
        return v

    def _update_map_and_motor(self, parsed: dict[str, str]) -> None:
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

        if cur_lat is not None and cur_lon is not None:
            self._track_points.append((cur_lat, cur_lon))
            if len(self._track_points) > 500:
                self._track_points = self._track_points[-500:]

        self._map_points = {}
        if start_lat is not None and start_lon is not None:
            self._map_points["start"] = (start_lat, start_lon)
        if target_lat is not None and target_lon is not None:
            self._map_points["target"] = (target_lat, target_lon)
        if carrot_lat is not None and carrot_lon is not None:
            self._map_points["carrot"] = (carrot_lat, carrot_lon)
        if cur_lat is not None and cur_lon is not None:
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
        self._heading_var.set(f"heading: {ch} / target: {dh}")
        gstate = parsed.get("guidance_state", "").strip() or "--"
        self._guidance_var.set(f"guidance: {gstate}")
        self._request_map_redraw()

    def _request_map_redraw(self) -> None:
        self._map_dirty = True
        if self._map_redraw_after_id is not None:
            return
        self._map_redraw_after_id = self.after(100, self._flush_map_redraw)

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
        d_lat = max(1e-8, lat_max - lat_min)
        d_lon = max(1e-8, lon_max - lon_min)
        lat_center = (lat_min + lat_max) / 2.0
        lon_center = (lon_min + lon_max) / 2.0
        lat_mid = lat_center
        meter_per_lon = 111320.0 * math.cos(math.radians(lat_mid))
        meter_per_lon = meter_per_lon if abs(meter_per_lon) > 1e-6 else 1.0
        width_m = d_lon * meter_per_lon
        height_m = d_lat * 111320.0
        span = max(width_m, height_m, 5.0)
        dec = _map_geo_decimals(span)

        pl = float(margin_l)
        pr = float(w - margin_r)
        pt = float(margin_t)
        pb = float(h - margin_b)
        pw = max(20.0, pr - pl)
        ph = max(20.0, pb - pt)

        def project(lat: float, lon: float) -> tuple[float, float]:
            east = (lon - lon_center) * meter_per_lon
            north = (lat - lat_center) * 111320.0
            x = pl + pw / 2.0 + (east / span) * pw
            y = pt + ph / 2.0 - (north / span) * ph
            return x, y

        def inv_lon(px: float) -> float:
            east = ((px - pl - pw / 2.0) / pw) * span
            return lon_center + east / meter_per_lon

        def inv_lat(py: float) -> float:
            north = ((pt + ph / 2.0 - py) / ph) * span
            return lat_center + north / 111320.0

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

        scale_txt = f"span ≈ {span:.0f} m"
        c.create_text(pl + 4, pt + 4, text=scale_txt, anchor="nw", fill=_MAP_AXIS_LABEL, font=_MAP_FONT_SMALL)

        if len(self._track_points) >= 2:
            pts: list[float] = []
            for lat, lon in self._track_points:
                x, y = project(lat, lon)
                pts.extend([x, y])
            c.create_line(*pts, fill="#38bdf8", width=3, smooth=False)

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
        try:
            while True:
                line = self._rx_queue.get_nowait()
                self._handle_line(line)
        except queue.Empty:
            pass
        finally:
            self.after(20, self._drain_rx)

    def _handle_line(self, line: str) -> None:
        host_ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        parsed = self._parse_tlm(line)
        if parsed is not None:
            self._packet_count += 1
            self._last_packet_ts = time.time()
            self._apply_to_ui(parsed)
            self._update_map_and_motor(parsed)
            self._append_console(f"{host_ts}  {line}", "ok")
            self._write_csv_row(parsed, line)
        else:
            self._bad_packet_count += 1
            tag = "warn"
            if line.startswith("[serial-error]"):
                tag = "err"
            self._append_console(f"{host_ts}  {line}", tag)

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
            val = parsed.get(key, "")
            var.set(val if val != "" else "—")

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
