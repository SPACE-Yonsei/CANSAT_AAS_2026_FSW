"""CANSAT AAS 2026 - Ground Station GUI.

Receives telemetry from XBee (USB serial), parses the 30-field CANSAT
TLM CSV emitted by `comm/commapp.py:send_tlm`, displays it live, logs
to CSV, and sends commands using the `CMD,1070,<CMD>,<option>` format
that `comm/commapp.py:_dispatch_command` accepts.

Run:
    python ground_station/ground_station.py

Standalone, no other repo modules required (only `pyserial` + tkinter).
"""

from __future__ import annotations

import csv
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
]

COMMAND_PRESETS = [
    ("CX,ON",   "Telemetry ON"),
    ("CX,OFF",  "Telemetry OFF"),
    ("ST,GPS",  "Sync time to GPS"),
    ("SIM,ENABLE",   "SIM mode enable"),
    ("SIM,ACTIVATE", "SIM mode activate"),
    ("SIM,DISABLE",  "SIM mode disable"),
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

        self._build_ui()
        self._refresh_ports()
        self.after(50, self._drain_rx)
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

        self._build_telemetry_panel(left)
        self._build_console_and_command(right)
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

    def _build_console_and_command(self, parent: ttk.Frame) -> None:
        parent.rowconfigure(0, weight=1)
        parent.rowconfigure(1, weight=0)
        parent.columnconfigure(0, weight=1)

        console_box = ttk.LabelFrame(parent, text="Raw RX (XBee)")
        console_box.grid(row=0, column=0, sticky="nsew")
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
        cmd_box.grid(row=1, column=0, sticky="ew", pady=(8, 0))
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
        if self._csv_file is not None:
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

    # ------------------------------------------------------ rx pipeline ---
    def _drain_rx(self) -> None:
        try:
            while True:
                line = self._rx_queue.get_nowait()
                self._handle_line(line)
        except queue.Empty:
            pass
        finally:
            self.after(50, self._drain_rx)

    def _handle_line(self, line: str) -> None:
        host_ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        parsed = self._parse_tlm(line)
        if parsed is not None:
            self._packet_count += 1
            self._last_packet_ts = time.time()
            self._apply_to_ui(parsed)
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
        if len(parts) < len(TLM_FIELDS):
            return None
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
            self._csv_file.flush()
        except Exception as exc:
            self._append_console(f"[csv-error] {exc}", "err")

    # ------------------------------------------------------ console UI ---
    def _append_console(self, text: str, tag: str = "") -> None:
        self._console.insert(tk.END, text + "\n", tag)
        if int(self._console.index("end-1c").split(".")[0]) > 4000:
            self._console.delete("1.0", "1500.0")
        if self._autoscroll_var.get():
            self._console.see(tk.END)

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
        self._disconnect()
        super().destroy()


def main() -> None:
    app = GroundStation()
    app.mainloop()


if __name__ == "__main__":
    main()
