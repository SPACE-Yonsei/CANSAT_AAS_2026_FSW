"""XBee diagnostic / fixer for the Pi-side module.

Most common cause when XCTU shows two XBees as paired but the ground
station never receives FSW telemetry: the Pi-side XBee is in **API mode**
(AP != 0) or has a different serial baud (BD != 3 = 9600), so the raw
ASCII bytes that FSW writes to `/dev/serial0` get discarded as malformed
API frames.

This script must run while `main.py` is **NOT** running (only one process
can hold the UART device at a time). On some boards `/dev/serial0` does not
exist; use `UART_DEVICE=/dev/ttyAMA0` or pass `--port /dev/ttyAMA0`.

It auto-detects whether the local XBee is in transparent mode (uses
`+++/AT...`) or API mode (uses 0x08 AT Command frames, with or without
escape).

Examples:

    python3 tools/xbee_diag.py                     # read settings
    python3 tools/xbee_diag.py --fix-transparent   # AP=0, BD=3, save
    python3 tools/xbee_diag.py --send-test --duration 30
    python3 tools/xbee_diag.py --reset --send-test

While `--send-test` runs, watch the ground station
(`ground_station/ground_station.py` on the laptop, connected to the other
XBee via USB Explorer) for `$LINKTEST,...` lines.
"""

from __future__ import annotations

import argparse
import glob
import os
import sys
import time
from typing import Optional, Tuple

try:
    import serial
except Exception:
    print("pyserial required: pip install pyserial", file=sys.stderr)
    sys.exit(1)


READ_KEYS = [
    ("AP", "API mode (0=transparent, 1/2=API)"),
    ("BD", "Serial baud index (3=9600, 7=115200)"),
    ("ID", "PAN ID"),
    ("CH", "RF channel (S2/S2C)"),
    ("DH", "Destination addr high"),
    ("DL", "Destination addr low (FFFF=broadcast)"),
    ("SH", "Source addr high"),
    ("SL", "Source addr low"),
    ("MY", "16-bit address"),
    ("D6", "DIO6/RTS function"),
    ("D7", "DIO7/CTS function"),
    ("VR", "Firmware version"),
    ("HV", "Hardware version"),
]

ESC_BYTES = {0x7E, 0x7D, 0x11, 0x13}


# --------------------------------------------------------------- common ---
def reset_pulse() -> None:
    pin = int(os.environ.get("XBEE_RESET_GPIO", "18"))
    try:
        import pigpio  # type: ignore
    except Exception:
        print("[reset] pigpio unavailable, skipping hardware reset")
        return
    pi = pigpio.pi()
    if not getattr(pi, "connected", False):
        print("[reset] pigpio daemon not running (sudo systemctl start pigpiod)")
        return
    try:
        pi.set_mode(pin, pigpio.INPUT)
        pi.set_mode(pin, pigpio.OUTPUT)
        pi.write(pin, 0)
        time.sleep(0.15)
        pi.set_mode(pin, pigpio.INPUT)
        print(f"[reset] pulsed GPIO{pin} low for 150 ms")
    finally:
        pi.stop()


def open_port(port: str, baud: int) -> serial.Serial:
    return serial.Serial(port, baud, timeout=1.0, write_timeout=1.0)


def uart_port_candidates(cli_port: str) -> list[str]:
    """Build an ordered list of device paths to try (CLI, env, common Pi names)."""
    out: list[str] = []
    for p in (cli_port or "").split(","):
        p = p.strip()
        if p and p not in out:
            out.append(p)
    env = os.environ.get("UART_DEVICE", "").strip()
    for part in env.split(","):
        part = part.strip()
        if part and part not in out:
            out.append(part)
    for fb in (
        "/dev/serial0",
        "/dev/serial1",
        "/dev/ttyAMA0",
        "/dev/ttyS0",
        "/dev/ttyAMA10",
    ):
        if fb not in out:
            out.append(fb)
    return out


def open_first_uart(candidates: list[str], baud: int) -> tuple[serial.Serial, str]:
    tried: list[str] = []
    for cand in candidates:
        if not os.path.exists(cand):
            tried.append(f"{cand} (missing)")
            continue
        try:
            ser = open_port(cand, baud)
            print(f"[open] {cand} @ {baud} baud")
            return ser, cand
        except Exception as exc:
            tried.append(f"{cand} ({exc})")
    detail = "\n  ".join(tried) if tried else "(no candidates)"
    raise OSError(
        "No UART port could be opened. Tried:\n  "
        + detail
        + "\n\nHints:\n"
        + "  - dietpi-config: enable primary UART; reboot if needed.\n"
        + "  - /boot/config.txt: ensure enable_uart=1 (Pi 3+: dtoverlay=disable-bt if conflicts).\n"
        + "  - export UART_DEVICE=/dev/ttyAMA0   # or the path that exists on your board\n"
        + "  - python3 tools/xbee_diag.py --list-ports"
    )


# ----------------------------------------------------- transparent mode ---
def at_enter_command_mode(ser: serial.Serial) -> bool:
    ser.reset_input_buffer()
    time.sleep(1.1)
    ser.write(b"+++")
    ser.flush()
    deadline = time.time() + 1.5
    buf = b""
    while time.time() < deadline:
        chunk = ser.read(64)
        if chunk:
            buf += chunk
            if b"OK" in buf:
                return True
    return b"OK" in buf


def at_cmd(ser: serial.Serial, cmd: str, timeout: float = 1.0) -> str:
    ser.reset_input_buffer()
    ser.write(cmd.encode("ascii") + b"\r")
    ser.flush()
    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        chunk = ser.read(64)
        if chunk:
            buf += chunk
            if buf.endswith(b"\r"):
                break
    return buf.decode("ascii", errors="ignore").rstrip("\r\n").strip()


# ------------------------------------------------------------- API mode ---
def _esc(b: int, escape: bool) -> bytes:
    if escape and b in ESC_BYTES:
        return bytes([0x7D, b ^ 0x20])
    return bytes([b])


def _build_api_at(cmd: str, value: bytes, escape: bool, frame_id: int = 0x42) -> bytes:
    body = bytes([0x08, frame_id]) + cmd.encode("ascii") + value
    length = len(body)
    cks = (0xFF - (sum(body) & 0xFF)) & 0xFF
    out = bytearray(b"\x7E")
    for b in length.to_bytes(2, "big"):
        out.extend(_esc(b, escape))
    for b in body:
        out.extend(_esc(b, escape))
    out.extend(_esc(cks, escape))
    return bytes(out)


def _unescape_after_start(buf: bytes) -> bytes:
    if not buf or buf[0] != 0x7E:
        return buf
    out = bytearray([0x7E])
    i = 1
    while i < len(buf):
        if buf[i] == 0x7D and i + 1 < len(buf):
            out.append(buf[i + 1] ^ 0x20)
            i += 2
        else:
            out.append(buf[i])
            i += 1
    return bytes(out)


def _read_api_response(
    ser: serial.Serial, escape: bool, timeout: float
) -> Optional[Tuple[str, int, bytes]]:
    deadline = time.time() + timeout
    raw = bytearray()
    while time.time() < deadline:
        chunk = ser.read(128)
        if chunk:
            raw.extend(chunk)
        idx = raw.find(0x7E)
        if idx < 0:
            continue
        if idx > 0:
            del raw[:idx]
        if escape:
            unesc = bytearray(_unescape_after_start(bytes(raw)))
            if len(unesc) < 4:
                continue
            llen = int.from_bytes(bytes(unesc[1:3]), "big")
            need = 3 + llen + 1
            if len(unesc) < need:
                continue
            body = bytes(unesc[3:3 + llen])
            cks_recv = unesc[3 + llen]
        else:
            if len(raw) < 4:
                continue
            llen = int.from_bytes(bytes(raw[1:3]), "big")
            need = 3 + llen + 1
            if len(raw) < need:
                continue
            body = bytes(raw[3:3 + llen])
            cks_recv = raw[3 + llen]
        cks_calc = (0xFF - (sum(body) & 0xFF)) & 0xFF
        if cks_calc != cks_recv or len(body) < 5 or body[0] != 0x88:
            del raw[0:1]
            continue
        return (body[2:4].decode("ascii", errors="ignore"), body[4], body[5:])
    return None


def api_at(
    ser: serial.Serial, cmd: str, value: bytes = b"", escape: bool = False
) -> Optional[Tuple[str, int, bytes]]:
    ser.reset_input_buffer()
    ser.write(_build_api_at(cmd, value, escape))
    ser.flush()
    return _read_api_response(ser, escape, timeout=1.5)


def detect_api_escape(ser: serial.Serial) -> Optional[bool]:
    """Try AP=1 (no escape) first, then AP=2 (escape). Return escape flag or None."""
    r = api_at(ser, "AP", escape=False)
    if r is not None and r[0] == "AP":
        return False
    r = api_at(ser, "AP", escape=True)
    if r is not None and r[0] == "AP":
        return True
    return None


# ----------------------------------------------------------- read modes ---
def _hexify(value: bytes) -> str:
    if not value:
        return ""
    s = value.hex().upper().lstrip("0")
    return s or "0"


def read_via_at(ser: serial.Serial) -> Optional[dict[str, str]]:
    if not at_enter_command_mode(ser):
        return None
    out: dict[str, str] = {}
    for key, _ in READ_KEYS:
        out[key] = at_cmd(ser, f"AT{key}")
    at_cmd(ser, "ATCN")
    return out


def read_via_api(ser: serial.Serial, escape: bool) -> Optional[dict[str, str]]:
    out: dict[str, str] = {}
    for key, _ in READ_KEYS:
        r = api_at(ser, key, escape=escape)
        if r is None:
            return None
        cmd, status, value = r
        if cmd != key or status != 0:
            out[key] = f"<status {status}>"
        else:
            out[key] = _hexify(value)
    return out


def cmd_read(ser: serial.Serial) -> int:
    print("[probe] trying transparent AT mode (+++)...")
    values = read_via_at(ser)
    mode = "transparent"
    escape: Optional[bool] = None
    if values is None:
        print("[probe] '+++' got no response, falling back to API mode probe...")
        escape = detect_api_escape(ser)
        if escape is None:
            print(
                "\n[ERR] Neither '+++' nor API frames got a response. Likely causes:\n"
                "  1) XBee not powered: check 3.3V on pin 1, GND on pin 10.\n"
                "  2) DOUT/DIN reversed: XBee DOUT (pin 2) must reach Pi RX (GPIO15, pin 10).\n"
                "  3) Pi UART busy: stop main.py, then disable kernel console / login:\n"
                "       systemctl is-active serial-getty@ttyAMA0.service\n"
                "       sudo systemctl disable --now serial-getty@ttyAMA0.service\n"
                "       sudo systemctl disable --now serial-getty@ttyS0.service\n"
                "  4) Wrong baud: rerun with --baud 115200 / 19200 / 38400.\n"
            )
            return 1
        mode = f"API (AP={2 if escape else 1})"
        values = read_via_api(ser, escape=escape)
        if values is None:
            print("[ERR] API probe started but reading register set failed.")
            return 1

    print(f"== Detected mode: {mode} ==")
    for key, desc in READ_KEYS:
        val = values.get(key, "")
        print(f"  AT{key:<2} = {val:<16}  ({desc})")

    print()
    print("== Diagnosis ==")
    issues = 0
    ap_val = values.get("AP", "").lstrip("0") or "0"
    if ap_val != "0":
        print(
            f"  [X] AP = {values.get('AP')} -> must be 0 for transparent FSW telemetry.\n"
            f"      Fix: python3 tools/xbee_diag.py --fix-transparent"
        )
        issues += 1
    bd_val = values.get("BD", "").lstrip("0") or "0"
    if bd_val != "3":
        print(
            f"  [!] BD = {values.get('BD')} -> FSW UART_BAUD default is 9600 (BD=3).\n"
            f"      Either set BD=3 here or export UART_BAUD=<matching speed> for FSW."
        )
        issues += 1
    if issues == 0:
        print("  [OK] AP/BD look correct on the Pi-side module.")
        print("       If the link still fails, verify on the LAPTOP module that:")
        print("         - AP = 0 (transparent), BD matches Pi (BD=3 -> 9600)")
        print("         - ID and CH match the Pi module")
        print("         - DH/DL = 0/FFFF (broadcast) or match the other module's SH/SL")
    return 0


# ------------------------------------------------------------ fix mode ---
def fix_via_at(ser: serial.Serial) -> bool:
    if not at_enter_command_mode(ser):
        return False
    seq = [
        ("ATAP0", "transparent mode"),
        ("ATBD3", "9600 baud"),
        ("ATD60", "RTS off"),
        ("ATD70", "CTS off"),
        ("ATDH0", "destination high = 0"),
        ("ATDLFFFF", "destination low = FFFF (broadcast)"),
        ("ATWR", "write to non-volatile memory"),
        ("ATAC", "apply changes"),
    ]
    for cmd, desc in seq:
        resp = at_cmd(ser, cmd)
        print(f"  {cmd:<10} -> {resp:<6}  ({desc})")
    at_cmd(ser, "ATCN")
    return True


def fix_via_api(ser: serial.Serial, escape: bool) -> bool:
    seq: list[tuple[str, bytes, str]] = [
        ("AP", b"\x00",       "transparent mode"),
        ("BD", b"\x03",       "9600 baud"),
        ("D6", b"\x00",       "RTS off"),
        ("D7", b"\x00",       "CTS off"),
        ("DH", b"\x00",       "destination high = 0"),
        ("DL", b"\xFF\xFF",   "destination low = FFFF (broadcast)"),
        ("WR", b"",           "write to non-volatile memory"),
        ("AC", b"",           "apply changes"),
    ]
    ok_all = True
    for cmd, value, desc in seq:
        r = api_at(ser, cmd, value=value, escape=escape)
        if r is None:
            print(f"  AT{cmd:<2}            -> NO RESP   ({desc})")
            ok_all = False
            continue
        _, status, _ = r
        tag = "OK" if status == 0 else f"status={status}"
        print(f"  AT{cmd:<2} <- {value.hex().upper() or '-':<6} -> {tag:<8}  ({desc})")
        if status != 0:
            ok_all = False
    return ok_all


def cmd_fix_transparent(ser: serial.Serial) -> int:
    print("[fix] trying transparent AT mode first...")
    if fix_via_at(ser):
        print("Done via AT mode. Power-cycle the XBee, then re-run xbee_diag.py to confirm.")
        return 0
    print("[fix] transparent AT mode failed, trying API mode...")
    escape = detect_api_escape(ser)
    if escape is None:
        print(
            "[ERR] Could not reach the XBee in either mode. "
            "Check power, wiring, and that no other process is using the UART."
        )
        return 1
    print(f"[fix] using API mode (escape={'AP=2' if escape else 'AP=1'})")
    if not fix_via_api(ser, escape=escape):
        print("[ERR] One or more API writes failed. Re-run --fix-transparent or use XCTU.")
        return 1
    print("Done via API mode. Power-cycle the XBee, then re-run xbee_diag.py to confirm.")
    return 0


# ----------------------------------------------------------- loopback ---
def cmd_loopback(ser: serial.Serial, duration: float) -> int:
    """Write a token and read it back. Use with TX↔RX shorted (XBee removed).

    Wire Pi physical pin 8 (GPIO14, TX0) directly to pin 10 (GPIO15, RX0).
    If this still receives nothing, the Pi UART itself is misconfigured (most
    often `serial-getty@ttyAMA0` running, kernel console on serial0, or
    `enable_uart=1` missing in /boot/config.txt).
    """
    print(f"== Loopback test on {ser.port} @ {ser.baudrate} for {duration:.1f}s ==")
    print(
        "Short Pi pin 8 (TX) <-> pin 10 (RX) with a jumper wire.\n"
        "If you read back the same lines, the Pi UART works."
    )
    t0 = time.time()
    sent = 0
    rx_lines = 0
    buf = bytearray()
    try:
        while time.time() - t0 < duration:
            line = f"PI-LOOPBACK,{sent}\n".encode("ascii")
            ser.write(line)
            ser.flush()
            print("TX", line.decode().rstrip())
            sent += 1
            t_send = time.time()
            while time.time() - t_send < 0.5:
                chunk = ser.read(64)
                if chunk:
                    buf.extend(chunk)
                    while True:
                        nl = buf.find(b"\n")
                        if nl < 0:
                            break
                        got = bytes(buf[:nl]).decode("ascii", errors="ignore").rstrip("\r")
                        del buf[: nl + 1]
                        if got:
                            print("RX", got)
                            rx_lines += 1
    except KeyboardInterrupt:
        print("\n[abort] interrupted by user")
    print(f"Sent {sent} / Received {rx_lines}")
    if rx_lines == 0:
        print(
            "\n[ERR] No bytes echoed back. The Pi UART read path is broken:\n"
            "  - Check `systemctl is-active serial-getty@ttyAMA0` and disable it.\n"
            "  - Check /boot/cmdline.txt does NOT contain console=serial0,*.\n"
            "  - Check /boot/config.txt has enable_uart=1 (and dtoverlay=disable-bt on Pi 3+).\n"
            "  - Try `--port /dev/ttyAMA0` directly.\n"
        )
        return 1
    print("[OK] Loopback succeeded -> Pi UART path is fine.")
    return 0


def cmd_list_ports() -> int:
    print("== Standard Raspberry Pi UART paths ==")
    for path in (
        "/dev/serial0",
        "/dev/serial1",
        "/dev/ttyAMA0",
        "/dev/ttyAMA10",
        "/dev/ttyS0",
        "/dev/ttyUSB0",
    ):
        try:
            real = os.readlink(path) if os.path.islink(path) else "(not a symlink)"
        except OSError:
            real = ""
        exists = os.path.exists(path)
        print(f"  {path:<22} exists={exists}  link->{real}")
    print("\n== Matching /dev/ttyAMA* and /dev/ttyS* (if any) ==")
    for pattern in ("/dev/ttyAMA*", "/dev/ttyS*"):
        for path in sorted(glob.glob(pattern)):
            print(f"  {path}")
    try:
        from serial.tools import list_ports

        print("\n== pyserial list_ports ==")
        for entry in list_ports.comports():
            dev = getattr(entry, "device", "")
            desc = getattr(entry, "description", "")
            print(f"  {dev:<16}  {desc!r}")
    except Exception as exc:
        print(f"\n(pyserial list_ports unavailable: {exc})")
    return 0


# ----------------------------------------------------------- send test ---
def cmd_send_test(ser: serial.Serial, duration: float, period: float) -> int:
    print(f"== Sending $LINKTEST frames for {duration:.1f}s on {ser.port} @ {ser.baudrate} ==")
    print("Watch the laptop ground station (or XCTU console) for matching lines.")
    t0 = time.time()
    i = 0
    try:
        while time.time() - t0 < duration:
            line = f"$LINKTEST,{i},{time.time() - t0:.2f}\n"
            ser.write(line.encode("utf-8"))
            ser.flush()
            print("TX", line.rstrip())
            i += 1
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[abort] interrupted by user")
    print(f"Sent {i} frames")
    return 0


def main() -> None:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument(
        "--port",
        default="",
        help=(
            "UART device path, comma-separated list allowed. "
            "Default: UART_DEVICE env + /dev/serial0, /dev/ttyAMA0, /dev/ttyS0, ..."
        ),
    )
    p.add_argument("--baud", type=int, default=int(os.environ.get("UART_BAUD", "9600")))
    p.add_argument("--reset", action="store_true",
                   help="Pulse XBee /RESET via GPIO18 before opening (uses pigpio)")
    p.add_argument("--fix-transparent", action="store_true",
                   help="Set AP=0, BD=3, DH=0, DL=FFFF, ATWR/ATAC")
    p.add_argument("--send-test", action="store_true",
                   help="Send $LINKTEST frames so the laptop side can verify reception")
    p.add_argument("--loopback", action="store_true",
                   help="Pi-side TX↔RX echo test (jumper Pi pin 8 ↔ pin 10, no XBee)")
    p.add_argument("--list-ports", action="store_true",
                   help="Print known serial device paths and where they resolve")
    p.add_argument("--duration", type=float, default=30.0)
    p.add_argument("--period", type=float, default=0.5)
    args = p.parse_args()

    if args.list_ports:
        sys.exit(cmd_list_ports())

    if args.reset:
        reset_pulse()
        time.sleep(0.5)

    try:
        ser, _ = open_first_uart(uart_port_candidates(args.port), args.baud)
    except OSError as exc:
        print(f"[ERR] {exc}")
        sys.exit(1)

    rc = 0
    try:
        if args.fix_transparent:
            rc = cmd_fix_transparent(ser)
        elif args.send_test:
            rc = cmd_send_test(ser, args.duration, args.period)
        elif args.loopback:
            rc = cmd_loopback(ser, args.duration)
        else:
            rc = cmd_read(ser)
    finally:
        ser.close()
    sys.exit(rc)


if __name__ == "__main__":
    main()
