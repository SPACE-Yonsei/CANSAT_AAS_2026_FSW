"""XBee diagnostic / fixer for the Pi-side module.

Use this when XCTU shows two XBees as paired but the ground station never
receives FSW telemetry. The most common cause is that the **Pi-side XBee**
is in API mode (AP != 0) or has a different serial baud (BD != 3 = 9600),
so the raw `$TEAMID,...` ASCII bytes that FSW writes to `/dev/serial0`
get discarded as malformed API frames.

This script must run while `main.py` is **NOT** running (only one process
can hold `/dev/serial0` at a time).

Examples:

    sudo python3 tools/xbee_diag.py                    # read settings
    sudo python3 tools/xbee_diag.py --fix-transparent  # AP=0, BD=3, save
    sudo python3 tools/xbee_diag.py --send-test --duration 30
    sudo python3 tools/xbee_diag.py --reset --send-test

While `--send-test` runs, watch the ground station (`ground_station/ground_station.py`
on the laptop, connected to the **other** XBee via USB Explorer) for `$LINKTEST,...`
lines. If you see them, the RF/serial path is healthy and the original FSW
output should also work.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

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


def enter_command_mode(ser: serial.Serial) -> bool:
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


def at(ser: serial.Serial, cmd: str, timeout: float = 1.0) -> str:
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


def cmd_read(ser: serial.Serial) -> int:
    if not enter_command_mode(ser):
        print(
            "[ERR] No response to '+++'. Possible causes:\n"
            "  - XBee not powered (check 3.3V on pin 1, GND on pin 10)\n"
            "  - DIN/DOUT wired backwards (XBee DOUT must reach Pi RX = GPIO15)\n"
            "  - Pi UART is busy (stop main.py, disable serial-getty@ttyAMA0)\n"
            "  - Wrong serial baud — try --baud 115200 / 19200 / 38400"
        )
        return 1

    print("== AT command mode entered ==")
    bad = []
    values: dict[str, str] = {}
    for key, desc in READ_KEYS:
        val = at(ser, f"AT{key}")
        values[key] = val
        print(f"  AT{key:<2} = {val:<16}  ({desc})")
    at(ser, "ATCN")

    print()
    print("== Diagnosis ==")
    issues = 0
    if values.get("AP") not in {"0", "00"}:
        print(
            f"  [X] AP = {values.get('AP')} -> must be 0 for transparent mode.\n"
            f"      Run: sudo python3 tools/xbee_diag.py --fix-transparent"
        )
        issues += 1
    if values.get("BD") not in {"3", "03"}:
        print(
            f"  [!] BD = {values.get('BD')} -> FSW UART_BAUD default is 9600 (BD=3).\n"
            f"      Either set BD=3 here or export UART_BAUD=<matching speed> for FSW."
        )
        issues += 1
    if issues == 0:
        print("  [OK] AP/BD look correct on the Pi-side module.")
        print("      If the link still fails, verify on the LAPTOP module that:")
        print("        - AP = 0 (transparent), BD matches Pi (BD=3 -> 9600)")
        print("        - ID and CH match the Pi module")
        print("        - DH/DL are 0/FFFF (broadcast) or match the other module's SH/SL")
    return 0


def cmd_fix_transparent(ser: serial.Serial) -> int:
    if not enter_command_mode(ser):
        print("[ERR] Could not enter AT command mode (see read mode help).")
        return 1
    print("== Configuring transparent mode (AP=0), 9600 baud (BD=3), broadcast DH/DL ==")
    seq = [
        ("ATAP0", "transparent mode"),
        ("ATBD3", "9600 baud"),
        ("ATD60", "RTS off"),
        ("ATD70", "CTS off (so flow control wiring won't block traffic)"),
        ("ATDH0", "destination high = 0"),
        ("ATDLFFFF", "destination low = FFFF (broadcast)"),
        ("ATWR", "write to non-volatile memory"),
        ("ATAC", "apply changes"),
    ]
    for cmd, desc in seq:
        resp = at(ser, cmd)
        print(f"  {cmd:<10} -> {resp:<6}  ({desc})")
    at(ser, "ATCN")
    print("Done. Power-cycle the XBee, then re-run: sudo python3 tools/xbee_diag.py")
    return 0


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
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=os.environ.get("UART_DEVICE", "/dev/serial0"))
    p.add_argument("--baud", type=int, default=int(os.environ.get("UART_BAUD", "9600")))
    p.add_argument("--reset", action="store_true",
                   help="Pulse XBee /RESET via GPIO18 before opening (uses pigpio)")
    p.add_argument("--fix-transparent", action="store_true",
                   help="Set AP=0, BD=3, DH=0, DL=FFFF, ATWR/ATAC")
    p.add_argument("--send-test", action="store_true",
                   help="Send $LINKTEST frames so the laptop side can verify reception")
    p.add_argument("--duration", type=float, default=30.0)
    p.add_argument("--period", type=float, default=0.5)
    args = p.parse_args()

    if args.reset:
        reset_pulse()
        time.sleep(0.5)

    try:
        ser = open_port(args.port, args.baud)
    except Exception as exc:
        print(f"[ERR] Could not open {args.port} @ {args.baud}: {exc}")
        sys.exit(1)

    rc = 0
    try:
        if args.fix_transparent:
            rc = cmd_fix_transparent(ser)
        elif args.send_test:
            rc = cmd_send_test(ser, args.duration, args.period)
        else:
            rc = cmd_read(ser)
    finally:
        ser.close()
    sys.exit(rc)


if __name__ == "__main__":
    main()
