"""Comm app: command parsing + telemetry aggregation/sending."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timedelta
import logging
import math
import os
import re
import threading
import time
from typing import Optional

from lib import appargs, msgstructure, prevstate
from comm import uartserial, xbeereset


logger = logging.getLogger(__name__)

COMMAPP_RUNSTATUS = True
TELEMETRY_ENABLE = True
ST_timedelta = timedelta(seconds=0)

_LOG_TLM_TO_CONSOLE = os.environ.get("FSW_LOG_TLM", "1").strip() != "0"
_TLM_SEND_FAIL_LOGGED = False
_LAST_TLM_FAIL_LOG_TS = 0.0

_RBT_AUTH_TOKEN = os.environ.get("RBT_AUTH_TOKEN", "").strip()
_RBT_REQUIRE_SEQ = os.environ.get("RBT_REQUIRE_SEQ", "1").strip() != "0"
_RBT_SAFE_STATES = {x.strip() for x in os.environ.get("RBT_SAFE_STATES", "0,5").split(",") if x.strip()}
_RBT_LAST_SEQ = -1
_RBT_RECENT_NONCES: list[str] = []
_RBT_NONCE_WINDOW = 32


@dataclass
class TelemetryData:
    mode: str = "F"
    state: str = "0"
    altitude: float = 0.0
    temperature: float = 0.0
    pressure: float = 0.0
    voltage: float = 0.0
    current: float = 0.0
    power: float = 0.0
    distance: float = 0.0
    gyro_roll: float = 0.0
    gyro_pitch: float = 0.0
    gyro_yaw: float = 0.0
    acc_x: float = 0.0
    acc_y: float = 0.0
    acc_z: float = 0.0
    mag_roll: float = 0.0
    mag_pitch: float = 0.0
    mag_yaw: float = 0.0
    gps_time: str = ""
    gps_alt: float = 0.0
    gps_lat: float = 0.0
    gps_lon: float = 0.0
    gps_sats: int = 0
    cmd_echo: str = ""
    filtered_roll: float = 0.0
    filtered_pitch: float = 0.0
    filtered_yaw: float = 0.0
    start_lat: float = float("nan")
    start_lon: float = float("nan")
    target_lat: float = float("nan")
    target_lon: float = float("nan")
    carrot_lat: float = float("nan")
    carrot_lon: float = float("nan")
    current_heading: float = float("nan")
    desired_heading: float = float("nan")
    left_pulse: int = 0
    right_pulse: int = 0
    guidance_state: str = ""
    motor_enabled: int = -1
    force_action_enabled: int = -1
    release_action_enabled: int = -1
    egg_action_enabled: int = -1
    packet_count: int = 0


tlm_data = TelemetryData()
TEAM_ID = "1070"

# Last start/target lat/lon actually printed on the wire — repeats omit coords to shorten CSV.
_TLM_DL_START: Optional[tuple[float, float]] = None
_TLM_DL_TARGET: Optional[tuple[float, float]] = None

# Set in commapp_main — used so CX,OFF can emit one final TLM line (cmd_echo = CX).
_comm_serial: Optional[object] = None

# Set by SIMP command; cleared on SIM,DISABLE (mode F). While mode A/S and this is set,
# barometer IPC must not overwrite tlm_data.altitude (otherwise one row shows SIMP then HW).
_simp_tlm_alt_hold: Optional[float] = None


def set_cmdecho(cmd_str: str) -> None:
    """Record last uplink command keyword only (no payload) in cmd_echo."""
    tlm_data.cmd_echo = cmd_str.strip()


def _reset_tlm_geo_dedupe() -> None:
    global _TLM_DL_START, _TLM_DL_TARGET
    _TLM_DL_START = None
    _TLM_DL_TARGET = None


def _fmt_latlon_pair_deduped(
    lat: float,
    lon: float,
    last_sent: Optional[tuple[float, float]],
    fmt: str,
    eps: float = 1e-6,
) -> tuple[str, str, Optional[tuple[float, float]]]:
    """Emit lat/lon strings once per distinct pair; repeats -> empty fields."""
    if not (math.isfinite(lat) and math.isfinite(lon)):
        return "", "", last_sent
    if last_sent is not None:
        if abs(lat - last_sent[0]) <= eps and abs(lon - last_sent[1]) <= eps:
            return "", "", last_sent
    return format(lat, fmt), format(lon, fmt), (lat, lon)


def get_current_time() -> str:
    return (datetime.now() - ST_timedelta).strftime("%H:%M:%S")


def set_timedelta(timestr: str) -> bool:
    global ST_timedelta

    if timestr.upper() == "GPS":
        ST_timedelta = timedelta(seconds=0)
        prevstate.update_st_timedelta(0.0)
        return True

    try:
        target = datetime.strptime(timestr, "%H:%M:%S")
    except ValueError:
        return False

    now = datetime.now()
    today_target = now.replace(
        hour=target.hour, minute=target.minute, second=target.second, microsecond=0
    )
    ST_timedelta = now - today_target
    prevstate.update_st_timedelta(ST_timedelta.total_seconds())
    return True


def cmd_cx(option: str, _main_queue) -> bool:
    global TELEMETRY_ENABLE
    upper = option.strip().upper()
    if upper == "ON":
        TELEMETRY_ENABLE = True
        return True
    if upper == "OFF":
        TELEMETRY_ENABLE = False
        # Downlink stops after this frame: set_cmdecho already applied CX,OFF above.
        ser = _comm_serial
        if ser is not None and getattr(ser, "is_open", False):
            _send_one_tlm_frame(ser)
        return True
    return False


def cmd_st(option: str, _main_queue) -> bool:
    return set_timedelta(option.strip())


def cmd_sim(option: str, main_queue) -> bool:
    option = option.strip().upper()
    if option not in {"ENABLE", "ACTIVATE", "DISABLE"}:
        return False
    return msgstructure.send_msg(
        main_queue,
        appargs.CommAppArg.AppID,
        appargs.FlightlogicAppArg.AppID,
        appargs.CommAppArg.MID_RouteCmd_SIM,
        option,
    )


def cmd_simp(option: str, main_queue) -> bool:
    global _simp_tlm_alt_hold
    try:
        value = float(option)
    except ValueError:
        return False
    _simp_tlm_alt_hold = value
    tlm_data.altitude = value
    return msgstructure.send_msg(
        main_queue,
        appargs.CommAppArg.AppID,
        appargs.FlightlogicAppArg.AppID,
        appargs.CommAppArg.MID_RouteCmd_SIMP,
        f"{value}",
    )


def cmd_simg(option: str, main_queue) -> bool:
    """lat,lon,course_deg,speed_m_s[,alt_m] — forwarded to FlightLogic (SIM ACTIVATE required on FSW)."""
    parts = [x.strip() for x in option.split(",") if x.strip() != ""]
    if len(parts) not in (4, 5):
        return False
    try:
        lat = float(parts[0])
        lon = float(parts[1])
        float(parts[2])
        float(parts[3])
        if len(parts) == 5:
            float(parts[4])
    except ValueError:
        return False
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return False
    if lat == 0.0 and lon == 0.0:
        return False
    return msgstructure.send_msg(
        main_queue,
        appargs.CommAppArg.AppID,
        appargs.FlightlogicAppArg.AppID,
        appargs.CommAppArg.MID_RouteCmd_SIMG,
        option.strip(),
    )


def cmd_cal(option: str, main_queue) -> bool:
    return msgstructure.send_msg(
        main_queue,
        appargs.CommAppArg.AppID,
        appargs.BarometerAppArg.AppID,
        appargs.CommAppArg.MID_RouteCmd_CAL,
        option.strip(),
    )


def cmd_mec(option: str, main_queue) -> bool:
    option = option.strip().upper()
    if option not in {"ON", "OFF"}:
        return False
    return msgstructure.send_msg(
        main_queue,
        appargs.CommAppArg.AppID,
        appargs.MotorAppArg.AppID,
        appargs.CommAppArg.MID_RouteCmd_MEC,
        option,
    )


def cmd_fac(option: str, main_queue) -> bool:
    opt = option.strip().upper().replace(" ", "")
    if opt in {"ON", "OFF"}:
        payload = f"ALL,{opt}"
    else:
        parts = [p for p in opt.split(",") if p]
        if len(parts) != 2:
            return False
        actor, state = parts[0], parts[1]
        actor_alias = {
            "REL": "REL",
            "RELEASE": "REL",
            "EGG": "EGG",
        }
        actor_norm = actor_alias.get(actor)
        if actor_norm is None or state not in {"ON", "OFF"}:
            return False
        payload = f"{actor_norm},{state}"
    return msgstructure.send_msg(
        main_queue,
        appargs.CommAppArg.AppID,
        appargs.MotorAppArg.AppID,
        appargs.CommAppArg.MID_RouteCmd_FAC,
        payload,
    )


def cmd_mtr(option: str, main_queue) -> bool:
    normalized = option.strip().upper()
    aliases = {
        "L": "LEFT",
        "LEFT": "LEFT",
        "N": "NEUTRAL",
        "NEUTRAL": "NEUTRAL",
        "R": "RIGHT",
        "RIGHT": "RIGHT",
    }
    mode = aliases.get(normalized)
    if mode is None:
        return False
    return msgstructure.send_msg(
        main_queue,
        appargs.CommAppArg.AppID,
        appargs.MotorAppArg.AppID,
        appargs.CommAppArg.MID_RouteCmd_MTR,
        mode,
    )


def cmd_ss(option: str, main_queue) -> bool:
    try:
        state = int(option)
    except ValueError:
        return False
    if state < 0 or state > 5:
        return False
    return msgstructure.send_msg(
        main_queue,
        appargs.CommAppArg.AppID,
        appargs.FlightlogicAppArg.AppID,
        appargs.CommAppArg.MID_RouteCmd_SS,
        str(state),
    )


def cmd_rbt(option: str, _main_queue) -> bool:
    global _RBT_LAST_SEQ, _RBT_RECENT_NONCES
    token, seq, nonce = _parse_rbt_auth(option)

    if not _RBT_AUTH_TOKEN or token != _RBT_AUTH_TOKEN:
        return False
    if _RBT_SAFE_STATES and str(tlm_data.state) not in _RBT_SAFE_STATES:
        return False
    if _RBT_REQUIRE_SEQ:
        if seq is None or seq <= _RBT_LAST_SEQ:
            return False
        _RBT_LAST_SEQ = seq
    if nonce:
        if nonce in _RBT_RECENT_NONCES:
            return False
        _RBT_RECENT_NONCES.append(nonce)
        if len(_RBT_RECENT_NONCES) > _RBT_NONCE_WINDOW:
            _RBT_RECENT_NONCES = _RBT_RECENT_NONCES[-_RBT_NONCE_WINDOW:]

    _execute_reboot()
    return True


def _execute_reboot() -> None:
    os.system("systemctl reboot -i")


def _parse_rbt_auth(option: str) -> tuple[str, Optional[int], Optional[str]]:
    # Supported forms:
    # - token
    # - token,seq
    # - token,seq,nonce
    # - token:seq:nonce
    opt = option.strip()
    parts = [p.strip() for p in (opt.split(":") if ":" in opt else opt.split(","))]
    parts = [p for p in parts if p != ""]
    if not parts:
        return "", None, None

    token = parts[0]
    seq = None
    nonce = None
    if len(parts) >= 2:
        try:
            seq = int(parts[1])
        except ValueError:
            seq = None
    if len(parts) >= 3:
        nonce = parts[2]
    return token, seq, nonce


def cmd_cam(option: str, main_queue) -> bool:
    option = option.strip().upper()
    if option not in {"ON", "OFF"}:
        return False
    return msgstructure.send_msg(
        main_queue,
        appargs.CommAppArg.AppID,
        appargs.CameraAppArg.AppID,
        appargs.CommAppArg.MID_RouteCmd_CAM,
        option,
    )


def cmd_tc(option: str, main_queue) -> bool:
    parts = [x.strip() for x in option.split(",")]
    if len(parts) != 2:
        return False
    try:
        lat = float(parts[0])
        lon = float(parts[1])
    except ValueError:
        return False
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return False
    return msgstructure.send_msg(
        main_queue,
        appargs.CommAppArg.AppID,
        appargs.FlightlogicAppArg.AppID,
        appargs.CommAppArg.MID_RouteCmd_TC,
        f"{lat},{lon}",
    )


def cmd_xrst(option: str, _main_queue) -> bool:
    upper = option.strip().upper()
    if upper not in {"NOW", "1", "ON"}:
        return False
    xbeereset.send_reset_pulse()
    return True


def command_handler(recv_msg: str) -> None:
    global COMMAPP_RUNSTATUS, _simp_tlm_alt_hold
    unpacked = msgstructure.unpack_msg(recv_msg)
    if unpacked is False:
        return

    mid = unpacked.msg_id
    fields = unpacked.data.split(",") if unpacked.data else []

    try:
        if mid == appargs.MainAppArg.MID_TerminateProcess:
            COMMAPP_RUNSTATUS = False
        elif mid == appargs.BarometerAppArg.MID_comm_alt and len(fields) >= 3:
            # Health field (index 3) is optional for backward compatibility
            # with older barometer payloads that emitted only "p,t,alt".
            baro_health = 1
            if len(fields) >= 4:
                try:
                    baro_health = int(float(fields[3]))
                except ValueError:
                    baro_health = 1
            tlm_data.pressure = float(fields[0])
            tlm_data.temperature = float(fields[1])
            if tlm_data.mode in {"A", "S"} and _simp_tlm_alt_hold is not None:
                # SIMP-injected altitude wins over a hardware sample (healthy or not).
                pass
            elif baro_health:
                tlm_data.altitude = float(fields[2])
            # baro_health == 0: hold last known altitude on the TLM line; do not
            # overwrite with the synthetic 0.0 frame the driver emits when the
            # BMP read fails.
        elif mid == appargs.ImuAppArg.MID_comm_euler and len(fields) >= 12:
            tlm_data.filtered_roll = float(fields[0])
            tlm_data.filtered_pitch = float(fields[1])
            tlm_data.filtered_yaw = float(fields[2])
            tlm_data.acc_x = float(fields[3])
            tlm_data.acc_y = float(fields[4])
            tlm_data.acc_z = float(fields[5])
            tlm_data.mag_roll = float(fields[6])
            tlm_data.mag_pitch = float(fields[7])
            tlm_data.mag_yaw = float(fields[8])
            tlm_data.gyro_roll = float(fields[9])
            tlm_data.gyro_pitch = float(fields[10])
            tlm_data.gyro_yaw = float(fields[11])
        elif mid == appargs.GpsAppArg.MID_comm_gga and len(fields) >= 5:
            tlm_data.gps_time = fields[0]
            tlm_data.gps_alt = float(fields[1])
            tlm_data.gps_lat = float(fields[2])
            tlm_data.gps_lon = float(fields[3])
            tlm_data.gps_sats = int(float(fields[4]))
        elif mid == appargs.ElectroAppArg.MID_comm_volt and len(fields) >= 3:
            tlm_data.voltage = float(fields[0])
            tlm_data.current = float(fields[1])
            tlm_data.power = float(fields[2])
        elif mid == appargs.DistanceAppArg.MID_comm_dis and len(fields) >= 1:
            tlm_data.distance = float(fields[0])
        elif mid == appargs.FlightlogicAppArg.MID_comm_state and len(fields) >= 1:
            tlm_data.state = fields[0]
        elif mid == appargs.FlightlogicAppArg.MID_comm_sim and len(fields) >= 1:
            tlm_data.mode = fields[0]
            if fields[0].strip().upper() == "F":
                _simp_tlm_alt_hold = None
                _reset_tlm_geo_dedupe()
        elif mid == appargs.MotorAppArg.MID_comm_motor_diag and len(fields) >= 11:
            tlm_data.left_pulse = int(float(fields[0]))
            tlm_data.right_pulse = int(float(fields[1]))
            tlm_data.start_lat = float(fields[2])
            tlm_data.start_lon = float(fields[3])
            tlm_data.target_lat = float(fields[4])
            tlm_data.target_lon = float(fields[5])
            tlm_data.carrot_lat = float(fields[6])
            tlm_data.carrot_lon = float(fields[7])
            tlm_data.current_heading = float(fields[8])
            tlm_data.desired_heading = float(fields[9])
            tlm_data.guidance_state = fields[10].strip()
            if len(fields) >= 12:
                tlm_data.motor_enabled = int(float(fields[11]))
            if len(fields) >= 13:
                tlm_data.force_action_enabled = int(float(fields[12]))
            if len(fields) >= 14:
                tlm_data.release_action_enabled = int(float(fields[13]))
            if len(fields) >= 15:
                tlm_data.egg_action_enabled = int(float(fields[14]))
    except (ValueError, TypeError) as exc:
        logger.warning(
            "Dropped malformed telemetry payload mid=%s data=%r (%s)",
            mid,
            unpacked.data,
            exc,
        )


def _tlm_multiline_for_console(line: str) -> str:
    """Pretty multi-line TLM for local logs only (radio line stays one CSV row)."""
    parts = line.rstrip("\n\r").split(",")
    if len(parts) >= 30:
        hdr = ",".join(parts[0:5])
        baro = ",".join(parts[5:8])
        elec = ",".join(parts[8:11])
        gyro = ",".join(parts[11:14])
        acc = ",".join(parts[14:17])
        mag = ",".join(parts[17:20])
        gps = ",".join(parts[20:25])
        extra = ",".join(parts[25:30])
        return (
            "TLM\n"
            f"  meta   : {hdr}\n"
            f"  baro   : {baro}\n"
            f"  power  : {elec}\n"
            f"  gyro   : {gyro}\n"
            f"  acc    : {acc}\n"
            f"  mag    : {mag}\n"
            f"  gps    : {gps}\n"
            f"  extra  : {extra}"
        )
    # Unusual field count (e.g. cmd_echo with comma): wrap every 5 fields
    lines = []
    for i in range(0, len(parts), 5):
        lines.append(",".join(parts[i : i + 5]))
    return "TLM\n" + "\n".join(f"  {ln}" for ln in lines)


def _fmt_opt_float(value: float, fmt: str) -> str:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return ""
    if not math.isfinite(v):
        return ""
    return format(v, fmt)


def _send_one_tlm_frame(serial_instance) -> None:
    global _TLM_SEND_FAIL_LOGGED, _LAST_TLM_FAIL_LOG_TS, _TLM_DL_START, _TLM_DL_TARGET
    tlm_data.packet_count += 1
    prevstate.update_packet_count(tlm_data.packet_count)
    s_lat_s, s_lon_s, _TLM_DL_START = _fmt_latlon_pair_deduped(
        tlm_data.start_lat, tlm_data.start_lon, _TLM_DL_START, ".6f"
    )
    t_lat_s, t_lon_s, _TLM_DL_TARGET = _fmt_latlon_pair_deduped(
        tlm_data.target_lat, tlm_data.target_lon, _TLM_DL_TARGET, ".6f"
    )
    motor_enabled_s = ""
    try:
        me = int(tlm_data.motor_enabled)
        if me >= 0:
            motor_enabled_s = str(int(bool(me)))
    except (TypeError, ValueError):
        motor_enabled_s = ""
    force_action_enabled_s = ""
    try:
        fe = int(tlm_data.force_action_enabled)
        if fe >= 0:
            force_action_enabled_s = str(int(bool(fe)))
    except (TypeError, ValueError):
        force_action_enabled_s = ""
    release_action_enabled_s = ""
    try:
        re = int(tlm_data.release_action_enabled)
        if re >= 0:
            release_action_enabled_s = str(int(bool(re)))
    except (TypeError, ValueError):
        release_action_enabled_s = ""
    egg_action_enabled_s = ""
    try:
        ee = int(tlm_data.egg_action_enabled)
        if ee >= 0:
            egg_action_enabled_s = str(int(bool(ee)))
    except (TypeError, ValueError):
        egg_action_enabled_s = ""

    line = (
        f"${TEAM_ID},{get_current_time()},{tlm_data.packet_count},"
        f"{tlm_data.mode},{tlm_data.state},"
        f"{tlm_data.altitude:.2f},{tlm_data.temperature:.2f},{tlm_data.pressure:.2f},"
        f"{tlm_data.voltage:.3f},{tlm_data.current:.3f},{tlm_data.power:.3f},"
        f"{tlm_data.gyro_roll:.3f},{tlm_data.gyro_pitch:.3f},{tlm_data.gyro_yaw:.3f},"
        f"{tlm_data.acc_x:.3f},{tlm_data.acc_y:.3f},{tlm_data.acc_z:.3f},"
        f"{tlm_data.mag_roll:.3f},{tlm_data.mag_pitch:.3f},{tlm_data.mag_yaw:.3f},"
        f"{tlm_data.gps_time},{tlm_data.gps_alt:.2f},{tlm_data.gps_lat:.6f},{tlm_data.gps_lon:.6f},{tlm_data.gps_sats},"
        f"{tlm_data.distance:.1f},{tlm_data.cmd_echo},"
        f"{tlm_data.filtered_roll:.3f},{tlm_data.filtered_pitch:.3f},{tlm_data.filtered_yaw:.3f},"
        f"{s_lat_s},{s_lon_s},"
        f"{t_lat_s},{t_lon_s},"
        f"{_fmt_opt_float(tlm_data.carrot_lat, '.6f')},{_fmt_opt_float(tlm_data.carrot_lon, '.6f')},"
        f"{_fmt_opt_float(tlm_data.current_heading, '.2f')},{_fmt_opt_float(tlm_data.desired_heading, '.2f')},"
        f"{tlm_data.left_pulse},{tlm_data.right_pulse},{tlm_data.guidance_state},{motor_enabled_s},{force_action_enabled_s},"
        f"{release_action_enabled_s},{egg_action_enabled_s}\n"
    )
    ok = uartserial.send_serial_data(serial_instance, line)
    if ok:
        if _LOG_TLM_TO_CONSOLE:
            logger.info("TLM TX OK\n%s", _tlm_multiline_for_console(line))
        _TLM_SEND_FAIL_LOGGED = False
        return

    now = time.time()
    if _LOG_TLM_TO_CONSOLE and (now - _LAST_TLM_FAIL_LOG_TS) >= 3.0:
        logger.warning("TLM TX FAIL (UART write failed)\n%s", _tlm_multiline_for_console(line))
        _LAST_TLM_FAIL_LOG_TS = now
    if not _TLM_SEND_FAIL_LOGGED:
        logger.warning(
            "TLM UART write failed (no bytes will reach the radio/USB adapter). "
            "Check UART mapping/permissions and explicit UART_DEVICE (Windows: COMx, Linux: /dev/tty*)."
        )
        _TLM_SEND_FAIL_LOGGED = True


def send_tlm(serial_instance) -> None:
    if not TELEMETRY_ENABLE:
        return
    _send_one_tlm_frame(serial_instance)


def _dispatch_command(line: str, main_queue) -> bool:
    # Normalize command "CMD,1070,<body>"
    line = line.strip()
    m = re.fullmatch(r"CMD,\s*1070,\s*([A-Za-z]+),(.*)", line, re.IGNORECASE)
    if not m:
        return False
    cmd = m.group(1).upper()
    option = m.group(2).strip()
    set_cmdecho(cmd)

    if cmd == "CX":
        return cmd_cx(option, main_queue)
    if cmd == "ST":
        return cmd_st(option, main_queue)
    if cmd == "SIM":
        return cmd_sim(option, main_queue)
    if cmd == "SIMP":
        return cmd_simp(option, main_queue)
    if cmd == "SIMG":
        return cmd_simg(option, main_queue)
    if cmd == "CAL":
        return cmd_cal(option, main_queue)
    if cmd == "MEC":
        return cmd_mec(option, main_queue)
    if cmd == "FAC":
        return cmd_fac(option, main_queue)
    if cmd == "MTR":
        return cmd_mtr(option, main_queue)
    if cmd == "SS":
        return cmd_ss(option, main_queue)
    if cmd == "RBT":
        return cmd_rbt(option, main_queue)
    if cmd == "CAM":
        return cmd_cam(option, main_queue)
    if cmd == "TC":
        return cmd_tc(option, main_queue)
    if cmd == "XRST":
        return cmd_xrst(option, main_queue)
    return False


def read_cmd(main_queue, serial_instance) -> None:
    while COMMAPP_RUNSTATUS:
        line = uartserial.receive_serial_data(serial_instance)
        if not line or line == "OK":
            time.sleep(0.05)
            continue
        _dispatch_command(line, main_queue)
        time.sleep(0.01)


# Telemetry transmit rate is fixed at 1 Hz (CanSat competition spec).
# Do not change unless you are explicitly off-spec for ground testing.
def _tlm_sender(serial_instance) -> None:
    while COMMAPP_RUNSTATUS:
        send_tlm(serial_instance)
        time.sleep(1.0)


def commapp_main(main_queue, main_pipe) -> None:
    global ST_timedelta, _comm_serial
    prevstate.init_prevstate()
    ST_timedelta = timedelta(seconds=prevstate.PREV_ST_TIMEDELTA)
    tlm_data.packet_count = prevstate.PREV_PACKET_COUNT

    serial_instance = uartserial.init_serial()
    _comm_serial = serial_instance
    try:
        logger.info("Sending XBee reset pulse on comm startup")
        xbeereset.send_reset_pulse()
    except Exception as exc:
        logger.warning("XBee reset pulse on startup failed: %s", exc)
    if uartserial.is_dummy_serial(serial_instance):
        logger.warning(
            "Comm UART is in DummySerial mode (no hardware TX/RX). "
            "Fix UART_DEVICE/pyserial/port permissions, or run with FSW_LOG_TLM=1 "
            "to print TLM lines to the FSW log."
        )
    sender = threading.Thread(target=_tlm_sender, args=(serial_instance,), daemon=True)
    reader = threading.Thread(target=read_cmd, args=(main_queue, serial_instance), daemon=True)
    sender.start()
    reader.start()

    try:
        while COMMAPP_RUNSTATUS:
            try:
                has_msg = main_pipe.poll(0.1)
            except (KeyboardInterrupt, EOFError, OSError):
                break
            if has_msg:
                recv_msg = main_pipe.recv()
                command_handler(recv_msg)
    except KeyboardInterrupt:
        pass
    finally:
        _comm_serial = None
        uartserial.terminate_serial(serial_instance)
