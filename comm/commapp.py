"""Comm app: command parsing + telemetry aggregation/sending."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
import logging
import math
import os
import re
import subprocess
import threading
import time
from typing import Optional

from lib import appargs, config, msgstructure, prevstate
from comm import uartserial, xbeereset


logger = logging.getLogger(__name__)

COMMAPP_RUNSTATUS = True
TELEMETRY_ENABLE = True
ST_timedelta = timedelta(seconds=0)

_LOG_TLM_TO_CONSOLE = os.environ.get("FSW_LOG_TLM", "1").strip() != "0"
_force_send_event = threading.Event()   # 단명 상태 진입 시 즉시 TLM 강제 송신용
_TLM_SEND_FAIL_LOGGED = False
_LAST_TLM_FAIL_LOG_TS = 0.0

_RBT_AUTH_TOKEN = os.environ.get("RBT_AUTH_TOKEN", "").strip()
_RBT_REQUIRE_SEQ = os.environ.get("RBT_REQUIRE_SEQ", "1").strip() != "0"
_RBT_SAFE_STATES = {x.strip() for x in os.environ.get("RBT_SAFE_STATES", "0,6").split(",") if x.strip()}
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
    acc_roll: float = 0.0
    acc_pitch: float = 0.0
    acc_yaw: float = 0.0
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
    left_pulse: int = 0
    right_pulse: int = 0
    guidance_state: str = ""
    motor_enabled: int = -1
    force_action_enabled: int = -1
    release_action_enabled: int = -1
    egg_action_enabled: int = -1
    nav_distance_mm: float = float("nan")  # GPS→타겟 haversine 거리 (mm)
    packet_count: int = 0


tlm_data = TelemetryData()
TEAM_ID = "1070"

_STATE_NAMES: dict[str, str] = {
    "0": "LAUNCH_PAD",
    "1": "ASCENT",
    "2": "APOGEE",
    "3": "DESCENT",
    "4": "PAYLOAD_RELEASE",   # 번와이어 → 패러포일 전개
    "5": "PROBE_RELEASE",     # 솔레노이드 → 에그 방출 (~2m, 단명)
    "6": "LANDED",
}
# PROBE_RELEASE(5)는 2m 고도에서 ~0.4초짜리 단명 상태.
# 반드시 _force_send_event를 통해 즉시 1프레임 강제 송신.
_FORCE_SEND_STATES = {"5"}   # PROBE_RELEASE 진입 시 즉시 송신


# Set in commapp_main — used so CX,OFF can emit one final TLM line (cmd_echo = CX).
_comm_serial: Optional[object] = None

# Set by SIMP command; cleared on SIM,DISABLE (mode F). While mode A/S and this is set,
# barometer IPC must not overwrite tlm_data.altitude (otherwise one row shows SIMP then HW).
_simp_tlm_alt_hold: Optional[float] = None


def set_cmdecho(cmd_str: str) -> None:
    """Record last uplink command keyword only (no payload) in cmd_echo."""
    tlm_data.cmd_echo = cmd_str.strip()



def get_current_time() -> str:
    # Mission time은 UTC 기준. datetime.now(timezone.utc)는 OS 타임존과 무관하게
    # 항상 UTC를 반환하므로, ST_timedelta=0(기본)이면 미션시간 = UTC 벽시계.
    return (datetime.now(timezone.utc) - ST_timedelta).strftime("%H:%M:%S")


def set_timedelta(timestr: str) -> bool:
    global ST_timedelta

    if timestr.upper() == "GPS":
        # GPS 시간 수신: tlm_data.gps_time (HH:MM:SS 형식, UTC)
        gps_time_str = tlm_data.gps_time.strip()
        
        if not gps_time_str or gps_time_str == "00:00:00":
            logger.warning("ST,GPS: GPS 시간 없음 (GPS 신호 미수신 또는 미동기)")
            return False
        
        try:
            # GPS 시간 파싱 (HH:MM:SS, UTC)
            gps_time_obj = datetime.strptime(gps_time_str, "%H:%M:%S")
            today = datetime.now().date()
            gps_datetime = datetime.combine(today, gps_time_obj.time())
            
            # 시스템 시간을 UTC 기반 GPS 시간으로 설정
            # subprocess를 사용해서 실제 시스템 시간 업데이트 (Raspberry Pi: date 명령어)
            time_str = gps_datetime.strftime("%H:%M:%S")
            try:
                # Linux에서는 date 명령어로 시간 설정 (root 권한 필요)
                subprocess.run(
                    ["sudo", "date", "-s", time_str],
                    check=False,
                    capture_output=True,
                    timeout=5
                )
                logger.info(f"ST,GPS: 시스템 시간 설정 → {time_str} (GPS UTC)")
            except Exception as e:
                logger.warning(f"ST,GPS: 시스템 시간 설정 실패 (sudo 권한?): {e}")
            
            # ST_timedelta 리셋 (GPS가 시스템 시간의 기준이 됨)
            ST_timedelta = timedelta(seconds=0)
            prevstate.update_st_timedelta(0.0)
            return True
        
        except ValueError as e:
            logger.error(f"ST,GPS: GPS 시간 파싱 실패 ({gps_time_str}): {e}")
            return False

    try:
        target = datetime.strptime(timestr, "%H:%M:%S")
    except ValueError:
        return False

    # ST 오프셋도 UTC 기준으로 계산 (get_current_time과 일관성 유지).
    now = datetime.now(timezone.utc)
    today_target = now.replace(
        hour=target.hour, minute=target.minute, second=target.second, microsecond=0
    )
    ST_timedelta = now - today_target
    prevstate.update_st_timedelta(ST_timedelta.total_seconds())
    return True


def _alt_m_to_pressure_hpa(alt_m: float) -> float:
    """ISA 표준 대기: 고도(m) → 기압(hPa)."""
    try:
        return 1013.25 * (1.0 - 2.25577e-5 * float(alt_m)) ** 5.25588
    except (TypeError, ValueError, ZeroDivisionError):
        return 1013.25


def _pressure_pa_to_alt_m(p_pa: float) -> float:
    """ISA 표준 대기: 기압(Pa) → 고도(m)."""
    try:
        ratio = float(p_pa) / 101325.0
        if ratio <= 0.0:
            return 0.0
        return (1.0 - ratio ** (1.0 / 5.25588)) / 2.25577e-5
    except (TypeError, ValueError, ZeroDivisionError):
        return 0.0


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
        pressure_pa = float(option)
    except ValueError:
        return False
    alt_m = _pressure_pa_to_alt_m(pressure_pa)
    alt_relative = alt_m - prevstate.PREV_ALT_CAL  # 해발 절대고도 → 발사대 기준 상대고도
    # TLM 덮어쓰기는 SIM ACTIVATE(mode S) 이후에만. mode A에서는 실 센서 유지.
    if tlm_data.mode == "S":
        _simp_tlm_alt_hold = alt_relative
        tlm_data.altitude = alt_relative
        tlm_data.pressure = pressure_pa / 100.0  # Pa → hPa (절대기압 그대로)
    return msgstructure.send_msg(
        main_queue,
        appargs.CommAppArg.AppID,
        appargs.FlightlogicAppArg.AppID,
        appargs.CommAppArg.MID_RouteCmd_SIMP,
        f"{alt_relative}",
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
        alt = float(parts[4]) if len(parts) == 5 else None
    except ValueError:
        return False
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return False
    if lat == 0.0 and lon == 0.0:
        return False
    tlm_data.gps_lat = lat
    tlm_data.gps_lon = lon
    if alt is not None:
        tlm_data.gps_alt = alt
    return msgstructure.send_msg(
        main_queue,
        appargs.CommAppArg.AppID,
        appargs.FlightlogicAppArg.AppID,
        appargs.CommAppArg.MID_RouteCmd_SIMG,
        option.strip(),
    )


def cmd_simgn(option: str, main_queue) -> bool:
    """SIMGN: GPS null 모드 — pos_health=0 강제로 즉시 stale. SIM ACTIVATE 필요."""
    return msgstructure.send_msg(
        main_queue,
        appargs.CommAppArg.AppID,
        appargs.FlightlogicAppArg.AppID,
        appargs.CommAppArg.MID_RouteCmd_SIMGN,
        "",
    )


def cmd_simgr(option: str, main_queue) -> bool:
    """east_m,north_m,course_deg,speed_m_s[,alt_m] — relative offset from target; forwarded to FlightLogic."""
    parts = [x.strip() for x in option.split(",") if x.strip() != ""]
    if len(parts) not in (4, 5):
        return False
    try:
        float(parts[0])  # east_m
        float(parts[1])  # north_m
        float(parts[2])  # course_deg
        float(parts[3])  # speed_m_s
        if len(parts) == 5:
            float(parts[4])  # alt_m
    except ValueError:
        return False
    return msgstructure.send_msg(
        main_queue,
        appargs.CommAppArg.AppID,
        appargs.FlightlogicAppArg.AppID,
        appargs.CommAppArg.MID_RouteCmd_SIMGR,
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
    # Spec: MEC,<DEVICE>,<ON|OFF>
    # DEVICE: MOTOR | RELEASE | EGG
    parts = [p.strip().upper() for p in option.split(",") if p.strip()]
    if len(parts) == 2:
        device, state = parts
    elif len(parts) == 1:
        # 하위 호환: MEC,ON|OFF (DEVICE 생략 시 MOTOR로 간주)
        device, state = "MOTOR", parts[0]
    else:
        return False

    if state not in {"ON", "OFF"}:
        return False

    if device == "MOTOR":
        return msgstructure.send_msg(
            main_queue,
            appargs.CommAppArg.AppID,
            appargs.MotorAppArg.AppID,
            appargs.CommAppArg.MID_RouteCmd_MEC,
            state,
        )
    if device in {"RELEASE", "EGG"}:
        return msgstructure.send_msg(
            main_queue,
            appargs.CommAppArg.AppID,
            appargs.MotorAppArg.AppID,
            appargs.CommAppArg.MID_RouteCmd_FAC,
            f"{device},{state}",
        )
    return False


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
    if state < 0 or state > 6:
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


def cmd_cmc(option: str, main_queue) -> bool:
    aliases = {
        "GPS":         config.MOTOR_CTRL_MODE_GPS_GUIDED,
        "GPS_GUIDED":  config.MOTOR_CTRL_MODE_GPS_GUIDED,
        "GPS_ONLY":    config.MOTOR_CTRL_MODE_GPS_ONLY,
        "IMU":         config.MOTOR_CTRL_MODE_IMU_HEADING,
        "IMU_HEADING": config.MOTOR_CTRL_MODE_IMU_HEADING,
    }
    mode = aliases.get(option.strip().upper())
    if mode is None:
        return False
    return msgstructure.send_msg(
        main_queue,
        appargs.CommAppArg.AppID,
        appargs.MotorAppArg.AppID,
        appargs.CommAppArg.MID_RouteCmd_CMC,
        mode,
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
            tlm_data.temperature = float(fields[1])
            if tlm_data.mode == "S" and _simp_tlm_alt_hold is not None:
                # SIM ACTIVATE 이후에만 SIMP 값이 하드웨어 바로미터를 대체.
                # mode "A"(ENABLE only)에서는 실 센서 유지.
                pass
            else:
                tlm_data.pressure = float(fields[0])
                if baro_health:
                    tlm_data.altitude = float(fields[2])
                # baro_health == 0: hold last known altitude on the TLM line; do not
                # overwrite with the synthetic 0.0 frame the driver emits when the
                # BMP read fails.
        elif mid == appargs.ImuAppArg.MID_comm_euler and len(fields) >= 12:
            tlm_data.filtered_roll = float(fields[0])
            tlm_data.filtered_pitch = float(fields[1])
            tlm_data.filtered_yaw = float(fields[2])
            tlm_data.acc_roll = float(fields[3])
            tlm_data.acc_pitch = float(fields[4])
            tlm_data.acc_yaw = float(fields[5])
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
            if fields[0] in _FORCE_SEND_STATES:
                _force_send_event.set()
        elif mid == appargs.FlightlogicAppArg.MID_comm_nav_dis and len(fields) >= 1:
            tlm_data.nav_distance_mm = float(fields[0])
        elif mid == appargs.FlightlogicAppArg.MID_comm_sim and len(fields) >= 1:
            tlm_data.mode = fields[0]
            if fields[0].strip().upper() == "F":
                _simp_tlm_alt_hold = None
        elif mid == appargs.MotorAppArg.MID_comm_motor_diag and len(fields) >= 10:
            tlm_data.left_pulse = int(float(fields[0]))
            tlm_data.right_pulse = int(float(fields[1]))
            tlm_data.start_lat = float(fields[2])
            tlm_data.start_lon = float(fields[3])
            tlm_data.target_lat = float(fields[4])
            tlm_data.target_lon = float(fields[5])
            tlm_data.carrot_lat = float(fields[6])
            tlm_data.carrot_lon = float(fields[7])
            tlm_data.current_heading = float(fields[8])
            tlm_data.guidance_state = fields[9].strip()
            if len(fields) >= 11:
                tlm_data.motor_enabled = int(float(fields[10]))
            if len(fields) >= 12:
                tlm_data.force_action_enabled = int(float(fields[11]))
            if len(fields) >= 13:
                tlm_data.release_action_enabled = int(float(fields[12]))
            if len(fields) >= 14:
                tlm_data.egg_action_enabled = int(float(fields[13]))
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
        def field(index: int) -> str:
            return parts[index] if index < len(parts) else ""

        hdr = ",".join(field(i) for i in range(0, 5))
        baro = ",".join(field(i) for i in range(5, 8))
        elec = ",".join((field(8), field(9), field(23)))
        gyro = ",".join(field(i) for i in range(10, 13))
        acc = ",".join(field(i) for i in range(13, 16))
        gps = ",".join(field(i) for i in range(16, 21))
        mag = ",".join(field(i) for i in range(24, 27))
        extra = ",".join((field(21), field(27), field(28), field(29), field(30)))
        text = (
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
        guide = ",".join(field(i) for i in range(31, 46))
        if any(field(i) for i in range(31, 46)):
            text += f"\n  guide  : {guide}"
        return text
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
    global _TLM_SEND_FAIL_LOGGED, _LAST_TLM_FAIL_LOG_TS
    tlm_data.packet_count += 1
    prevstate.update_packet_count(tlm_data.packet_count)
    s_lat_s = _fmt_opt_float(tlm_data.start_lat, '.6f')
    s_lon_s = _fmt_opt_float(tlm_data.start_lon, '.6f')
    t_lat_s = _fmt_opt_float(tlm_data.target_lat, '.6f')
    t_lon_s = _fmt_opt_float(tlm_data.target_lon, '.6f')
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
        # ── Required fields (spec 3.1.1.1, fields 1-22) ──────────────────────
        f"${TEAM_ID},{get_current_time()},{tlm_data.packet_count},"
        f"{tlm_data.mode},{_STATE_NAMES.get(str(tlm_data.state), tlm_data.state)},"
        f"{tlm_data.altitude:.2f},{tlm_data.temperature:.2f},{tlm_data.pressure / 10.0:.1f},"  # pressure hPa→kPa
        f"{tlm_data.voltage:.3f},{tlm_data.current:.2f},"                                       # current 0.01A res
        f"{tlm_data.gyro_roll:.3f},{tlm_data.gyro_pitch:.3f},{tlm_data.gyro_yaw:.3f},"
        f"{tlm_data.acc_roll:.3f},{tlm_data.acc_pitch:.3f},{tlm_data.acc_yaw:.3f},"
        f"{tlm_data.gps_time},{tlm_data.gps_alt:.2f},{tlm_data.gps_lat:.4f},{tlm_data.gps_lon:.4f},{tlm_data.gps_sats},"
        f"{tlm_data.cmd_echo},,"
        # ── Optional fields (after blank field ,, per spec) ───────────────────
        f"{tlm_data.power:.3f},"
        f"{tlm_data.mag_roll:.3f},{tlm_data.mag_pitch:.3f},{tlm_data.mag_yaw:.3f},"
        f"{tlm_data.distance:.1f},"
        f"{tlm_data.filtered_roll:.3f},{tlm_data.filtered_pitch:.3f},{tlm_data.filtered_yaw:.3f},"
        f"{s_lat_s},{s_lon_s},"
        f"{t_lat_s},{t_lon_s},"
        f"{_fmt_opt_float(tlm_data.carrot_lat, '.6f')},{_fmt_opt_float(tlm_data.carrot_lon, '.6f')},"
        f"{_fmt_opt_float(tlm_data.current_heading, '.2f')},"
        f"{tlm_data.left_pulse},{tlm_data.right_pulse},{tlm_data.guidance_state},{motor_enabled_s},{force_action_enabled_s},"
        f"{release_action_enabled_s},{egg_action_enabled_s},"
        f"{_fmt_opt_float(tlm_data.nav_distance_mm, '.1f')}\r"
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
    m = re.fullmatch(r"CMD,\s*1070,\s*([A-Za-z]+)(?:,(.*))?$", line, re.IGNORECASE)
    if not m:
        return False
    cmd = m.group(1).upper()
    option = (m.group(2) or "").strip()
    set_cmdecho(cmd + option.replace(",", ""))

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
    if cmd == "SIMGR":
        return cmd_simgr(option, main_queue)
    if cmd == "SIMGN":
        return cmd_simgn(option, main_queue)
    if cmd == "CAL":
        tlm_data.packet_count = 0
        prevstate.update_packet_count(0)
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
    if cmd == "CMC":
        return cmd_cmc(option, main_queue)
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
# _force_send_event가 set되면 1초를 기다리지 않고 즉시 추가 프레임 송신.
# PROBE_RELEASE 같은 단명 상태(<1s)가 CSV에 반드시 남도록 보장.
def _tlm_sender(serial_instance) -> None:
    while COMMAPP_RUNSTATUS:
        send_tlm(serial_instance)
        fired = _force_send_event.wait(timeout=1.0)
        if fired:
            _force_send_event.clear()
            send_tlm(serial_instance)   # 상태 진입 즉시 추가 1프레임


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
