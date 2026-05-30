# Python FSW V2 Gps App
# Author : Hyeon Lee

import math
import os

from lib import appargs
from lib import config
from lib import msgstructure
from lib import events
from Sensor_Gps import gps

import signal
import sys
from multiprocessing import Queue, connection
import threading
import time

# Runstatus of application. Application is terminated when false
GPSAPP_RUNSTATUS = True
gps_instance = None
GPS_STALE_TIMEOUT_SEC = 15.0  # guidance GPS_FRESH_MAX_AGE_S(15s)와 일치 (5.0→15.0)

# pos fidelity 상수 — 환경변수로 오버라이드 가능
GPS_MAX_HDOP     = float(os.environ.get("GPS_MAX_HDOP",     "3.0"))
GPS_MAX_JUMP_MPS = float(os.environ.get("GPS_MAX_JUMP_MPS", "30.0"))
GPS_MIN_MOTION_MPS = float(os.environ.get("GPS_MIN_MOTION_MPS", "0.3"))
GPS_NULL_LAT_TOL = 1.0e-4
GPS_NULL_LON_TOL = 1.0e-4

# jump rate 추적용 상태 (단일 스레드에서만 접근)
_prev_valid_lat: float = 0.0
_prev_valid_lon: float = 0.0
_prev_valid_ts:  float = 0.0

SIM_GPS_ACTIVE = False
SIM_GPS_NULL   = False  # SIMGN: pos_health=0 강제 (즉시 stale)
SIM_GPS_LAT = 0.0
SIM_GPS_LON = 0.0
SIM_GPS_ALT = 0.0
SIM_GPS_SPEED_MS = 0.0
SIM_GPS_COURSE = 0.0
_SIM_GPS_LOCK = threading.Lock()


def _is_finite(value: float) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _optional_float(value):
    return float(value) if _is_finite(value) else None


def _in_expected_area(lat: float, lon: float) -> bool:
    try:
        lat_center = float(config.GPS_EXPECTED_LAT_CENTER_DEG)
        lon_center = float(config.GPS_EXPECTED_LON_CENTER_DEG)
        lat_radius = float(config.GPS_EXPECTED_LAT_RADIUS_DEG)
        lon_radius = float(config.GPS_EXPECTED_LON_RADIUS_DEG)
    except (TypeError, ValueError):
        lat_center = 37.0
        lon_center = 126.6
        lat_radius = 0.009
        lon_radius = 0.011
    return (
        abs(float(lat) - lat_center) <= lat_radius
        and abs(float(lon) - lon_center) <= lon_radius
    )


def _is_placeholder_latlon(lat: float, lon: float) -> bool:
    return abs(float(lat)) <= GPS_NULL_LAT_TOL and abs(float(lon)) <= GPS_NULL_LON_TOL


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    R = 6_371_000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return R * 2.0 * math.asin(math.sqrt(max(0.0, min(1.0, a))))


def _eval_pos_fidelity(
    lat: float, lon: float, hdop: float,
    sats: int, fix_quality: int, now: float,
) -> bool:
    global _prev_valid_lat, _prev_valid_lon, _prev_valid_ts

    # Gate 1: 기본 유효성
    if not (_is_finite(lat) and _is_finite(lon)):
        return False
    if not (-90.0 <= float(lat) <= 90.0 and -180.0 <= float(lon) <= 180.0):
        return False
    if _is_placeholder_latlon(float(lat), float(lon)):
        return False
    if not _in_expected_area(float(lat), float(lon)):
        return False
    if int(fix_quality) < 1:
        return False
    if int(sats) < int(getattr(config, "GPS_MIN_SATS", 4)):
        return False

    # Gate 2: 정밀도 (HDOP)
    if not _is_finite(hdop) or float(hdop) > GPS_MAX_HDOP:
        return False

    # Gate 3: 연속성 (jump rate)
    if _prev_valid_ts > 0:
        dt = now - _prev_valid_ts
        if dt > 0:
            dist_m = _haversine_m(_prev_valid_lat, _prev_valid_lon, float(lat), float(lon))
            if dist_m / dt > GPS_MAX_JUMP_MPS:
                return False

    return True


def _eval_motion_fidelity(
    pos_health: bool,
    rmc_status: str,
    speed_mps: float,
    course_deg: float,
) -> bool:
    max_speed = float(getattr(config, "GPS_MAX_VALID_SPEED_MPS", 40.0))
    return (
        pos_health
        and str(rmc_status).strip().upper() == "A"
        and _is_finite(speed_mps)
        and _is_finite(course_deg)
        and GPS_MIN_MOTION_MPS <= float(speed_mps) <= max_speed
        and 0.0 <= float(course_deg) < 360.0
    )
def _valid_age(ts: float, now: float, max_age: float, future_tol: float = 0.02) -> bool:
    """Return True when ts is finite and within the allowed age window."""
    try:
        age = float(now) - float(ts)
    except (TypeError, ValueError):
        return False
    if not math.isfinite(age):
        return False
    return -future_tol <= age <= max_age


######################################################
## FUNDEMENTAL METHODS                              ##
######################################################

# SB Methods
# Methods for sending/receiving/handling SB messages

# Handles received message
def command_handler (recv_msg : msgstructure.MsgStructure):
    global GPSAPP_RUNSTATUS
    global SIM_GPS_ACTIVE, SIM_GPS_LAT, SIM_GPS_LON, SIM_GPS_ALT, SIM_GPS_SPEED_MS, SIM_GPS_COURSE

    if recv_msg.msg_id == appargs.MainAppArg.MID_TerminateProcess:
        # Change Runstatus to false to start termination process
        events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.info, f"GPSAPP TERMINATION DETECTED")
        GPSAPP_RUNSTATUS = False

    elif recv_msg.msg_id == appargs.GpsAppArg.MID_flight_gps_null:
        with _SIM_GPS_LOCK:
            SIM_GPS_NULL = True
        return

    elif recv_msg.msg_id == appargs.GpsAppArg.MID_flight_gps_sim:
        data = str(recv_msg.data).strip()
        if data.upper() == "CLEAR":
            with _SIM_GPS_LOCK:
                SIM_GPS_ACTIVE = False
                SIM_GPS_NULL   = False
                SIM_GPS_LAT = 0.0
                SIM_GPS_LON = 0.0
                SIM_GPS_ALT = 0.0
                SIM_GPS_SPEED_MS = 0.0
                SIM_GPS_COURSE = 0.0
            return
        parts = [x.strip() for x in data.split(",") if x.strip() != ""]
        if len(parts) not in (4, 5):
            return
        try:
            lat = float(parts[0])
            lon = float(parts[1])
            course = float(parts[2])
            speed = float(parts[3])
            alt = float(parts[4]) if len(parts) == 5 else 0.0
        except ValueError:
            return
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            return
        if lat == 0.0 and lon == 0.0:
            return
        with _SIM_GPS_LOCK:
            SIM_GPS_ACTIVE = True
            SIM_GPS_NULL   = False  # 새 위치 주입 시 null 모드 해제
            SIM_GPS_LAT = lat
            SIM_GPS_LON = lon
            SIM_GPS_ALT = alt
            SIM_GPS_SPEED_MS = speed
            SIM_GPS_COURSE = course % 360.0
        return
    else:
        events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.error, f"MID {recv_msg.msg_id} not handled")
    return

######################################################
## INITIALIZATION, TERMINATION                      ##
######################################################
# Initialization
def gpsapp_init():
    global GPSAPP_RUNSTATUS
    global gps_instance
    try:
        # Disable Keyboardinterrupt since Termination is handled by parent process
        signal.signal(signal.SIGINT, signal.SIG_IGN)

        events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.info, "Initializating gpsapp")
        ## User Defined Initialization goes HERE
        gps_instance = gps.init_gps()
        events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.info, "Gpsapp Initialization Complete")
        return gps_instance
    except Exception as e:
        events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.error, f"Error during initialization: {e}")
        GPSAPP_RUNSTATUS = False
        return None

# Termination
def gpsapp_terminate():
    global GPSAPP_RUNSTATUS
    global gps_instance
    GPSAPP_RUNSTATUS = False
    events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.info, "Terminating gpsapp")
    # Termination Process Comes Here

    # Join Each Thread to make sure all threads terminates
    for thread_name in thread_dict:
        events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.info, f"Terminating thread {thread_name}")
        thread_dict[thread_name].join()
        events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.info, f"Terminating thread {thread_name} Complete")

    if gps_instance is not None:
        gps.terminate_gps(gps_instance)
        gps_instance = None
        events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.info, "GPS connection terminated")

    # The termination flag should switch to false AFTER ALL TERMINATION PROCESS HAS ENDED
    events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.info, "Terminating gpsapp complete")
    return

######################################################
## USER METHOD                                      ##
######################################################

def read_and_send_gps_data(Main_Queue: Queue, gps_instance):
    global GPSAPP_RUNSTATUS

    # 초기값 설정
    GPS_LAT = 0.0
    GPS_LON = 0.0
    GPS_ALT = 0.0
    GPS_TIME = "00:00:00"
    GPS_SATS = 0
    GPS_FIX_QUALITY = 0
    GPS_RMC_STATUS = "V"
    GPS_SPEED_MS = 0.0
    GPS_COURSE = 0.0

    send_counter = 0
    last_valid_gps_ts = 0.0
    last_valid_rmc_ts = 0.0

    while GPSAPP_RUNSTATUS:
        sim_sample = None
        sim_null = False
        with _SIM_GPS_LOCK:
            if SIM_GPS_ACTIVE:
                sim_sample = (
                    SIM_GPS_LAT,
                    SIM_GPS_LON,
                    SIM_GPS_ALT,
                    SIM_GPS_SPEED_MS,
                    SIM_GPS_COURSE,
                )
                sim_null = SIM_GPS_NULL
        if sim_sample is not None:
            GPS_TIME = time.strftime("%H:%M:%S")
            GPS_LAT, GPS_LON, GPS_ALT, GPS_SPEED_MS, GPS_COURSE = sim_sample
            GPS_SATS = max(int(getattr(config, "GPS_MIN_SATS", 4)), 4)
            GPS_FIX_QUALITY = 1
            GPS_RMC_STATUS = "A"
            last_valid_gps_ts = time.monotonic()
            last_valid_rmc_ts = last_valid_gps_ts
            rcv_data = [GPS_TIME, GPS_ALT, GPS_LAT, GPS_LON, GPS_SATS, GPS_FIX_QUALITY, GPS_RMC_STATUS, GPS_SPEED_MS, GPS_COURSE, last_valid_gps_ts, 1.0, last_valid_rmc_ts]  # [10]=hdop=1.0 (sim)
        else:
            # Check if gps_instance is valid
            if gps_instance is None:
                time.sleep(0.04)
                continue

            try:
                rcv_data = gps.gps_readdata(gps_instance)
            except Exception as e:
                events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.error, f"Error reading GPS data: {e}")
                time.sleep(0.1)
                continue

        # 데이터가 유효할 때만 변수를 업데이트한다 (None이거나 유효하지 않으면 이전 값 유지)
        # gps.py 반환:
        # [time, alt, lat, lon, sats, fix_quality, rmc_status, speed_ms, course, gga_sample_ts, hdop, rmc_sample_ts]
        if rcv_data and len(rcv_data) >= 5:
            try:
                GPS_TIME = rcv_data[0]
                GPS_ALT  = _optional_float(rcv_data[1]) or 0.0
                GPS_LAT  = _optional_float(rcv_data[2])
                GPS_LON  = _optional_float(rcv_data[3])
                GPS_SATS = int(rcv_data[4])
                GPS_FIX_QUALITY = int(rcv_data[5]) if len(rcv_data) > 5 else 0
                GPS_RMC_STATUS = str(rcv_data[6]).strip() if len(rcv_data) > 6 else "V"
                GPS_SPEED_MS = _optional_float(rcv_data[7]) if len(rcv_data) > 7 else None
                GPS_COURSE = _optional_float(rcv_data[8]) if len(rcv_data) > 8 else None
                # Use driver-side GGA sample timestamp for stale accounting.
                # This prevents cache re-reads from resetting the stale timeout.
                if len(rcv_data) > 9 and _is_finite(rcv_data[9]) and float(rcv_data[9]) > 0.0:
                    last_valid_gps_ts = float(rcv_data[9])
                else:
                    last_valid_gps_ts = 0.0

                if len(rcv_data) > 11 and _is_finite(rcv_data[11]) and float(rcv_data[11]) > 0.0:
                    last_valid_rmc_ts = float(rcv_data[11])
                else:
                    last_valid_rmc_ts = 0.0
                # Print GPS data for debugging (disabled)
                # print(f"GPS: Time={GPS_TIME}, Lat={GPS_LAT:.6f}, Lon={GPS_LON:.6f}, Alt={GPS_ALT:.2f}, Sats={GPS_SATS}, FixQuality={GPS_FIX_QUALITY}")
                # sys.stdout.flush()
                # GPS 값이 유효한 경우에만 이벤트 로그 출력
                """
                if GPS_LAT != 0.0 or GPS_LON != 0.0 or GPS_SATS > 0:
                    events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.info, f"GPS data read: Lat={GPS_LAT:.6f}, Lon={GPS_LON:.6f}, Alt={GPS_ALT:.2f}, Sats={GPS_SATS}")
                # GPS fix가 없는 경우 (좌표가 0이지만 시간은 있는 경우) 주기적으로 로그 출력
                elif GPS_TIME != "00:00:00" and send_counter % 100 == 0:  # 10초마다
                    events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.warning, f"GPS reading but no fix: Time={GPS_TIME}, Sats={GPS_SATS}, FixQuality={GPS_FIX_QUALITY} (waiting for GPS fix...)")
                """
            except (ValueError, IndexError, TypeError) as e:
                # 파싱 에러 시 이전 값을 유지하고 로그만 남김
                events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.error, f"Error parsing GPS data: {e}, rcv_data={rcv_data}")
        else:
            # GPS 데이터가 없을 때 (None 또는 형식 불일치) - 이전 값 유지
            # 단, 오래된 값은 stale로 간주해 초기값으로 리셋한다.
            if last_valid_gps_ts <= 0 or (time.monotonic() - last_valid_gps_ts) > GPS_STALE_TIMEOUT_SEC:
                GPS_LAT = None
                GPS_LON = None
                GPS_ALT = 0.0
                GPS_TIME = "00:00:00"
                GPS_SATS = 0
                GPS_FIX_QUALITY = 0
                GPS_RMC_STATUS = "V"
                GPS_SPEED_MS = None
                GPS_COURSE = None
                last_valid_rmc_ts = 0.0

        # gps->motor: 항상 전송 (health 플래그로 유효성 표시)
        # 포맷 (8 fields): lat,lon,pos_health,pos_ts,course_deg,speed_mps,motion_health,motion_ts
        #   pos_health=0  → lat/lon/pos_ts = nan
        #   motion_health=0 → course/speed/motion_ts = nan
        if rcv_data and len(rcv_data) >= 5:
            now_mono = time.monotonic()
            hdop = float(rcv_data[10]) if len(rcv_data) > 10 and _is_finite(rcv_data[10]) else float('inf')

            pos_fresh = _valid_age(last_valid_gps_ts, now_mono, GPS_STALE_TIMEOUT_SEC)
            motion_fresh = _valid_age(last_valid_rmc_ts, now_mono, GPS_STALE_TIMEOUT_SEC)
            if sim_null:
                pos_health = 0
                motion_health = 0
                GPS_SATS = 0  # GCS fallback estimator도 location stale로 인식
            elif sim_sample is not None:
                pos_health = pos_fresh
                motion_health = motion_fresh and _eval_motion_fidelity(pos_health, GPS_RMC_STATUS, GPS_SPEED_MS, GPS_COURSE)
            else:
                pos_health = pos_fresh and _eval_pos_fidelity(GPS_LAT, GPS_LON, hdop, GPS_SATS, GPS_FIX_QUALITY, now_mono)
                motion_health = motion_fresh and _eval_motion_fidelity(pos_health, GPS_RMC_STATUS, GPS_SPEED_MS, GPS_COURSE)

            if pos_health:
                global _prev_valid_lat, _prev_valid_lon, _prev_valid_ts
                _prev_valid_lat = GPS_LAT
                _prev_valid_lon = GPS_LON
                _prev_valid_ts  = now_mono

            lat_s = f"{GPS_LAT:.7f}"          if pos_health else "nan"
            lon_s = f"{GPS_LON:.7f}"          if pos_health else "nan"
            pts_s = f"{last_valid_gps_ts:.4f}" if pos_health else "nan"
            crs_s = f"{GPS_COURSE:.4f}"        if motion_health else "nan"
            spd_s = f"{GPS_SPEED_MS:.4f}"      if motion_health else "nan"
            mts_s = f"{last_valid_rmc_ts:.4f}" if motion_health else "nan"

            msgstructure.send_msg(
                Main_Queue,
                appargs.GpsAppArg.AppID, appargs.MotorAppArg.AppID,
                appargs.GpsAppArg.MID_motor_gps,
                f"{lat_s},{lon_s},{int(pos_health)},{pts_s},{crs_s},{spd_s},{int(motion_health)},{mts_s}"
            )

        send_counter += 1
        if send_counter >= 10:
            # Send telemetry message to COMM app
            # 이제 변수들이 0으로 리셋되지 않으므로 마지막 유효 좌표가 전송됨

            status = msgstructure.send_msg(
                Main_Queue,
                appargs.GpsAppArg.AppID,
                appargs.CommAppArg.AppID,
                appargs.GpsAppArg.MID_comm_gga,
                f"{GPS_TIME},{GPS_ALT},{GPS_LAT if GPS_LAT is not None else 0.0},{GPS_LON if GPS_LON is not None else 0.0},{GPS_SATS}"
            )
            if status == False:
                events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.error, "Error When sending GPS Tlm Message")
            # GPS TLM 전송 성공 로그 비활성화
            send_counter = 0

        # GPS read rate: 10Hz (reduced from 25Hz to avoid I2C bus conflicts with other sensors)
        # read_gps() can take up to 1 second, so reading at 25Hz was causing overlapping calls
        time.sleep(0.1)

######################################################
## MAIN METHOD                                      ##
######################################################

thread_dict = dict[str, threading.Thread]()

# This method is called from main app. Initialization, runloop process
def gpsapp_main(Main_Queue : Queue, Main_Pipe : connection.Connection):
    global GPSAPP_RUNSTATUS
    GPSAPP_RUNSTATUS = True

    # Initialization Process
    gps_instance = gpsapp_init()
    
    # Check if initialization failed
    if gps_instance is None:
        events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.error, "GPS initialization failed, terminating gpsapp")
        GPSAPP_RUNSTATUS = False
        return

    # Spawn SB Message Listner Thread
    thread_dict["ReadAndSendGpsData_Thread"] = threading.Thread(target=read_and_send_gps_data, args=(Main_Queue, gps_instance,  ), name="ReadAndSendGpsData_Thread")

    # Spawn Each Threads
    for thread_name in thread_dict:
        thread_dict[thread_name].start()

    try:
        while GPSAPP_RUNSTATUS:
            # Receive Message From Pipe
            raw = Main_Pipe.recv()
            unpacked_msg = msgstructure.unpack_msg(raw)
            # Unpack Message, Skip this message if unpacked message is not valid
            if  unpacked_msg == False:
                continue
            
            # Validate Message, Skip this message if target AppID different from gpsapp's AppID
            # Exception when the message is from main app
            if unpacked_msg.receiver_app == appargs.GpsAppArg.AppID or unpacked_msg.receiver_app == appargs.MainAppArg.AppID:
                # Handle Command According to Message ID
                command_handler(unpacked_msg)
            else:
                events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.error, "Receiver MID does not match with gpsapp MID")

    # If error occurs, terminate app
    except Exception as e:
        events.LogEvent(appargs.GpsAppArg.AppName, events.EventType.error, f"gpsapp error : {e}")
        GPSAPP_RUNSTATUS = False

    # Termination Process after runloop
    gpsapp_terminate()

    return
