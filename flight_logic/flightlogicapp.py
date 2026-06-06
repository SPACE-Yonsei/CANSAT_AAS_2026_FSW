"""Flight state machine and mission event dispatch."""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import Optional

from lib import appargs, config, msgstructure, prevstate
from Sensor_Motor.Motor_Release_Cal import (
    ReleaseDecision,
    ReleasePredictorState,
    reset_release_predictor,
    should_trigger_release,
)


FLIGHTLOGIC_RUNSTATUS = True
logger = logging.getLogger(__name__)
state = 0
max_alt = 0.0
recent_alt: list[float] = []
distance_mm = 99999.0
distance_health = 0
sim_enable = False
sim_active = False

# 발사 감지 기준선: CAL 직후 첫 (보정된) 고도값을 저장해
# abs 30m가 아닌 "기준선 대비 +100m" 상승을 ASCENT 조건으로 사용.
# CAL 후 고도가 ~0m로 리셋되면 None으로 초기화 → 재샘플링.
_ascent_baseline_alt: Optional[float] = None

# 발사 전 CAL 강제 게이트: CAL 명령(MID_flight_alt_reset)을 받기 전에는
# state 0→1(ASCENT) 자동 전이를 잠근다. 고지대 발사장에서 미보정 raw 해발고도
# (~300m)가 baseline 오염과 맞물려 조기 ASCENT/APOGEE 전이를 일으키던 문제 방지.
# 수동 SS 명령(force)은 이 게이트를 우회한다.
_cal_done: bool = False

# Solenoid lower bound: reject "sensor dead" 0 mm; slight slack under TF-Luna min valid (200 mm).
SOLENOID_MIN_MM = 100.0


def _egg_distance_trigger_max_mm() -> float:
    """Upper range bound (mm) for egg-drop distance gate; see ``config.EGG_STATE_DISTANCE_TRIGGER_MM``."""
    try:
        mm = float(getattr(config, "EGG_STATE_DISTANCE_TRIGGER_MM", 2500))
    except (TypeError, ValueError):
        mm = 2500.0
    return max(SOLENOID_MIN_MM, mm)

cnt_ascent = 0
cnt_apogee = 0
cnt_release = 0
cnt_landed = 0
cnt_egg_drop = 0
egg_drop_sent = 0           # 에그 드롭 명령 송신 횟수 (state 5 진입마다 0으로 재무장)
EGG_DROP_MAX_SENDS = 2      # 솔레노이드 깜빡임 방지: 명령은 최대 2번만 송신
solenoid_count = 0
solenoid_done = False
release_predictor = ReleasePredictorState()
release_reason = "TRIGGER"


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in meters."""
    r = 6_371_000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = (
        math.sin(dphi / 2.0) ** 2
        + math.cos(p1) * math.cos(p2) * math.sin(dlmb / 2.0) ** 2
    )
    return 2.0 * r * math.asin(min(1.0, math.sqrt(a)))


def _set_state(main_queue, new_state: int, force: bool = False) -> None:
    global state
    if not force and state == new_state:
        return
    state = new_state
    prevstate.update_prevstate(state)
    msgstructure.send_msg(
        main_queue,
        appargs.FlightlogicAppArg.AppID,
        appargs.MotorAppArg.AppID,
        appargs.FlightlogicAppArg.MID_motor_state,
        str(state),
    )
    msgstructure.send_msg(
        main_queue,
        appargs.FlightlogicAppArg.AppID,
        appargs.CommAppArg.AppID,
        appargs.FlightlogicAppArg.MID_comm_state,
        str(state),
    )


def to_launch_pad(main_queue, force: bool = False) -> None:
    global max_alt, solenoid_count, solenoid_done, _cal_done, _ascent_baseline_alt, egg_drop_sent
    max_alt = 0.0
    prevstate.update_maxalt(max_alt)
    egg_drop_sent = 0
    solenoid_count = 0
    solenoid_done = False
    prevstate.update_solenoid_state(solenoid_count, solenoid_done)
    reset_release_predictor(release_predictor)
    # 발사대 복귀(재무장) → 발사 전 CAL 재요구
    _cal_done = False
    _ascent_baseline_alt = None
    _set_state(main_queue, 0, force=force)


def to_ascent(main_queue, force: bool = False) -> None:
    _set_state(main_queue, 1, force=force)
    msgstructure.send_msg(
        main_queue,
        appargs.FlightlogicAppArg.AppID,
        appargs.CameraAppArg.AppID,
        appargs.CameraAppArg.MID_cam_activate,
        "ON",
    )


def to_apogee(main_queue, force: bool = False) -> None:
    _set_state(main_queue, 2, force=force)


def to_descent(main_queue, force: bool = False) -> None:
    _set_state(main_queue, 3, force=force)


def _has_release_target() -> bool:
    lat = prevstate.PREV_TARGET_LAT
    lon = prevstate.PREV_TARGET_LON
    if not (math.isfinite(lat) and math.isfinite(lon)):
        return False
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return False
    return not (lat == 0.0 and lon == 0.0)


def to_release(main_queue, force: bool = False, reason: str = "TRIGGER") -> None:
    if not _has_release_target():
        logger.error("Release blocked: target coordinate must be set before release")
        return
    _set_state(main_queue, 4, force=force)   # PAYLOAD_RELEASE (번와이어)
    burnwire_payload = f"TRIGGER:{reason}"
    msgstructure.send_msg(
        main_queue,
        appargs.FlightlogicAppArg.AppID,
        appargs.MotorAppArg.AppID,
        appargs.FlightlogicAppArg.MID_motor_burnwire,
        burnwire_payload,
    )
    logger.info("Release command sent | reason=%s", reason)
    msgstructure.send_msg(
        main_queue,
        appargs.FlightlogicAppArg.AppID,
        appargs.MotorAppArg.AppID,
        appargs.FlightlogicAppArg.MID_motor_TargetCor,
        f"{prevstate.PREV_TARGET_LAT},{prevstate.PREV_TARGET_LON}",
    )


def to_egg(main_queue, force: bool = False) -> None:
    global egg_drop_sent
    egg_drop_sent = 0   # state 5 진입 시 에그 드롭 명령 카운터 재무장
    _set_state(main_queue, 5, force=force)   # PROBE_RELEASE (솔레노이드, 에그 2m)
    # SS,4 can skip SS,3; refresh motor target from prevstate so guidance is not TARGET_UNSET.
    if _has_release_target():
        msgstructure.send_msg(
            main_queue,
            appargs.FlightlogicAppArg.AppID,
            appargs.MotorAppArg.AppID,
            appargs.FlightlogicAppArg.MID_motor_TargetCor,
            f"{prevstate.PREV_TARGET_LAT},{prevstate.PREV_TARGET_LON}",
        )


def _send_egg_drop(main_queue) -> None:
    global egg_drop_sent
    if egg_drop_sent >= EGG_DROP_MAX_SENDS:
        return
    egg_drop_sent += 1
    logger.info("Egg drop triggered (%d/%d)", egg_drop_sent, EGG_DROP_MAX_SENDS)
    msgstructure.send_msg(
        main_queue,
        appargs.FlightlogicAppArg.AppID,
        appargs.MotorAppArg.AppID,
        appargs.FlightlogicAppArg.MID_motor_EggDrop,
        "",
    )


def to_landed(main_queue, force: bool = False) -> None:
    _set_state(main_queue, 6, force=force)


def _verify_inter_app_links(main_queue) -> None:
    """Best-effort non-actuating link audit for SIM prepare mode.

    There is no ACK channel for every app, so this routine validates routing
    by dispatching only benign frames that must not trigger actuators.
    """
    # Keep comm mode in "A" and attach audit marker in payload tail.
    msgstructure.send_msg(
        main_queue,
        appargs.FlightlogicAppArg.AppID,
        appargs.CommAppArg.AppID,
        appargs.FlightlogicAppArg.MID_comm_sim,
        "A,LINKCHK",
    )
    msgstructure.send_msg(
        main_queue,
        appargs.FlightlogicAppArg.AppID,
        appargs.CommAppArg.AppID,
        appargs.FlightlogicAppArg.MID_comm_state,
        str(state),
    )

    # Motor receives only state/target refresh (no burnwire/egg trigger).
    msgstructure.send_msg(
        main_queue,
        appargs.FlightlogicAppArg.AppID,
        appargs.MotorAppArg.AppID,
        appargs.FlightlogicAppArg.MID_motor_state,
        str(state),
    )
    msgstructure.send_msg(
        main_queue,
        appargs.FlightlogicAppArg.AppID,
        appargs.MotorAppArg.AppID,
        appargs.FlightlogicAppArg.MID_motor_TargetCor,
        f"{prevstate.PREV_TARGET_LAT},{prevstate.PREV_TARGET_LON}",
    )

    # Camera OFF is a safe no-op for routing/path check.
    msgstructure.send_msg(
        main_queue,
        appargs.FlightlogicAppArg.AppID,
        appargs.CameraAppArg.AppID,
        appargs.CommAppArg.MID_RouteCmd_CAM,
        "OFF",
    )
    logger.info("SIM A link check frames dispatched")


def handle_sim(data: str, main_queue) -> None:
    global sim_enable, sim_active
    option = data.strip().upper()
    if option == "ENABLE":
        sim_enable = True
        sim_active = False
        msgstructure.send_msg(
            main_queue,
            appargs.FlightlogicAppArg.AppID,
            appargs.GpsAppArg.AppID,
            appargs.GpsAppArg.MID_flight_gps_sim,
            "CLEAR",
        )
        msgstructure.send_msg(
            main_queue,
            appargs.FlightlogicAppArg.AppID,
            appargs.CommAppArg.AppID,
            appargs.FlightlogicAppArg.MID_comm_sim,
            "A",
        )
    elif option == "ACTIVATE":
        if sim_enable:
            sim_active = True
            msgstructure.send_msg(
                main_queue,
                appargs.FlightlogicAppArg.AppID,
                appargs.CommAppArg.AppID,
                appargs.FlightlogicAppArg.MID_comm_sim,
                "S",
            )
    elif option == "DISABLE":
        sim_enable = False
        sim_active = False
        msgstructure.send_msg(
            main_queue,
            appargs.FlightlogicAppArg.AppID,
            appargs.GpsAppArg.AppID,
            appargs.GpsAppArg.MID_flight_gps_sim,
            "CLEAR",
        )
        msgstructure.send_msg(
            main_queue,
            appargs.FlightlogicAppArg.AppID,
            appargs.CommAppArg.AppID,
            appargs.FlightlogicAppArg.MID_comm_sim,
            "F",
        )


def handle_simg(data: str, main_queue) -> None:
    """SIM GPS inject: lat,lon,course_deg,speed_m_s[,alt_m] — requires SIM ACTIVATE."""
    if not sim_active:
        logger.warning("SIMG ignored: SIM ACTIVATE required first")
        return
    parts = [x.strip() for x in data.split(",") if x.strip() != ""]
    if len(parts) not in (4, 5):
        logger.warning("SIMG: expected lat,lon,course_deg,speed_m_s[,alt_m] got %r", data)
        return
    try:
        lat = float(parts[0])
        lon = float(parts[1])
        course = float(parts[2])
        speed = float(parts[3])
        if len(parts) == 5:
            alt = float(parts[4])
        else:
            alt = 80.0
    except ValueError:
        logger.warning("SIMG: non-numeric fields %r", data)
        return
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return
    if lat == 0.0 and lon == 0.0:
        return
    payload = f"{lat},{lon},{course},{speed},{alt}"
    msgstructure.send_msg(
        main_queue,
        appargs.FlightlogicAppArg.AppID,
        appargs.GpsAppArg.AppID,
        appargs.GpsAppArg.MID_flight_gps_sim,
        payload,
    )

    # In SIM mode there is no real TF-Luna stream. Populate Comm distance field
    # with target distance (mm) so GCS "Distance" no longer stays fixed at 0.
    if _has_release_target():
        target_lat = float(prevstate.PREV_TARGET_LAT)
        target_lon = float(prevstate.PREV_TARGET_LON)
        dist_mm = _haversine_m(lat, lon, target_lat, target_lon) * 1000.0
        msgstructure.send_msg(
            main_queue,
            appargs.FlightlogicAppArg.AppID,
            appargs.CommAppArg.AppID,
            appargs.FlightlogicAppArg.MID_comm_nav_dis,
            f"{dist_mm:.1f}",
        )


def handle_simgn(data: str, main_queue) -> None:
    """SIMGN: GPS를 즉시 stale로 만든다 (pos_health=0 강제). SIM ACTIVATE 필요."""
    if not sim_active:
        logger.warning("SIMGN ignored: SIM ACTIVATE required first")
        return
    msgstructure.send_msg(
        main_queue,
        appargs.FlightlogicAppArg.AppID,
        appargs.GpsAppArg.AppID,
        appargs.GpsAppArg.MID_flight_gps_null,
        "",
    )


def handle_simgr(data: str, main_queue) -> None:
    """SIM GPS Relative: east_m,north_m,course_deg,speed_m_s[,alt_m] — offset from target origin."""
    if not sim_active:
        logger.warning("SIMGR ignored: SIM ACTIVATE required first")
        return
    if not _has_release_target():
        logger.warning("SIMGR ignored: TC (target coordinate) must be set first")
        return
    parts = [x.strip() for x in data.split(",") if x.strip() != ""]
    if len(parts) not in (4, 5):
        logger.warning("SIMGR: expected east_m,north_m,course_deg,speed_m_s[,alt_m] got %r", data)
        return
    try:
        east_m  = float(parts[0])
        north_m = float(parts[1])
        course  = float(parts[2])
        speed   = float(parts[3])
        alt     = float(parts[4]) if len(parts) == 5 else 80.0
    except ValueError:
        logger.warning("SIMGR: non-numeric fields %r", data)
        return

    ref_lat = float(prevstate.PREV_TARGET_LAT)
    ref_lon = float(prevstate.PREV_TARGET_LON)
    lat = ref_lat + north_m / 111111.0
    lon = ref_lon + east_m / (111111.0 * math.cos(math.radians(ref_lat)))
    lat = max(-90.0, min(90.0, lat))
    lon = ((lon + 180.0) % 360.0) - 180.0

    simg_data = f"{lat:.7f},{lon:.7f},{course},{speed},{alt}"
    handle_simg(simg_data, main_queue)


def handle_simp(data: str, main_queue) -> None:
    if not sim_active:
        return
    try:
        alt = float(data)
    except ValueError:
        return
    barometer_logic(main_queue, alt)


def handle_barometer(data: str, main_queue) -> None:
    if sim_active:
        return
    fields = data.split(",")
    try:
        alt = float(fields[0])
    except (ValueError, IndexError):
        return
    health = 1
    if len(fields) >= 2:
        try:
            health = int(float(fields[1]))
        except ValueError:
            health = 1
    if not health:
        # Stale/invalid altitude: do not advance state machine on garbage.
        return
    barometer_logic(main_queue, alt)


def handle_distance(data: str, main_queue) -> None:
    global distance_mm, distance_health
    fields = data.split(",")
    try:
        distance_mm = float(fields[0])
    except (ValueError, IndexError):
        return
    if len(fields) >= 2:
        try:
            distance_health = int(float(fields[1]))
        except ValueError:
            distance_health = 0
    else:
        distance_health = 1
    if state == 5 and SOLENOID_MIN_MM < distance_mm <= 3000.0:   # PROBE_RELEASE
        _send_egg_drop(main_queue)


def handle_ss(data: str, main_queue) -> None:
    try:
        target = int(data)
    except ValueError:
        return
    # 0=LAUNCH_PAD 1=ASCENT 2=APOGEE 3=DESCENT
    # 4=PAYLOAD_RELEASE(번와이어) 5=PROBE_RELEASE(솔레노이드) 6=LANDED
    if target < 0 or target > 6:
        return
    if target == 0:
        to_launch_pad(main_queue, force=True)
    elif target == 1:
        to_ascent(main_queue, force=True)
    elif target == 2:
        to_apogee(main_queue, force=True)
    elif target == 3:
        to_descent(main_queue, force=True)
    elif target == 4:
        to_release(main_queue, force=True, reason="SS_FORCE")
    elif target == 5:
        to_egg(main_queue, force=True)
    elif target == 6:
        to_landed(main_queue, force=True)


def handle_target_coord(data: str, main_queue) -> None:
    fields = [x.strip() for x in data.split(",")]
    if len(fields) != 2:
        return
    try:
        lat = float(fields[0])
        lon = float(fields[1])
    except ValueError:
        return
    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
        return
    if lat == 0.0 and lon == 0.0:
        logger.warning("Rejected zero target coordinate for safety policy")
        return
    if getattr(prevstate, "FIX_TARGET_GPS", False):
        fixed_lat, fixed_lon = prevstate.get_target_gps()
        logger.info(
            "Ignoring target command because prevstate target is fixed: lat=%.6f lon=%.6f",
            fixed_lat,
            fixed_lon,
        )
        return
    prevstate.update_target_gps(lat, lon)
    msgstructure.send_msg(
        main_queue,
        appargs.FlightlogicAppArg.AppID,
        appargs.MotorAppArg.AppID,
        appargs.FlightlogicAppArg.MID_motor_TargetCor,
        f"{lat},{lon}",
    )


def handle_reset_alt(_data: str, _main_queue) -> None:
    global max_alt, recent_alt, _ascent_baseline_alt, _cal_done
    max_alt = 0.0
    prevstate.update_maxalt(max_alt)
    recent_alt = []
    _ascent_baseline_alt = None   # CAL 후 재샘플링 → 새 기준선 확립
    _cal_done = True              # CAL 완료 → ASCENT 자동 전이 허용
    reset_release_predictor(release_predictor)



def _reset_transition_counters() -> None:
    global cnt_ascent, cnt_apogee, cnt_release, cnt_landed, cnt_egg_drop
    cnt_ascent = 0
    cnt_apogee = 0
    cnt_release = 0
    cnt_landed = 0
    cnt_egg_drop = 0


def _release_condition(alt: float, now_s: float) -> ReleaseDecision:
    if max_alt <= 0:
        return ReleaseDecision(False)
    return should_trigger_release(
        release_predictor,
        now_s=now_s,
        alt_m=alt,
        max_alt_m=max_alt,
    )


def barometer_logic(main_queue, alt: float) -> None:
    global max_alt, recent_alt, cnt_ascent, cnt_apogee, cnt_release, cnt_landed, cnt_egg_drop, release_reason
    global solenoid_count, solenoid_done
    now_s = time.time()
    filtered_alt = alt
    prev_max_alt = max_alt
    recent_alt.append(alt)
    if len(recent_alt) > 3:
        recent_alt = recent_alt[-3:]

    if len(recent_alt) >= 2:
        sorted_win = sorted(recent_alt, reverse=True)
        candidate = sorted_win[1] if len(sorted_win) > 1 else sorted_win[0]
        filtered_alt = candidate
        max_alt = max(max_alt, candidate)
    else:
        max_alt = max(max_alt, alt)

    if math.isfinite(max_alt) and max_alt > prev_max_alt:
        prevstate.update_maxalt(max_alt)

    if state == 0:
        global _ascent_baseline_alt
        if not _cal_done:
            # CAL 전: ASCENT 전이 잠금. baseline도 확정하지 않아 워밍업 오염을 피한다.
            return
        if _ascent_baseline_alt is None:
            _ascent_baseline_alt = alt   # CAL 후 첫 (보정된) 샘플을 기준선으로
        risen = alt - _ascent_baseline_alt
        cnt_ascent = cnt_ascent + 1 if risen > 100 else 0
        if cnt_ascent >= 3:
            _reset_transition_counters()
            reset_release_predictor(release_predictor)
            to_ascent(main_queue)
    elif state == 1:
        rel_decision = _release_condition(filtered_alt, now_s)
        rel_cond = rel_decision.trigger
        if rel_cond:
            release_reason = rel_decision.reason
        apo_cond = max_alt > 0 and (max_alt * 0.8 < alt < max_alt - 0.25)
        cnt_release = cnt_release + 1 if rel_cond else 0
        cnt_apogee = cnt_apogee + 1 if apo_cond else 0
        if cnt_release >= 3:
            _reset_transition_counters()
            to_release(main_queue, reason=release_reason)
        elif cnt_apogee >= 2:
            _reset_transition_counters()
            to_apogee(main_queue)
    elif state == 2:   # APOGEE → DESCENT 자동 전환 (TLM에 APOGEE 확보 후)
        cnt_apogee = cnt_apogee + 1
        if cnt_apogee >= 15:   # ~1.5s: TLM 1~2 패킷 확보 후 DESCENT 진입
            _reset_transition_counters()
            to_descent(main_queue)
    elif state == 3:   # DESCENT: 릴리즈 조건 감시
        rel_decision = _release_condition(filtered_alt, now_s)
        if rel_decision.trigger:
            release_reason = rel_decision.reason
        cnt_release = cnt_release + 1 if rel_decision.trigger else 0
        if cnt_release >= 3:
            _reset_transition_counters()
            to_release(main_queue, reason=release_reason)
    elif state == 4:   # PAYLOAD_RELEASE (번와이어): 에그 방출 고도 감시
        cnt_release = cnt_release + 1 if alt <= 50 else 0
        if cnt_release >= 3:
            _reset_transition_counters()
            to_egg(main_queue)
    elif state == 5:   # PROBE_RELEASE (솔레노이드, ~2m): 착지 감시
        cnt_egg_drop = cnt_egg_drop + 1 if alt <= 5 else 0
        if cnt_egg_drop >= 3:
            _send_egg_drop(main_queue)
        cnt_landed = cnt_landed + 1 if alt <= 10 else 0
        if cnt_landed >= 100:
            _reset_transition_counters()
            to_landed(main_queue)


def dispatch(msg: str, main_queue) -> None:
    unpacked = msgstructure.unpack_msg(msg)
    if unpacked is False:
        return

    mid = unpacked.msg_id
    if mid == appargs.MainAppArg.MID_TerminateProcess:
        global FLIGHTLOGIC_RUNSTATUS
        FLIGHTLOGIC_RUNSTATUS = False
        return
    if mid == appargs.CommAppArg.MID_RouteCmd_SIM:
        handle_sim(unpacked.data, main_queue)
    elif mid == appargs.CommAppArg.MID_RouteCmd_SIMP:
        handle_simp(unpacked.data, main_queue)
    elif mid == appargs.CommAppArg.MID_RouteCmd_SIMG:
        handle_simg(unpacked.data, main_queue)
    elif mid == appargs.CommAppArg.MID_RouteCmd_SIMGR:
        handle_simgr(unpacked.data, main_queue)
    elif mid == appargs.CommAppArg.MID_RouteCmd_SIMGN:
        handle_simgn(unpacked.data, main_queue)
    elif mid == appargs.BarometerAppArg.MID_flight_alt:
        handle_barometer(unpacked.data, main_queue)
    elif mid == appargs.DistanceAppArg.MID_flight_dis:
        handle_distance(unpacked.data, main_queue)
    elif mid == appargs.CommAppArg.MID_RouteCmd_SS:
        handle_ss(unpacked.data, main_queue)
    elif mid == appargs.CommAppArg.MID_RouteCmd_TC:
        handle_target_coord(unpacked.data, main_queue)
    elif mid == appargs.BarometerAppArg.MID_flight_alt_reset:
        handle_reset_alt(unpacked.data, main_queue)


def send_current_state_thread(main_queue) -> None:
    while FLIGHTLOGIC_RUNSTATUS:
        msgstructure.send_msg(
            main_queue,
            appargs.FlightlogicAppArg.AppID,
            appargs.CommAppArg.AppID,
            appargs.FlightlogicAppArg.MID_comm_state,
            str(state),
        )
        time.sleep(1.0)


def init() -> None:
    global state, max_alt, solenoid_count, solenoid_done, _cal_done
    prevstate.init_prevstate()
    state = prevstate.PREV_STATE
    max_alt = prevstate.PREV_MAX_ALT
    solenoid_count, solenoid_done = prevstate.get_solenoid_state()
    # 지상(state 0) 부팅이면 CAL 전까지 ASCENT 잠금. 비행 중 재부팅(state>0)이면
    # barometerapp이 PREV_ALT_CAL을 복원하므로 이미 보정된 것으로 간주.
    _cal_done = state > 0


def flightlogicapp_main(main_queue, main_pipe) -> None:
    init()
    state_thread = threading.Thread(
        target=send_current_state_thread,
        args=(main_queue,),
        daemon=True,
        name="FlightStateBroadcast",
    )
    state_thread.start()

    try:
        while FLIGHTLOGIC_RUNSTATUS:
            try:
                has_msg = main_pipe.poll(0.1)
            except (KeyboardInterrupt, EOFError, OSError):
                break
            if has_msg:
                msg = main_pipe.recv()
                dispatch(msg, main_queue)
    except KeyboardInterrupt:
        pass
