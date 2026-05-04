"""Flight state machine and mission event dispatch."""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional

from lib import appargs, msgstructure, prevstate


FLIGHTLOGIC_RUNSTATUS = True
logger = logging.getLogger(__name__)
state = 0
max_alt = 0.0
recent_alt: list[float] = []
distance_mm = 99999.0
sim_enable = False
sim_active = False

cnt_ascent = 0
cnt_apogee = 0
cnt_release = 0
cnt_landed = 0
cnt_egg_drop = 0
solenoid_count = 0
solenoid_done = False


def _send(main_queue, receiver: int, msg_id: int, data: str) -> None:
    msgstructure.send_msg(
        main_queue,
        appargs.FlightlogicAppArg.AppID,
        receiver,
        msg_id,
        data,
    )


def _set_state(main_queue, new_state: int, force: bool = False) -> None:
    global state
    if not force and state == new_state:
        return
    state = new_state
    prevstate.update_prevstate(state)
    _send(main_queue, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_motor_state, str(state))


def to_launch_pad(main_queue, force: bool = False) -> None:
    global max_alt
    max_alt = 0.0
    _set_state(main_queue, 0, force=force)


def to_ascent(main_queue, force: bool = False) -> None:
    _set_state(main_queue, 1, force=force)
    _send(main_queue, appargs.CameraAppArg.AppID, appargs.CameraAppArg.MID_cam_activate, "ON")


def to_apogee(main_queue, force: bool = False) -> None:
    _set_state(main_queue, 2, force=force)


def to_release(main_queue, force: bool = False) -> None:
    _set_state(main_queue, 3, force=force)
    _send(main_queue, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_motor_burnwire, "TRIGGER")
    _send(
        main_queue,
        appargs.MotorAppArg.AppID,
        appargs.FlightlogicAppArg.MID_motor_TargetCor,
        f"{prevstate.Target_lat},{prevstate.Target_lon}",
    )


def to_egg(main_queue, force: bool = False) -> None:
    _set_state(main_queue, 4, force=force)


def to_landed(main_queue, force: bool = False) -> None:
    _set_state(main_queue, 5, force=force)


def _verify_inter_app_links(main_queue) -> None:
    """Best-effort non-actuating link audit for SIM prepare mode.

    There is no ACK channel for every app, so this routine validates routing
    by dispatching only benign frames that must not trigger actuators.
    """
    # Keep comm mode in "A" and attach audit marker in payload tail.
    _send(main_queue, appargs.CommAppArg.AppID, appargs.FlightlogicAppArg.MID_comm_sim, "A,LINKCHK")
    _send(main_queue, appargs.CommAppArg.AppID, appargs.FlightlogicAppArg.MID_comm_state, str(state))

    # Motor receives only state/target refresh (no burnwire/egg trigger).
    _send(main_queue, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_motor_state, str(state))
    _send(
        main_queue,
        appargs.MotorAppArg.AppID,
        appargs.FlightlogicAppArg.MID_motor_TargetCor,
        f"{prevstate.Target_lat},{prevstate.Target_lon}",
    )

    # Camera OFF is a safe no-op for routing/path check.
    _send(main_queue, appargs.CameraAppArg.AppID, appargs.CommAppArg.MID_RouteCmd_CAM, "OFF")
    logger.info("SIM A link check frames dispatched")


def handle_sim(data: str, main_queue) -> None:
    global sim_enable, sim_active
    option = data.strip().upper()
    if option == "ENABLE":
        sim_enable = True
        sim_active = False
        _verify_inter_app_links(main_queue)
    elif option == "ACTIVATE":
        if sim_enable:
            sim_active = True
            _send(main_queue, appargs.CommAppArg.AppID, appargs.FlightlogicAppArg.MID_comm_sim, "S")
    elif option == "DISABLE":
        sim_enable = False
        sim_active = False
        _send(main_queue, appargs.CommAppArg.AppID, appargs.FlightlogicAppArg.MID_comm_sim, "F")


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
    try:
        alt = float(data.split(",")[0])
    except ValueError:
        return
    barometer_logic(main_queue, alt)


def handle_distance(data: str, main_queue) -> None:
    global distance_mm
    try:
        distance_mm = float(data.split(",")[0])
    except ValueError:
        return
    if state == 4:
        solenoid_logic(main_queue, distance_mm)


def handle_ss(data: str, main_queue) -> None:
    try:
        target = int(data)
    except ValueError:
        return
    if target < 0 or target > 5:
        return
    if target == 0:
        to_launch_pad(main_queue, force=True)
    elif target == 1:
        to_ascent(main_queue, force=True)
    elif target == 2:
        to_apogee(main_queue, force=True)
    elif target == 3:
        to_release(main_queue, force=True)
    elif target == 4:
        to_egg(main_queue, force=True)
    elif target == 5:
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
    prevstate.update_target_gps(lat, lon)
    _send(main_queue, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_motor_TargetCor, f"{lat},{lon}")


def handle_reset_alt(_data: str, _main_queue) -> None:
    global max_alt, recent_alt
    max_alt = 0.0
    recent_alt = []


def solenoid_logic(main_queue, distance: float) -> None:
    global solenoid_count, solenoid_done
    if solenoid_done:
        return
    if distance <= 2500 and solenoid_count < 3:
        _send(main_queue, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_motor_EggDrop, "TRIGGER")
        solenoid_count += 1
    if solenoid_count >= 3:
        solenoid_done = True


def _reset_transition_counters() -> None:
    global cnt_ascent, cnt_apogee, cnt_release, cnt_landed
    cnt_ascent = 0
    cnt_apogee = 0
    cnt_release = 0
    cnt_landed = 0


def barometer_logic(main_queue, alt: float) -> None:
    global max_alt, recent_alt, cnt_ascent, cnt_apogee, cnt_release, cnt_landed, cnt_egg_drop
    recent_alt.append(alt)
    if len(recent_alt) > 3:
        recent_alt = recent_alt[-3:]

    if len(recent_alt) >= 2:
        sorted_win = sorted(recent_alt, reverse=True)
        candidate = sorted_win[1] if len(sorted_win) > 1 else sorted_win[0]
        max_alt = max(max_alt, candidate)
    else:
        max_alt = max(max_alt, alt)

    if state == 0:
        cnt_ascent = cnt_ascent + 1 if alt > 200 else 0
        if cnt_ascent >= 3:
            _reset_transition_counters()
            to_ascent(main_queue)
    elif state == 1:
        rel_cond = max_alt > 0 and alt <= max_alt * 0.8
        apo_cond = max_alt > 0 and (max_alt * 0.8 < alt < max_alt - 0.25)
        cnt_release = cnt_release + 1 if rel_cond else 0
        cnt_apogee = cnt_apogee + 1 if apo_cond else 0
        if cnt_release >= 3:
            _reset_transition_counters()
            to_release(main_queue)
        elif cnt_apogee >= 2:
            _reset_transition_counters()
            to_apogee(main_queue)
    elif state == 2:
        cnt_release = cnt_release + 1 if (max_alt > 0 and alt <= max_alt * 0.8) else 0
        if cnt_release >= 3:
            _reset_transition_counters()
            to_release(main_queue)
    elif state == 3:
        cnt_release = cnt_release + 1 if alt <= 50 else 0
        if cnt_release >= 3:
            _reset_transition_counters()
            to_egg(main_queue)
    elif state == 4:
        cnt_egg_drop = cnt_egg_drop + 1 if alt <= 4 else 0
        if cnt_egg_drop >= 2:
            _send(main_queue, appargs.MotorAppArg.AppID, appargs.FlightlogicAppArg.MID_motor_EggDrop, "TRIGGER")
            cnt_egg_drop = 0
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
    elif mid == appargs.BarometerAppArg.MID_flight_alt:
        handle_barometer(unpacked.data, main_queue)
    elif mid == appargs.DistanceAppArg.MID_flight_dis:
        handle_distance(unpacked.data, main_queue)
    elif mid == appargs.CommAppArg.MID_RouteCmd_SS:
        handle_ss(unpacked.data, main_queue)
    elif mid == appargs.CommAppArg.MID_RouteCmd_TC:
        handle_target_coord(unpacked.data, main_queue)
    elif mid == appargs.CommAppArg.MID_RouteCmd_CAL:
        handle_reset_alt(unpacked.data, main_queue)


def send_current_state_thread(main_queue) -> None:
    while FLIGHTLOGIC_RUNSTATUS:
        _send(main_queue, appargs.CommAppArg.AppID, appargs.FlightlogicAppArg.MID_comm_state, str(state))
        time.sleep(1.0)


def init() -> None:
    global state, max_alt
    prevstate.init_prevstate()
    state = prevstate.PREV_STATE
    max_alt = prevstate.PREV_MAX_ALT


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
