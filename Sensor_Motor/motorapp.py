"""Motor app: data ingestion, FDIR, and control loop."""

from __future__ import annotations

import math
import threading
import time
from types import SimpleNamespace

from lib import appargs, msgstructure
from Sensor_Motor import Motor_Egg, Motor_Release, motor_control, motor_guidance


MOTORAPP_RUNSTATUS = True
motor_enabled = True
state = 0
pi = None

update_lock = threading.Lock()
sensor = SimpleNamespace(
    yaw=0.0,
    gyrz=0.0,
    imu_health=1,
    lat=0.0,
    lon=0.0,
    speed=0.0,
    course=0.0,
    fix_quality=0,
    sats=0,
    rmc_status="V",
    gps_health=0,
    baro_m=0.0,
)
target = SimpleNamespace(lat=0.0, lon=0.0)
last_gps_update = 0.0
GPS_STALE_TIMEOUT = 10.0


def _send(main_queue, msg_id: int, data: str) -> None:
    msgstructure.send_msg(main_queue, appargs.MotorAppArg.AppID, appargs.CommAppArg.AppID, msg_id, data)


def handle_gps(data: str) -> None:
    global last_gps_update
    fields = [x.strip() for x in data.split(",")]
    if len(fields) < 8:
        return
    try:
        with update_lock:
            sensor.lat = float(fields[0])
            sensor.lon = float(fields[1])
            sensor.speed = float(fields[2])
            sensor.course = float(fields[3])
            sensor.fix_quality = int(float(fields[4]))
            sensor.sats = int(float(fields[5]))
            sensor.rmc_status = fields[6]
            sensor.gps_health = int(float(fields[7]))
        last_gps_update = time.time()
    except ValueError:
        return


def handle_imu(data: str) -> None:
    fields = [x.strip() for x in data.split(",")]
    if len(fields) < 3:
        return
    try:
        with update_lock:
            sensor.yaw = float(fields[0])
            sensor.gyrz = float(fields[1])
            sensor.imu_health = int(float(fields[2]))
    except ValueError:
        return


def handle_barometer(data: str) -> None:
    try:
        alt = float(data.split(",")[0])
    except ValueError:
        return
    with update_lock:
        sensor.baro_m = alt


def handle_target_coord(data: str) -> None:
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
    with update_lock:
        target.lat = lat
        target.lon = lon
    motor_guidance.set_target_coord(lat, lon)


def handle_flight_state(data: str) -> None:
    global state
    try:
        state = int(data.split(",")[0])
    except ValueError:
        return


def handle_release() -> None:
    Motor_Release.activate_burnwire()


def handle_egg_drop() -> None:
    Motor_Egg.activate_solenoid()


def handle_mec(data: str) -> None:
    global motor_enabled
    cmd = data.strip().upper()
    if cmd == "ON":
        motor_enabled = True
    elif cmd == "OFF":
        motor_enabled = False


def _snapshot_sensors():
    with update_lock:
        return SimpleNamespace(
            yaw=sensor.yaw,
            gyrz=sensor.gyrz,
            imu_health=sensor.imu_health,
            lat=sensor.lat,
            lon=sensor.lon,
            speed=sensor.speed,
            course=sensor.course,
            fix_quality=sensor.fix_quality,
            sats=sensor.sats,
            rmc_status=sensor.rmc_status,
            gps_health=sensor.gps_health,
            baro_m=sensor.baro_m,
            target_lat=target.lat,
            target_lon=target.lon,
        )


def _is_finite(x: float) -> bool:
    return not (x is None or math.isnan(x) or math.isinf(x))


def _check_fdir(snap) -> str | None:
    if not _is_finite(snap.yaw) or not _is_finite(snap.gyrz) or not _is_finite(snap.baro_m):
        return "FDIR-0 invalid numeric"
    if snap.imu_health <= 0:
        return "FDIR-1 imu unhealthy"
    if time.time() - last_gps_update > GPS_STALE_TIMEOUT:
        return "FDIR-2 gps stale"

    gps_vector = SimpleNamespace(lat=snap.lat, lon=snap.lon, speed=snap.speed, course=snap.course)
    gps_fidelity = SimpleNamespace(
        fix_quality=snap.fix_quality,
        sats=snap.sats,
        rmc_status=snap.rmc_status,
    )
    if not motor_guidance.is_gps_valid(gps_vector, gps_fidelity):
        return "FDIR-2b gps invalid"
    if motor_guidance.is_gps_jump(snap.lat, snap.lon):
        return "FDIR-2c gps jump"
    if abs(snap.gyrz) > 100.0:
        return "FDIR-3 high yaw rate"
    if snap.baro_m <= 0:
        return "FDIR-4 low altitude"
    if abs(snap.target_lat) < 0.000001 and abs(snap.target_lon) < 0.000001:
        return "FDIR-5 target missing"
    return None


def dispatch(msg: str) -> None:
    global MOTORAPP_RUNSTATUS
    unpacked = msgstructure.unpack_msg(msg)
    if unpacked is False:
        return

    if unpacked.msg_id == appargs.MainAppArg.MID_TerminateProcess:
        MOTORAPP_RUNSTATUS = False
    elif unpacked.msg_id == appargs.GpsAppArg.MID_motor_gps:
        handle_gps(unpacked.data)
    elif unpacked.msg_id == appargs.ImuAppArg.MID_motor_imu:
        handle_imu(unpacked.data)
    elif unpacked.msg_id == appargs.BarometerAppArg.MID_motor_alt:
        handle_barometer(unpacked.data)
    elif unpacked.msg_id == appargs.FlightlogicAppArg.MID_motor_TargetCor:
        handle_target_coord(unpacked.data)
    elif unpacked.msg_id == appargs.FlightlogicAppArg.MID_motor_state:
        handle_flight_state(unpacked.data)
    elif unpacked.msg_id == appargs.FlightlogicAppArg.MID_motor_burnwire:
        handle_release()
    elif unpacked.msg_id == appargs.FlightlogicAppArg.MID_motor_EggDrop:
        handle_egg_drop()
    elif unpacked.msg_id == appargs.CommAppArg.MID_RouteCmd_MEC:
        handle_mec(unpacked.data)


def ctrl_paragldr() -> None:
    while MOTORAPP_RUNSTATUS:
        if state < 3 or not motor_enabled:
            time.sleep(0.1)
            continue
        if state == 5:
            motor_control.set_motors_off(pi)
            time.sleep(0.1)
            continue

        snap = _snapshot_sensors()
        fdir = _check_fdir(snap)
        if fdir is not None:
            motor_control.set_neutral(pi)
            time.sleep(0.1)
            continue

        imu_data = SimpleNamespace(yaw=snap.yaw, gyrz=snap.gyrz)
        gps_vector = SimpleNamespace(lat=snap.lat, lon=snap.lon, speed=snap.speed, course=snap.course)
        gps_fidelity = SimpleNamespace(
            fix_quality=snap.fix_quality,
            sats=snap.sats,
            rmc_status=snap.rmc_status,
        )
        tgt = SimpleNamespace(lat=snap.target_lat, lon=snap.target_lon)

        result = motor_guidance.guidance(imu_data, gps_vector, gps_fidelity, tgt, snap.baro_m)
        motor_control.control(pi, float(result.commanded_yaw_rate))
        time.sleep(0.1)


def init() -> None:
    global pi
    motor_guidance.init_guidance()
    Motor_Release.init_burnwire()
    Motor_Egg.init_solenoid()
    pi = motor_control.init_control()


def motorapp_main(main_pipe) -> None:
    init()
    ctrl_thread = threading.Thread(target=ctrl_paragldr, daemon=True, name="MotorControlLoop")
    ctrl_thread.start()

    try:
        while MOTORAPP_RUNSTATUS:
            try:
                has_msg = main_pipe.poll(0.1)
            except (KeyboardInterrupt, EOFError, OSError):
                break
            if has_msg:
                msg = main_pipe.recv()
                dispatch(msg)
    except KeyboardInterrupt:
        pass
