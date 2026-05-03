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
logger: Optional[MotorLogger] = None

target = types.SimpleNamespace(
    lat  = None,  # Optional[float] — deg, decimal degrees
    lon  = None,  # Optional[float] — deg, decimal degrees
)

altitude = types.SimpleNamespace(
    yaw     = None,   # Optional[float] — deg, 0-360
    gyrz    = None,   # Optional[float] — rad/s (pre-filtered in imu.py)
    healthy = False,  # bool — False when IMU reports stale/fault
)
baro_m: Optional[float] = None  # m, 기압계 고도

GpsVector = types.SimpleNamespace(
    lat    = None,  # Optional[float] — deg, decimal degrees
    lon    = None,  # Optional[float] — deg, decimal degrees
    speed  = None,  # Optional[float] — m/s
    course = None,  # Optional[float] — deg, 0-360
)
GpsFidelity = types.SimpleNamespace(
    rmc_status    = None,  # Optional[str]  — "A"(active) / "V"(void)
    fix_quality   = None,  # Optional[int]  — 0=no fix, 1=GPS, 2=DGPS
    sats          = None,  # Optional[int]  — 위성 수
    jump_rejected = True,  # bool — True until is_gps_jump() clears it
)

state: int = 0
_start_point_locked: bool = False  # True once a valid-GPS start_point is committed

GYRZ_RUNAWAY_THRESHOLD: float = math.radians(100.0) # rad/s (=100°/s), 제어 불능 판정

GPS_STALE_TIMEOUT: float       = 10.0   # s — drop test max valid gap was 9.0 s
GPS_MAX_PLAUSIBLE_SPEED: float = 15.0   # m/s — parafoil physical airspeed ceiling
_gps_last_received_time: float = 0.0    # epoch, 0 = never received
_last_valid_gps_speed: float   = 1.0    # m/s, hold-last on implausible GPS speed

threads: dict[str, threading.Thread] = {}
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


def _check_fdir(snap: types.SimpleNamespace) -> Optional[str]:
    # FDIR-0: 센서 미수신 / NaN·Inf
    gps_missing  = (snap.gps.lat is None or snap.gps.lon is None
                    or not math.isfinite(snap.gps.lat) or not math.isfinite(snap.gps.lon))
    imu_missing  = (snap.imu.yaw is None or snap.imu.gyrz is None
                    or not math.isfinite(snap.imu.yaw) or not math.isfinite(snap.imu.gyrz))
    baro_missing = (snap.baro_m is None or not math.isfinite(snap.baro_m))
    if gps_missing or imu_missing or baro_missing:
        missing = (["GPS"] if gps_missing else []) + \
                  (["IMU"] if imu_missing else []) + \
                  (["BARO"] if baro_missing else [])
        return f"No data received: {'+'.join(missing)}"

    # FDIR-1: IMU 센서-보고 건강 상태 (imu.py의 read_imu_data 루프가 판정)
    if not snap.imu.healthy:
        return "IMU stale (sensor-reported)"

    # FDIR-2: GPS 수신 freshness (마지막 수신 후 GPS_STALE_TIMEOUT 초 초과)
    if _gps_last_received_time > 0:
        gps_age = time.time() - _gps_last_received_time
        if gps_age > GPS_STALE_TIMEOUT:
            return f"GPS stale ({gps_age:.1f}s since last fix, limit={GPS_STALE_TIMEOUT}s)"

    # FDIR-2b: GPS 무결성
    if not motor_guidance.is_gps_valid(snap.gps, snap.gps_fidelity):
        return (f"GPS invalid (lat={snap.gps.lat}, lon={snap.gps.lon}, "
                f"fix={snap.gps_fidelity.fix_quality}, sats={snap.gps_fidelity.sats}, "
                f"rmc={snap.gps_fidelity.rmc_status})")

    # FDIR-2c: GPS 순간 이동 거부
    if snap.gps_fidelity.jump_rejected:
        return (f"GPS jump rejected (lat={snap.gps.lat:.6f}, lon={snap.gps.lon:.6f})")

    # FDIR-3: 극한 회전
    if abs(snap.imu.gyrz) > GYRZ_RUNAWAY_THRESHOLD:
        return (f"|gyrz|={abs(snap.imu.gyrz):.2f} rad/s "
                f"({math.degrees(abs(snap.imu.gyrz)):.1f}°/s) > "
                f"{GYRZ_RUNAWAY_THRESHOLD:.2f} rad/s")

    # FDIR-4: 기압계 고도
    if snap.baro_m <= 0.0:
        return f"Baro altitude invalid ({snap.baro_m:.1f}m)"

    # FDIR-5: 목표 좌표 미수신
    if snap.target.lat is None or snap.target.lon is None:
        return "No target coordinates received"

    return None


def _check_gps_stale() -> bool:
    """GPS stale 여부 반환. 새 경고는 GPS_STALE_TIMEOUT 간격으로만 로그."""
    global _gps_stale_logged_at
    if _gps_last_received_time <= 0:
        return False
    gps_age = time.time() - _gps_last_received_time
    if gps_age <= GPS_STALE_TIMEOUT:
        return False
    now = time.time()
    if now - _gps_stale_logged_at >= GPS_STALE_TIMEOUT:
        log(f"GPS stale ({gps_age:.1f}s) — continuing with last known position", events.EventType.warning)
        _gps_stale_logged_at = now
    return True


def ctrl_paragldr():
    motors_off        = False
    _fdir_last_reason = None
    _fdir_repeat_count = 0
    _ctrl_tick        = 0

    while running:
        try:
            snap = _snapshot_sensors()

            if snap.state >= 3 and snap.motor_enabled:

                if snap.state == 5:
                    if not motors_off:
                        motor_control.set_motors_off(pi)
                        motors_off = True
                        log("State 5: motors off", events.EventType.warning)
                    time.sleep(CONTROL_LOG_INTERVAL)
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
