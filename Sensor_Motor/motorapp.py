from dataclasses import dataclass, field
import logging
import math
import threading
import time
from typing import Optional

from lib import appargs, config, msgstructure, prevstate, sensorlog
from .sensor_types import _GpsFromApp, _ImuFromApp, _BaroFromApp, _Cache
from . import control, guidance

logger = logging.getLogger(__name__)

# ── 비행 전역 상태 ────────────────────────────────────────────────────────────
STATE:            int  = 0
MOTOR_ENABLED:    bool = False
MOTORAPP_RUNSTATUS: bool = True

RELEASE_ACTION_ENABLED: bool = True
EGG_ACTION_ENABLED:     bool = True

PI = None  # pigpio handle

# guidance origin → prevstate 저장 완료 여부 (1회만 저장)
_ORIGIN_SAVED: bool = False

# ── 스레드 공유 변수 ──────────────────────────────────────────────────────────
_CACHE_t     = _Cache()          # 최신 raw 센서 데이터 (handle_* 스레드가 씀)
_UPDATE_LOCK = threading.Lock()  # _CACHE_t 보호

_CTRLER_t: Optional[control.Ctrler] = None  # PID 상태
_CTRL_LOCK = threading.Lock()               # _CTRLER_t reset 동시성

_PREV_STATE: int = 0


# ── 캐시 스냅샷 ───────────────────────────────────────────────────────────────

def _cache_snapshot() -> _Cache:
    return _Cache(
        latest_gps=_GpsFromApp(**vars(_CACHE_t.latest_gps)),
        latest_imu=_ImuFromApp(**vars(_CACHE_t.latest_imu)),
        latest_baro=_BaroFromApp(**vars(_CACHE_t.latest_baro)),
    )


# ── 가속도계 중력 제거 ────────────────────────────────────────────────────────

def _compute_linear_acc(
    roll_deg: float,
    pitch_deg: float,
    ax: float, ay: float, az: float,
    g: float = 9.81,
) -> tuple:
    """body-frame 가속도에서 중력 성분 제거.

    NED z-down 기준 중력 body frame:
        g_x = -sin(pitch)*g
        g_y =  cos(pitch)*sin(roll)*g
        g_z =  cos(pitch)*cos(roll)*g
    """
    roll  = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    cp    = math.cos(pitch)
    g_x   = -math.sin(pitch) * g
    g_y   =  cp * math.sin(roll) * g
    g_z   =  cp * math.cos(roll) * g
    return ax - g_x, ay - g_y, az - g_z


# ── 센서 핸들러 ───────────────────────────────────────────────────────────────

def handle_gps(data: str) -> None:
    """GPS 페이로드 파싱 후 _CACHE 갱신.

    Payload (8 fields): lat,lon,pos_health,pos_ts,course_deg,speed_mps,motion_health,motion_ts
    """
    fields = data.split(",")
    if len(fields) != 8:
        return
    try:
        lat           = float(fields[0])
        lon           = float(fields[1])
        pos_health    = int(float(fields[2]))
        pos_ts        = float(fields[3])
        course_deg    = float(fields[4])
        speed_mps     = float(fields[5])
        motion_health = int(float(fields[6]))
        motion_ts     = float(fields[7])
    except (ValueError, IndexError):
        return

    sample = _GpsFromApp(
        lat=lat        if pos_health    else None,
        lon=lon        if pos_health    else None,
        pos_ts=pos_ts  if pos_health    else None,
        course_rad=math.radians(course_deg) if motion_health else None,
        speed_mps=speed_mps                 if motion_health else None,
        motion_ts=motion_ts                 if motion_health else None,
        rx_ts=time.monotonic(),
        pos_health=pos_health,
        motion_health=motion_health,
    )
    with _UPDATE_LOCK:
        _CACHE_t.latest_gps = sample


def handle_imu(data: str) -> None:
    """IMU 페이로드 파싱 후 _CACHE 갱신.

    Payload (11 fields): roll,pitch,yaw,ax,ay,az,gyrx,gyry,gyrz,health,sample_ts
    각도=deg, 가속도=m/s², 각속도=deg/s
    """
    fields = data.split(",")
    if len(fields) != 11:
        return
    try:
        health    = int(float(fields[9]))
        sample_ts = float(fields[10])
        rx_ts     = time.monotonic()
    except (ValueError, IndexError):
        return

    if health:
        try:
            roll_deg   = float(fields[0])
            pitch_deg  = float(fields[1])
            yaw_deg    = float(fields[2])
            accx_mps2  = float(fields[3])
            accy_mps2  = float(fields[4])
            accz_mps2  = float(fields[5])
            gyrx_deg_s = float(fields[6])
            gyry_deg_s = float(fields[7])
            gyrz_deg_s = float(fields[8])
        except (ValueError, IndexError):
            return

        lin_ax, lin_ay, lin_az = _compute_linear_acc(
            roll_deg, pitch_deg, accx_mps2, accy_mps2, accz_mps2
        )
        lin_valid = (math.isfinite(lin_ax)
                     and math.isfinite(lin_ay)
                     and math.isfinite(lin_az))
        imu = _ImuFromApp(
            roll_rad=math.radians(roll_deg),
            pitch_rad=math.radians(pitch_deg),
            yaw_rad=math.radians(yaw_deg),
            accx_mps2=accx_mps2,
            accy_mps2=accy_mps2,
            accz_mps2=accz_mps2,
            gyrx_rad_s=math.radians(gyrx_deg_s),
            gyry_rad_s=math.radians(gyry_deg_s),
            # IMU Z-up 기준 gz+=CCW; 반전 → nav gz+=CW=오른쪽 회전
            gyrz_rad_s=math.radians(-gyrz_deg_s),
            ts=sample_ts,
            rx_ts=rx_ts,
            lin_acc_x=lin_ax if lin_valid else None,
            lin_acc_y=lin_ay if lin_valid else None,
            lin_acc_z=lin_az if lin_valid else None,
            lin_acc_valid=lin_valid,
            health=1,
        )
    else:
        # 하드웨어 이상: 타임스탬프만 갱신
        imu = _ImuFromApp(ts=sample_ts, rx_ts=rx_ts, health=0)

    with _UPDATE_LOCK:
        _CACHE.latest_imu = imu


def handle_barometer(data: str) -> None:
    """기압계 페이로드 파싱 후 _CACHE 갱신.

    Payload (3 fields): alt_m,sink_rate,health
    """
    fields = data.split(",")
    if len(fields) != 3:
        return
    try:
        alt_raw  = float(fields[0].strip())
        sink_raw = float(fields[1].strip())
        health   = int(float(fields[2].strip()))
        rx_ts    = time.monotonic()
    except (ValueError, IndexError):
        return

    baro = _BaroFromApp(
        alt_m=alt_raw      if health and math.isfinite(alt_raw)  else None,
        sink_rate=sink_raw  if health and math.isfinite(sink_raw) else None,
        rx_ts=rx_ts,
        health=health,
    )
    with _UPDATE_LOCK:
        _CACHE.latest_baro = baro


def handle_target_coord(data: str) -> None:
    """타겟 좌표 수신. 유효성 검사 후 guidance.set_target()에 위임."""
    fields = data.split(",")
    if len(fields) != 2:
        return
    try:
        lat = float(fields[0])
        lon = float(fields[1])
    except (ValueError, IndexError):
        return
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return
    if abs(lat) < 1e-9 and abs(lon) < 1e-9:
        return   # (0,0) sentinel 거부
    guidance.set_target(lat, lon)


def handle_flight_state(data: str) -> None:
    """비행 상태 업데이트. 상태 3 미만이면 guidance/controller 리셋."""
    global STATE, _PREV_STATE, _CTRLER
    try:
        new_state = int(data.split(",")[0])
    except (ValueError, IndexError):
        return
    if new_state == STATE:
        return

    do_ctrl_reset = False
    with _UPDATE_LOCK:
        _PREV_STATE = STATE
        STATE       = new_state
        if new_state < 3:
            prevstate.clear_start_point()
            guidance.reset()
            do_ctrl_reset = True

    if do_ctrl_reset and _CTRLER is not None:
        with _CTRL_LOCK:
            control.controller_reset(_CTRLER)


def handle_release(data: str = "TRIGGER") -> None:
    if not RELEASE_ACTION_ENABLED:
        return
    try:
        from . import Motor_Release
    except Exception:
        return
    if not hasattr(Motor_Release, "activate_burnwire"):
        return
    threading.Thread(
        target=Motor_Release.activate_burnwire,
        daemon=True, name="Burnwire"
    ).start()


def handle_egg_drop() -> None:
    if not EGG_ACTION_ENABLED:
        return
    try:
        from . import Motor_Egg
    except Exception:
        return
    if not hasattr(Motor_Egg, "activate_solenoid"):
        return
    threading.Thread(
        target=Motor_Egg.activate_solenoid,
        daemon=True, name="Solenoid"
    ).start()


# ── 제어 루프 ─────────────────────────────────────────────────────────────────

def _ctrl_cycle(main_queue, now: float) -> Optional[control.CtrlOutput]:
    """한 사이클 제어 계산. 반환값은 진단용 (None이면 액션 없음)."""
    global _CTRLER

    with _UPDATE_LOCK:
        motor_enabled = MOTOR_ENABLED
        state         = STATE
        snap          = _cache_snapshot()

    # 비활성 또는 비행 전 상태
    if not motor_enabled or state < 3:
        if PI is not None:
            control.WriteZero(PI)
        return None

    # 착지 후
    if state == 5:
        if PI is not None:
            control.WriteOff(PI)
        return None

    # ── 컨트롤러 지연 초기화 ─────────────────────────────────────────────────
    if _CTRLER is None:
        with _CTRL_LOCK:
            if _CTRLER is None:
                _CTRLER = control.MakeCtrler()

    # ── 항법 파이프라인 ───────────────────────────────────────────────────────
    guidance.UpdateAnchors(snap.latest_gps, snap.latest_imu, snap.latest_baro, now)
    mode = guidance.DecideControlMode(now)

    if mode == guidance.ControlMode.DETUMBLING:
        ctrl_out = control.ProduceDetumbleOutput(now)
        control.MoveServo(PI, ctrl_out)
        return ctrl_out

    if mode in (guidance.ControlMode.GPS_TRACKING_CLOSED,
                guidance.ControlMode.GPS_TRACKING_OPEN,
                guidance.ControlMode.DR_TRACKING_CLOSED,
                guidance.ControlMode.DR_TRACKING_OPEN):
        l1_in   = guidance.ProduceL1Input(now)
        l1_out  = guidance.ProduceL1Output(l1_in)
        gz_meas = snap.latest_imu.gyrz_rad_s or 0.0
        gz_meas = math.degrees(float(gz_meas))   # rad/s → deg/s (ProduceCtrlOutput 기대 단위)
        ctrl_in  = control.ProduceCtrlInput(l1_out, now)
        ctrl_out = control.ProduceCtrlOutput(_CTRLER, ctrl_in, gz_meas, now)
        control.MoveServo(PI, ctrl_out)
        return ctrl_out

    # FAIL
    if PI is not None:
        control.WriteOff(PI)
    return None


def _sleep_for_period(cycle_start: float, period: float) -> None:
    elapsed = time.monotonic() - cycle_start
    remaining = period - elapsed
    if remaining > 0:
        time.sleep(remaining)


def ctrl_parafoil(main_queue=None) -> None:
    """패러포일 제어 루프. 루프율 = config.MOTOR_RATE_HZ."""
    period = 1.0 / max(0.1, float(config.MOTOR_RATE_HZ))
    while MOTORAPP_RUNSTATUS:
        cycle_start = time.monotonic()
        try:
            _ctrl_cycle(main_queue, time.monotonic())
        except Exception:
            logger.exception("_ctrl_cycle error")
        _sleep_for_period(cycle_start, period)
