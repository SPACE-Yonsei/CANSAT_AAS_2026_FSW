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
_DETUMBLE_ACTIVE: bool = False
_DETUMBLE_EXIT_START: float = math.nan

# 수동 조향 모드: "" = auto(L1 guidance), "LEFT"/"RIGHT"/"NEUTRAL" = 고정 override
_MANUAL_STEER_MODE: str = ""

# 모터 제어 소스 모드 (CMC 명령으로 변경)
MOTOR_CTRL_MODE: str = config.MOTOR_CTRL_MODE_GPS_GUIDED

# IMU_HEADING: GPS+target 없을 때 로그 중복 방지 플래그
_imu_heading_fallback_logged: bool = False
# IMU_HEADING: GPS 유실 시 마지막으로 유효했던 bearing (IMU 프레임, deg)
_imu_heading_last_bearing_deg: float | None = None

_IMU_HEADING_DEADBAND_DEG: float = 5.0   # ±5° 이내 → 서보 중립
_IMU_HEADING_MAX_ERR_DEG:  float = 90.0  # ±90° 이상 → 최대 deflection


def _publish_motor_diag(main_queue, ctrl_out, snap_t, guidance_state: str) -> None:
    """motor 진단 패킷을 commapp으로 발행 (MID_comm_motor_diag)."""
    if main_queue is None:
        return
    mi  = guidance._MISSION_t

    s_lat = f"{mi.origin_lat:.6f}"  if (mi.origin_ready and math.isfinite(mi.origin_lat))  else "nan"
    s_lon = f"{mi.origin_lon:.6f}"  if (mi.origin_ready and math.isfinite(mi.origin_lon))  else "nan"
    t_lat = f"{mi._target_lat:.6f}" if math.isfinite(mi._target_lat)                        else "nan"
    t_lon = f"{mi._target_lon:.6f}" if math.isfinite(mi._target_lon)                        else "nan"

    imu = snap_t.latest_imu
    if imu.yaw_rad is not None and math.isfinite(float(imu.yaw_rad)):
        heading = f"{math.degrees(float(imu.yaw_rad)):.2f}"
    else:
        heading = "nan"

    payload = (
        f"{ctrl_out.left_pw},{ctrl_out.right_pw},"
        f"{s_lat},{s_lon},"
        f"{t_lat},{t_lon},"
        f"nan,nan,"
        f"{heading},"
        f"{guidance_state},"
        f"{int(bool(MOTOR_ENABLED))},"
        f"0,"
        f"{int(bool(RELEASE_ACTION_ENABLED))},"
        f"{int(bool(EGG_ACTION_ENABLED))}"
    )
    msgstructure.send_msg(
        main_queue,
        appargs.MotorAppArg.AppID,
        appargs.CommAppArg.AppID,
        appargs.MotorAppArg.MID_comm_motor_diag,
        payload,
    )


# ── 캐시 스냅샷 ───────────────────────────────────────────────────────────────

def _cache_snapshot() -> _Cache:
    try:
        t_lat = float(guidance._MISSION_t._target_lat)
        t_lon = float(guidance._MISSION_t._target_lon)
    except Exception:
        t_lat = None
        t_lon = None
    return _Cache(
        latest_gps=_GpsFromApp(**vars(_CACHE_t.latest_gps)),
        latest_imu=_ImuFromApp(**vars(_CACHE_t.latest_imu)),
        latest_baro=_BaroFromApp(**vars(_CACHE_t.latest_baro)),
        target_lat=t_lat,
        target_lon=t_lon,
    )


def _fresh_gyrz_dps(snap_t: _Cache, now: float) -> Optional[float]:
    imu = snap_t.latest_imu
    ts = getattr(imu, "ts", None)
    gyrz = getattr(imu, "gyrz_rad_s", None)
    health = bool(getattr(imu, "health", 0))
    try:
        ts_f = float(ts)
        gyrz_f = float(gyrz)
    except (TypeError, ValueError):
        return None
    if not health or not math.isfinite(ts_f) or not math.isfinite(gyrz_f):
        return None
    if now - ts_f > config.IMU_FRESH_MAX_AGE_S:
        return None
    return math.degrees(gyrz_f)


def _should_detumble(snap_t: _Cache, now: float) -> bool:
    global _DETUMBLE_ACTIVE, _DETUMBLE_EXIT_START

    if not config.DETUMBLE_ENABLE:
        _DETUMBLE_ACTIVE = False
        _DETUMBLE_EXIT_START = math.nan
        return False

    gyrz_dps = _fresh_gyrz_dps(snap_t, now)
    if gyrz_dps is None:
        _DETUMBLE_ACTIVE = False
        _DETUMBLE_EXIT_START = math.nan
        return False

    abs_gyrz = abs(gyrz_dps)
    if _DETUMBLE_ACTIVE:
        if abs_gyrz <= config.DETUMBLE_EXIT_THRESHOLD_DPS:
            if not math.isfinite(_DETUMBLE_EXIT_START):
                _DETUMBLE_EXIT_START = now
                return True
            if now - _DETUMBLE_EXIT_START < config.DETUMBLE_EXIT_HOLD_S:
                return True
            _DETUMBLE_ACTIVE = False
            _DETUMBLE_EXIT_START = math.nan
            return False
        _DETUMBLE_EXIT_START = math.nan
        return True

    if abs_gyrz >= config.DETUMBLE_GYRZ_THRESHOLD_DPS:
        _DETUMBLE_ACTIVE = True
        _DETUMBLE_EXIT_START = math.nan
        return True
    return False


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
    """GPS 페이로드 파싱 후 _CACHE_t 갱신.

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
    """IMU 페이로드 파싱 후 _CACHE_t 갱신.

    Payload (12 fields): roll,pitch,yaw,ax,ay,az,gyrx,gyry,gyrz,health,sample_ts,yaw_offset
    각도=deg, 가속도=m/s², 각속도=deg/s
    """
    fields = data.split(",")
    if len(fields) < 11:
        return
    try:
        health         = int(float(fields[9]))
        sample_ts      = float(fields[10])
        yaw_offset_deg = float(fields[11]) if len(fields) >= 12 else 0.0
        rx_ts          = time.monotonic()
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
            yaw_offset_deg=yaw_offset_deg,
        )
    else:
        # 하드웨어 이상: 타임스탬프만 갱신
        imu = _ImuFromApp(ts=sample_ts, rx_ts=rx_ts, health=0, yaw_offset_deg=yaw_offset_deg)

    with _UPDATE_LOCK:
        _CACHE_t.latest_imu = imu


def handle_barometer(data: str) -> None:
    """기압계 페이로드 파싱 후 _CACHE_t 갱신.

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
        _CACHE_t.latest_baro = baro


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
    global STATE, _PREV_STATE, _CTRLER_t, _ORIGIN_SAVED
    global _DETUMBLE_ACTIVE, _DETUMBLE_EXIT_START
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
            _ORIGIN_SAVED = False   # guidance.reset()이 origin 초기화 → 재동기화 허용
            _DETUMBLE_ACTIVE = False
            _DETUMBLE_EXIT_START = math.nan

    if do_ctrl_reset and _CTRLER_t is not None:
        with _CTRL_LOCK:
            control.controller_reset(_CTRLER_t)


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


def handle_mec(data: str) -> None:
    """MEC 명령: 모터 활성/비활성 토글 + prevstate 저장."""
    global MOTOR_ENABLED
    cmd = data.strip().upper()
    if cmd == "ON":
        MOTOR_ENABLED = True
        prevstate.update_motor_enabled(True)
    elif cmd == "OFF":
        MOTOR_ENABLED = False
        prevstate.update_motor_enabled(False)
        if PI is not None:
            control.WriteZero(PI)


def _imu_heading_target_deg(yaw_rad: float, snap: _Cache) -> float:
    """GPS bearing → IMU 프레임 target heading (deg). GPS 없으면 마지막 bearing 유지."""
    global _imu_heading_fallback_logged, _imu_heading_last_bearing_deg

    gps     = snap.latest_gps
    t_lat   = snap.target_lat
    t_lon   = snap.target_lon
    gps_lat = gps.lat if gps is not None else None
    gps_lon = gps.lon if gps is not None else None

    if (gps_lat is not None and math.isfinite(gps_lat)
            and gps_lon is not None and math.isfinite(gps_lon)
            and t_lat is not None and math.isfinite(t_lat)
            and t_lon is not None and math.isfinite(t_lon)
            and abs(t_lat) > 1e-9 and abs(t_lon) > 1e-9):
        dlon = math.radians(t_lon - gps_lon)
        lat1 = math.radians(gps_lat)
        lat2 = math.radians(t_lat)
        y_b  = math.sin(dlon) * math.cos(lat2)
        x_b  = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        abs_bearing_rad    = math.atan2(y_b, x_b)
        yaw_off            = snap.latest_imu.yaw_offset_deg
        target_deg         = math.degrees(abs_bearing_rad) + yaw_off
        _imu_heading_last_bearing_deg = target_deg
        if _imu_heading_fallback_logged:
            logger.info(
                "IMU_HEADING: bearing restored — gps=(%.5f,%.5f) target=(%.5f,%.5f)"
                " bearing=%.1f° yaw_off=%.1f°",
                gps_lat, gps_lon, t_lat, t_lon,
                math.degrees(abs_bearing_rad), yaw_off,
            )
            _imu_heading_fallback_logged = False
        return target_deg
    else:
        if _imu_heading_last_bearing_deg is not None:
            target_deg = _imu_heading_last_bearing_deg
        else:
            target_deg = math.degrees(yaw_rad)
        if not _imu_heading_fallback_logged:
            logger.info(
                "IMU_HEADING fallback (last bearing=%.1f°): gps_lat=%s gps_lon=%s t_lat=%s t_lon=%s",
                target_deg, gps_lat, gps_lon, t_lat, t_lon,
            )
            _imu_heading_fallback_logged = True
        return target_deg


def _imu_heading_direct_output(
    now: float, yaw_rad: float, snap: _Cache, ctl: control.Ctrler
) -> control.CtrlOutput:
    """IMU_HEADING 직접 매핑: error_deg → 서보 delta 선형 보간.

    deadband ±5°: 서보 중립.
    |error| ≥ 90°: DELTA_ARM_MAX_DEG 포화.
    5° < |error| < 90°: 선형 보간.
    slew-rate 적용으로 포화→선형 전환 구간 부드럽게 처리.
    """
    target_deg = _imu_heading_target_deg(yaw_rad, snap)
    error_deg  = (target_deg - math.degrees(yaw_rad) + 180.0) % 360.0 - 180.0
    abs_err    = abs(error_deg)

    if abs_err <= _IMU_HEADING_DEADBAND_DEG:
        delta     = 0.0
        saturated = False
    elif abs_err >= _IMU_HEADING_MAX_ERR_DEG:
        delta     = math.copysign(control.DELTA_ARM_MAX_DEG, error_deg)
        saturated = True
    else:
        t     = (abs_err - _IMU_HEADING_DEADBAND_DEG) / (_IMU_HEADING_MAX_ERR_DEG - _IMU_HEADING_DEADBAND_DEG)
        delta = math.copysign(t * control.DELTA_ARM_MAX_DEG, error_deg)
        saturated = False

    _, _, left_des, right_des, delta_arm = control.ConnectRoMo(delta)

    dt       = max(0.01, min(0.2, now - ctl.pid.prev_time)) if ctl.pid.prev_time > 0 else 0.05
    max_step = ctl.config.MAX_ARM_RATE_DEG_S * dt
    left_angle  = max(ctl.prev_left_angle_deg  - max_step, min(ctl.prev_left_angle_deg  + max_step, left_des))
    right_angle = max(ctl.prev_right_angle_deg - max_step, min(ctl.prev_right_angle_deg + max_step, right_des))

    left_pw  = int(max(control.LEFT_MIN_PULSE,  min(control.LEFT_MAX_PULSE,
                       control.LEFT_ZERO  - left_angle  * control.PULSE_PER_DEG)))
    right_pw = int(max(control.RIGHT_MIN_PULSE, min(control.RIGHT_MAX_PULSE,
                       control.RIGHT_ZERO + right_angle * control.PULSE_PER_DEG)))

    ctl.prev_left_angle_deg  = left_angle
    ctl.prev_right_angle_deg = right_angle
    ctl.pid.prev_time        = now

    return control.CtrlOutput(
        timestamp=now,
        left_pw=left_pw,
        right_pw=right_pw,
        left_angle_deg=left_angle,
        right_angle_deg=right_angle,
        delta_arm_deg=delta_arm,
        delta_ff_deg=delta,
        saturated=saturated,
        valid=True,
    )


def handle_cmc(data: str) -> None:
    global MOTOR_CTRL_MODE
    mode = str(data or "").strip().upper()
    valid = {
        config.MOTOR_CTRL_MODE_GPS_GUIDED,
        config.MOTOR_CTRL_MODE_GPS_ONLY,
        config.MOTOR_CTRL_MODE_IMU_HEADING,
    }
    if mode not in valid:
        logger.debug("CMC: rejected unknown mode %r", mode)
        return
    MOTOR_CTRL_MODE = mode
    logger.info("CMC: switched to %s", mode)


def handle_mtr(data: str) -> None:
    """MTR 명령: 수동 조향 모드 설정.

    LEFT/RIGHT → 고정 deflection(±MANUAL_STEER_DELTA_DEG) 유지.
    NEUTRAL    → 서보 중립 고정 후 auto(L1 guidance)로 복귀.
    """
    global _MANUAL_STEER_MODE
    aliases = {
        "L": "LEFT", "LEFT": "LEFT",
        "N": "NEUTRAL", "NEUTRAL": "NEUTRAL",
        "R": "RIGHT", "RIGHT": "RIGHT",
    }
    mode = aliases.get(data.strip().upper())
    if mode is None:
        return
    _MANUAL_STEER_MODE = "" if mode == "NEUTRAL" else mode
    logger.info("MTR manual steer: %s → _MANUAL_STEER_MODE=%r", mode, _MANUAL_STEER_MODE)
    if mode == "NEUTRAL" and PI is not None:
        PI.set_servo_pulsewidth(control.PARAFOIL_LEFT_MOTOR_PIN,  control.LEFT_NEUTRAL)
        PI.set_servo_pulsewidth(control.PARAFOIL_RIGHT_MOTOR_PIN, control.RIGHT_NEUTRAL)


def handle_fac(data: str) -> None:
    """FAC 명령: 릴리즈/에그 액추에이터 활성화 제어.

    형식: "ON"/"OFF" (둘 다) 또는 "ALL|REL|EGG,ON|OFF".
    """
    global RELEASE_ACTION_ENABLED, EGG_ACTION_ENABLED
    raw = data.strip().upper().replace(" ", "")
    parts = [p for p in raw.split(",") if p]
    if len(parts) == 1 and parts[0] in {"ON", "OFF"}:
        actor, state = "ALL", parts[0]
    elif len(parts) == 2 and parts[0] in {"ALL", "REL", "EGG"} and parts[1] in {"ON", "OFF"}:
        actor, state = parts[0], parts[1]
    else:
        return

    enabled = state == "ON"
    if actor in {"ALL", "REL"}:
        RELEASE_ACTION_ENABLED = enabled
    if actor in {"ALL", "EGG"}:
        EGG_ACTION_ENABLED = enabled

# ── 제어 루프 ─────────────────────────────────────────────────────────────────

def _sync_origin_to_prevstate() -> bool:
    """guidance origin이 확정되면 prevstate에 1회 저장. 저장 시 True 반환."""
    mi_t = guidance._MISSION_t
    if not mi_t.origin_ready:
        return False
    lat = float(mi_t.origin_lat)
    lon = float(mi_t.origin_lon)
    prevstate.update_start_point(lat, lon, True)
    logger.info("Origin synced to prevstate: lat=%.6f lon=%.6f", lat, lon)
    return True


def _ctrl_cycle(main_queue, now: float) -> Optional[control.CtrlOutput]:
    """한 사이클 제어 계산. 반환값은 진단용 (None이면 액션 없음)."""
    global _CTRLER_t, _ORIGIN_SAVED

    with _UPDATE_LOCK:
        motor_enabled = MOTOR_ENABLED
        state         = STATE
        snap_t        = _cache_snapshot()

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
    if _CTRLER_t is None:
        with _CTRL_LOCK:
            if _CTRLER_t is None:
                _CTRLER_t = control.MakeCtrler()

    # ── 항법 파이프라인 ───────────────────────────────────────────────────────
    guidance.UpdateRaws(snap_t.latest_gps, snap_t.latest_imu, snap_t.latest_baro, now)

    # origin 확정 시 prevstate에 1회 저장
    if not _ORIGIN_SAVED and _sync_origin_to_prevstate():
        _ORIGIN_SAVED = True

    # ── [1] MANUAL STEER override ────────────────────────────────────────────
    if _MANUAL_STEER_MODE in ("LEFT", "RIGHT", "NEUTRAL"):
        _delta = {
            "LEFT":    -config.MANUAL_STEER_DELTA_DEG,
            "RIGHT":   +config.MANUAL_STEER_DELTA_DEG,
            "NEUTRAL":  0.0,
        }[_MANUAL_STEER_MODE]
        lp, rp, la, ra, _da = control.ConnectRoMo(_delta)
        manual_out = control.CtrlOutput(
            timestamp=now,
            left_pw=lp, right_pw=rp,
            left_angle_deg=la, right_angle_deg=ra,
            delta_arm_deg=_da,
            delta_ff_deg=_da,
            valid=True,
            control_mode=guidance.ControlMode.FAIL,
        )
        control.MoveServo(PI, manual_out)
        sensorlog.log_motor_raw(state, motor_enabled, MOTOR_CTRL_MODE, manual_out)
        _publish_motor_diag(main_queue, manual_out, snap_t, f"MANUAL_{MOTOR_CTRL_MODE}")
        return manual_out

    # ── [2] DETUMBLING ────────────────────────────────────────────────────────
    if _should_detumble(snap_t, now):
        guidance._STATE_t.nav.control_mode = guidance.ControlMode.DETUMBLING
        gz_meas = _fresh_gyrz_dps(snap_t, now)
        if gz_meas is None:
            gz_meas = math.nan
        ctrl_out_t = control.ProduceDetumbleOutput(now, gz_meas)
        _CTRLER_t.prev_left_angle_deg = ctrl_out_t.left_angle_deg
        _CTRLER_t.prev_right_angle_deg = ctrl_out_t.right_angle_deg
        control.MoveServo(PI, ctrl_out_t)
        sensorlog.log_motor_raw(state, motor_enabled, MOTOR_CTRL_MODE, ctrl_out_t)
        _publish_motor_diag(main_queue, ctrl_out_t, snap_t, "DETUMBLING")
        return ctrl_out_t

    mode = guidance.DecideControlMode(now)

    # ── [3] IMU_HEADING 모드 ──────────────────────────────────────────────────
    if MOTOR_CTRL_MODE == config.MOTOR_CTRL_MODE_IMU_HEADING:
        yaw = snap_t.latest_imu.yaw_rad
        if yaw is not None and math.isfinite(float(yaw)):
            ctrl_out_t = _imu_heading_direct_output(now, float(yaw), snap_t, _CTRLER_t)
            control.MoveServo(PI, ctrl_out_t)
            sensorlog.log_motor_raw(state, motor_enabled, MOTOR_CTRL_MODE, ctrl_out_t)
            _publish_motor_diag(main_queue, ctrl_out_t, snap_t, config.MOTOR_CTRL_MODE_IMU_HEADING)
            return ctrl_out_t
        # IMU yaw 무효 → GPS/DR fallthrough

    # ── [4] GPS/DR 자율 추종 ─────────────────────────────────────────────────
    if mode in (guidance.ControlMode.GPS_TRACKING_CLOSED,
                guidance.ControlMode.GPS_TRACKING_OPEN,
                guidance.ControlMode.DR_TRACKING_CLOSED,
                guidance.ControlMode.DR_TRACKING_OPEN):
        l1_in_t  = guidance.ProduceL1Input(now)
        l1_out_t = guidance.ProduceL1Output(l1_in_t)
        gz_meas  = snap_t.latest_imu.gyrz_rad_s or 0.0
        gz_meas  = math.degrees(float(gz_meas))   # rad/s → deg/s (ProduceCtrlOutput 기대 단위)
        ctrl_in_t  = control.ProduceCtrlInput(l1_out_t, now)
        ctrl_out_t = control.ProduceCtrlOutput(_CTRLER_t, ctrl_in_t, gz_meas, now)
        control.MoveServo(PI, ctrl_out_t)
        sensorlog.log_motor_raw(state, motor_enabled, MOTOR_CTRL_MODE, ctrl_out_t, l1_out_t)
        _publish_motor_diag(main_queue, ctrl_out_t, snap_t, guidance._STATE_t.nav.control_mode.value)
        return ctrl_out_t

    # ── [4] FAIL ─────────────────────────────────────────────────────────────
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


# ── 메시지 라우터 ─────────────────────────────────────────────────────────────

def dispatch(msg: str) -> None:
    """버스 메시지를 MID 기준으로 해당 핸들러에 전달."""
    global MOTORAPP_RUNSTATUS
    unpacked = msgstructure.unpack_msg(msg)
    if unpacked is False:
        return
    mid = unpacked.msg_id
    if mid == appargs.MainAppArg.MID_TerminateProcess:
        MOTORAPP_RUNSTATUS = False
    elif mid == appargs.GpsAppArg.MID_motor_gps:
        handle_gps(unpacked.data)
    elif mid == appargs.ImuAppArg.MID_motor_imu:
        handle_imu(unpacked.data)
    elif mid == appargs.BarometerAppArg.MID_motor_alt:
        handle_barometer(unpacked.data)
    elif mid == appargs.FlightlogicAppArg.MID_motor_TargetCor:
        handle_target_coord(unpacked.data)
    elif mid == appargs.FlightlogicAppArg.MID_motor_state:
        handle_flight_state(unpacked.data)
    elif mid == appargs.FlightlogicAppArg.MID_motor_burnwire:
        handle_release(unpacked.data)
    elif mid == appargs.FlightlogicAppArg.MID_motor_EggDrop:
        handle_egg_drop()
    elif mid == appargs.CommAppArg.MID_RouteCmd_CMC:
        handle_cmc(unpacked.data)
    elif mid == appargs.CommAppArg.MID_RouteCmd_MEC:
        handle_mec(unpacked.data)
    elif mid == appargs.CommAppArg.MID_RouteCmd_MTR:
        handle_mtr(unpacked.data)
    elif mid == appargs.CommAppArg.MID_RouteCmd_FAC:
        handle_fac(unpacked.data)


# ── 초기화 / 진입점 ───────────────────────────────────────────────────────────

def init() -> None:
    """prevstate 복원 + 컨트롤러/pigpio 초기화."""
    global PI, MOTOR_ENABLED, RELEASE_ACTION_ENABLED, EGG_ACTION_ENABLED
    global _CTRLER_t, _ORIGIN_SAVED, STATE
    global _DETUMBLE_ACTIVE, _DETUMBLE_EXIT_START

    prevstate.init_prevstate()
    # Restore flight state so _ctrl_cycle is not blocked on the first cycle.
    # Without this, STATE stays 0 until flightlogicapp sends MID_motor_state.
    STATE = prevstate.PREV_STATE
    MOTOR_ENABLED = prevstate.is_motor_enabled()
    RELEASE_ACTION_ENABLED = True
    EGG_ACTION_ENABLED = True
    _DETUMBLE_ACTIVE = False
    _DETUMBLE_EXIT_START = math.nan

    # target 좌표 복원
    t_lat, t_lon = prevstate.get_target_gps()
    if (-90.0 <= float(t_lat) <= 90.0
            and -180.0 <= float(t_lon) <= 180.0
            and not (t_lat == 0.0 and t_lon == 0.0)):
        guidance.set_target(float(t_lat), float(t_lon))

    # origin 복원 (PREV_START_LOCKED==1일 때만 반환)
    start = prevstate.get_start_point()
    if start is not None:
        lat, lon = start
        if (-90.0 <= float(lat) <= 90.0
                and -180.0 <= float(lon) <= 180.0
                and not (lat == 0.0 and lon == 0.0)):
            mi_t = guidance._MISSION_t
            mi_t.origin_lat   = float(lat)
            mi_t.origin_lon   = float(lon)
            mi_t.origin_ready = True
            mi_t._raw_lat     = float(lat)
            mi_t._raw_lon     = float(lon)
            _ORIGIN_SAVED = True
            logger.info("Origin restored from prevstate: lat=%.6f lon=%.6f", lat, lon)
            # set_target이 _target_lat/lon을 저장한 경우 즉시 투영
            if math.isfinite(mi_t._target_lat) and math.isfinite(mi_t._target_lon):
                tN, tE = guidance.latlon_to_ne(mi_t._target_lat, mi_t._target_lon,
                                               mi_t.origin_lat, mi_t.origin_lon)
                mi_t.target_E     = tE
                mi_t.target_N     = tN
                mi_t.target_ready = True
                logger.info("Target re-projected on init: E=%.1f N=%.1f", tE, tN)

    _CTRLER_t = control.MakeCtrler()
    PI = control.init_control()

    try:
        from . import Motor_Release
        if hasattr(Motor_Release, "init_burnwire"):
            Motor_Release.init_burnwire()
    except Exception:
        pass

    try:
        from . import Motor_Egg
        if hasattr(Motor_Egg, "init_solenoid"):
            Motor_Egg.init_solenoid()
    except Exception:
        pass

    logger.info("motorapp init complete: MOTOR_ENABLED=%s", MOTOR_ENABLED)


def motorapp_main(main_queue, main_pipe=None) -> None:
    """모터앱 진입점. ctrl_parafoil 스레드 + dispatch 루프."""
    global MOTORAPP_RUNSTATUS
    if main_pipe is None:
        main_pipe = main_queue
        main_queue = None

    init()

    ctrl_thread = threading.Thread(
        target=ctrl_parafoil,
        args=(main_queue,),
        daemon=True,
        name="MotorControlLoop",
    )
    ctrl_thread.start()

    poll_period = 1.0 / max(0.1, float(config.MOTOR_RATE_HZ))
    try:
        while MOTORAPP_RUNSTATUS:
            try:
                if main_pipe.poll(poll_period):
                    dispatch(main_pipe.recv())
            except (KeyboardInterrupt, EOFError, OSError):
                break
    except KeyboardInterrupt:
        pass

    MOTORAPP_RUNSTATUS = False
    ctrl_thread.join(timeout=1.0)
