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

# ── 스레드 공유 변수 ──────────────────────────────────────────────────────────
_CACHE_t     = _Cache()          # 최신 raw 센서 데이터 (handle_* 스레드가 씀)
_UPDATE_LOCK = threading.Lock()  # _CACHE_t 보호

_PREV_STATE: int = 0
_ORIGIN_LOCKED: bool = False

TARGET_LAT = 38.376016667    # 38°22'33.66"N
TARGET_LON = -79.607872222   # 79°36'28.34"W

# 수동 조향 모드: "" = auto(L1 guidance), "LEFT"/"RIGHT"/"NEUTRAL" = 고정 override
_STEER_MODE: str = ""

# 모터 제어 소스 선호 (CMC 명령으로 변경). 최종 control mode는 guidance가 결정하며
# 이 값은 로깅/소스 선호 표식으로만 쓰인다. (IMU_HEADING 독립 출력 분기는 제거됨)
MOTOR_CTRL_MODE: str = config.MOTOR_CTRL_MODE_GPS_GUIDED


def _publish_motor_diag(main_queue, ctrl_out, snap_t, guidance_state: str) -> None:
    """motor 진단 패킷을 commapp으로 발행 (MID_comm_motor_diag)."""
    if main_queue is None:
        return
    mi  = guidance._MISSION_t

    s_lat = f"{mi.origin_lat:.6f}" if math.isfinite(mi.origin_lat) else "nan"
    s_lon = f"{mi.origin_lon:.6f}" if math.isfinite(mi.origin_lon) else "nan"
    t_lat = f"{mi.target_lat:.6f}" if math.isfinite(mi.target_lat) else "nan"
    t_lon = f"{mi.target_lon:.6f}" if math.isfinite(mi.target_lon) else "nan"

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
        t_lat = float(guidance._MISSION_t.target_lat)
        t_lon = float(guidance._MISSION_t.target_lon)
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


# ── 가속도계 중력 제거 ────────────────────────────────────────────────────────

_RAW_ACC_MAX_MPS2 = getattr(config, "RAW_ACC_NORM_MAX_MPS2", 15.0)

_ACCEL_MODE_WARNED = False


def _accel_input_mode() -> str:
    """config.IMU_ACCEL_INPUT_MODE 정규화. RAW/LINEAR 외 값이면 RAW fallback(1회 warning)."""
    global _ACCEL_MODE_WARNED
    mode = str(getattr(config, "IMU_ACCEL_INPUT_MODE", "RAW")).strip().upper()
    if mode in ("RAW", "LINEAR"):
        return mode
    if not _ACCEL_MODE_WARNED:
        logger.warning("IMU_ACCEL_INPUT_MODE=%r unknown; falling back to RAW", mode)
        _ACCEL_MODE_WARNED = True
    return "RAW"


@dataclass(frozen=True)
class LinearAccResult:
    x: float
    y: float
    z: float
    valid: bool
    reject_reason: str
    raw_norm: float
    gravity_x: float
    gravity_y: float
    gravity_z: float
    xy_mag: float

def _compute_linear_acc(
    roll_deg: float,
    pitch_deg: float,
    ax: float, ay: float, az: float,
    gyrx_deg_s: float = 0.0,
    gyry_deg_s: float = 0.0,
    gyrz_deg_s: float = 0.0,
    sample_age_s: float = 0.0,
    g: float = 9.81,
) -> LinearAccResult:
    """body-frame 가속도 → linear acceleration.

    config.IMU_ACCEL_INPUT_MODE에 따라 분기:
      "RAW"    : ax/ay/az가 중력 포함 raw 가속도 → roll/pitch 기반 중력 제거.
                 NED z-down 기준 body-frame 중력:
                   g_x = -sin(pitch)*g
                   g_y =  cos(pitch)*sin(roll)*g
                   g_z =  cos(pitch)*cos(roll)*g
      "LINEAR" : ax/ay/az가 이미 중력 제거된 linear acceleration → 그대로 사용
                 (이중 중력 제거 버그 방지). gravity_*는 debug용 NaN.

    raw acc magnitude가 _RAW_ACC_MAX_MPS2를 초과하면 BNO085 spike로 간주,
    (nan, nan, nan) 반환하여 lin_acc_valid=False 처리.
    """
    nan = float("nan")
    inputs = (roll_deg, pitch_deg, ax, ay, az)
    try:
        inputs_ok = all(math.isfinite(float(v)) for v in inputs)
    except (TypeError, ValueError):
        inputs_ok = False
    if not inputs_ok:
        return LinearAccResult(
            nan, nan, nan, False, "MISSING_ATTITUDE_OR_ACC",
            nan, nan, nan, nan, nan
        )

    raw_mag = math.sqrt(ax * ax + ay * ay + az * az)
    if _accel_input_mode() == "LINEAR":
        # 이미 중력 제거됨 → 추가 제거 금지.
        lin_x, lin_y, lin_z = ax, ay, az
        g_x = g_y = g_z = nan
    else:  # RAW: roll/pitch 기반 중력 제거
        roll  = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        cp    = math.cos(pitch)
        g_x   = -math.sin(pitch) * g
        g_y   =  cp * math.sin(roll) * g
        g_z   =  cp * math.cos(roll) * g
        lin_x = ax - g_x
        lin_y = ay - g_y
        lin_z = az - g_z
    xy_mag = math.hypot(lin_x, lin_y)

    reason = "OK"
    _use_age_gate = bool(getattr(config, "LIN_ACC_USE_SAMPLE_AGE_GATE", True))
    if (_use_age_gate and math.isfinite(sample_age_s)
            and sample_age_s > getattr(config, "LIN_ACC_SAMPLE_MAX_AGE_S", 0.10)):
        reason = "IMU_STALE"
    elif raw_mag > getattr(config, "RAW_ACC_NORM_MAX_MPS2", _RAW_ACC_MAX_MPS2):
        reason = "RAW_ACC_SPIKE"
    else:
        gyr_vals = (gyrx_deg_s, gyry_deg_s, gyrz_deg_s)
        if any(math.isfinite(float(v)) and abs(float(v)) > getattr(config, "ACC_GYR_REJECT_DPS", config.ACC_GYRZ_REJECT_DPS)
               for v in gyr_vals):
            reason = "GYRO_TOO_FAST"
        elif xy_mag > getattr(config, "LIN_ACC_XY_MAX_MPS2", config.ACC_LIMIT_MPS2):
            reason = "LIN_ACC_TOO_LARGE"

    return LinearAccResult(
        lin_x, lin_y, lin_z, reason == "OK", reason,
        raw_mag, g_x, g_y, g_z, xy_mag
    )


# ── 센서 핸들러 ───────────────────────────────────────────────────────────────

def handle_gps(data: str) -> None:
    global _ORIGIN_LOCKED
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

    # origin lock: candidate 없음. STATE >= 3(DESCENT 이상)에서 처음 들어오는 유효
    # GPS 좌표를 무조건 origin으로 잠근다. 이미 잠겼으면 덮어쓰지 않으므로 DESCENT
    # 진입 후 "첫 유효 좌표"만 origin이 된다.
    if (STATE >= 3 and not _ORIGIN_LOCKED
            and pos_health and math.isfinite(lat) and math.isfinite(lon)):
        guidance.set_origin_point(lat, lon)
        _ORIGIN_LOCKED = True
        prevstate.update_start_point(lat, lon, True)


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

        lin = _compute_linear_acc(
            roll_deg, pitch_deg, accx_mps2, accy_mps2, accz_mps2,
            gyrx_deg_s, gyry_deg_s, gyrz_deg_s, rx_ts - sample_ts,
        )
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
            lin_acc_x=lin.x if math.isfinite(lin.x) else None,
            lin_acc_y=lin.y if math.isfinite(lin.y) else None,
            lin_acc_z=lin.z if math.isfinite(lin.z) else None,
            lin_acc_valid=lin.valid,
            lin_acc_reject_reason=lin.reject_reason,
            raw_acc_norm_mps2=lin.raw_norm if math.isfinite(lin.raw_norm) else None,
            gravity_body_x_mps2=lin.gravity_x if math.isfinite(lin.gravity_x) else None,
            gravity_body_y_mps2=lin.gravity_y if math.isfinite(lin.gravity_y) else None,
            gravity_body_z_mps2=lin.gravity_z if math.isfinite(lin.gravity_z) else None,
            lin_acc_xy_mag_mps2=lin.xy_mag if math.isfinite(lin.xy_mag) else None,
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




def handle_flight_state(data: str) -> None:
    """비행 상태 업데이트.

    origin 재취득은 더 이상 state 전이에서 자동으로 하지 않는다. 다음 발사 전
    prevstate.json의 PREV_START_LOCKED를 0으로 수동 클리어한 뒤 전원을 재투입하면,
    init()이 origin을 복원하지 않고(_ORIGIN_LOCKED=False) handle_gps가 state≥3에서
    첫 유효 GPS로 새 origin을 잠근다.
    """
    global STATE, _PREV_STATE
    try:
        new_state = int(data.split(",")[0])
    except (ValueError, IndexError):
        return
    if new_state == STATE:
        return

    with _UPDATE_LOCK:
        _PREV_STATE = STATE
        STATE       = new_state


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
    global _STEER_MODE
    aliases = {
        "L": "LEFT", "LEFT": "LEFT",
        "N": "NEUTRAL", "NEUTRAL": "NEUTRAL",
        "R": "RIGHT", "RIGHT": "RIGHT",
    }
    mode = aliases.get(data.strip().upper())
    if mode is None:
        return
    _STEER_MODE = "" if mode == "NEUTRAL" else mode
    logger.info("MTR manual steer: %s → _MANUAL_STEER_MODE=%r", mode, _STEER_MODE)
    if mode == "NEUTRAL" and PI is not None:
        PI.set_servo_pulsewidth(control.PARAFOIL_LEFT_MOTOR_PIN,  control.LEFT_NEUTRAL)
        PI.set_servo_pulsewidth(control.PARAFOIL_RIGHT_MOTOR_PIN, control.RIGHT_NEUTRAL)


def handle_fac(data: str) -> None:
    """FAC 명령: 릴리즈/에그 액추에이터 직접 트리거 또는 비활성화.

    ON  → 해당 액추에이터 즉시 발동 (GPIO 직접 구동).
    OFF → 비행 로직에 의한 자동 트리거 차단 플래그 설정.
    형식: "ON"/"OFF" (둘 다) 또는 "ALL|REL|EGG,ON|OFF".
    """
    raw = data.strip().upper().replace(" ", "")
    parts = [p for p in raw.split(",") if p]
    if len(parts) == 1 and parts[0] in {"ON", "OFF"}:
        actor, state = "ALL", parts[0]
    elif len(parts) == 2 and parts[0] in {"ALL", "REL", "EGG"} and parts[1] in {"ON", "OFF"}:
        actor, state = parts[0], parts[1]
    else:
        return

    if state == "ON":
        if actor in {"ALL", "REL"}:
            handle_release("TRIGGER")
        if actor in {"ALL", "EGG"}:
            handle_egg_drop()
    else:
        if actor in {"ALL", "REL"}:
            try:
                from . import Motor_Release
                if hasattr(Motor_Release, "set_burnwire"):
                    Motor_Release.set_burnwire(False)
            except Exception:
                pass
        if actor in {"ALL", "EGG"}:
            try:
                from . import Motor_Egg
                if hasattr(Motor_Egg, "set_solenoid"):
                    Motor_Egg.set_solenoid(False)
            except Exception:
                pass

# ── 제어 루프 ─────────────────────────────────────────────────────────────────


def _emit_neutral(main_queue, state, motor_enabled, snap_t, now, *,
                  event: str, reason: str,
                  l1_in=None, l1_out=None) -> control.CtrlOutput:
    """중립 서보 출력 + 로그. guidance mode가 FAIL이거나 파이프라인이 invalid일 때 사용.

    motorapp은 자체 control mode/fallback을 만들지 않는다. event는 guidance mode
    문자열, reason은 invalid 세부 사유를 그대로 전달한다.
    """
    neutral_out = control.WriteNeutral(now, guidance._STATE_t.nav.control_mode)
    neutral_out.reason = reason
    control.MoveServo(PI, neutral_out)
    sensorlog.log_motor_raw(
        state, motor_enabled, MOTOR_CTRL_MODE, neutral_out, l1_out,
        snap=snap_t, event=event, l1_in=l1_in,
    )
    _publish_motor_diag(main_queue, neutral_out, snap_t, event)
    return neutral_out


def _ctrl_cycle(main_queue, now: float) -> Optional[control.CtrlOutput]:
    """한 사이클 제어 계산. 반환값은 진단용 (None이면 액션 없음)."""
    with _UPDATE_LOCK:
        motor_enabled = MOTOR_ENABLED
        state         = STATE
        snap_t        = _cache_snapshot()

    # 비활성 또는 비행 전 상태
    if not motor_enabled or state < 4:   # PAYLOAD_RELEASE(4) 이전: 서보 중립
        if PI is not None:
            control.WriteZero(PI)
        zero_out = control.CtrlOutput(
            timestamp=now,
            left_pw=control.LEFT_ZERO_PULSE,
            right_pw=control.RIGHT_ZERO_PULSE,
            left_angle_deg=control.ARM_MIN_DEG,
            right_angle_deg=control.ARM_MIN_DEG,
            valid=False,
            control_mode=guidance.ControlMode.FAIL,
        )
        event = "MOTOR_DISABLED" if not motor_enabled else "STATE_BELOW_4"
        sensorlog.log_motor_raw(state, motor_enabled, MOTOR_CTRL_MODE, zero_out, snap=snap_t, event=event)
        return None

    # 착지 후 (state 6 = LANDED)
    if state == 6:
        if PI is not None:
            control.WriteOff(PI)
        off_out = control.CtrlOutput(
            timestamp=now,
            left_pw=0,
            right_pw=0,
            valid=False,
            control_mode=guidance.ControlMode.FAIL,
        )
        sensorlog.log_motor_raw(state, motor_enabled, MOTOR_CTRL_MODE, off_out, snap=snap_t, event="LANDED_OFF")
        return None

    # ── [1] MANUAL STEER override ────────────────────────────────────────────
    if _STEER_MODE in ("LEFT", "RIGHT", "NEUTRAL"):
        _delta = {
            "LEFT":    -config.MANUAL_STEER_DELTA_DEG,
            "RIGHT":   +config.MANUAL_STEER_DELTA_DEG,
            "NEUTRAL":  0.0,
        }[_STEER_MODE]
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
        sensorlog.log_motor_raw(
            state, motor_enabled, MOTOR_CTRL_MODE, manual_out,
            snap=snap_t, event=f"MANUAL_{_STEER_MODE}"
        )
        _publish_motor_diag(main_queue, manual_out, snap_t, f"MANUAL_{MOTOR_CTRL_MODE}")
        return manual_out

    # ── guidance 기반 자율 추종 (유일한 제어 경로) ───────────────────────────
    # 모든 control mode는 guidance.DecideControlMode만 만든다. motorapp은 자체
    # control mode나 fallback/heading 분기를 만들지 않는다. FAIL이면 ProduceL1Input이
    # invalid를 반환하고 곧장 neutral로 간다 (event=mode.value, reason=세부 사유).
    mode = guidance.DecideControlMode(
        snap_t.latest_gps, snap_t.latest_imu, snap_t.latest_baro, now)
    event = mode.value

    l1_in_t = guidance.ProduceL1Input(now)
    if not l1_in_t.valid:
        return _emit_neutral(main_queue, state, motor_enabled, snap_t, now,
                             event=event, reason=l1_in_t.reason, l1_in=l1_in_t)

    l1_out_t = guidance.ProduceL1Output(l1_in_t)
    if not l1_out_t.valid:
        return _emit_neutral(main_queue, state, motor_enabled, snap_t, now,
                             event=event, reason=l1_out_t.reason,
                             l1_in=l1_in_t, l1_out=l1_out_t)

    gz_meas = snap_t.latest_imu.gyrz_rad_s or 0.0
    gz_meas = math.degrees(float(gz_meas))   # rad/s → deg/s (control.step 기대 단위)
    ctrl_in_t  = control.ProduceCtrlInput(l1_out_t, now)
    ctrl_out_t = control.ProduceCtrlOutput(ctrl_in_t, gz_meas, now)
    # control invalid이면 step이 neutral 펄스를 반환하므로 MoveServo가 곧 neutral이다.
    control.MoveServo(PI, ctrl_out_t)
    sensorlog.log_motor_raw(
        state, motor_enabled, MOTOR_CTRL_MODE, ctrl_out_t, l1_out_t,
        snap=snap_t, event=event, l1_in=l1_in_t,
    )
    _publish_motor_diag(main_queue, ctrl_out_t, snap_t, event)
    return ctrl_out_t


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
    global STATE, _ORIGIN_LOCKED

    prevstate.init_prevstate()
    # Restore flight state so _ctrl_cycle is not blocked on the first cycle.
    # Without this, STATE stays 0 until flightlogicapp sends MID_motor_state.
    STATE = prevstate.PREV_STATE
    MOTOR_ENABLED = prevstate.is_motor_enabled()
    RELEASE_ACTION_ENABLED = True
    EGG_ACTION_ENABLED = True
    _ORIGIN_LOCKED = False
    guidance.set_target_point(TARGET_LAT, TARGET_LON)

    # origin 복원 (PREV_START_LOCKED==1일 때만 반환)
    # set_origin_point()은 frame origin과 현재 GPS local point만 초기화한다.
    start = prevstate.get_start_point()
    if start is not None:
        lat, lon = start
        if (-90.0 <= float(lat) <= 90.0
                and -180.0 <= float(lon) <= 180.0
                and not (lat == 0.0 and lon == 0.0)):
            guidance.set_origin_point(float(lat), float(lon))
            _ORIGIN_LOCKED = True
            logger.info("Origin restored from prevstate: lat=%.6f lon=%.6f", lat, lon)

    control.reset()
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

    if not MOTOR_ENABLED:
        logger.warning(
            "MOTOR DISABLED (prevstate). 서보 출력 없음 — 비행 전 'MEC ON' 명령 필요."
        )
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
