import os
import signal
import threading
import time
import types
from datetime import datetime
from multiprocessing import connection
from lib import appargs, msgstructure, events
from Sensor_Motor import motor_guidance, motor_control, Motor_Release, Motor_Egg


log_dir = "./sensorlogs"
if not os.path.exists(log_dir):
    os.makedirs(log_dir)
controllogfile = open(os.path.join(log_dir, "control.txt"), "a")
simlogfile = open("0320_sim.txt", "a")

def _dbg(line: str):
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    full = f"[{ts}] {line}"
    print(full)
    simlogfile.write(full + "\n")
    simlogfile.flush()

def log_control(g, m):
    t = datetime.now().isoformat(sep=" ", timespec="milliseconds")

    if m is None:
        m = types.SimpleNamespace(
            left_cmd_deg=0.0, right_cmd_deg=0.0,
            actual_delta_deg=0.0, expected_yaw_rate=0.0,
            left_pulse=0, right_pulse=0
        )

    # g 데이터는 state, distance, commanded_yaw_rate 3개만 존재함
    line = (
        f"{t},"
        f"state:{g.state},"
        f"dist:{g.distance:.2f},"
        f"cmd_yr:{g.commanded_yaw_rate:.2f},"
        f"L_deg:{m.left_cmd_deg:.1f},"
        f"R_deg:{m.right_cmd_deg:.1f},"
        f"L_pw:{m.left_pulse},"
        f"R_pw:{m.right_pulse}\n"
    )
    controllogfile.write(line)
    controllogfile.flush()

running = True
motor_enabled = True
pi = None
target = types.SimpleNamespace(lat=0.0, lon=0.0)

altitude = types.SimpleNamespace(yaw=0.0, gyrz=0.0)
baro_m = 0.0
GpsVector = types.SimpleNamespace(lat=0.0, lon=0.0, speed=0.0, course=0.0)
GpsFidelity = types.SimpleNamespace(rmc_status="V", fix_quality=0, sats=0)

state = 0

# ── [FIX-1] Stale 데이터 감지용 타임스탬프 ──
last_gps_time = 0.0
last_imu_time = 0.0
STALE_THRESHOLD = 1.5  # 1.5초 이상 갱신 없으면 stale 판정
# ── [/FIX-1] ──

# ── [FIX-GYRZ] gyrz 스파이크 게이트 ──
GYRZ_SPIKE_THRESHOLD = 45.0  # °/s — 틱 간 최대 허용 델타
_prev_gyrz = 0.0
# ── [/FIX-GYRZ] ──

# ── [FDIR] Estimation Rate Error — 제어 불능 판정 임계값 ──
GYRZ_RUNAWAY_THRESHOLD = 100.0  # °/s — 이 이상이면 센서 오류 또는 제어 불능
# ── [/FDIR] ──

threads: dict[str, threading.Thread] = {}
update_lock = threading.Lock()
CONTROL_LOG_INTERVAL = 0.1
DEBUG_GUIDANCE = True  # guidance 디버그 프린트 on/off

APP = appargs.MotorAppArg.AppName


def log(msg: str, level=events.EventType.info):
    events.LogEvent(APP, level, msg)


def handle_terminate(data: str):
    global running
    log("Termination detected")
    running = False


def handle_gps(data: str):
    global last_gps_time
    parts = data.split(",")
    if len(parts) == 7:
        # ── [FIX-4] 핸들러에도 lock 적용 ──
        with update_lock:
            GpsVector.lat    = float(parts[0])
            GpsVector.lon    = float(parts[1])
            GpsVector.speed  = float(parts[2])
            GpsVector.course = float(parts[3])
            GpsFidelity.fix_quality = int(parts[4])
            GpsFidelity.sats        = int(parts[5])
            GpsFidelity.rmc_status  = parts[6]
            last_gps_time = time.time()  # [FIX-1] 수신 시각 기록
        # ── [/FIX-4] ──
    else:
        log("GPS data format error", events.EventType.error)


def handle_imu(data: str):
    global last_imu_time, _prev_gyrz
    parts = data.split(",")
    if len(parts) == 2:
        with update_lock:
            new_yaw  = float(parts[0])
            new_gyrz = float(parts[1])
            # ── [FIX-GYRZ] ──
            if abs(new_gyrz - _prev_gyrz) <= GYRZ_SPIKE_THRESHOLD:
                altitude.gyrz = new_gyrz
                _prev_gyrz = new_gyrz
            else:
                log(f"gyrz spike rejected: {new_gyrz:.2f} deg/s (prev={_prev_gyrz:.2f})",
                    events.EventType.warning)
            # ── [/FIX-GYRZ] ──
            altitude.yaw  = new_yaw
            last_imu_time = time.time()
    else:
        log("IMU data format error", events.EventType.error)


def handle_barometer(data: str):
    global baro_m
    try:
        parts = data.split(",")
        # ── [FIX-4] 핸들러에도 lock 적용 ──
        with update_lock:
            baro_m = float(parts[0])
        # ── [/FIX-4] ──
    except (ValueError, IndexError):
        log("Barometer data format error", events.EventType.error)


def handle_target_coord(data: str):
    parts = data.split(",")
    if len(parts) == 2:
        # ── [FIX-4] 핸들러에도 lock 적용 ──
        with update_lock:
            target.lat, target.lon = float(parts[0]), float(parts[1])
        # ── [/FIX-4] ──
        motor_guidance.set_target_coord(target.lat, target.lon)
        log(f"Target set: ({target.lat:.6f}, {target.lon:.6f})")
    else:
        log("Target coords format error", events.EventType.error)


def handle_flight_state(data: str):
    global state
    # ── [FIX-4] 핸들러에도 lock 적용 ──
    with update_lock:
        state = int(data)
        if state == 3:
            motor_guidance.set_start_coordinates(GpsVector.lat, GpsVector.lon)
    # ── [/FIX-4] ──
    log(f"Flight state: {state}")


def handle_release():
    log("Activating burnwire")
    Motor_Release.activate_burnwire()


def handle_egg_drop():
    log("Activating solenoid")
    Motor_Egg.activate_solenoid()


def handle_mec(data: str):
    global motor_enabled
    log(f"MEC command: {data}")
    # ── [FIX-4] 핸들러에도 lock 적용 ──
    with update_lock:
        if data == "ON":
            motor_enabled = True
        elif data == "OFF":
            motor_enabled = False
        else:
            log(f"Invalid MEC option: {data}", events.EventType.error)
    # ── [/FIX-4] ──


MSG_HANDLERS = {
    appargs.MainAppArg.MID_TerminateProcess:       handle_terminate,
    appargs.GpsAppArg.MID_motor_gps:               handle_gps,
    appargs.ImuAppArg.MID_motor_imu:               handle_imu,
    appargs.BarometerAppArg.MID_motor_alt:         handle_barometer,
    appargs.FlightlogicAppArg.MID_motor_TargetCor: handle_target_coord,
    appargs.FlightlogicAppArg.MID_motor_state:     handle_flight_state,
    appargs.FlightlogicAppArg.MID_motor_burnwire:  lambda d: handle_release(),
    appargs.FlightlogicAppArg.MID_motor_EggDrop:   lambda d: handle_egg_drop(),
    appargs.CommAppArg.MID_RouteCmd_MEC:           handle_mec,
}


def dispatch(msg: msgstructure.MsgStructure):
    handler = MSG_HANDLERS.get(msg.MsgID)
    if handler:
        handler(msg.data)
    else:
        log(f"Unknown MID: {msg.MsgID}", events.EventType.error)


def _resolve_patterned(flight_state: int, alt_m: float) -> bool:
    """고도와 state로 8자 비행 여부 결정."""
    if flight_state == 4:
        return alt_m > 10.0   # EGG: 10m 초과 → 8자, 10m 이하 → 당근 (Final)
    return False               # state 3: 당근 제어 (호밍)


def control_parafoil():
    motors_off = False
    while running:
        # ── [FIX-4] lock 범위를 스냅샷 복사로 최소화 ──
        with update_lock:
            _state       = state
            _motor_en    = motor_enabled
            _baro_m      = baro_m
            _gps         = types.SimpleNamespace(
                lat=GpsVector.lat, lon=GpsVector.lon,
                speed=GpsVector.speed, course=GpsVector.course)
            _fidelity    = types.SimpleNamespace(
                rmc_status=GpsFidelity.rmc_status,
                fix_quality=GpsFidelity.fix_quality,
                sats=GpsFidelity.sats)
            _imu         = types.SimpleNamespace(
                yaw=altitude.yaw, gyrz=altitude.gyrz)
            _target      = types.SimpleNamespace(
                lat=target.lat, lon=target.lon)
            _last_gps_t  = last_gps_time
            _last_imu_t  = last_imu_time
        # ── [/FIX-4] ──

        if _state >= 3 and _motor_en:

            # State 5: 서보 신호 완전 차단 후 루프 유지 (재진입 방지)
            if _state == 5:
                if not motors_off:
                    motor_control.set_motors_off(pi)
                    motors_off = True
                    log("State 5: motors off", events.EventType.warning)
                time.sleep(CONTROL_LOG_INTERVAL)
                continue

            motors_off = False

            # ================================================================
            #  FDIR (Fault Detection, Isolation & Recovery) 게이트
            #  ── ADCS OBC Communications Watchdog / Failsafe 철학 적용 ──
            #
            #  정상 제어(guidance) 진입 전에 모든 이상 조건을 일괄 검사한다.
            #  하나라도 이상이 감지되면 Failsafe(모터 중립)로 진입하고,
            #  모든 검사를 통과한 경우에만 정상 제어 경로를 탄다.
            # ================================================================
            failsafe_reason = None  # None이면 정상, 문자열이면 Failsafe 진입

            # ── FDIR-1: OBC Watchdog — 센서 통신 타임아웃 감지 ──
            # ADCS에서 OBC-ADCS 간 Heartbeat/Watchdog이 timeout되면
            # ConNone(무제어) 모드로 전환하듯, 센서 데이터가 STALE_THRESHOLD
            # 이상 갱신되지 않으면 통신 두절로 간주한다.
            now = time.time()
            gps_stale = (now - _last_gps_t) > STALE_THRESHOLD if _last_gps_t > 0 else True
            imu_stale = (now - _last_imu_t) > STALE_THRESHOLD if _last_imu_t > 0 else True

            if gps_stale and imu_stale:
                failsafe_reason = "Sensor timeout: GPS+IMU stale (Watchdog)"
            elif gps_stale:
                failsafe_reason = "Sensor timeout: GPS stale (Watchdog)"
            elif imu_stale:
                failsafe_reason = "Sensor timeout: IMU stale (Watchdog)"

            # ── FDIR-2: Data Integrity — GPS 데이터 무결성 검증 ──
            # ADCS가 OrbitError(궤도 데이터 이상) 감지 시 Safe-mode로
            # 전환하듯, GPS 좌표·위성 수·Fix 품질이 신뢰 기준 미달이면
            # 유도 계산 자체가 위험하므로 Failsafe 진입.
            if failsafe_reason is None:
                if not motor_guidance.is_gps_valid(
                    _gps.lat, _gps.lon,
                    _fidelity.fix_quality, _fidelity.sats,
                    _fidelity.rmc_status
                ):
                    failsafe_reason = (
                        f"Data integrity: GPS invalid "
                        f"(lat={_gps.lat}, lon={_gps.lon}, "
                        f"fix={_fidelity.fix_quality}, sats={_fidelity.sats}, "
                        f"rmc={_fidelity.rmc_status})"
                    )

            # ── FDIR-3: Estimation Rate Error — 극한 회전 상태 차단 ──
            # ADCS가 자세 추정기(Estimator)의 각속도 오차가 임계를 초과하면
            # 제어 불능으로 판단하여 Failsafe 전환하듯, gyrz가 비현실적으로
            # 높으면 센서 오류이거나 텀블링 상태이므로 즉시 차단한다.
            if failsafe_reason is None:
                if abs(_imu.gyrz) > GYRZ_RUNAWAY_THRESHOLD:
                    failsafe_reason = (
                        f"Estimation rate error: |gyrz|={abs(_imu.gyrz):.1f} deg/s "
                        f"> {GYRZ_RUNAWAY_THRESHOLD} (runaway)"
                    )

            # ── FDIR-4: Baro Altitude Sanity — 기압계 고도 방어 ──
            # 기압 고도가 0 이하이면 센서 미초기화 또는 오류로 간주.
            if failsafe_reason is None:
                if _baro_m <= 0.0:
                    failsafe_reason = (
                        f"Data integrity: Baro altitude invalid ({_baro_m:.1f}m)"
                    )

            # ── Failsafe 분기: 이상 감지 → 모터 중립 / 정상 → 유도 제어 ──
            if failsafe_reason is not None:
                # === FAILSAFE MODE (Safe-mode Action) ===
                # ADCS Timeout 시 ConNone으로 복귀하듯, 모터를 중립으로 고정하여
                # 잘못된 데이터로 인한 위험한 조종면 명령을 원천 차단한다.
                log(f"Failsafe triggered: {failsafe_reason}", events.EventType.error)
                motor_control.set_neutral(pi)
            else:
                # === NOMINAL CONTROL PATH ===
                # 모든 FDIR 검증을 통과 — 정상 유도/제어 수행
                _patterned = _resolve_patterned(_state, _baro_m)

                if DEBUG_GUIDANCE:
                    _dbg(
                        f"[GUIDANCE IN ] "
                        f"state={_state} baro={_baro_m:.1f}m patterned={_patterned} | "
                        f"yaw={_imu.yaw:.1f}° gyrz={_imu.gyrz:.2f} | "
                        f"gps=({_gps.lat:.6f},{_gps.lon:.6f}) spd={_gps.speed:.1f} crs={_gps.course:.1f} | "
                        f"fix={_fidelity.fix_quality} sats={_fidelity.sats} rmc={_fidelity.rmc_status} | "
                        f"target=({_target.lat:.6f},{_target.lon:.6f})"
                    )

                result = motor_guidance.guidance(
                    _imu, _gps, _fidelity, _target,
                    baro_m=_baro_m, patterned=_patterned
                )

                motor_result = motor_control.control(
                    pi, result.commanded_yaw_rate
                )

                if DEBUG_GUIDANCE and motor_result is not None:
                    _dbg(
                        f"[MOTOR] "
                        f"L: {motor_result.left_cmd_deg:6.1f}°  pw={motor_result.left_pulse} | "
                        f"R: {motor_result.right_cmd_deg:6.1f}°  pw={motor_result.right_pulse} | "
                        f"delta={motor_result.actual_delta_deg:+.1f}°  exp_yr={motor_result.expected_yaw_rate:+.2f}°/s"
                    )
                    simlogfile.write("---\n")
                    simlogfile.flush()

                log_control(result, motor_result)

        time.sleep(CONTROL_LOG_INTERVAL)


def init() -> bool:
    global pi, running

    signal.signal(signal.SIGINT, signal.SIG_IGN)
    log("Initializing motorapp")

    try:
        Motor_Release.init_burnwire()
        motor_guidance.init_guidance()
        pi = motor_control.init_control()
        Motor_Egg.init_solenoid()
        threads["ControlLog_Thread"] = threading.Thread(
            target=control_parafoil,
            name="ControlLog_Thread",
            daemon=True
        )
        threads["ControlLog_Thread"].start()
        log("Motors initialized (parafoil, burnwire, solenoid)")
        return True
    except Exception as e:
        log(f"Init failed: {e}", events.EventType.error)
        running = False
        return False


def terminate():
    global running
    running = False
    log("Terminating motorapp")

    try:
        controllogfile.close()
    except Exception:
        pass

    if pi:
        motor_control.terminate_parafoil_motor(pi)

    Motor_Release.terminate_burnwire()
    Motor_Egg.terminate_solenoid()

    for name, thread in threads.items():
        log(f"Joining thread: {name}")
        thread.join()

    log("Motorapp terminated")


def motorapp_main(main_pipe: connection.Connection):
    global running
    running = True

    if not init():
        return

    try:
        while running:
            recv_msg = main_pipe.recv()
            unpacked_msg = msgstructure.unpack_msg(recv_msg)

            if unpacked_msg == False:
                continue

            if unpacked_msg.receiver_app in (appargs.MotorAppArg.AppID,
                                              appargs.MainAppArg.AppID):
                dispatch(unpacked_msg)

    except Exception as e:
        log(f"Error: {e}", events.EventType.error)

    finally:
        terminate()
