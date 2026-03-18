import os
import signal
import threading
import types
from datetime import datetime
from multiprocessing import connection
from lib import appargs, msgstructure, events
from Sensor_Motor import motor_guidance, motor_control, Motor_Release, Motor_Egg


log_dir = "./sensorlogs"
if not os.path.exists(log_dir):
    os.makedirs(log_dir)
controllogfile = open(os.path.join(log_dir, "control.txt"), "a")


def log_control(g, m):
    t = datetime.now().isoformat(sep=" ", timespec="milliseconds")

    if m is None:
        m = types.SimpleNamespace(
            left_cmd_deg=0.0, right_cmd_deg=0.0,
            actual_delta_deg=0.0, expected_yaw_rate=0.0,
            left_pulse=0, right_pulse=0
        )

    line = (
        f"{t},"
        f"state:{g.state},"
        f"alt:{g.altitude:.1f},"
        f"L:{g.l_distance:.1f},"
        f"pat:{int(g.patterned)},"
        f"pat_E:{g.pattern_wp_E:.1f},"
        f"pat_N:{g.pattern_wp_N:.1f},"
        f"dist:{g.distance:.2f},"
        f"d_crs:{g.desired_course:.1f},"
        f"d_hdg:{g.desired_heading:.1f},"
        f"h_err:{g.heading_error:.2f},"
        f"d_yr:{g.desired_yaw_rate:.2f},"
        f"m_yr:{g.measured_yaw_rate:.2f},"
        f"u_pre:{g.u_before_sat:.2f},"
        f"u_post:{g.u_after_sat:.2f},"
        f"integ:{g.integral:.3f},"
        f"wind:{g.wind_effect:.2f},"
        f"L_deg:{m.left_cmd_deg:.1f},"
        f"R_deg:{m.right_cmd_deg:.1f},"
        f"delta:{m.actual_delta_deg:.1f},"
        f"exp_yr:{m.expected_yaw_rate:.2f},"
        f"L_pw:{m.left_pulse},"
        f"R_pw:{m.right_pulse},"
        f"yaw:{g.yaw:.1f},"
        f"spd:{g.gps_speed:.2f},"
        f"crs:{g.gps_course:.1f}\n"
    )
    controllogfile.write(line)
    controllogfile.flush()


running = True
motor_enabled = True
patterned = False
pi = None
target = types.SimpleNamespace(lat=0.0, lon=0.0)

altitude = types.SimpleNamespace(yaw=0.0, gyrz=0.0)
GpsVector = types.SimpleNamespace(lat=0.0, lon=0.0, speed=0.0, course=0.0)
GpsFidelity = types.SimpleNamespace(rmc_status="V", fix_quality=0, sats=0)

state = 0

threads: dict[str, threading.Thread] = {}
update_lock = threading.Lock()
CONTROL_LOG_INTERVAL = 0.1

APP = appargs.MotorAppArg.AppName


def log(msg: str, level=events.EventType.info):
    events.LogEvent(APP, level, msg)


def handle_terminate(data: str):
    global running
    log("Termination detected")
    running = False


def handle_gps(data: str):
    parts = data.split(",")
    if len(parts) == 7:
        GpsVector.lat    = float(parts[0])
        GpsVector.lon    = float(parts[1])
        GpsVector.speed  = float(parts[2])
        GpsVector.course = float(parts[3])
        GpsFidelity.fix_quality = int(parts[4])
        GpsFidelity.sats        = int(parts[5])
        GpsFidelity.rmc_status  = parts[6]
    else:
        log("GPS data format error", events.EventType.error)


def handle_imu(data: str):
    parts = data.split(",")
    if len(parts) == 2:
        altitude.yaw  = float(parts[0])
        altitude.gyrz = float(parts[1])
    else:
        log("IMU data format error", events.EventType.error)


def handle_barometer(data: str):
    try:
        parts = data.split(",")
        if len(parts) >= 3:
            alt = float(parts[2])
        else:
            alt = float(parts[0])
        motor_guidance.update_altitude(alt)
    except (ValueError, IndexError):
        log("Barometer data format error", events.EventType.error)


def handle_target_coord(data: str):
    parts = data.split(",")
    if len(parts) == 2:
        target.lat, target.lon = float(parts[0]), float(parts[1])
        motor_guidance.set_target_coord(target.lat, target.lon)
        log(f"Target set: ({target.lat:.6f}, {target.lon:.6f})")
    else:
        log("Target coords format error", events.EventType.error)


def handle_flight_state(data: str):
    global state, patterned
    state = int(data)
    if state == 4:
        patterned = False
    log(f"Flight state: {state}")


def handle_pull_arms(data=None):
    global patterned
    patterned = True
    log("Pattern mode activated (figure-eight)")


def handle_release():
    log("Activating burnwire")
    Motor_Release.activate_burnwire()


def handle_egg_drop():
    log("Activating solenoid")
    Motor_Egg.activate_solenoid()


def handle_mec(data: str):
    global motor_enabled
    log(f"MEC command: {data}")
    if data == "ON":
        motor_enabled = True
    elif data == "OFF":
        motor_enabled = False
    else:
        log(f"Invalid MEC option: {data}", events.EventType.error)


MSG_HANDLERS = {
    appargs.MainAppArg.MID_TerminateProcess:       handle_terminate,
    appargs.GpsAppArg.MID_motor_gps:               handle_gps,
    appargs.ImuAppArg.MID_motor_imu:               handle_imu,
    appargs.BarometerAppArg.MID_motor_alt:         handle_barometer,
    appargs.FlightlogicAppArg.MID_motor_TargetCor: handle_target_coord,
    appargs.FlightlogicAppArg.MID_motor_state:     handle_flight_state,
    appargs.FlightlogicAppArg.MID_motor_burnwire:  lambda d: handle_release(),
    appargs.FlightlogicAppArg.MID_motor_EggDrop:   lambda d: handle_egg_drop(),
    appargs.FlightlogicAppArg.MID_motor_PullArms:  lambda d: handle_pull_arms(),
    appargs.CommAppArg.MID_RouteCmd_MEC:           handle_mec,
}


def dispatch(msg: msgstructure.MsgStructure):
    handler = MSG_HANDLERS.get(msg.MsgID)
    if handler:
        handler(msg.data)
    else:
        log(f"Unknown MID: {msg.MsgID}", events.EventType.error)


def control_parafoil():
    import time
    while running:
        with update_lock:
            if state >= 3 and motor_enabled and pi is not None:

                result = motor_guidance.guidance(
                    altitude, GpsVector, GpsFidelity, target, patterned=patterned
                )

                motor_result = motor_control.apply_differential_deflection(
                    pi, result.commanded_yaw_rate
                )

                log_control(result, motor_result)

                if state == 5:
                    log("Stopping motors", events.EventType.warning)
                    motor_control.set_neutral(pi)

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
