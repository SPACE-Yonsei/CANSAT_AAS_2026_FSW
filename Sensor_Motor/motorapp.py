"""
Parafoil Motor Application
===========================

Top-level application that integrates guidance, control, and hardware
for the autonomous parafoil system.

Responsibilities:
  - Receives sensor data (GPS, IMU, barometer) and commands via message pipes.
  - Runs a 10 Hz control loop that calls guidance and actuator control.
  - Logs the full control pipeline for post-flight analysis.
  - Manages hardware lifecycle (servo init / terminate, burnwire, solenoid).

Message flow:
  GPS app     --[MID_motor_gps]-->     motorapp -> GpsVector / GpsFidelity
  IMU app     --[MID_motor_imu]-->     motorapp -> altitude (imu_data)
  Barometer   --[MID_motor_alt]-->     motorapp -> motor_guidance.update_altitude()
  FlightLogic --[MID_motor_state]-->   motorapp -> state transitions
  FlightLogic --[MID_motor_TargetCor]->motorapp -> target coordinates
  FlightLogic --[MID_motor_PullArms]-->motorapp -> patterned = True
  COMM        --[MID_RouteCmd_MEC]-->  motorapp -> motor enable/disable

Control pipeline (per cycle):
  1. motor_guidance.guidance()        -> commanded_yaw_rate + metadata
  2. motor_control.apply_differential_deflection() -> servo commands + log data
  3. log_control()                    -> write all fields to control.txt
"""
import os
import signal
import threading
import types
from datetime import datetime
from multiprocessing import connection
from lib import appargs, msgstructure, events
from Sensor_Motor import motor_guidance, motor_control, Motor_Release, Motor_Egg


# =============================================================================
# Control Log (control.txt)
# =============================================================================
# The control log captures every guidance + actuator cycle for post-flight
# analysis.  Each line is a CSV-style record with a millisecond timestamp.
# Fields cover the full pipeline: guidance -> controller -> mixer -> servo.

log_dir = "./sensorlogs"
if not os.path.exists(log_dir):
    os.makedirs(log_dir)
controllogfile = open(os.path.join(log_dir, "control.txt"), "a")


def log_control(g, m):
    """Write one line of control telemetry to control.txt.

    Captures the complete state of the guidance-to-actuator pipeline in
    a single CSV line for post-flight analysis.  Fields include:

      Timing:      timestamp
      Phase:       state, altitude, L_DISTANCE, patterned flag
      Pattern:     pattern waypoint (E, N)
      Navigation:  distance, desired_course, desired_heading, heading_error
      Controller:  desired_yaw_rate, measured_yaw_rate, u_before/after_sat, integral
      Wind:        wind_effect (crab angle estimate)
      Actuator:    left/right arm angles, actual delta, expected yaw rate
      Servo:       left/right pulse widths
      Sensors:     yaw, GPS speed, GPS course

    Args:
        g: guidance result namespace (from motor_guidance.guidance())
        m: motor result namespace (from motor_control.apply_differential_deflection())
           May be None if motors were not commanded this cycle.
    """
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


# =============================================================================
# State Variables
# =============================================================================

running = True
motor_enabled = True
patterned = False  # True -> figure-eight pattern mode; False -> direct homing
pi = None  # pigpio instance
target = types.SimpleNamespace(lat=0.0, lon=0.0)

# Sensor data (updated by message handlers)
# NOTE: 'altitude' is named for legacy compatibility; it holds IMU data (yaw, gyrz)
altitude = types.SimpleNamespace(yaw=0.0, gyrz=0.0)
GpsVector = types.SimpleNamespace(lat=0.0, lon=0.0, speed=0.0, course=0.0)
GpsFidelity = types.SimpleNamespace(rmc_status="V", fix_quality=0, sats=0)

# Flight states:
#   0=LAUNCHPAD, 1=ASCENT, 2=APOGEE, 3=DESCENT, 4=EGG_RELEASE, 5=LANDED
state = 0

threads: dict[str, threading.Thread] = {}
update_lock = threading.Lock()
CONTROL_LOG_INTERVAL = 0.1  # 10 Hz control / logging rate

APP = appargs.MotorAppArg.AppName


def log(msg: str, level=events.EventType.info):
    events.LogEvent(APP, level, msg)


# =============================================================================
# Message Handlers
# =============================================================================

def handle_terminate(data: str):
    """Handle system-wide termination signal."""
    global running
    log("Termination detected")
    running = False


def handle_gps(data: str):
    """Parse GPS message: lat,lon,speed,course,fix_quality,sats,rmc_status

    Updates GpsVector and GpsFidelity which are read by the control loop.
    """
    parts = data.split(",")
    if len(parts) == 7:
        GpsVector.lat    = float(parts[0])
        GpsVector.lon    = float(parts[1])
        GpsVector.speed  = float(parts[2])  # m/s
        GpsVector.course = float(parts[3])
        GpsFidelity.fix_quality = int(parts[4])
        GpsFidelity.sats        = int(parts[5])
        GpsFidelity.rmc_status  = parts[6]
    else:
        log("GPS data format error", events.EventType.error)


def handle_imu(data: str):
    """Parse IMU message: yaw,gyrz

    yaw:  heading angle [deg]
    gyrz: yaw rate [deg/s] (verify units match guidance expectations)
    """
    parts = data.split(",")
    if len(parts) == 2:
        altitude.yaw  = float(parts[0])
        altitude.gyrz = float(parts[1])
    else:
        log("IMU data format error", events.EventType.error)


def handle_barometer(data: str):
    """Parse barometer message and forward altitude to guidance module.

    Updates the altitude-adaptive L_DISTANCE in motor_guidance and enables
    final-approach detection.  The barometer app should send altitude [m AGL]
    to this handler via MID_motor_alt.

    Supports two formats for robustness:
      - "pressure,temperature,altitude" (3-field telemetry format)
      - "altitude" (single value)

    NOTE: For this handler to receive data, the barometer application must
    be configured to send messages with MID_motor_alt (1301901) to the
    motor app.  Add the following to barometerapp.py's send routine:
      send_msg(BarometerAppArg.MID_motor_alt, MotorAppArg.AppID, str(ALTITUDE))
    """
    try:
        parts = data.split(",")
        if len(parts) >= 3:
            alt = float(parts[2])  # third field is altitude
        else:
            alt = float(parts[0])  # single value = altitude directly
        motor_guidance.update_altitude(alt)
    except (ValueError, IndexError):
        log("Barometer data format error", events.EventType.error)


def handle_target_coord(data: str):
    """Parse target coordinate message: lat,lon

    Forwards to guidance module for waypoint computation.
    """
    parts = data.split(",")
    if len(parts) == 2:
        target.lat, target.lon = float(parts[0]), float(parts[1])
        motor_guidance.set_target_coord(target.lat, target.lon)
        log(f"Target set: ({target.lat:.6f}, {target.lon:.6f})")
    else:
        log("Target coords format error", events.EventType.error)


def handle_flight_state(data: str):
    """Handle flight state transitions from flightlogic.

    States: 0=LAUNCHPAD, 1=ASCENT, 2=APOGEE, 3=DESCENT, 4=EGG_RELEASE, 5=LANDED

    State 4 (EGG_RELEASE): disables pattern mode so the parafoil switches
    from energy-management figure-eight back to direct homing / final approach.
    """
    global state, patterned
    state = int(data)
    if state == 4:
        patterned = False
    log(f"Flight state: {state}")


def handle_pull_arms(data=None):
    """Activate pattern mode (figure-eight near target).

    Sent by flightlogic when the parafoil should begin energy-management
    pattern flight near the target.  The guidance module will generate
    figure-eight waypoints until patterned is cleared (by EGG_RELEASE
    state or low-altitude final approach).
    """
    global patterned
    patterned = True
    log("Pattern mode activated (figure-eight)")


def handle_release():
    """Activate burnwire for parafoil release mechanism."""
    log("Activating burnwire")
    Motor_Release.activate_burnwire()


def handle_egg_drop():
    """Activate solenoid for egg drop mechanism."""
    log("Activating solenoid")
    Motor_Egg.activate_solenoid()


def handle_mec(data: str):
    """Handle Motor Enable/Control command from ground station.

    ON  -> enable motor control (guidance + actuation active)
    OFF -> disable motor control (servos hold last position)
    """
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
    """Route an incoming message to the appropriate handler by MID.

    Looks up msg.MsgID in MSG_HANDLERS and calls the registered function
    with msg.data as the argument.
    """
    handler = MSG_HANDLERS.get(msg.MsgID)
    if handler:
        handler(msg.data)
    else:
        log(f"Unknown MID: {msg.MsgID}", events.EventType.error)


# =============================================================================
# Parafoil Control Loop
# =============================================================================
def control_parafoil():
    """10 Hz control loop: guidance -> actuator -> log.

    Runs in a dedicated daemon thread.  Each cycle:
      1. Checks preconditions (state >= DESCENT, motor enabled, pi valid).
      2. Calls motor_guidance.guidance() with current sensor data and
         the patterned flag.  The guidance function internally handles
         homing / pattern / final-approach selection.
      3. Calls motor_control.apply_differential_deflection() to convert
         the commanded yaw rate into servo commands via the K_delta
         differential mixer model.
      4. Logs the full pipeline state to control.txt.
      5. On LANDED (state 5), sets servos to neutral.
    """
    import time
    while running:
        with update_lock:
            if state >= 3 and motor_enabled and pi is not None:

                # Build sensor data structures for guidance
                gps_data = types.SimpleNamespace(
                    lat=GpsVector.lat, lon=GpsVector.lon,
                    speed=GpsVector.speed, course=GpsVector.course,
                    fix_quality=GpsFidelity.fix_quality,
                    sats=GpsFidelity.sats, rmc_status=GpsFidelity.rmc_status
                )

                # Guidance: compute commanded yaw rate and all metadata
                # The patterned flag drives figure-eight vs. homing behavior
                # inside guidance() -- no separate draw_pattern() call needed.
                result = motor_guidance.guidance(
                    altitude, gps_data, target, patterned=patterned
                )

                # Actuator: convert commanded yaw rate to servo commands
                # using the empirical K_delta differential mixer model.
                motor_result = motor_control.apply_differential_deflection(
                    pi, result.commanded_yaw_rate
                )

                # Log the full pipeline for post-flight analysis
                log_control(result, motor_result)

                # LANDED: stop motors to prevent unnecessary actuation
                if state == 5:
                    log("Stopping motors", events.EventType.warning)
                    motor_control.set_neutral(pi)

        time.sleep(CONTROL_LOG_INTERVAL)


# =============================================================================
# Initialization / Termination
# =============================================================================

def init() -> bool:
    """Initialize all motor subsystems and start the control thread.

    Initialization sequence:
      1. Burnwire mechanism (Motor_Release)
      2. Guidance state (integral, wind, L_DISTANCE, pattern)
      3. Servo hardware (pigpio, neutral position)
      4. Solenoid mechanism (Motor_Egg)
      5. Control loop thread (10 Hz daemon)

    Returns:
        True on success, False on failure (motorapp should exit).
    """
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
    """Shut down all motor subsystems and join threads.

    Sequence:
      1. Signal control loop to stop.
      2. Close the control log file.
      3. Return servos to neutral and disable PWM.
      4. Shut down burnwire and solenoid.
      5. Join all worker threads.
    """
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


# =============================================================================
# Main Loop
# =============================================================================

def motorapp_main(main_pipe: connection.Connection):
    """Entry point for the motor application process.

    Listens on main_pipe for incoming messages and dispatches them
    to the appropriate handler.  The control loop runs in a separate
    thread at 10 Hz.

    Args:
        main_pipe: multiprocessing Connection for receiving routed messages.
    """
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
