"""AppID/MID contract for FSW inter-app messaging."""


class MainAppArg:
    AppID = 10
    AppName = "Main"
    MID_TerminateProcess = 1001001


class FlightlogicAppArg:
    AppID = 11
    AppName = "FlightLogic"

    MID_motor_TargetCor = 1101901
    MID_motor_state = 1101902
    MID_motor_burnwire = 1101903
    MID_motor_EggDrop = 1101904

    MID_comm_state = 1101201
    MID_comm_sim = 1101202


class CommAppArg:
    AppID = 12
    AppName = "Comm"

    MID_RouteCmd_SIM = 1201101
    MID_RouteCmd_SIMP = 1201102
    MID_RouteCmd_CAL = 1201103
    MID_RouteCmd_MEC = 1201104
    MID_RouteCmd_SS = 1201105
    MID_RouteCmd_CAM = 1201106
    MID_RouteCmd_TC = 1201107
    MID_RouteCmd_SIMG = 1201108
    MID_RouteCmd_FAC = 1201110


class BarometerAppArg:
    AppID = 13
    AppName = "Barometer"

    MID_motor_alt = 1301901
    MID_flight_alt = 1301101
    MID_flight_alt_reset = 1301102
    MID_comm_alt = 1301201


class ImuAppArg:
    AppID = 14
    AppName = "IMU"

    MID_motor_imu = 1401901

    MID_comm_euler = 1401201


class GpsAppArg:
    AppID = 15
    AppName = "GPS"

    MID_motor_gps = 1501901

    MID_comm_gga = 1501201
    MID_flight_gps_sim = 1501101


class DistanceAppArg:
    AppID = 16
    AppName = "Distance"

    MID_flight_dis = 1601101

    MID_comm_dis = 1601201


class ElectroAppArg:
    AppID = 17
    AppName = "Electro"

    MID_comm_volt = 1701201


class CameraAppArg:
    AppID = 18
    AppName = "Camera"

    MID_cam_activate = 1801801


class MotorAppArg:
    AppID = 19
    AppName = "Motor"
    MID_comm_motor_diag = 1901201

# IPC contract notes (documentation only, not runtime data):
# - Envelope: sender_app|receiver_app|msg_id|data
# - data must not contain the "|" delimiter.
# - Comm local commands that do not enter the app bus: CX, ST, RBT.
# - IMU MID_motor_imu gyrz unit is deg/s.
# - Motor and Camera are primarily bus receivers in the current architecture.
