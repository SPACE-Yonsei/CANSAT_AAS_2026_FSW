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
    MID_motor_PullArms = 1101905

    MID_comm_state = 1101201
    MID_comm_sim = 1101202


class CommAppArg:
    AppID = 12
    AppName = "Comm"

    MID_RouteCmd_SIM = 121101
    MID_RouteCmd_SIMP = 121102
    MID_RouteCmd_CAL = 121103
    MID_RouteCmd_MEC = 121104
    MID_RouteCmd_SS = 121105
    MID_RouteCmd_CAM = 121106
    MID_RouteCmd_TC = 121107


class BarometerAppArg:
    AppID = 13
    AppName = "Barometer"

    MID_motor_alt = 1301901
    MID_flight_alt = 1301101
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
