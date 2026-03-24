# define arguments of each app (identifier, message ID, ...)
class MainAppArg:
    AppID : int = 10
    AppName : str = "Main"

    MID_TerminateProcess : int = 1001001 #sender0receiver0order

class FlightlogicAppArg:
    AppID : int = 11
    AppName : str = "Flight Logic"


    MID_comm_state : int = 1101201
    MID_comm_sim : int = 1101202

    MID_cam_activate : int = 1101801

    MID_motor_TargetCor : int = 1101901
    MID_motor_state : int = 1101902
    MID_motor_burnwire : int = 1101903  # 번와이어
    MID_motor_EggDrop : int = 1101904  # 솔레노이드
    MID_motor_PullArms : int = 1101905  # EGG 진입 시 모터 암 중립
    # MID_SendGpsMotorData : int = 1101905
    # MID_SendImuMotorData : int = 1101906


class CommAppArg:
    AppID : int = 12
    AppName : str = "Communication"

    
    MID_RouteCmd_CX : int = 1602
    MID_RouteCmd_ST : int = 1603
    MID_RouteCmd_SIM : int = 1604
    MID_RouteCmd_SIMP : int = 1605
    MID_RouteCmd_CAL : int = 1606
    MID_RouteCmd_MEC : int = 1607
    MID_RouteCmd_SS : int = 1608
    MID_RouteCmd_CAM : int = 1609

class BarometerAppArg:
    AppID : int = 13
    AppName : str = "Barometer"

    MID_comm_alt : int = 1301201

    MID_flight_alt : int = 1301101
    MID_flight_ResetMaxAlt : int = 1301102
    MID_motor_alt : int = 1301901  # barometer altitude → motor (for altitude-adaptive guidance)

class ImuAppArg:
    AppID : int = 14
    AppName : str = "Imu"

    MID_comm_euler : int = 1401201
    MID_motor_imu : int = 1401901  # yaw, gyrz
class GpsAppArg:
    AppID : int = 15
    AppName : str = "GPS"

    MID_comm_gga : int = 1501201

    MID_motor_gps : int = 1501901  # lat, lon, speed_ms, course, fix_quality, sats, rmc_status

class DistanceAppArg:
    AppID : int = 16
    AppName : str = "Distance"

    MID_flight_dis : int = 1601101

    MID_comm_dis : int = 1601201

class ElectroAppArg:
    AppID : int = 17
    AppName : str = "Electro"

    MID_comm_volt : int = 1701201


class CameraAppArg:
    AppID : int = 18
    AppName : str = "Camera"

class MotorAppArg:
    AppID : int = 19
    AppName : str = "Motor"
