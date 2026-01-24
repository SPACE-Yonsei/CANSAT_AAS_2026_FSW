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

class ImuAppArg:
    AppID : int = 13
    AppName : str = "Imu"

    MID_SendImuTlmData : int = 1302
    MID_SendImuFlightLogicData : int = 1304
    # flightlogic 거치지 않고 motor로 바로 imu
class GpsAppArg:
    AppID : int = 12
    AppName : str = "GPS"

    MID_SendHK : int = 1201
    MID_SendGpsTlmData : int = 1202
    MID_SendGpsFlightLogicData : int = 1203
    #flightlogic 거치지 않고 motor로 바로

class DistanceAppArg:
    AppID : int = 19
    AppName : str = "Distance"

    MID_SendHK : int = 1901
    MID_SendDistanceFlightLogicData : int = 1902
    MID_SendDistanceTlmData : int = 1903

class ElectroAppArg:
    AppID : int = 17
    AppName : str = "Electro"

    MID_SendHK : int = 1701
    MID_SendElectroTlmData : int = 1702


class CameraAppArg:
    AppID : int = 11
    AppName : str = "Camera"

    MID_SendHK : int = 1101

class MotorAppArg:
    AppID : int = 18
    AppName : str = "Motor"

    MID_SendHK : int = 1801
