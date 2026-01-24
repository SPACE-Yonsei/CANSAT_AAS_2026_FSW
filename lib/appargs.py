# define arguments of each app (identifier, message ID, ...)
from lib import types

class MainAppArg:
    AppID : types.AppID = 10
    AppName : str = "Main"

    MID_TerminateProcess : types.MID = 1001001 #sender0receiver0order

class FlightlogicAppArg:
    AppID : types.AppID = 11
    AppName : str = "Flight Logic"


    MID_comm_state : types.MID = 1101201
    MID_comm_sim : types.MID = 1101202

    MID_cam_activate : types.MID = 1101801

    MID_motor_TargetCor : types.MID = 1101901
    MID_motor_state : types.MID = 1101902
    MID_motor_burnwire : types.MID = 1101903  # 번와이어
    MID_motor_EggDrop : types.MID = 1101904  # 솔레노이드


class CommAppArg:
    AppID : types.AppID = 12
    AppName : str = "Communication"

    MID_RouteCmd_CX : types.MID = 1602
    MID_RouteCmd_ST : types.MID = 1603
    MID_RouteCmd_SIM : types.MID = 1604
    MID_RouteCmd_SIMP : types.MID = 1605
    MID_RouteCmd_CAL : types.MID = 1606
    MID_RouteCmd_MEC : types.MID = 1607
    MID_RouteCmd_SS : types.MID = 1608
    MID_RouteCmd_CAM : types.MID = 1609

class BarometerAppArg:
    AppID : types.AppID = 13
    AppName : str = "Barometer"

    MID_comm_alt : types.MID = 1301201

    MID_flight_alt : types.MID = 1301101
    MID_flight_ResetMaxAlt : types.MID = 1301102

class ImuAppArg:
    AppID : types.AppID = 13
    AppName : str = "Imu"
    
    MID_SendImuTlmData : types.MID = 1302
    MID_SendImuFlightLogicData : types.MID = 1304
    # flightlogic 거치지 않고 motor로 바로 imu
class GpsAppArg:
    AppID : types.AppID = 12
    AppName : str = "GPS"

    MID_SendHK : int = 1201
    MID_SendGpsTlmData : int = 1202
    MID_SendGpsFlightLogicData : int = 1203
    #flightlogic 거치지 않고 motor로 바로

class DistanceAppArg:
    AppID : types.AppID = 19
    AppName : str = "Distance"

    MID_SendHK : int = 1901
    MID_SendDistanceFlightLogicData : int = 1902
    MID_SendDistanceTlmData : int = 1903

class ElectroAppArg:
    AppID : types.AppID = 17
    AppName : str = "Electro"

    MID_SendHK : int = 1701
    MID_SendElectroTlmData : int = 1702


class CameraAppArg:
    AppID : types.AppID = 11
    AppName : str = "Camera"

    MID_SendHK : int = 1101

class MotorAppArg:
    AppID : types.AppID = 18
    AppName : str = "Motor"

    MID_SendHK : int = 1801
