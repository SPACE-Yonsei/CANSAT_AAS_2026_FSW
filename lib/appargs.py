# define arguments of each app (identifier, message ID, ...)
from lib import types

class MainAppArg:
    AppID : types.AppID = 11
    AppName : str = "Main"

    MID_TerminateProcess : types.MID = 10101 #sender0receiver0order

class FlightlogicAppArg:
    AppID : types.AppID = 2
    AppName : str = "Flight Logic"

    MID_comm_state : types.MID = 20301
    MID_comm_sim : types.MID = 20302

    MID_cam_activate : types.MID = 201601

    MID_motor_TargetCor : types.MID = 201701
    MID_SendFlightStateToMotor : types.MID = 1407
    MID_Motor_Parafoil_Activate : types.MID = 1409
    MID_Motor_Release_Activate : types.MID = 1410  # 번와이어
    MID_Motor_Egg_Drop_Activate : types.MID = 1411  # 솔레노이드
    MID_PayloadMotorStop : types.MID = 1412
    MID_SendImuMotorData : types.MID = 1414


class CommAppArg:
    AppID : types.AppID = 16
    AppName : str = "Communication"

    MID_SendHK : types.MID = 1601

    MID_RouteCmd_CX : types.MID = 1602
    MID_RouteCmd_ST : types.MID = 1603
    MID_RouteCmd_SIM : types.MID = 1604
    MID_RouteCmd_SIMP : types.MID = 1605
    MID_RouteCmd_CAL : types.MID = 1606
    MID_RouteCmd_MEC : types.MID = 1607
    MID_RouteCmd_SS : types.MID = 1608
    MID_RouteCmd_CAM : types.MID = 1609

class BarometerAppArg:
    AppID : types.AppID = 10
    AppName : str = "Barometer"

    MID_SendHK : types.MID = 1001
    MID_SendBarometerTlmData : types.MID = 1002
    MID_SendBarometerFlightLogicData : types.MID = 1003
    MID_ResetBarometerMaxAlt : types.MID = 1004

class ImuAppArg:
    AppID : types.AppID = 13
    AppName : str = "Imu"

    MID_SendHK : types.MID = 1301
    MID_SendImuTlmData : types.MID = 1302
    MID_SendImuFlightLogicData : types.MID = 1304

class GpsAppArg:
    AppID : types.AppID = 12
    AppName : str = "GPS"

    MID_SendHK : types.MID = 1201
    MID_SendGpsTlmData : types.MID = 1202
    MID_SendGpsFlightLogicData : types.MID = 1203
    #flightlogic 거치지 않고 motor로 바로

class DistanceAppArg:
    AppID : types.AppID = 19
    AppName : str = "Distance"

    MID_SendHK : types.MID = 1901
    MID_SendDistanceFlightLogicData : types.MID = 1902
    MID_SendDistanceTlmData : types.MID = 1903

class ElectroAppArg:
    AppID : types.AppID = 17
    AppName : str = "Electro"

    MID_SendHK : types.MID = 1701
    MID_SendElectroTlmData : types.MID = 1702


class CameraAppArg:
    AppID : types.AppID = 11
    AppName : str = "Camera"

    MID_SendHK : types.MID = 1101

class MotorAppArg:
    AppID : types.AppID = 18
    AppName : str = "Motor"

    MID_SendHK : types.MID = 1801
