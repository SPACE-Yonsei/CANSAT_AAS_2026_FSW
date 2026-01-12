# define arguments of each app (identifier, message ID, ...)
from lib import types

class MainAppArg:
    AppID : types.AppID = 1
    AppName : str = "Main"

    MID_TerminateProcess : types.MID = 100
    MID_SendHK : types.MID = 101

class HkAppArg:
    AppID : types.AppID = 2
    AppName : str = "HK"

    MID_ReceiveHK : types.MID = 201
    MID_SendCombinedHK : types.MID = 202

class BarometerAppArg:
    AppID : types.AppID = 10
    AppName : str = "Barometer"

    MID_SendHK : types.MID = 1001
    MID_SendBarometerTlmData : types.MID = 1002
    MID_SendBarometerFlightLogicData : types.MID = 1003
    MID_ResetBarometerMaxAlt : types.MID = 1004

class CameraAppArg:
    AppID : types.AppID = 11
    AppName : str = "Camera"

    MID_SendHK : types.MID = 1101

class GpsAppArg:
    AppID : types.AppID = 12
    AppName : str = "GPS"

    MID_SendHK : types.MID = 1201
    MID_SendGpsTlmData : types.MID = 1202
    MID_SendGpsFlightLogicData : types.MID = 1203

class ImuAppArg:
    AppID : types.AppID = 13
    AppName : str = "Imu"

    MID_SendHK : types.MID = 1301
    MID_SendImuTlmData : types.MID = 1302
    MID_SendImuFlightLogicData : types.MID = 1304

class FlightlogicAppArg:
    AppID : types.AppID = 14
    AppName : str = "Flight Logic"

    MID_SendHK : types.MID = 1401
    MID_SendCurrentStateToTlm : types.MID = 1404
    MID_SendSimulationStatustoTlm : types.MID = 1407
    MID_SendCameraActivateToCam : types.MID = 1408

    MID_Motor_Payload_LaunchPad : types.MID = 1405
    MID_SetTargetCoordinates : types.MID = 1406
    MID_SendFlightStateToMotor : types.MID = 1407
    MID_Motor_Parafoil_Activate : types.MID = 1409
    MID_Motor_Release_Activate : types.MID = 1410  # 번와이어
    MID_Motor_Egg_Drop_Activate : types.MID = 1411  # 솔레노이드
    MID_PayloadMotorStop : types.MID = 1412
    MID_SendGpsMotorData : types.MID = 1413
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

class ElectroAppArg:
    AppID : types.AppID = 17
    AppName : str = "Electro"

    MID_SendHK : types.MID = 1701
    MID_SendElectroTlmData : types.MID = 1702

class MotorAppArg:
    AppID : types.AppID = 18
    AppName : str = "Motor"

    MID_SendHK : types.MID = 1801

class DistanceAppArg:
    AppID : types.AppID = 19
    AppName : str = "Distance"

    MID_SendHK : types.MID = 1901
    MID_SendDistanceFlightLogicData : types.MID = 1902
    MID_SendDistanceTlmData : types.MID = 1903