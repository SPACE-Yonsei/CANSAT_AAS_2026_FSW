# define arguments of each app (identifier, message ID, ...)
from lib import types

class MainAppArg:
    AppID : types.AppID = 1
    AppName : str = "Main"

    MID_TerminateProcess : types.MID = 100

class FlightlogicAppArg:
    AppID : types.AppID = 2
    AppName : str = "Flight Logic"

    MID_StateToTlm : types.MID = 204
    MID_SendSimulationStatustoTlm : types.MID = 207
    MID_SendCameraActivateToCam : types.MID = 208

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
    AppID : types.AppID = 3
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


# MID 변수명, send 

class BarometerAppArg:
    AppID : types.AppID = 11
    AppName : str = "Barometer"

    MID_Send_Barometer_FlightLogic : types.MID = 1101
    MID_Send_Barometer_Tlm : types.MID = 1102
    MID_Reset_Barometer_MaxAlt : types.MID = 1103

class ImuAppArg:
    AppID : types.AppID = 12
    AppName : str = "Imu"

    MID_Send_Imu_FlightLogic : types.MID = 1201
    MID_Send_Imu_Tlm : types.MID = 1202

class GpsAppArg:
    AppID : types.AppID = 13
    AppName : str = "GPS"

    MID_Send_Gps_FlightLogic : types.MID = 1301
    MID_SendGpsTlmData : types.MID = 1302
    
class DistanceAppArg:
    AppID : types.AppID = 14
    AppName : str = "Distance"

    MID_SendDistanceFlightLogicData : types.MID = 1401
    MID_SendDistanceTlmData : types.MID = 1402

class ElectroAppArg:
    AppID : types.AppID = 15
    AppName : str = "Electro"

    MID_SendElectroTlmData : types.MID = 1502

class CameraAppArg:
    AppID : types.AppID = 16
    AppName : str = "Camera"

class MotorAppArg:
    AppID : types.AppID = 17
    AppName : str = "Motor"

