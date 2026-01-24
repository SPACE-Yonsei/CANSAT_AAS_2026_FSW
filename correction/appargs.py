# define arguments of each app (identifier, message ID, ...)

class MainAppArg:
    AppID : int = 1
    AppName : str = "Main"

    MID_TerminateProcess : int = 100

class FlightlogicAppArg:
    AppID : int = 2
    AppName : str = "Flight Logic"

    MID_StateToTlm : int = 204
    MID_SendSimulationStatustoTlm : int = 207
    MID_SendCameraActivateToCam : int = 208

    MID_Motor_Payload_LaunchPad : int = 1405
    MID_SetTargetCoordinates : int = 1406
    MID_SendFlightStateToMotor : int = 1407
    MID_Motor_Parafoil_Activate : int = 1409
    MID_Motor_Release_Activate : int = 1410  # 번와이어
    MID_Motor_Egg_Drop_Activate : int = 1411  # 솔레노이드
    MID_PayloadMotorStop : int = 1412
    MID_SendGpsMotorData : int = 1413
    MID_SendImuMotorData : int = 1414

class CommAppArg:
    AppID : int = 3
    AppName : str = "Communication"

    MID_SendHK : int = 1601

    MID_RouteCmd_CX : int = 1602
    MID_RouteCmd_ST : int = 1603
    MID_RouteCmd_SIM : int = 1604
    MID_RouteCmd_SIMP : int = 1605
    MID_RouteCmd_CAL : int = 1606
    MID_RouteCmd_MEC : int = 1607
    MID_RouteCmd_SS : int = 1608
    MID_RouteCmd_CAM : int = 1609


# MID 변수명, send 

class BarometerAppArg:
    AppID : int = 11
    AppName : str = "Barometer"

    MID_Send_Barometer_FlightLogic : int = 1101
    MID_Send_Barometer_Tlm : int = 1102
    MID_Reset_Barometer_MaxAlt : int = 1103

class ImuAppArg:
    AppID : int = 12
    AppName : str = "Imu"

    MID_Send_Imu_FlightLogic : int = 1201
    MID_Send_Imu_Tlm : int = 1202

class GpsAppArg:
    AppID : int = 13
    AppName : str = "GPS"

    MID_Send_Gps_FlightLogic : int = 1301
    MID_SendGpsTlmData : int = 1302
    
class DistanceAppArg:
    AppID : int = 14
    AppName : str = "Distance"

    MID_SendDistanceFlightLogicData : int = 1401
    MID_SendDistanceTlmData : int = 1402

class ElectroAppArg:
    AppID : int = 15
    AppName : str = "Electro"

    MID_SendElectroTlmData : int = 1502

class CameraAppArg:
    AppID : int = 16
    AppName : str = "Camera"

class MotorAppArg:
    AppID : int = 17
    AppName : str = "Motor"

