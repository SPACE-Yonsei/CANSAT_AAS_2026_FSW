# define arguments of each app (identifier, message ID, ...)
from lib import types

class MainAppArg:
    # AppID should be unique
    AppID : types.AppID = 1
    # Application name used in log
    AppName : str = "Main"

    # Message ID used in Main App
    MID_TerminateProcess : types.MID = 100
    MID_SendHK : types.MID = 101

class HkAppArg:
    # AppID should be unique
    AppID : types.AppID = 2
    # Application name used in log
    AppName : str = "HK"

    # Message ID
    MID_ReceiveHK : types.MID = 201

    MID_SendCombinedHK : types.MID = 202

class SampleAppArg:
    # AppID should be unique
    AppID : types.AppID = 99
    # Application name used in log
    AppName : str = "Sample"

    # Message ID
    MID_SendHK : types.MID = 9901

class BarometerAppArg:
    # AppID should be unique
    AppID : types.AppID = 10
    # Application name used in log
    AppName : str = "Sensor_Barometer"

    # Message ID
    MID_SendHK : types.MID = 1001
    # Send Barometer Data to comms app for tememetry
    MID_SendBarometerTlmData : types.MID = 1002
    MID_SendBarometerFlightLogicData : types.MID = 1003
    MID_ResetBarometerMaxAlt : types.MID = 1004

class CameraAppArg:
    # AppID should be unique
    AppID : types.AppID = 11
    # Application name used in log
    AppName : str = "Camera"

    # Message ID
    MID_SendHK : types.MID = 1101

class GpsAppArg:
    # AppID should be unique
    AppID : types.AppID = 12
    # Application name used in log
    AppName : str = "GPS"

    # Message ID
    MID_SendHK : types.MID = 1201
    MID_SendGpsTlmData : types.MID = 1202
    MID_SendGpsFlightLogicData : types.MID = 1203

class ImuAppArg:
    # AppID should be unique
    AppID : types.AppID = 13
    # Application name used in log
    AppName : str = "Imu"

    # Message ID
    MID_SendHK : types.MID = 1301
    # Send Imu Data to comms app for tememetry
    MID_SendImuTlmData : types.MID = 1302
    MID_SendImuFlightLogicData : types.MID = 1304

class FlightlogicAppArg:
    # AppID should be unique
    AppID : types.AppID = 14
    # Application name used in log
    AppName : str = "Flight Logic"

    # Message ID
    MID_SendHK : types.MID = 1401
    MID_RocketMotorActivate : types.MID = 1402
    MID_PayloadReleaseMotorActivate : types.MID = 1403
    MID_SendCurrentStateToTlm : types.MID = 1404
    MID_PayloadReleaseMotorStandby : types.MID = 1405
    MID_RocketMotorStandby : types.MID = 1406
    MID_SendSimulationStatustoTlm : types.MID = 1407
    MID_SendCameraActivateToCam : types.MID = 1408
    MID_SendPayloadMotorRatation : types.MID = 1408
    
class TachometerAppArg:
    # AppID should be unique
    AppID : types.AppID = 15
    # Application name used in log
    AppName : str = "Tachometer"

    # Message ID
    MID_SendHK : types.MID = 1501
    MID_SendDegPerSec : types.MID = 1502

class CommAppArg:
    # AppID should be unique
    AppID : types.AppID = 16
    # Application name used in log
    AppName : str = "Communication"

    # Message ID
    MID_SendHK : types.MID = 1601

    MID_RouteCmd_CX : types.MID = 1602
    MID_RouteCmd_ST : types.MID = 1603
    MID_RouteCmd_SIM : types.MID = 1604
    MID_RouteCmd_SIMP : types.MID = 1605
    MID_RouteCmd_CAL : types.MID = 1606
    MID_RouteCmd_MEC : types.MID = 1607
    MID_RouteCmd_SS : types.MID = 1608
    MID_RouteCmd_CAM : types.MID = 1609

class VoltageAppArg:
    # AppID should be unique
    AppID : types.AppID = 17
    # Application name used in log
    AppName : str = "Voltage"

    # Message ID
    MID_SendHK : types.MID = 1701
    MID_SendVoltageTlmData : types.MID = 1702

class motorAppArg:
    # AppID should be unique
    AppID : types.AppID = 18
    # Application name used in log
    AppName : str = "Motor"

    # Message ID
    MID_SendHK : types.MID = 1801