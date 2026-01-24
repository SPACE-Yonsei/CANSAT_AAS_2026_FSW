# define arguments of each app (identifier, message ID, ...)
from lib import types

class MainAppArg:
    AppID : types.AppID = 10
    AppName : str = "Main"

<<<<<<< HEAD
    MID_TerminateProcess : int = 10101 #sender0receiver0order
=======
    MID_TerminateProcess : types.MID = 1001001 #sender0receiver0order
>>>>>>> ec54c716ccf3c252f4e74bd5f477d9d01a6cd83f

class FlightlogicAppArg:
    AppID : types.AppID = 11
    AppName : str = "Flight Logic"

<<<<<<< HEAD
    MID_comm_state : int = 20301
    MID_comm_sim : int = 20302

    MID_cam_activate : int = 201601

    MID_motor_TargetCor : int = 201701
    MID_SendFlightStateToMotor : int = 1407
    MID_Motor_Parafoil_Activate : int = 1409
    MID_Motor_Release_Activate : int = 1410  # 번와이어
    MID_Motor_Egg_Drop_Activate : int = 1411  # 솔레노이드
    MID_PayloadMotorStop : int = 1412
    MID_SendImuMotorData : int = 1414
=======
    MID_comm_state : types.MID = 1101201
    MID_comm_sim : types.MID = 1101202

    MID_cam_activate : types.MID = 1101801

    MID_motor_TargetCor : types.MID = 1101901
    MID_motor_state : types.MID = 1101902
    MID_motor_burnwire : types.MID = 1101903  # 번와이어
    MID_motor_EggDrop : types.MID = 1101904  # 솔레노이드
>>>>>>> ec54c716ccf3c252f4e74bd5f477d9d01a6cd83f


class CommAppArg:
    AppID : types.AppID = 12
    AppName : str = "Communication"

<<<<<<< HEAD
    MID_SendHK : int = 1601

    MID_RouteCmd_CX : int = 1602
    MID_RouteCmd_ST : int = 1603
    MID_RouteCmd_SIM : int = 1604
    MID_RouteCmd_SIMP : int = 1605
    MID_RouteCmd_CAL : int = 1606
    MID_RouteCmd_MEC : int = 1607
    MID_RouteCmd_SS : int = 1608
    MID_RouteCmd_CAM : int = 1609
=======
    MID_RouteCmd_CX : types.MID = 1602
    MID_RouteCmd_ST : types.MID = 1603
    MID_RouteCmd_SIM : types.MID = 1604
    MID_RouteCmd_SIMP : types.MID = 1605
    MID_RouteCmd_CAL : types.MID = 1606
    MID_RouteCmd_MEC : types.MID = 1607
    MID_RouteCmd_SS : types.MID = 1608
    MID_RouteCmd_CAM : types.MID = 1609
>>>>>>> ec54c716ccf3c252f4e74bd5f477d9d01a6cd83f

class BarometerAppArg:
    AppID : types.AppID = 13
    AppName : str = "Barometer"

<<<<<<< HEAD
    MID_SendHK : int = 1001
    MID_SendBarometerTlmData : int = 1002
    MID_SendBarometerFlightLogicData : int = 1003
    MID_ResetBarometerMaxAlt : int = 1004
=======
    MID_comm_alt : types.MID = 1301201

    MID_flight_alt : types.MID = 1301101
    MID_flight_ResetMaxAlt : types.MID = 1301102
>>>>>>> ec54c716ccf3c252f4e74bd5f477d9d01a6cd83f

class ImuAppArg:
    AppID : types.AppID = 13
    AppName : str = "Imu"

<<<<<<< HEAD
    MID_SendHK : int = 1301
    MID_SendImuTlmData : int = 1302
    MID_SendImuFlightLogicData : int = 1304

=======
    MID_SendImuTlmData : types.MID = 1302
    MID_SendImuFlightLogicData : types.MID = 1304
    # flightlogic 거치지 않고 motor로 바로 imu
>>>>>>> ec54c716ccf3c252f4e74bd5f477d9d01a6cd83f
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
