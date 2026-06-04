import importlib
import math
from types import SimpleNamespace

from lib import config
from Sensor_Motor import guidance


NOW = 1000.0
EPS = 0.01


def _fresh_guidance():
    return importlib.reload(guidance)


def test_gps_freshness_boundary():
    for age, expected in (
        (config.GPS_FRESH_MAX_AGE_S - EPS, True),
        (config.GPS_FRESH_MAX_AGE_S + EPS, False),
    ):
        g = _fresh_guidance()
        gps = SimpleNamespace(
            lat=37.0,
            lon=127.0,
            pos_ts=NOW - age,
            pos_health=1,
            course_rad=math.radians(90.0),
            speed_mps=5.0,
            motion_ts=NOW - age,
            motion_health=1,
        )
        g.UpdateRaw(gps=gps, now=NOW)
        flags = g.ComputeFreshFlags(NOW)

        assert flags.gps_pos_fresh is expected
        assert flags.gps_motion_fresh is expected


def test_imu_freshness_boundary():
    for age, expected in (
        (config.IMU_FRESH_MAX_AGE_S - EPS, True),
        (config.IMU_FRESH_MAX_AGE_S + EPS, False),
    ):
        g = _fresh_guidance()
        imu = SimpleNamespace(
            health=1,
            ts=NOW - age,
            yaw_rad=math.radians(90.0),
            gyrz_rad_s=math.radians(5.0),
            lin_acc_x=0.05,
            lin_acc_y=0.0,
            lin_acc_valid=True,
        )
        g.UpdateRaw(imu=imu, now=NOW)
        flags = g.ComputeFreshFlags(NOW)

        assert flags.imu_yaw_fresh is expected
        assert flags.imu_gyrz_fresh is expected
        assert flags.acc_fresh is expected


def test_baro_freshness_boundary():
    for age, expected in (
        (config.BARO_FRESH_MAX_AGE_S - EPS, True),
        (config.BARO_FRESH_MAX_AGE_S + EPS, False),
    ):
        g = _fresh_guidance()
        baro = SimpleNamespace(
            health=1,
            rx_ts=NOW - age,
            alt_m=120.0,
            sink_rate=3.0,
        )
        g.UpdateRaw(baro=baro, now=NOW)
        flags = g.ComputeFreshFlags(NOW)

        assert flags.baro_sink_fresh is expected
