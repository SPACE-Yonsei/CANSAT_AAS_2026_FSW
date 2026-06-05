import importlib
import math
from unittest import mock

from Sensor_Motor import control, guidance
from Sensor_Motor.sensor_types import _BaroFromApp, _GpsFromApp, _ImuFromApp


NOW = 1005.0
ANCHOR_NOW = 999.0
ORIGIN_LAT = 37.0
ORIGIN_LON = 127.0
TARGET_E_M = 80.0
TARGET_N_M = 20.0
COURSE_EAST = math.radians(90.0)


def reload_guidance_control():
    global guidance, control
    guidance = importlib.reload(guidance)
    control = importlib.reload(control)
    control.reset()


def latlon_from_ne(n_m, e_m):
    lat = ORIGIN_LAT + math.degrees(n_m / guidance.EARTH_RADIUS_M)
    lon = ORIGIN_LON + math.degrees(
        e_m / (guidance.EARTH_RADIUS_M * math.cos(math.radians(ORIGIN_LAT)))
    )
    return lat, lon


def prepare_mission():
    guidance.reset()
    guidance.set_origin_point(ORIGIN_LAT, ORIGIN_LON)
    target_lat, target_lon = latlon_from_ne(TARGET_N_M, TARGET_E_M)
    guidance.set_target_point(target_lat, target_lon)


def gps_at(now, *, n_m=0.0, e_m=0.0, pos=True, motion=True, course=COURSE_EAST, speed=4.0):
    lat, lon = latlon_from_ne(n_m, e_m)
    return _GpsFromApp(
        lat=lat,
        lon=lon,
        course_rad=course if motion else None,
        speed_mps=speed if motion else None,
        pos_ts=now if pos else None,
        motion_ts=now if motion else None,
        pos_health=1 if pos else 0,
        motion_health=1 if motion else 0,
    )


def imu_at(now, *, gyrz_dps=10.0, yaw=COURSE_EAST, acc=True):
    return _ImuFromApp(
        yaw_rad=yaw,
        gyrz_rad_s=math.radians(gyrz_dps),
        ts=now,
        lin_acc_x=0.2 if acc else None,
        lin_acc_y=0.0,
        lin_acc_valid=acc,
        health=1,
    )


def baro_at(now, *, sink=2.0):
    return _BaroFromApp(alt_m=120.0, sink_rate=sink, rx_ts=now, health=1)


def lock_dr_anchor_with_full_gps():
    mode = guidance.DecideControlMode(
        gps_at(ANCHOR_NOW, pos=True, motion=True, course=COURSE_EAST, speed=4.0),
        imu_at(ANCHOR_NOW, gyrz_dps=0.0, yaw=COURSE_EAST, acc=False),
        baro_at(ANCHOR_NOW),
        ANCHOR_NOW,
    )
    assert mode == guidance.ControlMode.GPS_TRACKING_CLOSED
    assert guidance.dr_current_valid(guidance._STATE_t.dr)


def dr_pm_gba_sensors():
    return None, imu_at(NOW, gyrz_dps=10.0, yaw=COURSE_EAST, acc=True), baro_at(NOW)


def dr_position():
    dr = guidance._STATE_t.dr
    return dr.current_E, dr.current_N


def test_produce_l1input_does_not_double_integrate_dr_pm_current():
    reload_guidance_control()
    prepare_mission()
    lock_dr_anchor_with_full_gps()
    gps, imu, baro = dr_pm_gba_sensors()

    with mock.patch.object(guidance, "DecideControlMode", wraps=guidance.DecideControlMode) as decide, \
            mock.patch.object(guidance, "FillNav", wraps=guidance.FillNav) as fill_nav:
        mode = guidance.DecideControlMode(gps, imu, baro, NOW)
        assert decide.call_count == 1
        assert fill_nav.call_count == 1

        assert mode == guidance.ControlMode.DR_PM_GBA_CLOSED
        assert guidance._STATE_t.nav.valid is True
        before_l1input = dr_position()

        l1_in = guidance.ProduceL1Input(NOW)
        after_l1input = dr_position()

        assert decide.call_count == 1
        assert fill_nav.call_count == 1

    assert l1_in.valid is True
    assert l1_in.E == guidance._STATE_t.nav.E
    assert l1_in.N == guidance._STATE_t.nav.N
    assert after_l1input == before_l1input


def test_diagnostic_reentering_fillnav_same_cycle_is_forbidden():
    reload_guidance_control()
    prepare_mission()
    lock_dr_anchor_with_full_gps()
    gps, imu, baro = dr_pm_gba_sensors()

    mode = guidance.DecideControlMode(gps, imu, baro, NOW)
    assert mode == guidance.ControlMode.DR_PM_GBA_CLOSED
    before_reentry = dr_position()

    guidance.FillNav(guidance._STATE_t.flags, NOW)

    after_reentry = dr_position()

    # This diagnostic intentionally performs the forbidden same-cycle FillNav
    # re-entry to document the hazard. In the current implementation same
    # `now` gives dt=0, so E/N may not move; if any harness re-enters with a
    # later timestamp or stale last_step_time, DR_PM propagation can be applied
    # again.
    assert after_reentry == before_reentry
