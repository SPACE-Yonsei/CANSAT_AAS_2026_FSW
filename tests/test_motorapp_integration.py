import math

from tools import sim_control_modes as sim


class FakePi:
    def __init__(self):
        self.calls = []

    def set_servo_pulsewidth(self, pin, pulsewidth):
        self.calls.append((pin, pulsewidth))


def _prepare_motorapp(monkeypatch):
    sim.apply_tuned_config()
    sim.reload_under_test(reload_motorapp=True)
    app = sim.motorapp
    assert app is not None

    monkeypatch.setattr(app.time, "monotonic", lambda: sim.NOW)
    monkeypatch.setattr(app.sensorlog, "log_motor_raw", lambda *args, **kwargs: None)
    monkeypatch.setattr(app.prevstate, "update_start_point", lambda *args, **kwargs: None)

    app.STATE = 4
    app.MOTOR_ENABLED = True
    app.PI = FakePi()
    app._STEER_MODE = ""
    app._ORIGIN_LOCKED = True
    sim.guidance.set_origin_point(sim.ORIGIN_LAT, sim.ORIGIN_LON)
    target_lat, target_lon = sim._latlon_from_ne(0.0, sim.TARGET_E_M)
    sim.guidance.set_target_point(target_lat, target_lon)
    sim.control.reset()
    return app


def test_motorapp_handlers_feed_guidance_control_cycle(monkeypatch):
    app = _prepare_motorapp(monkeypatch)

    app.handle_gps(
        f"{sim.ORIGIN_LAT},{sim.ORIGIN_LON},1,{sim.NOW},90.0,5.0,1,{sim.NOW}"
    )
    app.handle_imu(
        f"0.0,0.0,90.0,0.0,0.0,9.81,0.0,0.0,12.0,1,{sim.NOW},0.0"
    )
    app.handle_barometer("120.0,3.0,1")

    cached_imu = app._CACHE_t.latest_imu
    assert math.isclose(cached_imu.gyrz_rad_s, math.radians(-12.0))
    assert cached_imu.lin_acc_valid is True
    assert cached_imu.lin_acc_reject_reason == "OK"

    out = app._ctrl_cycle(None, sim.NOW)

    assert out is not None
    assert out.valid is True
    assert out.control_mode == sim.guidance.ControlMode.GPS_TRACKING_CLOSED
    assert math.isclose(sim.guidance._STATE_t.imu.gyr_z, math.radians(-12.0))
    assert sim.guidance._STATE_t.flags.gps_pos_fresh is True
    assert sim.guidance._STATE_t.flags.gps_motion_fresh is True
    assert sim.guidance._STATE_t.flags.imu_gyrz_fresh is True
    assert app.PI.calls[-2:] == [
        (sim.control.PARAFOIL_LEFT_MOTOR_PIN, out.left_pw),
        (sim.control.PARAFOIL_RIGHT_MOTOR_PIN, out.right_pw),
    ]
