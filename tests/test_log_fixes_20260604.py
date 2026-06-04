"""1·2·3차 낙하 로그 분석 기반 계산 로직 수정 검증 (2026-06-04).

검증 항목:
  [1] mode별 yaw-rate limit이 L1Output → CtrlInput → FF 정규화까지 전달.
  [3] IMU_ACCEL_INPUT_MODE 분기 (RAW 기본).
  [4] lin_acc invalid sample 이후 acc_fresh=False (이전 acc 잔존 금지).
  [5] DR_PM 위치 점프 reject (DR_POSITION_JUMP).
  [6] baro sink spike 이후 baro_sink_fresh=False, filtered 미오염.
  [공통] L1Output.yaw_rate_limit_dps 항상 finite.
"""
import math
import unittest

from lib import config
from Sensor_Motor import control, guidance
from Sensor_Motor.guidance import ControlMode
from Sensor_Motor.sensor_types import _BaroFromApp, _ImuFromApp


class TestYawRateLimitPropagation(unittest.TestCase):
    def test_gps_open_limit_to_ctrlinput(self):
        # guidance가 GPS_OPEN에 35 dps를 고른다.
        lim_dps = math.degrees(
            guidance._choose_yaw_rate_limit_rad_s(ControlMode.GPS_TRACKING_OPEN))
        self.assertAlmostEqual(lim_dps, 35.0)
        # 그 값이 CtrlInput으로 전달된다.
        l1 = guidance.L1Output(
            control_mode=ControlMode.GPS_TRACKING_OPEN,
            yaw_rate_limit_dps=lim_dps, yaw_rate_cmd=0.1,
            control_valid=True, nominal=True, ground_speed_mps=5.0,
        )
        ci = control.ProduceCtrlInput(l1, now=0.0)
        self.assertAlmostEqual(ci.yaw_rate_limit_dps, 35.0)

    def test_missing_limit_falls_back_to_gps_closed(self):
        l1 = guidance.L1Output(control_mode=ControlMode.GPS_TRACKING_OPEN,
                               yaw_rate_limit_dps=float("nan"),
                               control_valid=True, nominal=True)
        ci = control.ProduceCtrlInput(l1, now=0.0)
        self.assertAlmostEqual(ci.yaw_rate_limit_dps,
                               config.GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS)

    def test_dr_pm_g_limit_used_in_ff_normalization(self):
        control.reset()
        ci = control.CtrlInput(
            angular_velocity_cmd_deg_s=50.0, valid=True,
            control_mode=ControlMode.DR_PM_G_CLOSED,
            yaw_rate_limit_dps=config.DR_PM_G_YAW_RATE_LIMIT_DPS,  # 30
            pid_enabled=False,
        )
        out = control.ProduceCtrlOutput(ci, angular_velocity_meas_deg_s=float("nan"),
                                        now=10.0)
        # control이 GPS_CLOSED(60)이 아니라 mode limit(30)으로 클램프/정규화한다.
        self.assertAlmostEqual(out.yaw_rate_limit_dps, 30.0)
        self.assertLessEqual(abs(out.angular_velocity_cmd_deg_s), 30.0 + 1e-9)

    def test_ff_normalization_depends_on_limit(self):
        # 동일 cmd라도 limit가 작을수록 FF curve의 x가 커져 delta가 크다.
        d30 = control.angular_velocity_to_delta_ff(30.0, 30.0)   # x=1.0
        d60 = control.angular_velocity_to_delta_ff(30.0, 60.0)   # x=0.5
        self.assertGreater(abs(d30), abs(d60))


class TestAccInvalidClear(unittest.TestCase):
    def test_invalid_acc_sample_clears_fresh(self):
        guidance.reset()
        now = 100.0
        good = _ImuFromApp(yaw_rad=0.0, gyrz_rad_s=0.0,
                           lin_acc_x=0.5, lin_acc_y=0.2,
                           lin_acc_valid=True, health=1, ts=now)
        guidance.UpdateRaw(imu=good, now=now)
        f1 = guidance.ComputeFreshFlags(now)
        self.assertTrue(f1.acc_fresh)

        bad = _ImuFromApp(yaw_rad=0.0, gyrz_rad_s=0.0,
                          lin_acc_x=None, lin_acc_y=None,
                          lin_acc_valid=False, health=1, ts=now + 0.1)
        guidance.UpdateRaw(imu=bad, now=now + 0.1)
        # 이전 acc 값이 raw imu field에 남지 않는다.
        self.assertTrue(math.isnan(guidance._STATE_t.imu.lin_acc_x))
        self.assertFalse(guidance._STATE_t.imu.lin_acc_valid)
        f2 = guidance.ComputeFreshFlags(now + 0.1)
        self.assertFalse(f2.acc_fresh)

    def test_health_false_clears_all_valid(self):
        guidance.reset()
        now = 100.0
        good = _ImuFromApp(yaw_rad=0.3, gyrz_rad_s=0.1,
                           lin_acc_x=0.5, lin_acc_y=0.2,
                           lin_acc_valid=True, health=1, ts=now)
        guidance.UpdateRaw(imu=good, now=now)
        dead = _ImuFromApp(health=0, ts=now + 0.1)
        guidance.UpdateRaw(imu=dead, now=now + 0.1)
        st = guidance._STATE_t.imu
        self.assertFalse(st.yaw_valid)
        self.assertFalse(st.gyrz_valid)
        self.assertFalse(st.lin_acc_valid)
        self.assertTrue(math.isnan(st.yaw))


class TestBaroSinkFiltering(unittest.TestCase):
    def test_spike_drops_fresh_and_keeps_filtered(self):
        guidance.reset()
        now = 200.0
        ok = _BaroFromApp(alt_m=100.0, sink_rate=3.0, rx_ts=now, health=1)
        guidance.UpdateRaw(baro=ok, now=now)
        f1 = guidance.ComputeFreshFlags(now)
        self.assertTrue(f1.baro_sink_fresh)
        self.assertFalse(guidance._STATE_t.nav.baro_sink_spike)
        self.assertAlmostEqual(guidance._STATE_t.baro.filtered_sink_rate, 3.0)

        spike = _BaroFromApp(alt_m=100.0, sink_rate=12.0, rx_ts=now + 0.1, health=1)
        guidance.UpdateRaw(baro=spike, now=now + 0.1)
        f2 = guidance.ComputeFreshFlags(now + 0.1)
        self.assertFalse(f2.baro_sink_fresh)
        self.assertTrue(guidance._STATE_t.nav.baro_sink_spike)
        # spike는 filtered를 오염시키지 않는다(이전 3.0 유지).
        self.assertLessEqual(guidance._STATE_t.baro.filtered_sink_rate, config.DR_BARO_SINK_MAX_MPS)
        self.assertAlmostEqual(guidance._STATE_t.baro.filtered_sink_rate, 3.0)


class TestDrPositionJumpGuard(unittest.TestCase):
    def test_dr_pm_position_jump_rejected(self):
        guidance.reset()
        now = 300.0
        st = guidance._STATE_t
        st.dr.current_E = 0.0
        st.dr.current_N = 0.0
        st.dr.current_course = 0.0
        st.dr.current_V = config.V_MAX_DR_MPS      # 6.5
        st.dr.current_time = now - 1.0
        st.dr.last_step_time = now - 1.0           # dt → 0.5 (safe_dt 상한)
        st.dr.anchor_course = 0.0
        st.dr.yaw_at_anchor = 0.0
        st.dr.anchor_time = now - 1.0
        st.imu.yaw = 0.0
        st.imu.yaw_valid = True
        st.flags.imu_yaw_fresh = True
        # step = V * dt = 6.5 * 0.5 = 3.25 m > DR_MAX_POSITION_JUMP_M(3.0)
        ok, reason = guidance._fill_nav_for_dr_pm_mode(ControlMode.DR_PM_Y_OPEN, now)
        self.assertFalse(ok)
        self.assertEqual(reason, "DR_POSITION_JUMP")


class TestL1LimitAlwaysFinite(unittest.TestCase):
    def test_invalid_dr_output_has_finite_limit(self):
        li = guidance.L1Input(
            valid=True, control_mode=ControlMode.DR_PM_G_CLOSED,
            E=0.0, N=0.0, V=0.1, course=0.0,           # V < V_MIN → V_TOO_SMALL
            target_E=10.0, target_N=10.0, confidence=1.0,
        )
        out = guidance.ProduceL1Output(li)
        self.assertFalse(out.control_valid)
        self.assertTrue(math.isfinite(out.yaw_rate_limit_dps))
        self.assertGreater(out.yaw_rate_limit_dps, 0.0)   # DR_PM_G = 30

    def test_fail_output_has_finite_limit(self):
        out = guidance.ProduceL1Output(
            guidance.L1Input(valid=False, control_mode=ControlMode.FAIL, reason="FAIL"))
        self.assertTrue(math.isfinite(out.yaw_rate_limit_dps))


if __name__ == "__main__":
    unittest.main()
