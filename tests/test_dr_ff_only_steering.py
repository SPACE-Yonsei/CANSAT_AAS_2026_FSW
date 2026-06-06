"""DR = FF-only 조향 회귀 테스트.

배경(실측): DR 구간에서 V가 바닥(~0.5 m/s)이라 L1 yaw_rate_cmd가 작고(≤~5 dps),
GPS FF 데드밴드(5)에 걸려 delta_ff=0이 되었다. 남은 rate-damping PID가 측정 gyro만
죽이려다 heading 보정과 반대로(nu와 반대 부호) 팔을 움직였다(부호 반대 ~29%).

수정: DR은 기본적으로 PID 제거(FF-only) + DR 전용 데드밴드/정규화 기준으로 FF 감도 회복.
GPS 모드 동작은 불변. 본 테스트는 그 동작을 고정한다.
"""
import math
import unittest

from Sensor_Motor import control, guidance
from lib import config


def _dr_cmd(cmd_dps, gyrz_meas_dps, *, mode=None, limit=None,
            pid_enabled=False, now=1.0):
    """DR control 1-사이클 헬퍼: guidance가 내려줄 CtrlInput을 모사해 ProduceCtrlOutput 실행."""
    mode = mode or guidance.ControlMode.DR_M_GBA_CLOSED
    limit = config.DR_M_GBA_YAW_RATE_LIMIT_DPS if limit is None else limit
    control.reset()
    ci = control.CtrlInput(
        angular_velocity_cmd_deg_s=cmd_dps,
        ground_speed_mps=1.0,
        valid=True,
        timestamp=now,
        pid_enabled=pid_enabled,
        control_mode=mode,
        yaw_rate_limit_dps=limit,
    )
    return control.ProduceCtrlOutput(ci, gyrz_meas_dps, now)


class TestDrPidRemoved(unittest.TestCase):
    def test_guidance_disables_pid_for_dr_modes(self):
        """DR_PID_ENABLED=False면 모든 DR 모드에서 L1Output.pid_enabled=False, GPS는 True."""
        self.assertFalse(getattr(config, "DR_PID_ENABLED", False),
                         "이 테스트는 기본값 DR_PID_ENABLED=False 가정")
        dr_in = guidance.L1Input(
            valid=True, control_mode=guidance.ControlMode.DR_M_GBA_CLOSED,
            dr_method=guidance.DRMethod.GYRO_INTEGRATION, confidence=1.0,
            E=0.0, N=0.0, V=3.0, course=0.0, target_E=100.0, target_N=0.0,
        )
        dr_out = guidance.ProduceL1Output(dr_in)
        self.assertTrue(dr_out.control_valid)
        self.assertFalse(dr_out.pid_enabled)

        gps_in = guidance.L1Input(
            valid=True, control_mode=guidance.ControlMode.GPS_TRACKING_CLOSED,
            confidence=1.0, E=0.0, N=0.0, V=6.0, course=0.0,
            target_E=100.0, target_N=0.0,
        )
        gps_out = guidance.ProduceL1Output(gps_in)
        self.assertTrue(gps_out.control_valid)
        self.assertTrue(gps_out.pid_enabled)

    def test_dr_ff_only_steers_toward_command_despite_opposing_gyro(self):
        """핵심 회귀: nu>0(우회전 명령)인데 우측으로 빠르게 회전 중(gyro+)이어도
        FF-only DR은 명령 부호(우)대로 팔을 움직인다. (이전엔 PID가 -방향으로 뒤집음)"""
        out = _dr_cmd(cmd_dps=+3.77, gyrz_meas_dps=+31.45)
        self.assertGreater(out.delta_total_deg, 0.0)
        self.assertGreater(out.delta_arm_deg, 0.0)
        self.assertEqual(out.delta_pid_deg, 0.0)        # PID 제거 → 기여 없음
        self.assertFalse(out.dr_pid_active)
        # 부호 일치(명령 ↔ 최종 팔)
        self.assertEqual(math.copysign(1.0, out.delta_total_deg), 1.0)

    def test_dr_ff_only_left_command_despite_opposing_gyro(self):
        """좌회전 명령(nu<0) + 좌로 회전 중(gyro<0)에도 팔은 좌(-)로 간다."""
        out = _dr_cmd(cmd_dps=-3.77, gyrz_meas_dps=-31.45)
        self.assertLess(out.delta_total_deg, 0.0)
        self.assertLess(out.delta_arm_deg, 0.0)
        self.assertEqual(out.delta_pid_deg, 0.0)

    def test_dr_pid_never_active_even_with_huge_gyro(self):
        out = _dr_cmd(cmd_dps=+2.0, gyrz_meas_dps=+200.0)
        self.assertEqual(out.delta_pid_deg, 0.0)
        self.assertFalse(out.dr_pid_active)
        self.assertGreater(out.delta_total_deg, 0.0)


class TestDrFeedforwardDeadbandAndSensitivity(unittest.TestCase):
    def test_dr_deadband_separate_from_gps(self):
        """DR 데드밴드 < GPS 데드밴드: 그 사이 명령에서 GPS는 FF=0, DR은 FF≠0."""
        gps_db = config.CTRL_ANGULAR_VELOCITY_DEADBAND_DEG_S
        dr_db = config.DR_CTRL_ANGULAR_VELOCITY_DEADBAND_DEG_S
        self.assertLess(dr_db, gps_db)
        cmd = 0.5 * (dr_db + gps_db)   # dr_db < cmd < gps_db

        gps = _dr_cmd(cmd_dps=cmd, gyrz_meas_dps=0.0,
                      mode=guidance.ControlMode.GPS_TRACKING_OPEN,
                      limit=config.GPS_TRACKING_OPEN_YAW_RATE_LIMIT_DPS,
                      pid_enabled=False)
        self.assertEqual(gps.delta_ff_deg, 0.0)
        self.assertEqual(gps.delta_total_deg, 0.0)

        dr = _dr_cmd(cmd_dps=cmd, gyrz_meas_dps=0.0)
        self.assertGreater(dr.delta_ff_deg, 0.0)
        self.assertGreater(dr.delta_total_deg, 0.0)

    def test_ff_ref_decouples_sensitivity_from_limit(self):
        """정규화 기준(ref)을 limit보다 작게 두면 같은 명령에서 더 큰 deflection."""
        cmd, limit = 3.77, config.DR_M_GBA_YAW_RATE_LIMIT_DPS
        d_ref_small = control.angular_velocity_to_delta_ff(
            cmd, limit, deadband_dps=1.0, ref_dps=config.DR_FF_REF_DPS)
        d_ref_limit = control.angular_velocity_to_delta_ff(
            cmd, limit, deadband_dps=1.0, ref_dps=limit)
        self.assertGreater(d_ref_small, d_ref_limit)
        self.assertGreater(d_ref_limit, 0.0)

    def test_ff_monotonic_in_command(self):
        ref = config.DR_FF_REF_DPS
        small = control.angular_velocity_to_delta_ff(2.0, 50.0, deadband_dps=1.0, ref_dps=ref)
        large = control.angular_velocity_to_delta_ff(8.0, 50.0, deadband_dps=1.0, ref_dps=ref)
        self.assertGreater(large, small)

    def test_dr_ff_delta_cap(self):
        """감도 상향이 full hard-over로 가지 않도록 DR FF cap 적용."""
        cap = config.DR_FF_DELTA_LIMIT_DEG
        out = _dr_cmd(cmd_dps=config.DR_M_GBA_YAW_RATE_LIMIT_DPS, gyrz_meas_dps=0.0)
        self.assertLessEqual(abs(out.delta_total_deg), cap + 1e-6)
        # cap 전 FF가 cap보다 컸음을 확인(= cap이 실제로 동작)
        self.assertGreater(abs(out.delta_ff_pre_cap_deg), cap)


class TestGpsPathUnchanged(unittest.TestCase):
    def test_gps_closed_pid_still_active(self):
        """GPS_CLOSED는 PID·데드밴드(5)·일반 cap 그대로."""
        out = _dr_cmd(cmd_dps=0.0, gyrz_meas_dps=+30.0,
                      mode=guidance.ControlMode.GPS_TRACKING_CLOSED,
                      limit=config.GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS,
                      pid_enabled=True)
        self.assertNotEqual(out.delta_pid_deg, 0.0)   # PID 살아있음
        self.assertFalse(out.dr_pid_active)            # DR 아님
        self.assertEqual(out.ff_deadband_dps, config.CTRL_ANGULAR_VELOCITY_DEADBAND_DEG_S)


class TestL1SignConvention(unittest.TestCase):
    def _cmd_for_target(self, target_E, target_N):
        l1in = guidance.L1Input(
            valid=True, control_mode=guidance.ControlMode.GPS_TRACKING_CLOSED,
            confidence=1.0, E=0.0, N=0.0, V=6.0, course=0.0,
            target_E=target_E, target_N=target_N,
        )
        return guidance.ProduceL1Output(l1in)

    def test_positive_nu_gives_positive_yaw_rate(self):
        out = self._cmd_for_target(target_E=100.0, target_N=0.0)  # 동쪽 = 우현
        self.assertGreater(out.nu, 0.0)
        self.assertGreater(out.yaw_rate_cmd, 0.0)

    def test_negative_nu_gives_negative_yaw_rate(self):
        out = self._cmd_for_target(target_E=-100.0, target_N=0.0)  # 서쪽 = 좌현
        self.assertLess(out.nu, 0.0)
        self.assertLess(out.yaw_rate_cmd, 0.0)


class TestGyrzSignConvention(unittest.TestCase):
    def test_handle_imu_negates_gyrz(self):
        """motorapp.handle_imu: raw gyrz(+, CCW) → nav gyrz_rad_s(-, CW=우현)."""
        try:
            from Sensor_Motor import motorapp
        except Exception as exc:  # pragma: no cover - 환경 의존
            self.skipTest(f"motorapp import 불가: {exc}")
        # roll,pitch,yaw,ax,ay,az,gyrx,gyry,gyrz,health,sample_ts,yaw_offset
        motorapp.handle_imu("0,0,0,0,0,9.81,0,0,10,1,1000.0,0")
        gz = motorapp._CACHE_t.latest_imu.gyrz_rad_s
        self.assertIsNotNone(gz)
        self.assertAlmostEqual(gz, math.radians(-10.0), places=6)


class TestMotorLogSchemaParity(unittest.TestCase):
    def test_header_row_length_match(self):
        """추가 컬럼 후에도 헤더 길이 == 실제 row 길이 (CSV 소비 코드 보호)."""
        from unittest import mock
        from lib import sensorlog

        ctrl_out = control.CtrlOutput(timestamp=1.0,
                                      control_mode=guidance.ControlMode.DR_M_GBA_CLOSED)
        l1_out = guidance.L1Output(control_mode=guidance.ControlMode.DR_M_GBA_CLOSED)

        captured = []

        class _FakeWriter:
            def writerow(self, r):
                captured.append(r)

        with mock.patch.object(sensorlog, "_get_raw_writer", return_value=_FakeWriter()), \
             mock.patch.object(sensorlog, "_get_motor_control_writer", return_value=None), \
             mock.patch.object(sensorlog, "_flush_raw", lambda *a, **k: None):
            sensorlog.log_motor_raw(4, True, "GPS_GUIDED", ctrl_out, l1_out,
                                    snap=None, event="DR_M_GBA_CLOSED", l1_in=None)

        self.assertEqual(len(captured), 1, "row가 기록되지 않음(빌드 중 예외 가능)")
        self.assertEqual(len(captured[0]), len(sensorlog._MOTOR_RAW_HEADER))


if __name__ == "__main__":
    unittest.main()
