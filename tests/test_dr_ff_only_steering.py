"""DR = FF-only 조향 회귀 테스트.

배경(실측): DR 구간에서 V가 바닥(~0.5 m/s)이라 L1 yaw_rate_cmd가 작고(≤~5 dps),
GPS FF 데드밴드(5)에 걸려 delta_ff=0이 되었다. 남은 rate-damping PID가 측정 gyro만
죽이려다 heading 보정과 반대로(nu와 반대 부호) 팔을 움직였다(부호 반대 ~29%).

수정(1): DR은 기본적으로 PID 제거(FF-only). gyro가 heading FF를 뒤집지 못한다.
수정(2, 20260606 개정): DR FF는 GPS_TRACKING_CLOSED와 "동일한" 곡선(같은 deadband=5,
같은 정규화 기준=GPS_CLOSED limit)을 타고 per-mode limit과 ff_scale(<1)로만 약화된다.
→ 같은 (nu,V)에서 항상 DR ≤ GPS (강도 역전 없음). 저속 응답은 L1_STEER_V_FLOOR_MPS가
보강한다. GPS 모드 동작은 불변. 본 테스트는 그 동작을 고정한다.
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
        FF-only DR은 명령 부호(우)대로 팔을 움직인다. (이전엔 PID가 -방향으로 뒤집음)
        명령은 GPS와 공유하는 데드밴드(5 dps)보다 큰 대표값을 쓴다(실비행에선
        L1_STEER_V_FLOOR 덕에 의미있는 nu에서 cmd>5 dps)."""
        out = _dr_cmd(cmd_dps=+12.0, gyrz_meas_dps=+31.45)
        self.assertGreater(out.delta_total_deg, 0.0)
        self.assertGreater(out.delta_arm_deg, 0.0)
        self.assertEqual(out.delta_pid_deg, 0.0)        # PID 제거 → 기여 없음
        self.assertFalse(out.dr_pid_active)
        # 부호 일치(명령 ↔ 최종 팔)
        self.assertEqual(math.copysign(1.0, out.delta_total_deg), 1.0)

    def test_dr_ff_only_left_command_despite_opposing_gyro(self):
        """좌회전 명령(nu<0) + 좌로 회전 중(gyro<0)에도 팔은 좌(-)로 간다."""
        out = _dr_cmd(cmd_dps=-12.0, gyrz_meas_dps=-31.45)
        self.assertLess(out.delta_total_deg, 0.0)
        self.assertLess(out.delta_arm_deg, 0.0)
        self.assertEqual(out.delta_pid_deg, 0.0)

    def test_dr_pid_never_active_even_with_huge_gyro(self):
        out = _dr_cmd(cmd_dps=+12.0, gyrz_meas_dps=+200.0)
        self.assertEqual(out.delta_pid_deg, 0.0)
        self.assertFalse(out.dr_pid_active)
        self.assertGreater(out.delta_total_deg, 0.0)


class TestDrFeedforwardDeadbandAndSensitivity(unittest.TestCase):
    def test_dr_uses_gps_deadband_and_ref(self):
        """DR은 GPS와 동일한 FF 데드밴드(5)와 GPS_CLOSED 정규화 기준을 쓴다(분리 노브 폐기)."""
        out = _dr_cmd(cmd_dps=20.0, gyrz_meas_dps=0.0)   # DR_M_GBA_CLOSED
        self.assertAlmostEqual(out.ff_deadband_dps,
                               config.CTRL_ANGULAR_VELOCITY_DEADBAND_DEG_S, places=9)
        self.assertAlmostEqual(out.ff_ref_dps,
                               config.GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS, places=9)

    def test_ff_ref_smaller_than_limit_increases_deflection(self):
        """angular_velocity_to_delta_ff 함수 계약: ref<limit이면 같은 명령에서 더 큰
        deflection(이 곡선 특성 자체는 유효하나, DR 경로는 더 이상 작은 ref를 쓰지 않는다)."""
        cmd, limit = 3.77, 50.0
        d_ref_small = control.angular_velocity_to_delta_ff(
            cmd, limit, deadband_dps=1.0, ref_dps=6.0)
        d_ref_limit = control.angular_velocity_to_delta_ff(
            cmd, limit, deadband_dps=1.0, ref_dps=limit)
        self.assertGreater(d_ref_small, d_ref_limit)
        self.assertGreater(d_ref_limit, 0.0)

    def test_ff_monotonic_in_command(self):
        small = control.angular_velocity_to_delta_ff(8.0, 60.0, deadband_dps=5.0, ref_dps=60.0)
        large = control.angular_velocity_to_delta_ff(30.0, 60.0, deadband_dps=5.0, ref_dps=60.0)
        self.assertGreater(large, small)

    def test_dr_ff_delta_cap(self):
        """DR 권한 상한 cap이 실제로 동작(높은 명령에서 pre-cap > cap)."""
        cap = config.DR_FF_DELTA_LIMIT_DEG
        # 큰 명령(GPS limit)으로 강제해 DR 곡선이 cap을 넘게 한다.
        out = _dr_cmd(cmd_dps=config.GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS,
                      gyrz_meas_dps=0.0,
                      limit=config.GPS_TRACKING_CLOSED_YAW_RATE_LIMIT_DPS)
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


class TestDrNotStrongerThanGps(unittest.TestCase):
    """핵심 보장: 같은 (nu, V)에서 어떤 DR 모드도 GPS_TRACKING_CLOSED보다 세게(=더 큰
    팔 변위로) 조향하지 않는다. 또한 방향(부호)은 GPS와 동일하다.

    전체 파이프라인(ProduceL1Output → ProduceCtrlInput → ProduceCtrlOutput)을 실제
    per-mode yaw-rate limit과 함께 돌려 비교한다. DR confidence=1.0(최강 DR)로 둔다.
    """

    DR_MODES = [
        guidance.ControlMode.DR_M_GBA_CLOSED,
        guidance.ControlMode.DR_M_GB_CLOSED,
        guidance.ControlMode.DR_M_G_CLOSED,
        guidance.ControlMode.DR_M_YBA_OPEN,
        guidance.ControlMode.DR_M_YB_OPEN,
        guidance.ControlMode.DR_M_Y_OPEN,
        guidance.ControlMode.DR_PM_GBA_CLOSED,
        guidance.ControlMode.DR_PM_GB_CLOSED,
        guidance.ControlMode.DR_PM_G_CLOSED,
        guidance.ControlMode.DR_PM_YBA_OPEN,
        guidance.ControlMode.DR_PM_YB_OPEN,
        guidance.ControlMode.DR_PM_Y_OPEN,
    ]

    @staticmethod
    def _delta_ff_for(mode, nu_deg, V, conf=1.0):
        nu = math.radians(nu_deg)
        dist = 200.0
        is_gps = mode in (guidance.ControlMode.GPS_TRACKING_CLOSED,
                          guidance.ControlMode.GPS_TRACKING_OPEN)
        l1in = guidance.L1Input(
            valid=True, reason=mode.value, control_mode=mode,
            dr_method=(guidance.DRMethod.NONE if is_gps
                       else guidance.DRMethod.GYRO_INTEGRATION),
            confidence=conf, E=0.0, N=0.0, V=V, course=0.0,
            vE=0.0, vN=V,
            target_E=dist * math.sin(nu), target_N=dist * math.cos(nu),
        )
        l1out = guidance.ProduceL1Output(l1in)
        control.reset()
        ci = control.ProduceCtrlInput(l1out, now=1.0)
        # gyrz_meas=0: GPS는 FF+PID 최대 권한 — DR이 그보다도 작아야 함(보수적 비교).
        co = control.ProduceCtrlOutput(ci, angular_velocity_meas_deg_s=0.0, now=1.0)
        return l1out, co

    def test_dr_delta_ff_never_exceeds_gps_closed(self):
        nus = [3.0, 6.0, 10.0, 20.0, 30.0, 45.0, 60.0, 90.0]
        Vs = [4.0, 5.0, 6.0, 6.5]
        eps = 1e-6
        for V in Vs:
            for nu in nus:
                _, gps = self._delta_ff_for(
                    guidance.ControlMode.GPS_TRACKING_CLOSED, nu, V)
                for mode in self.DR_MODES:
                    _, dr = self._delta_ff_for(mode, nu, V)
                    with self.subTest(mode=mode.value, nu=nu, V=V):
                        # 세기: DR FF ≤ GPS_CLOSED FF
                        self.assertLessEqual(
                            dr.delta_ff_deg, gps.delta_ff_deg + eps,
                            f"{mode.value} delta_ff={dr.delta_ff_deg:.3f} > "
                            f"GPS={gps.delta_ff_deg:.3f} at nu={nu},V={V}")
                        # 총 팔 변위도 GPS_CLOSED(FF+PID) 이하
                        self.assertLessEqual(
                            abs(dr.delta_arm_deg), abs(gps.delta_arm_deg) + eps)

    def test_dr_ff_proportional_not_saturated_at_typical_cmd(self):
        """실측(run_20260606_203310) DR 명령 분포 |cmd| 중앙값 ~24 dps. NEW config에서
        이 영역의 DR FF는 cap(±60)에 붙지 않고 비례 응답이어야 한다(OLD ref=6는 94%가
        cap에 붙는 뱅뱅이었다). 대표점 V=6, nu=20°(cmd≈29 dps)로 고정한다."""
        cap = config.DR_FF_DELTA_LIMIT_DEG
        _, dr = self._delta_ff_for(guidance.ControlMode.DR_M_GBA_CLOSED, 20.0, 6.0)
        self.assertGreater(abs(dr.delta_ff_deg), config.CTRL_DELTA_MIN_EFFECTIVE_DEG)
        self.assertLess(abs(dr.delta_ff_deg), cap - 1.0)   # 포화 아님(비례 구간)
        # pre-cap도 cap을 넘지 않아야 비례(전 영역 뱅뱅이 아님을 확인)
        self.assertLessEqual(abs(dr.delta_ff_pre_cap_deg), cap + 1e-6)

    def test_dr_direction_matches_gps(self):
        for nu in (5.0, 20.0, 60.0):
            _, gps = self._delta_ff_for(
                guidance.ControlMode.GPS_TRACKING_CLOSED, nu, 6.0)
            for mode in self.DR_MODES:
                _, dr = self._delta_ff_for(mode, nu, 6.0)
                with self.subTest(mode=mode.value, nu=nu):
                    if abs(dr.delta_arm_deg) > 1e-6:
                        self.assertEqual(
                            math.copysign(1.0, dr.delta_arm_deg),
                            math.copysign(1.0, gps.delta_arm_deg))


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

    def test_handle_imu_yaw_matches_gyrz_rotation_sense(self):
        """yaw도 gz와 같은 nav 회전 규약(우회전 +)이어야 DR yaw-course가
        gyro-course/GPS와 같은 nu 부호로 조향한다.

        BNO는 Z-up CCW+: 물리적 우회전(CW)이면 raw yaw 감소 + raw gz 음수.
        nav 변환 후엔 둘 다 '우회전 = +'여야 한다 (gz>0, yaw 증가).
        수정 전엔 yaw_rad=+yaw_deg라 우회전 시 yaw가 감소 → DR yaw-course 역조향.
        """
        try:
            from Sensor_Motor import motorapp
        except Exception as exc:  # pragma: no cover - 환경 의존
            self.skipTest(f"motorapp import 불가: {exc}")
        # roll,pitch,yaw,ax,ay,az,gyrx,gyry,gyrz,health,sample_ts,yaw_offset
        motorapp.handle_imu("0,0,20,0,0,9.81,0,0,-25,1,1000.0,0")
        yaw_a = motorapp._CACHE_t.latest_imu.yaw_rad
        motorapp.handle_imu("0,0,15,0,0,9.81,0,0,-25,1,1000.1,0")  # 우회전: raw yaw 20→15
        snap = motorapp._CACHE_t.latest_imu
        self.assertGreater(snap.gyrz_rad_s, 0.0)        # 우회전 → nav gz > 0
        self.assertGreater(snap.yaw_rad, yaw_a)         # 우회전 → nav yaw 증가(동일 방향)


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
