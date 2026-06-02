"""motorapp._ctrl_cycle: guidance-mode-centric control flow + neutral policy.

Verifies motorapp no longer creates its own fallback mode: every path is driven
by guidance.ControlMode, FAIL/invalid → neutral, and the L1Input→L1Output→step
order is honored. Guidance/control calls are patched to isolate cycle logic.
"""
import contextlib
import unittest
from unittest import mock

from Sensor_Motor import motorapp, control, guidance
from lib import config

ControlMode = guidance.ControlMode


def _valid_l1in(mode):
    return guidance.L1Input(
        valid=True, reason=mode.value, control_mode=mode,
        confidence=1.0, E=0.0, N=0.0, V=5.0, course=0.0, vE=0.0, vN=5.0,
        target_E=50.0, target_N=86.6,
    )


def _valid_l1out(mode):
    return guidance.L1Output(
        control_valid=True, valid=True, nominal=True, reason="OK",
        control_mode=mode, yaw_rate_cmd=0.3, ground_speed_mps=5.0,
    )


class TestMotorappCycle(unittest.TestCase):
    def setUp(self):
        motorapp.STATE = 3
        motorapp.MOTOR_ENABLED = True
        motorapp.PI = None
        motorapp._STEER_MODE = ""
        motorapp.MOTOR_CTRL_MODE = config.MOTOR_CTRL_MODE_GPS_GUIDED
        guidance.reset()
        guidance._STATE_t.nav.control_mode = ControlMode.FAIL
        self.log_calls = []

    def _decide(self, mode):
        def _f(gps, imu, baro, now):
            guidance._STATE_t.nav.control_mode = mode
            return mode
        return _f

    def _capture_log(self, *args, **kwargs):
        # positional: (state, motor_enabled, motor_ctrl_mode, ctrl_out, [l1_out])
        self.log_calls.append({
            "event": kwargs.get("event"),
            "ctrl_out": args[3],
            "reason": getattr(args[3], "reason", None),
            "l1_in": kwargs.get("l1_in"),
        })

    def _run(self, *, mode, l1in=None, l1out=None, ctrlout=None):
        self.m_l1out = mock.Mock(return_value=l1out)
        self.m_step = mock.Mock(return_value=ctrlout)
        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(motorapp, "_should_detumble",
                                                  return_value=False))
            stack.enter_context(mock.patch.object(motorapp.guidance, "DecideControlMode",
                                                  side_effect=self._decide(mode)))
            stack.enter_context(mock.patch.object(motorapp.sensorlog, "log_motor_raw",
                                                  side_effect=self._capture_log))
            self.move_mock = stack.enter_context(
                mock.patch.object(motorapp.control, "MoveServo"))
            stack.enter_context(mock.patch.object(motorapp, "_publish_motor_diag"))
            if l1in is not None:
                stack.enter_context(mock.patch.object(motorapp.guidance, "ProduceL1Input",
                                                      return_value=l1in))
            stack.enter_context(mock.patch.object(motorapp.guidance, "ProduceL1Output",
                                                  self.m_l1out))
            stack.enter_context(mock.patch.object(motorapp.control, "step", self.m_step))
            return motorapp._ctrl_cycle(None, 100.0)

    # ── Case 1: FAIL → neutral, no self-fallback ────────────────────────────
    def test_case_1_fail_neutral(self):
        out = self._run(mode=ControlMode.FAIL,
                        l1in=guidance.L1Input(valid=False, reason="FAIL",
                                              control_mode=ControlMode.FAIL))
        self.m_l1out.assert_not_called()       # no L1Output / step on FAIL
        self.m_step.assert_not_called()
        self.assertEqual(self.log_calls[-1]["event"], "FAIL")
        self.assertEqual(out.left_pw, control.LEFT_NEUTRAL)
        self.assertEqual(out.right_pw, control.RIGHT_NEUTRAL)
        self.assertEqual(out.reason, "FAIL")

    # ── Case 2: DR_PM_GB_CLOSED → full pipeline + servo ─────────────────────
    def test_case_2_dr_pm_gb_full_pipeline(self):
        mode = ControlMode.DR_PM_GB_CLOSED
        ctrlout = control.CtrlOutput(timestamp=100.0, valid=True, reason="OK",
                                     left_pw=1500, right_pw=1500, control_mode=mode)
        out = self._run(mode=mode, l1in=_valid_l1in(mode),
                        l1out=_valid_l1out(mode), ctrlout=ctrlout)
        self.m_l1out.assert_called_once()
        self.m_step.assert_called_once()
        self.move_mock.assert_called_once()
        self.assertEqual(self.log_calls[-1]["event"], "DR_PM_GB_CLOSED")
        self.assertIs(out, ctrlout)

    # ── Case 3: L1Input invalid → neutral + reason ──────────────────────────
    def test_case_3_l1input_invalid(self):
        mode = ControlMode.DR_PM_G_CLOSED
        l1in = guidance.L1Input(valid=False, reason="NO_SPEED_SOURCE", control_mode=mode)
        out = self._run(mode=mode, l1in=l1in)
        self.m_l1out.assert_not_called()
        self.m_step.assert_not_called()
        self.assertEqual(out.reason, "NO_SPEED_SOURCE")
        self.assertEqual(out.left_pw, control.LEFT_NEUTRAL)
        self.assertEqual(self.log_calls[-1]["event"], "DR_PM_G_CLOSED")

    # ── Case 4: L1Output invalid → neutral + reason ─────────────────────────
    def test_case_4_l1output_invalid(self):
        mode = ControlMode.DR_PM_GB_CLOSED
        l1out = guidance.L1Output(control_valid=False, valid=False,
                                  reason="LOW_CONFIDENCE", control_mode=mode)
        out = self._run(mode=mode, l1in=_valid_l1in(mode), l1out=l1out)
        self.m_l1out.assert_called_once()
        self.m_step.assert_not_called()
        self.assertEqual(out.reason, "LOW_CONFIDENCE")
        self.assertEqual(out.left_pw, control.LEFT_NEUTRAL)

    # ── Case 5: control invalid → neutral (step returns neutral pulses) ─────
    def test_case_5_control_invalid(self):
        mode = ControlMode.GPS_TRACKING_CLOSED
        ctrlout = control.CtrlOutput(timestamp=100.0, valid=False, reason="NAN_CMD",
                                     control_mode=mode)  # defaults → neutral pulses
        out = self._run(mode=mode, l1in=_valid_l1in(mode),
                        l1out=_valid_l1out(mode), ctrlout=ctrlout)
        self.m_step.assert_called_once()
        self.move_mock.assert_called_once()
        self.assertFalse(out.valid)
        self.assertEqual(out.reason, "NAN_CMD")
        self.assertEqual(out.left_pw, control.LEFT_NEUTRAL)


if __name__ == "__main__":
    unittest.main()
