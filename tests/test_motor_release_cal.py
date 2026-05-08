import unittest

from Sensor_Motor import Motor_Release_Cal as mrc


class TestMotorReleaseCal(unittest.TestCase):
    def test_predict_trigger_before_target_ratio(self):
        st = mrc.ReleasePredictorState()
        max_alt = 1000.0
        # 90% 아래로 내려온 뒤 10 m/s 하강이면 80%(800m)까지 3초.
        samples = [
            (1.0, 860.0),
            (2.0, 850.0),
            (3.0, 830.0),
        ]
        fired = False
        for t, alt in samples:
            fired = mrc.should_trigger_release(
                st,
                now_s=t,
                alt_m=alt,
                max_alt_m=max_alt,
                burnwire_delay_sec=3.0,
            )
        self.assertTrue(fired)

    def test_no_trigger_when_descent_too_slow(self):
        st = mrc.ReleasePredictorState()
        max_alt = 1000.0
        samples = [
            (1.0, 880.0),
            (2.0, 879.9),
            (3.0, 879.8),
        ]
        fired = False
        for t, alt in samples:
            fired = mrc.should_trigger_release(
                st,
                now_s=t,
                alt_m=alt,
                max_alt_m=max_alt,
                burnwire_delay_sec=3.0,
            )
        self.assertFalse(fired)

    def test_force_trigger_at_hard_ratio(self):
        st = mrc.ReleasePredictorState()
        max_alt = 1000.0
        fired = mrc.should_trigger_release(
            st,
            now_s=1.0,
            alt_m=849.0,
            max_alt_m=max_alt,
            target_ratio=0.8,
            predict_start_ratio=0.9,
            hard_trigger_ratio=0.85,
            burnwire_delay_sec=99.0,
            min_desc_rate_mps=999.0,
            min_samples=10,
        )
        self.assertTrue(fired)


if __name__ == "__main__":
    unittest.main()
