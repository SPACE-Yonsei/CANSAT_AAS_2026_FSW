import unittest

from Sensor_Motor import Motor_Release_Cal as mrc


class TestMotorReleaseCal(unittest.TestCase):
    def test_predict_trigger_before_target_ratio(self):
        st = mrc.ReleasePredictorState()
        max_alt = 1000.0
        # 90% 아래로 내려온 뒤 10 m/s 하강이면 80%(800m)까지 3초.
        samples = [
            (1.0, 890.0),
            (2.0, 860.0),
            (3.0, 850.0),
        ]
        fired = False
        first_reason = None
        for t, alt in samples:
            decision = mrc.should_trigger_release(
                st,
                now_s=t,
                alt_m=alt,
                max_alt_m=max_alt,
                burnwire_delay_sec=3.0,
            )
            fired = decision.trigger
            if fired and first_reason is None:
                first_reason = decision.reason
        self.assertTrue(fired)
        self.assertEqual(first_reason, "PREDICTIVE")

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
            decision = mrc.should_trigger_release(
                st,
                now_s=t,
                alt_m=alt,
                max_alt_m=max_alt,
                burnwire_delay_sec=3.0,
            )
            fired = decision.trigger
        self.assertFalse(fired)

    def test_force_trigger_at_hard_ratio(self):
        st = mrc.ReleasePredictorState()
        max_alt = 1000.0
        decision = mrc.should_trigger_release(
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
        self.assertTrue(decision.trigger)
        self.assertEqual(decision.reason, "FORCE_85_NO_PREDICTION")

    def test_no_force_85_when_prediction_exists(self):
        st = mrc.ReleasePredictorState()
        max_alt = 1000.0
        # Prediction becomes valid before crossing 85%.
        dec1 = mrc.should_trigger_release(
            st, now_s=1.0, alt_m=890.0, max_alt_m=max_alt, burnwire_delay_sec=0.5
        )
        dec2 = mrc.should_trigger_release(
            st, now_s=2.0, alt_m=870.0, max_alt_m=max_alt, burnwire_delay_sec=0.5
        )
        dec3 = mrc.should_trigger_release(
            st, now_s=3.0, alt_m=855.0, max_alt_m=max_alt, burnwire_delay_sec=0.5
        )
        dec4 = mrc.should_trigger_release(
            st, now_s=4.0, alt_m=845.0, max_alt_m=max_alt, burnwire_delay_sec=0.5
        )
        self.assertFalse(dec1.trigger)
        self.assertFalse(dec2.trigger)
        self.assertFalse(dec3.trigger)
        self.assertFalse(dec4.trigger)

    def test_force_after_90m_timeout(self):
        st = mrc.ReleasePredictorState()
        max_alt = 1000.0
        dec1 = mrc.should_trigger_release(
            st,
            now_s=10.0,
            alt_m=899.0,
            max_alt_m=max_alt,
            force_start_ratio=0.9,
            force_after_sec=5.0,
            burnwire_delay_sec=99.0,
            min_desc_rate_mps=999.0,
            min_samples=10,
        )
        dec2 = mrc.should_trigger_release(
            st,
            now_s=16.0,
            alt_m=898.0,
            max_alt_m=max_alt,
            force_start_ratio=0.9,
            force_after_sec=5.0,
            burnwire_delay_sec=99.0,
            min_desc_rate_mps=999.0,
            min_samples=10,
        )
        self.assertFalse(dec1.trigger)
        self.assertTrue(dec2.trigger)
        self.assertEqual(dec2.reason, "FORCE_90PCT_TIMEOUT")


if __name__ == "__main__":
    unittest.main()
