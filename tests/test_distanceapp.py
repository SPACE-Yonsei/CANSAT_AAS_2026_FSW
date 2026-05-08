import unittest

from Sensor_Distance import distanceapp


class TestDistanceApp(unittest.TestCase):
    def setUp(self):
        distanceapp.DISTANCE_MM = 0.0

    def test_median_mm(self):
        self.assertEqual(distanceapp._median_mm([1000, 1100, 1200]), 1100)
        self.assertEqual(distanceapp._median_mm([1000, 1100, 1200, 1300]), 1150)

    def test_valid_distance(self):
        self.assertTrue(distanceapp._is_valid_distance(200))
        self.assertTrue(distanceapp._is_valid_distance(8000))
        self.assertFalse(distanceapp._is_valid_distance(199))
        self.assertFalse(distanceapp._is_valid_distance(9000))

    def test_fallback_distance_is_zero(self):
        self.assertEqual(distanceapp._synthetic_read_distance(), 0.0)


if __name__ == "__main__":
    unittest.main()
