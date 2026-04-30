import unittest

from Sensor_Distance import distanceapp


class TestDistanceApp(unittest.TestCase):
    def setUp(self):
        distanceapp.DISTANCE_MM = 5000.0

    def test_median_mm(self):
        self.assertEqual(distanceapp._median_mm([1000, 1100, 1200]), 1100)
        self.assertEqual(distanceapp._median_mm([1000, 1100, 1200, 1300]), 1150)

    def test_valid_distance(self):
        self.assertTrue(distanceapp._is_valid_distance(200))
        self.assertTrue(distanceapp._is_valid_distance(8000))
        self.assertFalse(distanceapp._is_valid_distance(199))
        self.assertFalse(distanceapp._is_valid_distance(9000))

    def test_synthetic_read_moves_toward_target(self):
        d1 = distanceapp._synthetic_read_distance()
        distanceapp.DISTANCE_MM = d1
        d2 = distanceapp._synthetic_read_distance()
        self.assertLessEqual(d2, d1)


if __name__ == "__main__":
    unittest.main()
