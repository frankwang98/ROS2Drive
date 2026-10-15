import unittest

from collect_metrics import summarize


class BenchmarkSummaryTest(unittest.TestCase):
    def test_empty_capture(self):
        report = summarize([], [], 1.0, "")
        self.assertIsNone(report["loop_ms_p95"])
        self.assertEqual(report["distance_traveled_m"], 0)
        self.assertIsNone(report["fault_sample_ratio"])

    def test_statistics(self):
        samples = [
            {"loop_duration_ms": 1.0, "active_faults": 0, "trajectory_points": 10},
            {"loop_duration_ms": 2.0, "active_faults": 1, "trajectory_points": 20},
            {"loop_duration_ms": 4.0, "active_faults": 0, "trajectory_points": 30},
        ]
        report = summarize(samples, [(0, 0), (3, 4)], 3.0, "car01")
        self.assertEqual(report["loop_ms_p95"], 4.0)
        self.assertEqual(report["distance_traveled_m"], 5.0)
        self.assertEqual(report["samples_with_faults"], 1)
        self.assertAlmostEqual(report["fault_sample_ratio"], 1 / 3)
        self.assertEqual(report["stationary_odometry_steps"], 0)


if __name__ == "__main__":
    unittest.main()
