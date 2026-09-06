"""PROTECTED additive rho benchmark-schema contract."""

import unittest


class TestRhoAdditiveMetrics(unittest.TestCase):
    REQUIRED = (
        "rho_objective_version",
        "rho_additive_full_time_ms",
        "rho_task_assignments",
        "rho_idle_assignments",
    )

    def test_runner_persists_additive_objective_metrics(self):
        from run_benchmark import FIELDS

        for key in self.REQUIRED:
            self.assertIn(key, FIELDS)


if __name__ == "__main__":
    unittest.main()
