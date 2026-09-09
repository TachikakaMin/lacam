"""PROTECTED benchmark export contract for incremental rho telemetry."""

import unittest


class TestRhoIncrementalMetrics(unittest.TestCase):
    REQUIRED = (
        "rho_incremental_full_solves",
        "rho_incremental_repairs",
        "rho_incremental_zero_row_reuses",
        "rho_incremental_changed_rows_total",
    )

    def test_runner_persists_incremental_rho_metrics(self):
        from run_benchmark import FIELDS

        for key in self.REQUIRED:
            self.assertIn(key, FIELDS)


if __name__ == "__main__":
    unittest.main()
