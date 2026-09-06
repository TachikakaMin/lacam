"""PROTECTED incremental-rho state-memory telemetry contract."""

import unittest


class TestRhoIncrementalMemoryMetrics(unittest.TestCase):
    def test_runner_persists_state_payload_metrics(self):
        from run_benchmark import FIELDS

        self.assertIn("rho_incremental_state_bytes_total", FIELDS)
        self.assertIn("rho_incremental_state_bytes_max", FIELDS)


if __name__ == "__main__":
    unittest.main()
