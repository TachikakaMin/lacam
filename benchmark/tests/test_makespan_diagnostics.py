"""Makespan quality and timed-guidance diagnostics stay exportable."""

import unittest


class TestMakespanDiagnostics(unittest.TestCase):
    def test_runner_persists_quality_and_waiting_metrics(self):
        from run_benchmark import FIELDS

        required = (
            "first_solution_makespan",
            "first_solution_soc",
            "first_solution_work_scaled",
            "best_makespan",
            "best_soc",
            "best_work_scaled",
            "weighted_work_scaled",
            "improvement_attempts",
            "improvement_candidates",
            "improvement_improvements",
            "improvement_generator_failures",
            "improvement_exit_reason",
            "timed_transport_expansions",
            "timed_transport_frames",
            "timed_transport_time_ms",
            "owner_handoffs",
            "causal_waiting",
            "traffic_waiting",
        )
        for key in required:
            self.assertIn(key, FIELDS)


if __name__ == "__main__":
    unittest.main()
