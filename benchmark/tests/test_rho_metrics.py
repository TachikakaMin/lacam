"""PROTECTED rho matching telemetry export contract."""

import unittest


class TestRhoMetrics(unittest.TestCase):
    REQUIRED = (
        "rho_match_calls_execute",
        "rho_match_calls_prepare",
        "rho_candidates_input",
        "rho_candidates_after_claims",
        "rho_candidates_after_key_dedupe",
        "rho_candidates_after_shelf_preselect",
        "rho_candidates_after_priority",
        "rho_priority_filtered",
        "rho_matrix_rows_total",
        "rho_matrix_cols_total",
        "rho_matrix_max_rows",
        "rho_candidate_time_ms",
        "rho_matrix_time_ms",
        "rho_bottleneck_time_ms",
        "rho_secondary_full_time_ms",
        "rho_canonical_time_ms",
        "rho_column_identity_same",
        "rho_column_value_same",
        "rho_mode_or_conflict_same",
        "rho_changed_rows_0",
        "rho_changed_rows_1",
        "rho_changed_rows_2",
        "rho_changed_rows_gt2",
        "rho_assignment_changes",
    )

    def test_runner_persists_rho_metrics(self):
        from run_benchmark import FIELDS

        for key in self.REQUIRED:
            self.assertIn(key, FIELDS)


if __name__ == "__main__":
    unittest.main()
