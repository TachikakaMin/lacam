"""PROTECTED incremental-rho shadow telemetry export contract."""

import unittest


class TestRhoIncrementalMetrics(unittest.TestCase):
    REQUIRED = (
        "rho_incremental_full_solves",
        "rho_incremental_repairs",
        "rho_incremental_zero_row_reuses",
        "rho_incremental_augmentations",
        "rho_incremental_changed_rows_0",
        "rho_incremental_changed_rows_1",
        "rho_incremental_changed_rows_2",
        "rho_incremental_changed_rows_gt2",
        "rho_shadow_mismatches",
        "rho_incremental_copy_time_ms",
        "rho_incremental_repair_time_ms",
        "rho_incremental_full_time_ms",
        "rho_fallback_no_parent_state",
        "rho_fallback_stale_or_rewired_parent",
        "rho_fallback_shape_changed",
        "rho_fallback_column_identity_changed",
        "rho_fallback_column_value_changed",
        "rho_fallback_mode_changed",
        "rho_fallback_conflict_changed",
        "rho_fallback_objective_version_changed",
        "rho_fallback_scaling_version_changed",
        "rho_fallback_inf_version_changed",
        "rho_fallback_canonical_version_changed",
        "rho_fallback_state_validation_failed",
        "rho_fallback_shadow_mismatch",
    )

    def test_runner_persists_incremental_shadow_metrics(self):
        from run_benchmark import FIELDS

        for key in self.REQUIRED:
            self.assertIn(key, FIELDS)


if __name__ == "__main__":
    unittest.main()
