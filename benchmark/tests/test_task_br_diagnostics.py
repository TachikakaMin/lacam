"""PROTECTED Task-BR diagnostics and deliverable contract."""

import subprocess
import sys
import unittest
from pathlib import Path


class TestTaskBRDiagnostics(unittest.TestCase):
    REQUIRED = (
        "upper_epoch_builds",
        "pair_cache_hits",
        "pair_cache_misses",
        "pair_rollout_steps",
        "pair_rollout_truncations",
        "pair_rollout_stalls",
        "tau_guide_changes_on_upper_move",
        "joint_task_nodes",
        "joint_task_edges",
        "joint_shared_effects",
        "joint_effect_conflicts",
        "joint_candidate_backtracks",
        "joint_paused_roots",
        "ready_task_count",
        "rho_repairs",
        "custody_continuations",
        "zero_empty_no_ready",
        "deliverable_ms",
    )

    def test_runner_fields_persist_all_task_br_diagnostics(self):
        from run_benchmark import FIELDS

        for key in self.REQUIRED:
            self.assertIn(key, FIELDS)

    def test_success_deliverable_is_machine_checked(self):
        from run_benchmark import validate_deliverable_ms

        self.assertEqual(validate_deliverable_ms({"deliverable_ms": "9999.5"},
                                                10), 9999.5)
        with self.assertRaisesRegex(ValueError, "deliverable_ms"):
            validate_deliverable_ms({"deliverable_ms": "10000.1"}, 10)
        with self.assertRaisesRegex(ValueError, "deliverable_ms"):
            validate_deliverable_ms({}, 10)

    def test_runner_accepts_an_explicit_carrier_binary(self):
        runner = Path(__file__).parents[1] / "run_benchmark.py"
        result = subprocess.run(
            [sys.executable, str(runner), "--help"],
            capture_output=True,
            text=True,
            check=True,
        )
        self.assertIn("--carrier-bin", result.stdout)


if __name__ == "__main__":
    unittest.main()
