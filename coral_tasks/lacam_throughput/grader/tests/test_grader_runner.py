#!/usr/bin/env python3
"""Unit tests for grader runner integration helpers."""

from __future__ import annotations

import importlib.util
import sys
import types
import unittest
from pathlib import Path


def load_grader_module():
    coral = types.ModuleType("coral")
    coral_grader = types.ModuleType("coral.grader")
    coral_types = types.ModuleType("coral.types")

    class TaskGrader:
        pass

    class ScoreBundle:
        pass

    coral_grader.TaskGrader = TaskGrader
    coral_types.ScoreBundle = ScoreBundle
    sys.modules["coral"] = coral
    sys.modules["coral.grader"] = coral_grader
    sys.modules["coral.types"] = coral_types

    module_path = (
        Path(__file__).resolve().parents[1]
        / "src"
        / "lacam_throughput_grader"
        / "grader.py"
    )
    spec = importlib.util.spec_from_file_location("grader_under_test", module_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class GraderRunnerCommandTest(unittest.TestCase):
    def setUp(self) -> None:
        self.grader = load_grader_module()

    def test_runner_command_uses_existing_parallel_runner_and_fixed_scenario(self) -> None:
        repo = Path("/repo")
        log_dir = Path("/logs/attempt")
        seeds = [10, 11]

        cmd = self.grader._runner_command(
            repo=repo,
            binary=repo / "build" / "coral_eval" / "lifelong_benchmark",
            log_dir=log_dir,
            seeds=seeds,
            workers=4,
        )

        self.assertIn(str(repo / "tools" / "run_symbotic_requested_grid.py"), cmd)
        self.assertIn("--seeds", cmd)
        self.assertIn("10,11", cmd)
        self.assertIn("--workers", cmd)
        self.assertIn("4", cmd)
        self.assertIn("--maps", cmd)
        self.assertIn("symbotic_star", cmd)
        self.assertIn("--ks", cmd)
        self.assertIn("2", cmd)
        self.assertIn("--agent-counts", cmd)
        self.assertIn("100", cmd)
        self.assertIn("--out-dir", cmd)
        self.assertIn(str(log_dir / "runner"), cmd)
        slot_index = cmd.index("--slots")
        self.assertEqual(cmd[slot_index + 1], "-1")
        cost_index = cmd.index("--cost-modes")
        self.assertEqual(cmd[cost_index + 1], "-1")
        service_index = cmd.index("--service-commit-agents")
        self.assertEqual(cmd[service_index + 1], "-1")

    def test_real_feedback_hides_held_out_seed_values(self) -> None:
        rows = [
            {
                "seed": "1538",
                "valid": "1",
                "throughput": "1.0",
                "completed_tasks": "400",
                "alternating_throughput": "0.5",
                "total_planner_runtime": "10",
                "total_assignment_runtime": "20",
            }
        ]

        feedback = self.grader._feedback(
            Path("/logs/attempt"), [1538], rows, [], reveal_seeds=False
        )

        self.assertIn("Hidden held-out seed set: 1 cases", feedback)
        self.assertNotIn("1538", feedback)
        self.assertNotIn("Per-seed results", feedback)

    def test_real_summary_metadata_hides_held_out_seeds_and_rows(self) -> None:
        rows = [
            {
                "seed": "1538",
                "throughput": "1.0",
                "completed_tasks": "400",
                "alternating_throughput": "0.5",
                "total_planner_runtime": "10",
                "total_assignment_runtime": "20",
                "average_delivery_time": "30",
                "pickup_while_loaded_count": "40",
            }
        ]

        summary = self.grader._summarize(
            rows, seeds=[1538], mode="real", reveal_seeds=False
        )

        self.assertEqual(summary["hidden_seed_count"], 1)
        self.assertNotIn("seeds", summary)
        self.assertNotIn("rows", summary)

    def test_tune_feedback_keeps_seed_diagnostics(self) -> None:
        rows = [
            {
                "seed": "0",
                "valid": "1",
                "throughput": "1.0",
                "completed_tasks": "400",
                "alternating_throughput": "0.5",
                "total_planner_runtime": "10",
                "total_assignment_runtime": "20",
            }
        ]

        feedback = self.grader._feedback(
            Path("/logs/attempt"), [0], rows, [], reveal_seeds=True
        )

        self.assertIn("Seeds: [0]", feedback)
        self.assertIn("Per-seed results", feedback)
        self.assertIn("0 | 1 | 1.0", feedback)


if __name__ == "__main__":
    unittest.main()
