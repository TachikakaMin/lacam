import csv
import json
import tempfile
import unittest
from pathlib import Path

from generate_rho_v2_final_report import generate_report


class RhoV2FinalReportTest(unittest.TestCase):
    def _write_rows(self, path, records):
        fields = [
            "instance",
            "family",
            "method",
            "success",
            "executed_makespan",
            "weighted_soc",
            "weighted_work_scaled",
            "first_solution_ms",
            "rho_objective_version",
            "rho_candidates_input",
            "rho_candidates_after_priority",
            "rho_priority_filtered",
            "rho_matrix_rows_total",
            "rho_assignment_changes",
            "guidance_time_ms",
            "plan_sha256",
            "runtime_sec",
            "status",
        ]
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writer.writerows(records)

    def _row(self, name, family, success, makespan="", work="",
             first_ms="", runtime=1.0):
        scaled = int(work * 1_000_000) if success else ""
        return {
            "instance": name,
            "family": family,
            "method": "carrier",
            "success": int(success),
            "executed_makespan": makespan,
            "weighted_soc": work,
            "weighted_work_scaled": scaled,
            "first_solution_ms": first_ms,
            "rho_objective_version":
                "BOTTLENECK_TARGET_FRONTIER_CONTINUITY_V2"
                if success else "",
            "rho_candidates_input": 10 if success else "",
            "rho_candidates_after_priority": 8 if success else "",
            "rho_priority_filtered": 0 if success else "",
            "rho_matrix_rows_total": 8 if success else "",
            "rho_assignment_changes": 3 if success else "",
            "guidance_time_ms": 5 if success else "",
            "plan_sha256": (name * 8)[:64] if success else "",
            "runtime_sec": runtime,
            "status": "ok" if success else "timeout",
        }

    def _write_timing(self, path, solved, total, wall, binary):
        path.write_text(
            json.dumps(
                {
                    "wall_time_sec": wall,
                    "jobs": 3,
                    "n_tasks": total,
                    "timeout_per_run_sec": 10,
                    "provenance": {"binary_sha256": binary},
                    "methods": {
                        "carrier": {
                            "solved": solved,
                            "total": total,
                            "solver_time_sum_sec": wall * 2,
                        }
                    },
                    "suite": {"definition_sha256": "s" * 64},
                }
            ),
            encoding="utf-8",
        )

    def test_report_is_derived_and_keeps_negative_evidence(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            baseline_rows = root / "baseline.csv"
            current_rows = root / "current.csv"
            baseline_timing = root / "baseline-timing.json"
            current_timing = root / "current-timing.json"
            manifest = root / "manifest.json"
            evaluation = root / "evaluation.json"
            approval = root / "approval.json"
            out = root / "site"

            baseline = [
                self._row("quick-a", "quick", True, 10, 20, 5),
                self._row("quick-b", "quick", True, 20, 30, 7),
                self._row(
                    "factor-a", "warehouse_random_factorial",
                    True, 10, 20, 9,
                ),
                self._row(
                    "factor-b", "warehouse_random_factorial",
                    True, 10, 20, 11,
                ),
                self._row("fail", "quick", False, runtime=10),
            ]
            current = [
                self._row("quick-a", "quick", True, 8, 22, 4),
                self._row("quick-b", "quick", True, 20, 30, 8),
                self._row(
                    "factor-a", "warehouse_random_factorial",
                    True, 12, 24, 10,
                ),
                self._row(
                    "factor-b", "warehouse_random_factorial",
                    True, 10, 18, 6,
                ),
                self._row("fail", "quick", False, runtime=10),
            ]
            self._write_rows(baseline_rows, baseline)
            self._write_rows(current_rows, current)
            self._write_timing(
                baseline_timing, 4, 5, 12.0, "a" * 64
            )
            self._write_timing(
                current_timing, 4, 5, 13.0, "b" * 64
            )
            manifest.write_text(
                json.dumps(
                    [
                        {
                            "id": "factor-a",
                            "map_family": "g1",
                            "density_level": "low",
                            "agent_level": "scarce",
                            "task_profile": "local",
                            "goal_mode": "singleton",
                        },
                        {
                            "id": "factor-b",
                            "map_family": "g1",
                            "density_level": "low",
                            "agent_level": "equal",
                            "task_profile": "mixed",
                            "goal_mode": "shared_pool",
                        },
                    ]
                ),
                encoding="utf-8",
            )
            evaluation.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "production_commit": "deadbee",
                        "objective_version":
                            "BOTTLENECK_TARGET_FRONTIER_CONTINUITY_V2",
                        "protected_examples": [
                            {
                                "label": "示例 X",
                                "before_label": "F2S",
                                "before": [7, 11],
                                "after": [6, 9],
                            }
                        ],
                        "rejected_experiments": [
                            {
                                "label": "additive",
                                "reason": "目标回归",
                                "testcase_cost": [99, 101],
                            }
                        ],
                        "known_issues": [
                            "相同成本可能对应不同计划 SHA"
                        ],
                    }
                ),
                encoding="utf-8",
            )
            approval.write_text(
                json.dumps(
                    {
                        "decision": "APPROVE",
                        "reviewer_model": "openai.gpt-5.6-sol",
                        "binary_sha256": "b" * 64,
                    }
                ),
                encoding="utf-8",
            )

            data = generate_report(
                baseline_rows_path=baseline_rows,
                baseline_timing_path=baseline_timing,
                current_rows_path=current_rows,
                current_timing_path=current_timing,
                manifest_path=manifest,
                evaluation_path=evaluation,
                approval_path=approval,
                out_dir=out,
                cpp_tests=12,
                python_tests=34,
            )
            page = (out / "index.html").read_text(encoding="utf-8")
            summary = json.loads(
                (out / "summary.json").read_text(encoding="utf-8")
            )

        self.assertEqual(data["overview"]["current_solved"], 4)
        self.assertEqual(
            data["overview"]["lexicographic"]["better_equal_worse"],
            [2, 1, 1],
        )
        self.assertEqual(
            summary["factorial"]["lexicographic"]["better_equal_worse"],
            [1, 0, 1],
        )
        self.assertIn("4/5", page)
        self.assertIn("2 / 1 / 1", page)
        self.assertIn("示例 X", page)
        self.assertIn("(7, 11) → (6, 9)", page)
        self.assertIn("99, 101", page)
        self.assertIn("scarce", page)
        self.assertIn("12 / 12", page)
        self.assertIn("34 / 34", page)
        self.assertIn("相同成本可能对应不同计划 SHA", page)
        self.assertNotIn("479/509", page)
        self.assertNotIn("60 / 261 / 158", page)
        self.assertNotIn("full 中 60 个实例", page)
        self.assertNotIn("432 factorial", page)
        self.assertNotIn("../../../results", page)


if __name__ == "__main__":
    unittest.main()
