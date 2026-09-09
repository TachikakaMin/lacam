"""Dense-channel V2 dashboard contracts."""

import csv
import json
import tempfile
import unittest
from pathlib import Path

from generate_dense_channel_report import build_report_data, render_dashboard


class DenseChannelReportTest(unittest.TestCase):
    def test_dashboard_describes_40x40_block_edge_goal_sets(self):
        page = render_dashboard(
            {
                "records": [
                    {
                        "block_size": 9,
                        "success": False,
                        "actual_density": 76 / 81,
                        "instance": "dense_channel_bedge_case",
                        "density_level": "76/81",
                        "start_edge": 0,
                        "start_inner": 48,
                        "start_deep": 20,
                        "goal_edge": 48,
                        "goal_inner": 0,
                        "first_ms": None,
                        "return_ms": None,
                        "first_makespan": None,
                        "final_makespan": None,
                        "weighted_soc": None,
                        "exit_reason": "",
                        "animation": None,
                        "preview": "preview.html",
                        "yaml": "case.yaml",
                        "certificate": "case.plan",
                        "target_profile": "interior_to_block_edge_set",
                        "robots": 32,
                        "targets": 48,
                        "goals_per_target": 32,
                    }
                ],
                "summary": {
                    "total": 1,
                    "solved": 0,
                    "inner_starts": 48,
                    "deep_starts": 20,
                    "edge_goals": 48,
                    "inner_goals": 0,
                    "target_slots": 48,
                    "min_density": 76 / 81,
                    "max_density": 76 / 81,
                    "min_goals_per_target": 32,
                    "max_goals_per_target": 32,
                    "wall_time_sec": 0,
                    "timeout_sec": 10,
                    "jobs": 1,
                },
                "timing": {
                    "provenance": {},
                    "suite": {
                        "name": "dense_channel_block_edge_40x40_v1_9"
                    },
                },
            }
        )
        self.assertIn("40×40", page)
        self.assertIn("最终 goal 可选择其起始 block 的任意边缘", page)
        self.assertIn("只约束终态", page)
        self.assertIn("其他合法 storage 临时落箱", page)
        self.assertIn("32 robots", page)
        self.assertIn("48 targets", page)
        self.assertIn("32 edge goals/target", page)

    def test_dashboard_describes_interior_to_edge_profile(self):
        page = render_dashboard(
            {
                "records": [
                    {
                        "block_size": 3,
                        "success": False,
                        "actual_density": 8 / 9,
                        "instance": "dense_channel_i2e_case",
                        "density_level": "8/9",
                        "start_edge": 0,
                        "start_inner": 12,
                        "start_deep": 0,
                        "goal_edge": 12,
                        "goal_inner": 0,
                        "first_ms": None,
                        "return_ms": None,
                        "first_makespan": None,
                        "final_makespan": None,
                        "weighted_soc": None,
                        "exit_reason": "",
                        "animation": None,
                        "preview": "preview.html",
                        "yaml": "case.yaml",
                        "certificate": "case.plan",
                        "target_profile": "interior_to_edge",
                    }
                ],
                "summary": {
                    "total": 1,
                    "solved": 0,
                    "inner_starts": 12,
                    "deep_starts": 0,
                    "edge_goals": 12,
                    "inner_goals": 0,
                    "target_slots": 12,
                    "min_density": 8 / 9,
                    "max_density": 8 / 9,
                    "wall_time_sec": 0,
                    "timeout_sec": 10,
                    "jobs": 1,
                },
                "timing": {
                    "provenance": {},
                    "suite": {"name": "dense_channel_i2e_v1_8"},
                },
            }
        )
        self.assertIn("内部货箱到边缘目标", page)
        self.assertIn("边缘 goals", page)
        self.assertNotIn("高密度与内层目标", page)

    def test_dashboard_joins_depth_metadata_and_search_milestones(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            rows_path = root / "rows.csv"
            with rows_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=[
                        "instance",
                        "family",
                        "success",
                        "status",
                        "executed_makespan",
                        "first_solution_makespan",
                        "weighted_soc",
                        "first_solution_ms",
                        "deliverable_ms",
                        "runtime_sec",
                        "improvement_improvements",
                        "improvement_exit_reason",
                        "plan_sha256",
                    ],
                )
                writer.writeheader()
                writer.writerow(
                    {
                        "instance": "dense_a",
                        "family": "dense_channel",
                        "success": 1,
                        "status": "ok",
                        "executed_makespan": 42,
                        "first_solution_makespan": 55,
                        "weighted_soc": 300,
                        "first_solution_ms": 190,
                        "deliverable_ms": 8670,
                        "runtime_sec": 8.69,
                        "improvement_improvements": 1,
                        "improvement_exit_reason": "STRICT_IMPROVEMENT",
                        "plan_sha256": "a" * 64,
                    }
                )

            manifest_path = root / "manifest.json"
            manifest_path.write_text(
                json.dumps(
                    [
                        {
                            "id": "dense_a",
                            "block_size": 4,
                            "density_level": "15/16",
                            "actual_density": 15 / 16,
                            "start_edge": 6,
                            "start_inner": 6,
                            "start_deep": 0,
                            "goal_edge": 0,
                            "goal_inner": 12,
                            "yaml": "instances/dense_a.yaml",
                            "certificate": "certificates/dense_a.plan",
                            "preview": "cases/dense_a.html",
                        }
                    ]
                ),
                encoding="utf-8",
            )
            timing_path = root / "timing.json"
            timing_path.write_text(
                json.dumps(
                    {
                        "wall_time_sec": 8.8,
                        "jobs": 8,
                        "timeout_per_run_sec": 10,
                        "provenance": {
                            "git_commit": "abc123",
                            "binary_sha256": "b" * 64,
                        },
                        "suite": {
                            "name": "dense_channel_v2_8",
                            "definition_sha256": "c" * 64,
                        },
                    }
                ),
                encoding="utf-8",
            )

            data = build_report_data(
                rows_path, timing_path, manifest_path
            )
            self.assertEqual(data["summary"]["solved"], 1)
            self.assertEqual(data["summary"]["inner_starts"], 6)
            self.assertEqual(data["summary"]["inner_goals"], 12)

            page = render_dashboard(data)
            self.assertIn("1 / 1", page)
            self.assertIn("93.8%", page)
            self.assertIn("edge 6 · inner 6", page)
            self.assertIn("首解 0.190 s", page)
            self.assertIn("最终返回 8.670 s", page)
            self.assertIn("首解 makespan 55 → 最终 42", page)
            self.assertIn("cases/dense_a.html", page)
            self.assertIn(
                "../dense_channel_suite_v2_20260906/"
                "cases/dense_a.html",
                page,
            )


if __name__ == "__main__":
    unittest.main()
