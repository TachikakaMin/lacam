import csv
import json
import tempfile
import unittest
from pathlib import Path

from generate_full_benchmark_dashboard import generate_dashboard


BENCH = Path(__file__).resolve().parent.parent


class TestFullBenchmarkDashboard(unittest.TestCase):
    def test_cards_and_failure_copy_are_derived_from_supplied_data(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            out = root / "site"
            rows = root / "rows.csv"
            timing = root / "timing.json"
            manifest = root / "manifest.json"
            records = [
                {
                    "instance": "factor-a",
                    "family": "factorial",
                    "success": 1,
                    "status": "ok",
                    "executed_makespan": 11,
                    "weighted_soc": 21,
                    "runtime_sec": 0.11,
                    "plan_sha256": "a" * 64,
                },
                {
                    "instance": "factor-b",
                    "family": "factorial",
                    "success": 1,
                    "status": "ok",
                    "executed_makespan": 13,
                    "weighted_soc": 23,
                    "runtime_sec": 0.13,
                    "plan_sha256": "b" * 64,
                },
                {
                    "instance": "quick-ok",
                    "family": "quick_ok",
                    "success": 1,
                    "status": "ok",
                    "executed_makespan": 17,
                    "weighted_soc": 27,
                    "runtime_sec": 0.17,
                    "plan_sha256": "c" * 64,
                },
                {
                    "instance": "quick-timeout",
                    "family": "fam_timeout",
                    "success": 0,
                    "status": "timeout",
                    "executed_makespan": "",
                    "weighted_soc": "",
                    "runtime_sec": 10.0,
                    "plan_sha256": "",
                },
                {
                    "instance": "quick-invalid",
                    "family": "fam_invalid",
                    "success": 0,
                    "status": "invalid_plan",
                    "executed_makespan": "",
                    "weighted_soc": "",
                    "runtime_sec": 0.2,
                    "plan_sha256": "",
                },
            ]
            with rows.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(
                    handle, fieldnames=list(records[0])
                )
                writer.writeheader()
                writer.writerows(records)
            timing.write_text(
                json.dumps(
                    {
                        "wall_time_sec": 12.3,
                        "jobs": 3,
                        "timeout_per_run_sec": 10,
                        "methods": {
                            "carrier": {"solver_time_sum_sec": 10.61}
                        },
                        "suite": {"definition_sha256": "d" * 64},
                        "provenance": {"binary_sha256": "e" * 64},
                    }
                ),
                encoding="utf-8",
            )
            manifest.write_text(
                json.dumps(
                    [
                        {
                            "id": instance,
                            "map_family": "g1",
                            "height": 5,
                            "width": 5,
                            "block_size": 2,
                            "density_level": "low",
                            "agent_level": "baseline",
                            "robots": 2,
                            "targets": 1,
                            "task_profile": "local",
                            "goal_mode": "singleton",
                            "goal_pool_size": 1,
                            "yaml": "instances/{}.yaml".format(instance),
                        }
                        for instance in ("factor-a", "factor-b")
                    ]
                ),
                encoding="utf-8",
            )
            data = generate_dashboard(
                rows_path=rows,
                timing_path=timing,
                manifest_path=manifest,
                out_dir=out,
            )
            page = (out / "index.html").read_text(encoding="utf-8")
            summary = json.loads(
                (out / "summary.json").read_text(encoding="utf-8")
            )

        self.assertEqual(data["overview"]["total"], 5)
        self.assertEqual(data["overview"]["solved"], 3)
        self.assertEqual(data["factorial"]["solved"], 2)
        self.assertEqual(summary["factorial"]["total"], 2)
        self.assertIn("全量 benchmark", page)
        self.assertIn("3/5", page)
        self.assertIn("2/2", page)
        self.assertIn("1/3", page)
        self.assertIn("12.3s", page)
        self.assertIn("solver sum 10.6s", page)
        self.assertIn("fam_timeout · timeout", page)
        self.assertIn("fam_invalid · invalid_plan", page)
        self.assertNotIn("479/509", page)
        self.assertNotIn("432/432", page)
        self.assertNotIn("49.2s", page)
        self.assertNotIn("所有失败都来自既有 BRAP", page)
        self.assertIn("density_level", page)
        self.assertIn("agent_level", page)
        self.assertIn("task_profile", page)
        self.assertIn("goal_mode", page)
        self.assertIn("caseScatter", page)


if __name__ == "__main__":
    unittest.main()
