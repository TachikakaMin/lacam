import csv
import json
import tempfile
import unittest
from pathlib import Path

from generate_full_benchmark_dashboard import generate_dashboard
from generate_full_comparison_dashboard import (
    generate_comparison_dashboard,
)


class Full518ScopeGroupingTest(unittest.TestCase):
    FIELDS = [
        "instance",
        "family",
        "method",
        "success",
        "executed_makespan",
        "weighted_soc",
        "plan_sha256",
        "runtime_sec",
        "status",
    ]

    @staticmethod
    def _row(name, family, success, makespan="", soc=""):
        return {
            "instance": name,
            "family": family,
            "method": "carrier",
            "success": int(success),
            "executed_makespan": makespan,
            "weighted_soc": soc,
            "plan_sha256": "a" * 64 if success else "",
            "runtime_sec": 1,
            "status": "ok" if success else "timeout",
        }

    def _write_rows(self, path, rows):
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=self.FIELDS)
            writer.writeheader()
            writer.writerows(rows)

    @staticmethod
    def _write_timing(path, digest):
        path.write_text(
            json.dumps(
                {
                    "wall_time_sec": 3,
                    "jobs": 1,
                    "timeout_per_run_sec": 10,
                    "methods": {
                        "carrier": {"solver_time_sum_sec": 3}
                    },
                    "provenance": {"binary_sha256": digest * 64},
                    "suite": {"definition_sha256": "s" * 64},
                }
            ),
            encoding="utf-8",
        )

    @staticmethod
    def _write_manifest(path):
        path.write_text(
            json.dumps(
                [
                    {
                        "id": "factorial-case",
                        "map_family": "g1",
                        "height": 8,
                        "width": 8,
                        "block_size": 3,
                        "density_level": "low",
                        "agent_level": "baseline",
                        "robots": 2,
                        "targets": 4,
                        "task_profile": "local",
                        "goal_mode": "singleton",
                        "goal_pool_size": 0,
                        "yaml": "instances/factorial-case.yaml",
                    }
                ]
            ),
            encoding="utf-8",
        )

    def test_full_dashboard_keeps_dense_cases_out_of_quick_77(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            rows = root / "rows.csv"
            timing = root / "timing.json"
            manifest = root / "manifest.json"
            output = root / "site"
            self._write_rows(
                rows,
                [
                    self._row("quick-case", "g4x10", True, 4, 8),
                    self._row(
                        "factorial-case",
                        "warehouse_random_factorial",
                        True,
                        5,
                        9,
                    ),
                    self._row(
                        "dense-case",
                        "dense_channel_block_edge_40x40",
                        False,
                    ),
                ],
            )
            self._write_timing(timing, "a")
            self._write_manifest(manifest)

            data = generate_dashboard(
                rows, timing, manifest, output
            )
            page = (output / "index.html").read_text(encoding="utf-8")

        self.assertEqual(data["quick"]["total"], 1)
        self.assertEqual(data["factorial"]["total"], 1)
        self.assertEqual(data["dense"]["total"], 1)
        self.assertEqual(
            data["failure_groups"],
            [
                {
                    "key": (
                        "dense_channel_block_edge_40x40 · timeout"
                    ),
                    "family": "dense_channel_block_edge_40x40",
                    "status": "timeout",
                    "count": 1,
                }
            ],
        )
        self.assertIn(
            "quick 1 cases + factorial 1 cases + dense 1 cases",
            page,
        )
        self.assertIn("dense 40×40", page)

    def test_comparison_uses_a_separate_dense_scope(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            baseline_rows = root / "baseline.csv"
            current_rows = root / "current.csv"
            baseline_timing = root / "baseline-timing.json"
            current_timing = root / "current-timing.json"
            manifest = root / "manifest.json"
            output = root / "site"
            baseline = [
                self._row("quick-case", "g4x10", True, 4, 8),
                self._row(
                    "factorial-case",
                    "warehouse_random_factorial",
                    True,
                    5,
                    9,
                ),
                self._row(
                    "dense-case",
                    "dense_channel_block_edge_40x40",
                    False,
                ),
            ]
            current = [
                self._row("quick-case", "g4x10", True, 4, 8),
                self._row(
                    "factorial-case",
                    "warehouse_random_factorial",
                    True,
                    5,
                    9,
                ),
                self._row(
                    "dense-case",
                    "dense_channel_block_edge_40x40",
                    True,
                    6,
                    10,
                ),
            ]
            self._write_rows(baseline_rows, baseline)
            self._write_rows(current_rows, current)
            self._write_timing(baseline_timing, "a")
            self._write_timing(current_timing, "b")
            self._write_manifest(manifest)

            data = generate_comparison_dashboard(
                baseline_rows_path=baseline_rows,
                baseline_timing_path=baseline_timing,
                current_rows_path=current_rows,
                current_timing_path=current_timing,
                manifest_path=manifest,
                out_dir=output,
                baseline_case_prefix="../baseline/cases",
                current_case_prefix="../current/cases",
            )
            page = (output / "index.html").read_text(encoding="utf-8")

        self.assertEqual(data["quick"]["total"], 1)
        self.assertEqual(data["factorial"]["total"], 1)
        self.assertEqual(data["dense"]["total"], 1)
        scopes = {
            record["instance"]: record["scope"]
            for record in data["cases"]
        }
        self.assertEqual(scopes["quick-case"], "quick")
        self.assertEqual(scopes["factorial-case"], "factorial")
        self.assertEqual(scopes["dense-case"], "dense")
        self.assertIn(
            '<option value="dense">dense 40×40</option>',
            page,
        )
        self.assertIn(
            "<td>dense</td><td>1</td><td>0/1 → 1/1</td>",
            page,
        )


if __name__ == "__main__":
    unittest.main()
