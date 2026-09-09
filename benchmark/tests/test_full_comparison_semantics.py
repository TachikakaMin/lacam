import csv
import json
import tempfile
import unittest
from pathlib import Path

from generate_full_comparison_dashboard import generate_comparison_dashboard


class FullComparisonSemanticsTest(unittest.TestCase):
    def test_regressions_are_not_styled_as_positive_summary_metrics(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            baseline_rows = root / "baseline.csv"
            current_rows = root / "current.csv"
            baseline_timing = root / "baseline-timing.json"
            current_timing = root / "current-timing.json"
            manifest = root / "manifest.json"
            output = root / "site"
            fields = [
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
            with baseline_rows.open(
                "w", newline="", encoding="utf-8"
            ) as handle:
                writer = csv.DictWriter(handle, fieldnames=fields)
                writer.writeheader()
                writer.writerow(
                    {
                        "instance": "case-a",
                        "family": "synthetic",
                        "method": "carrier",
                        "success": 1,
                        "executed_makespan": 10,
                        "weighted_soc": 20,
                        "plan_sha256": "a" * 64,
                        "runtime_sec": 1,
                        "status": "ok",
                    }
                )
            with current_rows.open(
                "w", newline="", encoding="utf-8"
            ) as handle:
                writer = csv.DictWriter(handle, fieldnames=fields)
                writer.writeheader()
                writer.writerow(
                    {
                        "instance": "case-a",
                        "family": "synthetic",
                        "method": "carrier",
                        "success": 1,
                        "executed_makespan": 12,
                        "weighted_soc": 24,
                        "plan_sha256": "b" * 64,
                        "runtime_sec": 2,
                        "status": "ok",
                    }
                )
            for path, wall, solver, digest in (
                (baseline_timing, 1, 1, "a" * 64),
                (current_timing, 2, 2, "b" * 64),
            ):
                path.write_text(
                    json.dumps(
                        {
                            "wall_time_sec": wall,
                            "methods": {
                                "carrier": {
                                    "solver_time_sum_sec": solver
                                }
                            },
                            "provenance": {
                                "binary_sha256": digest
                            },
                            "suite": {
                                "definition_sha256": "s" * 64
                            },
                        }
                    ),
                    encoding="utf-8",
                )
            manifest.write_text("[]", encoding="utf-8")

            generate_comparison_dashboard(
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

        self.assertIn(
            '<small>严格 (T,W) B / E / W</small>'
            '<b class="bad">0 / 0 / 1</b>',
            page,
        )
        self.assertIn(
            '<small>Makespan T B / E / W</small><b>0 / 0 / 1</b>'
            '\n    <span class="bad">几何比 1.200000</span>',
            page,
        )
        self.assertNotIn(
            '<span class="good">几何比 1.200000</span>',
            page,
        )


if __name__ == "__main__":
    unittest.main()
