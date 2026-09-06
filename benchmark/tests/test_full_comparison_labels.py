import csv
import json
import tempfile
import unittest
from pathlib import Path

from generate_full_comparison_dashboard import generate_comparison_dashboard


class FullComparisonLabelsTest(unittest.TestCase):
    def test_supplied_run_labels_replace_historical_template_copy(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            baseline_rows = root / "baseline.csv"
            current_rows = root / "current.csv"
            baseline_timing = root / "baseline-timing.json"
            current_timing = root / "current-timing.json"
            manifest = root / "manifest.json"
            out = root / "site"
            fields = [
                "instance", "family", "method", "success",
                "executed_makespan", "weighted_soc", "plan_sha256",
                "runtime_sec", "status",
            ]
            for path, makespan, digest in (
                (baseline_rows, 10, "a" * 64),
                (current_rows, 9, "b" * 64),
            ):
                with path.open("w", newline="", encoding="utf-8") as handle:
                    writer = csv.DictWriter(handle, fieldnames=fields)
                    writer.writeheader()
                    writer.writerow(
                        {
                            "instance": "case-a",
                            "family": "synthetic",
                            "method": "carrier",
                            "success": 1,
                            "executed_makespan": makespan,
                            "weighted_soc": 20,
                            "plan_sha256": digest,
                            "runtime_sec": 1,
                            "status": "ok",
                        }
                    )
            for path, digest in (
                (baseline_timing, "a" * 64),
                (current_timing, "b" * 64),
            ):
                path.write_text(
                    json.dumps(
                        {
                            "wall_time_sec": 1,
                            "methods": {
                                "carrier": {"solver_time_sum_sec": 1}
                            },
                            "provenance": {"binary_sha256": digest},
                            "suite": {"definition_sha256": "s" * 64},
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
                out_dir=out,
                baseline_case_prefix="../baseline/cases",
                current_case_prefix="../current/cases",
                baseline_label="审计基线",
                current_label="rho V2",
                report_url="../rho-report/index.html",
                baseline_dashboard_url="../baseline/index.html",
                current_dashboard_url="../current/index.html",
            )
            page = (out / "index.html").read_text(encoding="utf-8")

        self.assertIn("审计基线 vs rho V2", page)
        self.assertIn("状态 审计基线 → rho V2", page)
        self.assertIn(">审计基线</a>", page)
        self.assertIn(">rho V2</a>", page)
        self.assertIn("../rho-report/index.html", page)
        self.assertIn("../baseline/index.html", page)
        self.assertIn("../current/index.html", page)
        self.assertNotIn("pre-v5", page)
        self.assertNotIn("当前 v5", page)
        self.assertNotIn(">old</a>", page)


if __name__ == "__main__":
    unittest.main()
