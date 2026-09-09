import csv
import json
import tempfile
import unittest
from pathlib import Path

from generate_full_comparison_dashboard import generate_comparison_dashboard


class FullComparisonPortabilityTest(unittest.TestCase):
    def _generate_site(self, root):
        source = root / "private-results"
        source.mkdir()
        baseline_rows = source / "baseline.csv"
        current_rows = source / "current.csv"
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

        timing = source / "timing.json"
        timing.write_text(
            json.dumps(
                {
                    "wall_time_sec": 1,
                    "methods": {"carrier": {"solver_time_sum_sec": 1}},
                    "provenance": {"binary_sha256": "c" * 64},
                    "suite": {"definition_sha256": "s" * 64},
                }
            ),
            encoding="utf-8",
        )
        manifest = source / "manifest.json"
        manifest.write_text("[]", encoding="utf-8")
        out = root / "published-site"

        generate_comparison_dashboard(
            baseline_rows_path=baseline_rows,
            baseline_timing_path=timing,
            current_rows_path=current_rows,
            current_timing_path=timing,
            manifest_path=manifest,
            out_dir=out,
            baseline_case_prefix="../baseline/cases",
            current_case_prefix="../current/cases",
        )
        return out, baseline_rows, current_rows

    def test_rows_csv_is_published_inside_the_static_site(self):
        with tempfile.TemporaryDirectory() as tmp:
            out, baseline_rows, current_rows = self._generate_site(
                Path(tmp)
            )
            page = (out / "index.html").read_text(encoding="utf-8")
            published_baseline = out / "baseline_rows.csv"
            published_current = out / "current_rows.csv"

            self.assertTrue(published_baseline.is_file())
            self.assertTrue(published_current.is_file())
            self.assertEqual(
                published_baseline.read_bytes(), baseline_rows.read_bytes()
            )
            self.assertEqual(
                published_current.read_bytes(), current_rows.read_bytes()
            )
            self.assertIn('href="baseline_rows.csv"', page)
            self.assertIn('href="current_rows.csv"', page)
            self.assertNotIn("private-results/baseline.csv", page)
            self.assertNotIn("private-results/current.csv", page)

    def test_search_input_fits_a_320px_viewport(self):
        with tempfile.TemporaryDirectory() as tmp:
            out, _, _ = self._generate_site(Path(tmp))
            page = (out / "index.html").read_text(encoding="utf-8")

            self.assertIn("@media(max-width:600px)", page)
            self.assertIn(
                "input { min-width:0; width:100%; max-width:100%; }",
                page,
            )


if __name__ == "__main__":
    unittest.main()
