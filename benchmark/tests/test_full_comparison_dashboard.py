import csv
import json
import tempfile
import unittest
from pathlib import Path

from generate_full_comparison_dashboard import generate_comparison_dashboard


class TestFullComparisonDashboard(unittest.TestCase):
    def _write_rows(self, path, records):
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
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writer.writerows(records)

    def _row(self, name, success, makespan="", work="", runtime=1):
        return {
            "instance": name,
            "family": "synthetic",
            "method": "carrier",
            "success": int(success),
            "executed_makespan": makespan,
            "weighted_soc": work,
            "plan_sha256": name * 8 if success else "",
            "runtime_sec": runtime,
            "status": "ok" if success else "timeout",
        }

    def test_all_rows_and_comparisons_are_derived_from_inputs(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            base_rows = root / "base.csv"
            new_rows = root / "new.csv"
            base_timing = root / "base-timing.json"
            new_timing = root / "new-timing.json"
            manifest = root / "manifest.json"
            out = root / "site"

            self._write_rows(
                base_rows,
                [
                    self._row("a", True, 10, 20, 1),
                    self._row("b", True, 20, 30, 2),
                    self._row("c", False, runtime=10),
                    self._row("d", True, 8, 8, 1),
                ],
            )
            self._write_rows(
                new_rows,
                [
                    self._row("a", True, 8, 22, 3),
                    self._row("b", True, 20, 30, 4),
                    self._row("c", True, 15, 25, 5),
                    self._row("d", False, runtime=10),
                ],
            )
            base_timing.write_text(
                json.dumps(
                    {
                        "wall_time_sec": 4,
                        "methods": {
                            "carrier": {"solver_time_sum_sec": 14}
                        },
                        "provenance": {"binary_sha256": "a" * 64},
                        "suite": {"definition_sha256": "s" * 64},
                    }
                ),
                encoding="utf-8",
            )
            new_timing.write_text(
                json.dumps(
                    {
                        "wall_time_sec": 12,
                        "methods": {
                            "carrier": {"solver_time_sum_sec": 22}
                        },
                        "provenance": {"binary_sha256": "b" * 64},
                        "suite": {"definition_sha256": "s" * 64},
                    }
                ),
                encoding="utf-8",
            )
            manifest.write_text(
                json.dumps(
                    [
                        {
                            "id": name,
                            "map_family": "g1",
                            "density_level": "low",
                            "agent_level": "baseline",
                            "task_profile": "local",
                            "goal_mode": "singleton",
                        }
                        for name in ("a", "b")
                    ]
                ),
                encoding="utf-8",
            )

            data = generate_comparison_dashboard(
                baseline_rows_path=base_rows,
                baseline_timing_path=base_timing,
                current_rows_path=new_rows,
                current_timing_path=new_timing,
                manifest_path=manifest,
                out_dir=out,
                baseline_case_prefix="../old/cases",
                current_case_prefix="../new/cases",
            )
            page = (out / "index.html").read_text(encoding="utf-8")
            summary = json.loads(
                (out / "summary.json").read_text(encoding="utf-8")
            )

        self.assertEqual(data["overview"]["total"], 4)
        self.assertEqual(data["overview"]["baseline_solved"], 3)
        self.assertEqual(data["overview"]["current_solved"], 3)
        self.assertEqual(data["overview"]["common_solved"], 2)
        self.assertEqual(data["overview"]["gained"], 1)
        self.assertEqual(data["overview"]["lost"], 1)
        self.assertEqual(
            data["overview"]["makespan"]["better_equal_worse"],
            [1, 1, 0],
        )
        self.assertEqual(
            data["overview"]["work"]["better_equal_worse"],
            [0, 1, 1],
        )
        self.assertEqual(
            data["overview"]["lexicographic"]["better_equal_worse"],
            [1, 1, 0],
        )
        self.assertEqual(summary["factorial"]["total"], 2)
        self.assertIn("全部 4 行", page)
        self.assertIn("3/4 → 3/4", page)
        self.assertIn("gained 1 · lost 1", page)
        self.assertIn("1 / 1 / 0", page)
        self.assertIn("caseTable", page)
        self.assertIn("scatterT", page)
        self.assertIn("../old/cases/a.html", page)
        self.assertIn("../new/cases/a.html", page)
        self.assertNotIn("479/509", page)
        self.assertNotIn("348 / 85 / 46", page)

    def test_case_set_mismatch_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            base_rows = root / "base.csv"
            new_rows = root / "new.csv"
            timing = root / "timing.json"
            manifest = root / "manifest.json"
            self._write_rows(base_rows, [self._row("a", True, 1, 1)])
            self._write_rows(new_rows, [self._row("b", True, 1, 1)])
            timing.write_text(
                json.dumps(
                    {
                        "wall_time_sec": 1,
                        "methods": {
                            "carrier": {"solver_time_sum_sec": 1}
                        },
                        "provenance": {"binary_sha256": "a" * 64},
                        "suite": {"definition_sha256": "s" * 64},
                    }
                ),
                encoding="utf-8",
            )
            manifest.write_text("[]", encoding="utf-8")

            with self.assertRaisesRegex(
                ValueError, "case sets differ"
            ):
                generate_comparison_dashboard(
                    baseline_rows_path=base_rows,
                    baseline_timing_path=timing,
                    current_rows_path=new_rows,
                    current_timing_path=timing,
                    manifest_path=manifest,
                    out_dir=root / "site",
                    baseline_case_prefix="../old/cases",
                    current_case_prefix="../new/cases",
                )


if __name__ == "__main__":
    unittest.main()
