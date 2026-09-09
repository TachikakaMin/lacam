import csv
import json
import tempfile
import unittest
from pathlib import Path

from generate_full_benchmark_dashboard import build_dashboard_data
from generate_full_comparison_dashboard import _timing_payload


class FullDashboardMethodTimingTest(unittest.TestCase):
    def test_full_dashboard_accepts_single_carrier_brd_timing_method(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            rows_path = root / "rows.csv"
            timing_path = root / "timing.json"
            manifest_path = root / "manifest.json"
            with rows_path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(
                    handle,
                    fieldnames=[
                        "instance",
                        "family",
                        "method",
                        "success",
                        "executed_makespan",
                        "weighted_soc",
                        "runtime_sec",
                        "status",
                        "plan_sha256",
                    ],
                )
                writer.writeheader()
                writer.writerow(
                    {
                        "instance": "case-a",
                        "family": "synthetic",
                        "method": "carrier_brd",
                        "success": 1,
                        "executed_makespan": 4,
                        "weighted_soc": 7,
                        "runtime_sec": 0.2,
                        "status": "ok",
                        "plan_sha256": "a" * 64,
                    }
                )
            timing_path.write_text(
                json.dumps(
                    {
                        "wall_time_sec": 0.3,
                        "jobs": 1,
                        "timeout_per_run_sec": 10,
                        "methods": {
                            "carrier_brd": {
                                "solver_time_sum_sec": 0.2
                            }
                        },
                        "provenance": {"binary_sha256": "b" * 64},
                        "suite": {"definition_sha256": "c" * 64},
                    }
                ),
                encoding="utf-8",
            )
            manifest_path.write_text("[]", encoding="utf-8")

            data = build_dashboard_data(
                rows_path, timing_path, manifest_path
            )

        self.assertEqual(data["timing"]["method"], "carrier_brd")
        self.assertEqual(data["timing"]["solver_time_sum_sec"], 0.2)

    def test_comparison_accepts_single_carrier_brd_timing_method(self):
        payload = _timing_payload(
            {
                "wall_time_sec": 0.3,
                "methods": {
                    "carrier_brd": {"solver_time_sum_sec": 0.2}
                },
                "provenance": {"binary_sha256": "b" * 64},
                "suite": {"definition_sha256": "c" * 64},
            }
        )

        self.assertEqual(payload["method"], "carrier_brd")
        self.assertEqual(payload["solver_time_sum_sec"], 0.2)


if __name__ == "__main__":
    unittest.main()
