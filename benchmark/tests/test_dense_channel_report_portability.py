"""Dense-channel report paths and counts must come from its inputs."""

import csv
import json
import tempfile
import unittest
from pathlib import Path

from generate_dense_channel_report import build_report_data


class DenseChannelReportPortabilityTest(unittest.TestCase):
    def test_suite_url_and_target_count_are_data_driven(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            rows = root / "rows.csv"
            with rows.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=["instance", "success"],
                )
                writer.writeheader()
                writer.writerow({"instance": "case_a", "success": 0})
            manifest = root / "manifest.json"
            manifest.write_text(
                json.dumps(
                    [
                        {
                            "id": "case_a",
                            "block_size": 3,
                            "density_level": "8/9",
                            "actual_density": 8 / 9,
                            "start_edge": 2,
                            "start_inner": 1,
                            "start_deep": 0,
                            "goal_edge": 1,
                            "goal_inner": 2,
                            "yaml": "instances/case_a.yaml",
                            "certificate": "certificates/case_a.plan",
                            "preview": "cases/case_a.html",
                        }
                    ]
                ),
                encoding="utf-8",
            )
            timing = root / "timing.json"
            timing.write_text(
                json.dumps(
                    {
                        "wall_time_sec": 0,
                        "jobs": 1,
                        "timeout_per_run_sec": 10,
                    }
                ),
                encoding="utf-8",
            )

            data = build_report_data(
                rows,
                timing,
                manifest,
                suite_url="../custom-suite/",
            )
            self.assertEqual(data["summary"]["target_slots"], 3)
            self.assertEqual(
                data["records"][0]["preview"],
                "../custom-suite/cases/case_a.html",
            )


if __name__ == "__main__":
    unittest.main()
