import csv
import inspect
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from generate_v5_final_report import generate_report


class TestV5FinalReport(unittest.TestCase):
    def _write_rows(self, path, records):
        fields = [
            "instance",
            "family",
            "success",
            "status",
            "executed_makespan",
            "weighted_soc",
            "plan_sha256",
            "runtime_sec",
        ]
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writer.writerows(records)

    def test_report_is_data_driven_and_keeps_negative_evidence(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            full_rows = root / "full.csv"
            quick_rows = root / "quick.csv"
            baseline_rows = root / "baseline.csv"
            historical_rows = root / "historical.csv"
            e1_rows = root / "e1.csv"
            e2_rows = root / "e2.csv"
            e4_rows = root / "e4.csv"
            timing = root / "timing.json"
            historical_timing = root / "historical-timing.json"
            manifest = root / "manifest.json"
            samples = root / "samples.json"
            out = root / "site"

            quick = [
                {
                    "instance": "quick-ok",
                    "family": "quick",
                    "success": 1,
                    "status": "ok",
                    "executed_makespan": 10,
                    "weighted_soc": 20,
                    "plan_sha256": "a" * 64,
                    "runtime_sec": 1.0,
                },
                {
                    "instance": "quick-timeout",
                    "family": "quick",
                    "success": 0,
                    "status": "timeout",
                    "executed_makespan": "",
                    "weighted_soc": "",
                    "plan_sha256": "",
                    "runtime_sec": 10.0,
                },
            ]
            full = quick + [
                {
                    "instance": "factor-ok",
                    "family": "factorial",
                    "success": 1,
                    "status": "ok",
                    "executed_makespan": 5,
                    "weighted_soc": 8,
                    "plan_sha256": "b" * 64,
                    "runtime_sec": 0.5,
                }
            ]
            baseline = [
                dict(quick[0], executed_makespan=12, weighted_soc=18),
                quick[1],
            ]
            historical = [
                dict(quick[0], executed_makespan=12, weighted_soc=18),
                quick[1],
                dict(full[2], executed_makespan=7, weighted_soc=7),
            ]
            self._write_rows(full_rows, full)
            self._write_rows(quick_rows, quick)
            self._write_rows(baseline_rows, baseline)
            self._write_rows(historical_rows, historical)
            self._write_rows(
                e1_rows,
                [dict(quick[0], executed_makespan=9, weighted_soc=19)],
            )
            self._write_rows(e2_rows, [quick[0]])
            self._write_rows(
                e4_rows,
                [dict(quick[0], executed_makespan=11, weighted_soc=22)],
            )
            timing.write_text(
                json.dumps(
                    {
                        "wall_time_sec": 12.3,
                        "jobs": 2,
                        "timeout_per_run_sec": 10,
                        "methods": {
                            "carrier": {
                                "solver_time_sum_sec": 21.5
                            }
                        },
                        "suite": {
                            "definition_sha256": "c" * 64
                        },
                        "provenance": {
                            "binary_sha256": "d" * 64,
                            "execution_snapshot": "sealed_linux_memfd",
                        },
                    }
                ),
                encoding="utf-8",
            )
            historical_timing.write_text(
                json.dumps({"wall_time_sec": 4.2}),
                encoding="utf-8",
            )
            manifest.write_text(
                json.dumps(
                    [
                        {
                            "id": "factor-ok",
                            "map_family": "g1",
                            "density_level": "low",
                            "agent_level": "baseline",
                            "task_profile": "local",
                            "goal_mode": "singleton",
                        }
                    ]
                ),
                encoding="utf-8",
            )
            samples.write_text(
                json.dumps(
                    [
                        {
                            "id": (
                                "warehouse_cert_h12w20_b3_a1_"
                                "s7of9_r6_t6_remote_seed0"
                            ),
                            "planner": {
                                "binary_sha256": "d" * 64,
                                "metrics": {
                                    "executed_makespan": 31,
                                    "weighted_soc": 93,
                                },
                            },
                        }
                    ]
                ),
                encoding="utf-8",
            )

            signature = inspect.signature(generate_report)
            self.assertIs(
                signature.parameters["cpp_tests"].default,
                inspect.Parameter.empty,
            )
            self.assertIs(
                signature.parameters["python_tests"].default,
                inspect.Parameter.empty,
            )

            cli = subprocess.run(
                [
                    sys.executable,
                    str(
                        Path(__file__).resolve().parent.parent
                        / "generate_v5_final_report.py"
                    ),
                    "--full-rows",
                    str(full_rows),
                    "--full-timing",
                    str(timing),
                    "--manifest",
                    str(manifest),
                    "--release-baseline-rows",
                    str(baseline_rows),
                    "--quick-current-rows",
                    str(quick_rows),
                    "--historical-full-rows",
                    str(historical_rows),
                    "--historical-full-timing",
                    str(historical_timing),
                    "--ablation-control-rows",
                    str(quick_rows),
                    "--ablation-e1-rows",
                    str(e1_rows),
                    "--ablation-e2-rows",
                    str(e2_rows),
                    "--ablation-e4-rows",
                    str(e4_rows),
                    "--samples",
                    str(samples),
                    "--out-dir",
                    str(root / "cli-site"),
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(cli.returncode, 2)
            self.assertIn("--cpp-tests", cli.stderr)
            self.assertIn("--python-tests", cli.stderr)

            data = generate_report(
                full_rows_path=full_rows,
                full_timing_path=timing,
                manifest_path=manifest,
                release_baseline_rows_path=baseline_rows,
                quick_current_rows_path=quick_rows,
                historical_full_rows_path=historical_rows,
                historical_full_timing_path=historical_timing,
                ablation_rows_paths={
                    "control": quick_rows,
                    "E1 off": e1_rows,
                    "E2 off": e2_rows,
                    "E4 off": e4_rows,
                },
                samples_path=samples,
                out_dir=out,
                cpp_tests=4,
                python_tests=5,
            )
            page = (out / "index.html").read_text(encoding="utf-8")
            summary = json.loads(
                (out / "summary.json").read_text(encoding="utf-8")
            )

        self.assertEqual(data["full"]["total"], 3)
        self.assertEqual(data["full"]["solved"], 2)
        self.assertEqual(summary["factorial"]["solved"], 1)
        self.assertIn("2/3", page)
        self.assertIn("1/1", page)
        self.assertIn("Testcase C", page)
        self.assertIn("单一路径", page)
        self.assertIn("fail-closed", page)
        self.assertIn("E1 off", page)
        self.assertIn("负结果", page)
        self.assertIn("4+5", page)
        self.assertNotIn("285+154", page)
        self.assertIn(
            "warehouse_case_proposal/planner_runs/"
            "warehouse_cert_h12w20_b3_a1_s7of9_r6_t6_remote_seed0.html",
            page,
        )
        self.assertNotIn("warehouse_case_proposal/animations/", page)
        self.assertIn(
            'href="../../full_review_approval_v5.json"',
            page,
        )
        self.assertNotIn("479/509", page)
        self.assertNotIn("432/432", page)
        self.assertNotIn(
            "854df1e692316017cbc26ab462ab3b22735d61caa08ff0645e28ad0cab4a574c",
            page,
        )


if __name__ == "__main__":
    unittest.main()
