"""PROTECTED CLI regression for suite-provided benchmark defaults."""

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


BENCH = Path(__file__).resolve().parent.parent
REPO = BENCH.parent
BIN = REPO / "build" / "dd_benchmark"


@unittest.skipUnless(BIN.exists(), "dd_benchmark not built")
class TestReleaseBenchmarkCli(unittest.TestCase):
    def test_suite_config_supplies_methods_and_parallelism(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            case = root / "case.yaml"
            case.write_text(
                (REPO / "tests" / "fixtures" / "dd_tiny.yaml").read_text()
            )
            suite = root / "suite.json"
            suite.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "name": "cli_probe",
                        "protocol": {
                            "methods": ["carrier"],
                            "timeout_sec": 10,
                            "jobs": 14,
                            "solver_seed": 0,
                            "objective_weights": [1, 1, 1, 1],
                            "following": "allowed",
                        },
                        "groups": [
                            {
                                "name": "probe",
                                "root": ".",
                                "pattern": "case.yaml",
                                "family": "probe",
                                "expected_cases": 1,
                            }
                        ],
                    }
                )
            )
            out = root / "results"
            proc = subprocess.run(
                [
                    sys.executable,
                    str(BENCH / "run_benchmark.py"),
                    "--suite-config",
                    str(suite),
                    "--out-dir",
                    str(out),
                ],
                capture_output=True,
                text=True,
                timeout=30,
            )
            self.assertEqual(proc.returncode, 0, proc.stderr)
            timing = json.loads((out / "timing.json").read_text())
            self.assertEqual(timing["jobs"], 14)
            self.assertEqual(timing["n_tasks"], 1)
            self.assertEqual(timing["methods"]["carrier"]["total"], 1)


if __name__ == "__main__":
    unittest.main()
