#!/usr/bin/env python3
"""Unit tests for the symbotic requested-grid runner."""

from __future__ import annotations

import importlib.util
import csv
import sys
import tempfile
import types
import unittest
from pathlib import Path


def load_runner_module():
    module_path = Path(__file__).with_name("run_symbotic_requested_grid.py")
    spec = importlib.util.spec_from_file_location(
        "run_symbotic_requested_grid", module_path
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class RunSymboticRequestedGridTest(unittest.TestCase):
    def setUp(self) -> None:
        self.runner = load_runner_module()

    def test_parse_seed_values_prefers_multi_seed_argument(self) -> None:
        args = types.SimpleNamespace(seed=99, seeds="0,1,2")

        self.assertEqual(self.runner.parse_seed_values(args), [0, 1, 2])

    def test_build_cases_expands_seed_dimension_into_labels(self) -> None:
        args = types.SimpleNamespace(
            maps="symbotic_star",
            ks="2",
            slots="5",
            agent_counts="100",
            dists="50_50",
            durations="4",
            cost_modes="0",
            seed=99,
            seeds="10,11",
        )

        cases = self.runner.build_cases(args)

        self.assertEqual([case.seed for case in cases], [10, 11])
        self.assertEqual(
            [case.label for case in cases],
            [
                "map-symbotic_star__k-2__slot-5__agents-100__dist-50_50"
                "__dur-4__cost-0__seed-10",
                "map-symbotic_star__k-2__slot-5__agents-100__dist-50_50"
                "__dur-4__cost-0__seed-11",
            ],
        )

    def test_cli_accepts_grader_seed_list_argument(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            out_dir = tmp_path / "out"
            fake_binary = tmp_path / "fake_benchmark.py"
            fake_binary.write_text(
                """#!/usr/bin/env python3
import csv
import sys

with open(sys.argv[5], "w", newline="") as handle:
    writer = csv.DictWriter(handle, fieldnames=["seed", "valid", "throughput"])
    writer.writeheader()
    writer.writerow({"seed": sys.argv[4], "valid": "1", "throughput": sys.argv[4]})
""",
                encoding="utf-8",
            )
            fake_binary.chmod(0o755)

            argv = [
                "run_symbotic_requested_grid.py",
                "--binary",
                str(fake_binary),
                "--out-dir",
                str(out_dir),
                "--maps",
                "symbotic_star",
                "--ks",
                "2",
                "--slots",
                "-1",
                "--agent-counts",
                "100",
                "--dists",
                "50_50",
                "--durations",
                "4",
                "--cost-modes",
                "-1",
                "--horizon",
                "400",
                "--seeds",
                "10,11",
                "--workers",
                "1",
                "--force",
            ]
            previous_argv = sys.argv
            try:
                sys.argv = argv
                self.assertEqual(self.runner.main(), 0)
            finally:
                sys.argv = previous_argv

            with (out_dir / "all_results.csv").open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))

            self.assertEqual([row["seed_requested"] for row in rows], ["10", "11"])


if __name__ == "__main__":
    unittest.main()
