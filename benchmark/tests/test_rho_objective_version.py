"""PROTECTED production rho objective-version export contract."""

import subprocess
import tempfile
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[2]
PLANNER = REPO / "build" / "dd_benchmark"
TINY = REPO / "tests" / "fixtures" / "dd_tiny.yaml"


class TestRhoObjectiveVersion(unittest.TestCase):
    def test_runner_persists_explicit_objective_version(self):
        from run_benchmark import FIELDS

        self.assertIn("rho_objective_version", FIELDS)

    @unittest.skipUnless(PLANNER.is_file(), "dd_benchmark not built")
    def test_binary_exports_exact_v2_objective_name(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = subprocess.run(
                [
                    str(PLANNER),
                    str(TINY),
                    "1",
                    str(Path(tmp) / "plan.txt"),
                    "0",
                ],
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
        self.assertEqual(result.returncode, 0, result.stderr)
        metrics = dict(
            line.split("=", 1)
            for line in result.stdout.splitlines()
            if "=" in line
        )
        self.assertEqual(
            metrics.get("rho_objective_version"),
            "BOTTLENECK_TARGET_FRONTIER_CONTINUITY_V2",
        )


if __name__ == "__main__":
    unittest.main()
