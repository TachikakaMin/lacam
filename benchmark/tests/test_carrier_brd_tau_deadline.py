"""PROTECTED regression for deadline-aware BRD tau construction."""

import subprocess
import tempfile
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[2]
BIN = REPO / "build" / "dd_benchmark"
CASE = (
    REPO
    / "benchmark"
    / "instances_brap_pool"
    / "g80x80"
    / "brap_h80w80_a128_e1600_B_seed0_pool.yaml"
)


def parse_metrics(stdout):
    metrics = {}
    for line in stdout.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            metrics[key.strip()] = value.strip()
    return metrics


@unittest.skipUnless(BIN.is_file(), "dd_benchmark not built")
class TestCarrierBRDTauDeadline(unittest.TestCase):
    def test_large_goal_pool_obeys_short_shared_budget(self):
        with tempfile.TemporaryDirectory() as tmp:
            proc = subprocess.run(
                [
                    str(BIN),
                    str(CASE),
                    "0.1",
                    str(Path(tmp) / "out.plan"),
                    "0",
                    "brd",
                ],
                capture_output=True,
                text=True,
                timeout=2,
            )

        self.assertEqual(proc.returncode, 0, proc.stderr)
        metrics = parse_metrics(proc.stdout)
        self.assertEqual(metrics.get("solved"), "0")
        self.assertEqual(metrics.get("timed_out"), "1")
        self.assertEqual(
            metrics.get("brd_exit_reason"), "SEARCH_TIMEOUT"
        )
        self.assertLessEqual(float(metrics["runtime_ms"]), 1000.0)


if __name__ == "__main__":
    unittest.main()
