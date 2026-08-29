"""PROTECTED: Proposition-2 same-instance separation (debug.md P2-10).

On the fixed zero-empty-cell cycle fixture:
  - Carrier-LaCAM (C++ binary) SOLVES it (convoy rotation);
  - B4's sequential pebble-style executor requires an empty cell and must
    fail HONESTLY (raise B4Failure), matching Theorem 1's scope;
  - the C++ suite (test_dd_g1.dd_prop2_*) proves every move-transition on
    this instance requires following, so no-following models cannot move.
"""

import subprocess
import unittest
from pathlib import Path

from ddbench.b4_baseline import B4Failure, solve_b4
from ddbench.instance import load_instance

BENCH = Path(__file__).resolve().parent.parent
REPO = BENCH.parent
BIN = REPO / "build/dd_benchmark"
FIXTURE = REPO / "tests/fixtures/prop2_cycle_2x2.yaml"


class TestProp2Separation(unittest.TestCase):
    def test_b4_fails_honestly_on_zero_empty_cycle(self):
        ins = load_instance(FIXTURE)
        with self.assertRaises(B4Failure):
            solve_b4(ins)

    @unittest.skipUnless(BIN.exists(), "dd_benchmark not built")
    def test_carrier_solves_same_instance(self):
        plan_out = BENCH / "results_probe/prop2_cycle.plan"
        plan_out.parent.mkdir(exist_ok=True)
        p = subprocess.run(
            [str(BIN), str(FIXTURE), "5", str(plan_out), "0"],
            capture_output=True, text=True, timeout=30,
        )
        metrics = dict(
            line.split("=", 1) for line in p.stdout.splitlines() if "=" in line
        )
        self.assertEqual(metrics.get("solved"), "1", p.stdout)


if __name__ == "__main__":
    unittest.main()
