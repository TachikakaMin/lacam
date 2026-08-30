"""Non-unit weight cross-language cost consistency (debug.md round-2 P1-9).

The default-weight consistency is covered by test_carrier_integration; the
audit noted DD_ALPHA..DD_DELTA != 1 had no cross-language coverage.  Same
protocol: the C++ driver solves a fixture that REQUIRES an anonymous-shelf
move (blocker on the goal), reports weighted_soc under (2,1,5,3); the plan
is replayed through the authoritative Python plan_cost with the same
weights and the numbers must match exactly.
"""

import os
import subprocess
import tempfile
import unittest
from pathlib import Path

from ddbench.instance import Instance, Target, parse_map_str, save_instance
from ddbench.validator import plan_cost, validate_plan

BENCH = Path(__file__).resolve().parent.parent
BIN = BENCH.parent / "build/dd_benchmark"

WEIGHTS = {"DD_ALPHA": "2", "DD_BETA": "1", "DD_GAMMA": "5", "DD_DELTA": "3"}


def parse_plan(path):
    plan = []
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        joint = []
        for tok in line.split(";"):
            parts = tok.split()
            joint.append(
                ("wait",) if parts[0] == "w"
                else ("move", (int(parts[1]), int(parts[2])))
                if parts[0] == "m"
                else ("lift",) if parts[0] == "l" else ("drop",)
            )
        plan.append(joint)
    return plan


class TestNonUnitWeights(unittest.TestCase):
    def test_cost_2153_matches(self):
        # anon blocker sits ON the target goal -> any solution moves it
        ins = Instance(
            grid=parse_map_str(".....\n.....\n....."),
            robots=[(0, 0), (2, 4)],
            shelves=[(1, 1), (1, 3)],
            targets=[Target("b0", (1, 1), (1, 3))],
        )
        with tempfile.TemporaryDirectory() as tmp:
            ypath = Path(tmp) / "w.yaml"
            save_instance(ins, ypath)
            plan_out = Path(tmp) / "w.plan"
            env = dict(os.environ, **WEIGHTS)
            p = subprocess.run(
                [str(BIN), str(ypath), "10", str(plan_out), "0"],
                capture_output=True, text=True, timeout=30, env=env,
            )
            self.assertEqual(p.returncode, 0, p.stderr)
            metrics = dict(
                line.split("=", 1) for line in p.stdout.splitlines()
                if "=" in line
            )
            self.assertEqual(metrics["solved"], "1")
            plan = parse_plan(plan_out)
            ok, errs, _ = validate_plan(ins, plan)
            self.assertTrue(ok, errs)
            c = plan_cost(ins, plan, alpha=2, beta=1, gamma=5, delta=3)
            self.assertGreater(c["anon_moves"], 0,
                               "fixture must force an anonymous move")
            self.assertEqual(float(metrics["weighted_soc"]),
                             c["weighted_soc"],
                             "C++ (2,1,5,3) weighted_soc != Python")
            # component counters must agree too
            for k in ("loaded_moves", "free_moves", "lift_drop",
                      "anon_moves"):
                self.assertEqual(float(metrics[k]), c[k], k)


if __name__ == "__main__":
    unittest.main()
