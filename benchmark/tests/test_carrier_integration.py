"""PROTECTED integration tests for the Carrier-LaCAM C++ solver.

Written BEFORE implementation (TDD RED).  The C++ binary build/dd_benchmark
must:
  usage: dd_benchmark INSTANCE.yaml TIME_LIMIT_SEC PLAN_OUT [SEED]
  - print 'solved=0|1' plus metrics on stdout
  - write PLAN_OUT with one line per timestep; per robot (YAML robots order)
    actions separated by ';':  'w' | 'm R C' | 'l' | 'd'
The plan is replayed through the AUTHORITATIVE Python two-deck validator
(ddbench.validator) and must reach the goal.
"""

import subprocess
import unittest
from pathlib import Path

from ddbench.instance import load_instance
from ddbench.validator import is_goal, initial_state, apply_joint_action

BENCH = Path(__file__).resolve().parent.parent
REPO = BENCH.parent
BIN = REPO / "build/dd_benchmark"


def parse_plan(path):
    plan = []
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        joint = []
        for tok in line.split(";"):
            parts = tok.split()
            if parts[0] == "w":
                joint.append(("wait",))
            elif parts[0] == "m":
                joint.append(("move", (int(parts[1]), int(parts[2]))))
            elif parts[0] == "l":
                joint.append(("lift",))
            elif parts[0] == "d":
                joint.append(("drop",))
            else:
                raise ValueError(f"bad action token: {tok!r}")
        plan.append(joint)
    return plan


def run_solver(instance_path, time_limit, plan_out, seed=0):
    p = subprocess.run(
        [str(BIN), str(instance_path), str(time_limit), str(plan_out),
         str(seed)],
        capture_output=True, text=True, timeout=time_limit + 30,
    )
    metrics = {}
    for line in p.stdout.splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            metrics[k.strip()] = v.strip()
    return p, metrics


def replay(ins, plan):
    s = initial_state(ins)
    for t, joint in enumerate(plan):
        s = apply_joint_action(ins, s, joint)  # raises on illegal
    return s


@unittest.skipUnless(BIN.exists(), "dd_benchmark not built")
class TestCostCrossConsistency(unittest.TestCase):
    """debug.md P0-3: C++ metrics must match the authoritative Python
    plan_cost, INCLUDING the delta * anonymous-shelf-move term (design 2.3:
    cost = a*loaded + b*free + g*liftdrop + d*anon)."""

    def test_weighted_soc_includes_anon_moves(self):
        import yaml as _yaml
        ins_yaml = {
            "name": "cost_anon_case",
            "map": "....\n....\n",
            "robots": [[1, 0]],
            "shelves": [[0, 1], [0, 3]],
            "targets": [{"id": "b0", "start": [0, 1], "goal": [0, 3]}],
            "flags": {},
        }
        base = BENCH / "results_probe"
        base.mkdir(exist_ok=True)
        ins_path = base / "cost_anon_case.yaml"
        ins_path.write_text(_yaml.safe_dump(ins_yaml, sort_keys=False))
        plan_out = base / "cost_anon_case.plan"
        p, metrics = run_solver(ins_path, 5, plan_out)
        self.assertEqual(metrics.get("solved"), "1", p.stdout)

        ins = load_instance(ins_path)
        plan = parse_plan(plan_out)
        from ddbench.validator import plan_cost
        c = plan_cost(ins, plan)  # alpha=beta=gamma=delta=1
        # the blocker at (0,2) must be relocated: anonymous moves happen
        self.assertGreater(c["anon_moves"], 0,
                           "test instance must exercise anonymous moves")
        # C++ must report anon_moves and a consistent weighted_soc
        self.assertIn("anon_moves", metrics,
                      "C++ driver does not report anon_moves (P0-3)")
        self.assertEqual(int(metrics["anon_moves"]), c["anon_moves"])
        self.assertEqual(float(metrics["weighted_soc"]), c["weighted_soc"],
                         "C++ weighted_soc must include delta*anon_moves")
        self.assertEqual(int(metrics["loaded_moves"]), c["loaded_moves"])
        self.assertEqual(int(metrics["free_moves"]), c["free_moves"])
        self.assertEqual(int(metrics["lift_drop"]), c["lift_drop"])


@unittest.skipUnless(BIN.exists(), "dd_benchmark not built")
class TestCarrierLacamIntegration(unittest.TestCase):
    def test_tiny_fixture_solves_and_validates(self):
        ins_path = REPO / "tests/fixtures/dd_tiny.yaml"
        plan_out = BENCH / "results_probe/dd_tiny.plan"
        plan_out.parent.mkdir(exist_ok=True)
        p, metrics = run_solver(ins_path, 5, plan_out)
        self.assertEqual(p.returncode, 0, p.stderr[-400:])
        self.assertEqual(metrics.get("solved"), "1", p.stdout)
        ins = load_instance(ins_path)
        plan = parse_plan(plan_out)
        s = replay(ins, plan)  # every step legal per authoritative validator
        self.assertTrue(is_goal(ins, s))
        self.assertEqual(int(metrics.get("makespan")), len(plan))

    def test_dev_case_small_scramble_10s(self):
        """First fixed dev case must solve within the 10 s budget."""
        ins_path = BENCH / "instances_small/scramble/scramble_h6w6r2s8t2k40_seed1.yaml"
        plan_out = BENCH / "results_probe/dd_dev0.plan"
        plan_out.parent.mkdir(exist_ok=True)
        p, metrics = run_solver(ins_path, 10, plan_out)
        self.assertEqual(p.returncode, 0, p.stderr[-400:])
        self.assertEqual(metrics.get("solved"), "1", p.stdout)
        ins = load_instance(ins_path)
        s = replay(ins, parse_plan(plan_out))
        self.assertTrue(is_goal(ins, s))

    def test_deterministic_with_seed(self):
        ins_path = REPO / "tests/fixtures/dd_tiny.yaml"
        out1 = BENCH / "results_probe/dd_det1.plan"
        out2 = BENCH / "results_probe/dd_det2.plan"
        run_solver(ins_path, 5, out1, seed=7)
        run_solver(ins_path, 5, out2, seed=7)
        self.assertEqual(out1.read_text(), out2.read_text())
