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

import resource
import shutil
import subprocess
import tempfile
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
        ins_path = REPO / "tests/fixtures/dd_scramble_dev.yaml"
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

    def test_unsolved_run_removes_stale_plan_outputs(self):
        """A failed run must not leave a plan from an earlier invocation."""
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            ins_path = tmp / "unsolved.yaml"
            ins_path.write_text(
                """\
name: stale_plan_unsolved
map: |
  ..
robots:
  - [0, 0]
  - [0, 1]
shelves:
  - [0, 0]
targets:
  - id: b0
    start: [0, 0]
    goal: [0, 1]
flags: {}
"""
            )
            plan_out = tmp / "result.plan"
            stale_outputs = [
                plan_out,
                Path(str(plan_out) + ".tmp"),
                Path(str(plan_out) + ".best_effort"),
                Path(str(plan_out) + ".best_effort.tmp"),
            ]
            for path in stale_outputs:
                path.write_text("d;d\n")

            p, metrics = run_solver(ins_path, 0.05, plan_out)

            self.assertEqual(p.returncode, 0, p.stderr[-400:])
            self.assertEqual(metrics.get("solved"), "0", p.stdout)
            for path in stale_outputs:
                self.assertFalse(path.exists(), f"stale output survived: {path}")

    def test_successful_run_publishes_only_the_final_plan(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            plan_out = tmp / "result.plan"
            p, metrics = run_solver(
                REPO / "tests/fixtures/dd_tiny.yaml", 5, plan_out
            )
            self.assertEqual(p.returncode, 0, p.stderr)
            self.assertEqual(metrics.get("solved"), "1", p.stdout)
            self.assertTrue(plan_out.is_file())
            self.assertEqual(list(tmp.glob("result.plan.tmp*")), [])
            ins = load_instance(REPO / "tests/fixtures/dd_tiny.yaml")
            self.assertTrue(is_goal(ins, replay(ins, parse_plan(plan_out))))

    def test_output_path_aliases_never_delete_the_instance(self):
        source = REPO / "tests/fixtures/dd_tiny.yaml"
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            (tmp / "sub").mkdir()

            aliased_input = tmp / "input.yaml"
            shutil.copyfile(source, aliased_input)
            aliased_output = tmp / "sub" / ".." / "input.yaml"
            p = subprocess.run(
                [
                    str(BIN),
                    str(aliased_input),
                    "1",
                    str(aliased_output),
                    "0",
                ],
                capture_output=True,
                text=True,
                timeout=30,
            )
            self.assertEqual(p.returncode, 2, p.stderr)
            self.assertTrue(aliased_input.is_file())
            self.assertEqual(load_instance(aliased_input).validate_static(), [])

            derived_input = tmp / "result.plan.tmp"
            shutil.copyfile(source, derived_input)
            p = subprocess.run(
                [
                    str(BIN),
                    str(derived_input),
                    "1",
                    str(tmp / "result.plan"),
                    "0",
                ],
                capture_output=True,
                text=True,
                timeout=30,
            )
            self.assertEqual(p.returncode, 2, p.stderr)
            self.assertTrue(derived_input.is_file())
            self.assertEqual(load_instance(derived_input).validate_static(), [])

    def test_output_directory_is_preserved_and_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            plan_out = Path(tmp) / "result.plan"
            plan_out.mkdir()
            p = subprocess.run(
                [
                    str(BIN),
                    str(REPO / "tests/fixtures/dd_tiny.yaml"),
                    "1",
                    str(plan_out),
                    "0",
                ],
                capture_output=True,
                text=True,
                timeout=30,
            )
            self.assertEqual(p.returncode, 2, p.stderr)
            self.assertTrue(plan_out.is_dir())
            self.assertNotIn("solved=1", p.stdout)

    def test_invalid_numeric_arguments_clean_stale_outputs_without_abort(self):
        with tempfile.TemporaryDirectory() as tmp:
            plan_out = Path(tmp) / "result.plan"
            outputs = [
                plan_out,
                Path(str(plan_out) + ".tmp"),
                Path(str(plan_out) + ".best_effort"),
                Path(str(plan_out) + ".best_effort.tmp"),
            ]
            bad_arguments = (("not-a-time", "0"), ("1", "not-a-seed"))
            for time_limit, seed in bad_arguments:
                for path in outputs:
                    path.write_text("stale\n")
                p = subprocess.run(
                    [
                        str(BIN),
                        str(REPO / "tests/fixtures/dd_tiny.yaml"),
                        time_limit,
                        str(plan_out),
                        seed,
                    ],
                    capture_output=True,
                    text=True,
                    timeout=30,
                )
                self.assertEqual(p.returncode, 2, p.stderr)
                self.assertGreaterEqual(p.returncode, 0, "must not die by signal")
                for path in outputs:
                    self.assertFalse(
                        path.exists(), f"stale output survived: {path}"
                    )

    def test_file_size_failure_leaves_no_plan_or_temp_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            plan_out = tmp / "result.plan"

            def deny_file_growth():
                resource.setrlimit(resource.RLIMIT_FSIZE, (0, 0))

            p = subprocess.run(
                [
                    str(BIN),
                    str(REPO / "tests/fixtures/dd_tiny.yaml"),
                    "5",
                    str(plan_out),
                    "0",
                ],
                capture_output=True,
                text=True,
                timeout=30,
                preexec_fn=deny_file_growth,
            )
            metrics = {}
            for line in p.stdout.splitlines():
                if "=" in line:
                    key, value = line.split("=", 1)
                    metrics[key.strip()] = value.strip()
            self.assertEqual(p.returncode, 0, p.stderr)
            self.assertEqual(metrics.get("solved"), "0", p.stdout)
            self.assertFalse(plan_out.exists())
            self.assertEqual(list(tmp.glob("result.plan.tmp*")), [])


@unittest.skipUnless(BIN.exists(), "dd_benchmark not built")
class TestV3DiagnosticsExported(unittest.TestCase):
    # 2026-09-02 R5 (debug.md §10, TDD RED): the v3.0 counters/timings
    # must be observable end-to-end — binary stdout AND the runner's CSV
    # columns — or gate analyses cannot audit price/rewire activity and
    # the guidance budget (design_final §11.6(6)).
    REQUIRED = ("tau_price_repairs", "rewire_guidance_rebuilds",
                "tau_time_ms", "guidance_time_ms")

    def test_binary_emits_v3_diagnostics(self):
        ins_path = REPO / "tests/fixtures/dd_tiny.yaml"
        plan_out = BENCH / "results_probe/dd_tiny_diag.plan"
        plan_out.parent.mkdir(exist_ok=True)
        p, metrics = run_solver(ins_path, 5, plan_out)
        self.assertEqual(p.returncode, 0, p.stderr[-400:])
        for key in self.REQUIRED:
            self.assertIn(key, metrics, f"binary must print {key}=")

    def test_runner_persists_v3_diagnostics(self):
        from run_benchmark import FIELDS
        for key in self.REQUIRED:
            self.assertIn(key, FIELDS, f"rows.csv must persist {key}")


@unittest.skipUnless(BIN.exists(), "dd_benchmark not built")
class TestAuditability(unittest.TestCase):
    # 2026-09-02 R6 (debug.md §10, TDD RED): committed results must be
    # byte-auditable (work/ plans stay gitignored, so rows.csv carries
    # each plan's sha256), timing.json must carry provenance, and the
    # runner must refuse to silently overwrite an existing result dir.

    def test_provenance_keys(self):
        import sys
        sys.path.insert(0, str(BENCH))
        from run_benchmark import provenance_info
        info = provenance_info()
        for key in ("git_commit", "binary_sha256", "host"):
            self.assertIn(key, info)
        self.assertGreaterEqual(len(info["git_commit"]), 7)
        self.assertEqual(len(info["binary_sha256"]), 64)

    def test_plan_sha256_field_and_fill(self):
        import hashlib
        import sys
        sys.path.insert(0, str(BENCH))
        from run_benchmark import FIELDS, row_carrier
        self.assertIn("plan_sha256", FIELDS)
        ins_path = REPO / "tests/fixtures/dd_tiny.yaml"
        ins = load_instance(ins_path)
        work = BENCH / "results_probe"
        work.mkdir(exist_ok=True)
        row = row_carrier(ins, ins_path, "dd_tiny_sha", "probe", work, 5)
        self.assertEqual(row["success"], 1)
        expected = hashlib.sha256(
            (work / "dd_tiny_sha.carrier.plan").read_bytes()).hexdigest()
        self.assertEqual(row["plan_sha256"], expected)

    def test_out_dir_overwrite_guard(self):
        import sys
        import tempfile
        sys.path.insert(0, str(BENCH))
        from run_benchmark import ensure_out_dir
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "res"
            ensure_out_dir(out, force=False)  # fresh dir: fine
            (out / "rows.csv").write_text("x")
            with self.assertRaises(SystemExit):
                ensure_out_dir(out, force=False)  # would overwrite
            ensure_out_dir(out, force=True)  # explicit override
