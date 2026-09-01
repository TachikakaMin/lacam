"""Baseline binary regression tests.

Pins two bugs found on 2026-08-29:

1. BUILD BUG (ours to guard): upstream CREST CMakeLists.txt hardcodes
   `-g -O0` and force-overrides CMAKE_BUILD_TYPE, so a naive build produces
   an unoptimized binary that is 6-100x slower than the paper's numbers
   (demo: 7.9 s at -O0 vs 1.2 s at -O3; crest_base on R2R-M: 28 s vs 1.9 s).
   We patch the vendored CMakeLists to -O3; these tests fail loudly if a
   re-clone/rebuild silently reverts to -O0.

2. UPSTREAM LOGIC BUG (canary): CREST with --STR --DW --GTR on the pinned
   DnE-M instance either exits nonzero with a known path/dependency error
   or exceeds the fixed 10-second budget; crest_base solves seed0. If
   upstream fixes it within budget, the canary alerts us to re-measure.

Run:  PYTHONPATH=. python3 -m unittest tests.test_baselines -v
(from benchmark/; requires built baselines and the ddtool env libs)
"""

import os
import re
import subprocess
import time
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent.parent
CREST_DIR = REPO / "baselines/CREST"
CREST_BIN = CREST_DIR / "build/CREST"
MAWR_BIN = REPO / "baselines/wh-rearrangement/build/MAWR"
FIXTURES = Path(__file__).resolve().parent / "fixtures"
ENV = dict(
    os.environ,
    LD_LIBRARY_PATH=str(Path.home() / "micromamba/envs/ddtool/lib"),
)
SOLVER_TIME_LIMIT_SEC = 10
PROCESS_TIMEOUT_SEC = SOLVER_TIME_LIMIT_SEC + 5


def run(cmd, timeout):
    t0 = time.time()
    p = subprocess.run(
        [str(c) for c in cmd], capture_output=True, text=True,
        timeout=timeout, env=ENV,
    )
    return p, time.time() - t0


@unittest.skipUnless(CREST_BIN.exists(), "CREST not built")
class TestCrestBuildOptimized(unittest.TestCase):
    """Guard against the upstream -O0 default (bug 1)."""

    def test_flags_do_not_contain_O0(self):
        flags_make = CREST_DIR / "build/CMakeFiles/CREST.dir/flags.make"
        self.assertTrue(flags_make.exists(), "flags.make missing")
        text = flags_make.read_text()
        m = re.search(r"^CXX_FLAGS = (.*)$", text, re.M)
        self.assertIsNotNone(m)
        flags = m.group(1).split()
        # the LAST -O flag wins for gcc; it must not be -O0
        olevels = [f for f in flags if re.fullmatch(r"-O[0-9sz]?", f)]
        self.assertTrue(olevels, f"no -O flag found in: {flags}")
        self.assertNotEqual(olevels[-1], "-O0",
                            "CREST built at -O0: upstream CMakeLists "
                            "hardcodes '-g -O0'; re-apply the local -O3 patch")

    def test_demo_full_mode_fast(self):
        """Demo with all strategies: ~1.2 s at -O3, ~7.9 s at -O0.
        Bound of 5 s separates the two reliably on this machine."""
        p, wall = run(
            [CREST_BIN, "--suboptimality=1.6", "-m", CREST_DIR / "data/1.map",
             "-a", CREST_DIR / "data/1.scen", "-k", "8", "--STR=true",
             "--DW=true", "--GTR=true", "-t", SOLVER_TIME_LIMIT_SEC],
            timeout=PROCESS_TIMEOUT_SEC,
        )
        self.assertEqual(p.returncode, 0, p.stderr[-300:])
        m = re.search(r"runtime : ([0-9.]+)", p.stdout)
        self.assertIsNotNone(m, p.stdout[-300:])
        self.assertLess(float(m.group(1)), 5.0,
                        "demo runtime regressed; likely -O0 rebuild")

    def test_demo_baseline_mode_solves(self):
        p, wall = run(
            [CREST_BIN, "--suboptimality=1.6", "-m", CREST_DIR / "data/1.map",
             "-a", CREST_DIR / "data/1.scen", "-k", "8", "--STR=false",
             "--DW=false", "--GTR=false", "-t", SOLVER_TIME_LIMIT_SEC],
            timeout=PROCESS_TIMEOUT_SEC,
        )
        self.assertEqual(p.returncode, 0, p.stderr[-300:])
        self.assertIn("makespan :", p.stdout)


@unittest.skipUnless(CREST_BIN.exists(), "CREST not built")
class TestCrestUpstreamPathfindingBug(unittest.TestCase):
    """Canary for upstream bug 2: internal MLSIPP "can not find path"
    failure in the shared execution layer (exit 1 after a retry loop).

    Two distinct upstream failure signatures observed:
      A. "can not find path for agent ..."  (MLSIPP pathfinding, exit 1)
      B. "dep from shelf X step k to shelf Y step k is added before the
          planned step" (dependency-graph consistency, exit 1)
    Pinned fixtures:
      - seed6: BOTH modes fail with A -> bug lives in the shared executor;
      - seed0: base solves; full fails (A or B depending on timing/limit)
        -> constraint release raises the trigger probability.
    """

    MAP6 = FIXTURES / "dneM_n32_seed6.crest.map"
    SCEN6 = FIXTURES / "dneM_n32_seed6.crest.scen"
    MAP0 = FIXTURES / "dneM_n32_seed0.crest.map"
    SCEN0 = FIXTURES / "dneM_n32_seed0.crest.scen"

    SIGNATURES = (
        "can not find path for agent",
        "is added before the planned step",
    )

    def _run(self, mp, sp, full):
        flag = "true" if full else "false"
        try:
            return run(
                [CREST_BIN, "--suboptimality=1.6", "-m", mp, "-a", sp,
                 "-k", "32", f"--STR={flag}", f"--DW={flag}",
                 f"--GTR={flag}", "-t", SOLVER_TIME_LIMIT_SEC],
                timeout=PROCESS_TIMEOUT_SEC,
            )
        except subprocess.TimeoutExpired:
            return None, PROCESS_TIMEOUT_SEC

    def _assert_known_failure(self, p, label):
        if p is None:
            return  # expected failure to finish within the fixed budget
        combined = p.stdout + p.stderr
        if p.returncode == 0:
            self.fail(
                f"UPSTREAM BUG APPEARS FIXED ({label}): re-run the DnE "
                "crest suites and update benchmark/README.md results."
            )
        self.assertTrue(
            any(sig in combined for sig in self.SIGNATURES),
            f"{label}: rc={p.returncode} with UNKNOWN failure signature "
            f"(new bug?): {combined[-300:]}",
        )

    def test_seed6_full_mode_fails_or_exceeds_budget(self):
        p, _ = self._run(self.MAP6, self.SCEN6, full=True)
        self._assert_known_failure(p, "seed6 crest_full")

    def test_seed6_baseline_mode_also_fails(self):
        """Bug is in the shared executor, not only in constraint release."""
        p, _ = self._run(self.MAP6, self.SCEN6, full=False)
        self._assert_known_failure(p, "seed6 crest_base")

    def test_seed0_baseline_solves_but_full_fails(self):
        """Isolation pair: base solves; full fails or exceeds 10 seconds."""
        p_base, _ = self._run(self.MAP0, self.SCEN0, full=False)
        self.assertIsNotNone(p_base, "seed0 crest_base exceeded 10 seconds")
        self.assertEqual(p_base.returncode, 0,
                         (p_base.stdout + p_base.stderr)[-300:])
        self.assertIn("makespan :", p_base.stdout)
        p_full, _ = self._run(self.MAP0, self.SCEN0, full=True)
        self._assert_known_failure(p_full, "seed0 crest_full")


@unittest.skipUnless(MAWR_BIN.exists(), "MAWR not built")
class TestMawrSmoke(unittest.TestCase):
    def test_tiny_instance_optimal_makespan(self):
        """3x4 map, 1 agent under the shelf, shelf moves 1 cell: mk must be
        small and constant — pins conversion semantics AND solver sanity."""
        import tempfile
        with tempfile.TemporaryDirectory() as td:
            mp = Path(td) / "t.map"
            sp = Path(td) / "t.scen"
            mp.write_text("3 4\n....\n....\n....\n")
            # agent at (row1,col1) -> file 'col row'; shelf (1,1)->(1,2)
            sp.write_text("1 1 1\n1 1\n1 1 2 1\n")
            out = Path(td) / "r.csv"
            out.touch()
            p, wall = run(
                [MAWR_BIN, "-m", mp, "-s", sp, "-a", "NATCBS", "-t",
                 SOLVER_TIME_LIMIT_SEC, "-o", out],
                timeout=PROCESS_TIMEOUT_SEC,
            )
            self.assertEqual(p.returncode, 0, (p.stdout + p.stderr)[-300:])
            line = out.read_text().strip().splitlines()[-1]
            mk = line.split(";")[2]
            self.assertTrue(mk.isdigit(), f"unexpected output: {line}")
            self.assertLessEqual(int(mk), 3)


if __name__ == "__main__":
    unittest.main()
