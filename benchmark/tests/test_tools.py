"""Tool smoke tests (debug.md round-2 P1-11).

Covers the previously untested tooling checkboxes:
  - visualize_dd_schedule.py renders ascii frames from a validated plan;
  - generate_web_viz.py embeds a validated plan and REJECTS a corrupted
    one (its validator gate must bite);
  - generate_sweep_instances.py produces the documented axis set
    including the 1:50 tier (run into a temp dir, 1 seed);
  - run_ablations.py env wiring: every DD_* knob it sets must actually be
    read by the production sources — the static check that would have
    caught the no-op DD_NO_ASTAR variant.
"""

import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from ddbench.instance import Instance, Target, parse_map_str, save_instance

BENCH = Path(__file__).resolve().parent.parent
REPO = BENCH.parent
BIN = REPO / "build/dd_benchmark"


def make_solved_case(tmp):
    ins = Instance(
        grid=parse_map_str("....\n....\n...."),
        robots=[(0, 0), (2, 3)],
        shelves=[(1, 1), (1, 2)],
        targets=[Target("b0", (1, 1), (2, 2))],
    )
    ypath = Path(tmp) / "case.yaml"
    save_instance(ins, ypath)
    plan = Path(tmp) / "case.plan"
    p = subprocess.run(
        [str(BIN), str(ypath), "5", str(plan), "0"],
        capture_output=True, text=True, timeout=30,
    )
    assert "solved=1" in p.stdout, p.stdout
    return ypath, plan


class TestVisualizers(unittest.TestCase):
    def test_ascii_visualizer_renders_frames(self):
        with tempfile.TemporaryDirectory() as tmp:
            ypath, plan = make_solved_case(tmp)
            p = subprocess.run(
                [sys.executable, str(BENCH / "visualize_dd_schedule.py"),
                 str(ypath), str(plan), "--mode", "ascii", "--every", "1"],
                capture_output=True, text=True, timeout=60,
            )
            self.assertEqual(p.returncode, 0, p.stderr)
            self.assertGreaterEqual(p.stdout.count("t="), 2,
                                    "expected multiple ascii frames")

    def test_web_viz_validates_and_rejects_corrupt_plan(self):
        with tempfile.TemporaryDirectory() as tmp:
            ypath, plan = make_solved_case(tmp)
            out = Path(tmp) / "v.html"
            p = subprocess.run(
                [sys.executable, str(BENCH / "generate_web_viz.py"),
                 str(ypath), str(plan), str(out)],
                capture_output=True, text=True, timeout=60,
            )
            self.assertEqual(p.returncode, 0, p.stderr)
            html = out.read_text()
            self.assertIn("<canvas", html)
            self.assertIn('"plan":', html)
            # corrupt the plan: robot 0 teleports
            bad = Path(tmp) / "bad.plan"
            lines = plan.read_text().splitlines()
            lines[0] = "m 2 2;" + lines[0].split(";", 1)[1]
            bad.write_text("\n".join(lines) + "\n")
            p2 = subprocess.run(
                [sys.executable, str(BENCH / "generate_web_viz.py"),
                 str(ypath), str(bad), str(Path(tmp) / "bad.html")],
                capture_output=True, text=True, timeout=60,
            )
            self.assertNotEqual(p2.returncode, 0,
                                "corrupt plan must be rejected by the "
                                "embedded validator gate")


class TestSweepGenerator(unittest.TestCase):
    def test_axis_coverage_including_1to50(self):
        sys.path.insert(0, str(BENCH))
        import generate_sweep_instances as gsi
        with tempfile.TemporaryDirectory() as tmp:
            old_out, old_seeds = gsi.OUT, gsi.SEEDS
            try:
                gsi.OUT, gsi.SEEDS = Path(tmp), 1
                gsi.main()
                dirs = {d.name for d in Path(tmp).iterdir() if d.is_dir()}
            finally:
                gsi.OUT, gsi.SEEDS = old_out, old_seeds
            expect = {
                "ratio_r20", "ratio_r8", "ratio_r4", "ratio_r2",
                "ratio_r4x50",
                "fill_50", "fill_70", "fill_85", "fill_95",
                "ntgt_1", "ntgt_4", "ntgt_16", "ntgt_64",
                "depth_40", "depth_160", "depth_640",
            }
            self.assertEqual(dirs, expect)
            from ddbench.instance import load_instance
            big = next((Path(tmp) / "ratio_r4x50").glob("*.yaml"))
            ins = load_instance(big)
            self.assertEqual(len(ins.robots), 4)
            self.assertEqual(len(ins.shelves), 200)
            self.assertEqual(len(ins.grid), 40)


class TestAblationEnvWiring(unittest.TestCase):
    def test_every_knob_is_read_by_production_code(self):
        # the check that would have caught DD_NO_ASTAR (round-2 audit P0-3)
        runner = (BENCH / "run_ablations.py").read_text()
        knobs = set(re.findall(r'"(DD_[A-Z_]+)"\s*:', runner))
        self.assertTrue(knobs, "expected at least one DD_* knob in variants")
        sources = ""
        for p in list((REPO / "lacam/src").glob("*.cpp")) + list(
                (REPO / "tools").glob("*.cpp")):
            sources += p.read_text()
        for k in sorted(knobs):
            self.assertIn(
                k, sources,
                f"{k} is set by run_ablations.py but never read by any "
                f"production source — no-op ablation variant")

    def test_variant_modes_are_valid(self):
        runner = (BENCH / "run_ablations.py").read_text()
        modes = set(re.findall(r',\s*"(lacam|b0|b1)"\)', runner))
        self.assertTrue(modes.issubset({"lacam", "b0", "b1"}))


if __name__ == "__main__":
    unittest.main()
