"""CLI contract tests for the dd_benchmark driver (debug.md round-2 P0-4).

- An UNKNOWN mode must fail loudly (non-zero exit), not silently fall back
  to the default solver (silent fallback burned us once with the no-op
  no_astar ablation: a typo in a mode/knob name must never masquerade as a
  valid run).
- The three supported modes must be distinguishable from stdout via a
  'mode=' line so result rows can be attributed to the solver that actually
  produced them.
"""

import subprocess
import tempfile
import unittest
from pathlib import Path

from ddbench.instance import Instance, Target, parse_map_str, save_instance

BENCH = Path(__file__).resolve().parent.parent
BIN = BENCH.parent / "build/dd_benchmark"


def tiny_instance(tmp):
    ins = Instance(
        grid=parse_map_str("....\n....\n...."),
        robots=[(0, 0), (2, 3)],
        shelves=[(1, 1), (1, 2)],
        targets=[Target("b0", (1, 1), (2, 2))],
    )
    path = Path(tmp) / "tiny.yaml"
    save_instance(ins, path)
    return path


def run(args):
    return subprocess.run(
        [str(BIN)] + [str(a) for a in args],
        capture_output=True,
        text=True,
        timeout=60,
    )


class TestCliModes(unittest.TestCase):
    def test_unknown_mode_fails_loudly(self):
        with tempfile.TemporaryDirectory() as tmp:
            ins = tiny_instance(tmp)
            p = run([ins, 5, Path(tmp) / "out.plan", 0, "bogus_mode"])
            self.assertNotEqual(
                p.returncode, 0,
                "unknown MODE must exit non-zero, not fall back silently",
            )
            self.assertIn("mode", (p.stderr + p.stdout).lower())

    def test_three_modes_distinguishable(self):
        with tempfile.TemporaryDirectory() as tmp:
            ins = tiny_instance(tmp)
            for mode in ("lacam", "b0", "b1"):
                p = run([ins, 5, Path(tmp) / f"{mode}.plan", 0, mode])
                self.assertEqual(p.returncode, 0, p.stderr)
                self.assertIn(
                    f"mode={mode}", p.stdout,
                    f"stdout must attribute the run to mode {mode}",
                )
                self.assertIn("solved=1", p.stdout)

    def test_default_mode_is_lacam(self):
        with tempfile.TemporaryDirectory() as tmp:
            ins = tiny_instance(tmp)
            p = run([ins, 5, Path(tmp) / "d.plan", 0])
            self.assertEqual(p.returncode, 0, p.stderr)
            self.assertIn("mode=lacam", p.stdout)


if __name__ == "__main__":
    unittest.main()
