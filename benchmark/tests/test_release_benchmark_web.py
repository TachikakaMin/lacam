"""PROTECTED release benchmark dashboard regression."""

import sys
import unittest
from pathlib import Path


BENCH = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(BENCH))

from generate_release_benchmark_web import load_release_data, render_dashboard


ROWS = BENCH / "results_release_77_20260904" / "rows.csv"
TIMING = BENCH / "results_release_77_20260904" / "timing.json"
TARGET = "warehouse_blocks_h20w20_b3_a1_d75_r8_t12_seed0"


class TestReleaseBenchmarkWeb(unittest.TestCase):
    def test_dashboard_covers_the_whole_release_suite(self):
        data = load_release_data(ROWS, TIMING)
        self.assertEqual(data["summary"]["total"], 77)
        self.assertEqual(data["summary"]["solved"], 47)
        self.assertEqual(data["summary"]["warehouse_total"], 9)
        self.assertEqual(data["summary"]["warehouse_solved"], 9)
        self.assertEqual(data["summary"]["brap_total"], 68)
        self.assertEqual(data["summary"]["brap_solved"], 38)

        html = render_dashboard(data)
        self.assertIn('<html lang="zh-CN">', html)
        self.assertIn(TARGET, html)
        self.assertIn("47 / 77", html)
        self.assertIn("9 / 9", html)
        self.assertIn('id="familyFilter"', html)
        self.assertIn('id="statusFilter"', html)

        for row in data["rows"]:
            self.assertIn(row["instance"], html)
            animation = f'cases/{row["instance"]}.html'
            if row["success"]:
                self.assertIn(animation, html)
            else:
                self.assertNotIn(animation, html)


if __name__ == "__main__":
    unittest.main()
