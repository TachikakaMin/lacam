"""Regressions found by the independent Dense-channel V2 review."""

import tempfile
import unittest
from pathlib import Path

from generate_dense_channel_report import render_dashboard
from generate_dense_channel_suite import generate_suite


class DenseChannelReviewRegressionTest(unittest.TestCase):
    def test_solver_yaml_does_not_embed_reverse_witness_pairings(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "suite"
            rows = generate_suite(output)
            for row in rows:
                yaml_text = (output / row["yaml"]).read_text(
                    encoding="utf-8"
                )
                self.assertNotIn("anon_goals:", yaml_text)

    def test_report_calls_first_solution_a_kernel_raw_solution(self):
        page = render_dashboard(
            {
                "records": [],
                "summary": {
                    "total": 0,
                    "solved": 0,
                    "inner_starts": 0,
                    "deep_starts": 0,
                    "inner_goals": 0,
                    "target_slots": 0,
                    "min_density": 0,
                    "max_density": 0,
                    "wall_time_sec": 0,
                    "timeout_sec": 10,
                    "jobs": 1,
                },
                "timing": {
                    "provenance": {},
                    "suite": {},
                },
            }
        )
        self.assertIn("kernel 原始首解", page)
        self.assertNotIn("第一份可交付解的时刻", page)

    def test_root_index_uses_the_actual_density_minimum(self):
        root = Path(__file__).resolve().parent.parent / "viz_web"
        page = (root / "index.html").read_text(encoding="utf-8")
        self.assertIn("75.0%–93.8%", page)
        self.assertNotIn("77.8%–93.8% 货架密度", page)


if __name__ == "__main__":
    unittest.main()
