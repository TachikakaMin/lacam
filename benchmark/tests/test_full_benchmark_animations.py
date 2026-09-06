import tempfile
import unittest
from pathlib import Path

from generate_full_benchmark_dashboard import (
    build_dashboard_data,
    generate_case_animation,
)


BENCH = Path(__file__).resolve().parent.parent
RESULTS = (
    BENCH
    / "historical"
    / "pre_v5"
    / "results_full_factorial_20260905"
)
MANIFEST = (
    BENCH
    / "viz_web"
    / "warehouse_case_proposal"
    / "factorial_suite"
    / "manifest.json"
)


class TestFullBenchmarkAnimations(unittest.TestCase):
    def test_historical_plan_remains_a_valid_animation_fixture(self):
        data = build_dashboard_data(
            RESULTS / "rows.csv",
            RESULTS / "timing.json",
            MANIFEST,
        )
        case = next(
            item
            for item in data["all_cases"]
            if item["success"] and item["instance"].startswith("g1_")
        )
        instance = (
            BENCH
            / "viz_web"
            / "warehouse_case_proposal"
            / "factorial_suite"
            / "instances"
            / (case["instance"] + ".yaml")
        )
        plan = RESULTS / "work" / (
            case["instance"] + ".carrier.plan"
        )
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "animation.html"
            generate_case_animation(case, instance, plan, output)
            page = output.read_text(encoding="utf-8")

        self.assertIn("requestAnimationFrame", page)
        self.assertIn("p * p * (3 - 2 * p)", page)
        self.assertIn(case["instance"], page)
        self.assertIn("Eligible goal", page)


if __name__ == "__main__":
    unittest.main()
