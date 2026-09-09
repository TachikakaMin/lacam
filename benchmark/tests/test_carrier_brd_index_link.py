import unittest
from pathlib import Path


class CarrierBrdIndexLinkTest(unittest.TestCase):
    def test_main_visualization_index_links_the_final_artifacts(self):
        index = (
            Path(__file__).resolve().parents[1]
            / "viz_web"
            / "index.html"
        ).read_text(encoding="utf-8")

        self.assertIn(
            'href="carrier_brd_final_report_20260906/index.html"',
            index,
        )
        self.assertIn(
            'href="full_benchmark_carrier_brd_20260906/index.html"',
            index,
        )
        self.assertIn(
            'href="full_comparison_rho_v2_vs_carrier_brd_20260906/'
            'index.html"',
            index,
        )


if __name__ == "__main__":
    unittest.main()
