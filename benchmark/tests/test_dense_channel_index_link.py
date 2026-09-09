"""The benchmark landing page must expose Dense-channel V2."""

import unittest
from pathlib import Path


class DenseChannelIndexLinkTest(unittest.TestCase):
    def test_root_index_links_40x40_block_edge_scale_results(self):
        root = Path(__file__).resolve().parent.parent / "viz_web"
        page = (root / "index.html").read_text(encoding="utf-8")
        self.assertIn(
            "dense_channel_block_edge_40x40_benchmark_"
            "terminal_scope_20260907/index.html",
            page,
        )
        self.assertIn(
            "dense_channel_block_edge_40x40_suite_v1_20260907/"
            "index.html",
            page,
        )
        self.assertIn(
            "dense_channel_block_edge_40x40_diagnostic_60s_"
            "20260907.html",
            page,
        )

    def test_root_index_links_interior_to_edge_suite_and_results(self):
        root = Path(__file__).resolve().parent.parent / "viz_web"
        page = (root / "index.html").read_text(encoding="utf-8")
        self.assertIn(
            "dense_channel_i2e_benchmark_terminal_scope_20260907/"
            "index.html",
            page,
        )
        self.assertIn(
            "dense_channel_i2e_suite_v1_20260907/index.html",
            page,
        )

    def test_root_index_links_benchmark_and_generation_pages(self):
        root = Path(__file__).resolve().parent.parent / "viz_web"
        page = (root / "index.html").read_text(encoding="utf-8")
        self.assertIn(
            "dense_channel_benchmark_v2_20260906/index.html", page
        )
        self.assertIn(
            "dense_channel_suite_v2_20260906/index.html", page
        )


if __name__ == "__main__":
    unittest.main()
