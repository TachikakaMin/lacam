"""Old full results remain available only as explicit pre-v5 history."""

import json
import unittest
from pathlib import Path


BENCH = Path(__file__).resolve().parent.parent


class TestHistoricalFullArtifacts(unittest.TestCase):
    def test_old_approval_and_dashboard_are_prominently_quarantined(self):
        self.assertFalse((BENCH / "full_review_approval.json").exists())
        approval_path = (
            BENCH
            / "historical"
            / "pre_v5"
            / "full_review_approval.json"
        )
        approval = json.loads(
            approval_path.read_text(encoding="utf-8")
        )
        self.assertEqual(approval["schema_version"], 1)

        archive = (
            BENCH
            / "viz_web"
            / "historical_pre_v5_full_benchmark_509_20260905"
            / "index.html"
        ).read_text(encoding="utf-8")
        index = (BENCH / "viz_web" / "index.html").read_text(
            encoding="utf-8"
        )
        for text in ("历史 pre-v5", "旧二进制", "不作为 v5 完成证据"):
            self.assertIn(text, archive)
            self.assertIn(text, index)

        self.assertIn(
            "historical_pre_v5_full_benchmark_509_20260905/index.html",
            index,
        )
        self.assertNotIn(
            'href="full_benchmark_509_20260905/index.html"',
            index,
        )

    def test_pre_v5_projection_repair_notes_are_not_current_v5_semantics(self):
        readme = (BENCH / "README.md").read_text(encoding="utf-8")
        self.assertNotIn(
            "## Projection-repair results (2026-09-01, current)",
            readme,
        )
        self.assertIn(
            "## Projection-repair results (2026-09-01, historical pre-v5)",
            readme,
        )
        self.assertIn(
            "当前 v5 production 使用严格 `(T,W)`",
            readme,
        )
        self.assertIn(
            "singleton 也会使用剩余共享预算做一次有界改进",
            readme,
        )


if __name__ == "__main__":
    unittest.main()
