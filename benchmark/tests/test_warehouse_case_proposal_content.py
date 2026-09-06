"""Content-accuracy contracts for the warehouse proposal webpage."""

import tempfile
import unittest
from pathlib import Path

from generate_warehouse_case_proposal_web import (
    generate_site,
    proposal_cases,
)


class WarehouseCaseProposalContentTest(unittest.TestCase):
    def test_candidate_rows_are_explicitly_pending_and_define_core_goals(self):
        rows = proposal_cases()
        for row in rows:
            self.assertEqual(row["certification"], "validator_checked")
            self.assertEqual(row["core_goal_count"], row["targets"] // 2)
            self.assertLessEqual(row["core_goal_count"], row["targets"])
        self.assertTrue(
            all(
                row["shared_layout_group"] == row["pairing_key"]
                for row in rows
            )
        )
        self.assertTrue(
            all(
                row["eligible_goal_union_size"] >= row["targets"]
                for row in rows
            )
        )

    def test_page_distinguishes_samples_from_pending_candidates(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "site"
            generate_site(output)
            page = (output / "index.html").read_text(encoding="utf-8")

        self.assertIn("4 个已认证样例", page)
        self.assertIn("432 个随机配对 testcase", page)
        self.assertIn("全部通过构造可解性认证", page)
        self.assertIn("可解性保障", page)
        self.assertIn(
            "eligible goal：某件货允许作为最终位置的全部格子",
            page,
        )
        self.assertIn(
            "Planner assignment：Planner 实跑最终为每件货选择的终点",
            page,
        )
        self.assertIn("每件货有 4 个 goal", page)
        self.assertIn(
            "同区/邻区/远区按 Planner assignment 统计",
            page,
        )
        self.assertIn("intra：起点和终点位于同一个货架区", page)
        self.assertIn(
            "起点到 Planner assignment 的连线，不是 Planner 的实际运动轨迹",
            page,
        )
        self.assertIn(
            "后台可解性证书已由 validator 验证，不是 Planner 输出，也不会在本页播放",
            page,
        )
        self.assertIn(
            "本页只展示 Planner 实跑",
            page,
        )
        self.assertIn(
            "active agent 数量不是正确性目标",
            page,
        )
        self.assertIn(
            "静态紫色标记表示 Planner assignment",
            page,
        )
        self.assertNotIn("播放认证 witness", page)
        self.assertNotIn("Witness assignment:", page)
        self.assertNotIn("Witness 动画", page)
        self.assertNotIn("Witness plan", page)
        self.assertNotIn("按 witness assignment 统计", page)
        self.assertNotIn("构造后台证书", page)
        self.assertNotIn("lift、运输和 storage drop", page)
        self.assertNotIn("逐步重放验证", page)
        self.assertEqual(page.count("后台可解性证书"), 1)
        self.assertIn("<th>Goal 模式</th>", page)
        self.assertIn("低、中、高密度使用嵌套货架布局", page)
        self.assertIn("Agent 起点使用嵌套前缀", page)
        self.assertNotIn("先定义完成状态", page)
        self.assertNotIn("倒放得到 witness", page)


if __name__ == "__main__":
    unittest.main()
