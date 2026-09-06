"""Controlled-comparison semantics shown by the proposal webpage."""

import tempfile
import unittest
from pathlib import Path

from generate_warehouse_case_proposal_web import (
    generate_site,
    proposal_cases,
)


class WarehouseCaseProposalComparisonTest(unittest.TestCase):
    def test_all_factorial_pairs_are_machine_readable(self):
        rows = proposal_cases()
        self.assertTrue(
            all(
                row["seed_controls"]
                == "paired_tasks_goals_shelf_priority_robot_order"
                for row in rows
            )
        )
        groups = {}
        for row in rows:
            groups.setdefault(row["pairing_key"], []).append(row)
        self.assertEqual(len(groups), 36)
        for group in groups.values():
            self.assertEqual(len(group), 12)
            self.assertEqual(
                {
                    (row["density_level"], row["agent_level"])
                    for row in group
                },
                {
                    (density, agent)
                    for density in ("low", "medium", "high")
                    for agent in (
                        "scarce",
                        "baseline",
                        "equal",
                        "surplus",
                    )
                },
            )
            self.assertTrue(
                all(
                    row["robot_start_policy"] == "nested_prefix"
                    and row["layout_policy"] == "nested_density_prefix"
                    for row in group
                )
            )

    def test_page_explains_controlled_comparisons(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "site"
            generate_site(output)
            page = (output / "index.html").read_text(encoding="utf-8")

        self.assertIn(
            "每个地图族都完整覆盖 3 个密度 × 4 个 Agent 数",
            page,
        )
        self.assertIn(
            "Agent 起点使用嵌套前缀",
            page,
        )
        self.assertIn(
            "同一配对组保持 target 起点、goal 定义和随机基准不变",
            page,
        )
        self.assertIn(
            "低、中、高密度使用嵌套货架布局",
            page,
        )
        self.assertIn(
            "不会根据 Planner 成功率、makespan 或 SOC 筛选 seed",
            page,
        )
        self.assertIn("<th>对照关系</th>", page)


if __name__ == "__main__":
    unittest.main()
